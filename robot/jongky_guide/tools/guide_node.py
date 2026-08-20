#!/usr/bin/env python3
"""안내로봇 본체 노드 — 목적지를 받아 Nav2 로 데려가고 음성으로 안내한다.

역할이 셋이다.

  1. 웹 UI(터치스크린)를 띄우고 목적지 요청을 받는다
  2. waypoint 를 Nav2 목표로 바꿔 보내고 도착을 지켜본다
  3. 상태가 바뀔 때마다 TTS 로 말한다

waypoint 는 teleop_key.py 가 맵핑 주행 중에 찍어 둔 YAML 을 읽는다.
같은 좌표계(map)이므로 그대로 Nav2 goal 이 된다.

층이 갈리면 지도도 waypoint 도 갈아야 한다. --floors 를 주면 층별 자원을
guide_mission 이 관리하고, 다른 층 목적지를 누르면 엘리베이터 상태머신이
돈다 (fleet/guide_mission/README.md). 안 주면 지금까지처럼 한 층만 돈다.

실행:
    ros2 run jongky_guide guide_node.py
    ros2 run jongky_guide guide_node.py --waypoints ~/waypoints_10f.yaml --port 8080
    ros2 run jongky_guide guide_node.py --floors ~/floors.yaml --floor 10f

브라우저에서 http://<젯슨IP>:8080 (터치스크린은 로컬이므로 localhost).
"""
from __future__ import annotations

import argparse
import json
import os
import threading
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from brain import Brain
from follow_client import Follower
from listen import Listener
from speech import Speaker

# 층 전환. 별도 패키지(fleet/guide_mission)라 없을 수 있다 — 없으면 한 층만
# 돈다. **조용히 없어지지 않는다**: --floors 를 줬는데 임포트가 안 되면
# 기동 자체를 거부한다 (main 참조).
try:
    from guide_mission import transfer as mission
    from guide_mission.detect import FloorDetector
    from guide_mission.floors import ERROR, FloorBook
    from guide_mission.korean import ro
    HAVE_MISSION = True
    MISSION_IMPORT_ERROR = ""
except ImportError as _e:      # pragma: no cover - 빌드 안 된 환경
    HAVE_MISSION = False
    MISSION_IMPORT_ERROR = str(_e)

    def ro(word: str) -> str:  # noqa: D103 - guide_mission 이 없을 때의 대타
        return f"{word} 으로"

# map->odom 이 실제로 나오는지 보는 데만 쓴다. 없으면 그 확인만 건너뛴다.
try:
    import tf2_ros
except ImportError:            # pragma: no cover
    tf2_ros = None

WEB_DIR = os.path.join(get_package_share_directory("jongky_guide"), "web")

# amcl_pose 는 latched 다. BasicNavigator 와 같은 QoS 를 써야 붙는다.
AMCL_QOS = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

LOAD_MAP_REASON = {
    1: "지도 파일이 없다 (RESULT_MAP_DOES_NOT_EXIST)",
    2: "지도 이미지가 깨졌다 (RESULT_INVALID_MAP_DATA)",
    3: "지도 YAML 이 이상하다 (RESULT_INVALID_MAP_METADATA)",
    255: "map_server 가 알 수 없는 이유로 실패했다",
}


@dataclass(frozen=True)
class Wp:
    """waypoint 하나. floors.Waypoint 와 같은 모양이라 둘을 같이 다룬다."""

    name: str
    label: str
    x: float
    y: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = "map"


@dataclass
class GuideState:
    """UI 가 폴링해 가는 현재 상태."""

    status: str = "idle"          # idle | navigating | waiting | arrived | failed
                                  # | alert | listening | transfer | fault
    destination: str = ""
    message: str = "목적지를 선택해 주세요"
    distance: float = 0.0
    follower_m: float = -1.0      # 뒤따라오는 사람까지 거리. -1 = 모름
    floor: str = ""               # 지금 층 키. 빈 문자열이면 모름
    floor_label: str = ""
    # 층 전환 중일 때만 채워진다. UI 가 이걸 보고 사람이 할 일을 그린다.
    #   {"state","message","detail","actions":[{"event","label"}],"target","here"}
    transfer: dict | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "status": self.status,
                "destination": self.destination,
                "message": self.message,
                "distance": round(self.distance, 2),
                "follower_m": round(self.follower_m, 1),
                "floor": self.floor,
                "floor_label": self.floor_label,
                "transfer": self.transfer,
            }

    def set(self, **kw) -> None:
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


class GuideNode(Node):
    def __init__(
        self,
        waypoint_path: str,
        speaker: Speaker,
        state: GuideState,
        listener: Listener | None = None,
        brain: Brain | None = None,
        follower: Follower | None = None,
        book=None,
        detector=None,
        floor: str = "",
    ):
        super().__init__("jongky_guide")
        self._speaker = speaker
        self._state = state
        self._listener = listener
        self._brain = brain
        self._follower = follower
        self._nav = BasicNavigator()
        self._task: threading.Thread | None = None
        # AMCL 이 초기 위치를 받았는가. 받기 전에는 map 프레임 목표를 변환할 수
        # 없으므로 안내를 시작하면 안 된다 (아래 set_start 주석 참조).
        self._localized = False

        # ── 층 ────────────────────────────────────────────────────────────
        # book 이 없으면 예전과 같다 — waypoint 파일 하나, 층 개념 없음.
        self._book = book
        self._detector = detector
        self._floor = floor if book is not None else ""
        self._gate = mission.Gate() if (book is not None and HAVE_MISSION) else None
        self._transfer = None          # 진행 중인 FloorTransfer
        if book is not None:
            self._waypoints = self._floor_waypoints(self._floor)
            self._sync_floor_label()
        else:
            self._waypoints = self._load_waypoints(waypoint_path)

        # ── AMCL 이 실제로 잡혔는지 보는 눈 ────────────────────────────────
        # 지도를 갈아끼운 뒤 초기 위치를 줬을 때 **정말로 먹었는지** 확인해야
        # 한다. 안 그러면 map->odom 없이 목표만 쌓이고 로그는 조용하다.
        # 콜백은 MultiThreadedExecutor 가 돌린다 — 여기서 spin 하지 않는다.
        self._amcl: tuple[float, float] | None = None
        self._amcl_at = 0.0
        self._amcl_lock = threading.Lock()
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, AMCL_QOS)
        self._tf_buffer = None
        if tf2_ros is not None:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 외부(예: 음성 노드)에서도 목적지를 넣을 수 있게 열어 둔다
        self.create_subscription(String, "/guide/destination", self._on_destination_topic, 10)
        self._status_pub = self.create_publisher(String, "/guide/status", 10)

        # ── 돌발상황 판단용 카메라 ────────────────────────────────────────
        # 압축 토픽을 우선한다 — 이미 JPEG 이라 인코딩이 필요 없다.
        # 젯슨 컨테이너의 cv_bridge 는 numpy ABI 충돌로 임포트가 안 되므로
        # 원본 토픽으로 떨어질 때만 PIL 로 굽는다.
        self._jpeg: bytes | None = None
        self._raw: Image | None = None
        if brain is not None:
            self.create_subscription(
                CompressedImage, "/camera/rgb/image_raw/compressed", self._on_compressed, 1
            )
            self.create_subscription(Image, "/camera/rgb/image_raw", self._on_raw, 1)

        self.get_logger().info(f"waypoint {len(self._waypoints)}개: {list(self._waypoints)}")

    # ── waypoint ──────────────────────────────────────────────────────────
    def _load_waypoints(self, path: str) -> dict:
        """--floors 없이 돌 때. 파일 하나를 그대로 읽는다 (예전 동작)."""
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            self.get_logger().warn(f"waypoint 파일이 없다: {path} — 맵핑 주행부터 할 것")
            return {}
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        out: dict[str, Wp] = {}
        for name, w in doc.items():
            try:
                pos, ori = (w or {})["position"], (w or {}).get("orientation") or {}
                out[name] = Wp(
                    name=name, label=name,
                    x=float(pos["x"]), y=float(pos["y"]),
                    qx=float(ori.get("x", 0.0)), qy=float(ori.get("y", 0.0)),
                    qz=float(ori.get("z", 0.0)), qw=float(ori.get("w", 1.0)),
                    frame_id=(w or {}).get("frame_id", "map"),
                )
            except Exception as e:
                self.get_logger().warn(f"waypoint '{name}' 을 건너뛴다: {e}")
        return out

    def _floor_waypoints(self, key: str) -> dict:
        """대장에 적힌 그 층의 waypoint. 층을 모르면 빈 사전."""
        fl = self._book.get(key) if (self._book and key) else None
        return dict(fl.waypoints) if fl else {}

    def _sync_floor_label(self) -> None:
        fl = self._book.get(self._floor) if (self._book and self._floor) else None
        self._state.set(floor=self._floor or "",
                        floor_label=fl.label if fl else "")

    @property
    def destinations(self) -> list[str]:
        return list(self._waypoints)

    def catalog(self) -> dict:
        """UI 가 그릴 목록. 층이 있으면 층별로 준다."""
        if self._book is None:
            return {
                "destinations": list(self._waypoints),
                "floor": "", "floor_label": "", "floors": [],
                "multi_floor": False,
            }
        floors = []
        for fl in self._book.ordered():
            floors.append({
                "key": fl.key,
                "label": fl.label,
                "usable": fl.usable,
                "problems": [p.text for p in fl.problems],
                "elevator": {"board": fl.board, "exit": fl.exit_wp},
                "waypoints": [{"name": n, "label": fl.label_of(n)}
                              for n in fl.waypoints],
                "destinations": fl.destinations(),
            })
        return {
            "destinations": list(self._waypoints),
            "floor": self._floor,
            "floor_label": self._state.snapshot()["floor_label"],
            "floors": floors,
            "multi_floor": len(floors) > 1,
        }

    def _to_pose(self, name: str, wp=None) -> PoseStamped:
        w = wp if wp is not None else self._waypoints[name]
        p = PoseStamped()
        p.header.frame_id = getattr(w, "frame_id", "map")
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(w.x)
        p.pose.position.y = float(w.y)
        p.pose.orientation.x = float(w.qx)
        p.pose.orientation.y = float(w.qy)
        p.pose.orientation.z = float(w.qz)
        p.pose.orientation.w = float(w.qw)
        return p

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        with self._amcl_lock:
            self._amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self._amcl_at = time.monotonic()

    # ── 초기 위치 ─────────────────────────────────────────────────────────
    #
    # AMCL 은 초기 위치를 받기 전까지 map->odom 을 안 낸다. 그러면 _to_pose()
    # 가 만든 frame_id="map" 목표를 변환할 수 없어서 **어떤 목적지도 안 간다.**
    # 로그는 조용하고 goal 은 접수된 것처럼 보인다.
    #
    # 지금까지 유일한 방법이 개발 PC 에서 RViz 를 띄워 2D Pose Estimate 를
    # 찍는 것이었다. 터치스크린만 있는 현장에서는 쓸 수 없다.
    #
    # 그래서 waypoint 를 그대로 초기 위치로 쓴다. 운영자가 로봇을 아는 지점에
    # (예: 엘리베이터 앞) 놓고 UI 에서 그 이름을 누르면 된다.
    def set_start(self, name: str, floor: str = "") -> tuple[bool, str]:
        if self._task and self._task.is_alive():
            return False, "안내 중에는 초기 위치를 바꿀 수 없다"

        # 층까지 바꿔야 하면 지도부터 갈아야 한다. 지도 로드는 몇 초 걸리고
        # 실패할 수 있으므로 HTTP 응답을 붙잡지 않고 작업 스레드로 넘긴다.
        if floor and self._book is not None and floor != self._floor:
            fl = self._book.get(floor)
            if fl is None:
                return False, f"'{floor}' 는 모르는 층이다"
            self._task = threading.Thread(
                target=self._restart_on_floor, args=(floor, name), daemon=True)
            self._task.start()
            return True, ""

        if name not in self._waypoints:
            return False, f"'{name}' 은 등록되지 않은 지점이다"
        ok, err = self._relocalize_at(name)
        if not ok:
            self._state.set(status="fault", message=f"위치를 못 잡았다: {err}")
            self._publish_status()
            self._speaker.say("위치를 잡지 못했습니다. 화면을 확인해 주세요")
            return False, err
        label = self._label(name)
        self._state.set(status="idle", transfer=None, message=f"'{label}' 에서 시작합니다")
        self.get_logger().info(f"초기 위치를 '{name}' 로 잡았다")
        self._speaker.say(f"{label} 에서 시작합니다")
        return True, ""

    def _label(self, name: str) -> str:
        w = self._waypoints.get(name)
        return getattr(w, "label", name) if w is not None else name

    def _restart_on_floor(self, floor: str, name: str) -> None:
        """수동 복구 경로. 사람이 로봇을 다른 층에 옮겨 놓고 층+지점을 고른 것.

        엘리베이터 상태머신이 fault 로 끝났을 때 여기로 돌아온다.
        """
        fl = self._book.get(floor)
        self._state.set(status="transfer", transfer={
            "state": "swap_map", "message": f"{fl.label} 지도를 불러오는 중입니다",
            "detail": "", "actions": [], "target": floor, "here": floor})
        self._publish_status()

        ok, err = self._book.reload_waypoints(floor)
        if not ok:
            return self._fail_floor(err)
        ok, err = self.load_map(fl)
        if not ok:
            return self._fail_floor(err)
        self._floor = floor
        self._waypoints = self._floor_waypoints(floor)
        self._sync_floor_label()
        if name not in self._waypoints:
            return self._fail_floor(f"{fl.label} 에 '{name}' 지점이 없다")
        ok, err = self._relocalize_at(name)
        if not ok:
            return self._fail_floor(err)
        label = self._label(name)
        self._state.set(status="idle", transfer=None,
                        message=f"{fl.label} '{label}' 에서 시작합니다")
        self._publish_status()
        self._speaker.say(f"{fl.label} {label} 에서 시작합니다")

    def _fail_floor(self, reason: str) -> None:
        self._localized = False
        self._state.set(status="fault", message=f"층을 바꾸지 못했습니다: {reason}",
                        transfer={
            "state": "fault", "message": f"층을 바꾸지 못했습니다: {reason}",
            "detail": "지도와 waypoint 파일을 확인한 뒤 다시 골라 주세요",
            "actions": [], "target": "", "here": ""})
        self._publish_status()
        self.get_logger().error(f"층 전환 실패: {reason}")
        self._speaker.say("층을 바꾸지 못했습니다. 화면을 확인해 주세요")

    # ── 지도 교체 ─────────────────────────────────────────────────────────
    def load_map(self, floor, timeout_s: float = 20.0) -> tuple[bool, str]:
        """map_server 에 그 층 지도를 올리고 **결과를 확인한다.**

        BasicNavigator.changeMap() 을 안 쓴다. 그 메서드는 실패하면 error 로그만
        찍고 아무것도 안 돌려준다(robot_navigator.py:647-660). 실패를 못 알아채면
        **다른 층 지도로 길을 찾는다** — 이 기능에서 제일 위험한 조용한 실패다.
        게다가 서비스가 안 뜨면 무한히 기다린다. 그래서 클라이언트만 빌려 쓰고
        호출은 여기서 한다.
        """
        path = os.path.expanduser(floor.map_path)
        if not os.path.exists(path):
            return False, f"지도 파일이 없다: {path}"
        ok, err = self._check_amcl_accepts_new_maps()
        if not ok:
            return False, err
        cli = self._nav.change_maps_srv
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, ("map_server/load_map 서비스가 없다 — Nav2 가 안 떠 있거나 "
                           "map_server 가 active 가 아니다")
        req = LoadMap.Request()
        req.map_url = path
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self._nav, fut, timeout_sec=timeout_s)
        if not fut.done():
            fut.cancel()
            return False, f"map_server 가 {timeout_s:.0f}초 안에 답하지 않았다"
        res = fut.result()
        if res is None:
            return False, "map_server 응답이 비었다"
        if res.result != LoadMap.Response().RESULT_SUCCESS:
            return False, LOAD_MAP_REASON.get(res.result, f"코드 {res.result}")

        # 올라간 지도가 정말 그 층 것인지 크기로 한 번 더 본다.
        info = getattr(floor, "map_info", None)
        got = res.map.info
        if info is not None and (got.width, got.height) != (info.width, info.height):
            return False, (f"올라간 지도 크기가 다르다 "
                           f"({got.width}×{got.height} ≠ {info.width}×{info.height})")
        self.get_logger().info(
            f"지도 교체: {path} ({got.width}×{got.height} @ {got.resolution:.3f})")
        return True, ""

    def _check_amcl_accepts_new_maps(self) -> tuple[bool, str]:
        """AMCL 이 `first_map_only` 로 잠겨 있지 않은지 한 번 확인한다.

        이게 true 면 map_server 는 새 지도를 올리고 응답도 성공으로 주는데
        **AMCL 만 옛 지도를 계속 본다.** 로봇은 11층 복도를 10층 지도에 맞추며
        멀쩡히 주행하는 것처럼 보인다 — 실패가 어디에도 안 찍힌다.
        기본값은 false 고 nav2_params.yaml 에도 안 적혀 있지만, 그건 지금
        얘기다. 한 번 물어보는 값이 싸다.
        """
        if getattr(self, "_amcl_map_ok", False):
            return True, ""
        try:
            from rcl_interfaces.srv import GetParameters
        except ImportError:
            return True, ""
        cli = getattr(self, "_amcl_param_cli", None)
        if cli is None:
            cli = self.create_client(GetParameters, "/amcl/get_parameters")
            self._amcl_param_cli = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            # amcl 이 없으면 어차피 뒤의 재초기화에서 걸린다. 여기서 막지 않는다.
            self.get_logger().warn("amcl 파라미터를 못 물어봤다 — 확인을 건너뛴다")
            return True, ""
        req = GetParameters.Request()
        req.names = ["first_map_only"]
        fut = cli.call_async(req)
        # 콜백은 executor 가 돌린다. 여기서는 기다리기만 한다.
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 3.0:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None or not fut.result().values:
            self.get_logger().warn("amcl 파라미터 응답이 없다 — 확인을 건너뛴다")
            return True, ""
        v = fut.result().values[0]
        if getattr(v, "type", 0) == 1 and v.bool_value:
            return False, ("amcl 의 first_map_only 가 true 다 — 지도를 갈아도 AMCL 은 "
                           "옛 지도를 계속 본다. nav2_params.yaml 의 amcl 에서 "
                           "false 로 둘 것")
        self._amcl_map_ok = True
        return True, ""

    # ── AMCL 재초기화 ─────────────────────────────────────────────────────
    def _relocalize_at(self, name: str, timeout_s: float = 20.0) -> tuple[bool, str]:
        """초기 위치를 주고 **정말 잡혔는지 확인**한다.

        지도만 갈고 초기 위치를 안 주면 AMCL 이 map->odom 을 안 낸다. 그러면
        frame_id=map 목표를 변환할 수 없어 어떤 목적지도 안 가는데 **로그는
        조용하고 goal 은 접수된 것처럼 보인다.** 오늘 고친 그 문제와 같다.

        그래서 세 가지를 본다.
          1) 초기 위치를 준 **뒤에** 새 amcl_pose 가 왔는가
          2) 그 값이 우리가 준 자리 근처인가 (엉뚱한 데 수렴하면 실패로 본다)
          3) map->odom TF 가 실제로 나오는가
        """
        wp = self._waypoints.get(name)
        if wp is None:
            return False, f"'{name}' 은 등록되지 않은 지점이다"
        self._localized = False
        # 여기서 최대 20초 걸린다. 화면이 멈춘 것처럼 보이면 안 된다.
        self._state.set(message=f"'{getattr(wp, 'label', name)}' 에서 위치를 잡는 중입니다")
        self._publish_status()
        pose = self._to_pose(name, wp)

        with self._amcl_lock:
            self._amcl, self._amcl_at = None, 0.0
        sent_at = time.monotonic()
        self._nav.setInitialPose(pose)

        # 처음 한 번은 놓치기 쉽다 (AMCL 이 아직 구독을 안 붙였을 수 있다).
        # 콜백은 executor 가 돌리므로 여기서는 자기만 하고 다시 쏜다.
        deadline = sent_at + timeout_s
        seen = None
        tick = 0
        while time.monotonic() < deadline:
            time.sleep(0.5)
            with self._amcl_lock:
                if self._amcl is not None and self._amcl_at > sent_at:
                    seen = self._amcl
                    break
            tick += 1
            if tick % 4 == 0:      # 2초마다 다시 쏜다 (로그를 덜 더럽힌다)
                self._nav._setInitialPose()
        if seen is None:
            return False, (f"AMCL 이 {timeout_s:.0f}초 안에 amcl_pose 를 안 냈다 "
                           f"(amcl 이 죽었거나 지도를 아직 못 받았다)")

        dx, dy = seen[0] - wp.x, seen[1] - wp.y
        if (dx * dx + dy * dy) ** 0.5 > 2.0:
            return False, (f"AMCL 이 준 자리와 {(dx*dx+dy*dy)**0.5:.1f} m 떨어진 곳에 "
                           f"수렴했다 — 지도와 waypoint 의 층이 어긋난 것 아닌가")

        if self._tf_buffer is not None:
            ok_tf = False
            for _ in range(20):
                if self._tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
                    ok_tf = True
                    break
                time.sleep(0.25)
            if not ok_tf:
                return False, "map->odom TF 가 안 나온다 (AMCL 이 초기 위치를 못 먹었다)"

        self._localized = True
        # 옛 층 장애물이 코스트맵에 남아 있으면 새 층에서 유령 벽이 된다.
        self._clear_costmaps()
        return True, ""

    def _clear_costmaps(self) -> None:
        """BasicNavigator.clearAllCostmaps() 는 서비스가 없으면 영원히 기다린다.
        층 전환 중에 거기서 멈추면 안 되므로 시간을 걸어 부른다."""
        from nav2_msgs.srv import ClearEntireCostmap
        for cli, what in ((self._nav.clear_costmap_global_srv, "global"),
                          (self._nav.clear_costmap_local_srv, "local")):
            try:
                if not cli.wait_for_service(timeout_sec=2.0):
                    self.get_logger().warn(f"{what} 코스트맵 초기화 서비스가 없다 — 건너뛴다")
                    continue
                fut = cli.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self._nav, fut, timeout_sec=5.0)
            except Exception as e:
                self.get_logger().warn(f"{what} 코스트맵 초기화 실패 (계속 진행): {e}")

    def wait_for_nav2(self, timeout_s: float = 30.0) -> bool:
        """Nav2 lifecycle 이 active 가 될 때까지 기다린다.

        이걸 안 하면 첫 goToPose 가 아직 안 뜬 액션 서버로 가서 조용히
        버려진다. BasicNavigator 가 내부에서 서버를 기다리긴 하지만,
        여기서 한 번 걸어 두면 UI 가 "준비 중" 을 표시할 수 있다.
        """
        t0 = time.time()
        self._state.set(status="idle", message="Nav2 를 기다리는 중입니다")
        try:
            self._nav.waitUntilNav2Active(localizer="amcl" if self._localized else None)
        except Exception as exc:
            self.get_logger().warn(f"Nav2 대기 중 예외 (계속 진행): {exc}")
            return False
        self.get_logger().info(f"Nav2 준비됨 ({time.time() - t0:.1f}초)")
        return True

    # ── 안내 ──────────────────────────────────────────────────────────────
    def _floor_of(self, name: str) -> tuple[str, str]:
        """이 목적지가 어느 층 것인가. (층키, 오류)."""
        if name in self._waypoints:
            return self._floor, ""
        hits = self._book.find_destination(name)
        if not hits:
            return "", f"'{name}' 은 등록되지 않은 목적지다"
        if len(hits) > 1:
            names = " / ".join(f.label for f in hits)
            return "", f"'{name}' 이 여러 층에 있다 ({names}). 층을 골라 주세요"
        return hits[0].key, ""

    def start_guiding(self, name: str, floor: str = "") -> tuple[bool, str]:
        if self._task and self._task.is_alive():
            return False, "이미 안내 중이다"
        if not self._localized:
            return False, ("초기 위치를 먼저 정해야 한다. 로봇을 아는 지점에 놓고 "
                           "'여기서 시작' 에서 그 지점을 누를 것")

        # 한 층만 도는 구성 — 예전 그대로.
        if self._book is None:
            if name not in self._waypoints:
                return False, f"'{name}' 은 등록되지 않은 목적지다"
            self._task = threading.Thread(target=self._guide_loop, args=(name,),
                                          daemon=True)
            self._task.start()
            return True, ""

        target = floor
        if not target:
            target, err = self._floor_of(name)
            if err:
                return False, err
        if self._book.get(target) is None:
            return False, f"'{target}' 는 모르는 층이다"

        if target == self._floor:
            if name not in self._waypoints:
                return False, f"'{name}' 은 이 층에 없다"
            self._task = threading.Thread(target=self._guide_loop, args=(name,),
                                          daemon=True)
            self._task.start()
            return True, ""

        # ── 층이 다르다. 엘리베이터를 타야 한다 ────────────────────────────
        if not self._floor:
            return False, "지금 층을 모른다. '여기서 시작' 에서 층과 지점을 골라 주세요"
        here = self._book.get(self._floor)
        there = self._book.get(target)
        if not here.board or here.board not in self._waypoints:
            return False, (f"{here.label} 에 엘리베이터 지점('{here.board}')이 없다 — "
                           f"floors.yaml 과 waypoint 를 확인할 것")
        if not there.usable:
            why = "; ".join(p.text for p in there.errors) or "지도/waypoint 문제"
            return False, f"{there.label} 로는 갈 수 없다: {why}"
        if name not in there.waypoints:
            return False, f"{there.label} 에 '{name}' 이 없다"

        self._task = threading.Thread(target=self._mission_loop,
                                      args=(target, name), daemon=True)
        self._task.start()
        return True, ""

    def transfer_event(self, event: str, **data) -> tuple[bool, str]:
        """UI 에서 온 사람의 응답 ('탔습니다' 같은 것)."""
        if self._gate is None:
            return False, "층 전환 기능이 꺼져 있습니다"
        if self._transfer is None:
            return False, "지금은 층 이동 중이 아닙니다"
        return self._gate.post(event, **data)

    def _mission_loop(self, target: str, destination: str) -> None:
        """층을 넘는 안내 하나. 상태머신은 guide_mission 이 돌리고
        여기서는 그 결과를 화면·주행에 옮긴다."""
        fx = NavEffects(self)
        self._transfer = mission.FloorTransfer(
            self._book, fx, gate=self._gate,
            log=lambda m: self.get_logger().info(m))
        self._state.set(status="transfer", destination=destination)
        try:
            out = self._transfer.run(self._floor, target, destination)
        except Exception as e:          # 여기서 새면 로봇이 어중간하게 남는다
            self.get_logger().error(f"층 전환이 예외로 끝났다: {e}")
            self._nav.cancelTask()
            self._localized = False
            self._state.set(status="fault", message=f"층 이동 중 오류: {e}")
            self._publish_status()
            self._transfer = None
            return

        # **self._floor 는 건드리지 않는다.** 이 값의 뜻은 "지금 map_server 에
        # 올라가 있는 지도의 층" 이고, 그건 NavEffects.load_map 이 성공했을 때만
        # 바뀐다. 상태머신이 믿는 층(transfer.here)을 여기서 덮으면, 지도 교체가
        # 실패했는데도 새 층이라고 적히고 self._waypoints 는 옛 층인 상태가
        # 남는다. 그 상태에서 사람이 '여기서 시작' 을 누르면 같은 층이라고
        # 판단해 지도를 안 갈고 새 층 좌표로 초기 위치를 준다 — 조용한 사고다.
        self._localized = bool(out.localized)
        if out.ok:
            label = self._label(destination)
            self._state.set(status="arrived", transfer=None, distance=0.0,
                            message=f"{label} 에 도착했습니다")
            self._speaker.say(f"{label} 에 도착했습니다. 안내를 마치겠습니다")
        elif out.state == mission.ABORTED and not out.needs_manual_start:
            # 타기 전에 접었다. 로봇은 제 위치를 알고 있으므로 목적지 화면으로
            # 그냥 돌아간다.
            self._state.set(status="idle", transfer=None, distance=0.0,
                            message="안내를 취소했습니다")
        else:
            # fault 이거나 위치를 잃은 취소다. **화면을 그대로 둔다** — 무엇이
            # 잘못됐는지와 사람이 할 일이 거기 적혀 있다. UI 가 '층·위치 다시
            # 잡기' 버튼을 붙여 준다.
            self._state.set(status="fault", distance=0.0)
        self._sync_floor_label()
        self._publish_status()
        self._transfer = None

    def cancel(self) -> None:
        # 층 이동 중이면 상태머신에 먼저 알린다. 그래야 '취소했다' 를 사람에게
        # 말하고, 엘리베이터 안이었으면 초기 위치를 버린 상태로 끝난다.
        if self._transfer is not None and self._gate is not None:
            ok, _ = self._gate.post(mission.ABORT)
            if ok:
                self._nav.cancelTask()
                return
        self._nav.cancelTask()
        self._state.set(status="idle", destination="", message="안내를 취소했습니다",
                        distance=0.0, transfer=None)
        self._speaker.say("안내를 취소합니다")

    # ── 카메라 ────────────────────────────────────────────────────────────
    def _on_compressed(self, msg: CompressedImage) -> None:
        self._jpeg = bytes(msg.data)

    def _on_raw(self, msg: Image) -> None:
        self._raw = msg

    def _latest_jpeg(self) -> bytes | None:
        """압축 토픽이 있으면 그대로, 없으면 원본을 PIL 로 굽는다."""
        if self._jpeg is not None:
            return self._jpeg
        msg = self._raw
        if msg is None:
            return None
        try:
            import io

            import numpy as np
            from PIL import Image as PILImage

            enc = msg.encoding.lower()
            if enc not in ("rgb8", "bgr8"):
                self.get_logger().warn(f"다룰 수 없는 인코딩: {msg.encoding}")
                return None
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if enc == "bgr8":
                arr = arr[:, :, ::-1]
            buf = io.BytesIO()
            PILImage.fromarray(arr).save(buf, format="JPEG", quality=80)
            return buf.getvalue()
        except Exception as e:  # 판단은 부가 기능이다 — 주행을 막지 않는다
            self.get_logger().warn(f"JPEG 인코딩 실패: {e}")
            return None

    # ── 돌발상황 ──────────────────────────────────────────────────────────
    def _handle_obstacle(self, name: str, stuck_seconds: float) -> bool:
        """VLM 에 한 장 물어보고 행동한다. 안내를 접어야 하면 True."""
        jpeg = self._latest_jpeg()
        if jpeg is None:
            self.get_logger().warn("카메라 영상이 없어 판단을 건너뛴다")
            return False

        action, say = self._brain.judge_obstacle(jpeg, stuck_seconds)
        self.get_logger().info(f"돌발상황 판단: {action} ({stuck_seconds:.0f}초 정체)")
        if say:
            self._speaker.say(say)

        if action == "resume":
            return False
        if action in ("wait", "ask_to_move"):
            self._state.set(message="길이 막혀 기다리는 중입니다")
            self._publish_status()
            return False
        if action == "reroute":
            # 코스트맵의 낡은 장애물을 털고 같은 목표로 다시 계획한다
            self._state.set(message="다른 경로로 돌아갑니다")
            self._publish_status()
            self._nav.clearAllCostmaps()
            self._nav.goToPose(self._to_pose(name))
            return False
        if action == "alert":
            self._state.set(status="alert", message="위급 상황으로 정지했습니다", distance=0.0)
            self._publish_status()
            self._nav.cancelTask()
            return True
        return False

    # ── 사람 추종 ─────────────────────────────────────────────────────────
    def _wait_for_follower(self, name: str) -> bool:
        """뒤처진 사람을 기다린다. 다시 따라오면 True, 포기하면 False.

        Nav2 에는 '일시정지' 가 없다. 취소하고 다시 goToPose 하는 것이
        같은 효과이고, 그 사이 로봇은 제자리에 선다.
        """
        LOST_LIMIT = 60.0  # 이보다 오래 안 오면 안내를 접는다

        self._nav.cancelTask()
        self._state.set(status="waiting", message="따라오실 때까지 기다립니다")
        self._publish_status()
        self._speaker.say("잠시 기다리겠습니다")

        t0 = time.monotonic()
        nagged = False
        while time.monotonic() - t0 < LOST_LIMIT:
            # spin 하지 않는다. main 의 MultiThreadedExecutor 가 이 노드를
            # 이미 돌리고 있어서, 여기서 또 spin_once 를 부르면 같은 노드를
            # 두 스레드가 돌린다 (rclpy 가 하지 말라고 명시한 것).
            # 콜백은 executor 가 알아서 돈다 — 여기서는 자기만 하면 된다.
            time.sleep(0.2)
            st = self._follower.poll()
            # 탐지가 죽으면 '모름' 이다. 모른다고 계속 서 있으면 안 되므로
            # 안내를 재개한다 — 탐지기 장애로 복도에 멈춰 서는 편이 더 나쁘다.
            if not st.known:
                self.get_logger().info("탐지 불명 — 안내를 재개한다")
                break
            if st.present:
                self._speaker.say("다시 안내하겠습니다")
                break
            if not nagged and time.monotonic() - t0 > 15.0:
                self._speaker.say("괜찮으세요? 이쪽입니다")
                nagged = True
        else:
            self._state.set(status="idle", message="따라오지 않아 안내를 마칩니다", distance=0.0)
            self._publish_status()
            self._speaker.say("안내를 마치겠습니다. 필요하시면 다시 불러 주세요")
            return False

        self._state.set(status="navigating", message=f"{ro(self._label(name))} 안내합니다")
        self._publish_status()
        self._nav.goToPose(self._to_pose(name))
        return True

    def _guide_loop(self, name: str) -> None:
        label = self._label(name)
        self._speaker.say(f"{ro(label)} 안내하겠습니다. 뒤따라와 주세요")
        ok, code, text = self._drive(name)
        if ok:
            self._state.set(status="arrived", message=f"{label} 에 도착했습니다",
                            distance=0.0)
            self._speaker.say(f"{label} 에 도착했습니다. 안내를 마치겠습니다")
        elif code == "canceled":
            self._state.set(status="idle", message="안내를 취소했습니다", distance=0.0)
        elif code in ("follower_lost", "alert"):
            pass          # 그 안에서 이미 화면과 음성으로 알렸다
        else:
            self._state.set(status="failed", message=text, distance=0.0)
            self._speaker.say("경로를 찾지 못했습니다. 잠시 후 다시 시도해 주세요")
        self._publish_status()

    def _drive_to(self, name: str) -> tuple[bool, str]:
        """상태머신(NavEffects)이 부르는 얼굴. (성공, 사람이 읽을 이유)."""
        ok, _code, text = self._drive(name)
        return ok, text

    def _drive(self, name: str) -> tuple[bool, str, str]:
        """이 층 안에서 한 지점까지 데려간다. 도착할 때까지 블로킹.

        일반 안내와 '엘리베이터 앞으로 이동' 이 같은 몸통을 쓴다 — 뒤처진
        사람을 기다리는 것도, 막혔을 때 VLM 에 물어보는 것도 똑같이 필요하다.
        """
        label = self._label(name)
        self._state.set(status="navigating", destination=name,
                        message=f"{ro(label)} 안내합니다")
        self._publish_status()

        self._nav.goToPose(self._to_pose(name))

        # 정체 판정. 남은 거리가 STUCK_EPS 이상 줄지 않은 채 STUCK_AFTER 초가
        # 지나면 막힌 것으로 본다. VLM 호출은 몇 초 걸리고 관제 노트북을 타므로
        # COOLDOWN 을 두어 연타하지 않는다.
        STUCK_EPS, STUCK_AFTER, COOLDOWN = 0.05, 6.0, 15.0
        # 뒤처짐 판정. 순간 미검출로 멈춰 서면 안 되므로 연속 LOST_AFTER 초
        # 동안 안 보일 때만 기다린다. 사람이 문틀에 잠깐 가리는 일은 흔하다.
        FOLLOW_PERIOD, LOST_AFTER, TOO_FAR_M = 1.0, 4.0, 4.0
        best = None
        moved_at = time.monotonic()
        judged_at = 0.0
        seen_at = time.monotonic()
        polled_at = 0.0

        while not self._nav.isTaskComplete():
            fb = self._nav.getFeedback()
            now = time.monotonic()
            if fb:
                dist = float(fb.distance_remaining)
                self._state.set(distance=dist)
                if best is None or best - dist > STUCK_EPS:
                    best, moved_at = dist, now
                elif (
                    self._brain is not None
                    and now - moved_at >= STUCK_AFTER
                    and now - judged_at >= COOLDOWN
                ):
                    judged_at = now
                    if self._handle_obstacle(name, now - moved_at):
                        return False, "alert", "위급 상황으로 정지했다"
                    moved_at = time.monotonic()  # 판단 뒤 다시 관찰

            # 따라오고 있나. 안내로봇은 앞장서므로 이걸 안 보면 사람을 두고 간다.
            if self._follower is not None and now - polled_at >= FOLLOW_PERIOD:
                polled_at = now
                st = self._follower.poll()
                if not st.known:
                    seen_at = now  # 모르면 있다고 친다
                elif st.present and st.distance_m <= TOO_FAR_M:
                    seen_at = now
                    self._state.set(follower_m=round(st.distance_m, 1))
                elif now - seen_at >= LOST_AFTER:
                    if not self._wait_for_follower(name):
                        return False, "follower_lost", "따라오지 않아 안내를 마쳤다"
                    best, moved_at, seen_at = None, time.monotonic(), time.monotonic()
            # 여기도 마찬가지다 — executor 가 콜백을 돌린다 (위 주석 참조).
            time.sleep(0.1)

        result = self._nav.getResult()
        if result == TaskResult.SUCCEEDED:
            return True, "", ""
        if result == TaskResult.CANCELED:
            return False, "canceled", "안내를 취소했다"
        return False, "failed", "경로를 찾지 못했습니다"

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self._state.snapshot(), ensure_ascii=False)
        self._status_pub.publish(msg)

    def _on_destination_topic(self, msg: String) -> None:
        ok, err = self.start_guiding(msg.data.strip())
        if not ok:
            self.get_logger().warn(err)

    # ── 음성 입력 (PTT) ───────────────────────────────────────────────────
    def listen_and_go(self) -> tuple[bool, str]:
        """버튼 한 번 = 한 마디 듣고 목적지로 출발.

        해석은 두 단계다. 문자열 매칭을 먼저 하고, 실패했을 때만 LLM 에 넘긴다.
        빠르고 확실한 경로를 우선해야 지연도 오판도 줄어든다.
        """
        if not self._listener or not self._listener.ready:
            return False, "음성 입력이 준비되지 않았습니다"
        if self._listener.busy:
            return False, "이미 듣고 있습니다"

        threading.Thread(target=self._listen_loop, daemon=True).start()
        return True, ""

    def _listen_loop(self) -> None:
        prev = self._state.snapshot()
        self._state.set(message="말씀해 주세요", status="listening")

        text, wav = self._listener.listen_once()
        if not text and not wav:
            self._state.set(status=prev["status"], message="잘 못 들었습니다. 다시 말씀해 주세요")
            return

        if text:
            self._state.set(message=f"'{text}'")
        dest = self._resolve(text, wav)

        if not dest:
            self._state.set(status=prev["status"], message="어디로 갈지 알아듣지 못했습니다")
            self._speaker.say("어디로 갈지 알아듣지 못했습니다. 화면에서 골라 주세요")
            return

        ok, err = self.start_guiding(dest)
        if not ok:
            self._state.set(status=prev["status"], message=err)

    def _resolve(self, text: str | None, wav: bytes | None = None) -> str | None:
        """발화 → 목적지. 싼 경로부터 차례로 시도한다.

          1) Whisper 텍스트 + 문자열 매칭 — 온보드, 1~2초, 네트워크 불필요
          2) Whisper 텍스트 + LLM        — "삼백사호" 같은 표현을 메운다

        1) 에서 끝나면 관제에 아무것도 안 보낸다.

        원본 오디오를 LLM 에 직접 넘기는 3단계도 생각했지만 뺐다 — ollama 가
        오디오 입력을 안 넘겨줘서 근거 없는 답이 나온다 (brain.py 주석 참조).
        wav 인자는 그때를 대비해 자리만 남겨 둔다.
        """
        # 층이 있으면 다른 층 목적지도 알아들어야 한다 — 어느 층인지는
        # start_guiding 이 알아서 찾는다. 표시 이름(labels)으로도 맞춰 본다:
        # 사람이 말하는 것은 "1004호 강의장" 이지 "10a" 가 아니다.
        spoken: dict[str, str] = {}     # 말한 표현 → waypoint 이름
        if self._book is None:
            for name, w in self._waypoints.items():
                spoken[name] = name
        else:
            for fl in self._book.ordered():
                for name in fl.waypoints:
                    spoken.setdefault(name, name)
                    spoken.setdefault(fl.label_of(name), name)
        names = list(spoken)

        if text:
            flat = text.replace(" ", "")
            # 긴 이름부터 본다 — "1004호" 가 "1004호 강의장" 을 가로채지 않게
            for phrase in sorted(names, key=len, reverse=True):
                if phrase.replace(" ", "") in flat:
                    self.get_logger().info(f"[매칭] {phrase} → {spoken[phrase]}")
                    return spoken[phrase]

        if text and self._brain:
            dest = self._brain.resolve_destination(text, names)
            if dest:
                self.get_logger().info(f"[LLM] {dest}")
                return spoken.get(dest, dest)
        return None



# ── 상태머신이 실제로 부리는 손발 ─────────────────────────────────────────
class NavEffects:
    """guide_mission.transfer 가 시키는 일을 진짜 로봇에 옮긴다.

    상태머신은 ROS 를 모른다 (그래야 로봇 없이 시험이 돈다). 그 경계가 여기다.
    **어떤 메서드도 예외를 밖으로 던지지 않는다** — 실패는 (False, 이유) 로
    돌려주고, 상태머신이 그걸 받아 fault 로 간다.
    """

    def __init__(self, node: "GuideNode") -> None:
        self.node = node

    def announce(self, *, state, message, speak, actions, target, here, detail) -> None:
        n = self.node
        book = n._book
        payload = {
            "state": state,
            "message": message,
            "detail": detail,
            "actions": [{"event": e, "label": mission.BUTTON_LABEL.get(e, e)}
                        for e in actions],
            "target": target,
            "target_label": book.get(target).label if book.get(target) else target,
            "here": here or "",
            "here_label": book.get(here).label if (here and book.get(here)) else "",
            # 층을 물을 때 UI 가 그릴 목록
            "floors": [{"key": f.key, "label": f.label, "usable": f.usable}
                       for f in book.ordered()],
        }
        n._state.set(status="transfer", message=message, transfer=payload)
        n._publish_status()
        n.get_logger().info(f"[층전환:{state}] {message}")
        if speak:
            n._speaker.say(speak)

    def navigate(self, floor_key, waypoint):
        n = self.node
        if floor_key != n._floor:
            return False, f"'{floor_key}' 층 지도가 아니다 (지금 '{n._floor}')"
        if waypoint not in n._waypoints:
            return False, f"'{waypoint}' 지점이 이 층에 없다"
        if not n._localized:
            return False, "초기 위치가 없다"
        return n._drive_to(waypoint)

    def hold(self):
        try:
            self.node._nav.cancelTask()
        except Exception as e:
            self.node.get_logger().warn(f"정지 요청 실패 (계속 진행): {e}")

    def set_localized(self, ok: bool):
        self.node._localized = bool(ok)

    def load_map(self, floor):
        n = self.node
        ok, err = n.load_map(floor)
        if not ok:
            return False, err
        # 지도가 바뀌었으면 이 노드가 보는 층·목적지도 같이 바뀐다.
        n._floor = floor.key
        n._waypoints = n._floor_waypoints(floor.key)
        n._sync_floor_label()
        return True, ""

    def relocalize(self, floor, waypoint):
        return self.node._relocalize_at(waypoint)

    def detect_floor(self):
        n = self.node
        if n._detector is None:
            from guide_mission.detect import DISABLED, FloorGuess
            return FloorGuess(None, None, False, DISABLED,
                              "자동 층 판정이 꺼져 있습니다. 층을 골라 주세요")
        try:
            g = n._detector.guess()
        except Exception as e:
            from guide_mission.detect import NO_TOOL, FloorGuess
            n.get_logger().warn(f"층 판정 실패: {e}")
            return FloorGuess(None, None, False, NO_TOOL,
                              f"층을 자동으로 알 수 없습니다 ({e}). 층을 골라 주세요")
        n.get_logger().info(f"층 판정: {g.floor or '모름'} — {g.reason}")
        return g

    def resume(self, destination):
        return self.node._drive_to(destination)


# ── 웹 UI ─────────────────────────────────────────────────────────────────
def make_handler(node: GuideNode, state: GuideState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):  # 요청마다 콘솔을 더럽히지 않는다
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(WEB_DIR, "index.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/api/destinations":
                self._json(node.catalog())
            elif self.path == "/api/status":
                s = state.snapshot()
                s["can_listen"] = bool(node._listener and node._listener.ready)
                s["localized"] = node._localized
                self._json(s)
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or b"{}")

            if self.path == "/api/go":
                ok, err = node.start_guiding(payload.get("destination", ""),
                                             payload.get("floor", ""))
                self._json({"ok": ok, "error": err}, 200 if ok else 400)
            elif self.path == "/api/cancel":
                node.cancel()
                self._json({"ok": True})
            elif self.path == "/api/start-here":
                ok, err = node.set_start(payload.get("waypoint", ""),
                                         payload.get("floor", ""))
                self._json({"ok": ok, "error": err}, 200 if ok else 400)
            elif self.path == "/api/transfer":
                # 사람이 '탔습니다' 같은 버튼을 눌렀다. 상태머신 창구로 넘긴다.
                event = payload.get("event", "")
                data = {k: v for k, v in payload.items() if k != "event"}
                ok, err = node.transfer_event(event, **data)
                self._json({"ok": ok, "error": err}, 200 if ok else 400)
            elif self.path == "/api/listen":
                ok, err = node.listen_and_go()
                self._json({"ok": ok, "error": err}, 200 if ok else 400)
            else:
                self._send(404, b"not found", "text/plain")

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--waypoints", default="~/waypoints.yaml", help="teleop_key.py 가 찍어 둔 YAML")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--voice", default="", help="piper 음성 모델 경로 (없으면 무음 폴백)")
    parser.add_argument("--audio-device", default="", help="aplay -D 값 (예: plughw:2,0)")
    parser.add_argument("--mic", default="", help="arecord -D 값 (예: plughw:1,0). 주면 PTT 가 켜진다")
    parser.add_argument("--stt-model", default="tiny", help="whisper 모델 (tiny/base/small)")
    parser.add_argument("--llm", default="", help="ollama 모델 이름 (예: gemma4:e2b). 주면 LLM 해석이 켜진다")
    # 기본은 온보드(localhost)다. 관제 노트북에 두려면 그쪽 주소를 준다 —
    # 젯슨 8GB 에 안 들어가는 큰 모델은 그렇게 쓴다. 판단이 이벤트 기반이라
    # 네트워크가 끊겨도 nav2 기본 동작으로 떨어질 뿐 주행은 계속된다.
    parser.add_argument("--llm-url", default="", help="ollama 주소 (예: http://192.168.129.97:11434/api/generate)")
    parser.add_argument("--start-waypoint", default="",
                        help="시작 위치로 쓸 waypoint 이름. 주면 기동 직후 AMCL 초기 위치를 "
                             "거기로 잡는다. 안 주면 UI 의 '여기서 시작' 에서 고른다")
    parser.add_argument("--follow-url", default="",
                        help="후면 사람 탐지 서비스 (예: http://localhost:8641/follower). "
                             "주면 뒤처진 사람을 기다린다")
    parser.add_argument("--floors", default="",
                        help="층별 지도·waypoint 대장 (fleet/guide_mission 의 floors.yaml). "
                             "주면 층 전환이 켜지고 --waypoints 는 무시된다")
    parser.add_argument("--floor", default="",
                        help="기동 시점의 층 키 (예: 10f). --floors 와 같이 준다. "
                             "안 주면 UI 에서 층부터 고른다")
    parser.add_argument("--floor-detect", default="auto", choices=["auto", "always", "off"],
                        help="층 자동 판정. auto=SSID 가 확실하면 그대로 진행, "
                             "always=매번 사람 확인, off=아예 안 봄")
    args, ros_args = parser.parse_known_args()

    # ── 층 대장 ───────────────────────────────────────────────────────────
    # --floors 를 줬는데 못 쓰면 **기동을 거부한다.** 조용히 한 층짜리로
    # 떨어지면 다른 층 목적지 버튼이 그냥 안 보이고, 왜 안 보이는지 아무도
    # 모른 채 현장에서 시간을 버린다.
    book = detector = None
    if args.floors:
        if not HAVE_MISSION:
            parser.error(
                f"--floors 를 줬는데 guide_mission 을 임포트할 수 없다 "
                f"({MISSION_IMPORT_ERROR}). "
                f"colcon build --packages-select guide_mission 을 먼저 할 것")
        try:
            book = FloorBook.load(args.floors)
        except Exception as e:
            parser.error(f"floors.yaml 을 못 읽는다: {e}")
        bad = [p for p in book.problems if p.level == ERROR]
        for p in book.problems:
            print(f"  {p}")
        if bad:
            parser.error(
                "floors.yaml 에 오류가 있다 (위 ✗ 표시). "
                "`ros2 run guide_mission check_floors` 로 확인할 것")
        if args.floor and book.get(args.floor) is None:
            parser.error(f"--floor {args.floor} 는 floors.yaml 에 없다")
        detector = FloorDetector(book, policy=args.floor_detect)

    rclpy.init(args=ros_args)
    speaker = Speaker(voice=args.voice, device=args.audio_device)
    # 둘 다 선택적이다. 없으면 그 기능만 빠지고 버튼 UI 는 그대로 돈다.
    listener = Listener(args.stt_model, args.mic) if args.mic else None
    brain_kw = {"model": args.llm}
    if args.llm_url:
        brain_kw["url"] = args.llm_url
    brain = Brain(**brain_kw) if args.llm else None
    follower = Follower(url=args.follow_url) if args.follow_url else None
    state = GuideState()
    node = GuideNode(args.waypoints, speaker, state, listener, brain, follower,
                     book=book, detector=detector, floor=args.floor)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(node, state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"UI: http://localhost:{args.port}")

    if book is not None:
        node.get_logger().info(
            f"층 {len(book.floors)}개: "
            + ", ".join(f"{f.label}({f.key})" for f in book.ordered())
            + f" · 지금 층 {args.floor or '미정'} · 자동 판정 {args.floor_detect}")
        if detector is not None and args.floor_detect != "off":
            g = detector.guess()
            node.get_logger().info(f"무선으로 본 층: {g.floor or '모름'} — {g.reason}")

    if args.start_waypoint:
        # **스레드로 돌린다.** set_start 는 amcl_pose 가 올 때까지 기다리는데,
        # 그 콜백은 아래 executor 가 돌아야 온다. 여기서 그냥 부르면 executor
        # 가 아직 안 도는 상태에서 20초를 기다리다 반드시 실패한다.
        def _start_later():
            ok, err = node.set_start(args.start_waypoint, args.floor)
            if not ok:
                node.get_logger().warn(
                    f"시작 위치를 못 잡았다: {err} — UI 의 '여기서 시작' 에서 "
                    f"다시 골라 주세요")

        threading.Thread(target=_start_later, daemon=True).start()
    else:
        node.get_logger().warn(
            "초기 위치가 없다. UI 의 '여기서 시작' 에서 지점을 고르기 전까지 "
            "AMCL 이 map->odom 을 못 내고, 그러면 어떤 목적지도 가지 않는다."
        )

    # 여러 스레드가 같은 노드를 돌린다 — 메인 spin, _guide_loop 워커,
    # _wait_for_follower 워커, 그리고 BasicNavigator 의 spin_until_future_complete.
    # 기본 SingleThreadedExecutor 를 인자 없이 쓰면 넷이 **같은 전역 executor**
    # 를 공유해서, 워커가 goal 을 보내는 동안 카메라·/guide/destination 콜백이
    # 간헐적으로 안 돈다. rclpy 문서도 같은 노드를 여러 스레드에서 spin 하지
    # 말라고 한다. 명시적으로 다중 스레드 executor 를 준다.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

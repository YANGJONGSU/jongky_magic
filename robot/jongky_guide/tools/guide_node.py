#!/usr/bin/env python3
"""안내로봇 본체 노드 — 목적지를 받아 Nav2 로 데려가고 음성으로 안내한다.

역할이 셋이다.

  1. 웹 UI(터치스크린)를 띄우고 목적지 요청을 받는다
  2. waypoint 를 Nav2 목표로 바꿔 보내고 도착을 지켜본다
  3. 상태가 바뀔 때마다 TTS 로 말한다

waypoint 는 teleop_key.py 가 맵핑 주행 중에 찍어 둔 YAML 을 읽는다.
같은 좌표계(map)이므로 그대로 Nav2 goal 이 된다.

실행:
    ros2 run jongky_guide guide_node.py
    ros2 run jongky_guide guide_node.py --waypoints ~/waypoints_10f.yaml --port 8080

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
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from brain import Brain
from follow_client import Follower
from listen import Listener
from speech import Speaker

WEB_DIR = os.path.join(get_package_share_directory("jongky_guide"), "web")


@dataclass
class GuideState:
    """UI 가 폴링해 가는 현재 상태."""

    status: str = "idle"          # idle | navigating | waiting | arrived | failed | alert
    destination: str = ""
    message: str = "목적지를 선택해 주세요"
    distance: float = 0.0
    follower_m: float = -1.0      # 뒤따라오는 사람까지 거리. -1 = 모름
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "status": self.status,
                "destination": self.destination,
                "message": self.message,
                "distance": round(self.distance, 2),
                "follower_m": round(self.follower_m, 1),
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
    ):
        super().__init__("jongky_guide")
        self._speaker = speaker
        self._state = state
        self._listener = listener
        self._brain = brain
        self._follower = follower
        self._nav = BasicNavigator()
        self._waypoints = self._load_waypoints(waypoint_path)
        self._task: threading.Thread | None = None

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
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            self.get_logger().warn(f"waypoint 파일이 없다: {path} — 맵핑 주행부터 할 것")
            return {}
        with open(path) as f:
            return yaml.safe_load(f) or {}

    @property
    def destinations(self) -> list[str]:
        return list(self._waypoints)

    def _to_pose(self, name: str) -> PoseStamped:
        w = self._waypoints[name]
        p = PoseStamped()
        p.header.frame_id = w.get("frame_id", "map")
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(w["position"]["x"])
        p.pose.position.y = float(w["position"]["y"])
        o = w["orientation"]
        p.pose.orientation.x = float(o["x"])
        p.pose.orientation.y = float(o["y"])
        p.pose.orientation.z = float(o["z"])
        p.pose.orientation.w = float(o["w"])
        return p

    # ── 안내 ──────────────────────────────────────────────────────────────
    def start_guiding(self, name: str) -> tuple[bool, str]:
        if name not in self._waypoints:
            return False, f"'{name}' 은 등록되지 않은 목적지다"
        if self._task and self._task.is_alive():
            return False, "이미 안내 중이다"

        self._task = threading.Thread(target=self._guide_loop, args=(name,), daemon=True)
        self._task.start()
        return True, ""

    def cancel(self) -> None:
        self._nav.cancelTask()
        self._state.set(status="idle", destination="", message="안내를 취소했습니다", distance=0.0)
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
            rclpy.spin_once(self, timeout_sec=0.2)
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

        self._state.set(status="navigating", message=f"{name} 으로 안내합니다")
        self._publish_status()
        self._nav.goToPose(self._to_pose(name))
        return True

    def _guide_loop(self, name: str) -> None:
        self._state.set(status="navigating", destination=name, message=f"{name} 으로 안내합니다")
        self._publish_status()
        self._speaker.say(f"{name} 으로 안내하겠습니다. 뒤따라와 주세요")

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
                        break
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
                        return
                    best, moved_at, seen_at = None, time.monotonic(), time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self._nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self._state.set(status="arrived", message=f"{name} 에 도착했습니다", distance=0.0)
            self._speaker.say(f"{name} 에 도착했습니다. 안내를 마치겠습니다")
        elif result == TaskResult.CANCELED:
            self._state.set(status="idle", message="안내를 취소했습니다", distance=0.0)
        else:
            self._state.set(status="failed", message="경로를 찾지 못했습니다", distance=0.0)
            self._speaker.say("경로를 찾지 못했습니다. 잠시 후 다시 시도해 주세요")
        self._publish_status()

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
        names = list(self._waypoints)

        if text:
            flat = text.replace(" ", "")
            for name in names:
                if name.replace(" ", "") in flat:
                    self.get_logger().info(f"[매칭] {name}")
                    return name

        if text and self._brain:
            dest = self._brain.resolve_destination(text, names)
            if dest:
                self.get_logger().info(f"[LLM] {dest}")
                return dest
        return None


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
                self._json({"destinations": node.destinations})
            elif self.path == "/api/status":
                s = state.snapshot()
                s["can_listen"] = bool(node._listener and node._listener.ready)
                self._json(s)
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or b"{}")

            if self.path == "/api/go":
                ok, err = node.start_guiding(payload.get("destination", ""))
                self._json({"ok": ok, "error": err}, 200 if ok else 400)
            elif self.path == "/api/cancel":
                node.cancel()
                self._json({"ok": True})
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
    parser.add_argument("--follow-url", default="",
                        help="후면 사람 탐지 서비스 (예: http://localhost:8641/follower). "
                             "주면 뒤처진 사람을 기다린다")
    args, ros_args = parser.parse_known_args()

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
    node = GuideNode(args.waypoints, speaker, state, listener, brain, follower)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(node, state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"UI: http://localhost:{args.port}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

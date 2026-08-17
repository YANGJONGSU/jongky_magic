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

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import String

from speech import Speaker

WEB_DIR = os.path.join(get_package_share_directory("jongky_guide"), "web")


@dataclass
class GuideState:
    """UI 가 폴링해 가는 현재 상태."""

    status: str = "idle"          # idle | navigating | arrived | failed
    destination: str = ""
    message: str = "목적지를 선택해 주세요"
    distance: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "status": self.status,
                "destination": self.destination,
                "message": self.message,
                "distance": round(self.distance, 2),
            }

    def set(self, **kw) -> None:
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


class GuideNode(Node):
    def __init__(self, waypoint_path: str, speaker: Speaker, state: GuideState):
        super().__init__("jongky_guide")
        self._speaker = speaker
        self._state = state
        self._nav = BasicNavigator()
        self._waypoints = self._load_waypoints(waypoint_path)
        self._task: threading.Thread | None = None

        # 외부(예: 음성 노드)에서도 목적지를 넣을 수 있게 열어 둔다
        self.create_subscription(String, "/guide/destination", self._on_destination_topic, 10)
        self._status_pub = self.create_publisher(String, "/guide/status", 10)

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

    def _guide_loop(self, name: str) -> None:
        self._state.set(status="navigating", destination=name, message=f"{name} 으로 안내합니다")
        self._publish_status()
        self._speaker.say(f"{name} 으로 안내하겠습니다. 뒤따라와 주세요")

        self._nav.goToPose(self._to_pose(name))

        while not self._nav.isTaskComplete():
            fb = self._nav.getFeedback()
            if fb:
                self._state.set(distance=float(fb.distance_remaining))
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
                self._json(state.snapshot())
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
            else:
                self._send(404, b"not found", "text/plain")

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--waypoints", default="~/waypoints.yaml", help="teleop_key.py 가 찍어 둔 YAML")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--voice", default="", help="piper 음성 모델 경로 (없으면 무음 폴백)")
    parser.add_argument("--audio-device", default="", help="aplay -D 값 (예: plughw:2,0)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    speaker = Speaker(voice=args.voice, device=args.audio_device)
    state = GuideState()
    node = GuideNode(args.waypoints, speaker, state)

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

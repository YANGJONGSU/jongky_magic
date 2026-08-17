#!/usr/bin/env python3
"""키보드 텔레옵 + waypoint 기록. SLAM 맵핑 주행용.

표준 teleop_twist_keyboard 를 못 쓰는 이유가 두 가지다.

  1. Jazzy 의 diff_drive_controller 는 TwistStamped 를 받는다. 표준 텔레옵은
     Twist 를 쏘므로 **에러 없이 조용히 안 움직인다**
     (jongky_control/README.md 44줄).
  2. 맵핑 주행 중에 강의장 앞 waypoint 를 같이 찍어야 하는데 그 수단이 없다.

실행:
    ros2 run jongky_bringup teleop_key.py
    ros2 run jongky_bringup teleop_key.py --speed 0.15 --out ~/waypoints.yaml

조작:
    i / , : 전진 / 후진          j / l : 좌회전 / 우회전
    u / o : 전진+좌 / 전진+우     스페이스 or k : 즉시 정지
    z / x : 최고 속도 -/+ 10%     c / v : 최고 각속도 -/+ 10%
    w     : 지금 위치를 waypoint 로 저장 (이름을 물어본다)
    p     : 저장된 waypoint 목록
    q     : 종료 (정지 명령을 내고 나간다)

주의: 키를 누르고 있는 동안만 가는 게 아니라, 마지막 명령이 유지된다.
멈추려면 스페이스를 칠 것. cmd_vel_timeout 0.5s 라 노드가 죽으면 자동으로 선다.
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tty

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener

# 실차 한계. jongky_navigation/config/nav2_params.yaml 과 같은 값이어야 한다.
V_MAX = 0.40      # m/s
OMEGA_MAX = 1.50  # rad/s

# 맵핑 주행 기본값. 천천히 돌아야 스캔이 촘촘히 쌓이고 루프도 잘 닫힌다.
DEFAULT_SPEED = 0.15
DEFAULT_TURN = 0.5

MOVES = {
    "i": (1.0, 0.0),
    ",": (-1.0, 0.0),
    "j": (0.0, 1.0),
    "l": (0.0, -1.0),
    "u": (1.0, 1.0),
    "o": (1.0, -1.0),
    "k": (0.0, 0.0),
    " ": (0.0, 0.0),
}


class TeleopKey(Node):
    def __init__(self, speed: float, turn: float, out_path: str, map_frame: str, base_frame: str):
        super().__init__("jongky_teleop_key")
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel", QoSProfile(depth=10))
        self._speed = speed
        self._turn = turn
        self._out_path = os.path.expanduser(out_path)
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._waypoints: dict[str, dict] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if os.path.exists(self._out_path):
            with open(self._out_path) as f:
                self._waypoints = yaml.safe_load(f) or {}
            print(f"기존 waypoint {len(self._waypoints)}개를 읽었다: {self._out_path}")

    def publish(self, vx: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self._pub.publish(msg)

    def save_waypoint(self, name: str) -> bool:
        """현재 map->base_footprint 를 waypoint 로 남긴다."""
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception as exc:  # TF 가 아직 없거나 SLAM 이 안 떠 있는 경우
            print(f"\r  TF 를 못 읽었다 ({self._map_frame} -> {self._base_frame}): {exc}")
            print("\r  SLAM 이 떠 있고 /map 이 나오는지 확인할 것")
            return False

        t, r = tf.transform.translation, tf.transform.rotation
        self._waypoints[name] = {
            "position": {"x": round(t.x, 4), "y": round(t.y, 4), "z": 0.0},
            "orientation": {"x": round(r.x, 6), "y": round(r.y, 6), "z": round(r.z, 6), "w": round(r.w, 6)},
            "frame_id": self._map_frame,
        }
        os.makedirs(os.path.dirname(self._out_path) or ".", exist_ok=True)
        with open(self._out_path, "w") as f:
            yaml.safe_dump(self._waypoints, f, allow_unicode=True, sort_keys=False)
        print(f"\r  '{name}' 저장: x={t.x:.3f} y={t.y:.3f}  -> {self._out_path}")
        return True


def read_key(timeout: float = 0.1) -> str:
    """비차단 1글자 읽기. 아무 키도 없으면 빈 문자열."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return ""


def prompt(settings, message: str) -> str:
    """waypoint 이름처럼 한 줄을 받을 때만 잠깐 정상 터미널로 돌아간다."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    try:
        return input(message).strip()
    finally:
        tty.setcbreak(sys.stdin.fileno())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help=f"기본 선속도 [m/s] (기본 {DEFAULT_SPEED})")
    parser.add_argument("--turn", type=float, default=DEFAULT_TURN, help=f"기본 각속도 [rad/s] (기본 {DEFAULT_TURN})")
    parser.add_argument("--out", type=str, default="~/waypoints.yaml", help="waypoint 저장 경로")
    parser.add_argument("--map-frame", type=str, default="map")
    parser.add_argument("--base-frame", type=str, default="base_footprint")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TeleopKey(
        min(args.speed, V_MAX), min(args.turn, OMEGA_MAX), args.out, args.map_frame, args.base_frame
    )

    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = wz = 0.0

    print(__doc__)
    print(f"속도 {node._speed:.2f} m/s · 회전 {node._turn:.2f} rad/s  (한계 {V_MAX} / {OMEGA_MAX})")
    print("-" * 60)

    try:
        while rclpy.ok():
            key = read_key()

            if key == "q":
                break
            elif key in MOVES:
                mx, mz = MOVES[key]
                vx, wz = mx * node._speed, mz * node._turn
            elif key in ("z", "x"):
                node._speed = max(0.02, min(V_MAX, node._speed * (0.9 if key == "z" else 1.1)))
                print(f"\r  속도 {node._speed:.3f} m/s")
            elif key in ("c", "v"):
                node._turn = max(0.05, min(OMEGA_MAX, node._turn * (0.9 if key == "c" else 1.1)))
                print(f"\r  회전 {node._turn:.3f} rad/s")
            elif key == "w":
                vx = wz = 0.0          # 찍는 동안은 세운다
                node.publish(0.0, 0.0)
                name = prompt(settings, "\r  waypoint 이름: ")
                if name:
                    node.save_waypoint(name)
                else:
                    print("\r  이름이 비어서 취소했다")
            elif key == "p":
                if node._waypoints:
                    print(f"\r  waypoint {len(node._waypoints)}개:")
                    for n, w in node._waypoints.items():
                        print(f"\r    {n:20} x={w['position']['x']:+.3f} y={w['position']['y']:+.3f}")
                else:
                    print("\r  아직 없다")

            node.publish(vx, wz)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)       # 나가기 전에 반드시 세운다
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n정지 명령을 내고 종료했다.")


if __name__ == "__main__":
    main()

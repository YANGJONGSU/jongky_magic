#!/usr/bin/env python3
"""공중 엔코더 검증 — 엔코더가 실제 회전을 정확히 세는가.

[무엇을 가리는가]
"직진 명령인데 로봇이 휜다" 의 원인이 ①좌우 타이어 반지름 차이인지
②좌우 엔코더 스케일 차이인지 가른다. 둘은 odom 상에서 **같은 서명**을
내므로 주행 기록만으로는 구분되지 않는다.

  타이어 원인 → 엔코더는 정확. 지면 이동거리만 다름
  엔코더 원인 → 엔코더가 실제 회전과 다르게 셈

엔코더가 정확한지는 **외부 기준**으로만 확인된다. 그래서 사람이 본다.

[준비]
  1. 로봇을 들어 바퀴를 공중에 띄운다. 바퀴가 자유롭게 돌아야 한다
  2. 양쪽 타이어에 테이프를 붙이고 **12시 방향**에 맞춘다
  3. 이 스크립트를 돌린다

[판정]
스크립트는 엔코더가 정확히 N 바퀴를 셀 때까지 돌리고 멈춘다.
엔코더가 정확하면 테이프가 다시 12시로 온다.

  테이프가 12시            → 그 바퀴 엔코더는 정확
  θ 도 어긋남              → 스케일 오차 = θ / (360 x N)

N=30 에서 0.44% 오차는 **약 48도**(1시 반 방향)로 나타난다. 눈에 보인다.

주의: 바퀴가 돈다. 로봇이 반드시 공중에 있어야 한다.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEFT, RIGHT = "l_wheel_joint", "r_wheel_joint"


class AirTest(Node):
    def __init__(self, vx: float):
        super().__init__("air_test")
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.pos: dict[str, float] = {}
        self.seen = 0
        self._vx = vx

    def _js(self, m: JointState) -> None:
        for n, p in zip(m.name, m.position):
            if n in (LEFT, RIGHT):
                self.pos[n] = p
        if LEFT in self.pos and RIGHT in self.pos:
            self.seen += 1

    def drive(self, vx: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        self._pub.publish(msg)


def spin_for(node, sec: float) -> None:
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--revs", type=float, default=30.0, help="엔코더 기준 회전수")
    ap.add_argument("--vx", type=float, default=0.20, help="바퀴를 돌릴 속도 명령")
    ap.add_argument("--creep", type=float, default=0.03, help="마지막 한 바퀴 속도")
    ap.add_argument("--timeout", type=float, default=240.0)
    a = ap.parse_args()

    rclpy.init()
    n = AirTest(a.vx)

    print("joint_states 대기...")
    t0 = time.time()
    while n.seen < 5 and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.seen < 5:
        print("실패: /joint_states 가 안 온다. 컨트롤러 스택이 떠 있는지 확인할 것")
        sys.exit(1)

    start = dict(n.pos)
    print(f"시작 위치  L {start[LEFT]:+.4f} rad   R {start[RIGHT]:+.4f} rad")
    print(f"\n엔코더 기준 {a.revs:.0f} 바퀴까지 돌린다. 바퀴가 공중에 있는지 확인할 것.")
    for i in (3, 2, 1):
        print(f"  {i}...")
        time.sleep(1)

    target = a.revs * 2 * math.pi

    def revs():
        dl = abs(n.pos[LEFT] - start[LEFT])
        dr = abs(n.pos[RIGHT] - start[RIGHT])
        return dl, dr, (dl + dr) / 2

    # 1단계: 목표 1바퀴 전까지 빠르게.
    fast_until = target - 2 * math.pi
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < a.timeout:
        n.drive(a.vx)
        rclpy.spin_once(n, timeout_sec=0.02)
        dl, dr, avg = revs()
        if avg >= fast_until:
            break
        if time.time() - last > 2.0:
            last = time.time()
            print(f"  L {dl/2/math.pi:6.2f}바퀴   R {dr/2/math.pi:6.2f}바퀴")

    # 2단계: 마지막 1바퀴는 기어가듯. 오버슈트를 줄여 N.000 에 가깝게 세운다.
    print("  마지막 한 바퀴는 천천히...")
    while time.time() - t0 < a.timeout:
        n.drive(a.creep)
        rclpy.spin_once(n, timeout_sec=0.01)
        _, _, avg = revs()
        if avg >= target:
            break

    # 정지 후 관성이 멎기를 기다린다. 바퀴가 0.05kg 이라 금방 선다.
    for _ in range(40):
        n.drive(0.0)
        rclpy.spin_once(n, timeout_sec=0.02)
    spin_for(n, 2.0)

    dl = (n.pos[LEFT] - start[LEFT]) / (2 * math.pi)
    dr = (n.pos[RIGHT] - start[RIGHT]) / (2 * math.pi)
    print("\n" + "=" * 60)
    print("엔코더가 센 회전수")
    print(f"  왼쪽   {dl:+9.4f} 바퀴")
    print(f"  오른쪽 {dr:+9.4f} 바퀴")
    # 목표에서 얼마나 지나쳤는지 = 바늘이 기준선에서 벗어나 있어야 할 각도
    ol = (abs(dl) - a.revs) * 360.0
    orr = (abs(dr) - a.revs) * 360.0
    print(f"\n엔코더 기준 '바늘이 있어야 할 자리' (기준선에서 시계방향)")
    print(f"  왼쪽   {ol:+7.1f} 도")
    print(f"  오른쪽 {orr:+7.1f} 도")
    print("=" * 60)
    print("\n이제 실제 바늘을 읽는다. 기준선에서 시계방향으로 몇 도인가.")
    print("  (12시=0, 3시=90, 6시=180, 9시=270)\n")
    print("  실제 각도가 위 값과 같으면        → 그 바퀴 엔코더는 정확")
    print("  다르면 그 차이가 엔코더 오차다")
    print(f"  차이 1도 = 스케일 오차 {1/(360*a.revs)*100:.4f}%\n")
    print("판정")
    print("  둘 다 일치      → 엔코더 정상. 원인은 타이어 반지름")
    print("  한쪽만 어긋남   → 그쪽 엔코더. counts_per_rev 를 좌우 분리")
    print("  둘 다 어긋남    → 공통 스케일. counts_per_rev 자체를 다시")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""라이다 0도가 로봇의 어느 쪽을 향하는지 알아낸다.

쓰는 법
  1. 로봇 정면 30~50cm 에 상자나 벽을 놓는다.
     그보다 가까운 물체가 주변에 없어야 한다 (책상 다리, 사람 발 등).
  2. 라이다 드라이버를 띄운다.
  3. ros2 run jongky_description check_laser_yaw.py

TF도 RViz도 필요 없다. /scan 원본 각도만 본다.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

N = 10  # 평균 낼 스캔 수


class YawCheck(Node):
    def __init__(self):
        super().__init__('check_laser_yaw')
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.samples = []
        print('로봇 정면 30~50cm 에 물체를 놓으세요.')
        print('/scan 을 기다리는 중...\n')

    def cb(self, msg):
        best_r = best_a = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if best_r is None or r < best_r:
                best_r = r
                best_a = msg.angle_min + i * msg.angle_increment
        if best_r is None:
            return
        deg = math.degrees(best_a) % 360.0
        self.samples.append(deg)
        print('  가장 가까운 점  %7.0f mm   각도 %6.1f도' % (best_r * 1000, deg))
        if len(self.samples) >= N:
            self.report()
            raise SystemExit

    def report(self):
        s = sum(math.sin(math.radians(d)) for d in self.samples)
        c = sum(math.cos(math.radians(d)) for d in self.samples)
        avg = math.degrees(math.atan2(s, c)) % 360.0
        need = (-avg) % 360.0
        print('\n평균 각도 %.1f도' % avg)
        print('─' * 52)
        if avg < 30 or avg > 330:
            print('라이다 0도가 로봇 정면을 향한다.')
            print('  ->  laser_yaw_deg:=0')
        elif 150 < avg < 210:
            print('라이다 0도가 로봇 뒤를 향한다.')
            print('  ->  laser_yaw_deg:=180   (현재 기본값이 맞다)')
        else:
            print('정면도 후면도 아니다. 둘 중 하나다:')
            print('  - 물체가 로봇 정면에 정확히 놓이지 않았다 -> 다시 놓고 재실행')
            print('  - 라이다가 옆으로 돌아 달렸다')
            print('  ->  laser_yaw_deg:=%.0f 을 시도해 볼 것' % need)
        print('─' * 52)


def main():
    rclpy.init()
    try:
        rclpy.spin(YawCheck())
    except (SystemExit, KeyboardInterrupt):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

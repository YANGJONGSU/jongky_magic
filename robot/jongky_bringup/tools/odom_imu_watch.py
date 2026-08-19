#!/usr/bin/env python3
"""바퀴와 자이로가 어긋나는 순간을 잡아낸다.

10층 1차 촬영에서 지도가 틀어진 원인이 이것이었다. 545초 지점에서 바퀴는
2.8도만 돌았다고 하는데 자이로는 32도 돌았다고 했다 — 바퀴가 헛돌았거나
누가 로봇을 건드려 돌린 것이다. 700~740초에는 같은 일이 크게 났다
(바퀴 0도 / 자이로 최대 209도).

EKF 는 회전을 자이로에 맡기므로 그 순간 자세 추정이 통째로 돌아간다. 그
뒤에 그린 지도가 앞부분에 대해 틀어지고, 루프가 닫히지 않으면 영영 남는다.
11층에는 이런 사건이 없었고 지도가 잘 나왔다.

**증상이 원인을 안 알려주는 종류다.** 에러는 한 줄도 안 나고, 현장에서는
지도가 멀쩡해 보인다. 몇 시간 뒤 재처리해야 드러난다. 그래서 그 순간에
소리쳐 주는 노드가 필요하다.

    ros2 run jongky_bringup odom_imu_watch.py
    ros2 run jongky_bringup odom_imu_watch.py --threshold-deg 10 --window 5
"""
import argparse
import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def integrate(buf, t_from):
    """(시각, 각속도) 큐를 t_from 이후 구간에서 적분한다 [rad]."""
    total, prev = 0.0, None
    for t, w in buf:
        if prev is not None and t > t_from:
            dt = t - prev[0]
            if 0.0 < dt < 0.5:
                total += 0.5 * (w + prev[1]) * dt
        prev = (t, w)
    return total


class OdomImuWatch(Node):
    def __init__(self, args):
        super().__init__('odom_imu_watch')
        self.window = args.window
        self.thresh = math.radians(args.threshold_deg)
        self.skip = args.skip_start
        self.odom = deque(maxlen=2000)
        self.imu = deque(maxlen=2000)
        self.t0 = None
        self.hits = 0
        self.worst = 0.0
        self.quiet_until = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, qos)
        self.alert = self.create_publisher(String, '/mapping_alert', 10)
        self.create_timer(1.0, self.check)

        self.get_logger().info(
            '감시 시작 — %.0f초 창에서 바퀴와 자이로가 %.0f도 넘게 어긋나면 알린다'
            % (self.window, args.threshold_deg))

    def _t(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_odom(self, m):
        t = self._t(m)
        if self.t0 is None:
            self.t0 = t
        self.odom.append((t, m.twist.twist.angular.z))

    def on_imu(self, m):
        self.imu.append((self._t(m), m.angular_velocity.z))

    def check(self):
        if not self.odom or not self.imu or self.t0 is None:
            return
        now = self.odom[-1][0]
        if now - self.t0 < self.skip:      # 기동 직후는 건너뛴다
            return
        if now < self.quiet_until:
            return
        t_from = now - self.window
        o = integrate(self.odom, t_from)
        g = integrate(self.imu, t_from)
        d = o - g
        if abs(d) < self.thresh:
            return

        self.hits += 1
        self.worst = max(self.worst, abs(d))
        msg = ('바퀴 %+.1f도 / 자이로 %+.1f도 · 차이 %+.1f도  (주행 %.0f초 지점)'
               % (math.degrees(o), math.degrees(g), math.degrees(d), now - self.t0))
        self.get_logger().error('로봇이 밀렸거나 바퀴가 헛돌았다 — ' + msg)
        self.alert.publish(String(data=msg))
        self.quiet_until = now + self.window   # 같은 사건을 한 번만 알린다

    def summary(self):
        if self.hits:
            self.get_logger().error(
                '총 %d회 어긋남, 최대 %.1f도. 그 구간은 지도가 틀어졌을 수 있다 — '
                '다시 지나가서 루프를 닫아 두는 편이 안전하다'
                % (self.hits, math.degrees(self.worst)))
        else:
            self.get_logger().info('어긋남 없음 — 바퀴와 자이로가 끝까지 일치했다')


def main():
    p = argparse.ArgumentParser(description='바퀴와 자이로의 회전 불일치 감시')
    p.add_argument('--window', type=float, default=5.0, help='비교 창 [s]')
    p.add_argument('--threshold-deg', type=float, default=15.0,
                   help='알릴 기준 [도]. 10층 사고는 29도, 11층 정상은 최대 19도(기동 중)')
    p.add_argument('--skip-start', type=float, default=10.0,
                   help='기동 직후 건너뛸 시간 [s]')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--imu-topic', default='/imu/data')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    n = OdomImuWatch(args)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

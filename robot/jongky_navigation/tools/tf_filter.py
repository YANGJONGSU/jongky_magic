#!/usr/bin/env python3
"""bag 재생용 /tf 필터 — 녹화된 map->odom 을 걸러낸다.

현장 촬영 bag 은 slam_toolbox 가 돌고 있는 상태에서 기록됐다. 그래서 /tf 안에
그때의 map->odom 이 47Hz 로 통째로 들어 있다 (10f_0818_2246 기준 8,533개).

이 bag 을 그대로 재생하면서 새 slam_toolbox 를 띄우면 같은 map->odom 을
둘이 동시에 쏜다. TF 버퍼는 마지막에 들어온 걸 쓰므로 두 위치 추정이 번갈아
이기고, 에러는 한 줄도 안 난다. 증상은 "복도가 여러 겹으로 어긋나 그려짐" 이다.

그렇다고 재생에서 /tf 를 통째로 빼면 안 된다. odom->base_footprint 가 같은
토픽에 있어서 그것까지 사라지면 SLAM 이 로봇 위치를 잃는다. map->odom 만
걸러내야 한다.

사용법:
  # 1) 이 노드를 먼저 띄운다
  ros2 run jongky_navigation tf_filter.py --ros-args -p use_sim_time:=true

  # 2) bag 의 /tf 를 /tf_bag 으로 돌려서 재생한다
  ros2 bag play <bag> --clock \
      --topics /scan /odom /imu/data /tf /tf_static \
      --remap /tf:=/tf_bag

여러 쌍을 막으려면:
  ros2 run jongky_navigation tf_filter.py --block map:odom --block map:base_footprint
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


def _norm(frame):
    """tf2 는 앞의 '/' 를 쓰지 않는다. 비교 전에 떼어낸다.

    이 bag 의 /tf_static 에는 openni2_camera 가 낸 구식 표기(`/camera_link`)와
    URDF 가 낸 표기(`camera_link`) 가 섞여 있다. 비교만이라도 통일해 둔다.
    """
    return frame.lstrip('/')


class TfFilter(Node):
    def __init__(self, in_topic, out_topic, blocked, report_period):
        super().__init__('tf_filter')

        self.blocked = blocked
        self.in_topic = in_topic
        self.passed = 0
        self.dropped = 0
        self.seen = {}

        # tf2 의 /tf 는 reliable, 큐가 깊다. 재생이 빠를 때 흘리지 않게 맞춘다.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.pub = self.create_publisher(TFMessage, out_topic, qos)
        self.sub = self.create_subscription(TFMessage, in_topic, self.on_tf, qos)

        self.get_logger().info(
            '%s -> %s, 차단: %s'
            % (in_topic, out_topic, ', '.join('%s->%s' % p for p in sorted(blocked)))
        )

        # 조용히 아무것도 안 거르는 상태를 못 알아채는 게 제일 위험하다.
        # 주기적으로 통과/차단 수를 찍어서 눈으로 확인할 수 있게 한다.
        #
        # 타이머는 반드시 벽시계로 돌린다. use_sim_time 이면 노드 기본 시계는
        # /clock 을 기다리는데, bag 이 20GB mcap 을 여는 동안에는 /clock 이
        # 안 온다. 그 시계에 타이머를 걸면 재생이 시작될 때까지 로그가 한 줄도
        # 안 나와서, 필터가 죽은 것과 멀쩡히 대기 중인 것이 구분되지 않는다.
        if report_period > 0:
            self.create_timer(
                report_period, self.report, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_tf(self, msg):
        kept = []
        for tr in msg.transforms:
            pair = (_norm(tr.header.frame_id), _norm(tr.child_frame_id))
            self.seen[pair] = self.seen.get(pair, 0) + 1
            if pair in self.blocked:
                self.dropped += 1
            else:
                kept.append(tr)
                self.passed += 1

        # 전부 걸러졌으면 빈 TFMessage 를 굳이 내보내지 않는다.
        if kept:
            self.pub.publish(TFMessage(transforms=kept))

    def report(self):
        pairs = ', '.join(
            '%s->%s:%d' % (p[0], p[1], n)
            for p, n in sorted(self.seen.items(), key=lambda x: -x[1])
        )
        self.get_logger().info(
            '통과 %d / 차단 %d | %s' % (self.passed, self.dropped, pairs or '수신 없음')
        )
        if not self.seen:
            self.get_logger().warn(
                '%s 로 들어온 게 아직 없다 — bag 재생이 시작되지 않았거나 '
                'remap(/tf:=%s) 이 빠졌을 수 있다' % (self.in_topic, self.in_topic))
        if self.dropped == 0 and self.passed > 0:
            self.get_logger().warn(
                '차단된 게 하나도 없다 — --block 쌍이 bag 의 프레임 이름과 '
                '다르거나, 이미 필터된 bag 일 수 있다'
            )


def main():
    parser = argparse.ArgumentParser(
        description='bag 재생 시 녹화된 map->odom 을 /tf 에서 걸러낸다')
    parser.add_argument(
        '--in-topic', default='/tf_bag',
        help='bag 의 /tf 를 remap 해 받을 토픽 (기본값: /tf_bag)')
    parser.add_argument(
        '--out-topic', default='/tf',
        help='걸러낸 결과를 낼 토픽 (기본값: /tf)')
    parser.add_argument(
        '--block', action='append', default=None, metavar='PARENT:CHILD',
        help='차단할 프레임 쌍. 여러 번 줄 수 있다 (기본값: map:odom)')
    parser.add_argument(
        '--report-period', type=float, default=10.0,
        help='통과/차단 수를 찍는 주기 [s]. 0 이면 끔 (기본값: 10)')

    args, ros_args = parser.parse_known_args()

    blocked = set()
    for spec in (args.block or ['map:odom']):
        if ':' not in spec:
            print("[-] --block 은 PARENT:CHILD 형식이어야 한다: %r" % spec, file=sys.stderr)
            sys.exit(2)
        parent, child = spec.split(':', 1)
        blocked.add((_norm(parent), _norm(child)))

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = TfFilter(args.in_topic, args.out_topic, blocked, args.report_period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

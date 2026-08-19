#!/usr/bin/env python3
"""bag 재생용 IMU 보정 릴레이 — 자이로 스케일과 공분산을 채워 다시 낸다.

bag 은 보정 전 자이로로 녹화됐다. 소스를 고쳐도 이미 찍힌 bag 은 안 바뀌므로,
재생 경로에서는 이 노드가 대신 고친다.

고치는 것:

1. 공분산 — 보드가 안 준다. 0 은 "오차가 전혀 없다"는 뜻이라 필터가 IMU 만
   100% 신뢰하거나 행렬이 특이해져 발산한다. 기본값은 10층 bag 정지 구간
   실측(분산 2.73e-05)에 3배 여유를 준 값이다.

자세(orientation)는 공분산 [0] = -1 로 표시해 내보낸다. 보드가 자기 자이로로
적분한 값이라 같은 스케일 오차를 물고 있어서 EKF 에 넣으면 안 된다.

사용법:
  ros2 run jongky_navigation imu_relay.py --ros-args -p use_sim_time:=true
  # bag 재생에 /imu/data 를 포함시키면 /imu/data_corrected 로 나온다
"""
import argparse
import math
import sys

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

# 스케일 보정은 기본적으로 하지 않는다 (1.0).
#
# 계획서에는 보정계수 1.047 이 적혀 있다 — 회전 시험에서 실제 1845도일 때
# 자이로가 1762도를 보고했다는 근거다. 그런데 10층 촬영 bag 12분 42초에서
# 회전 이벤트 40개를 하나씩 통째로 적분해 오도메트리와 비교하니 이렇게 나왔다:
#
#   합산      odom 2858.4도 · gyro 2830.7도 · 비 1.0098
#   구간별 비  중앙 1.0109 · 평균 1.0097 · 표준오차 0.0058
#   95% 구간  0.9983 ~ 1.0211   → 1.047 은 밖, 1.000 은 안
#
# 비교 기준인 오도메트리 회전 스케일은 wheel_separation 119.09mm(제자리
# 5바퀴 x3회 줄자 실측)에 묶여 있어 자이로와 독립이다. 순환 논리가 아니다.
#
# 즉 자이로는 1% 안쪽으로 맞다. 1.047 을 씌우면 4.7% 오차를 고치는 게 아니라
# 3.7% 오차를 새로 만든다. 스윕에서 시험하려면 --scale 로 넣을 것.
#
# (구간별 적분으로 비교한 이유: 순간 각속도끼리 회귀하면 값이 안 맞는다.
#  diff_drive_controller 가 엔코더 차분으로 각속도를 만들며 지연·평활이
#  들어가 자이로와 위상이 어긋난다 — 상관 0.95, OLS 1.027 / TLS 0.979 로
#  방법마다 다른 답이 나온다. 회전 이벤트를 통째로 적분하면 지연이 상쇄된다.)
DEFAULT_SCALE = 1.0
# 10층 bag 정지 구간 실측 분산에 3배 여유
DEFAULT_VAR = 8.182e-05


class ImuRelay(Node):
    def __init__(self, args):
        super().__init__('imu_relay')
        self.scale = args.scale
        self.bias = args.bias
        self.var = args.variance
        self.n = 0
        self.yaw = 0.0          # 보정된 각속도의 적분 — 검산용
        self.raw_yaw = 0.0      # 원본 적분
        self.last_t = None
        self.peak = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.pub = self.create_publisher(Imu, args.out_topic, qos)
        self.create_subscription(Imu, args.in_topic, self.on_imu, qos)

        self.get_logger().info(
            '%s -> %s · 스케일 %.4f · 편향 %+.6f · 분산 %.3e'
            % (args.in_topic, args.out_topic, self.scale, self.bias, self.var))

        # tf_filter 와 같은 이유로 벽시계에 건다. use_sim_time 이면 노드 기본
        # 시계가 /clock 을 기다리는데, bag 이 뜨기 전에는 안 온다 — 그동안
        # 죽은 노드와 대기 중인 노드가 구분되지 않는다.
        if args.report_period > 0:
            self.create_timer(args.report_period, self.report,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_imu(self, msg):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.linear_acceleration = msg.linear_acceleration

        gz = (msg.angular_velocity.z - self.bias) * self.scale
        out.angular_velocity.x = (msg.angular_velocity.x - 0.0) * self.scale
        out.angular_velocity.y = (msg.angular_velocity.y - 0.0) * self.scale
        out.angular_velocity.z = gz

        # 자세는 쓰지 말라고 표시한다 (-1 은 "측정 없음" 규약)
        out.orientation_covariance = [-1.0] + [0.0] * 8
        out.angular_velocity_covariance = [
            self.var, 0.0, 0.0, 0.0, self.var, 0.0, 0.0, 0.0, self.var]
        out.linear_acceleration_covariance = [
            2.388e-02, 0.0, 0.0, 0.0, 3.332e-02, 0.0, 0.0, 0.0, 1.605e-02]

        # 검산용 적분 — 한 바퀴가 360도로 나오는지 보려는 것
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.5:
                self.yaw += gz * dt
                self.raw_yaw += msg.angular_velocity.z * dt
        self.last_t = t
        self.peak = max(self.peak, abs(gz))

        self.n += 1
        self.pub.publish(out)

    def report(self):
        if self.n == 0:
            self.get_logger().warn(
                '들어온 IMU 가 없다 — bag 재생이 시작되지 않았거나 '
                '/imu/data 가 재생 토픽에 빠졌을 수 있다')
            return
        self.get_logger().info(
            '%d개 · 적분 yaw 보정 %.1f도 / 원본 %.1f도 (차 %.1f도) · 최대 %.3f rad/s'
            % (self.n, math.degrees(self.yaw), math.degrees(self.raw_yaw),
               math.degrees(self.yaw - self.raw_yaw), self.peak))


def main():
    p = argparse.ArgumentParser(description='재생용 IMU 자이로 스케일·공분산 보정')
    p.add_argument('--in-topic', default='/imu/data')
    p.add_argument('--out-topic', default='/imu/data_corrected')
    p.add_argument('--scale', type=float, default=DEFAULT_SCALE,
                   help='자이로 스케일 보정계수 (기본 %(default)s — 위 주석 참조)')
    p.add_argument('--bias', type=float, default=0.0,
                   help='각속도 z 에서 뺄 편향 [rad/s]. 10층 bag 실측 편향은 '
                        '4e-05 미만이라 기본은 0 이다')
    p.add_argument('--variance', type=float, default=DEFAULT_VAR,
                   help='각속도 공분산 대각 (기본 %(default).3e)')
    p.add_argument('--report-period', type=float, default=30.0)
    args, ros_args = p.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = ImuRelay(args)
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

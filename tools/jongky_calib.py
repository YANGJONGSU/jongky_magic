#!/usr/bin/env python3
"""종키 오도메트리 검산 · 주행 품질 진단.

컨테이너 안에서 컨트롤러 스택이 떠 있는 상태로 실행한다.

  # 직진 — counts_per_rev 검산용. 끝나면 줄자로 실제 거리를 잰다
  python3 jongky_calib.py straight --vx 0.2 --dur 6

  # 실측값을 넣어 보정값 계산 (주행 없음)
  python3 jongky_calib.py fix --odom 826.7 --actual 760 --cpr 1788

  # 제자리 회전 — wheel_separation 검산용
  python3 jongky_calib.py spin --vz 0.8 --turns 5

  # 주행 품질 진단 — 지그재그 원인 가리기
  python3 jongky_calib.py trace --vx 0.2 --dur 8

  # 보드 PID 게인 스윕 — 진동이 최소가 되는 지점 찾기
  #   바퀴를 띄워놓고 하면 공간도 안 쓰고 결과도 같다
  python3 jongky_calib.py pidsweep --vx 0.2

주의: straight · spin · trace 는 로봇을 실제로 움직인다.
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

WHEEL_RADIUS = 0.0335


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    """cmd_vel 을 쏘면서 odom / joint_states / imu 를 고속으로 기록한다."""

    def __init__(self):
        super().__init__('jongky_calib')
        self.pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.odom = None
        self.js = None
        self.imu = None
        self.trace = []          # (t, vl, vr, gz, odom_wz)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._odom, 20)
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.create_subscription(Imu, '/imu_sensor_broadcaster/imu', self._imu, 20)

    def _odom(self, m):
        self.odom = m

    def _js(self, m):
        self.js = m

    def _imu(self, m):
        self.imu = m

    def send(self, vx=0.0, vz=0.0):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = vx
        m.twist.angular.z = vz
        self.pub.publish(m)

    def wait_ready(self, need_imu=False, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom and self.js and (self.imu or not need_imu):
                return True
        return False

    def snapshot(self):
        p = self.odom.pose.pose
        return (p.position.x, p.position.y, yaw_of(p.orientation),
                list(self.js.position))

    def run(self, vx, vz, dur, record=False):
        t0 = time.time()
        try:
            while time.time() - t0 < dur:
                self.send(vx, vz)
                rclpy.spin_once(self, timeout_sec=0.01)
                if record and self.js and len(self.js.velocity) >= 2:
                    self.trace.append((
                        time.time() - t0,
                        self.js.velocity[0], self.js.velocity[1],
                        self.imu.angular_velocity.z if self.imu else float('nan'),
                        self.odom.twist.twist.angular.z if self.odom else float('nan'),
                    ))
                time.sleep(0.01)
        finally:
            for _ in range(12):
                self.send(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.04)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            rclpy.spin_once(self, timeout_sec=0.05)


def dominant_freq(t, y):
    """등간격이 아닌 표본에서 대략적인 지배 주파수를 영교차로 추정한다."""
    if len(y) < 20:
        return None
    m = statistics.fmean(y)
    dev = [v - m for v in y]
    crossings = [i for i in range(1, len(dev)) if dev[i - 1] <= 0 < dev[i]]
    if len(crossings) < 2:
        return None
    span = t[crossings[-1]] - t[crossings[0]]
    if span <= 0:
        return None
    return (len(crossings) - 1) / span


def trace_stats(tr, skip):
    """정상상태 구간의 요약 통계. (좌변동%, 우변동%, 상관, 자이로표준편차)"""
    tr = [r for r in tr if r[0] >= skip]
    if len(tr) < 20:
        return None
    vl = [r[1] for r in tr]
    vr = [r[2] for r in tr]
    gz = [r[3] for r in tr if not math.isnan(r[3])]
    ml, mr = statistics.fmean(vl), statistics.fmean(vr)
    sl, sr = statistics.pstdev(vl), statistics.pstdev(vr)
    dl = [v - ml for v in vl]
    dr = [v - mr for v in vr]
    den = math.sqrt(sum(a * a for a in dl) * sum(b * b for b in dr))
    corr = sum(a * b for a, b in zip(dl, dr)) / den if den > 1e-12 else 0.0
    return dict(
        ml=ml, mr=mr,
        cvl=100 * sl / max(abs(ml), 1e-6), cvr=100 * sr / max(abs(mr), 1e-6),
        corr=corr, gz=statistics.pstdev(gz) if gz else float('nan'))


def report_trace(tr, skip=2.5):
    # 가속 램프 구간을 버린다. 보드가 목표 속도까지 천천히 올리므로
    # 그 구간을 섞으면 상승분이 통째로 표준편차로 잡혀 진단이 무의미해진다.
    full = len(tr)
    tr = [r for r in tr if r[0] >= skip]
    if len(tr) < 20:
        print("정상상태 표본 부족 (전체 %d개 중 %.1f초 이후 %d개). "
              "--dur 를 늘리거나 --skip 을 줄일 것" % (full, skip, len(tr)))
        return
    print("\n(가속 램프 %.1f초 제외: 전체 %d개 -> 정상상태 %d개)" % (skip, full, len(tr)))
    if len(tr) < 20:
        print("표본 부족 — 진단 불가")
        return
    t = [r[0] for r in tr]
    vl = [r[1] for r in tr]
    vr = [r[2] for r in tr]
    gz = [r[3] for r in tr if not math.isnan(r[3])]
    wz = [r[4] for r in tr if not math.isnan(r[4])]

    print("=== 주행 품질 (정상상태) ===")
    print("표본 %d개, %.1f~%.1f초" % (len(tr), t[0], t[-1]))
    for name, v in (("왼쪽 바퀴", vl), ("오른쪽 바퀴", vr)):
        f = dominant_freq(t, v)
        print("  %-10s 평균 %6.3f  표준편차 %6.3f  변동 %5.1f%%  진동 %s"
              % (name, statistics.fmean(v), statistics.pstdev(v),
                 100 * statistics.pstdev(v) / max(abs(statistics.fmean(v)), 1e-6),
                 ("%.1f Hz" % f) if f else "불명"))

    # 좌우 변동의 위상 관계
    ml, mr = statistics.fmean(vl), statistics.fmean(vr)
    dl = [v - ml for v in vl]
    dr = [v - mr for v in vr]
    num = sum(a * b for a, b in zip(dl, dr))
    den = math.sqrt(sum(a * a for a in dl) * sum(b * b for b in dr))
    corr = num / den if den > 1e-12 else 0.0
    print("  좌우 변동 상관계수 %+.2f" % corr)

    if gz and wz:
        n = min(len(gz), len(wz))
        fg = dominant_freq(t[:n], gz[:n])
        fw = dominant_freq(t[:n], wz[:n])
        mg, mw = statistics.fmean(gz), statistics.fmean(wz)
        print("  자이로 z      평균 %+7.4f  표준편차 %6.3f rad/s  진동 %s"
              % (mg, statistics.pstdev(gz), ("%.1f Hz" % fg) if fg else "불명"))
        print("  odom 각속도   평균 %+7.4f  표준편차 %6.3f rad/s  진동 %s"
              % (mw, statistics.pstdev(wz), ("%.1f Hz" % fw) if fw else "불명"))
        ratio = statistics.pstdev(gz) / max(statistics.pstdev(wz), 1e-6)
        print("  자이로/odom 변동비 %.2f" % ratio)

        # 평균 각속도의 차이가 곧 "오도메트리가 못 보는 회전" 이다.
        # 좌우 바퀴 반지름이 다르면 엔코더는 각도만 재므로 이 차이가 생긴다.
        gap = mg - mw
        print("\n  --- 편향 ---")
        print("  자이로가 본 회전   %+.4f rad/s  (%+.2f 도/초)"
              % (mg, math.degrees(mg)))
        print("  odom 이 본 회전    %+.4f rad/s  (%+.2f 도/초)"
              % (mw, math.degrees(mw)))
        print("  odom 이 놓친 회전  %+.4f rad/s  (%+.2f 도/초)"
              % (gap, math.degrees(gap)))
        if abs(mg) > 0.01:
            side = "오른쪽" if mg < 0 else "왼쪽"
            print("  -> 차체가 %s으로 %.2f 도/초 꾸준히 휜다" % (side, abs(math.degrees(mg))))
            if abs(gap) > 0.6 * abs(mg):
                print("     그중 대부분을 odom 이 못 본다 -> 좌우 바퀴 반지름 차이")
                print("     대응: diff_drive_controller 의 "
                      "left/right_wheel_radius_multiplier")
            else:
                print("     odom 도 같이 본다 -> 좌우 바퀴 속도 자체가 비대칭")
                print("     대응: 모터/PID 쪽")

    print("\n=== 판정 ===")
    if corr < -0.3:
        print("  좌우가 반대 위상으로 진동한다 -> 보드 PID 진동이 유력")
        print("  대응: FUNC_SET_MOTOR_PID 로 게인을 낮춘다")
    elif gz and wz and statistics.pstdev(gz) > 2.5 * statistics.pstdev(wz):
        print("  바퀴는 매끄러운데 차체만 흔들린다 -> 캐스터 시미가 유력")
        print("  대응: 캐스터 조임/교체, 또는 속도 대역 회피")
    elif corr > 0.5:
        print("  좌우가 같은 위상으로 변동한다 -> 노면이나 명령 자체의 변동")
    else:
        print("  뚜렷한 패턴 없음. 변동 폭이 작으면 정상 범위다")


def cmd_straight(n, a):
    if not n.wait_ready():
        print("odom/joint_states 수신 실패 — 컨트롤러가 떠 있는지 확인")
        return
    x0, y0, th0, j0 = n.snapshot()
    print(">>> 전진 %.2f m/s, %.1f초" % (a.vx, a.dur))
    n.run(a.vx, 0.0, a.dur, record=True)
    x1, y1, th1, j1 = n.snapshot()

    dist = math.hypot(x1 - x0, y1 - y0)
    dj = [j1[i] - j0[i] for i in range(len(j0))]
    print("\n=== 직진 결과 ===")
    print("odom 이동거리 %.1f mm   (x %+.1f, y %+.1f)"
          % (dist * 1000, (x1 - x0) * 1000, (y1 - y0) * 1000))
    print("yaw 변화      %+.1f 도" % math.degrees(th1 - th0))
    print("관절 변화     좌 %+.3f  우 %+.3f rad  (좌가 %+.1f%%)"
          % (dj[0], dj[1], (dj[0] / dj[1] - 1) * 100 if abs(dj[1]) > 1e-9 else 0))
    print("바퀴 기하 이동 %.1f mm" % ((dj[0] + dj[1]) / 2 * WHEEL_RADIUS * 1000))
    print("\n줄자로 실제 거리를 잰 뒤:")
    print("  python3 jongky_calib.py fix --odom %.1f --actual <실측mm> --cpr <현재값>"
          % (dist * 1000))
    report_trace(n.trace, a.skip)


def cmd_spin(n, a):
    if not n.wait_ready():
        print("odom/joint_states 수신 실패")
        return
    _, _, th0, j0 = n.snapshot()
    total = 0.0
    prev = th0
    target = a.turns * 2 * math.pi
    print(">>> 제자리 회전 %.2f rad/s, 목표 %.1f 바퀴" % (a.vz, a.turns))
    t0 = time.time()
    try:
        while abs(total) < target and time.time() - t0 < a.turns * 20:
            n.send(0.0, a.vz)
            rclpy.spin_once(n, timeout_sec=0.01)
            cur = yaw_of(n.odom.pose.pose.orientation)
            d = cur - prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            total += d
            prev = cur
            time.sleep(0.01)
    finally:
        for _ in range(12):
            n.send(0.0, 0.0)
            rclpy.spin_once(n, timeout_sec=0.01)
            time.sleep(0.04)
    print("\n=== 회전 결과 ===")
    print("odom 누적 회전 %.1f 도 (%.3f 바퀴)"
          % (math.degrees(total), total / (2 * math.pi)))
    print("\n실제 회전한 바퀴 수를 세어서:")
    print("  새 wheel_separation = 현재값 × (odom바퀴수 / 실제바퀴수)")
    print("  odom 이 더 많이 돌았다고 하면 트레드를 늘린다")


def cmd_trace(n, a):
    if not n.wait_ready(need_imu=True, timeout=8.0):
        print("odom/joint_states/imu 수신 실패 — imu_sensor_broadcaster 확인")
        return
    print(">>> 주행 %.2f m/s, %.1f초 기록" % (a.vx, a.dur))
    n.run(a.vx, 0.0, a.dur, record=True)
    report_trace(n.trace, a.skip)


def cmd_pidsweep(n, a):
    """보드 PID 게인을 바꿔가며 바퀴 속도 진동을 잰다."""
    # 하드웨어 컴포넌트는 자기 이름의 노드를 따로 갖는다.
    # 파라미터는 controller_manager 가 아니라 그쪽에 붙는다.
    srv = '/%s/set_parameters' % a.hw
    cli = n.create_client(SetParameters, srv)
    if not cli.wait_for_service(timeout_sec=5.0):
        print("파라미터 서비스를 못 찾음: %s" % srv)
        print("  ros2 node list 로 하드웨어 노드 이름을 확인하고 --hw 로 지정할 것")
        return

    def set_pid(kp, ki, kd):
        req = SetParameters.Request()
        for name, v in (("motor_pid_kp", kp), ("motor_pid_ki", ki), ("motor_pid_kd", kd)):
            p = Parameter()
            p.name = "%s.%s" % (a.hw, name)
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            req.parameters.append(p)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
        r = fut.result()
        return bool(r and all(x.successful for x in r.results))

    if not n.wait_ready(need_imu=True, timeout=8.0):
        print("토픽 수신 실패")
        return

    # 목표 바퀴 각속도. 이보다 크게 처지면 게인이 너무 낮아 제어가 죽은 것이다.
    target_w = a.vx / WHEEL_RADIUS
    gains = [float(x) for x in a.kp.split(',')]
    print("게인 스윕: kp = %s  (ki=%.2f kd=%.2f 고정)" % (gains, a.ki, a.kd))
    print("각 시행 %.0f초, 앞 %.1f초 제외\n" % (a.dur, a.skip))
    print("   kp     좌변동   우변동   상관    자이로σ    평균속도   목표대비")
    print("  " + "-" * 68)

    rows = []
    for kp in gains:
        if not set_pid(kp, a.ki, a.kd):
            print("  %.2f   PID 설정 실패" % kp)
            continue
        time.sleep(0.5)
        n.trace = []
        n.run(a.vx, 0.0, a.dur, record=True)
        st = trace_stats(n.trace, a.skip)
        if not st:
            print("  %.2f   표본 부족" % kp)
            continue
        reach = (st['ml'] + st['mr']) / 2 / max(target_w, 1e-6)
        st['reach'] = reach
        rows.append((kp, st))
        flag = "" if reach >= 0.85 else "  <- 속도 미달"
        print("  %5.2f   %6.1f%%  %6.1f%%  %+.2f   %7.4f   %5.2f/%5.2f  %3.0f%%%s"
              % (kp, st['cvl'], st['cvr'], st['corr'], st['gz'],
                 st['ml'], st['mr'], reach * 100, flag))
        time.sleep(1.0)

    # 목표 속도의 85% 를 못 내는 시행은 후보에서 뺀다. 바퀴가 멈춰 있으면
    # 진동도 0 이라 그걸 "최적" 으로 고르는 사고가 난다.
    ok = [r for r in rows if r[1]['reach'] >= 0.85]
    if not ok:
        print("\n목표 속도에 도달한 게인이 없다.")
        print("ki 가 0 이면 정상상태 오차를 못 없애 목표에 영영 도달하지 못한다.")
        print("보드 기본 게인을 확인하고 --ki 를 그 값으로 두고 다시 시도할 것:")
        print("  ros2 param get /%s %s.motor_pid_ki" % (a.hw, a.hw))
        return
    if ok:
        best = min(ok, key=lambda r: (r[1]['cvl'] + r[1]['cvr']) / 2)
        print("\n진동이 가장 작은 kp = %.2f  (평균 변동 %.1f%%, 목표속도 %.0f%% 도달)"
              % (best[0], (best[1]['cvl'] + best[1]['cvr']) / 2, best[1]['reach'] * 100))
        print("적용:  ros2 param set /%s %s.motor_pid_kp %.2f" % (a.hw, a.hw, best[0]))
        print("\n주의: 게인을 낮추면 진동은 줄지만 응답이 느려진다.")
        print("      평균속도가 목표보다 크게 처지면 너무 낮춘 것이다.")


def cmd_fix(a):
    k = a.odom / a.actual
    print("odom %.1f mm, 실제 %.1f mm  ->  비율 %.4f" % (a.odom, a.actual, k))
    print("새 counts_per_rev = %.0f × %.4f = %.0f" % (a.cpr, k, a.cpr * k))
    print("\nodom 이 실제보다 %s 보고하고 있었다."
          % ("많이" if k > 1 else "적게"))
    print("jongky_description 의 <param name=\"counts_per_rev\"> 를 고칠 것.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('straight', help='직진 후 odom·관절 보고')
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--dur', type=float, default=6.0)
    p.add_argument('--skip', type=float, default=2.5)

    p = sub.add_parser('spin', help='제자리 회전')
    p.add_argument('--vz', type=float, default=0.8)
    p.add_argument('--turns', type=float, default=5.0)

    p = sub.add_parser('trace', help='주행 품질 진단')
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--dur', type=float, default=8.0)
    p.add_argument('--skip', type=float, default=2.5,
                   help='앞부분 가속 램프를 제외할 초 (기본 2.5)')

    p = sub.add_parser('pidsweep', help='보드 PID 게인 스윕')
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--dur', type=float, default=6.0)
    p.add_argument('--skip', type=float, default=3.0)
    p.add_argument('--kp', type=str, default='0.2,0.4,0.6,0.8,1.0,1.5',
                   help='시험할 kp 값들 (쉼표 구분)')
    p.add_argument('--ki', type=float, required=True,
                   help='보드 기본 ki. 0 으로 두면 목표 속도에 도달하지 못한다. '
                        'ros2 param get /jongky jongky.motor_pid_ki 로 확인할 것')
    p.add_argument('--kd', type=float, default=0.0)
    p.add_argument('--hw', type=str, default='jongky',
                   help='URDF 의 ros2_control 이름')

    p = sub.add_parser('fix', help='실측값으로 counts_per_rev 역산 (주행 없음)')
    p.add_argument('--odom', type=float, required=True)
    p.add_argument('--actual', type=float, required=True)
    p.add_argument('--cpr', type=float, required=True)

    a = ap.parse_args()

    if a.cmd == 'fix':
        cmd_fix(a)
        return

    rclpy.init()
    n = Recorder()
    try:
        {'straight': cmd_straight, 'spin': cmd_spin, 'trace': cmd_trace,
         'pidsweep': cmd_pidsweep}[a.cmd](n, a)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

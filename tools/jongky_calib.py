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

  # 보드 PID 게인 스윕
  #   kp: 진동이 최소가 되는 지점 찾기 (바퀴를 띄워놓고 해도 된다)
  #   ki: 저속에서 약한 모터가 데드밴드를 넘게 하는 값 찾기
  python3 jongky_calib.py pidsweep --sweep kp --values 0.4,0.8,1.2 --ki 0.06 --kd 0.5
  python3 jongky_calib.py pidsweep --sweep ki --values 0.06,0.1,0.2,0.3 --vx 0.1

  # 방향 제어 직진 — 폐루프로 헤딩을 잡으며 간다
  python3 jongky_calib.py hold --vx 0.2 --dur 10

주의: straight · spin · trace · hold 는 로봇을 실제로 움직인다.
"""
import argparse
import math
import statistics
import sys
import time

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
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
    """cmd_vel 을 쏘면서 odom / joint_states / imu 를 고속으로 기록한다.

    [중요] 콜백은 반드시 별도 스레드에서 계속 돌려야 한다.
    토픽 3개가 각 50Hz 면 초당 150개인데, 제어 루프 안에서
    spin_once 를 한 번씩만 부르면 큐가 밀려 낡은 값을 보게 된다.
    그 상태로 되먹임 제어를 하면 발산한다.
    """

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
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._spin = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin.start()

    def shutdown(self):
        self._exec.shutdown()

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
            if self.odom and self.js and (self.imu or not need_imu):
                return True
            time.sleep(0.02)
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
                time.sleep(0.04)
        time.sleep(1.5)


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
        corr=corr,
        gz=statistics.pstdev(gz) if gz else float('nan'),
        gzmean=statistics.fmean(gz) if gz else float('nan'))


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
    """제자리 회전으로 wheel_separation 을 검산한다.

    odom 기준으로 정확히 N 바퀴 돈 뒤 멈춘다. 로봇 정면이 출발 표시로
    돌아왔으면 트레드가 맞은 것이고, 어긋난 만큼이 오차다.

    자이로도 같이 적분한다. 회전량이 크므로(5바퀴=1800도) 바이어스
    영향은 무시할 수준이고, odom 과 독립적인 검증이 된다.
    """
    if not n.wait_ready(need_imu=True, timeout=8.0):
        print("odom/joint_states/imu 수신 실패")
        return

    _, _, th0, j0 = n.snapshot()
    s = []
    t = time.time()
    while time.time() - t < 2.0:
        time.sleep(0.02)
        if n.imu:
            s.append(n.imu.angular_velocity.z)
    bias = statistics.fmean(s) if s else 0.0
    print("자이로 바이어스 %+.5f rad/s" % bias)

    target = a.turns * 2 * math.pi
    print(">>> 제자리 회전 %.2f rad/s, odom 기준 %.1f 바퀴" % (a.vz, a.turns))
    print("    바닥에 로봇 정면 방향을 표시해 두었는지 확인하세요\n")

    total = 0.0
    gyro = 0.0
    prev_th = th0
    prev_t = time.time()
    t0 = prev_t
    try:
        while abs(total) < target and time.time() - t0 < a.turns * 25:
            n.send(0.0, a.vz)
            _, _, cur, _ = n.snapshot()
            d = cur - prev_th
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            total += d
            prev_th = cur

            now = time.time()
            if n.imu:
                gyro += (n.imu.angular_velocity.z - bias) * (now - prev_t)
            prev_t = now
            time.sleep(0.02)
    finally:
        for _ in range(12):
            n.send(0.0, 0.0)
            time.sleep(0.04)
    time.sleep(1.5)

    _, _, th1, j1 = n.snapshot()
    dj = [j1[i] - j0[i] for i in range(len(j0))]
    odom_turns = total / (2 * math.pi)
    gyro_turns = gyro / (2 * math.pi)

    # [중요] 트레드는 관절 변화량으로 역산한다. 위의 total 은 목표에
    # 닿는 순간 누적을 끊으므로 감속하며 더 도는 부분이 빠져 있다.
    # 눈으로 읽는 각도는 완전히 멈춘 뒤의 값이라 그 감속분을 포함한다.
    # 둘을 섞어 비교하면 보정 부호가 뒤집힌다 — 실제로 한 번 그랬다.
    joint_diff = dj[1] - dj[0]

    print("=== 결과 ===")
    print("odom 누적 회전   %+.1f 도  (%.3f 바퀴)" % (math.degrees(total), odom_turns))
    print("자이로 적분      %+.1f 도  (%.3f 바퀴)" % (math.degrees(gyro), gyro_turns))
    print("관절 변화        좌 %+.2f  우 %+.2f rad" % (dj[0], dj[1]))
    if abs(odom_turns) > 1e-6:
        ratio = gyro_turns / odom_turns
        print("자이로/odom 비   %.4f" % ratio)
        print()
        print("자이로 기준 보정: wheel_separation %.5f -> %.5f"
              % (a.tread, a.tread * ratio))
    print()
    print("--- 눈으로 확인 ---")
    print("로봇 정면이 출발 표시에서 몇 도 어긋났는지 보세요.")
    print("  표시와 일치        -> 트레드가 맞다")
    print("  덜 돌았다(모자람)  -> odom 이 과대평가. 트레드를 줄인다")
    print("  더 돌았다(지나침)  -> odom 이 과소평가. 트레드를 늘린다")
    print()
    print("--- 트레드 역산 (관절 기준, 감속 포함) ---")
    print("  L = 바퀴반지름 x (관절_우 - 관절_좌) / 실제회전각")
    print("    = %.4f x %.2f rad / (실제각/57.3)" % (WHEEL_RADIUS, joint_diff))
    for deg in (math.degrees(total) - 20, math.degrees(total), math.degrees(total) + 20):
        L = WHEEL_RADIUS * joint_diff / math.radians(deg)
        print("    실제 %7.1f 도 -> %.2f mm" % (deg, L * 1000))
    print("  실제 회전각을 넣어 계산하세요. odom 누적값(%.1f도)을 쓰면 안 됩니다."
          % math.degrees(total))
    print()
    print("주의: 캐스터가 2개라 제자리 회전 시 둘 다 비틀리며 바닥을 긁는다.")
    print("      그 저항으로 구동륜이 미끄러지면 odom 과 자이로가 크게")
    print("      어긋난다. 비가 1.0 에서 많이 벗어나면 슬립을 의심할 것.")


def cmd_trace(n, a):
    """주행 품질 진단 — 지그재그·편향의 원인을 가린다."""
    if not n.wait_ready(need_imu=True, timeout=8.0):
        print("odom/joint_states/imu 수신 실패 — imu_sensor_broadcaster 확인")
        return
    print(">>> 주행 %.2f m/s, %.1f초 기록" % (a.vx, a.dur))
    n.run(a.vx, 0.0, a.dur, record=True)
    report_trace(n.trace, a.skip)


def cmd_hold(n, a):
    """방향 제어 직진.

    차동구동 로봇은 개루프로 직진할 수 없다. 좌우 어떤 미세한 차이도
    헤딩 오차로 적분되기 때문이다. 방향을 되먹임해서 vz 로 밀어주면
    좌우 특성이 달라도 직진한다.

    기준은 odom yaw 다 — nav2 가 쓰는 것과 같은 값이라, 이게 되면
    캘리브레이션과 제어 사슬 전체가 검증된다. 물리적으로도 곧게
    가는지는 자이로로 따로 잰다.
    """
    if not n.wait_ready(need_imu=True, timeout=8.0):
        print("odom/joint_states/imu 수신 실패")
        return

    x0, y0, th0, _ = n.snapshot()
    # 자이로 바이어스. 짧은 주행이라 이 정도면 충분하다.
    s = []
    t = time.time()
    while time.time() - t < 1.5:
        time.sleep(0.02)
        if n.imu:
            s.append(n.imu.angular_velocity.z)
    bias = statistics.fmean(s) if s else 0.0

    print(">>> 방향 제어 직진 %.2f m/s, %.1f초  (kp=%.1f)" % (a.vx, a.dur, a.kp))
    gyro_yaw = 0.0
    errs = []
    vzs = []
    rates = []
    rate = 0.0
    i_term = 0.0
    prev = time.time()
    t0 = prev
    try:
        while time.time() - t0 < a.dur:
            _, _, th, _ = n.snapshot()
            err = th - th0
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi
            # 헤딩 오차에 비례해 반대로 민다. 좌우 모터가 달라도
            # 틀어지는 즉시 되돌리므로 직진한다.
            #
            # 미분항은 자이로 각속도를 그대로 쓴다. 보드 가속 램프가
            # 느려 보정이 늦게 반영되는데, P 만 쓰면 그동안 오차가 더
            # 쌓여 과보정하고 반대로 넘어가기를 반복한다(좌우 진동).
            # 지금 돌고 있는 속도만큼 미리 빼주면 그게 줄어든다.
            # 적분항: 거의 일정한 편향(모터 특성차 등)을 학습해서 상쇄한다.
            # 비례항만 쓰면 오차가 생길 때마다 툭툭 밀어 차체가 움찔거리는데,
            # 적분은 일정한 보정을 걸어두므로 애초에 오차가 잘 안 생긴다.
            now_i = time.time()
            i_term += err * (now_i - prev)
            # 와인드업 방지
            i_lim = a.vzmax / max(a.ki, 1e-6)
            i_term = max(-i_lim, min(i_lim, i_term))

            raw_rate = (n.imu.angular_velocity.z - bias) if n.imu else 0.0
            # 자이로 원시 각속도는 잡음이 크다(σ 0.05 rad/s 수준). 그대로
            # 미분항에 쓰면 vz 명령에 잡음이 실려 부호가 계속 뒤집힌다.
            # 1차 저역통과로 걸러서 쓴다.
            rate = a.rate_lpf * rate + (1.0 - a.rate_lpf) * raw_rate
            rates.append(raw_rate)
            vz = -a.kp * err - a.ki * i_term - a.kd * rate
            # 데드밴드: 오차가 아주 작으면 건드리지 않는다. 미세 보정이
            # 계속 들어가면 그 자체가 흔들림이 된다.
            if abs(err) < math.radians(a.deadband):
                vz = -a.ki * i_term
            vz = max(-a.vzmax, min(a.vzmax, vz))
            n.send(a.vx, vz)
            errs.append(err)
            vzs.append(vz)
            now = time.time()
            if n.imu:
                gyro_yaw += (n.imu.angular_velocity.z - bias) * (now - prev)
            prev = now
            time.sleep(0.02)
    finally:
        for _ in range(12):
            n.send(0.0, 0.0)
            time.sleep(0.04)
    time.sleep(1.5)

    x1, y1, th1, _ = n.snapshot()
    dx, dy = x1 - x0, y1 - y0
    # odom 좌표계 그대로 y 를 보면 안 된다. 출발 시 로봇이 odom 상에서
    # 어느 방향을 보고 있었는지에 따라 x·y 가 섞이기 때문이다.
    # 출발 헤딩 기준으로 회전시켜 전진/가로 성분을 나눈다.
    fwd = dx * math.cos(th0) + dy * math.sin(th0)
    lat = -dx * math.sin(th0) + dy * math.cos(th0)
    dth = th1 - th0
    while dth > math.pi:
        dth -= 2 * math.pi
    while dth < -math.pi:
        dth += 2 * math.pi

    print("\n=== 결과 ===")
    print("전진 거리        %.0f mm" % (fwd * 1000))
    print("가로 이탈        %+.0f mm   (%.2f%% of 전진)"
          % (lat * 1000, 100 * lat / fwd if abs(fwd) > 1e-6 else 0))
    print("odom 헤딩 변화   %+.2f 도" % math.degrees(dth))
    print("자이로 적분      %+.2f 도   <- 물리적으로 실제 돈 각도" % math.degrees(gyro_yaw))
    if errs:
        print("헤딩 오차 RMS    %.2f 도"
              % math.degrees(math.sqrt(sum(e * e for e in errs) / len(errs))))
    if rates:
        # 실제 흔들림은 자이로 각속도의 변동으로 잰다. vz 명령의 부호
        # 전환 횟수는 명령의 잡음일 뿐 물리적 흔들림이 아니다.
        print("자이로 각속도 σ  %.4f rad/s  (%.2f 도/초)  <- 실제 흔들림"
              % (statistics.pstdev(rates), math.degrees(statistics.pstdev(rates))))
    if vzs:
        print("보정 명령 RMS    %.3f rad/s" % math.sqrt(sum(v * v for v in vzs) / len(vzs)))
    print()
    # 판정은 자이로로 한다. 제어기가 odom 을 기준으로 잡고 있으므로
    # odom 헤딩이 0 인 건 당연하고, 물리적으로 곧았는지는 자이로만 안다.
    gyro_deg = abs(math.degrees(gyro_yaw))
    if gyro_deg < 2.0:
        print("물리적으로 직진했다 (%.1f m 에 %.2f 도)." % (fwd, gyro_deg))
    elif gyro_deg < 5.0:
        print("거의 직진했다 (%.1f m 에 %.2f 도). kp 를 올리면 더 줄어든다." % (fwd, gyro_deg))
    else:
        print("아직 휜다 (%.1f m 에 %.2f 도)." % (fwd, gyro_deg))
        print("odom 기준으로는 잡고 있는데 물리적으로 휜다면 "
              "오도메트리 캘리브레이션이 남은 것이다.")


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
        for _ in range(100):
            if fut.done():
                break
            time.sleep(0.05)
        r = fut.result()
        return bool(r and all(x.successful for x in r.results))

    if not n.wait_ready(need_imu=True, timeout=8.0):
        print("토픽 수신 실패")
        return

    # 목표 바퀴 각속도. 이보다 크게 처지면 게인이 너무 낮아 제어가 죽은 것이다.
    target_w = a.vx / WHEEL_RADIUS
    values = [float(x) for x in a.values.split(',')]
    base = {'kp': a.kp, 'ki': a.ki, 'kd': a.kd}
    fixed = ", ".join("%s=%.3f" % (k, v) for k, v in base.items() if k != a.sweep)
    print("게인 스윕: %s = %s   (%s 고정)" % (a.sweep, values, fixed))
    print("명령 %.2f m/s, 각 시행 %.0f초, 앞 %.1f초 제외\n" % (a.vx, a.dur, a.skip))
    print("  %-5s  좌평균  우평균   좌우차   좌변동  우변동  자이로평균  목표대비"
          % a.sweep)
    print("  " + "-" * 70)

    rows = []
    for val in values:
        g = dict(base)
        g[a.sweep] = val
        if not set_pid(g['kp'], g['ki'], g['kd']):
            print("  %.2f   PID 설정 실패" % val)
            continue
        time.sleep(0.5)
        n.trace = []
        n.run(a.vx, 0.0, a.dur, record=True)
        st = trace_stats(n.trace, a.skip)
        if not st:
            print("  %.2f   표본 부족" % val)
            continue
        reach = (st['ml'] + st['mr']) / 2 / max(target_w, 1e-6)
        st['reach'] = reach
        # 좌우 평균 속도 차이 — 저속 비대칭의 직접 지표
        mean_w = max(abs(st['ml'] + st['mr']) / 2, 1e-6)
        st['asym'] = 100 * (st['ml'] - st['mr']) / mean_w
        rows.append((val, st))
        flag = "" if reach >= 0.85 else "  <- 속도미달"
        print("  %-5.2f  %6.2f  %6.2f  %+6.1f%%  %5.1f%%  %5.1f%%  %+9.4f  %3.0f%%%s"
              % (val, st['ml'], st['mr'], st['asym'], st['cvl'], st['cvr'],
                 st['gzmean'], reach * 100, flag))
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
        least_asym = min(ok, key=lambda r: abs(r[1]['asym']))
        least_osc = min(ok, key=lambda r: (r[1]['cvl'] + r[1]['cvr']) / 2)
        print("\n좌우 비대칭이 가장 작은 %s = %.2f  (좌우차 %+.1f%%)"
              % (a.sweep, least_asym[0], least_asym[1]['asym']))
        print("진동이 가장 작은   %s = %.2f  (평균 변동 %.1f%%)"
              % (a.sweep, least_osc[0], (least_osc[1]['cvl'] + least_osc[1]['cvr']) / 2))
        print("\n적용:  ros2 param set /%s %s.motor_pid_%s <값>" % (a.hw, a.hw, a.sweep))
        print("\n주의: ki 를 올리면 데드밴드를 넘기는 힘이 생겨 저속 비대칭이")
        print("      줄지만, 너무 크면 오버슈트해서 진동(변동%%)이 늘어난다.")
        print("      둘이 갈리면 실제 운용 속도에서 다시 재볼 것.")


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

    p = sub.add_parser('spin', help='제자리 회전 — wheel_separation 검산')
    p.add_argument('--vz', type=float, default=1.0)
    p.add_argument('--turns', type=float, default=5.0)
    p.add_argument('--tread', type=float, default=0.11625,
                   help='현재 설정된 wheel_separation')

    p = sub.add_parser('trace', help='주행 품질 진단')
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--dur', type=float, default=8.0)
    p.add_argument('--skip', type=float, default=2.5,
                   help='앞부분 가속 램프를 제외할 초 (기본 2.5)')

    p = sub.add_parser('hold', help='방향 제어 직진')
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--dur', type=float, default=10.0)
    p.add_argument('--kp', type=float, default=2.0,
                   help='헤딩 오차 -> 각속도 비례 게인 (rad/s per rad)')
    p.add_argument('--kd', type=float, default=0.0,
                   help='자이로 각속도 감쇠 게인. 좌우 진동을 줄인다')
    p.add_argument('--ki', type=float, default=0.0,
                   help='헤딩 오차 적분 게인. 일정한 편향을 학습해 상쇄한다')
    p.add_argument('--deadband', type=float, default=0.0,
                   help='이 각도(도) 안에서는 비례 보정을 하지 않는다')
    p.add_argument('--rate-lpf', dest='rate_lpf', type=float, default=0.8,
                   help='미분항에 쓰는 각속도의 저역통과 계수 (0~1, 클수록 부드러움)')
    p.add_argument('--vzmax', type=float, default=0.6,
                   help='보정 각속도 상한 (rad/s)')

    p = sub.add_parser('pidsweep', help='보드 PID 게인 스윕')
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--dur', type=float, default=6.0)
    p.add_argument('--skip', type=float, default=3.0)
    p.add_argument('--sweep', choices=['kp', 'ki', 'kd'], default='kp',
                   help='어느 게인을 훑을지')
    p.add_argument('--values', type=str, default='0.4,0.8,1.2',
                   help='시험할 값들 (쉼표 구분)')
    p.add_argument('--kp', type=float, default=0.8, help='고정할 kp (보드 기본 0.8)')
    p.add_argument('--ki', type=float, default=0.06,
                   help='고정할 ki (보드 기본 0.06). 0 으로 두면 목표 속도에 '
                        '도달하지 못한다')
    p.add_argument('--kd', type=float, default=0.5, help='고정할 kd (보드 기본 0.5)')
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
         'hold': cmd_hold, 'pidsweep': cmd_pidsweep}[a.cmd](n, a)
    finally:
        n.shutdown()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

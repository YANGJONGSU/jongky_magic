# -*- coding: utf-8 -*-
"""제자리 회전 계측 — 최대 각속도로 N초 돌려 실제 회전량을 이론값과 비교한다.

무엇을 재는가
-------------
"최대 각속도를 명령했을 때 로봇이 실제로 그만큼 도는가" 하나만 본다.
회전량은 루트 쿼터니언의 yaw 를 언랩해 누적하므로 여러 바퀴를 돌아도 맞다.

**이론값을 상수로 박지 않는다.** env 의 액션 파이프라인을 그대로 태운 뒤
그 결과인 ``env._wheel_target`` 에서 역기구학으로 명령 각속도를 되뽑는다.

    omega_cmd = (w_right - w_left) * WHEEL_RADIUS / WHEEL_SEPARATION

액션 스케일이 바뀌어도(예: tanh 한 겹을 없애 act=1.0 의 실효 명령이
1.142 -> 1.500 rad/s 로 바뀌어도) 이 도구는 따라온다. 상수를 박아 뒀다면
정확히 그 변경에서 기준선이 틀어졌을 것이다.

왜 만들었나 — 캐스터가 끌리고 있었다
------------------------------------
캐스터가 URDF 상 base_link 의 고정 충돌 구슬이라 구르지 못하고 끌렸다.
제자리 회전에서 구동륜과 캐스터가 같은 마찰계수를 쓰므로 비에서 mu 가
상쇄된다 — 즉 지면 마찰을 아무리 바꿔도 결과가 안 변한다.

    캐스터 하중 분담 = |COM_x| / |caster_x| = 0.0494 / 0.1252 = 39.5%
    저항 토크 = mu x 10.08 N x 0.12832 m = mu x 1.293 N.m
    구동 토크 = mu x 15.46 N x 0.059545 m = mu x 0.920 N.m
    비 = 0.71 < 1  ->  쿨롱 기준 제자리 회전 불가

계측값도 그대로였다. 최대 각속도 20초에 3.1 rad — 이론값의 1/7.4.

고친 방법은 URDF 쪽이다. ``is_sim:=true`` 로 전개하면 캐스터가 별도 링크
2개 + 관절 2개(수직 스위블 + Y축 롤)로 펼쳐져 구른다. 저항이 미끄럼
마찰에서 구름 저항으로 바뀌므로 비가 0.69 -> 14 이상이 된다.
근거는 ``robot/jongky_description/urdf/jongky.urdf.xacro`` 의 캐스터 절.

그래서 이 도구는 회전량만 재는 게 아니라 **USD 에 캐스터 관절이 살아
있는지, 그 관절이 자유롭게 구를 수 있는지**도 같이 본다. 둘 중 하나라도
어긋나면 회전량만 보고는 원인을 못 가른다.

노트북 검증 순서 (192.168.129.97)
---------------------------------
이 기계에는 Isaac Lab 이 없어 실행 검증을 못 했다. 노트북에서 아래 순서로
돌릴 것. 1~3 은 URDF 를 고쳤으니 **반드시 다시 해야 한다** — 예전 USD 에는
캐스터 관절이 없다.

0) 저장소를 노트북으로 동기화한다 (map-quality 브랜치).

1) xacro 를 평문 URDF 로. **is_sim:=true 를 반드시 붙일 것.**
   빠뜨리면 예전과 똑같은 고정 구슬이 나오고 이 도구가 5번에서 잡아낸다.

       cd ~/jongky_magic
       source /opt/ros/jazzy/setup.bash
       mkdir -p ~/jongky_usd
       xacro robot/jongky_description/urdf/robot.urdf.xacro \
             is_sim:=true use_mock:=true > ~/jongky_usd/jongky.urdf

2) package:// 참조를 상대경로로 바꾸고 STL 을 같이 둔다.

       sed -i 's|package://jongky_description/meshes/|meshes/|g' ~/jongky_usd/jongky.urdf
       cp -r robot/jongky_description/meshes ~/jongky_usd/

3) USD 변환. **joint-damping 을 0.0 으로 둘 것** (예전 값 100).

       cd ~/isaac/IsaacLab
       OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python \
         scripts/tools/convert_urdf.py ~/jongky_usd/jongky.urdf \
         ~/jongky_usd_merged/jongky.usd \
         --merge-joints --joint-target-type velocity \
         --joint-stiffness 0.0 --joint-damping 0.0 --headless

   convert_urdf.py 의 게인 인자는 **모든 관절에 일괄로** 들어간다.
   캐스터는 수동 관절이라 env 에 덮어쓸 액추에이터 항목이 없으므로,
   100 으로 변환하면 감쇠 100 에 묶여 못 구른다 — 고정 구슬과 같아진다.
   구동륜은 env 의 ImplicitActuatorCfg(damping 2.0)가 어차피 다시 쓰므로
   0.0 으로 변환해도 아무 영향이 없다.
   (이 도구는 5번에서 캐스터 감쇠를 읽어 확인하고, 0 이 아니면 계측 전에
    0 으로 눌러 준다. 그래도 USD 를 제대로 만들어 두는 게 맞다 —
    학습은 이 도구를 안 거치기 때문이다.)

4) 병합 결과 확인. 질량 합 2.603 kg, 관절 6개가 나와야 한다
   (l/r_wheel_joint + l/r_caster_swivel_joint + l/r_caster_roll_joint).
   바디는 7개다 (base_footprint · 바퀴2 · 요크2 · 볼2).

       cd ~/jongky_magic/sim/jongky_rl
       OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \
         tools/check_merged.py --usd ~/jongky_usd_merged/jongky.usd --headless

5) 회전 계측. 이 파일이다.

       cd ~/jongky_magic/sim/jongky_rl
       OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \
         tools/check_rotation.py --headless --enable_cameras

   --enable_cameras 없으면 env 가 카메라 초기화에서 죽는다.
   -u 없으면 stdout 버퍼링으로 print 가 유실된다.
   exit code 는 믿지 말 것 — 실패해도 0 이 나온다. 출력을 직접 볼 것.

6) 판정. 마지막 "판정" 줄이 전부다.

       비 >= 0.90   고쳐졌다
       비 0.5~0.9   구르긴 하는데 어딘가 저항이 남았다.
                    캐스터 관절 감쇠/마찰과 effort_limit_sim 을 볼 것
       비 <= 0.2    고치기 전과 같다. 위 1번의 is_sim:=true 또는
                    3번의 joint-damping 0.0 을 빠뜨렸을 가능성이 가장 크다

   비교 대조가 필요하면 예전 USD 를 지우지 말고 이렇게 돌린다.

       JONGKY_USD=~/jongky_usd_merged_old/jongky.usd ... tools/check_rotation.py ...

7) 직진이 안 망가졌는지도 한 번 본다 (캐스터를 링크로 떼면서 차체 질량이
   2.503 -> 2.463 kg 으로 바뀌었다. 합계는 2.603 그대로다).

       OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \
         tools/diag_drive.py --headless --enable_cameras
       # v_body 가 0.40 근처(예전 0.398)로 유지되면 정상
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# tools/ 에서 실행하든 sim/jongky_rl 에서 실행하든 env 를 찾게 한다.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)
for _p in (_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def build_parser() -> tuple[argparse.ArgumentParser, bool]:
    """인자 파서. Isaac 이 없어도 --help 는 떠야 하므로 임포트를 감싼다."""
    parser = argparse.ArgumentParser(
        description="제자리 회전 계측 — 최대 각속도 N초의 실제 회전량 대 이론값",
        epilog=(
            "이론값은 상수가 아니라 env 의 액션 파이프라인에서 되뽑는다. "
            "노트북 실행 순서는 이 파일 맨 위 docstring 의 '노트북 검증 순서' 절 참조."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seconds", type=float, default=20.0, help="회전 계측 시간 (초)")
    parser.add_argument(
        "--action", type=float, default=1.0,
        help="omega 액션값. 1.0 = 정책이 낼 수 있는 최대 (v 액션은 0 으로 고정)",
    )
    parser.add_argument("--envs", type=int, default=2, help="동시에 돌릴 env 개수")
    parser.add_argument(
        "--friction", type=float, default=None,
        help="지면 마찰 덮어쓰기. 안 주면 env 기본값. "
             "[참고] 구동륜과 캐스터가 같은 재질을 쓰는 한 미끄럼 기준 비에서 "
             "mu 는 상쇄된다 — 마찰 스윕으로는 이 문제가 안 움직인다",
    )
    parser.add_argument(
        "--keep-caster-drive", action="store_true",
        help="캐스터 관절의 USD 드라이브 게인을 손대지 않는다. 기본은 0 으로 눌러 "
             "'변환 인자를 잘못 줘서 못 구르는 것'과 '기구학이 잘못된 것'을 가른다",
    )
    parser.add_argument(
        "--usd", type=str, default=None,
        help="쓸 USD 경로. 안 주면 JONGKY_USD 환경변수 / env 기본값",
    )

    try:
        from isaaclab.app import AppLauncher  # noqa: F401
    except Exception:
        parser.epilog += (
            "\n\n[알림] 이 기계에는 Isaac Lab 이 없다. --help 만 뜨고 계측은 못 한다. "
            "--headless / --enable_cameras 같은 AppLauncher 인자도 지금은 안 붙는다."
        )
        return parser, False

    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    return parser, True


parser, HAS_ISAAC = build_parser()
args = parser.parse_args()

if not HAS_ISAAC:
    print(
        "Isaac Lab 을 못 찾았다. 계측은 노트북(192.168.129.97, ~/isaac/env_isaaclab)에서 돌릴 것.\n"
        "  OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \\\n"
        "    tools/check_rotation.py --headless --enable_cameras\n"
        "자세한 순서는 이 파일 맨 위 docstring 의 '노트북 검증 순서' 절.",
        file=sys.stderr,
    )
    raise SystemExit(2)

if args.usd:
    os.environ["JONGKY_USD"] = os.path.expanduser(args.usd)

from isaaclab.app import AppLauncher  # noqa: E402

simulation_app = AppLauncher(args).app

import torch  # noqa: E402

import jongky_corridor_env as jce  # noqa: E402
from jongky_corridor_env import JongkyCorridorEnv, JongkyCorridorEnvCfg  # noqa: E402

WHEEL_RADIUS = jce.WHEEL_RADIUS
WHEEL_SEPARATION = jce.WHEEL_SEPARATION

RULE = "=" * 78
CASTER_KEY = "caster"


def yaw_of(quat: torch.Tensor) -> torch.Tensor:
    """(w, x, y, z) -> yaw. env._robot_yaw 와 같은 식이지만 private 에 안 묶인다."""
    return torch.atan2(
        2.0 * (quat[:, 0] * quat[:, 3] + quat[:, 1] * quat[:, 2]),
        1.0 - 2.0 * (quat[:, 2] ** 2 + quat[:, 3] ** 2),
    )


def wrap_pi(x: torch.Tensor) -> torch.Tensor:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def col(robot, name: str):
    """(num_envs, num_joints) 관절 속성을 env 0 리스트로. 없으면 None."""
    t = getattr(robot.data, name, None)
    if t is None:
        return None
    return t[0].tolist() if t.dim() == 2 else t.tolist()


# ── env 구성 ────────────────────────────────────────────────────────────────
cfg = JongkyCorridorEnvCfg()
cfg.scene.num_envs = args.envs
if args.friction is not None:
    cfg.ground_static_friction = args.friction
    cfg.ground_dynamic_friction = args.friction

env = JongkyCorridorEnv(cfg)
env.reset()

robot = env._robot
dt = env.sim.get_physics_dt()
decim = cfg.decimation
names = list(robot.joint_names)

print(RULE)
print("USD          :", jce.JONGKY_USD)
print("지면 마찰    : static %.3f / dynamic %.3f"
      % (cfg.ground_static_friction, cfg.ground_dynamic_friction))
print("바디 %d개    : %s" % (len(robot.body_names), robot.body_names))
print("질량 합      : %.4f kg  (실차 2.603)" % sum(robot.data.default_mass[0].tolist()))
print(RULE)

# ── 관절 점검 ───────────────────────────────────────────────────────────────
stiff = col(robot, "joint_stiffness")
damp = col(robot, "joint_damping")
arma = col(robot, "joint_armature")
fric = col(robot, "joint_friction")
if fric is None:
    fric = col(robot, "joint_friction_coeff")

print("관절 %d개" % len(names))
print("  %-24s %10s %10s %10s %10s" % ("이름", "stiffness", "damping", "armature", "friction"))
for i, n in enumerate(names):
    def s(v):
        return "%10.4f" % v[i] if v is not None else "%10s" % "-"
    print("  %-24s %s %s %s %s" % (n, s(stiff), s(damp), s(arma), s(fric)))

caster_ids = [i for i, n in enumerate(names) if CASTER_KEY in n.lower()]
roll_ids = [i for i in caster_ids if "roll" in names[i].lower()]

print("-" * 78)
problems: list[str] = []

if not caster_ids:
    problems.append(
        "USD 에 캐스터 관절이 하나도 없다. 고정 구슬 버전이다.\n"
        "     -> xacro 를 is_sim:=true 로 다시 전개해서 USD 를 다시 만들 것\n"
        "        (docstring '노트북 검증 순서' 1~3번). 아래 회전량은 고치기 전 값이다."
    )
    print("[경고] 캐스터 관절 없음 — 끌리는 고정 구슬 버전이다.")
else:
    print("캐스터 관절 %d개: %s" % (len(caster_ids), [names[i] for i in caster_ids]))
    bad = [i for i in caster_ids if (damp and damp[i] > 1e-6) or (stiff and stiff[i] > 1e-6)]
    if bad:
        msg = ("캐스터 관절에 드라이브 게인이 살아 있다 (%s). USD 를 "
               "joint-damping 0.0 으로 다시 변환할 것." % [names[i] for i in bad])
        if args.keep_caster_drive:
            problems.append(msg + " (--keep-caster-drive 라 그대로 둔다)")
            print("[경고] " + msg)
        else:
            robot.write_joint_stiffness_to_sim(0.0, joint_ids=caster_ids)
            robot.write_joint_damping_to_sim(0.0, joint_ids=caster_ids)
            problems.append(msg + " 계측을 위해 이번 실행에서는 0 으로 눌렀다 — "
                                  "학습은 이 도구를 안 거치므로 USD 를 고쳐야 한다.")
            print("[경고] " + msg)
            print("       이번 실행에 한해 stiffness/damping 을 0 으로 눌렀다.")
    else:
        print("캐스터 드라이브 게인 0 확인 — 자유롭게 구를 수 있다.")

# 구동륜 인덱스가 좌·우 순서인지. 뒤집혀 있으면 회전 부호가 통째로 뒤집힌다.
widx = list(env._wheel_idx)
mapped = [names[i] for i in widx]
print("구동륜 인덱스: %s -> %s" % (widx, mapped))
if mapped != ["l_wheel_joint", "r_wheel_joint"]:
    problems.append("구동륜 인덱스 순서가 [좌, 우] 가 아니다 (%s). "
                    "env 의 _wheel_target 이 좌우 바뀌어 들어간다." % mapped)
    print("[경고] 구동륜 인덱스 순서가 [좌, 우] 가 아니다.")

# ── 명령 각속도를 env 에서 되뽑는다 ─────────────────────────────────────────
action = torch.zeros(env.num_envs, 2, device=env.device)
action[:, 1] = args.action
env._pre_physics_step(action)

wt = getattr(env, "_wheel_target", None)
if wt is not None:
    omega_cmd = ((wt[:, 1] - wt[:, 0]) * WHEEL_RADIUS / WHEEL_SEPARATION)[0].item()
    src = "env._wheel_target 역기구학"
elif hasattr(jce, "scale_action"):
    omega_cmd = jce.scale_action(action)[0, 1].item()
    src = "jongky_corridor_env.scale_action"
else:
    omega_cmd = jce.OMEGA_MAX * args.action
    src = "OMEGA_MAX x action (최후 수단 — 액션 스케일을 못 읽었다)"

wheel_cmd = wt[0].tolist() if wt is not None else None
theory = abs(omega_cmd) * args.seconds

print("-" * 78)
print("액션 [v, omega] = [0.0, %+.3f]" % args.action)
print("명령 각속도     : %+.4f rad/s   (출처: %s)" % (omega_cmd, src))
if wheel_cmd:
    print("바퀴 목표       : 좌 %+.3f / 우 %+.3f rad/s" % (wheel_cmd[0], wheel_cmd[1]))
print("이론 회전량     : %.3f rad = %.2f 바퀴  (%.1f초)"
      % (theory, theory / (2 * math.pi), args.seconds))
if abs(omega_cmd) < 1e-6:
    print("[경고] 명령 각속도가 0 이다. --action 을 확인할 것.")
print(RULE)

# ── 회전 계측 ───────────────────────────────────────────────────────────────
# env.step 을 안 쓴다. 렌더링·보상·종료판정·자동리셋이 계측을 끊기 때문이다.
# 대신 env 의 액션 매핑(_pre_physics_step/_apply_action)은 그대로 태운다 —
# 액션 스케일이 바뀌어도 이 도구가 따라가야 하므로 그 경로를 우회하면 안 된다.
n_policy = max(1, int(round(args.seconds / (dt * decim))))
prev_yaw = yaw_of(robot.data.root_quat_w).clone()
total = torch.zeros(env.num_envs, device=env.device)
report_every = max(1, n_policy // 4)

print("계측 %d 스텝 (정책 dt %.4f s x %d)" % (n_policy, dt * decim, n_policy))
print("  %6s %10s %10s %22s %18s" % ("t[s]", "누적[rad]", "omega", "바퀴 실측[rad/s]", "캐스터롤[rad/s]"))

for k in range(n_policy):
    env._pre_physics_step(action)
    for _ in range(decim):
        env._apply_action()
        robot.write_data_to_sim()
        env.sim.step(render=False)
        robot.update(dt)

    yaw = yaw_of(robot.data.root_quat_w)
    total += wrap_pi(yaw - prev_yaw)
    prev_yaw = yaw.clone()

    if k % report_every == 0 or k == n_policy - 1:
        jv = robot.data.joint_vel[0]
        w = [round(jv[i].item(), 2) for i in widx]
        r = [round(jv[i].item(), 2) for i in roll_ids] if roll_ids else []
        print("  %6.2f %10.3f %10.3f %22s %18s"
              % ((k + 1) * dt * decim, total[0].item(),
                 robot.data.root_ang_vel_w[0, 2].item(), w, r))

# ── 결과 ────────────────────────────────────────────────────────────────────
actual = total.abs()
mean_actual = actual.mean().item()
ratio = mean_actual / theory if theory > 0 else float("nan")
tilt = robot.data.projected_gravity_b[:, 2]

print(RULE)
print("이론  %.3f rad (%.2f 바퀴)" % (theory, theory / (2 * math.pi)))
for e in range(env.num_envs):
    print("  env%-2d 실측 %8.3f rad (%5.2f 바퀴)  비 %.3f   평균 %+.3f rad/s"
          % (e, actual[e].item(), actual[e].item() / (2 * math.pi),
             actual[e].item() / theory if theory > 0 else float("nan"),
             total[e].item() / args.seconds))
print("  평균  실측 %8.3f rad             비 %.3f" % (mean_actual, ratio))
print("  정상속도 omega %+.3f rad/s (명령 %+.3f)"
      % (robot.data.root_ang_vel_w[:, 2].mean().item(), omega_cmd))

if (tilt > -0.5).any():
    problems.append("계측 중 로봇이 넘어졌다 (projected_gravity_b.z > -0.5). 회전량은 못 믿는다.")

print("-" * 78)
if ratio >= 0.90:
    verdict = "판정: 정상 — 이론값의 %.0f%% 를 돈다." % (ratio * 100)
elif ratio >= 0.50:
    verdict = ("판정: 부분 개선 — 이론값의 %.0f%%. 구르긴 하는데 저항이 남았다. "
               "캐스터 관절 감쇠/마찰과 effort_limit_sim(0.5) 을 볼 것." % (ratio * 100))
else:
    verdict = ("판정: 실패 — 이론값의 %.0f%% (1/%.1f). 끌리고 있다. "
               "xacro is_sim:=true 와 USD 의 joint-damping 0.0 을 확인할 것."
               % (ratio * 100, 1.0 / ratio if ratio > 0 else float("inf")))
print(verdict)
for p in problems:
    print("  [!] " + p)
print(RULE)

env.close()
simulation_app.close()

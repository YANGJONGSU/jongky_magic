# 실측 복도에서 로봇이 가장 좁은 구간을 실제로 통과하는지 확인한다.
#
# 기하 계산으로 "1.20 m 복도에 폭 0.17 m 로봇이 들어간다" 는 당연하지만,
# 실제로 통과하는지는 별개다 — 캐스터 마찰, 가속 램프, 헤딩 드리프트가
# 겹치면 직진 명령만으로도 벽에 붙는다. 그래서 물리를 돌려서 확인한다.
#
# 확인 항목
#   1) 최협 구간을 직진으로 통과하는가 (충돌 없이 반대편으로 나오는가)
#   2) 종료 판정선이 설계한 위치(반폭 - robot_half_width)에 있는가
#   3) 최협 구간에서 제자리 회전이 되는가 (외접원 반지름 기준)
#
# 실행:
#   python -u tools/check_corridor_fit.py --headless --enable_cameras

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--json", default="corridor_L10b.json")
parser.add_argument("--steps", type=int, default=900)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import torch  # noqa: E402

from jongky_map_corridor_env import JongkyMapCorridorEnv, JongkyMapCorridorEnvCfg  # noqa: E402

# 실차 footprint (nav2_params.yaml 과 같아야 한다)
FOOTPRINT = [(-0.14, -0.085), (-0.14, 0.085), (0.08, 0.085), (0.08, -0.085)]
ROBOT_WIDTH = 0.17
CIRCUMSCRIBED_R = max((x * x + y * y) ** 0.5 for x, y in FOOTPRINT)

cfg = JongkyMapCorridorEnvCfg()
cfg.geometry_json = args.json
cfg.scene.num_envs = 4
cfg.episode_length_s = 120.0
env = JongkyMapCorridorEnv(cfg)

secs = env._sections
narrow = min(secs, key=lambda s: s.width)
print("=" * 68)
print(f"최협 구간 : x {narrow.x0:.2f}~{narrow.x1:.2f} m, 폭 {narrow.width:.3f} m")
print(f"로봇      : 전폭 {ROBOT_WIDTH} m, 외접원 R={CIRCUMSCRIBED_R:.4f} m")
print(f"기하 여유 : 직진 {(narrow.width - ROBOT_WIDTH)/2:.3f} m/측, "
      f"회전 {narrow.width/2 - CIRCUMSCRIBED_R:.3f} m/측")
print(f"종료판정선: |y| > {narrow.width/2 - cfg.robot_half_width:.3f} m")
print("=" * 68)

env.reset()


def place(x, y, yaw=0.0):
    """로봇을 지정 위치에 놓는다 (모든 env 동일)."""
    n = env.num_envs
    root = env._robot.data.default_root_state.clone()
    root[:, 0] = x
    root[:, 1] = y
    root[:, 3] = torch.cos(torch.tensor(yaw / 2, device=env.device))
    root[:, 4:6] = 0.0
    root[:, 6] = torch.sin(torch.tensor(yaw / 2, device=env.device))
    root[:, :3] += env.scene.env_origins
    env._robot.write_root_pose_to_sim(root[:, :7])
    env._robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=env.device))
    env._collided[:] = False
    # 목표를 멀리 두어 도달 종료가 안 걸리게 한다
    env._goal[:, 0] = env._corridor_length - 0.5
    env._goal[:, 1] = 0.0
    env._prev_dist[:] = torch.norm(env._goal - env._robot_xy(), dim=-1)
    env._prev_cmd[:] = 0.0
    env.episode_length_buf[:] = 0


results = {}

# ── 1. 최협 구간 직진 통과 ────────────────────────────────────────────────
print(f"\n[1] 최협 구간 직진 통과  (x {narrow.x0:.2f} 앞에서 출발 → x {narrow.x1:.2f} 통과 목표)")
place(max(0.3, narrow.x0 - 0.8), 0.0, 0.0)
entered = exited = collided_in = False
min_clear = 99.0
for i in range(args.steps):
    act = torch.zeros(env.num_envs, 2, device=env.device)
    act[:, 0] = 1.0
    env.step(act)
    xy = env._robot_xy()
    x0, y0 = float(xy[0, 0]), float(xy[0, 1])
    if narrow.x0 <= x0 <= narrow.x1:
        entered = True
        min_clear = min(min_clear, narrow.width / 2 - abs(y0) - ROBOT_WIDTH / 2)
        if bool(env._collided[0]):
            collided_in = True
    if x0 > narrow.x1:
        exited = True
        break
    if i % 150 == 0:
        w = 2 * float(env._half_width_at(xy[:, 0])[0])
        print(f"    step {i:4d} | x={x0:+.3f} y={y0:+.3f} | 그 x 의 폭 {w:.2f} "
              f"| collided={bool(env._collided[0])}")
xy = env._robot_xy()
print(f"    최종 x={float(xy[0,0]):+.3f} y={float(xy[0,1]):+.3f}  "
      f"진입={entered} 통과={exited} 구간내충돌={collided_in}")
if entered:
    print(f"    최협 구간 통과 중 최소 측면 여유: {min_clear:.3f} m")
results["최협 구간 직진 통과"] = exited and not collided_in

# ── 2. 종료 판정선이 설계 위치에 있는가 ───────────────────────────────────
# 물리를 한 스텝 굴려서 확인하면 안 된다. 판정선(반폭-0.09=0.510)과 로봇이
# 벽에 실제로 닿는 위치(반폭-0.085=0.515)의 차이가 5 mm 뿐이라, 판정선 바깥에
# 로봇을 놓으면 이미 벽과 겹쳐서 PhysX 가 밀어낸다 → 판정 전에 안으로 돌아온다.
# 그래서 포즈만 써 넣고 _get_dones() 를 직접 불러 판정 로직만 격리해 본다.
print(f"\n[2] 종료 판정선 검증  (최협 구간 x={0.5*(narrow.x0+narrow.x1):.2f} 에서 y 를 바꿔가며)")
limit = narrow.width / 2 - cfg.robot_half_width
xm = 0.5 * (narrow.x0 + narrow.x1)
ok2 = True


def collide_at(x, y):
    """물리를 굴리지 않고 판정 로직만 본다."""
    root = env._robot.data.default_root_state.clone()
    root[:, 0] = x
    root[:, 1] = y
    root[:, :3] += env.scene.env_origins
    env._robot.write_root_pose_to_sim(root[:, :7])
    env.scene.update(dt=0.0)          # root_pos_w 캐시 갱신
    env._get_dones()
    return bool(env._collided[0])


for dy, expect in [(limit - 0.02, False), (limit + 0.02, True),
                   (-(limit - 0.02), False), (-(limit + 0.02), True)]:
    got = collide_at(xm, dy)
    mark = "OK" if got == expect else "NG"
    if got != expect:
        ok2 = False
    print(f"    y={dy:+.3f} (판정선 ±{limit:.3f}) → collided={got}, 기대={expect}  {mark}")

# 넓은 구간에서는 같은 y 가 충돌이 아니어야 한다 (구간별 판정이 실제로 되는가)
wide = max(secs, key=lambda s: s.width)
xw = 0.5 * (wide.x0 + wide.x1)
got = collide_at(xw, limit + 0.02)
print(f"    넓은 구간(x={xw:.2f}, 폭 {wide.width:.2f}) 같은 y={limit+0.02:+.3f} → "
      f"collided={got}, 기대=False  {'OK' if not got else 'NG'}")
if got:
    ok2 = False
results["종료 판정선 위치"] = ok2

# ── 3. 최협 구간에서 제자리 회전 ──────────────────────────────────────────
# 주의: 회전이 느리게 나오면 복도 탓인지 로봇 동역학 탓인지 갈라야 한다.
# 종키는 캐스터가 URDF 상 고정 구슬이라 시뮬에서 구르지 못하고 끌린다
# (jongky_corridor_env.py 머리주석 참고). 그래서 아래 [4] 에서 벽이 전혀
# 없는 조건으로 같은 회전을 돌려 대조군을 만든다.
print(f"\n[3] 최협 구간 제자리 회전  (외접원 R={CIRCUMSCRIBED_R:.4f} m)")
place(xm, 0.0, 0.0)
spun = 0.0
prev_yaw = float(env._robot_yaw()[0])
hit = False
for i in range(600):
    act = torch.zeros(env.num_envs, 2, device=env.device)
    act[:, 1] = 1.0            # 최대 각속도
    env.step(act)
    yaw = float(env._robot_yaw()[0])
    d = yaw - prev_yaw
    while d > 3.14159:
        d -= 2 * 3.14159
    while d < -3.14159:
        d += 2 * 3.14159
    spun += abs(d)
    prev_yaw = yaw
    if bool(env._collided[0]):
        hit = True
        break
xy = env._robot_xy()
print(f"    회전량 {spun:.2f} rad ({spun/6.283:.2f} 바퀴), 충돌={hit}, "
      f"최종 y={float(xy[0,1]):+.3f}")
narrow_spin = spun
narrow_hit = hit

# ── 4. 대조군: 가장 넓은 구간에서 같은 회전 ──────────────────────────────
# 최협 구간과 넓은 구간의 회전량이 같으면 회전이 느린 원인은 복도 폭이
# 아니라 로봇 동역학이다 (캐스터 끌림). 그 경우 이 파이프라인의 문제가 아니다.
print(f"\n[4] 대조군: 가장 넓은 구간({wide.width:.2f} m)에서 같은 회전")
place(xw, 0.0, 0.0)
spun_w = 0.0
prev_yaw = float(env._robot_yaw()[0])
hit_w = False
for i in range(600):
    act = torch.zeros(env.num_envs, 2, device=env.device)
    act[:, 1] = 1.0
    env.step(act)
    yaw = float(env._robot_yaw()[0])
    d = yaw - prev_yaw
    while d > 3.14159:
        d -= 2 * 3.14159
    while d < -3.14159:
        d += 2 * 3.14159
    spun_w += abs(d)
    prev_yaw = yaw
    if bool(env._collided[0]):
        hit_w = True
        break
print(f"    회전량 {spun_w:.2f} rad ({spun_w/6.283:.2f} 바퀴), 충돌={hit_w}")
ratio = (narrow_spin / spun_w) if spun_w > 1e-6 else 0.0
print(f"\n    최협/넓은 회전량 비 = {ratio:.2f}")
print(f"    이론 회전량 (omega_max 1.5 rad/s x 20 s) = 30.0 rad — 둘 다 크게 못 미치면")
print(f"    복도가 아니라 로봇 동역학(캐스터) 문제다.")
# 판정: 최협 구간이 넓은 구간 대비 회전을 못 하는가 (복도 탓인가) 만 본다
results["최협 구간 회전이 넓은 구간과 동등"] = (not narrow_hit) and ratio > 0.8

# ── 정리 ──────────────────────────────────────────────────────────────────
print("\n" + "=" * 68)
for k, v in results.items():
    print(f"  {'통과' if v else '실패'}  {k}")
print("=" * 68)
print("전체 통과" if all(results.values()) else "실패 항목 있음")

env.close()
simulation_app.close()

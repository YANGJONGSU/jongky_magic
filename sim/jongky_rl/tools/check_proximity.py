# 근접 패널티(S2) 배선 검증 — 같은 x 에서 중앙 vs 벽 옆에 로봇을 놓고
# 보상 차이가 reward_spec.proximity_penalty 의 예측과 일치하는지 잰다.
#
#     OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \
#       tools/check_proximity.py --headless --enable_cameras
#
# 학습을 안 걸고도 보상 항 하나를 따로 검증하는 도구다. 통과 기준:
#   1) 중앙 로봇의 근접 항 = 0 (이격 > C0)
#   2) 벽 옆 로봇의 보상이 정확히 패널티 예측만큼 낮다 (허용 오차 0.06 —
#      정지 상태 잔여 진행/회전 항의 노이즈 몫)
import argparse
import os
import sys

_p = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _p not in sys.path:
    sys.path.insert(0, _p)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import torch  # noqa: E402

import reward_spec  # noqa: E402
from jongky_map_corridor_env import JongkyMapCorridorEnv, JongkyMapCorridorEnvCfg  # noqa: E402

WALL_CLEAR = 0.06          # 벽 옆 로봇의 목표 이격 [m]

cfg = JongkyMapCorridorEnvCfg()
cfg.scene.num_envs = 2
env = JongkyMapCorridorEnv(cfg)
env.reset()

# env0 = 복도 중앙, env1 = 벽에서 WALL_CLEAR 만 띄움. x 는 리셋값 유지.
xy = env._robot_xy()
half = env._half_width_at(xy[:, 0])
y_target = torch.stack([
    torch.zeros(1, device=env.device)[0],
    half[1] - cfg.robot_half_width - WALL_CLEAR,
])

root = env._robot.data.root_state_w.clone()
root[:, 1] = env.scene.env_origins[:, 1] + y_target
env._robot.write_root_pose_to_sim(root[:, :7])
env._robot.write_root_velocity_to_sim(torch.zeros(2, 6, device=env.device))

# 순간이동으로 목표 거리가 바뀌었으니 진행 항 기준점을 다시 맞춘다.
# 안 맞추면 첫 스텝 진행 보상에 순간이동 거리가 통째로 들어간다.
env._prev_dist = torch.norm(env._goal - env._robot_xy(), dim=-1)

print("=" * 64)
ok = True
SETTLE = 4                                  # 순간이동 후 물리 정착 스텝 수.
# 첫 실행에서 step1~2 는 접촉 정착의 진행 항 노이즈가 ±0.2 까지 나왔고
# step3 부터 예측과 0.04 이내로 수렴했다 — 판정은 SETTLE 이후만 한다.
for i in range(10):
    # 정착 드리프트를 누르기 위해 매 스텝 루트 속도를 0 으로 되돌린다
    env._robot.write_root_velocity_to_sim(torch.zeros(2, 6, device=env.device))
    act = torch.zeros(2, 2, device=env.device)
    obs, rew, term, trunc, _ = env.step(act)
    clear = env._clearance()
    pen = reward_spec.proximity_penalty(clear, torch)
    diff_meas = float(rew[1] - rew[0])
    diff_pred = float(pen[1] - pen[0])
    print(f"step {i} | 이격 중앙 {clear[0]:.3f}m / 벽 {clear[1]:.3f}m "
          f"| 패널티 예측 {pen[0]:+.3f} / {pen[1]:+.3f} "
          f"| 보상 {rew[0]:+.3f} / {rew[1]:+.3f} "
          f"| 차이 실측 {diff_meas:+.3f} vs 예측 {diff_pred:+.3f} "
          f"| term={term.tolist()}")
    if i >= SETTLE:
        if abs(pen[0]) > 1e-6:
            print("  FAIL: 중앙 로봇에 근접 패널티가 붙었다")
            ok = False
        if abs(diff_meas - diff_pred) > 0.08:
            print("  FAIL: 보상 차이가 패널티 예측과 다르다")
            ok = False
        if bool(term[1]):
            print("  FAIL: 벽 옆 로봇이 충돌 판정됨 — 이격 계산이 어긋났다")
            ok = False

print("=" * 64)
print("근접 패널티 검증", "통과" if ok else "실패")
env.close()
simulation_app.close()
raise SystemExit(0 if ok else 1)

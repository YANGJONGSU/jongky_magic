# 종키 복도 env 스모크 테스트 — 인스턴스화 + 몇 스텝 굴려보기
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402

from jongky_corridor_env import JongkyCorridorEnv, JongkyCorridorEnvCfg  # noqa: E402

cfg = JongkyCorridorEnvCfg()
cfg.scene.num_envs = 4
env = JongkyCorridorEnv(cfg)

print("=" * 60)
print("obs space :", env.observation_space)
print("act space :", env.action_space)
print("num_envs  :", env.num_envs, "| device:", env.device)
print("관절      :", env._robot.joint_names)
print("바디      :", env._robot.body_names)
print("=" * 60)

obs, _ = env.reset()
print("reset obs['policy'] shape:", obs["policy"].shape, obs["policy"].dtype)

# 전진 명령을 넣고 실제로 움직이는지 본다
for i in range(30):
    act = torch.zeros(env.num_envs, 2, device=env.device)
    act[:, 0] = 1.0  # 최대 전진
    obs, rew, term, trunc, _ = env.step(act)
    if i % 10 == 0:
        xy = env._robot_xy()
        wv = env._robot.data.joint_vel[0]
        v_est = float(wv.mean()) * 0.0335  # 바퀴 각속도 → 선속도 환산
        print(
            f"step {i:3d} | x={xy[0,0]:+.3f} y={xy[0,1]:+.3f} "
            f"| rew={rew[0]:+.3f} | term={bool(term[0])} "
            f"| wheel={[round(w, 2) for w in wv.tolist()]} rad/s -> v≈{v_est:.3f} m/s "
            f"| grav_z={float(env._robot.data.projected_gravity_b[0, 2]):+.2f}"
        )

print(f"종료 카운트: term={int(term.sum())} / {env.num_envs}")

print("=" * 60)
print("스모크 테스트 통과")
env.close()
simulation_app.close()

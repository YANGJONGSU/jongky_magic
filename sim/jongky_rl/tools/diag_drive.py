# 구동 진단 — 램프/기구학을 우회하고 바퀴 속도를 직접 꽂아본다.
# 목적: "안 움직이는 게 내 액션 코드 탓인가, 물리 탓인가" 를 가른다.
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import torch  # noqa: E402

from jongky_corridor_env import JongkyCorridorEnv, JongkyCorridorEnvCfg  # noqa: E402

import os
cfg = JongkyCorridorEnvCfg()
cfg.scene.num_envs = 2
_f = float(os.environ.get("GF", "1.0"))
cfg.ground_static_friction = _f
cfg.ground_dynamic_friction = _f
print(f"### 바닥 마찰 = {_f}")
env = JongkyCorridorEnv(cfg)
env.reset()

robot = env._robot
widx = env._wheel_idx
TARGET = 11.94  # rad/s = 0.40 m/s / 0.0335

print("=" * 70)
print("바디 질량 :", robot.data.default_mass[0].tolist())
print("관절 이름 :", robot.joint_names, "| idx:", widx)
print("=" * 70)

for i in range(360):
    tgt = torch.full((env.num_envs, len(widx)), TARGET, device=env.device)
    robot.set_joint_velocity_target(tgt, joint_ids=widx)
    robot.write_data_to_sim()
    env.sim.step(render=False)
    robot.update(env.sim.get_physics_dt())

    if i % 60 == 0 or i == 359:
        p = robot.data.root_pos_w[0]
        v = robot.data.root_lin_vel_b[0]
        jv = robot.data.joint_vel[0]
        jt = robot.data.applied_torque[0]
        print(
            f"i={i:3d} | pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.4f}) "
            f"| v_body=({v[0]:+.3f},{v[1]:+.3f}) "
            f"| wheel={[round(x,2) for x in jv.tolist()]} (목표 {TARGET}) "
            f"| torque={[round(x,4) for x in jt.tolist()]}"
        )

print("=" * 70)
print("바퀴가 목표에 도달하고 pos.x 가 늘면 물리 정상 → 액션 코드 문제")
print("바퀴가 목표에 못 가면 → 드래그/마찰/토크 한계 문제")
env.close()
simulation_app.close()

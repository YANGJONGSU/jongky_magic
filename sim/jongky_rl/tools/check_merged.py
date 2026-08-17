# 병합 USD 검증 — 질량 합이 실차(2.603kg)와 맞는지, 카메라 프레임이 남았는지.
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--usd", type=str, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402

sim = SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cuda:0"))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())

cfg = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=args.usd),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.1)),
    actuators={},
)
robot = Articulation(cfg)
sim.reset()

print("=" * 70)
print("USD:", args.usd)
names = robot.body_names
masses = robot.data.default_mass[0].tolist()
total = 0.0
for n, m in zip(names, masses):
    print(f"  {n:20} {m:8.4f} kg")
    total += m
print(f"  {'합계':20} {total:8.4f} kg   (실차 2.603 kg)")
print(f"  관절: {robot.joint_names}")

print("-" * 70)
stage = get_current_stage()
for frame in ["camera_link", "rear_camera_link", "laser", "imu_link", "base_link", "base_footprint"]:
    hits = [str(p.GetPath()) for p in stage.Traverse() if p.GetName() == frame]
    print(f"  {frame:20} prim: {hits if hits else '없음'}")
print("=" * 70)

simulation_app.close()

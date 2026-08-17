# 병합 USD 의 충돌 prim 경로와 물리 머티리얼을 훑는다.
# 목적: 캐스터 구슬에 저마찰 재질을 따로 물릴 수 있는지 확인.
import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage  # noqa: E402

USD = os.environ.get("JONGKY_USD", os.path.expanduser("~/jongky_usd_merged/jongky.usd"))

SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cuda:0"))
add_reference_to_stage(USD, "/Robot")

stage = get_current_stage()
print("=" * 78)
for p in stage.Traverse():
    path, tname = str(p.GetPath()), str(p.GetTypeName())
    schemas = [s for s in p.GetAppliedSchemas() if "Physics" in s or "Collision" in s or "Material" in s]
    if tname in ("Sphere", "Cube", "Mesh") or schemas:
        print(f"{path}")
        print(f"    type={tname}  schemas={schemas}")
        if tname == "Sphere":
            r = p.GetAttribute("radius").Get()
            xf = p.GetAttribute("xformOp:translate").Get()
            print(f"    radius={r}  translate={xf}")
print("=" * 78)
simulation_app.close()

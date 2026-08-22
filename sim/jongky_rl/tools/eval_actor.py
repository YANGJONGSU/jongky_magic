# 학습된 액터를 임의 복도 형상에서 평가한다 — 성공률·충돌률·최소 이격거리.
#
#     OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \
#       tools/eval_actor.py --headless --enable_cameras \
#       --logdir ~/jongky_dreamer_runs/corridor_50k \
#       [--geometry-json corridor_L11.json] [--episodes 20] [--out eval.json]
#
# 용도 둘:
#   1) S2 판정 — 최소 이격거리 분포에서 벽 스치기 모드(0.1 m 봉우리) 소멸 확인.
#      학습 로그에는 이격이 없어서 (에피소드 npz 는 픽셀·보상뿐) 따로 재야 한다.
#   2) 11층 제로샷 — 10층에서 학습한 정책을 corridor_L11.json 형상에 놓고
#      성공률이 유지되는지 본다.
#
# acting 경로는 dreamer.py:_policy(training=False) 를 그대로 복제했다
# (actor.mode(), obs_step, eval_state_mean). 체크포인트 키의 torch.compile
# `_orig_mod.` 접두사는 dump_openl 과 같은 이유로 벗긴다.
import argparse
import json
import os
import sys

_p = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _p not in sys.path:
    sys.path.insert(0, _p)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--logdir", default="~/jongky_dreamer_runs/corridor_50k")
parser.add_argument("--dreamer-repo", default="~/dreamerv3-torch")
parser.add_argument("--geometry-json", default=None, help="기본 = env 기본값 (corridor_L10b)")
parser.add_argument("--episodes", type=int, default=20)
parser.add_argument("--out", default=None, help="결과 JSON 경로 (기본: logdir/eval_<형상>.json)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import pathlib  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402

from dump_openl import build_config  # noqa: E402  (tools/ 가 sys.path 에 있음)

from dreamer_env import get_env  # noqa: E402

repo = pathlib.Path(os.path.expanduser(args.dreamer_repo))
logdir = pathlib.Path(os.path.expanduser(args.logdir))
cfg = build_config(repo)
cfg.device = "cuda:0" if torch.cuda.is_available() else "cpu"

import models  # noqa: E402

overrides = {}
if args.geometry_json:
    overrides["geometry_json"] = os.path.abspath(args.geometry_json)
env = get_env(size=(64, 64), env_kind="map", **overrides)
isaac = env._env

wm = models.WorldModel(env.observation_space, env.action_space, 0, cfg).to(cfg.device)
behavior = models.ImagBehavior(cfg, wm).to(cfg.device)

ck = torch.load(logdir / "latest.pt", map_location=cfg.device)
state = ck["agent_state_dict"] if "agent_state_dict" in ck else ck


def sub_state(prefix):
    out = {}
    for k, v in state.items():
        if not k.startswith(prefix):
            continue
        k = k[len(prefix):]
        if k.startswith("_orig_mod."):
            k = k[len("_orig_mod."):]
        out[k] = v
    return out


for mod, prefix in ((wm, "_wm."), (behavior, "_task_behavior.")):
    missing, unexpected = mod.load_state_dict(sub_state(prefix), strict=False)
    bad = [m for m in missing if "_world_model" not in m] + list(unexpected)
    if bad:
        print("FATAL: %s state 불일치 %d건, 예: %s" % (prefix, len(bad), bad[:4]))
        raise SystemExit(1)
wm.eval()
behavior.eval()
print("가중치 로드 완료 (wm %d + behavior %d 텐서)"
      % (len(sub_state("_wm.")), len(sub_state("_task_behavior."))))


def batch(obs):
    return {k: np.asarray(v)[None] for k, v in obs.items()}


results = []
for ep in range(args.episodes):
    obs = env.reset()
    latent = action = None
    min_clear, steps, ret = np.inf, 0, 0.0
    outcome = "timeout"
    while True:
        with torch.no_grad():
            o = wm.preprocess(batch(obs))
            embed = wm.encoder(o)
            latent, _ = wm.dynamics.obs_step(latent, action, embed, o["is_first"])
            if cfg.eval_state_mean:
                latent["stoch"] = latent["mean"]
            feat = wm.dynamics.get_feat(latent)
            action = behavior.actor(feat).mode()
        # 이격은 스텝 **전** 상태에서 잰다 — 종료 스텝 뒤에는 Isaac 이 이미
        # 자동 리셋을 해서 _clearance/_collided/_goal 이 다음 에피소드 것이다.
        min_clear = min(min_clear, float(isaac._clearance()[0].item()))
        obs, rew, done, _ = env.step(action.detach().cpu().numpy()[0])
        ret += rew
        steps += 1
        if done:
            # 같은 이유로 종료 원인도 Isaac 상태가 아니라 마지막 보상으로
            # 가른다: 도달 +50, 충돌 −25 가 지배해 부호로 확실히 갈린다.
            if rew > 20.0:
                outcome = "reached"
            elif rew < -10.0:
                outcome = "collision"
                min_clear = min(min_clear, 0.0)   # 충돌 = 이격 0 도달
            elif obs["is_terminal"]:
                outcome = "terminal-other"        # 뒤집힘 등 — 나오면 조사
            break
    results.append({"outcome": outcome, "steps": steps, "return": round(ret, 2),
                    "min_clear": round(min_clear, 3)})
    print("ep %2d | %-9s | %4d스텝 | 리턴 %+7.1f | 최소이격 %.3f m"
          % (ep, outcome, steps, ret, min_clear))

n = len(results)
reached = sum(r["outcome"] == "reached" for r in results)
coll = sum(r["outcome"] == "collision" for r in results)
clears = [r["min_clear"] for r in results]
geometry = os.path.basename(getattr(isaac.cfg, "geometry_json", "") or "L10b-default")
summary = {
    "geometry": geometry, "checkpoint": str(logdir / "latest.pt"), "episodes": n,
    "success_rate": reached / n, "collision_rate": coll / n,
    "timeout_rate": (n - reached - coll) / n,
    "mean_return": round(float(np.mean([r["return"] for r in results])), 2),
    "min_clear_p10": round(float(np.percentile(clears, 10)), 3),
    "min_clear_median": round(float(np.median(clears)), 3),
    "episodes_detail": results,
}
print("=" * 64)
print("형상 %s | 성공 %.0f%% | 충돌 %.0f%% | 시간초과 %.0f%% | 평균리턴 %+.1f"
      % (geometry, 100 * summary["success_rate"], 100 * summary["collision_rate"],
         100 * summary["timeout_rate"], summary["mean_return"]))
print("최소 이격거리: p10 %.3f m · 중앙값 %.3f m  (벽 스치기 = 0.1 m 부근 몰림)"
      % (summary["min_clear_p10"], summary["min_clear_median"]))

out = args.out or str(logdir / ("eval_%s.json" % os.path.splitext(geometry)[0]))
json.dump(summary, open(os.path.expanduser(out), "w"), ensure_ascii=False, indent=1)
print("저장:", out)
env.close()
simulation_app.close()

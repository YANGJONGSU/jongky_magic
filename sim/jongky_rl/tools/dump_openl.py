"""latest.pt 에서 상상 롤아웃(open-loop) 영상을 뽑는다 — D2 산출물.

    ~/isaac/env_isaaclab/bin/python -u tools/dump_openl.py \
        --logdir ~/jongky_dreamer_runs/corridor --out ~/openl.mp4

Isaac 이 필요 없다 — 월드모델(RSSM+디코더)만 torch 로 복원해서
train_eps 의 실제 에피소드에 wm.video_pred() 를 돌린다:
컨텍스트 5프레임 이후는 잠재 공간에서만 전개해 디코딩한 영상이다.
출력 프레임은 세로로 [실제 관측 / 모델 상상 / 오차] 3단이고,
가로로 에피소드 6개가 나란히 붙는다 (dreamerv3-torch 규격 그대로).

train_dreamer.build_config 와 같은 설정을 만들어야 가중치 형상이 맞는다 —
여기의 오버라이드 목록은 train_dreamer.py 의 것을 복제한 것이다.
둘이 어긋나면 load_state_dict 가 형상 불일치로 죽는다 (조용히 틀리는
것보다 낫다).
"""
import argparse
import os
import pathlib
import sys

import numpy as np


def build_config(repo, size=(64, 64)):
    import ruamel.yaml as yaml
    sys.path.insert(0, str(repo))
    import tools as dreamer_tools
    configs = yaml.YAML(typ="safe").load((repo / "configs.yaml").read_text())
    merged = {}
    for name in ("defaults", "dmc_vision"):
        for k, v in configs[name].items():
            if isinstance(v, dict) and k in merged:
                merged[k].update(v)
            else:
                merged[k] = v
    p = argparse.ArgumentParser()
    for k, v in sorted(merged.items()):
        t = dreamer_tools.args_type(v)
        p.add_argument(f"--{k}", type=t, default=t(v))
    cfg = p.parse_args([])
    cfg.encoder["mlp_keys"] = "proprio"
    cfg.decoder["mlp_keys"] = "proprio"
    cfg.action_repeat = 1
    cfg.task = "jongky_corridor"
    cfg.envs = 1
    cfg.parallel = False
    cfg.size = list(size)
    cfg.time_limit = 600
    cfg.num_actions = 2          # dreamer.py 가 act_space 에서 뽑아 넣는 값
    return cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logdir", default="~/jongky_dreamer_runs/corridor")
    ap.add_argument("--dreamer-repo", default="~/dreamerv3-torch")
    ap.add_argument("--out", default="~/openl.mp4")
    ap.add_argument("--batch", type=int, default=6)
    ap.add_argument("--length", type=int, default=64)
    a = ap.parse_args()

    import torch
    import gym

    repo = pathlib.Path(os.path.expanduser(a.dreamer_repo))
    logdir = pathlib.Path(os.path.expanduser(a.logdir))
    cfg = build_config(repo)
    cfg.device = "cuda:0" if torch.cuda.is_available() else "cpu"

    import models

    obs_space = gym.spaces.Dict({
        "image": gym.spaces.Box(0, 255, tuple(cfg.size) + (3,), dtype=np.uint8),
        "proprio": gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32),
        "is_first": gym.spaces.Box(0, 1, (1,), dtype=np.uint8),
        "is_terminal": gym.spaces.Box(0, 1, (1,), dtype=np.uint8),
    })
    act_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
    act_space.discrete = False

    wm = models.WorldModel(obs_space, act_space, 0, cfg).to(cfg.device)
    ckpt = torch.load(logdir / "latest.pt", map_location=cfg.device)
    state = ckpt["agent_state_dict"] if "agent_state_dict" in ckpt else ckpt
    # 학습이 torch.compile 로 돌았으면 키가 _wm._orig_mod.* 다 — 접두사를
    # 둘 다 벗긴다. 하나라도 남으면 load 가 전부 빗나가 랜덤 가중치 영상이
    # 나온다 (실제로 그랬다).
    wm_state = {}
    for k, v in state.items():
        if not k.startswith("_wm."):
            continue
        k = k[len("_wm."):]
        if k.startswith("_orig_mod."):
            k = k[len("_orig_mod."):]
        wm_state[k] = v
    missing, unexpected = wm.load_state_dict(wm_state, strict=False)
    if missing or unexpected:
        print("FATAL: state dict 불일치 — missing %d unexpected %d" % (len(missing), len(unexpected)))
        for m in (list(missing) + list(unexpected))[:6]:
            print("   ", m)
        raise SystemExit(1)
    print("가중치 로드 완료: %d 텐서" % len(wm_state))
    wm.eval()

    # 최근 에피소드에서 배치 구성 (초반 랜덤보다 주행다운 궤적).
    # length 미만 짧은 에피소드(목표가 코앞에 스폰된 경우 등)는 stack 이
    # 안 되므로 거른다.
    import glob
    all_eps = sorted(glob.glob(str(logdir / "train_eps" / "*.npz")))
    eps = [f for f in all_eps if len(np.load(f)["reward"]) >= a.length][-a.batch:]
    if len(eps) < a.batch:
        raise SystemExit("길이 %d 이상 에피소드가 %d개뿐" % (a.length, len(eps)))
    keys = ["image", "proprio", "action", "reward", "discount", "is_first", "is_terminal"]
    data = {k: [] for k in keys}
    for f in eps:
        ep = np.load(f)
        T = len(ep["reward"])
        s = max(0, (T - a.length) // 2)       # 가운데 토막 — 벽·사물함이 보이는 구간
        for k in keys:
            data[k].append(ep[k][s:s + a.length])
    data = {k: np.stack(v) for k, v in data.items()}
    data["cont"] = 1.0 - data["is_terminal"].astype(np.float32)

    with torch.no_grad():
        openl = wm.video_pred({k: torch.tensor(v) for k, v in data.items()})
    arr = openl.detach().cpu().numpy()        # (B, T, H*3, W, C) 또는 유사
    print("video_pred 출력:", arr.shape, arr.dtype, "범위 %.2f~%.2f" % (arr.min(), arr.max()))

    # (B, T, H, W, C) → 배치를 가로로 붙여 (T, H, B*W, C)
    if arr.ndim == 5:
        frames = np.concatenate(list(arr.transpose(0, 1, 2, 3, 4)), axis=2)
    else:
        frames = arr
    frames = np.clip(frames * 255 if frames.max() <= 1.5 else frames, 0, 255).astype(np.uint8)

    out = os.path.expanduser(a.out)
    try:
        import cv2
        h, w = frames.shape[1:3]
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), 10, (w, h))
        for f in frames:
            vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        vw.release()
        print("저장:", out, frames.shape)
    except ImportError:
        alt = os.path.splitext(out)[0] + ".npy"
        np.save(alt, frames)
        print("cv2 없음 — npy 로 저장:", alt)


if __name__ == "__main__":
    main()

"""실물 리플레이로 월드모델 미세조정 — D3 전반부 (로봇 없이 되는 부분).

    ~/isaac/env_isaaclab/bin/python -u tools/finetune_real.py \
        --logdir ~/jongky_dreamer_runs/corridor_50k \
        --episodes ~/labels_out/episodes [--steps 300]

프로토콜:
  1) 실물 에피소드 27개 중 5개 홀드아웃 (episode 이름 정렬 후 [::6] 앞 5개)
  2) 제로샷: corridor_50k 월드모델의 홀드아웃 개방루프(컨텍스트 5프레임 뒤
     액션만으로 전개) 이미지 MSE 측정
  3) 나머지 22개로 wm._train 을 오프라인으로 N 스텝 (배치 16×길이 64)
  4) 같은 홀드아웃에서 MSE 재측정 + 전/후 openl 영상 저장

측정하는 주장: "시뮬 사전학습 모델이 실물 리플레이 미세조정으로 실물
동역학을 빨아들인다" — C3 의 시뮬 밖 첫 증거. 액터는 여기서 안 건드린다
(액터 개선의 평가는 실차가 필요하다 — 그건 D3 후반).

Isaac 불필요 (torch 만). 체크포인트 키의 _orig_mod. 접두사는 dump_openl 과
동일하게 벗긴다.
"""
import argparse
import glob
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_openl import build_config  # noqa: E402

KEYS = ["image", "proprio", "action", "reward", "discount", "is_first", "is_terminal"]


def load_eps(files, length):
    """에피소드 → 길이 length 창 목록 (dict of np arrays)."""
    wins = []
    for f in files:
        ep = np.load(f)
        T = len(ep["reward"])
        for s in range(0, T - length + 1, length):
            w = {k: ep[k][s:s + length] for k in KEYS}
            # 창 시작을 에피소드 시작처럼 다룬다 — 잠재 초기화 기준점
            w["is_first"] = w["is_first"].copy()
            w["is_first"][0] = True
            wins.append(w)
    return wins


def batchify(wins, idxs):
    return {k: np.stack([wins[i][k] for i in idxs]) for k in KEYS}


def openl_mse(wm, data, torch):
    """video_pred 와 같은 전개(컨텍스트 5 + 액션 조건 상상)의 이미지 MSE."""
    with torch.no_grad():
        d = wm.preprocess(data)
        embed = wm.encoder(d)
        B = d["image"].shape[0]
        states, _ = wm.dynamics.observe(
            embed[:, :5], d["action"][:, :5], d["is_first"][:, :5])
        init = {k: v[:, -1] for k, v in states.items()}
        prior = wm.dynamics.imagine_with_action(d["action"][:, 5:], init)
        openl = wm.heads["decoder"](wm.dynamics.get_feat(prior))["image"].mode()
        truth = d["image"][:, 5:]
        mse = ((openl - truth) ** 2).mean().item()
    return mse, openl, truth


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logdir", default="~/jongky_dreamer_runs/corridor_50k")
    ap.add_argument("--dreamer-repo", default="~/dreamerv3-torch")
    ap.add_argument("--episodes", default="~/labels_out/episodes")
    ap.add_argument("--steps", type=int, default=300)
    ap.add_argument("--length", type=int, default=64)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--out", default="~/jongky_dreamer_runs/corridor_ft_real")
    a = ap.parse_args()

    import torch
    import gym

    repo = pathlib.Path(os.path.expanduser(a.dreamer_repo))
    cfg = build_config(repo)
    cfg.device = "cuda:0" if torch.cuda.is_available() else "cpu"
    import models

    obs_space = gym.spaces.Dict({
        "image": gym.spaces.Box(0, 255, (64, 64, 3), dtype=np.uint8),
        "proprio": gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32),
        "is_first": gym.spaces.Box(0, 1, (1,), dtype=np.uint8),
        "is_terminal": gym.spaces.Box(0, 1, (1,), dtype=np.uint8),
    })
    act_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
    act_space.discrete = False
    wm = models.WorldModel(obs_space, act_space, 0, cfg).to(cfg.device)

    ck = torch.load(pathlib.Path(os.path.expanduser(a.logdir)) / "latest.pt",
                    map_location=cfg.device)
    state = ck["agent_state_dict"] if "agent_state_dict" in ck else ck
    wm_state = {}
    for k, v in state.items():
        if not k.startswith("_wm."):
            continue
        k = k[len("_wm."):]
        if k.startswith("_orig_mod."):
            k = k[len("_orig_mod."):]
        wm_state[k] = v
    missing, unexpected = wm.load_state_dict(wm_state, strict=False)
    assert not missing and not unexpected, (missing, unexpected)
    print("가중치 로드: %d 텐서" % len(wm_state))

    files = sorted(glob.glob(os.path.join(os.path.expanduser(a.episodes), "*.npz")))
    held = files[::6][:5]
    train = [f for f in files if f not in held]
    print("에피소드: 학습 %d · 홀드아웃 %d" % (len(train), len(held)))
    train_w = load_eps(train, a.length)
    held_w = load_eps(held, a.length)
    print("창(%d프레임): 학습 %d · 홀드아웃 %d" % (a.length, len(train_w), len(held_w)))

    rng = np.random.default_rng(0)
    eval_idx = list(range(min(16, len(held_w))))
    eval_batch = batchify(held_w, eval_idx)

    mse0, openl0, truth0 = openl_mse(wm, eval_batch, torch)
    print("제로샷 개방루프 MSE (홀드아웃): %.5f" % mse0)

    wm.train()
    for i in range(a.steps):
        idxs = rng.integers(0, len(train_w), a.batch)
        post, context, metrics = wm._train(batchify(train_w, idxs))
        if i % 50 == 0 or i == a.steps - 1:
            print("step %3d | image_loss %.1f | reward_loss %.3f | kl %.2f"
                  % (i, metrics["image_loss"], metrics["reward_loss"], metrics["kl"]))
    wm.eval()

    mse1, openl1, _ = openl_mse(wm, eval_batch, torch)
    print("미세조정 후 MSE (같은 홀드아웃): %.5f  (개선 %.1f%%)"
          % (mse1, 100 * (mse0 - mse1) / mse0))

    out = pathlib.Path(os.path.expanduser(a.out))
    out.mkdir(parents=True, exist_ok=True)
    torch.save({"wm_state_dict": wm.state_dict(),
                "mse_zero_shot": mse0, "mse_finetuned": mse1,
                "train_eps": train, "held_eps": held}, out / "wm_ft.pt")

    # 전/후 openl 영상 (홀드아웃 6개 창, 진실/전/후 3단)
    import cv2
    tr = truth0[:6].cpu().numpy()
    o0 = openl0[:6].cpu().numpy()
    o1 = openl1[:6].cpu().numpy()
    strip = np.concatenate([tr, o0, o1], axis=2)          # (6, T, 3H, W, C)
    frames = np.concatenate(list(strip), axis=2)          # (T, 3H, 6W, C)
    frames = np.clip(frames * 255, 0, 255).astype(np.uint8)
    h, w = frames.shape[1:3]
    vw = cv2.VideoWriter(str(out / "openl_real_before_after.mp4"),
                         cv2.VideoWriter_fourcc(*"mp4v"), 10, (w, h))
    for f in frames:
        vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
    vw.release()
    print("저장:", out / "wm_ft.pt", "·", out / "openl_real_before_after.mp4",
          "(상=실물, 중=제로샷 상상, 하=미세조정 상상)")


if __name__ == "__main__":
    main()

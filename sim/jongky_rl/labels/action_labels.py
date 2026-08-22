#!/usr/bin/env python3
"""Cosmos 클립에 액션 라벨을 붙인다.

두 모드가 있고 성격이 완전히 다르다.

## seed — 진짜 라벨 (기본)

씨앗 클립은 bag 의 실주행 구간이다 (make_seed clip --at N). 그 구간의
/cmd_vel·odom·scan 을 프레임 시각마다 뽑으면 **실측 액션·proprio·보상**이
된다. 씨앗과 프레임 단위로 정렬된 변형본(갈래 A 리스타일)은 이 라벨을
그대로 물려받는다. 현재 배치(연속 생성)는 앞 aligned_frames(9)장만
물려받을 수 있고, 그 뒤는 모델이 지어낸 궤적이라 이 라벨이 **없다.**

    python3 action_labels.py seed --index /root/labels_out/index.json \
        --bag CAMERA_BAG.mcap --floor 10f --out /root/labels_out/actions

정렬 규칙: make_seed 의 cmd_clip 은 --at 이후의 **유효 프레임을 연속으로**
받아쓴다 (30Hz 원 스트림, 서브샘플 없음. 잘린 프레임은 건너뜀). 따라서
씨앗 프레임 k 의 시각 = --at 이후 k번째 유효 이미지의 log_time 이고,
여기서도 같은 규칙으로 센다.

## idm — 유사 라벨 (실험, 게이트 필수)

생성 구간에는 액션이 존재하지 않는다. 역동역학 모델(IDM)로 프레임 쌍에서
액션을 추정해 붙이는 길이 있으나, 이는 월드모델-설계 문서의 "갈래 C 금지"
(지어낸 액션 금지) 경계에 서 있는 방법이다. 그래서

  1) 실주행 bag 의 (프레임쌍 → cmd_vel) 로만 학습하고,
  2) 떼어 둔 실주행 검증셋에서 MAE 가 게이트(v 0.05 m/s · ω 0.15 rad/s)를
     넘으면 **라벨을 내보내지 않는다.**
  3) 내보낸 라벨에는 action_source="idm" 이 박혀서, 버퍼 적재 단계
     (pack_episodes.py)가 기본 설정에서 거부한다. 실험으로 넣을 때만
     --allow-idm 을 명시한다.

    python3 action_labels.py idm-train --bag CAMERA_BAG.mcap --out idm.pt
    python3 action_labels.py idm-apply --model idm.pt --clip GEN.mp4 --out ...

주의: idm-* 는 카메라 bag 이 이 기계에 없어 아직 실행해 본 적이 없다.
seed 모드도 같은 이유로 미실행 — bag 확보 후 첫 실행에서 프레임 수가
씨앗 mp4 와 일치하는지부터 확인할 것 (아래 검사 내장).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import reward_spec as R  # noqa: E402

import mcap_lite as M  # noqa: E402
from relabel_bag import load_streams, relabel_segment  # noqa: E402

SECONDS = 2.0        # 씨앗 실측: 60프레임 @30fps = 2초 (mp4 로 확인, 8/22)
IDM_GATE_V = 0.05    # [m/s] 검증 MAE 게이트
IDM_GATE_W = 0.15    # [rad/s]


def seed_frame_times(bag, image_topic, at_list, seconds):
    """--at 지점들의 유효 프레임 log_time 목록. make_seed 와 같은 규칙."""
    windows = {at: [] for at in at_list}
    t0 = None
    for topic, lt, d in M.iter_messages(bag, {image_topic}):
        t_abs = lt / 1e9
        if t0 is None:
            t0 = t_abs
        t = t_abs - t0                      # make_seed 는 bag 상대시각을 쓴다
        _, im, _ = M.decode_image(d)
        if im is None:
            continue                        # 잘린 프레임 — make_seed 도 버린다
        for at in at_list:
            w = windows[at]
            if t >= at and len(w) < int(seconds * 30):
                w.append(t_abs)
    return windows


def cmd_seed(a):
    idx = json.load(open(a.index))
    seeds = sorted({(r["seed_file"], r["bag_at_s"]) for r in idx
                    if r["floor"] == a.floor and r["bag_at_s"] is not None})
    if not seeds:
        sys.exit("인덱스에 %s 씨앗이 없다" % a.floor)
    at_list = [at for _, at in seeds]
    print("씨앗 %d개 (bag 오프셋: %s)" % (len(seeds), at_list))

    frames = seed_frame_times(a.bag, a.image_topic, at_list, a.seconds)
    cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear, _, _ = load_streams(
        a.bag, None, a.odom_topic, a.cmd_topic, a.scan_topic)

    os.makedirs(a.out, exist_ok=True)
    for (seed_file, at) in seeds:
        ft = np.array(frames[at])
        expect = int(a.seconds * 30)
        if len(ft) < expect:
            print("경고 %s: 프레임 %d < 기대 %d — bag 이 짧거나 잘린 프레임 과다"
                  % (seed_file, len(ft), expect))
        if len(ft) < 2:
            continue
        act, prop, rew, term, dist, diag = relabel_segment(
            ft, cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear, fragment=True)
        out = os.path.join(a.out, seed_file.replace(".mp4", ".npz"))
        np.savez_compressed(out, t=ft[:len(rew)], action=act, proprio=prop,
                            reward=rew, is_terminal=term)
        print("%s: %d프레임 |v̄|=%.2f |ω̄|=%.2f 이격최소 %.2fm → %s"
              % (seed_file, len(rew),
                 float(np.abs(act[:, 0] * R.V_MAX).mean()),
                 float(np.abs(act[:, 1] * R.OMEGA_MAX).mean()),
                 diag["min_clear"], out))


# ── IDM (실험) ────────────────────────────────────────────────────────────

def _idm_model():
    import torch.nn as nn
    # 입력: 프레임 2장 concat (6, 64, 64) → [v_norm, ω_norm]
    return nn.Sequential(
        nn.Conv2d(6, 32, 4, 2, 1), nn.ReLU(),
        nn.Conv2d(32, 64, 4, 2, 1), nn.ReLU(),
        nn.Conv2d(64, 128, 4, 2, 1), nn.ReLU(),
        nn.Conv2d(128, 128, 4, 2, 1), nn.ReLU(),
        nn.Flatten(), nn.Linear(128 * 4 * 4, 128), nn.ReLU(),
        nn.Linear(128, 2), nn.Tanh(),
    )


def cmd_idm_train(a):
    import torch
    cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear, img_t, imgs = load_streams(
        a.bag, a.image_topic, a.odom_topic, a.cmd_topic, a.scan_topic)
    if len(imgs) < 100:
        sys.exit("이미지가 %d장뿐 — 카메라 토픽 확인: %s" % (len(imgs), a.image_topic))
    from relabel_bag import hold
    X, Y = [], []
    # 30Hz 전량이면 25분 bag 에서 X 가 4GB를 넘는다 — stride 로 솎는다
    for i in range(0, len(imgs) - 1, a.stride):
        v, w = hold(cmd_t, cmd_v, img_t[i])
        X.append(np.concatenate([imgs[i], imgs[i + 1]], axis=-1))
        Y.append((np.clip(v / R.V_MAX, -1, 1), np.clip(w / R.OMEGA_MAX, -1, 1)))
    X = torch.tensor(np.array(X), dtype=torch.float32).permute(0, 3, 1, 2) / 255.0
    Y = torch.tensor(np.array(Y), dtype=torch.float32)
    n = len(X)
    n_val = max(n // 10, 1)
    # 시간순 뒤 10% 를 검증으로 — 무작위 분할은 인접 프레임 누수로 낙관된다
    Xtr, Ytr, Xva, Yva = X[:-n_val], Y[:-n_val], X[-n_val:], Y[-n_val:]

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    m = _idm_model().to(dev)
    opt = torch.optim.Adam(m.parameters(), lr=3e-4)
    for ep in range(a.epochs):
        perm = torch.randperm(len(Xtr))
        tot = 0.0
        for j in range(0, len(Xtr), 64):
            b = perm[j:j + 64]
            loss = torch.nn.functional.mse_loss(m(Xtr[b].to(dev)), Ytr[b].to(dev))
            opt.zero_grad(); loss.backward(); opt.step()
            tot += loss.item() * len(b)
        with torch.no_grad():
            pv = m(Xva.to(dev)).cpu()
        mae = (pv - Yva).abs().mean(0)
        mae_v = float(mae[0]) * R.V_MAX
        mae_w = float(mae[1]) * R.OMEGA_MAX
        print("epoch %d: train %.4f · val MAE v %.3f m/s · ω %.3f rad/s"
              % (ep, tot / len(Xtr), mae_v, mae_w))
    ok = mae_v <= IDM_GATE_V and mae_w <= IDM_GATE_W
    torch.save({"state": m.state_dict(), "mae_v": mae_v, "mae_w": mae_w,
                "gate_ok": ok}, a.out)
    print("게이트 %s (v %.3f<=%.2f, ω %.3f<=%.2f) → %s"
          % ("통과" if ok else "**탈락 — idm-apply 가 거부한다**",
             mae_v, IDM_GATE_V, mae_w, IDM_GATE_W, a.out))


def cmd_idm_apply(a):
    import cv2
    import torch
    ck = torch.load(a.model, map_location="cpu")
    if not ck.get("gate_ok"):
        sys.exit("이 IDM 은 검증 게이트를 못 넘었다 (v MAE %.3f · ω MAE %.3f). "
                 "라벨을 내보내지 않는다." % (ck["mae_v"], ck["mae_w"]))
    m = _idm_model()
    m.load_state_dict(ck["state"])
    m.eval()
    cap = cv2.VideoCapture(a.clip)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2RGB), (64, 64),
                                 interpolation=cv2.INTER_AREA))
    if len(frames) < 2:
        sys.exit("프레임 부족: %s" % a.clip)
    X = torch.tensor(np.array([np.concatenate([frames[i], frames[i + 1]], axis=-1)
                               for i in range(len(frames) - 1)]),
                     dtype=torch.float32).permute(0, 3, 1, 2) / 255.0
    with torch.no_grad():
        act = m(X).numpy()
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    np.savez_compressed(a.out, action=act.astype(np.float32),
                        action_source="idm",
                        idm_mae_v=ck["mae_v"], idm_mae_w=ck["mae_w"])
    print("%s: %d프레임 → %s (action_source=idm — pack 단계 기본 거부 대상)"
          % (os.path.basename(a.clip), len(act), a.out))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(required=True)

    s = sub.add_parser("seed", help="씨앗 구간의 실측 라벨")
    s.add_argument("--index", default="/root/labels_out/index.json")
    s.add_argument("--bag", required=True, help="씨앗을 뽑았던 카메라 bag")
    s.add_argument("--floor", required=True, choices=["10f", "11f"])
    s.add_argument("--image-topic", default="/camera/rgb/image_raw")
    s.add_argument("--odom-topic", default="/odom")
    s.add_argument("--cmd-topic", default="/cmd_vel")
    s.add_argument("--scan-topic", default="/scan")
    s.add_argument("--seconds", type=float, default=SECONDS)
    s.add_argument("--out", default="/root/labels_out/actions")
    s.set_defaults(func=cmd_seed)

    t = sub.add_parser("idm-train", help="실주행 bag 으로 역동역학 학습 (실험)")
    t.add_argument("--bag", required=True)
    t.add_argument("--image-topic", default="/camera/rgb/image_raw")
    t.add_argument("--odom-topic", default="/odom")
    t.add_argument("--cmd-topic", default="/cmd_vel")
    t.add_argument("--scan-topic", default="/scan")
    t.add_argument("--epochs", type=int, default=10)
    t.add_argument("--stride", type=int, default=3, help="프레임쌍 표본 간격 (메모리)")
    t.add_argument("--out", default="/root/labels_out/idm.pt")
    t.set_defaults(func=cmd_idm_train)

    p = sub.add_parser("idm-apply", help="생성 클립에 유사 액션 (게이트 통과 시만)")
    p.add_argument("--model", required=True)
    p.add_argument("--clip", required=True)
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_idm_apply)

    a = ap.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""라벨이 붙은 클립을 DreamerV3 리플레이 에피소드(.npz)로 적재한다.

    python3 pack_episodes.py --index /root/labels_out/index.json \
        --cosmos /root/cosmos_out --actions /root/labels_out/actions \
        --out /root/labels_out/episodes

## 적재 자격 — 여기가 이 파일의 존재 이유다

월드모델 버퍼에는 "그 전이를 실제로 일으킨 액션" 이 있는 시퀀스만 들어간다
(월드모델-설계 7절 갈래 A/B/C). 클립별 자격은 index.json 의 필드로 정한다.

    branch=A + action_source=seed  → 적재 (리스타일 — 픽셀만 다르고 궤적 실측)
    branch=B                       → 거부. 인코더 강건화 경로로 갈 것
    action_source=idm              → 기본 거부. --allow-idm 일 때만,
                                     provenance 에 idm 표기를 남기고 적재
    action_source=none             → 거부

현재 배치는 전부 branch=B 라 이 도구가 적재할 클립이 아직 없다.
리스타일(A) 배치가 나오면 index 의 branch 를 A 로 올리고 여기로 통과시킨다.

에피소드 키는 dreamer_env.py 관측 규격과 같다:
    image (T,64,64,3 u8) · proprio (T,2 f32) · action (T,2 f32)
    reward (T f32) · is_first/is_last/is_terminal (T bool)

    python3 pack_episodes.py --selftest    # 합성 데이터로 포맷 검증
"""
import argparse
import json
import os
import sys

import numpy as np


def pack(images, actions, proprio, reward, is_terminal, out_path, provenance):
    T = len(images)
    for name, arr in [("action", actions), ("proprio", proprio),
                      ("reward", reward), ("is_terminal", is_terminal)]:
        if len(arr) != T:
            sys.exit("길이 불일치: image %d vs %s %d — 정렬이 깨졌다" % (T, name, len(arr)))
    is_first = np.zeros(T, dtype=bool)
    is_first[0] = True
    is_last = np.zeros(T, dtype=bool)
    is_last[-1] = True
    term = np.asarray(is_terminal, dtype=bool)
    np.savez_compressed(
        out_path,
        image=np.asarray(images, dtype=np.uint8),
        proprio=np.asarray(proprio, dtype=np.float32),
        action=np.asarray(actions, dtype=np.float32),
        reward=np.asarray(reward, dtype=np.float32),
        is_first=is_first, is_last=is_last,
        is_terminal=term,
        # dreamer 가 자기 에피소드에 저장하는 두 키. 오프라인 에피소드도
        # 같은 키를 갖춰야 load_episodes 가 구분 없이 먹는다 (8/21 스모크
        # train_eps 실물 npz 와 대조해 확정한 규격).
        discount=(1.0 - term.astype(np.float32)),
        logprob=np.zeros(T, dtype=np.float32),
    )
    json.dump(provenance, open(os.path.splitext(out_path)[0] + ".json", "w"),
              ensure_ascii=False)


def clip_frames(path):
    import cv2
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2RGB), (64, 64),
                                 interpolation=cv2.INTER_AREA))
    return frames


def selftest():
    import tempfile
    T = 32
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "ep.npz")
        pack(np.zeros((T, 64, 64, 3)), np.zeros((T, 2)), np.zeros((T, 2)),
             np.zeros(T), np.zeros(T, dtype=bool), p, {"src": "selftest"})
        ep = np.load(p)
        assert ep["image"].dtype == np.uint8 and ep["image"].shape == (T, 64, 64, 3)
        assert ep["action"].dtype == np.float32 and ep["action"].shape == (T, 2)
        assert ep["proprio"].shape == (T, 2)
        assert bool(ep["is_first"][0]) and bool(ep["is_last"][-1])
        assert ep["reward"].dtype == np.float32
    print("selftest OK — 에피소드 키·dtype·형상 규격 일치")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", default="/root/labels_out/index.json")
    ap.add_argument("--cosmos", default="/root/cosmos_out")
    ap.add_argument("--actions", default="/root/labels_out/actions")
    ap.add_argument("--out", default="/root/labels_out/episodes")
    ap.add_argument("--allow-idm", action="store_true",
                    help="IDM 유사 라벨 클립도 적재 (실험 — provenance 에 남는다)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return

    idx = json.load(open(a.index))
    os.makedirs(a.out, exist_ok=True)
    packed = 0
    reasons = {}
    for r in idx:
        why = None
        if r["qc"] == "exclude":
            why = "qc-exclude"
        elif r.get("branch") != "A":
            why = "branch!=A (인코더 강건화 경로)"
        elif r.get("action_source") == "idm" and not a.allow_idm:
            why = "idm 라벨 (--allow-idm 없이는 거부)"
        elif r.get("action_source") not in ("seed", "idm"):
            why = "액션 라벨 없음"
        if why:
            reasons[why] = reasons.get(why, 0) + 1
            continue

        lab = np.load(os.path.join(a.actions,
                                   r["seed_file"].replace(".mp4", ".npz")))
        frames = clip_frames(os.path.join(a.cosmos, "Cosmos1GP", "outputs", r["clip"]))
        T = min(len(frames), len(lab["reward"]))
        tag = os.path.splitext(r["clip"])[0][:80]
        pack(frames[:T], lab["action"][:T], lab["proprio"][:T],
             lab["reward"][:T], lab["is_terminal"][:T],
             os.path.join(a.out, tag + ".npz"),
             {"clip": r["clip"], "seed": r["seed_file"], "cond": r["cond"],
              "action_source": r["action_source"], "branch": r["branch"]})
        packed += 1
    print("적재 %d개 · 거부 사유: %s" % (packed, reasons or "없음"))


if __name__ == "__main__":
    main()

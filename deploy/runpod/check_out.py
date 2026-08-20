#!/usr/bin/env python3
"""생성된 클립을 씨앗과 나란히 붙여서 한 장으로 만든다.

    cd /workspace/Cosmos1GP
    python3 /workspace/jongky_magic/deploy/runpod/check_out.py --n 3

## 무엇을 보려는 것인가

Cosmos 는 생성 모델이라 기하를 보장하지 않는다. 복도 폭·벽 위치·카메라 높이가
바뀌면 depth 와 액션의 관계가 깨져서 "이 상황에서 전진해도 안전하다" 가 거짓
신호가 된다. 그러면 그 클립은 학습에 쓸 수 없다.

그래서 씨앗의 첫 프레임과 결과의 첫·중간·끝 프레임을 한 줄로 붙인다.
왼쪽에서 오른쪽으로 보면서 벽이 제자리에 있는지, 바닥선의 기울기가 같은지,
주문한 물건이 실제로 나왔는지를 한눈에 본다.
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np


def frames(path, idxs):
    cap = cv2.VideoCapture(path)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    out = []
    for f in idxs:
        cap.set(cv2.CAP_PROP_POS_FRAMES, min(int(f * (n - 1)), n - 1))
        ok, im = cap.read()
        out.append(im if ok else None)
    cap.release()
    return out


def label(im, txt, w):
    h = int(im.shape[0] * w / im.shape[1])
    im = cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA)
    pad = 28
    o = np.full((h + pad, w, 3), 30, np.uint8)
    o[pad:] = im
    cv2.putText(o, txt, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (230, 230, 230), 1, cv2.LINE_AA)
    return o


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--manifest", default="/workspace/batch_manifest.jsonl")
    p.add_argument("--seeds-dir", default="/workspace/jongky_magic/deploy/runpod/seeds")
    p.add_argument("--outputs", default="outputs")
    p.add_argument("--n", type=int, default=3, help="최근 몇 개를 볼지")
    p.add_argument("--out", default="/workspace/check.jpg")
    p.add_argument("--width", type=int, default=380)
    a = p.parse_args()

    recs = []
    if os.path.isfile(a.manifest):
        for ln in open(a.manifest, encoding="utf-8"):
            try:
                r = json.loads(ln)
            except ValueError:
                continue
            if r.get("ok") and r.get("out"):
                recs.append(r)
    if not recs:
        raise SystemExit("아직 완료된 클립이 없다: %s" % a.manifest)

    rows = []
    for r in recs[-a.n:]:
        seed = os.path.join(a.seeds_dir, r["seed_file"])
        gen = os.path.join(a.outputs, r["out"][0])
        if not (os.path.isfile(seed) and os.path.isfile(gen)):
            print("건너뜀:", r["seed_file"]); continue
        s0 = frames(seed, [0.0])[0]
        g0, g1, g2 = frames(gen, [0.0, 0.5, 1.0])
        cells = [label(s0, "SEED  " + r["seed_file"].replace("corridor_", ""), a.width)]
        for im, t in ((g0, "생성 0%"), (g1, "생성 50%"), (g2, "생성 100%")):
            if im is None:
                continue
            cells.append(label(im, "%s   %s" % (t, r["cond"].replace("cond_", "").replace(".txt", "")),
                               a.width))
        h = max(c.shape[0] for c in cells)
        cells = [np.vstack([c, np.full((h - c.shape[0], c.shape[1], 3), 30, np.uint8)])
                 if c.shape[0] < h else c for c in cells]
        rows.append(np.hstack(cells))

    if not rows:
        raise SystemExit("붙일 게 없다")
    w = max(r.shape[1] for r in rows)
    rows = [np.hstack([r, np.full((r.shape[0], w - r.shape[1], 3), 30, np.uint8)])
            if r.shape[1] < w else r for r in rows]
    img = np.vstack(rows)
    cv2.imwrite(a.out, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print("%s  %dx%d  (%d개)" % (a.out, img.shape[1], img.shape[0], len(rows)))
    print("RunPod 콘솔에서 파일 내려받거나, 아래로 로컬에 가져올 것:")
    print("  scp <파드주소>:%s ." % a.out)


if __name__ == "__main__":
    main()

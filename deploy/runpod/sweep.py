#!/usr/bin/env python3
"""한 씨앗에 여러 조합을 돌려서 한 장으로 비교한다.

    cd /workspace/Cosmos1GP
    python3 ../jongky_magic/deploy/runpod/sweep.py

## 왜 이게 필요한가

한 판 돌리고 보고 고치고 또 한 판 돌리면, 판단 하나에 9분씩 든다. 조합을
세 개만 비교해도 반나절이 간다. 어차피 GPU 는 순차로 도니 **한 번에 걸어
두고 결과만 한 장으로 보는 것**이 같은 시간에 훨씬 많이 알려준다.

기본 조합은 프롬프트 2종 × guidance 2종이다. 무엇이 문제인지 모를 때
"프롬프트 구조 때문인가, 추종 강도 때문인가" 를 한 번에 가른다.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

from gradio_client import Client, handle_file

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_batch import NEG          # 같은 네거티브를 쓴다


def load(path):
    return re.sub(r"\s+", " ", open(path, encoding="utf-8").read()).strip()


def main():
    p = argparse.ArgumentParser(description="프롬프트·설정 조합 비교")
    p.add_argument("--seed-video",
                   default=os.path.join(HERE, "seeds", "corridor_10f_0330.mp4"))
    p.add_argument("--prompts", nargs="+",
                   default=[os.path.join(HERE, "seeds", "cond_1_low_obstacles.txt")])
    p.add_argument("--guidance", type=float, nargs="+", default=[7.0, 11.0])
    p.add_argument("--steps", type=int, nargs="+", default=[20])
    p.add_argument("--resolution", default="832x624")
    p.add_argument("--length", type=int, default=121)
    p.add_argument("--max-frames", type=int, default=9)
    p.add_argument("--url", default="http://localhost:7860")
    p.add_argument("--outputs", default="/workspace/Cosmos1GP/outputs")
    p.add_argument("--out", default="/workspace/sweep.jpg")
    p.add_argument("--frames", type=int, default=6, help="비교 시트에 넣을 프레임 수")
    a = p.parse_args()

    combos = [(pr, g, st) for pr in a.prompts for g in a.guidance for st in a.steps]
    est = sum(8.7 * (st / 20) for _, _, st in combos)
    print("조합 %d개 · 씨앗 %s · 예상 %.0f분"
          % (len(combos), os.path.basename(a.seed_video), est), flush=True)
    for pr, g, st in combos:
        print("   %-28s g=%.1f steps=%d" % (os.path.basename(pr), g, st), flush=True)

    c = Client(a.url, verbose=False)
    rows = []
    for i, (pr, g, st) in enumerate(combos, 1):
        before = set(glob.glob(os.path.join(a.outputs, "*.mp4")))
        t0 = time.time()
        tag = "%s g%.0f s%d" % (
            os.path.basename(pr).replace("cond_", "").replace(".txt", ""), g, st)
        print("\n[%d/%d] %s" % (i, len(combos), tag), flush=True)
        try:
            c.predict(
                prompt=load(pr), neg_prompt=NEG, resolution=a.resolution,
                video_length=a.length, seed=42, num_inference_steps=st,
                embedded_guidance_scale=g, repeat_generation=1, tea_cache=0,
                image_to_continue=None,
                video_to_continue=handle_file(os.path.abspath(a.seed_video)),
                max_frames=a.max_frames, api_name="/generate_video",
            )
        except Exception as e:
            print("      실패: %s: %s" % (type(e).__name__, e), flush=True)
            continue
        new = sorted(set(glob.glob(os.path.join(a.outputs, "*.mp4"))) - before,
                     key=os.path.getmtime)
        if not new:
            print("      새 파일이 없다", flush=True); continue
        rows.append((tag, new[-1]))
        print("      %.1f분 · %s" % ((time.time() - t0) / 60,
                                     os.path.basename(new[-1])), flush=True)

    if not rows:
        raise SystemExit("만들어진 게 없다")

    # 비교 시트는 check_out.py 를 재사용하지 않고 여기서 직접 만든다 —
    # manifest 가 아니라 이 실행에서 나온 목록으로 짝지어야 하기 때문이다.
    import cv2
    import numpy as np

    def grab(path, n):
        cap = cv2.VideoCapture(path)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
        out = []
        for k in range(n):
            cap.set(cv2.CAP_PROP_POS_FRAMES, min(int(k / max(n - 1, 1) * (total - 1)),
                                                 total - 1))
            ok, im = cap.read()
            if ok:
                out.append(im)
        cap.release()
        return out

    W = 250
    sheet = []
    seed_im = grab(a.seed_video, 1)
    for tag, path in rows:
        cells = []
        for k, im in enumerate([seed_im[0]] + grab(path, a.frames) if seed_im else grab(path, a.frames)):
            h = int(im.shape[0] * W / im.shape[1])
            im = cv2.resize(im, (W, h), interpolation=cv2.INTER_AREA)
            cell = np.full((h + 26, W, 3), 30, np.uint8)
            cell[26:] = im
            label = "SEED" if (k == 0 and seed_im) else "%3d%%" % round(
                100 * (k - 1) / max(a.frames - 1, 1))
            cv2.putText(cell, "%s  %s" % (label, tag if k <= 1 else ""), (5, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (225, 225, 225), 1, cv2.LINE_AA)
            cells.append(cell)
        sheet.append(np.hstack(cells))
    w = max(r.shape[1] for r in sheet)
    sheet = [np.hstack([r, np.full((r.shape[0], w - r.shape[1], 3), 30, np.uint8)])
             if r.shape[1] < w else r for r in sheet]
    img = np.vstack(sheet)
    cv2.imwrite(a.out, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print("\n%s  %dx%d  (%d조합)" % (a.out, img.shape[1], img.shape[0], len(rows)))


if __name__ == "__main__":
    main()

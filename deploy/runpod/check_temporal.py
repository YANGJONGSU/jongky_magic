#!/usr/bin/env python3
"""리스타일 QC — 시간 일관성 검사 (리스타일-배치-설계.md 6절 1번 게이트).

    python3 check_temporal.py ORIG.mp4 RESTYLED.mp4

원본과 리스타일본의 연속 프레임 광류(Farneback)를 각각 계산해 비교한다.
리스타일이 프레임마다 어긋나면(조명·질감이 프레임 사이에 요동) 광류가
원본과 달라지고, 월드모델이 그 요동을 "로봇이 움직인 것" 으로 배운다.

지표: 프레임쌍별 |flow_restyled − flow_orig| 평균(EPE, px).
기준선: 원본을 자기 자신과 비교한 값(≈0) + 원본을 살짝 밝기 변형한 것과
비교한 값 — 노이즈 바닥. 판정 임계는 파일럿에서 캘리브레이션한다.
해상도가 다르면 리스타일본을 원본 크기로 리샘플해 비교한다.
"""
import sys

import cv2
import numpy as np


def read_gray(path, size=None):
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if size is not None:
            f = cv2.resize(f, size)
        frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    return frames


def flows(frames, stride=4):
    out = []
    for i in range(0, len(frames) - 1, stride):
        fl = cv2.calcOpticalFlowFarneback(frames[i], frames[i + 1], None,
                                          0.5, 3, 21, 3, 5, 1.2, 0)
        out.append(fl)
    return out


def main():
    orig_p, rest_p = sys.argv[1], sys.argv[2]
    orig = read_gray(orig_p)
    h, w = orig[0].shape
    rest = read_gray(rest_p, size=(w, h))
    n = min(len(orig), len(rest))
    orig, rest = orig[:n], rest[:n]
    print("프레임 %d (원본 %dx%d 기준 비교)" % (n, w, h))

    fo = flows(orig)
    fr = flows(rest)
    epe = [float(np.mean(np.linalg.norm(a - b, axis=-1))) for a, b in zip(fo, fr)]

    # 노이즈 바닥: 원본에 감마/밝기만 살짝 바꾼 것 (외형 변화의 정당한 몫)
    bright = [cv2.convertScaleAbs(f, alpha=1.15, beta=12) for f in orig]
    fb = flows(bright)
    floor = [float(np.mean(np.linalg.norm(a - b, axis=-1))) for a, b in zip(fo, fb)]

    print("EPE  중앙값 %.3f px · p90 %.3f px · 최대 %.3f px"
          % (np.median(epe), np.percentile(epe, 90), max(epe)))
    print("노이즈 바닥(밝기 변형만): 중앙값 %.3f px" % np.median(floor))
    ratio = np.median(epe) / max(np.median(floor), 1e-6)
    print("바닥 대비 배율: %.1fx  — 파일럿 캘리브레이션 값, 본배치 판정 기준" % ratio)
    worst = int(np.argmax(epe)) * 4
    print("최악 프레임쌍: %d→%d" % (worst, worst + 1))


if __name__ == "__main__":
    main()

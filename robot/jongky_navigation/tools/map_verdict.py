#!/usr/bin/env python3
"""지도가 건물과 맞는지 판정한다 — 내부 지표가 못 잡는 두 결함을 잡는다.

    python3 map_verdict.py 지도.pgm [기준지도.pgm]

## 왜 이 검사인가

벽 두께·복도 폭·직교성은 전부 지도 **내부** 지표라 아래 둘을 통과시킨다.
2026-08-19~21 에 실제로 둘 다 그렇게 통과됐다.

  복제      평면도의 복도 하나가 지도에 두 줄로 나온다. 8/19 10층 지도에서
            1.6m 와 1.5m 복도가 0.3m 벽을 사이에 두고 나란히 있었다.
            → 폭 0.8~2.5m 자유공간 밴드가 얇은 벽을 두고 인접하면 잡는다
  각 왜곡   루프 클로저가 없는 구간에 누적 각오차가 박힌다. 8/21 10층 지도는
            긴 복도가 25도 꺾여 있었다. 같은 건물 11층이 0/87도로 직각을
            확인해 주므로 건물 탓이 아니었다.
            → 벽 방향 봉우리(길이 가중 허프)가 90도 격자에서 벗어나면 잡는다

기준지도를 주면(예: 같은 건물 다른 층) 그 벽 방향과도 대조한다.
"""
import sys

import cv2
import numpy as np


def wall_peaks(path):
    im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if im is None:
        sys.exit(f"못 읽음: {path}")
    occ = ((im < 65) * 255).astype(np.uint8)
    edges = cv2.Canny(occ, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 720, 30, minLineLength=30, maxLineGap=5)
    hist = np.zeros(180)
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            hist[int(np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 180)] += np.hypot(x2 - x1, y2 - y1)
    h2 = np.convolve(np.concatenate([hist[-3:], hist, hist[:3]]), np.ones(7) / 7, "same")[3:-3]
    pk = []
    for i in range(180):
        if h2[i] == max(h2[(i + d) % 180] for d in range(-8, 9)) and h2[i] > h2.max() * 0.15:
            pk.append((h2[i] / max(h2.sum(), 1) * 7, i))
    pk.sort(reverse=True)
    return im, [(a, w) for w, a in pk[:5]]


def bands(im, rot, res=0.05):
    free = ((im > 205) * 255).astype(np.uint8)
    h, w = im.shape
    M = cv2.getRotationMatrix2D((w / 2, h / 2), rot, 1.0)
    fr = cv2.warpAffine(free, M, (w, h), flags=cv2.INTER_NEAREST)
    prof = (fr > 127).sum(axis=1).astype(float)
    ps = np.convolve(prof, np.ones(7) / 7, "same")
    above = ps > ps.max() * 0.35
    out, start = [], None
    for i, a in enumerate(above):
        if a and start is None:
            start = i
        if not a and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(above)))
    return [(a, b, (b - a) * res) for a, b in out if b - a >= 8]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    im, pk = wall_peaks(sys.argv[1])
    print("벽 방향 봉우리:")
    for a, wgt in pk:
        print("  %5.1f도  비중 %.0f%%" % (a, wgt * 100))

    bad = False
    # 검사 1: 각 왜곡 — 모든 봉우리가 1위 기준 90도 격자 안(±5)이어야 한다
    if pk:
        base = pk[0][0]
        for a, wgt in pk[1:]:
            d = abs(a - base) % 90
            off = min(d, 90 - d)
            if off > 5 and wgt > 0.10:
                print("  !! %.0f도 봉우리가 90도 격자에서 %.0f도 벗어남 — 각 왜곡" % (a, off))
                bad = True

    # 검사 2: 복도 이중 밴드 — 지배 방향으로 회전 후 인접 밴드 간격
    rot = pk[0][0] if pk else 0
    rot = rot if rot < 45 else rot - 90
    bl = bands(im, rot)
    print("복도 밴드 (지배 방향 %.0f도 보정):" % rot)
    for a, b, t in bl:
        print("  행 %3d~%3d  두께 %.1f m" % (a, b, t))
    for (a1, b1, t1), (a2, b2, t2) in zip(bl, bl[1:]):
        gap = (a2 - b1) * 0.05
        if gap < 0.6 and 0.8 <= t1 <= 2.5 and 0.8 <= t2 <= 2.5:
            print("  !! %.1fm 와 %.1fm 밴드가 %.2fm 벽을 두고 인접 — 복제 의심" % (t1, t2, gap))
            bad = True

    # 검사 3: 기준지도와 격자 대조
    if len(sys.argv) > 2:
        _, rpk = wall_peaks(sys.argv[2])
        if pk and rpk:
            d = abs(pk[0][0] - rpk[0][0]) % 90
            print("기준지도 대비 지배 방향 차: %.0f도 (90 격자 기준 %.0f도)"
                  % (abs(pk[0][0] - rpk[0][0]), min(d, 90 - d)))

    print()
    print("판정: %s" % ("불합격 — 위 !! 항목" if bad else "합격 (복제·각왜곡 없음)"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()

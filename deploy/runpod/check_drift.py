#!/usr/bin/env python3
"""생성 클립에서 복도 기하가 얼마나 흘러갔는지 잰다.

    python3 check_drift.py outputs/2026-08-20-....mp4

## 무엇을 재고 왜 그것인가

이 데이터의 용도는 인코더 표현 강건화다 — **같은 장면의 변형본**이어야 짝이
된다. 조명이나 질감이 흔들리는 건 오히려 목적이지만, 복도 자체가 다른 복도가
되면 짝이 아니게 된다.

그래서 "화질" 이 아니라 **기하가 유지되는가** 를 잰다. 세 가지다.

  소실점 x     카메라가 어느 쪽을 보는가. 움직이면 방향이 바뀐 것
  소실점 y     카메라 높이·틸트. 움직이면 내려다보기 시작한 것
  벽선 각도    좌우 벽-바닥 경계의 기울기. 벌어지면 복도 폭이 변한 것

소실점은 프레임의 긴 직선들(천장선·바닥선·사물함 모서리)을 허프로 뽑아
교점의 중앙값으로 잡는다. 복도를 정면으로 보고 전진하는 영상이라 이 선들이
한 점으로 모인다 — 그게 안 모이면 그 자체가 복도가 아니라는 신호다.

## 판정선

기준은 로봇이 아니라 **사람이 같은 복도로 알아보는가** 에서 왔다.
소실점 5% 는 화각 57.86도에서 약 3도에 해당하고, 벽선 3도는 1.20 m 복도에서
2 m 앞 폭이 10 cm 달라지는 정도다 — 인플레이션 반경 0.30 m 안에 들어간다.
"
"""
import argparse
import os
import sys

import cv2
import numpy as np


def vanishing_point(gray):
    """긴 직선들의 교점 중앙값. 못 구하면 None."""
    e = cv2.Canny(gray, 60, 160)
    h, w = gray.shape
    lines = cv2.HoughLinesP(e, 1, np.pi / 180, threshold=60,
                            minLineLength=w // 5, maxLineGap=12)
    if lines is None or len(lines) < 2:
        return None, []
    segs = []
    for x1, y1, x2, y2 in lines[:, 0]:
        a = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
        a = min(a, 180 - a)
        if 8 < a < 82:            # 수평·수직선은 소실점을 안 만든다
            segs.append((x1, y1, x2, y2, np.degrees(np.arctan2(y2 - y1, x2 - x1))))
    if len(segs) < 2:
        return None, segs
    pts = []
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            x1, y1, x2, y2, _ = segs[i]
            x3, y3, x4, y4, _ = segs[j]
            d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(d) < 1e-6:
                continue
            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d
            # 화면에서 지나치게 먼 교점은 평행선 잡음이다
            if -w < px < 2 * w and -h < py < 2 * h:
                pts.append((px, py))
    if not pts:
        return None, segs
    return np.median(np.array(pts), axis=0), segs


def wall_angles(segs):
    """좌·우 벽선 각도. 부호로 갈라 각각 중앙값."""
    down = [s[4] for s in segs if s[4] > 0]
    up = [s[4] for s in segs if s[4] < 0]
    return (np.median(up) if up else np.nan,
            np.median(down) if down else np.nan)


def main():
    p = argparse.ArgumentParser(description="생성 클립의 복도 기하 흐름 측정")
    p.add_argument("video")
    p.add_argument("--samples", type=int, default=9, help="몇 프레임을 잴지")
    p.add_argument("--vp-tol", type=float, default=5.0, help="소실점 허용 이동 [화면 %]")
    p.add_argument("--ang-tol", type=float, default=3.0, help="벽선 허용 각도 변화 [도]")
    a = p.parse_args()

    cap = cv2.VideoCapture(a.video)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    rows = []
    for k in range(a.samples):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(k / max(a.samples - 1, 1) * (n - 1)))
        ok, im = cap.read()
        if not ok:
            continue
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        vp, segs = vanishing_point(g)
        la, ra = wall_angles(segs)
        rows.append((k / max(a.samples - 1, 1) * 100, vp, la, ra, g.shape))
    cap.release()
    if not rows:
        sys.exit("프레임을 못 읽었다")

    h, w = rows[0][4]
    print("%-6s %10s %10s %8s %8s" % ("구간%", "소실점x", "소실점y", "좌벽", "우벽"))
    for pct, vp, la, ra, _ in rows:
        if vp is None:
            print("%5.0f%% %10s %10s %8.1f %8.1f   <- 소실점 못 찾음" %
                  (pct, "-", "-", la, ra))
        else:
            print("%5.0f%% %10.1f %10.1f %8.1f %8.1f" % (pct, vp[0], vp[1], la, ra))

    got = [(pct, vp, la, ra) for pct, vp, la, ra, _ in rows if vp is not None]
    if len(got) < 2:
        sys.exit("\n소실점을 두 프레임 이상에서 못 찾았다 — 복도 구조가 무너졌을 수 있다")

    vx = np.array([g[1][0] for g in got]); vy = np.array([g[1][1] for g in got])
    dx = (vx.max() - vx.min()) / w * 100
    dy = (vy.max() - vy.min()) / h * 100
    las = np.array([g[2] for g in got]); ras = np.array([g[3] for g in got])
    dla = np.nanmax(las) - np.nanmin(las)
    dra = np.nanmax(ras) - np.nanmin(ras)

    print("\n%-22s %7s %7s %s" % ("", "변화", "허용", "판정"))
    ok = True
    for name, val, tol, unit in (("소실점 x", dx, a.vp_tol, "%"),
                                 ("소실점 y", dy, a.vp_tol, "%"),
                                 ("좌벽 각도", dla, a.ang_tol, "도"),
                                 ("우벽 각도", dra, a.ang_tol, "도")):
        good = not np.isnan(val) and val <= tol
        ok &= good
        print("%-22s %6.1f%s %6.1f%s %s" % (name, val, unit, tol, unit,
                                            "OK" if good else "초과"))
    print()
    if ok:
        print("기하 유지됨 — 학습에 쓸 수 있다")
    else:
        print("기하가 흘렀다. 질감·조명 흔들림은 상관없지만 이건 다른 문제다.")
        print("  손잡이: --max-frames 를 9 에서 12~16 으로 (씨앗을 더 길게 조건으로 준다)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

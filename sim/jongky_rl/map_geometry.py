#!/usr/bin/env python3
# 점유 격자 지도 → 복도 형상 (벽 선분 + 폭 프로파일)
#
# ROS map_server 형식(.pgm + .yaml)을 읽어서
#   (1) 벽을 선분 목록으로 뽑고
#   (2) 거리변환 중심선을 따라 복도 폭 프로파일을 내고
#   (3) 그 폭이 외부 실측값과 맞는지 검증한다.
#
# Isaac Lab 의존성이 전혀 없다 — numpy/scipy/opencv 만 쓴다. 매핑 PC 에서 돌린다.
# 결과 JSON 을 jongky_map_corridor_env.py 가 읽어 시뮬 복도를 세운다.
#
# ── 왜 이게 필요한가 ──────────────────────────────────────────────────────
# jongky_corridor_env.py 는 복도를 corridor_width = 2.4 스칼라 하나로 세운다.
# 실측은 개방 구간 1.68~1.70 m, 사물함 구간 1.20 m 다. 2.4 m 는 개방 구간 대비
# 42% 넓고 사물함 구간 대비 100% 넓다. 게다가 실제 복도는 한 숫자가 아니라
# 구간마다 폭이 바뀌는 이봉(bimodal) 구조다. 스칼라로는 표현이 안 된다.
#
# ── 측정 방법과 그 편향 ───────────────────────────────────────────────────
# 폭 추정치는 두 가지를 낸다.
#   2*distanceTransform : 주 추정치. 합성 복도로 검증했을 때 편향 0 이고
#                         복도가 격자축에 대해 기울어져도 유지된다.
#   perpendicular raycast : 교차 검증용. 중심선에서 법선 방향으로 쏴서
#                         양쪽 벽까지 거리를 잰다. 원시값은 정확히 한 셀
#                         (=resolution) 과대평가되므로 res 를 빼서 보정한다.
#                         (합성 복도 1.20/1.68/1.70/2.40 전부에서 +0.050 확인)
# raycast 는 문·개구부를 만나면 튀므로(중심선 법선이 문으로 빠짐) 주 추정치로
# 쓰지 않는다. 대신 두 값이 어긋나는 구간을 신뢰도 낮음으로 표시하는 데 쓴다.
#
# ── 잡음 제거 ─────────────────────────────────────────────────────────────
# 유리창 너머로 라이다가 나가서 생기는 부채꼴은 (a) 점유 셀 쪽에서는 작은
# 연결 성분으로, (b) 자유 셀 쪽에서는 가느다란 쐐기로 나타난다.
#   (a) 는 min_wall_cells 미만 성분을 버려서 지운다.
#   (b) 는 반지름 open_radius 원반으로 opening 해서 지운다 — "지름 0.6 m 원이
#       들어가는 자유 공간만 복도로 본다". 가장 좁은 실측 구간(1.20 m)보다
#       훨씬 작은 기준이라 진짜 복도는 안 깎인다.

from __future__ import annotations

import argparse
import json
import math
import os
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import scipy.ndimage as ndi
import yaml

# ── 외부 실측값 ────────────────────────────────────────────────────────────
# 이 파일에서 유일하게 "지도 바깥에서 온" 숫자다. 검증의 기준선이고,
# 지도 내부 지표만으로 좋고 나쁨을 판정하지 않기 위해 반드시 필요하다.
# 줄자 실측: 개방 구간 1.68~1.70 m, 사물함 있는 구간 1.20 m.
MEASURED_OPEN = (1.68, 1.70)
MEASURED_LOCKER = 1.20

# ── 로봇 제원 (실차) ───────────────────────────────────────────────────────
# nav2_params.yaml 의 footprint 와 같은 값이어야 한다:
#   [[-0.14,-0.085],[-0.14,0.085],[0.08,0.085],[0.08,-0.085]]
# 즉 base_link 기준 뒤로 0.14, 앞으로 0.08, 좌우 0.085 인 직사각형이다.
ROBOT_FOOTPRINT = [(-0.14, -0.085), (-0.14, 0.085), (0.08, 0.085), (0.08, -0.085)]
ROBOT_HALF_WIDTH = 0.085
ROBOT_WIDTH = 2 * ROBOT_HALF_WIDTH
# 제자리 회전 시 쓸어내는 원의 반지름 (base_link 원점에서 가장 먼 꼭짓점).
# 직진 통과는 전폭 0.17 만 보면 되지만, 복도에서 돌아서려면 이 값이 기준이다.
ROBOT_CIRCUMSCRIBED_R = max((x * x + y * y) ** 0.5 for x, y in ROBOT_FOOTPRINT)

# 실차 주행 한계 — 시뮬과 반드시 일치해야 하는 값
V_MAX = 0.40        # m/s
OMEGA_MAX = 1.50    # rad/s
A_MAX = 0.30        # m/s^2

NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


# ── 지도 로드 ──────────────────────────────────────────────────────────────
@dataclass
class OccMap:
    grid: np.ndarray        # 원본 pgm 값
    free: np.ndarray        # bool
    occ: np.ndarray         # bool
    unknown: np.ndarray     # bool
    res: float              # m/cell
    origin: tuple           # (x, y, yaw) — 지도 좌하단의 월드 좌표
    name: str

    def rc_to_world(self, r, c):
        """격자 (row, col) → 월드 (x, y).

        map_server 규약: 이미지 행 0 이 지도의 '위' 이고 월드 y 는 아래에서
        위로 증가한다. 그래서 행을 뒤집는다. 셀 중심을 쓰려고 +0.5 한다.
        """
        x = self.origin[0] + (np.asarray(c) + 0.5) * self.res
        y = self.origin[1] + (self.grid.shape[0] - 1 - np.asarray(r) + 0.5) * self.res
        return x, y


def load_map(yaml_path: str) -> OccMap:
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    d = os.path.dirname(os.path.abspath(yaml_path))
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(d, img_path)
    grid = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
    if grid is None:
        raise FileNotFoundError(img_path)
    if grid.ndim == 3:
        grid = cv2.cvtColor(grid, cv2.COLOR_BGR2GRAY)
    if int(meta.get("negate", 0)):
        grid = 255 - grid

    # pgm 값 규약. occupied_thresh/free_thresh 는 점유확률 기준이라
    # 픽셀값으로는 (255 - v)/255 가 확률이다. 다만 trinary 지도는 사실상
    # 0 / 205 / 254 세 값뿐이라 아래 임계로 충분하고, 임계 자체도 yaml 을 따른다.
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))
    p = (255.0 - grid.astype(np.float32)) / 255.0   # 점유 확률
    occ = p > occ_th
    free = p < free_th
    unknown = ~(occ | free)

    return OccMap(
        grid=grid, free=free, occ=occ, unknown=unknown,
        res=float(meta["resolution"]),
        origin=tuple(meta["origin"]),
        name=os.path.splitext(os.path.basename(yaml_path))[0],
    )


# ── 잡음 제거 ──────────────────────────────────────────────────────────────
def clean_occupancy(m: OccMap, min_wall_cells: int = 10) -> np.ndarray:
    """작은 연결 성분을 버려 부채꼴 잡음의 점 끝을 지운다."""
    lab, n = ndi.label(m.occ, structure=np.ones((3, 3)))
    if n == 0:
        return m.occ.copy()
    sizes = np.bincount(lab.ravel())
    keep = sizes >= min_wall_cells
    keep[0] = False                       # 라벨 0 은 배경이다
    return keep[lab]


def free_space(m: OccMap, close_cells: int = 2) -> np.ndarray:
    """주행 가능한 자유 공간. 가장 큰 성분만 남기고 작은 미탐사 구멍을 메운다."""
    free = m.free
    lab, n = ndi.label(free)
    if n:
        sizes = ndi.sum(free, lab, range(1, n + 1))
        free = lab == (int(np.argmax(sizes)) + 1)
    k = np.ones((close_cells * 2 + 1,) * 2, np.uint8)
    return cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_CLOSE, k)


# ── 벽 선분 추출 ───────────────────────────────────────────────────────────
def extract_wall_segments(m: OccMap, occ_clean: np.ndarray,
                          min_len_m: float = 0.40,
                          max_gap_m: float = 0.15) -> list:
    """정리된 점유 격자에서 벽을 선분 목록으로 뽑는다 (월드 좌표, m).

    확률적 허프 변환을 쓴다. 벽이 몇 셀 두께의 띠라서 같은 벽에 선분이
    여러 개 겹쳐 나오는 것은 정상이다 — 시뮬에서는 어차피 벽을 상자로
    세우므로 겹쳐도 무해하고, 오히려 끊긴 벽을 메워 준다.
    """
    img = (occ_clean.astype(np.uint8)) * 255
    min_len = max(2, int(round(min_len_m / m.res)))
    max_gap = max(1, int(round(max_gap_m / m.res)))
    lines = cv2.HoughLinesP(img, rho=1, theta=np.pi / 180, threshold=max(8, min_len // 2),
                            minLineLength=min_len, maxLineGap=max_gap)
    out = []
    if lines is None:
        return out
    for x1, y1, x2, y2 in lines[:, 0, :]:
        wx1, wy1 = m.rc_to_world(y1, x1)
        wx2, wy2 = m.rc_to_world(y2, x2)
        L = math.hypot(wx2 - wx1, wy2 - wy1)
        if L < min_len_m:
            continue
        out.append({"x1": round(float(wx1), 4), "y1": round(float(wy1), 4),
                    "x2": round(float(wx2), 4), "y2": round(float(wy2), 4),
                    "len": round(float(L), 4)})
    out.sort(key=lambda s: -s["len"])
    return out


# ── 중심선 (거리변환 medial axis) ──────────────────────────────────────────
def corridor_spine(m: OccMap, free_c: np.ndarray, open_radius_m: float = 0.30):
    """가장 긴 복도 중심선을 격자 좌표 경로로 낸다.

    부채꼴 잡음(유리창 너머로 새어 나간 빔이 만드는 가느다란 자유 공간 쐐기)은
    반지름 open_radius 원반 opening 으로 지운다. 실측 최소 폭 1.20 m 의 절반도
    안 되는 기준이라 진짜 복도는 손상되지 않는다.

    그 다음 세선화(thinning) 로 1픽셀 골격을 얻고, 골격 그래프의 지름
    (두 번 BFS) 을 복도 척추로 삼는다.
    """
    rr = max(1, int(round(open_radius_m / m.res)))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * rr + 1, 2 * rr + 1))
    opened = cv2.morphologyEx(free_c, cv2.MORPH_OPEN, k)
    lab, n = ndi.label(opened)
    if n == 0:
        return [], opened
    sizes = ndi.sum(opened, lab, range(1, n + 1))
    opened = (lab == (int(np.argmax(sizes)) + 1)).astype(np.uint8)

    skel = cv2.ximgproc.thinning(opened * 255) > 0
    pts = set(zip(*[a.tolist() for a in np.nonzero(skel)]))
    if not pts:
        return [], opened

    def bfs(src):
        dist = {src: 0}
        par = {src: None}
        q = deque([src])
        while q:
            u = q.popleft()
            for a, b in NB8:
                v = (u[0] + a, u[1] + b)
                if v in pts and v not in dist:
                    dist[v] = dist[u] + 1
                    par[v] = u
                    q.append(v)
        far = max(dist, key=dist.get)
        return far, par

    a, _ = bfs(next(iter(pts)))
    b, par = bfs(a)
    path = []
    cur = b
    while cur is not None:
        path.append(cur)
        cur = par[cur]
    return path[::-1], opened


def smooth_path(path, win: int = 9):
    """세선화 경로의 계단 모양을 눌러 법선 방향이 튀지 않게 한다."""
    if len(path) < win:
        return [(float(r), float(c)) for r, c in path]
    arr = np.asarray(path, float)
    ker = np.ones(win) / win
    out = np.stack([np.convolve(arr[:, i], ker, mode="same") for i in range(2)], 1)
    # 양 끝은 convolve 가 0 쪽으로 끌어당기므로 원본으로 되돌린다
    h = win // 2
    out[:h] = arr[:h]
    out[-h:] = arr[-h:]
    return [tuple(p) for p in out]


# ── 폭 프로파일 ────────────────────────────────────────────────────────────
def _ray(free_c, occ, r, c, dr, dc, max_cells=400):
    """(r,c) 에서 (dr,dc) 방향으로 쏴서 자유 공간이 끝나는 지점까지 셀 수."""
    H, W = free_c.shape
    for t in range(1, max_cells):
        rr = int(round(r + dr * t))
        cc = int(round(c + dc * t))
        if not (0 <= rr < H and 0 <= cc < W):
            return t, "out"
        if occ[rr, cc]:
            return t, "occ"
        if not free_c[rr, cc]:
            return t, "unk"
    return max_cells, "out"


def width_profile(m: OccMap, free_c: np.ndarray, occ_clean: np.ndarray, path,
                  dir_win: int = 6):
    """중심선을 따라 복도 폭을 잰다.

    반환: dict of arrays
      s        중심선 호길이 [m]
      w_dt     2*거리변환 [m]  — 주 추정치 (합성 검증에서 편향 0)
      w_ray    법선 레이캐스트 [m] — 한 셀 과대평가를 보정한 값 (교차검증용)
      both_occ 양쪽 다 '점유' 셀에 닿았는가 (미탐사에 닿았으면 벽이 아님)
      x, y     중심선 월드 좌표
    """
    dist = cv2.distanceTransform(free_c, cv2.DIST_L2, 5) * m.res
    sm = smooth_path(path)
    n = len(sm)
    s, w_dt, w_ray, both, xs, ys, nx, ny = ([] for _ in range(8))
    for i in range(n):
        r, c = path[i]
        a = max(0, i - dir_win)
        b = min(n - 1, i + dir_win)
        lv = np.array(sm[b]) - np.array(sm[a])
        nl = np.linalg.norm(lv)
        if nl < 1e-6:
            continue
        lv /= nl
        nrm = np.array([-lv[1], lv[0]])          # 법선
        d1, k1 = _ray(free_c, occ_clean, r, c, nrm[0], nrm[1])
        d2, k2 = _ray(free_c, occ_clean, r, c, -nrm[0], -nrm[1])
        wx, wy = m.rc_to_world(r, c)
        s.append(i * m.res)
        w_dt.append(2.0 * dist[r, c])
        w_ray.append((d1 + d2) * m.res - m.res)   # ← 한 셀 편향 보정
        both.append(k1 == "occ" and k2 == "occ")
        xs.append(float(wx)); ys.append(float(wy))
        nx.append(float(nrm[0])); ny.append(float(nrm[1]))
    return {
        "s": np.array(s), "w_dt": np.array(w_dt), "w_ray": np.array(w_ray),
        "both_occ": np.array(both, bool),
        "x": np.array(xs), "y": np.array(ys),
    }


# ── 폭 프로파일을 구간으로 쪼개기 ──────────────────────────────────────────
def segment_profile(prof, tol: float = 0.12, min_len_m: float = 1.0,
                    med_win: int = 15):
    """폭 프로파일을 '폭이 거의 일정한 구간' 목록으로 만든다.

    실제 복도는 사물함 구간에서 좁아지는 계단형이다. 이걸 구간별 상수로
    근사해서 시뮬 벽 배치에 그대로 쓴다.

    양쪽이 진짜 벽(점유 셀)인 표본만 쓴다 — 미탐사 영역에 닿은 표본은
    벽까지의 거리가 아니라 '지도가 끝난 곳' 까지의 거리라서 폭이 아니다.
    """
    w = prof["w_dt"].copy()
    ok = prof["both_occ"]
    if ok.sum() < 5:
        return []
    # 미탐사에 닿은 표본은 이웃 값으로 메운다 (구간 경계가 밀리지 않게)
    idx = np.arange(len(w))
    w[~ok] = np.interp(idx[~ok], idx[ok], w[ok])
    w = ndi.median_filter(w, size=med_win, mode="nearest")

    # 바텀업 병합: 인접 구간의 중앙값 차이가 tol 미만이면 합친다
    segs = [[i, i + 1, w[i]] for i in range(len(w))]
    changed = True
    while changed and len(segs) > 1:
        changed = False
        best, bd = None, tol
        for i in range(len(segs) - 1):
            d = abs(segs[i][2] - segs[i + 1][2])
            if d < bd:
                bd, best = d, i
        if best is not None:
            a, b = segs[best], segs[best + 1]
            merged = [a[0], b[1], float(np.median(w[a[0]:b[1]]))]
            segs[best:best + 2] = [merged]
            changed = True

    # 너무 짧은 구간은 폭이 더 가까운 이웃에 흡수시킨다
    step = prof["s"][1] - prof["s"][0] if len(prof["s"]) > 1 else 0.05
    min_n = max(1, int(round(min_len_m / step)))
    while len(segs) > 1:
        short = [i for i, sg in enumerate(segs) if (sg[1] - sg[0]) < min_n]
        if not short:
            break
        i = min(short, key=lambda j: segs[j][1] - segs[j][0])
        if i == 0:
            j = 1
        elif i == len(segs) - 1:
            j = len(segs) - 2
        else:
            j = i - 1 if abs(segs[i - 1][2] - segs[i][2]) <= abs(segs[i + 1][2] - segs[i][2]) else i + 1
        lo, hi = min(i, j), max(i, j)
        a, b = segs[lo], segs[hi]
        segs[lo:hi + 1] = [[a[0], b[1], float(np.median(w[a[0]:b[1]]))]]

    out = []
    for i0, i1, wm in segs:
        sl = slice(i0, i1)
        out.append({
            "s0": round(float(prof["s"][i0]), 3),
            "s1": round(float(prof["s"][min(i1, len(prof["s"]) - 1)]), 3),
            "length": round(float(prof["s"][min(i1, len(prof["s"]) - 1)] - prof["s"][i0]), 3),
            "width": round(float(wm), 4),
            "width_p10": round(float(np.percentile(prof["w_dt"][sl], 10)), 4),
            "width_p90": round(float(np.percentile(prof["w_dt"][sl], 90)), 4),
            "width_ray": round(float(np.median(prof["w_ray"][sl])), 4),
            "wall_backed_frac": round(float(prof["both_occ"][sl].mean()), 3),
            "n": int(i1 - i0),
        })
    return out


# ── 복도망 전체 폭 센서스 ──────────────────────────────────────────────────
# 중심선(척추)은 건물을 관통하는 '한 경로' 일 뿐이라 복도망 전체를 대표하지
# 않는다. 폭 분포 검증은 골격의 모든 '복도다운' 가지에서 표본을 모아야 한다.
def _skeleton_branches(skel):
    """골격을 분기점에서 끊어 가지별 픽셀 경로 목록으로 만든다."""
    s = skel.astype(np.uint8)
    deg = cv2.filter2D(s, -1, np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], np.uint8),
                       borderType=cv2.BORDER_CONSTANT) * s
    seg = skel & ~((deg >= 3) & skel)
    lab, n = ndi.label(seg, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        ys, xs = np.nonzero(lab == i)
        if len(ys) < 5:
            continue
        pts = list(zip(ys.tolist(), xs.tolist()))
        ps = set(pts)
        deg_local = {p: sum((p[0] + a, p[1] + b) in ps for a, b in NB8) for p in pts}
        ends = [p for p in pts if deg_local[p] <= 1]
        start = ends[0] if ends else pts[0]
        path, seen = [start], {start}
        while True:
            cur = path[-1]
            nxt = None
            for a, b in NB8:
                c = (cur[0] + a, cur[1] + b)
                if c in ps and c not in seen:
                    nxt = c
                    break
            if nxt is None:
                break
            path.append(nxt)
            seen.add(nxt)
        out.append(path)
    return out


def width_census(m: OccMap, free_c, occ_clean, opened,
                 min_branch_len: float = 1.5, straightness: float = 0.90,
                 end_skip_m: float = 0.30):
    """복도망 전체에서 폭 표본을 모은다.

    분기점 근처는 폭이 의미가 없으므로(교차로라 넓게 나온다) 가지 양 끝
    end_skip_m 를 버린다. 굽은 가지는 법선 방향 추정이 부정확하므로
    직진도(끝점거리/경로길이) 기준으로 거른다.
    """
    dist = cv2.distanceTransform(free_c, cv2.DIST_L2, 5) * m.res
    skel = cv2.ximgproc.thinning(opened * 255) > 0
    W, OK = [], []
    for path in _skeleton_branches(skel):
        L = len(path) * m.res
        if L < min_branch_len:
            continue
        p0 = np.array(path[0], float)
        p1 = np.array(path[-1], float)
        if np.linalg.norm(p1 - p0) * m.res / L < straightness:
            continue
        sm = smooth_path(path)
        skip = max(3, int(round(end_skip_m / m.res)))
        for i in range(skip, len(path) - skip):
            r, c = path[i]
            a = max(0, i - 6)
            b = min(len(path) - 1, i + 6)
            lv = np.array(sm[b]) - np.array(sm[a])
            nl = np.linalg.norm(lv)
            if nl < 1e-6:
                continue
            lv /= nl
            nrm = np.array([-lv[1], lv[0]])
            _, k1 = _ray(free_c, occ_clean, r, c, nrm[0], nrm[1])
            _, k2 = _ray(free_c, occ_clean, r, c, -nrm[0], -nrm[1])
            W.append(2.0 * dist[r, c])
            OK.append(k1 == "occ" and k2 == "occ")
    return np.array(W), np.array(OK, bool)


def _kde_modes(w, lo=0.6, hi=3.0, bw=0.06, min_prom=0.05):
    w = w[(w > lo) & (w < hi)]
    if len(w) < 20:
        return []
    from scipy.signal import find_peaks
    grid = np.arange(lo, hi, 0.01)
    kde = np.exp(-0.5 * ((grid[:, None] - w[None, :]) / bw) ** 2).sum(1)
    pk, props = find_peaks(kde, prominence=min_prom * kde.max())
    order = np.argsort(props["prominences"])[::-1]
    return [{"width": round(float(grid[pk[i]]), 3),
             "prominence": round(float(props["prominences"][i] / kde.max()), 3)}
            for i in order[:5]]


# ── 검증 ───────────────────────────────────────────────────────────────────
def validate(sections, prof, robot_width: float = ROBOT_WIDTH,
             census_w=None, census_ok=None):
    """뽑아낸 폭을 외부 실측값과 대조한다.

    지도 내부 지표(일관성·잔차 같은 것)로는 판정하지 않는다. 기준은 오직
    줄자 실측 1.68~1.70 m / 1.20 m 와 로봇 실폭 0.17 m 다.
    """
    open_mid = 0.5 * (MEASURED_OPEN[0] + MEASURED_OPEN[1])

    # 벽이 뒷받침되는 구간만, 길이 가중으로 본다
    solid = [s for s in sections if s["wall_backed_frac"] >= 0.5 and s["length"] >= 1.0]

    def match(target, lo, hi):
        cand = [s for s in solid if lo <= s["width"] < hi]
        if not cand:
            return None
        tot = sum(c["length"] for c in cand)
        wmean = sum(c["width"] * c["length"] for c in cand) / tot
        return {
            "target_measured": target,
            "extracted": round(wmean, 4),
            "abs_err_m": round(wmean - target, 4),
            "pct_err": round(100.0 * (wmean - target) / target, 2),
            "total_length_m": round(tot, 2),
            "n_sections": len(cand),
            "sections": [{"s0": c["s0"], "s1": c["s1"], "width": c["width"]} for c in cand],
        }

    # 무편향 모드 탐지 (실측값을 미리 넣지 않고 KDE 로 봉우리를 찾는다)
    modes_spine = _kde_modes(prof["w_dt"][prof["both_occ"]])

    # 복도망 전체 센서스 — 폭 분포 검증의 주 근거. 척추 한 경로보다 표본이 넓다.
    census = None
    if census_w is not None and len(census_w):
        cw = census_w[census_ok]
        census = {
            "n_samples": int(len(cw)),
            "wall_backed_frac": round(float(census_ok.mean()), 3),
            "modes": _kde_modes(cw),
            "histogram": {str(round(e, 2)): int(n) for e, n in zip(
                np.arange(0.6, 3.0, 0.05),
                np.histogram(cw, bins=np.arange(0.6, 3.05, 0.05))[0]) if n},
        }
        for lbl, lo, hi, tgt in [("open_band", 1.50, 1.90, 0.5 * sum(MEASURED_OPEN)),
                                 ("locker_band", 1.00, 1.40, MEASURED_LOCKER)]:
            sel = cw[(cw >= lo) & (cw < hi)]
            census[lbl] = None if not len(sel) else {
                "target_measured": tgt,
                "extracted_median": round(float(np.median(sel)), 4),
                "abs_err_m": round(float(np.median(sel)) - tgt, 4),
                "pct_err": round(100.0 * (float(np.median(sel)) - tgt) / tgt, 2),
                "n": int(len(sel)), "frac_of_census": round(float(len(sel) / len(cw)), 3),
            }

    narrowest = min((s for s in solid), key=lambda s: s["width"], default=None)
    clear = (narrowest["width"] - robot_width) if narrowest else None

    return {
        "measured_reference": {"open_m": list(MEASURED_OPEN), "locker_m": MEASURED_LOCKER,
                               "note": "줄자 외부 실측. 지도와 독립."},
        "unsupervised_modes": modes_spine,
        "network_census": census,
        "open_band": match(open_mid, 1.50, 1.90),
        "locker_band": match(MEASURED_LOCKER, 1.00, 1.40),
        "robot": {
            "footprint": ROBOT_FOOTPRINT,
            "width_m": robot_width,
            "circumscribed_radius_m": round(ROBOT_CIRCUMSCRIBED_R, 4),
            "narrowest_section_m": narrowest["width"] if narrowest else None,
            # 직진 통과: 전폭만 보면 된다
            "straight_clearance_total_m": round(clear, 4) if clear is not None else None,
            "straight_clearance_per_side_m": round(clear / 2, 4) if clear is not None else None,
            "straight_fits": bool(clear is not None and clear > 0),
            # 제자리 회전: 외접원이 들어가야 한다
            "turn_clearance_per_side_m": (
                round(narrowest["width"] / 2 - ROBOT_CIRCUMSCRIBED_R, 4) if narrowest else None),
            "can_turn_in_place": bool(
                narrowest is not None and narrowest["width"] / 2 > ROBOT_CIRCUMSCRIBED_R),
        },
        "legacy_hardcoded": {
            "corridor_width": 2.4,
            "vs_open_pct": round(100 * (2.4 - open_mid) / open_mid, 1),
            "vs_locker_pct": round(100 * (2.4 - MEASURED_LOCKER) / MEASURED_LOCKER, 1),
        },
    }


# ── 파이프라인 ─────────────────────────────────────────────────────────────
def run(yaml_path: str, open_radius: float = 0.30, min_wall_cells: int = 10,
        seg_tol: float = 0.12, min_section_len: float = 1.0):
    m = load_map(yaml_path)
    occ_clean = clean_occupancy(m, min_wall_cells)
    free_c = free_space(m)
    path, opened = corridor_spine(m, free_c, open_radius)
    if not path:
        raise RuntimeError("복도 중심선을 못 찾았다 — open_radius 를 줄여 볼 것")
    prof = width_profile(m, free_c, occ_clean, path)
    sections = segment_profile(prof, seg_tol, min_section_len)
    walls = extract_wall_segments(m, occ_clean)
    cw, cok = width_census(m, free_c, occ_clean, opened)
    val = validate(sections, prof, ROBOT_WIDTH, cw, cok)

    return {
        "meta": {
            "source_map": os.path.abspath(yaml_path),
            "map_name": m.name,
            "resolution": m.res,
            "origin": list(m.origin),
            "grid_shape": list(m.grid.shape),
            "params": {"open_radius_m": open_radius, "min_wall_cells": min_wall_cells,
                       "segment_tol_m": seg_tol, "min_section_len_m": min_section_len},
            "cell_counts": {"occupied": int(m.occ.sum()),
                            "occupied_after_cleanup": int(occ_clean.sum()),
                            "free": int(m.free.sum()), "unknown": int(m.unknown.sum())},
        },
        "spine": {
            "length_m": round(float(prof["s"][-1]), 3),
            "n_samples": int(len(prof["s"])),
            "wall_backed_frac": round(float(prof["both_occ"].mean()), 3),
            "centerline": [[round(float(x), 3), round(float(y), 3)]
                           for x, y in zip(prof["x"], prof["y"])],
        },
        "profile": [
            {"s": round(float(a), 3), "w_dt": round(float(b), 3),
             "w_ray": round(float(c), 3), "wall_backed": bool(d)}
            for a, b, c, d in zip(prof["s"], prof["w_dt"], prof["w_ray"], prof["both_occ"])
        ],
        "sections": sections,
        "walls": walls,
        "validation": val,
    }, m, prof, occ_clean, free_c, path


def render_debug(png_path, m, occ_clean, path, prof, scale=4):
    vis = np.zeros(m.grid.shape + (3,), np.uint8)
    vis[m.unknown] = (70, 70, 70)
    vis[m.free] = (255, 255, 255)
    vis[m.occ] = (60, 60, 60)
    vis[occ_clean] = (0, 0, 200)                       # 살아남은 벽 = 빨강
    vis = cv2.resize(vis, (vis.shape[1] * scale, vis.shape[0] * scale),
                     interpolation=cv2.INTER_NEAREST)

    def col(w, ok):
        if not ok:
            return (160, 160, 160)
        if w < 1.00:
            return (255, 0, 255)
        if w < 1.40:
            return (0, 210, 255)      # 사물함 대역 ~1.2
        if w < 1.55:
            return (255, 170, 0)
        if w < 1.90:
            return (0, 255, 0)        # 개방 대역 ~1.7
        return (0, 120, 255)

    for i, (r, c) in enumerate(path):
        if i >= len(prof["s"]):
            break
        cv2.circle(vis, (int(c * scale + scale // 2), int(r * scale + scale // 2)),
                   max(2, scale // 2), col(prof["w_dt"][i], prof["both_occ"][i]), -1)
    cv2.imwrite(png_path, vis)


def main():
    ap = argparse.ArgumentParser(description="점유 격자 지도 → 복도 형상 추출 + 실측 대조 검증")
    ap.add_argument("map_yaml")
    ap.add_argument("-o", "--out", default=None, help="결과 JSON 경로")
    ap.add_argument("--debug-png", default=None, help="중심선/폭 시각화 PNG")
    ap.add_argument("--open-radius", type=float, default=0.30)
    ap.add_argument("--min-wall-cells", type=int, default=10)
    ap.add_argument("--seg-tol", type=float, default=0.12)
    ap.add_argument("--min-section-len", type=float, default=1.0)
    a = ap.parse_args()

    doc, m, prof, occ_clean, free_c, path = run(
        a.map_yaml, a.open_radius, a.min_wall_cells, a.seg_tol, a.min_section_len)

    out = a.out or (os.path.splitext(a.map_yaml)[0] + "_corridor.json")
    with open(out, "w") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)

    v = doc["validation"]
    print(f"지도      : {doc['meta']['map_name']}  {doc['meta']['grid_shape']} @ {m.res} m/cell")
    print(f"벽 정리   : 점유 {doc['meta']['cell_counts']['occupied']} → "
          f"{doc['meta']['cell_counts']['occupied_after_cleanup']} 셀 "
          f"(작은 성분 {a.min_wall_cells} 셀 미만 제거)")
    print(f"벽 선분   : {len(doc['walls'])} 개")
    print(f"중심선    : {doc['spine']['length_m']} m, 양쪽 벽 뒷받침 "
          f"{100*doc['spine']['wall_backed_frac']:.0f}%")
    print()
    print("구간별 폭 (벽이 뒷받침되는 구간만 검증에 씀):")
    print(f"  {'s0':>6} {'s1':>6} {'len':>6} {'width':>7} {'p10':>6} {'p90':>6} {'ray':>6} {'wall%':>6}")
    for s in doc["sections"]:
        print(f"  {s['s0']:6.2f} {s['s1']:6.2f} {s['length']:6.2f} {s['width']:7.3f} "
              f"{s['width_p10']:6.2f} {s['width_p90']:6.2f} {s['width_ray']:6.2f} "
              f"{100*s['wall_backed_frac']:5.0f}%")
    print()
    print("무편향 모드 탐지 (실측값을 안 넣고 KDE 봉우리):")
    print("  [척추 경로]      ", ", ".join(f"{mo['width']:.3f} m ({mo['prominence']:.2f})"
                                          for mo in v["unsupervised_modes"]) or "없음")
    cen = v.get("network_census")
    if cen:
        print("  [복도망 전체]    ", ", ".join(f"{mo['width']:.3f} m ({mo['prominence']:.2f})"
                                              for mo in cen["modes"]) or "없음",
              f"  (n={cen['n_samples']})")
    print()
    print("외부 실측 대조 — 복도망 전체 센서스:")
    if cen:
        for key, label in [("open_band", f"개방 {MEASURED_OPEN[0]}~{MEASURED_OPEN[1]} m"),
                           ("locker_band", f"사물함 {MEASURED_LOCKER} m")]:
            b = cen.get(key)
            if b is None:
                print(f"  {label}: 해당 폭의 표본 없음  ← 지도에서 이 폭이 안 나왔다")
            else:
                print(f"  {label}: 중앙값 {b['extracted_median']:.3f} m  "
                      f"오차 {b['abs_err_m']:+.3f} m ({b['pct_err']:+.1f}%)  "
                      f"n={b['n']} ({100*b['frac_of_census']:.0f}%)")
    print()
    print("외부 실측 대조 — 척추 구간별:")
    for key, label in [("open_band", f"개방 {MEASURED_OPEN[0]}~{MEASURED_OPEN[1]} m"),
                       ("locker_band", f"사물함 {MEASURED_LOCKER} m")]:
        b = v[key]
        if b is None:
            print(f"  {label}: 해당 구간 없음  ← 지도에서 이 폭이 안 나왔다")
            continue
        print(f"  {label}: 추출 {b['extracted']:.3f} m  오차 {b['abs_err_m']:+.3f} m "
              f"({b['pct_err']:+.1f}%)  총 {b['total_length_m']} m / {b['n_sections']} 구간")
    r = v["robot"]
    print()
    print(f"로봇 통과성 (최협 구간 {r['narrowest_section_m']} m 기준):")
    print(f"  직진   : 전폭 {r['width_m']} m → 여유 {r['straight_clearance_per_side_m']} m/측  "
          f"{'통과 가능' if r['straight_fits'] else '통과 불가'}")
    print(f"  제자리회전: 외접원 R={r['circumscribed_radius_m']} m → "
          f"여유 {r['turn_clearance_per_side_m']} m/측  "
          f"{'회전 가능' if r['can_turn_in_place'] else '회전 불가'}")
    lg = v["legacy_hardcoded"]
    print(f"기존 하드코딩 2.4 m: 개방 대비 {lg['vs_open_pct']:+.0f}%, "
          f"사물함 대비 {lg['vs_locker_pct']:+.0f}%")
    print()
    print(f"→ {out}")

    if a.debug_png:
        render_debug(a.debug_png, m, occ_clean, path, prof)
        print(f"→ {a.debug_png}")


if __name__ == "__main__":
    main()

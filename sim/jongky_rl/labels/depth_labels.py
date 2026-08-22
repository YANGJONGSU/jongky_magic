#!/usr/bin/env python3
"""Cosmos 클립(및 실주행 클립)에 프레임별 depth·이격 라벨을 붙인다.

    python3 depth_labels.py --index /root/labels_out/index.json \
        --cosmos /root/cosmos_out --out /root/labels_out/depth [--limit 5]

## 무엇에 쓰나 (정책 관측이 아니다)

정책 관측은 RGB 64x64 로 확정돼 있다 (전체-작업계획 "depth 는 안 쓴다").
이 라벨의 소비처는 셋이다.

  1) 안전 라벨 — 프레임별 전방 최소거리로 "가까움/안전" 을 매겨,
     생성 클립이 만든 장애물 상황의 통계를 잰다 (QC·씨앗 선정 피드백).
  2) 인코더 보조 지도 — 갈래 B 강건화 때 depth 예측을 보조 손실로 걸 수
     있다 (선택). 동역학은 안 건드린다.
  3) 기하 검증 — 복도 폭 실측(1.68m)과 추정 depth 의 벽 간격이 맞는지로
     "Cosmos 가 기하를 지켰다" 를 정량화한다. check_drift(소실점)가 합성
     클립에 못 쓰는 자리를 이게 메운다.

## 척도 복원

단안 depth(Depth Anything V2)는 아핀 불변 역심도만 준다. 미터 척도는
바닥 평면으로 복원한다 — 카메라 높이가 실측으로 고정돼 있어서 가능하다.

    카메라 높이 h = 0.0335(바퀴 반경) + 0.1656(base_link 기준 장착)
                 = 0.1991 m, 틸트 0 (URDF cam_tilt_deg 기본값)
    바닥 행 v 의 기하 심도  z(v) = h * fy / (v - cy)
    화면 하단 중앙(바닥으로 가정)에서  d_rel = a * (1/z) + b  를 최소제곱
    → 전 픽셀  z = a / (d_rel - b)

바닥에 물건이 놓인 프레임은 이 가정이 깨진다. 잔차가 큰 프레임은
scale_ok=False 로 남기고 이격 라벨을 비운다 — 틀린 척도로 낙관적 이격을
적는 것보다 빈 라벨이 낫다.

## 상태

이 스크립트는 아직 실행 전이다 (모델 가중치 미다운로드). 첫 실행:
    pip install transformers  # + torch (이미 있음)
    python3 depth_labels.py --limit 2   # 두 클립으로 척도 잔차부터 볼 것
"""
import argparse
import json
import os
import sys

import numpy as np

# 아스트라 실측 내참 (640x480, camera_info K). 832x624 클립은 비례 확대.
FY_640 = 579.01
CY_480 = 239.5
CAM_H = 0.1991      # [m] 바닥 기준 광학 중심 높이 (위 주석의 근거)


def load_depth_model():
    from transformers import pipeline
    return pipeline("depth-estimation",
                    model="depth-anything/Depth-Anything-V2-Small-hf")


def floor_scale(d_rel, fy, cy):
    """바닥 하단 밴드로 (a, b) 를 맞춘다. 반환: (a, b, 상대 잔차)."""
    H, W = d_rel.shape
    rows = np.arange(int(H * 0.85), H - 2)          # 하단 15%
    cols = slice(int(W * 0.3), int(W * 0.7))        # 중앙 40% 열
    inv_z = (rows - cy) / (CAM_H * fy)              # = 1/z(v)
    d = np.median(d_rel[rows][:, cols], axis=1)
    A = np.stack([inv_z, np.ones_like(inv_z)], axis=1)
    (a, b), res, _, _ = np.linalg.lstsq(A, d, rcond=None)
    pred = A @ np.array([a, b])
    rel_res = float(np.abs(pred - d).mean() / (np.abs(d).mean() + 1e-9))
    return a, b, rel_res


def frame_clearance(d_rel, fy, cy):
    """전방 최소거리 [m] 또는 None(척도 실패)."""
    a, b, res = floor_scale(d_rel, fy, cy)
    if a <= 0 or res > 0.10:
        return None, res
    H, W = d_rel.shape
    band = d_rel[int(H * 0.45):int(H * 0.80), int(W * 0.2):int(W * 0.8)]
    with np.errstate(divide="ignore", invalid="ignore"):
        z = a / (band - b)
    z = z[(z > 0.1) & (z < 20.0)]
    if z.size == 0:
        return None, res
    return float(np.percentile(z, 2)), res          # 최솟값 대신 2% 분위 (노이즈)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", default="/root/labels_out/index.json")
    ap.add_argument("--cosmos", default="/root/cosmos_out")
    ap.add_argument("--out", default="/root/labels_out/depth")
    ap.add_argument("--stride", type=int, default=4, help="프레임 표본 간격")
    ap.add_argument("--limit", type=int, default=0, help="앞 N개 클립만 (0=전부)")
    ap.add_argument("--save-maps", action="store_true",
                    help="depth 맵 자체도 저장 (float16, 용량 큼)")
    a = ap.parse_args()

    import cv2
    idx = json.load(open(a.index))
    todo = [r for r in idx if r["qc"] != "exclude"]
    if a.limit:
        todo = todo[:a.limit]
    model = load_depth_model()
    os.makedirs(a.out, exist_ok=True)

    from PIL import Image
    for r in todo:
        path = os.path.join(a.cosmos, "Cosmos1GP", "outputs", r["clip"])
        cap = cv2.VideoCapture(path)
        H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fy = FY_640 * (H / 480.0)
        cy = CY_480 * (H / 480.0)
        rows, maps = [], []
        i = 0
        while True:
            ok, f = cap.read()
            if not ok:
                break
            if i % a.stride:
                i += 1
                continue
            rgb = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
            d_rel = np.array(model(Image.fromarray(rgb))["predicted_depth"])
            if d_rel.shape != rgb.shape[:2]:
                d_rel = cv2.resize(d_rel, (rgb.shape[1], rgb.shape[0]))
            clr, res = frame_clearance(d_rel, fy, cy)
            rows.append({"frame": i, "clearance_m": clr, "scale_res": round(res, 4)})
            if a.save_maps:
                maps.append(d_rel.astype(np.float16))
            i += 1
        tag = os.path.splitext(r["clip"])[0][:80]
        json.dump({"clip": r["clip"], "stride": a.stride, "frames": rows},
                  open(os.path.join(a.out, tag + ".json"), "w"))
        if a.save_maps:
            np.savez_compressed(os.path.join(a.out, tag + "_depth.npz"),
                                depth=np.stack(maps))
        ok_rows = [x["clearance_m"] for x in rows if x["clearance_m"]]
        print("%s: 표본 %d · 척도성공 %d · 이격중앙값 %s"
              % (tag[:40], len(rows), len(ok_rows),
                 "%.2fm" % np.median(ok_rows) if ok_rows else "-"))


if __name__ == "__main__":
    main()

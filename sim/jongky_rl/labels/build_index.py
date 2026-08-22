#!/usr/bin/env python3
"""Cosmos 산출물의 출처 인덱스를 만든다 — 라벨 파이프라인의 0단계.

    python3 build_index.py --cosmos /root/cosmos_out --out /root/labels_out

클립 하나마다 다음을 한 줄로 묶는다.

    출력 mp4 ↔ 씨앗 클립 ↔ bag 오프셋(파일명의 초) ↔ 조건 ↔ QC 판정 ↔ 갈래

갈래(branch)는 이 클립이 어느 학습 경로에 들어갈 수 있는지다.

    B  인코더 강건화 (augmentation pair) — 씨앗과 짝만 맞으면 됨. 기본값.
    A  궤적 리스타일 — 프레임 t 가 실주행 프레임 t 의 변형일 때만.
       현재 배치(연속 생성)는 씨앗 이후 궤적이 갈리므로 해당 없음.
    없음 — QC 제외 클립.

액션 라벨은 여기서 붙이지 않는다. 이 인덱스가 "어느 클립에 무엇을 붙일 수
있는가" 를 못 박고, action_labels.py / depth_labels.py 가 이걸 읽는다.
"""
import argparse
import json
import os
import re


def seed_meta(seed_file):
    """corridor_10f_0330.mp4 → (층, bag 오프셋 초). make_seed clip --at 의 값."""
    m = re.match(r"corridor_(\d+f)_(\d+)\.mp4", seed_file)
    if not m:
        return None, None
    return m.group(1), int(m.group(2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cosmos", default="/root/cosmos_out")
    ap.add_argument("--out", default="/root/labels_out")
    ap.add_argument("--seed-frames", type=int, default=9,
                    help="생성 조건으로 들어간 씨앗 프레임 수 (--max-frames)")
    a = ap.parse_args()

    verdict = json.load(open(os.path.join(a.cosmos, "qc_verdict.json")))
    exclude = set(verdict.get("exclude", []))
    borderline = set(verdict.get("borderline", []))

    rows = []
    with open(os.path.join(a.cosmos, "batch_manifest.jsonl")) as f:
        for line in f:
            rec = json.loads(line)
            floor, at_s = seed_meta(rec["seed_file"])
            for out_name in rec.get("out", []):
                qc = ("exclude" if out_name in exclude
                      else "borderline" if out_name in borderline
                      else "pass")
                rows.append({
                    "clip": out_name,
                    "seed_file": rec["seed_file"],
                    "floor": floor,
                    "bag_at_s": at_s,
                    "cond": rec["cond"],
                    "resolution": rec["resolution"],
                    "qc": qc,
                    # 현재 배치는 전부 연속 생성 — 씨앗 프레임 수만큼만
                    # 실주행과 정렬되고 그 뒤는 모델이 지어낸 궤적이다.
                    "branch": None if qc == "exclude" else "B",
                    "aligned_frames": a.seed_frames,
                    "action_source": "none",
                    "depth": None,
                })

    # 매니페스트 밖의 mp4 = 8/20 수동시험 7개 (프롬프트 분할 시절).
    # 씨앗 추적이 안 되므로 갈래 없이 기록만 남긴다 — 학습에 쓰려면
    # seed_file 을 사람이 채워야 한다.
    outputs_dir = os.path.join(a.cosmos, "Cosmos1GP", "outputs")
    known = {r["clip"] for r in rows}
    for name in sorted(os.listdir(outputs_dir)):
        if not name.endswith(".mp4") or name in known:
            continue
        qc = ("exclude" if name in exclude
              else "borderline" if name in borderline else "pass")
        rows.append({
            "clip": name, "seed_file": None, "floor": None, "bag_at_s": None,
            "cond": "manual", "resolution": None, "qc": qc,
            "branch": None, "aligned_frames": 0,
            "action_source": "none", "depth": None,
        })

    os.makedirs(a.out, exist_ok=True)
    out_path = os.path.join(a.out, "index.json")
    json.dump(rows, open(out_path, "w"), ensure_ascii=False, indent=1)

    n = len(rows)
    from collections import Counter
    print("클립 %d개 → %s" % (n, out_path))
    print("QC:", dict(Counter(r["qc"] for r in rows)))
    print("조건:", dict(Counter(r["cond"] for r in rows)))
    missing = [r["clip"] for r in rows
               if not os.path.exists(os.path.join(a.cosmos, "Cosmos1GP", "outputs", r["clip"]))]
    if missing:
        print("경고: mp4 실물이 없는 항목 %d개 (첫 항목: %s)" % (len(missing), missing[0]))


if __name__ == "__main__":
    main()

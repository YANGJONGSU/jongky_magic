#!/usr/bin/env python3
"""리스타일 본배치 출력 → dreamer 에피소드 적재 (갈래 A 개통).

    python3 pack_restyle.py --batch-dir <회수한 outputs/main_batch> \
        --manifest /root/labels_out/restyle_batch/main_batch_manifest.json \
        --labels /root/labels_out/window_labels \
        --out /root/labels_out/episodes_restyle

각 video_i 에 대해:
  1) QC-lite — 프레임 수 120 일치 (아니면 거부: 라벨 정렬 불가)
  2) 960x704 출력 → 64x64 리샘플 (파일럿 기하 검사로 무해 확인됨)
  3) 창 라벨(window_labels/<창>.npz — 실측 액션·proprio·보상) 상속
  4) pack_episodes.pack 으로 dreamer 규격 npz + provenance

시간 일관성·기하 정밀 QC 는 표본 검사로 별도 수행 (check_temporal /
geo_compare — 파일럿에서 캘리브레이션한 기준선 사용).

파일럿 검증: --pilot 플래그는 파일럿 2클립 + 해당 창 라벨로 같은 경로를
끝까지 태운다 — 본배치 도착 전에 파이프라인을 실물로 확인하는 용도.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pack_episodes import clip_frames, pack  # noqa: E402


def pack_one(mp4, label_npz, out_npz, provenance):
    frames = clip_frames(mp4)                 # 64x64 리샘플 포함
    lab = np.load(label_npz)
    T_f, T_l = len(frames), len(lab["reward"])
    if T_f != T_l:
        print("  거부 %s: 프레임 %d ≠ 라벨 %d" % (os.path.basename(mp4), T_f, T_l))
        return False
    pack(frames, lab["action"], lab["proprio"], lab["reward"],
         lab["is_terminal"], out_npz, provenance)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch-dir")
    ap.add_argument("--manifest", default="/root/labels_out/restyle_batch/main_batch_manifest.json")
    ap.add_argument("--labels", default="/root/labels_out/window_labels")
    ap.add_argument("--out", default="/root/labels_out/episodes_restyle")
    ap.add_argument("--pilot", action="store_true", help="파일럿 2클립으로 경로 검증")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if a.pilot:
        jobs = [
            ("/home/user/Downloads/outputs/pilot_straight/output.mp4",
             "win_11f_0903", "pilot_straight", "evening_light(폐기된 변형)"),
            ("/home/user/Downloads/pilot_nearwall_distilled/output.mp4",
             "win_11f_1025", "pilot_nearwall", "wet_floor"),
        ]
        ok = 0
        for mp4, win, tag, var in jobs:
            ok += pack_one(mp4, os.path.join(a.labels, win + ".npz"),
                           os.path.join(a.out, "pilot_%s.npz" % win),
                           {"clip": mp4, "window": win, "variation": var,
                            "branch": "A", "action_source": "seed", "origin": tag})
        print("파일럿 적재 %d/2" % ok)
        if ok:
            ep = np.load(os.path.join(a.out, "pilot_win_11f_0903.npz"))
            print("검증: 키 %s · image %s %s · reward 합 %.2f"
                  % (sorted(ep.files), ep["image"].shape, ep["image"].dtype,
                     float(ep["reward"].sum())))
        return

    man = json.load(open(a.manifest))
    packed = skipped = 0
    for m in man:
        mp4 = os.path.join(a.batch_dir, m["subfolder"], "output.mp4")
        win = os.path.splitext(m["window"])[0]
        if not os.path.exists(mp4):
            print("  없음: %s" % mp4)
            skipped += 1
            continue
        done = pack_one(mp4, os.path.join(a.labels, win + ".npz"),
                        os.path.join(a.out, "%s__%s.npz" % (win, m["variation"])),
                        {"clip": mp4, "window": win, "variation": m["variation"],
                         "branch": "A", "action_source": "seed",
                         "batch_index": m["index"]})
        packed += done
        skipped += (not done)
    print("적재 %d · 거부/누락 %d (총 %d)" % (packed, skipped, len(man)))


if __name__ == "__main__":
    main()

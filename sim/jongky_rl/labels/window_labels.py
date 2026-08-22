#!/usr/bin/env python3
"""리스타일 창 16개의 실측 라벨 추출 — restyle_windows.json 기반.

    python3 window_labels.py --windows /root/labels_out/restyle_windows.json \
        --out /root/labels_out/window_labels

action_labels.py seed 와 같은 기계(relabel_segment, fragment 의미론)를 쓰되,
씨앗 인덱스가 아니라 창 매니페스트(pick_windows 산출)를 소비한다.
창 이름 규칙은 make_seed clips 와 동일: win_{층}_{int(at):04d}.
리스타일본(변형 3종)은 같은 창 라벨을 그대로 상속한다 — 픽셀만 다르므로.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import reward_spec as R  # noqa: E402

from action_labels import seed_frame_times  # noqa: E402
from relabel_bag import load_streams, relabel_segment  # noqa: E402

SECONDS = 4.0    # 창 규격 (120프레임 @30fps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--windows", default="/root/labels_out/restyle_windows.json")
    ap.add_argument("--out", default="/root/labels_out/window_labels")
    ap.add_argument("--odom-topic", default="/odom")
    a = ap.parse_args()

    m = json.load(open(a.windows))
    by_bag = {}
    for w in m["windows"]:
        by_bag.setdefault(w["bag"], []).append(w)

    os.makedirs(a.out, exist_ok=True)
    for bag, wins in by_bag.items():
        floor = "10f" if "10f" in os.path.basename(bag) else "11f"
        ats = [w["at_rel_s"] for w in wins]
        print("%s: 창 %d개 라벨 추출" % (os.path.basename(bag), len(wins)))
        frames = seed_frame_times(bag, "/camera/rgb/image_raw", ats, SECONDS)
        cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear, _, _ = load_streams(
            bag, None, a.odom_topic, "/cmd_vel", "/scan")
        for w in wins:
            ft = np.array(frames[w["at_rel_s"]])
            name = "win_%s_%04d" % (floor, int(w["at_rel_s"]))
            if len(ft) < int(SECONDS * 30):
                print("  경고 %s: 프레임 %d < 120" % (name, len(ft)))
            if len(ft) < 2:
                continue
            act, prop, rew, term, dist, diag = relabel_segment(
                ft, cmd_t, cmd_v, odo_t, odo, scan_t, scan_clear, fragment=True)
            np.savez_compressed(os.path.join(a.out, name + ".npz"),
                                t=ft[:len(rew)], action=act, proprio=prop,
                                reward=rew, is_terminal=term)
            print("  %s: %d프레임 |v̄|=%.2f 이격최소 %.2fm (%s)"
                  % (name, len(rew), float(np.abs(act[:, 0] * R.V_MAX).mean()),
                     diag["min_clear"], w["kind"]))


if __name__ == "__main__":
    main()

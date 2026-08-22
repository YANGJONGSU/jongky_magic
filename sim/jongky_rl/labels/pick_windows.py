#!/usr/bin/env python3
"""리스타일(갈래 A) 배치용 입력 창 선정 — 층화 표본.

    python3 pick_windows.py --episodes /root/labels_out/episodes \
        --out /root/labels_out/restyle_windows.json

라벨 완료된 실물 에피소드에서 120프레임(4초) 창을 뽑는다. 층화 기준
(리스타일-배치-설계.md 3절):
  · 근접 8개 — 근접 패널티 활성 프레임 많고 이격이 가장 좁은 창 우선
  · 직진 8개 — 전진 속도 높고 근접 없는 창
  · 에피소드당 최대 2창 (한 구간 몰림 방지)

출력 매니페스트의 at_rel_s 는 **bag 첫 이미지 기준 상대 초** — make_seed
clip --at 과 같은 좌표계다. 창 시작을 몇 프레임 어긋나게 잡아도 문제없다:
추출(make_seed)과 라벨(action_labels seed)이 같은 규칙("--at 이후 연속 유효
프레임")을 쓰므로 서로는 항상 정렬된다.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcap_lite as M  # noqa: E402

WIN = 120           # 프레임 (4초 @30Hz — Cosmos 네이티브 121 이하)


def first_image_time(bag, topic="/camera/rgb/image_raw"):
    for _, lt, _ in M.iter_messages(bag, {topic}):
        return lt / 1e9
    raise RuntimeError("이미지가 없다: %s" % bag)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", default="/root/labels_out/episodes")
    ap.add_argument("--out", default="/root/labels_out/restyle_windows.json")
    ap.add_argument("--per-kind", type=int, default=8)
    ap.add_argument("--per-episode", type=int, default=2)
    a = ap.parse_args()

    bag_t0 = {}
    cands = []
    for f in sorted(glob.glob(os.path.join(a.episodes, "*.npz"))):
        meta = json.load(open(f.replace(".npz", ".json")))
        bag = meta["bag"]
        if bag not in bag_t0:
            bag_t0[bag] = first_image_time(bag)
            print("bag 첫 이미지: %s → %.3f" % (os.path.basename(bag), bag_t0[bag]))
        ep = np.load(f)
        act, rew = ep["action"], ep["reward"]
        T = len(rew)
        # 에피소드 프레임 시각은 t0~t1 균등 근사 — 위 머리주석의 이유로 충분
        times = np.linspace(meta["t0"], meta["t1"], T)
        for s in range(0, T - WIN + 1, WIN // 2):
            w_act, w_rew = act[s:s + WIN], rew[s:s + WIN]
            prox = (w_rew < -0.05) & (w_rew > -1.0)
            cands.append({
                "ep": os.path.basename(f), "bag": bag, "start_frame": s,
                "at_rel_s": round(float(times[s] - bag_t0[bag]), 2),
                "mean_v": round(float(w_act[:, 0].mean()), 3),
                "prox_frames": int(prox.sum()),
                "min_rew": round(float(w_rew.min()), 3),
            })

    def cap_per_ep(rows):
        picked, ep_count, bag_count = [], {}, {}
        for r in rows:
            if ep_count.get(r["ep"], 0) >= a.per_episode:
                continue
            # 층 쏠림 방지 (bag 당 상한) + 같은 에피소드 내 겹침 금지
            if bag_count.get(r["bag"], 0) >= max(a.per_kind - 3, 1):
                continue
            if any(p["ep"] == r["ep"] and abs(p["start_frame"] - r["start_frame"]) < WIN
                   for p in picked):
                continue
            picked.append(r)
            ep_count[r["ep"]] = ep_count.get(r["ep"], 0) + 1
            bag_count[r["bag"]] = bag_count.get(r["bag"], 0) + 1
            if len(picked) >= a.per_kind:
                break
        return picked

    near = cap_per_ep(sorted([c for c in cands if c["prox_frames"] >= 10],
                             key=lambda c: c["min_rew"]))
    used = {(c["ep"], c["start_frame"]) for c in near}
    straight = cap_per_ep(sorted(
        [c for c in cands if c["prox_frames"] == 0
         and (c["ep"], c["start_frame"]) not in used],
        key=lambda c: -c["mean_v"]))

    for r in near:
        r["kind"] = "near_wall"
    for r in straight:
        r["kind"] = "straight"
    out = near + straight
    json.dump({"win_frames": WIN, "windows": out},
              open(a.out, "w"), ensure_ascii=False, indent=1)
    print("\n창 %d개 (근접 %d · 직진 %d) → %s" % (len(out), len(near), len(straight), a.out))
    for r in out:
        print("  %-9s %s @%7.1fs | v̄ %+0.2f | 근접 %3d프레임 | min_rew %+.2f"
              % (r["kind"], r["ep"][:18], r["at_rel_s"], r["mean_v"],
                 r["prox_frames"], r["min_rew"]))


if __name__ == "__main__":
    main()

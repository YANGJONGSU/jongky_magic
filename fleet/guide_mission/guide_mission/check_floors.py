#!/usr/bin/env python3
"""floors.yaml 을 현장 가기 전에 검사한다. ROS 도 로봇도 필요 없다.

    ros2 run guide_mission check_floors ~/floors.yaml
    python3 -m guide_mission.check_floors ~/floors.yaml

지도와 waypoint 의 짝이 어긋난 것, 엘리베이터 지점이 없는 것, 맵핑 중 터미널
버퍼가 새어 들어간 이름을 잡는다. 종료 코드 0 이면 그대로 띄워도 된다.
"""
from __future__ import annotations

import argparse
import sys

from guide_mission.floors import ERROR, FloorBook


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", default="~/floors.yaml")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="waypoint 가 지도 밖이라고 판정하는 여유 [m]")
    args = ap.parse_args(argv)

    try:
        book = FloorBook.load(args.path, validate_now=False)
    except Exception as e:
        print(f"✗ floors.yaml 을 못 읽는다: {e}", file=sys.stderr)
        return 2

    problems = book.validate(args.margin)
    for fl in book.ordered():
        mark = "✓" if fl.usable else "✗"
        n = len(fl.waypoints)
        size = ""
        if fl.map_info:
            x0, y0, x1, y1 = fl.map_info.bounds
            size = f"  지도 {x1-x0:.0f}×{y1-y0:.0f} m @ {fl.map_info.resolution} m/px"
        print(f"{mark} {fl.key:<6} {fl.label:<6} waypoint {n}개"
              f"  엘리베이터 {fl.board}/{fl.exit_wp}"
              f"  SSID {','.join(fl.ssids) or '-'}{size}")
        for name in fl.destinations():
            print(f"      · {name['label']}"
                  + (f"  ({name['name']})" if name["label"] != name["name"] else ""))

    if problems:
        print()
        for p in problems:
            print(f"  {p}")

    bad = [p for p in problems if p.level == ERROR]
    warn = len(problems) - len(bad)
    print()
    if bad:
        print(f"✗ 오류 {len(bad)}건 — 이대로는 층 전환이 안 된다")
        return 1
    print("✓ 짝이 맞는다" + (f" (경고 {warn}건은 읽어 볼 것)" if warn else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

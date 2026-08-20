#!/usr/bin/env python3
"""후면 사람 탐지 서비스(follow_service.py)를 물어보는 클라이언트.

`brain.py` 와 같은 자리에 있는 부품이다 — 컨테이너 안 ROS 노드가
호스트에서 도는 서비스를 HTTP 로 부른다. 컨테이너의 cv2 가 numpy 2.0 ABI
충돌로 죽기 때문에 탐지는 호스트에 있다.

[안전 규약]
**탐지가 죽어도 주행은 죽지 않는다.** 서비스가 없거나 응답이 늦으면
`present=None` 을 돌려주고, 호출하는 쪽은 그걸 "모름" 으로 다뤄 안내를
계속한다. 사람이 없다고 단정해 멈춰 서는 것보다 낫다 — 탐지기 장애로
로봇이 복도 한가운데 서 있는 편이 더 나쁘다.
"""
from __future__ import annotations

import json
import logging
import os
import urllib.error
import urllib.request
from dataclasses import dataclass

log = logging.getLogger("follow")

FOLLOW_URL = os.environ.get("JONGKY_FOLLOW_URL", "http://localhost:8641/follower")


@dataclass
class FollowerState:
    present: bool | None      # None = 모름 (서비스 장애)
    distance_m: float = 0.0
    bearing_deg: float = 0.0
    score: float = 0.0
    age_s: float = -1.0

    @property
    def known(self) -> bool:
        return self.present is not None


class Follower:
    def __init__(self, url: str = FOLLOW_URL, timeout: float = 1.0, stale_s: float = 3.0):
        self._url = url
        self._timeout = timeout
        self._stale = stale_s
        self._warned = False

    def poll(self) -> FollowerState:
        try:
            with urllib.request.urlopen(self._url, timeout=self._timeout) as r:
                d = json.load(r)
        except (urllib.error.URLError, OSError, ValueError) as e:
            if not self._warned:
                log.info(f"탐지 서비스에 못 닿는다 — 추종 확인 없이 간다: {e}")
                self._warned = True
            return FollowerState(present=None)

        self._warned = False
        if d.get("error"):
            return FollowerState(present=None)
        # 서비스는 살아 있는데 프레임이 낡았으면 판단을 믿지 않는다.
        age = float(d.get("age_s", -1.0))
        if age < 0 or age > self._stale:
            return FollowerState(present=None, age_s=age)

        return FollowerState(
            present=bool(d.get("present", False)),
            distance_m=float(d.get("distance_m", 0.0)),
            bearing_deg=float(d.get("bearing_deg", 0.0)),
            score=float(d.get("score", 0.0)),
            age_s=age,
        )

    def available(self) -> bool:
        return self.poll().known

#!/usr/bin/env python3
"""상태머신이 바깥세상에 하는 일 — 그 경계면.

transfer.py 는 ROS 를 모른다. 실제 행동(Nav2 목표, 지도 교체, AMCL 초기화,
TTS, 화면)은 전부 이 인터페이스를 통해 나간다. 로봇에서는 guide_node.py 의
`NavEffects` 가, 시험에서는 아래 `FakeEffects` 가 이 자리를 채운다.

**모든 메서드는 예외를 던지지 않고 (성공여부, 사람이 읽을 이유) 를 돌려준다.**
실패를 예외로 던지면 어딘가에서 삼켜지고, 삼켜진 실패가 곧 "다른 층 지도로
주행" 이다.
"""
from __future__ import annotations

from dataclasses import dataclass, field


class Effects:
    """구현해야 하는 것 (덕 타이핑. 상속은 안 해도 된다)."""

    def announce(self, *, state, message, speak, actions, target, here, detail) -> None:
        """화면과 음성. speak 가 비면 말은 안 한다."""
        raise NotImplementedError

    def navigate(self, floor_key: str, waypoint: str) -> tuple[bool, str]:
        """그 층 지도 위의 한 지점까지 Nav2 로 간다. 끝날 때까지 블로킹."""
        raise NotImplementedError

    def hold(self) -> None:
        """Nav2 목표를 취소하고 제자리에 선다. 여러 번 불러도 안전해야 한다."""
        raise NotImplementedError

    def set_localized(self, ok: bool) -> None:
        """초기 위치가 유효한가. False 면 어떤 목적지 요청도 거부돼야 한다."""
        raise NotImplementedError

    def load_map(self, floor) -> tuple[bool, str]:
        """map_server 에 그 층 지도를 올린다. **결과 코드를 반드시 확인한다.**"""
        raise NotImplementedError

    def relocalize(self, floor, waypoint: str) -> tuple[bool, str]:
        """AMCL 초기 위치를 그 지점으로 주고 **실제로 잡혔는지 확인**한다."""
        raise NotImplementedError

    def detect_floor(self):
        """FloorGuess 를 돌려준다 (detect.py)."""
        raise NotImplementedError

    def resume(self, destination: str) -> tuple[bool, str]:
        """새 층에서 원래 목적지까지 안내를 이어 간다. 블로킹."""
        raise NotImplementedError


@dataclass
class FakeEffects:
    """시험용. 무엇을 시켰는지 기록하고, 실패를 지정할 수 있다."""

    guess: object = None
    nav_fail: str = ""          # 비지 않으면 navigate 가 그 이유로 실패
    map_fail: str = ""
    pose_fail: str = ""
    resume_fail: str = ""
    calls: list = field(default_factory=list)
    announcements: list = field(default_factory=list)
    spoken: list = field(default_factory=list)
    localized: bool = True
    holds: int = 0
    loaded_maps: list = field(default_factory=list)
    poses: list = field(default_factory=list)

    def announce(self, *, state, message, speak, actions, target, here, detail) -> None:
        self.announcements.append(
            {"state": state, "message": message, "actions": tuple(actions),
             "target": target, "here": here, "detail": detail})
        if speak:
            self.spoken.append(speak)

    def navigate(self, floor_key, waypoint):
        self.calls.append(("navigate", floor_key, waypoint))
        return (False, self.nav_fail) if self.nav_fail else (True, "")

    def hold(self):
        self.holds += 1
        self.calls.append(("hold",))

    def set_localized(self, ok):
        self.localized = ok
        self.calls.append(("set_localized", ok))

    def load_map(self, floor):
        self.calls.append(("load_map", floor.key))
        if self.map_fail:
            return False, self.map_fail
        self.loaded_maps.append(floor.key)
        return True, ""

    def relocalize(self, floor, waypoint):
        self.calls.append(("relocalize", floor.key, waypoint))
        if self.pose_fail:
            return False, self.pose_fail
        self.poses.append((floor.key, waypoint))
        return True, ""

    def detect_floor(self):
        self.calls.append(("detect_floor",))
        return self.guess

    def resume(self, destination):
        self.calls.append(("resume", destination))
        return (False, self.resume_fail) if self.resume_fail else (True, "")

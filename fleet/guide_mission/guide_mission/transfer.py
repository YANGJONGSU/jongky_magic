#!/usr/bin/env python3
"""층 전환 상태머신.

**엘리베이터는 주행 구간이 아니라 상태 전이 구간이다.**

  버튼을 못 누른다. 좁은 금속 박스라 AMCL 이 어느 층 지도에도 안 맞는다.
  로봇은 수직으로 움직이는데 엔코더는 정지라고 한다. 문이 열리면 다른 층이다.

그래서 이 구간은 "목표를 주고 도착을 기다리는" 코드가 아니라, **사람이 하는
일을 기다리는 코드**다. 로봇이 하는 일은 셋뿐이다 — 제 자리에 서 있기,
사람에게 다음에 무엇을 해 달라고 말하기, 문이 열린 뒤 자기가 어디인지 다시
알아내기.

이 파일은 **ROS 를 임포트하지 않는다.** 실제 행동은 Effects 로 주입한다.
그래야 Isaac 도 젯슨도 없이 전이 전부를 시험할 수 있다 (test/test_transfer.py).

┌─────────────────────────────────────────────────────────────────────────┐
│ 상태 전이표                                                              │
├────────────────┬────────────────┬───────────────────────────────────────┤
│ 상태            │ 사건            │ 다음                                  │
├────────────────┼────────────────┼───────────────────────────────────────┤
│ idle           │ start          │ to_elevator                           │
│ to_elevator    │ nav_ok         │ at_elevator                           │
│ (엘리베이터 앞  │ nav_fail       │ fault    ← 못 가면 층 전환 자체를 접는다  │
│  까지 자율주행) │ abort          │ aborted                               │
│ at_elevator    │ called         │ boarding  (사람이 버튼을 눌렀다)         │
│ (도착·버튼 요청)│ timeout/abort  │ fault / aborted                       │
│ boarding       │ boarded        │ riding    (사람이 로봇을 태웠다)         │
│ (탑승 대기)     │ timeout/abort  │ fault / aborted                       │
│ riding         │ arrived        │ exiting   (목표 층에 섰다)              │
│ (이동중·nav2 X)│ timeout/abort  │ fault / aborted                       │
│ exiting        │ exited         │ confirm_floor (사람이 내려 줬다)        │
│ (하차 대기)     │ timeout/abort  │ fault / aborted                       │
│ confirm_floor  │ floor_ok       │ swap_map                              │
│ (층 판정·확정)  │ timeout/abort  │ fault / aborted                       │
│ swap_map       │ map_ok         │ relocalize                            │
│ (지도 교체)     │ map_fail       │ fault    ← 여기서 안 멈추면 다른 층 지도로│
│                │                │            길을 찾는다                  │
│ relocalize     │ pose_ok        │ landed                                │
│ (AMCL 재초기화) │ pose_fail      │ fault    ← map->odom 이 안 나오면 정지   │
│ landed         │ same_floor     │ resume                                │
│ (층 대조)       │ wrong_floor    │ at_elevator  (다시 태워 달라)           │
│                │ give_up        │ fault    (재탑승 한도 초과)              │
│ resume         │ nav_ok         │ done                                  │
│ (안내 재개)     │ nav_fail/abort │ fault / aborted                       │
└────────────────┴────────────────┴───────────────────────────────────────┘

**localized 는 boarding 에 들어가는 순간 False 가 되고 relocalize 성공에서만
True 로 돌아온다.** 그 사이에는 어떤 목적지 요청도 거부된다. fault·aborted 는
False 인 채로 끝난다 — 사람이 UI 에서 층과 위치를 다시 잡아 줘야 움직인다.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

from guide_mission.korean import ro

# ── 상태 ──────────────────────────────────────────────────────────────────
IDLE = "idle"
TO_ELEVATOR = "to_elevator"
AT_ELEVATOR = "at_elevator"
BOARDING = "boarding"
RIDING = "riding"
EXITING = "exiting"
CONFIRM_FLOOR = "confirm_floor"
SWAP_MAP = "swap_map"
RELOCALIZE = "relocalize"
LANDED = "landed"
RESUME = "resume"
DONE = "done"
ABORTED = "aborted"
FAULT = "fault"

TERMINAL = (DONE, ABORTED, FAULT)

# ── 사건 ──────────────────────────────────────────────────────────────────
START = "start"
NAV_OK = "nav_ok"
NAV_FAIL = "nav_fail"
CALLED = "called"          # 사람: 엘리베이터를 불렀다
BOARDED = "boarded"        # 사람: 로봇을 태웠다
ARRIVED = "arrived"        # 사람: 목표 층에 섰다
EXITED = "exited"          # 사람: 로봇을 내려놨다
PICK_FLOOR = "pick_floor"  # 사람: 층을 골랐다
FLOOR_OK = "floor_ok"
MAP_OK = "map_ok"
MAP_FAIL = "map_fail"
POSE_OK = "pose_ok"
POSE_FAIL = "pose_fail"
SAME_FLOOR = "same_floor"
WRONG_FLOOR = "wrong_floor"
GIVE_UP = "give_up"
ABORT = "abort"
TIMEOUT = "timeout"

# 사람이 누를 수 있는 것만 모아 둔다. UI 가 이 이름으로 버튼을 그린다.
HUMAN_EVENTS = (CALLED, BOARDED, ARRIVED, EXITED, PICK_FLOOR, ABORT)

BUTTON_LABEL = {
    CALLED: "엘리베이터를 불렀습니다",
    BOARDED: "로봇을 태웠습니다",
    ARRIVED: "도착했습니다",
    EXITED: "로봇을 내려놨습니다",
    PICK_FLOOR: "층 고르기",
    ABORT: "취소",
}

TRANSITIONS: dict[tuple[str, str], str] = {
    (IDLE, START): TO_ELEVATOR,
    (TO_ELEVATOR, NAV_OK): AT_ELEVATOR,
    (TO_ELEVATOR, NAV_FAIL): FAULT,
    (TO_ELEVATOR, ABORT): ABORTED,
    (AT_ELEVATOR, CALLED): BOARDING,
    (AT_ELEVATOR, ABORT): ABORTED,
    (AT_ELEVATOR, TIMEOUT): FAULT,
    (BOARDING, BOARDED): RIDING,
    (BOARDING, ABORT): ABORTED,
    (BOARDING, TIMEOUT): FAULT,
    (RIDING, ARRIVED): EXITING,
    (RIDING, ABORT): ABORTED,
    (RIDING, TIMEOUT): FAULT,
    (EXITING, EXITED): CONFIRM_FLOOR,
    (EXITING, ABORT): ABORTED,
    (EXITING, TIMEOUT): FAULT,
    (CONFIRM_FLOOR, FLOOR_OK): SWAP_MAP,
    (CONFIRM_FLOOR, ABORT): ABORTED,
    (CONFIRM_FLOOR, TIMEOUT): FAULT,
    (SWAP_MAP, MAP_OK): RELOCALIZE,
    (SWAP_MAP, MAP_FAIL): FAULT,
    (RELOCALIZE, POSE_OK): LANDED,
    (RELOCALIZE, POSE_FAIL): FAULT,
    (LANDED, SAME_FLOOR): RESUME,
    (LANDED, WRONG_FLOOR): AT_ELEVATOR,
    (LANDED, GIVE_UP): FAULT,
    (RESUME, NAV_OK): DONE,
    (RESUME, NAV_FAIL): FAULT,
    (RESUME, ABORT): ABORTED,
}


class TransitionError(RuntimeError):
    """전이표에 없는 (상태, 사건). 코드 버그다 — 로봇은 fault 로 떨어진다."""


# ── 사람 입력 창구 ─────────────────────────────────────────────────────────
class Gate:
    """UI 스레드가 넣고 임무 스레드가 꺼내 간다.

    허용하지 않은 사건은 버린다 — 이동 중에 '내려놨습니다' 를 잘못 누른 것이
    다음 상태로 새어 들어가면 안 된다.
    """

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._pending: list[tuple[str, dict]] = []
        self.allowed: tuple[str, ...] = ()

    def post(self, event: str, **data) -> tuple[bool, str]:
        with self._cv:
            if event not in self.allowed:
                return False, f"'{event}' 는 지금 받을 수 없습니다"
            self._pending.append((event, data))
            self._cv.notify_all()
        return True, ""

    def wait(self, allowed, timeout: float) -> tuple[str, dict]:
        allowed = tuple(allowed)
        with self._cv:
            self.allowed = allowed
            self._pending = [(e, d) for e, d in self._pending if e in allowed]
            if not self._cv.wait_for(lambda: bool(self._pending), timeout=timeout):
                self.allowed = ()
                return TIMEOUT, {}
            ev, data = self._pending.pop(0)
            self.allowed = ()
            return ev, data


class ScriptedGate:
    """시험용. 미리 적어 둔 사건을 차례로 내준다."""

    def __init__(self, script) -> None:
        self.script = list(script)
        self.asked: list[tuple[str, ...]] = []

    def post(self, event: str, **data):
        self.script.append((event, data))
        return True, ""

    def wait(self, allowed, timeout: float) -> tuple[str, dict]:
        allowed = tuple(allowed)
        self.asked.append(allowed)
        while self.script:
            item = self.script.pop(0)
            ev, data = item if isinstance(item, tuple) else (item, {})
            if ev == TIMEOUT or ev in allowed:
                return ev, data
        return TIMEOUT, {}


# ── 결과 ──────────────────────────────────────────────────────────────────
@dataclass
class Outcome:
    state: str
    floor: str | None            # 끝났을 때 서 있는 층 (모르면 None)
    reason: str = ""
    localized: bool = False
    needs_manual_start: bool = False   # UI 에서 층·위치를 다시 잡아야 하는가
    history: list[tuple[str, str, str]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.state == DONE


@dataclass
class Config:
    call_timeout: float = 300.0     # 엘리베이터를 부를 때까지
    board_timeout: float = 300.0    # 태워 줄 때까지
    ride_timeout: float = 600.0     # 목표 층에 설 때까지
    exit_timeout: float = 300.0     # 내려놓을 때까지
    confirm_timeout: float = 300.0  # 층을 골라 줄 때까지
    max_rides: int = 2              # 엉뚱한 층에 내렸을 때 다시 타 볼 횟수


class FloorTransfer:
    """한 번의 층 전환. run() 은 임무 스레드에서 돈다 (블로킹)."""

    def __init__(self, book, effects, gate=None, config: Config | None = None,
                 log=None) -> None:
        self.book = book
        self.fx = effects
        self.gate = gate if gate is not None else Gate()
        self.cfg = config or Config()
        self.state = IDLE
        self.history: list[tuple[str, str, str]] = []
        self.here: str | None = None      # 지금 서 있다고 믿는 층
        self.target: str | None = None    # 가야 하는 층
        self.destination: str | None = None
        self.reason = ""
        self.rides = 0
        self._log = log or (lambda msg: None)

    # ── 전이 ──────────────────────────────────────────────────────────────
    def _fire(self, event: str) -> None:
        nxt = TRANSITIONS.get((self.state, event))
        if nxt is None:
            raise TransitionError(f"전이표에 없다: ({self.state}, {event})")
        self.history.append((self.state, event, nxt))
        self._log(f"[층전환] {self.state} --{event}--> {nxt}")
        self.state = nxt

    def _say(self, message: str, speak: str = "", actions=(), detail: str = "") -> None:
        self.fx.announce(
            state=self.state,
            message=message,
            speak=speak,
            actions=tuple(actions),
            target=self.target,
            here=self.here,
            detail=detail,
        )

    def _ask(self, allowed, timeout: float) -> tuple[str, dict]:
        return self.gate.wait(allowed, timeout)

    # ── 본체 ──────────────────────────────────────────────────────────────
    def run(self, here: str, target: str, destination: str | None = None) -> Outcome:
        self.here, self.target, self.destination = here, target, destination
        if here == target:
            return Outcome(DONE, here, "같은 층이라 층 전환이 필요 없다", localized=True,
                           history=self.history)
        if self.book.get(target) is None:
            return self._fault(f"'{target}' 는 모르는 층이다")
        if self.book.get(here) is None:
            return self._fault(f"'{here}' 는 모르는 층이다")

        try:
            self._fire(START)
            while self.state not in TERMINAL:
                self._fire(self._HANDLERS[self.state](self))
        except TransitionError as e:
            return self._fault(f"상태머신 오류: {e}")
        except Exception as e:                      # 어떤 예외도 로봇을 세운다
            return self._fault(f"층 전환 중 예외: {e}")

        return self._finish()

    def _finish(self) -> Outcome:
        if self.state == DONE:
            return Outcome(DONE, self.here, self.reason, localized=True,
                           history=self.history)
        if self.state == FAULT:
            # 전이표를 타고 fault 에 들어온 경우 (map_fail·pose_fail·timeout 등).
            # _fault() 와 같은 뒤처리를 반드시 거친다 — 조용히 끝나면 안 된다.
            return self._enter_fault(self.reason or "층 전환에 실패했다")

        # aborted 는 초기 위치를 잃은 채 끝날 수 있다. 태워진 뒤에 취소했다면
        # 로봇은 자기가 어디인지 모른다.
        lost = any(h[0] in (BOARDING, RIDING, EXITING, CONFIRM_FLOOR,
                            SWAP_MAP, RELOCALIZE)
                   for h in self.history)
        msg = "층 이동을 취소했습니다"
        if lost:
            msg += ". 로봇을 내려놓으신 뒤 화면에서 층과 위치를 골라 주세요"
            self.fx.set_localized(False)
        self.fx.hold()
        self._say(msg, msg, actions=(), detail=self.reason)
        return Outcome(ABORTED, None if lost else self.here, self.reason,
                       localized=not lost, needs_manual_start=lost,
                       history=self.history)

    def _enter_fault(self, reason: str) -> Outcome:
        """어떤 실패든 여기로 온다. 로봇을 세우고, 초기 위치를 무효로 하고,
        화면과 음성으로 무엇이 잘못됐는지 말한다. **조용히 넘어가지 않는다.**

        초기 위치를 무효로 하는 것이 핵심이다. 그래야 guide_node 가 그 뒤의
        어떤 목적지 요청도 거부한다 — 지도 교체가 실패했는데 계속 주행하면
        다른 층 지도로 길을 찾는다.
        """
        self.reason = reason
        self.state = FAULT
        try:
            self.fx.hold()
            self.fx.set_localized(False)
        except Exception:
            pass
        self._say(
            f"층 이동을 멈췄습니다: {reason}",
            "층 이동을 멈췄습니다. 화면을 확인해 주세요",
            actions=(),
            detail="화면에서 지금 층과 로봇 위치를 골라 주면 다시 움직입니다",
        )
        return Outcome(FAULT, None, reason, localized=False, needs_manual_start=True,
                       history=self.history)

    def _fault(self, reason: str) -> Outcome:
        return self._enter_fault(reason)

    # ── 상태별 처리 ────────────────────────────────────────────────────────
    def _h_to_elevator(self) -> str:
        fl = self.book.get(self.here)
        tf = self.book.get(self.target)
        wp = fl.board
        self._say(
            f"{ro(tf.label)} 가기 위해 엘리베이터 앞으로 이동합니다",
            f"{ro(tf.label)} 가겠습니다. 엘리베이터 앞으로 이동합니다",
            actions=(ABORT,),
        )
        ok, err = self.fx.navigate(fl.key, wp)
        if not ok:
            self.reason = f"엘리베이터 앞({fl.label_of(wp)})까지 가지 못했다: {err}"
            return NAV_FAIL
        return NAV_OK

    def _h_at_elevator(self) -> str:
        tf = self.book.get(self.target)
        self.fx.hold()
        self._say(
            f"엘리베이터 앞에 도착했습니다. {tf.label} 버튼을 눌러 주세요",
            f"엘리베이터 앞입니다. {tf.label} 버튼을 눌러 주세요",
            actions=(CALLED, ABORT),
            detail="로봇은 버튼을 누르지 못합니다. 눌러 주신 뒤 아래를 눌러 주세요",
        )
        ev, _ = self._ask((CALLED, ABORT), self.cfg.call_timeout)
        if ev == TIMEOUT:
            self.reason = "엘리베이터를 불러 주기를 기다리다 시간이 지났다"
        return ev

    def _h_boarding(self) -> str:
        tf = self.book.get(self.target)
        # 여기서부터 로봇은 자기 위치를 모른다. 태워지는 순간 AMCL 은 헛것을 본다.
        self.fx.set_localized(False)
        self._say(
            f"문이 열리면 로봇을 안으로 넣어 주세요. {ro(tf.label)} 갑니다",
            "문이 열리면 로봇을 안으로 넣어 주세요",
            actions=(BOARDED, ABORT),
            detail="로봇은 스스로 타지 않습니다. 밀어서 넣어 주세요",
        )
        ev, _ = self._ask((BOARDED, ABORT), self.cfg.board_timeout)
        if ev == TIMEOUT:
            self.reason = "로봇을 태워 주기를 기다리다 시간이 지났다"
        return ev

    def _h_riding(self) -> str:
        tf = self.book.get(self.target)
        self.rides += 1
        # nav2 에 목표가 없다. 로봇은 아무 데도 안 간다 — 그게 이 구간의 전부다.
        self.fx.hold()
        self._say(
            f"{ro(tf.label)} 가는 중입니다. 도착하면 눌러 주세요",
            f"{tf.label}에서 내려 주세요",
            actions=(ARRIVED, ABORT),
            detail="이동 중에는 로봇이 스스로 움직이지 않습니다",
        )
        ev, _ = self._ask((ARRIVED, ABORT), self.cfg.ride_timeout)
        if ev == TIMEOUT:
            self.reason = "엘리베이터 안에서 기다리다 시간이 지났다"
        return ev

    def _h_exiting(self) -> str:
        tf = self.book.get(self.target)
        self._say(
            f"{tf.label} 입니다. 로봇을 엘리베이터 밖으로 내려 주세요",
            "로봇을 내려 주세요",
            actions=(EXITED, ABORT),
            detail=f"내려놓는 자리는 '{tf.label_of(tf.exit_wp)}' 입니다",
        )
        ev, _ = self._ask((EXITED, ABORT), self.cfg.exit_timeout)
        if ev == TIMEOUT:
            self.reason = "로봇을 내려 주기를 기다리다 시간이 지났다"
        return ev

    def _h_confirm_floor(self) -> str:
        guess = self.fx.detect_floor()
        tf = self.book.get(self.target)

        if guess.known and guess.confident:
            self.here = guess.floor
            here = self.book.get(self.here)
            self._say(f"{here.label} 으로 확인했습니다", "", actions=(),
                      detail=guess.reason)
            return FLOOR_OK

        # 자동으로 모른다. **찍지 않는다** — 사람에게 묻는다.
        hint = guess.reason or "층을 알 수 없습니다"
        self._say(
            f"지금 몇 층인가요? {hint}",
            "지금 몇 층인지 화면에서 골라 주세요",
            actions=(PICK_FLOOR, ABORT),
            detail=f"가려던 층은 {tf.label} 입니다",
        )
        ev, data = self._ask((PICK_FLOOR, ABORT), self.cfg.confirm_timeout)
        if ev == TIMEOUT:
            self.reason = "층을 골라 주기를 기다리다 시간이 지났다"
            return TIMEOUT
        if ev == ABORT:
            return ABORT
        picked = data.get("floor")
        if self.book.get(picked) is None:
            self.reason = f"'{picked}' 는 모르는 층이다"
            return TIMEOUT       # fault 로 간다
        self.here = picked
        return FLOOR_OK

    def _h_swap_map(self) -> str:
        """지도를 갈아끼운다.

        순서가 중요하다. **waypoint 를 먼저 읽고 지도를 나중에 바꾼다.**
        반대로 하면 지도만 새 층이고 waypoint 는 옛 층인 상태가 생기는데,
        그러면 목표가 벽 너머에 찍힌다.
        """
        fl = self.book.get(self.here)
        self._say(f"{fl.label} 지도를 불러오는 중입니다", "", actions=())

        ok, err = self.book.reload_waypoints(fl.key)
        if not ok:
            self.reason = err
            return MAP_FAIL

        ok, err = self.fx.load_map(fl)
        if not ok:
            self.reason = f"{fl.label} 지도를 불러오지 못했다: {err}"
            return MAP_FAIL
        return MAP_OK

    def _h_relocalize(self) -> str:
        """지도를 갈았으면 초기 위치를 반드시 다시 준다.

        안 주면 AMCL 이 map->odom 을 안 낸다. 그러면 frame_id=map 목표를
        변환할 수 없어 **어떤 목적지도 안 가는데 로그는 조용하다.**
        """
        fl = self.book.get(self.here)
        wp = fl.exit_wp
        self._say(f"{fl.label} 에서 위치를 다시 잡는 중입니다", "", actions=())
        ok, err = self.fx.relocalize(fl, wp)
        if not ok:
            self.reason = f"{fl.label} 에서 위치를 다시 잡지 못했다: {err}"
            return POSE_FAIL
        self.fx.set_localized(True)
        return POSE_OK

    def _h_landed(self) -> str:
        fl = self.book.get(self.here)
        if self.here == self.target:
            self._say(f"{fl.label} 에 도착했습니다", f"{fl.label}에 도착했습니다",
                      actions=())
            return SAME_FLOOR
        tf = self.book.get(self.target)
        if self.rides >= self.cfg.max_rides:
            self.reason = (f"{ro(tf.label)} 가려 했는데 {fl.label}입니다. "
                           f"{self.rides}번 시도해 그만둡니다")
            return GIVE_UP
        # 층이 어긋났다. 지금 층 지도로 이미 갈아탔으므로 로봇은 제 위치를 안다.
        # 다시 태워 달라고 한다.
        self._say(
            f"{ro(tf.label)} 가려 했는데 {fl.label}입니다. 다시 태워 주세요",
            f"{fl.label}입니다. {ro(tf.label)} 다시 태워 주세요",
            actions=(),
        )
        return WRONG_FLOOR

    def _h_resume(self) -> str:
        if not self.destination:
            self.reason = "층 이동만 하고 마칩니다"
            return NAV_OK
        fl = self.book.get(self.here)
        if self.destination not in fl.waypoints:
            self.reason = (f"{fl.label} 에 '{self.destination}' 이 없다 "
                           f"— 목적지가 다른 층 것이다")
            return NAV_FAIL
        self._say(
            f"{ro(fl.label_of(self.destination))} 안내를 이어 갑니다",
            f"{ro(fl.label_of(self.destination))} 안내하겠습니다. 뒤따라와 주세요",
            actions=(ABORT,),
        )
        ok, err = self.fx.resume(self.destination)
        if not ok:
            self.reason = err
            return NAV_FAIL
        return NAV_OK

    _HANDLERS = {
        TO_ELEVATOR: _h_to_elevator,
        AT_ELEVATOR: _h_at_elevator,
        BOARDING: _h_boarding,
        RIDING: _h_riding,
        EXITING: _h_exiting,
        CONFIRM_FLOOR: _h_confirm_floor,
        SWAP_MAP: _h_swap_map,
        RELOCALIZE: _h_relocalize,
        LANDED: _h_landed,
        RESUME: _h_resume,
    }

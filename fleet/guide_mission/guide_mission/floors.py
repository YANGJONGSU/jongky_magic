#!/usr/bin/env python3
"""층별 자원 대장 — 어느 층에 어떤 지도와 waypoint 가 짝인지.

층이 갈리면 **지도도 waypoint 도 따로**다. `map` 프레임은 2D 평면이라 10층과
11층을 한 지도에 못 담는다. 지금까지 그 짝을 맞추는 것은 사람이 명령줄에
`map:=fastcampus_10f.yaml waypoints:=~/waypoints_10f.yaml` 을 손으로 치는
일이었고, **틀려도 아무도 알려주지 않았다.** 10층 지도에 11층 waypoint 를
얹으면 목표는 접수되고 경로도 나온다 — 벽 너머로.

그래서 짝을 파일 하나(floors.yaml)에 적고, 여기서 검증한다.

    floors:
      "10f":
        label: "10층"
        map: ~/jongky_ws/src/jongky_navigation/maps/fastcampus_10f.yaml
        waypoints: ~/waypoints_10f.yaml
        ssid: [FASTCAMPUS_10F]
        elevator: {board: ev1, exit: ev1}
        labels: {ev1: "엘리베이터 앞", 10a: "304호 강의장"}

검증은 ROS 없이 돈다. 현장에 가기 전에 노트북에서

    ros2 run guide_mission check_floors ~/floors.yaml
    python3 -m guide_mission.check_floors ~/floors.yaml

로 돌려 보면 짝이 어긋난 것을 미리 잡는다.

[짝 검증을 어떻게 하는가]
지도 YAML 의 `origin`·`resolution` 과 이미지(pgm/png) 헤더의 픽셀 크기로
지도가 덮는 사각형을 구하고, waypoint 가 그 안에 있는지 본다. 다른 층
waypoint 를 얹으면 좌표가 통째로 지도 밖으로 나가므로 바로 걸린다.
같은 건물 같은 크기의 두 층이면 안 걸릴 수도 있다 — 그건 검사의 한계이고,
그래서 층 판정(detect.py)과 사람 확인이 따로 있다.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml

ERROR = "error"
WARN = "warn"

# 맵핑 주행 중 터미널 버퍼가 새어 들어간 이름들. teleop_key.py:228 참조.
# 예: `wwwwwwwwwwwwwwwwwev2`, `.,,`
_JUNK = ".,;'\"/"


@dataclass(frozen=True)
class Problem:
    level: str      # error | warn
    floor: str
    text: str

    def __str__(self) -> str:
        mark = "✗" if self.level == ERROR else "!"
        return f"{mark} [{self.floor}] {self.text}"


@dataclass(frozen=True)
class Waypoint:
    name: str
    label: str
    x: float
    y: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = "map"


@dataclass(frozen=True)
class MapInfo:
    yaml_path: str
    image_path: str
    width: int          # 픽셀
    height: int         # 픽셀
    resolution: float   # m/픽셀
    origin_x: float
    origin_y: float

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """(xmin, ymin, xmax, ymax) — 지도가 덮는 사각형 [m]."""
        return (
            self.origin_x,
            self.origin_y,
            self.origin_x + self.width * self.resolution,
            self.origin_y + self.height * self.resolution,
        )

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        x0, y0, x1, y1 = self.bounds
        return (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin)


@dataclass
class Floor:
    key: str
    label: str
    map_path: str
    waypoint_path: str
    ssids: tuple[str, ...] = ()
    board: str = ""             # 엘리베이터를 타러 가는 지점
    exit_wp: str = ""           # 내려서 서 있게 되는 지점 (보통 board 와 같다)
    labels: dict[str, str] = field(default_factory=dict)
    order: int = 0
    waypoints: dict[str, Waypoint] = field(default_factory=dict)
    map_info: MapInfo | None = None
    problems: list[Problem] = field(default_factory=list)

    @property
    def errors(self) -> list[Problem]:
        return [p for p in self.problems if p.level == ERROR]

    @property
    def usable(self) -> bool:
        """이 층으로 주행을 시작해도 되는가."""
        return not self.errors

    def label_of(self, name: str) -> str:
        return self.labels.get(name, name)

    def destinations(self) -> list[dict]:
        """UI 가 그릴 목록. 엘리베이터 지점은 목적지가 아니라 경유지다."""
        out = []
        for name in self.waypoints:
            if name in (self.board, self.exit_wp):
                continue
            out.append({"name": name, "label": self.label_of(name)})
        return out


# ── 지도 읽기 ─────────────────────────────────────────────────────────────
def _image_size(path: str) -> tuple[int, int]:
    """pgm/png 헤더에서 픽셀 크기만 뽑는다. PIL 없이 돈다 (젯슨 컨테이너의
    numpy/PIL ABI 충돌과 무관하게 검증이 돌아야 한다)."""
    with open(path, "rb") as f:
        head = f.read(64)
        if head[:8] == b"\x89PNG\r\n\x1a\n":
            w = int.from_bytes(head[16:20], "big")
            h = int.from_bytes(head[20:24], "big")
            return w, h
        if head[:2] not in (b"P5", b"P2"):
            raise ValueError(f"pgm/png 이 아니다 (매직 {head[:2]!r})")
        # P5\n[# 주석]\nW H\nMAX\n — 주석과 공백이 섞일 수 있어 토큰으로 센다
        f.seek(2)
        toks: list[bytes] = []
        buf = b""
        while len(toks) < 2:
            ch = f.read(1)
            if not ch:
                raise ValueError("pgm 헤더가 잘렸다")
            if ch == b"#":
                while ch not in (b"\n", b""):
                    ch = f.read(1)
                continue
            if ch.isspace():
                if buf:
                    toks.append(buf)
                    buf = b""
                continue
            buf += ch
        return int(toks[0]), int(toks[1])


def read_map(yaml_path: str) -> MapInfo:
    """지도 YAML + 이미지 헤더 → MapInfo. 실패하면 예외를 던진다."""
    yaml_path = os.path.expanduser(yaml_path)
    with open(yaml_path) as f:
        doc = yaml.safe_load(f) or {}
    image = doc.get("image")
    if not image:
        raise ValueError("지도 YAML 에 image 항목이 없다")
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    if not os.path.exists(image):
        raise FileNotFoundError(f"지도 이미지가 없다: {image}")
    res = float(doc.get("resolution", 0.0))
    if res <= 0:
        raise ValueError(f"resolution 이 이상하다: {doc.get('resolution')}")
    origin = doc.get("origin") or [0.0, 0.0, 0.0]
    w, h = _image_size(image)
    return MapInfo(yaml_path, image, w, h, res, float(origin[0]), float(origin[1]))


# ── waypoint 읽기 ─────────────────────────────────────────────────────────
def read_waypoints(path: str, labels: dict[str, str] | None = None) -> dict[str, Waypoint]:
    """teleop_key.py 가 찍어 둔 YAML → Waypoint 사전. 실패하면 예외."""
    path = os.path.expanduser(path)
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    labels = labels or {}
    out: dict[str, Waypoint] = {}
    for name, w in doc.items():
        pos = (w or {}).get("position") or {}
        ori = (w or {}).get("orientation") or {}
        if "x" not in pos or "y" not in pos:
            raise ValueError(f"waypoint '{name}' 에 position 이 없다")
        out[name] = Waypoint(
            name=name,
            label=labels.get(name, name),
            x=float(pos["x"]),
            y=float(pos["y"]),
            qx=float(ori.get("x", 0.0)),
            qy=float(ori.get("y", 0.0)),
            qz=float(ori.get("z", 0.0)),
            qw=float(ori.get("w", 1.0)),
            frame_id=(w or {}).get("frame_id", "map"),
        )
    return out


# ── 검증 ──────────────────────────────────────────────────────────────────
def validate(floor: Floor, margin: float = 1.0) -> list[Problem]:
    """지도·waypoint 를 읽어 floor 에 채우고 문제 목록을 돌려준다.

    error 가 하나라도 있으면 그 층으로는 주행을 시작하지 않는다.
    """
    ps: list[Problem] = []
    key = floor.key

    # 1) 지도
    try:
        floor.map_info = read_map(floor.map_path)
    except FileNotFoundError as e:
        ps.append(Problem(ERROR, key, f"지도를 못 읽는다: {e}"))
    except Exception as e:
        ps.append(Problem(ERROR, key, f"지도 YAML 이 이상하다: {e}"))

    # 2) waypoint
    try:
        floor.waypoints = read_waypoints(floor.waypoint_path, floor.labels)
    except FileNotFoundError:
        ps.append(Problem(ERROR, key,
                          f"waypoint 파일이 없다: {floor.waypoint_path} — 맵핑 주행부터 할 것"))
    except Exception as e:
        ps.append(Problem(ERROR, key, f"waypoint YAML 이 이상하다: {e}"))

    if not floor.waypoints:
        if not any(p.level == ERROR for p in ps):
            ps.append(Problem(ERROR, key, "waypoint 가 하나도 없다"))
        floor.problems = ps
        return ps

    # 3) 엘리베이터 지점이 실제로 있는가.
    #    이게 없으면 이 층에서는 층 전환을 시작할 수도, 이 층으로 내릴 수도 없다.
    for role, name in (("board(탑승)", floor.board), ("exit(하차)", floor.exit_wp)):
        if not name:
            ps.append(Problem(ERROR, key, f"elevator.{role} 지점이 floors.yaml 에 없다"))
        elif name not in floor.waypoints:
            ps.append(Problem(
                ERROR, key,
                f"elevator.{role} 로 적은 '{name}' 이 waypoint 파일에 없다 "
                f"(있는 이름: {', '.join(list(floor.waypoints)[:8])})"))

    # 4) 짝 검사 — waypoint 가 이 지도 안에 있는가
    if floor.map_info is not None:
        outside = [w for w in floor.waypoints.values()
                   if not floor.map_info.contains(w.x, w.y, margin)]
        if outside:
            x0, y0, x1, y1 = floor.map_info.bounds
            names = ", ".join(w.name for w in outside[:6])
            level = ERROR if len(outside) == len(floor.waypoints) else WARN
            ps.append(Problem(
                level, key,
                f"waypoint {len(outside)}/{len(floor.waypoints)} 개가 지도 밖이다 "
                f"({names}). 지도 범위 x[{x0:.1f},{x1:.1f}] y[{y0:.1f},{y1:.1f}] — "
                f"**지도와 waypoint 의 층이 어긋난 것 아닌지 확인할 것**"))
            # 엘리베이터 지점이 밖이면 층 전환 자체가 성립하지 않는다
            for w in outside:
                if w.name in (floor.board, floor.exit_wp):
                    ps.append(Problem(ERROR, key,
                                      f"엘리베이터 지점 '{w.name}' 이 지도 밖이다"))

    # 5) 오염된 이름 — 맵핑 중 터미널 버퍼가 샌 것
    for name in floor.waypoints:
        if name.startswith("w") and name.count("w") > 3:
            ps.append(Problem(WARN, key,
                              f"waypoint 이름이 오염된 것 같다: '{name}' "
                              f"(teleop 의 w 연타가 샌 흔적). labels 로 표시 이름을 줄 것"))
        elif all(c in _JUNK for c in name):
            ps.append(Problem(WARN, key, f"waypoint 이름이 기호뿐이다: '{name}'"))
        elif name not in floor.labels and not any(c.isalnum() for c in name):
            ps.append(Problem(WARN, key, f"사람이 못 읽을 waypoint 이름이다: '{name}'"))

    floor.problems = ps
    return ps


@dataclass
class FloorBook:
    """floors.yaml 한 권. 층 키 → Floor."""

    floors: dict[str, Floor] = field(default_factory=dict)
    hotspot_ssids: tuple[str, ...] = ()
    source: str = ""

    # ── 만들기 ────────────────────────────────────────────────────────────
    @staticmethod
    def load(path: str, validate_now: bool = True) -> "FloorBook":
        path = os.path.expanduser(path)
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        return FloorBook.from_dict(doc, source=path, validate_now=validate_now)

    @staticmethod
    def from_dict(doc: dict, source: str = "", validate_now: bool = True) -> "FloorBook":
        raw = doc.get("floors") or {}
        if not raw:
            raise ValueError("floors: 항목이 비어 있다")
        book = FloorBook(
            hotspot_ssids=tuple(doc.get("hotspot_ssids") or ()),
            source=source,
        )
        for i, (key, f) in enumerate(raw.items()):
            f = f or {}
            ev = f.get("elevator") or {}
            board = ev.get("board", "")
            book.floors[str(key)] = Floor(
                key=str(key),
                label=f.get("label", str(key)),
                map_path=os.path.expanduser(f.get("map", "")),
                waypoint_path=os.path.expanduser(f.get("waypoints", "")),
                ssids=tuple(f.get("ssid") or ()),
                board=board,
                exit_wp=ev.get("exit", board),
                labels=dict(f.get("labels") or {}),
                order=int(f.get("order", i)),
            )
        if validate_now:
            book.validate()
        return book

    # ── 쓰기 ──────────────────────────────────────────────────────────────
    def validate(self, margin: float = 1.0) -> list[Problem]:
        ps: list[Problem] = []
        for fl in self.ordered():
            ps.extend(validate(fl, margin))
        # 층이 하나뿐이면 층 전환이 성립하지 않는다 — 쓸 수는 있으나 알려 준다
        if len(self.floors) < 2:
            ps.append(Problem(WARN, "-", "층이 하나뿐이라 층 전환은 일어나지 않는다"))
        return ps

    @property
    def problems(self) -> list[Problem]:
        return [p for fl in self.ordered() for p in fl.problems]

    def ordered(self) -> list[Floor]:
        return sorted(self.floors.values(), key=lambda f: (f.order, f.key))

    def get(self, key: str) -> Floor | None:
        return self.floors.get(key)

    def by_ssid(self, ssid: str) -> Floor | None:
        if not ssid:
            return None
        low = ssid.strip().lower()
        for fl in self.ordered():
            if any(s.strip().lower() == low for s in fl.ssids):
                return fl
        return None

    def find_destination(self, name: str) -> list[Floor]:
        """이 이름의 목적지를 가진 층들. 여러 층에 같은 이름이 있으면 둘 다."""
        return [fl for fl in self.ordered() if name in fl.waypoints]

    def reload_waypoints(self, key: str) -> tuple[bool, str]:
        """층 전환 직전에 waypoint 를 다시 읽는다.

        지도를 갈아끼우기 **전에** 부른다. 여기서 실패하면 지도를 안 건드리고
        멈춘다 — 지도만 바뀌고 waypoint 가 옛 층이면 벽 너머로 길을 찾는다.
        """
        fl = self.floors.get(key)
        if fl is None:
            return False, f"'{key}' 는 모르는 층이다"
        try:
            wps = read_waypoints(fl.waypoint_path, fl.labels)
        except Exception as e:
            return False, f"{fl.label} waypoint 를 못 읽는다: {e}"
        if not wps:
            return False, f"{fl.label} waypoint 가 비었다"
        if fl.exit_wp and fl.exit_wp not in wps:
            return False, f"{fl.label} 에 하차 지점 '{fl.exit_wp}' 이 없다"
        fl.waypoints = wps
        return True, ""

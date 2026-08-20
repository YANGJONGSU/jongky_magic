#!/usr/bin/env python3
"""waypoint YAML 검사·이관 도구.

teleop_key.py 가 맵핑 주행 중에 찍은 waypoint 파일을 읽어

  1. 오염된 이름을 찾아 보고하고 (제어문자, 반복 문자, 비정상 길이, 기호만 남은 것)
  2. 좌표를 검증하고 (중복·근접·frame_id·쿼터니언·터무니없는 값)
  3. guide_interfaces 의 POI 스키마로 이관한다 (display_name/kind/floor 추가)

**원본은 절대 고치지 않는다.** 결과는 --out-dir (기본 /tmp) 에 새 파일로 쓴다.

실행:
    waypoint_doctor.py ~/waypoints_10f.yaml ~/waypoints_11f.yaml
    waypoint_doctor.py --check-only ~/waypoints_*.yaml     # CI 용. 문제가 있으면 종료코드 1
    waypoint_doctor.py --rename-corrupt ~/waypoints_10f.yaml

[왜 이런 도구가 필요한가]
오염의 원인 자체는 teleop_key.prompt() 의 버퍼 플러시였고 고쳤다. 하지만 이미
찍힌 파일은 그대로다. 그리고 오염보다 더 오래 가는 문제가 있다 — waypoint 이름이
내부 코드('10a' 'ev1' 'm1')라서 방문객이 말할 수도, 버튼에서 알아볼 수도 없다.
이관은 그 코드를 좌표 키로 남겨 두고 사람이 읽을 이름을 따로 붙이는 일이다.

[무엇을 사람이 정해야 하나]
이 도구는 짐작하되 짐작을 숨기지 않는다. 이름에서 종류를 뽑는 규칙은 층마다
사람마다 다르고('m' 이 남자화장실인지 회의실인지 파일만 봐서는 모른다), 이름이
깨진 항목은 원래 뭐였는지 찍은 사람만 안다. 짐작한 것은 전부 REVIEW 목록에
올라가고, 이관 파일에도 짐작한 값이 그대로 적혀 사람이 고칠 수 있다.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
import unicodedata
from dataclasses import dataclass

import yaml

# ── 판정 기준 ─────────────────────────────────────────────────────────────
# 이름 길이. 실제 쓰이는 이름은 '10a' 'exit11-1' 정도라 24자면 넉넉하다.
MAX_NAME_LEN = 24
# 같은 글자가 이만큼 이어지면 키가 눌린 채 흘러들어간 것으로 본다.
REPEAT_RUN = 3
# 이보다 가까운 두 지점은 사람이 구분해서 고를 수 없다. 안내로봇의 도착 판정
# 반경(Nav2 xy_goal_tolerance 기본 0.25m)의 네 배쯤을 기준으로 잡았다.
MIN_SEPARATION_M = 1.0
# 완전히 같은 지점으로 보는 거리.
SAME_POINT_M = 0.05
# 실내 한 층의 지도가 이보다 클 수 없다. 넘으면 TF 를 잘못 읽은 것이다.
MAX_ABS_COORD_M = 200.0
# 쿼터니언 크기 허용 오차. teleop 이 소수점 6자리로 반올림해 저장한다.
QUAT_TOL = 1e-3

# teleop_key.py 의 조작키. 버퍼에 남아 이름 앞에 새는 것이 이 글자들이다.
TELEOP_KEYS = set("i,jloukzxcvwphq")

# ── 종류 추론 규칙 ────────────────────────────────────────────────────────
# (정규식, 종류, 확신, 표시이름 생성기)
# 확신 'high' 는 그대로 써도 되는 것, 'low' 는 사람이 봐야 하는 것이다.
KIND_ETC, KIND_LECTURE, KIND_ELEVATOR, KIND_ENTRANCE, KIND_RESTROOM = (
    "etc", "lecture", "elevator", "entrance", "restroom",
)


def _num(s: str) -> str:
    return s if s else ""


RULES = [
    # ev1, ev2, ev3 — 세 층에 걸쳐 일관되게 쓰였다
    (re.compile(r"^ev(\d*)$", re.I), KIND_ELEVATOR, "high",
     lambda m: f"엘리베이터 {m.group(1)}".strip()),
    # exit1..exit4, exit11-1, exit11-2
    (re.compile(r"^exit[\-_]?(\d+(?:-\d+)?)?$", re.I), KIND_ENTRANCE, "high",
     lambda m: f"출입구 {_num(m.group(1) or '')}".strip()),
    # 10a 10b 10c 10e 11a..11e 10c1 — 층번호 + 알파벳 = 강의장
    (re.compile(r"^(\d{1,2})([a-eA-E])(\d*)$"), KIND_LECTURE, "medium",
     lambda m: f"{m.group(1)}층 {m.group(2).upper()} 강의장"
               + (f"-{m.group(3)}" if m.group(3) else "")),
    # m1 m2 / w1 w2 / 11w — 남녀 화장실로 **추정**한다. 파일만 봐서는 모른다.
    (re.compile(r"^m(\d*)$", re.I), KIND_RESTROOM, "low",
     lambda m: "남자 화장실"),
    (re.compile(r"^w(\d*)$", re.I), KIND_RESTROOM, "low",
     lambda m: "여자 화장실"),
    (re.compile(r"^(\d{1,2})m$", re.I), KIND_RESTROOM, "low",
     lambda m: f"{m.group(1)}층 남자 화장실"),
    (re.compile(r"^(\d{1,2})w$", re.I), KIND_RESTROOM, "low",
     lambda m: f"{m.group(1)}층 여자 화장실"),
]


@dataclass
class Issue:
    """한 건의 문제. code 로 기계가 세고, text 로 사람이 읽는다."""

    code: str
    name: str
    text: str
    # True 면 도구가 못 정한다 — 사람이 답을 알아야 한다.
    needs_human: bool = False


@dataclass
class Entry:
    key: str                 # YAML 원본 키
    raw: dict
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    frame_id: str = ""
    ok_geometry: bool = True
    suggested_id: str = ""
    display_name: str = ""
    kind: str = KIND_ETC
    confidence: str = "low"


# ── 이름 검사 ─────────────────────────────────────────────────────────────
def name_issues(name: str) -> list[Issue]:
    out: list[Issue] = []

    if not name.strip():
        out.append(Issue("NAME_EMPTY", name, "이름이 비어 있다", True))
        return out

    ctrl = [c for c in name if unicodedata.category(c) in ("Cc", "Cf", "Zl", "Zp")]
    if ctrl:
        codes = " ".join(f"U+{ord(c):04X}" for c in ctrl)
        out.append(Issue("NAME_CONTROL", name, f"제어문자 {len(ctrl)}개 ({codes})", True))

    for m in re.finditer(r"(.)\1{%d,}" % (REPEAT_RUN - 1), name):
        ch, n = m.group(1), len(m.group(0))
        hint = " — teleop 조작키가 샌 것" if ch.lower() in TELEOP_KEYS else ""
        out.append(Issue("NAME_REPEAT", name,
                         f"'{ch}' 가 {n}번 반복{hint}", True))
        break  # 한 이름에 하나만 보고하면 충분하다

    if len(name) > MAX_NAME_LEN:
        out.append(Issue("NAME_LONG", name, f"{len(name)}자 (기준 {MAX_NAME_LEN}자)", True))

    # 한글·영숫자가 하나도 없으면 이름이 아니다 ('.,,')
    if not re.search(r"[0-9A-Za-z가-힣]", name):
        out.append(Issue("NAME_NO_WORD", name, "글자가 하나도 없다 — 오타로 저장된 것", True))

    # 쓸 수 있는 글자 밖의 것. '10c\' 의 역슬래시가 여기 걸린다.
    bad = sorted({c for c in name if not re.match(r"[0-9A-Za-z가-힣 _\-.]", c)})
    if bad:
        out.append(Issue("NAME_CHARSET", name,
                         "쓸 수 없는 글자 " + " ".join(repr(c) for c in bad), True))
    return out


def clean_candidates(name: str) -> list[str]:
    """오염된 이름에서 원래 이름 후보를 **짐작**한다. 답이 아니라 제안이다.

      · 맨 앞의 반복 덩어리는 버린다 — 키를 누른 채 프롬프트에 들어간 흔적이라
        이름의 일부가 아니다 ('wwwww...ev2' -> 'ev2')
      · 나머지 반복은 한 글자로 줄인다 ('11eeeee...' -> '11e')

    맨 앞 덩어리를 통째로 버리는 게 항상 옳지는 않다. 반복된 글자가 숫자면
    그 숫자가 층 번호의 일부일 수 있다 — '1111111111111m' 은 'm' 일 수도
    '11m' 일 수도 있고, 같은 층에 '11w' 가 있으니 '11m' 이 더 그럴듯하다.
    그래서 하나로 정하지 않고 후보를 순서대로 돌려준다. 고르는 건 사람이다.
    """
    s = "".join(c for c in name if unicodedata.category(c) not in ("Cc", "Cf"))
    m = re.match(r"^(.)\1{%d,}" % (REPEAT_RUN - 1), s)
    rest = s[m.end():] if m else s
    rest = re.sub(r"(.)\1{%d,}" % (REPEAT_RUN - 1), r"\1", rest)

    def tidy(t: str) -> str:
        return t.strip().strip("\\/.,")

    out = [tidy(rest)]
    if m and m.group(1).isdigit():
        # 층 번호가 잘려 나갔을 수 있다. 한 자리·두 자리를 되살린 후보를 더한다.
        out += [tidy(m.group(1) + rest), tidy(m.group(1) * 2 + rest)]
    return [c for i, c in enumerate(out) if c and c not in out[:i]]


def clean_name(name: str) -> str:
    c = clean_candidates(name)
    return c[0] if c else ""


def infer(name: str) -> tuple[str, str, str]:
    """이름 -> (종류, 표시이름, 확신)."""
    for rx, kind, conf, label in RULES:
        m = rx.match(name)
        if m:
            return kind, label(m), conf
    return KIND_ETC, name, "low"


def infer_floor(name: str, path: str) -> tuple[int, str]:
    """층 -> (층, 근거). 파일 이름을 먼저 믿는다 — 파일이 층별로 나뉘어 있다."""
    m = re.search(r"(\d{1,2})\s*f\b", os.path.basename(path), re.I)
    if m:
        return int(m.group(1)), "파일명"
    m = re.match(r"^(\d{1,2})[a-zA-Z]", name)
    if m:
        return int(m.group(1)), "이름"
    return 0, "모름"


# ── 좌표 검사 ─────────────────────────────────────────────────────────────
def parse_entry(key: str, raw, issues: list[Issue]) -> Entry:
    e = Entry(key=key, raw=raw if isinstance(raw, dict) else {})
    if not isinstance(raw, dict):
        issues.append(Issue("GEOM_SHAPE", key, f"항목이 매핑이 아니다 ({type(raw).__name__})", True))
        e.ok_geometry = False
        return e

    pos, ori = raw.get("position"), raw.get("orientation")
    if not isinstance(pos, dict) or not all(k in pos for k in "xyz"):
        issues.append(Issue("GEOM_MISSING", key, "position 이 없거나 x/y/z 가 빠졌다", True))
        e.ok_geometry = False
    else:
        try:
            e.x, e.y, e.z = float(pos["x"]), float(pos["y"]), float(pos["z"])
        except (TypeError, ValueError):
            issues.append(Issue("GEOM_MISSING", key, "position 값이 숫자가 아니다", True))
            e.ok_geometry = False

    if not isinstance(ori, dict) or not all(k in ori for k in "xyzw"):
        issues.append(Issue("GEOM_MISSING", key, "orientation 이 없거나 x/y/z/w 가 빠졌다", True))
        e.ok_geometry = False
    else:
        try:
            e.quat = tuple(float(ori[k]) for k in "xyzw")
        except (TypeError, ValueError):
            issues.append(Issue("GEOM_MISSING", key, "orientation 값이 숫자가 아니다", True))
            e.ok_geometry = False

    # frame_id. guide_node._to_pose 는 없으면 'map' 으로 치지만, 다른 값이
    # 적혀 있으면 그대로 헤더에 실려 나가 Nav2 가 변환에 실패한다.
    e.frame_id = raw.get("frame_id", "")
    if e.frame_id == "":
        issues.append(Issue("GEOM_NO_FRAME", key, "frame_id 가 없다 (읽는 쪽이 map 으로 친다)"))
    elif e.frame_id != "map":
        issues.append(Issue("GEOM_FRAME", key,
                            f"frame_id 가 '{e.frame_id}' 다 — map 이어야 Nav2 목표가 된다", True))

    if e.ok_geometry:
        n = math.sqrt(sum(v * v for v in e.quat))
        if abs(n - 1.0) > QUAT_TOL:
            issues.append(Issue("GEOM_QUAT", key, f"쿼터니언 크기가 {n:.5f} 다 (1 이어야 한다)", True))
        if abs(e.z) > 1e-6:
            issues.append(Issue("GEOM_Z", key, f"z 가 {e.z} 다 — 평면 주행이라 0 이어야 한다"))
        if max(abs(e.x), abs(e.y)) > MAX_ABS_COORD_M:
            issues.append(Issue("GEOM_RANGE", key,
                                f"좌표가 ({e.x:.1f}, {e.y:.1f}) 로 지도 밖이다", True))
    return e


def cross_checks(entries: list[Entry], min_sep: float) -> list[Issue]:
    out: list[Issue] = []

    # 대소문자만 다른 키. YAML 은 다른 키지만 사람은 같은 이름으로 읽고,
    # 발화 매칭은 대소문자를 안 가리게 될 수도 있다.
    seen: dict[str, list[str]] = {}
    for e in entries:
        seen.setdefault(e.key.lower(), []).append(e.key)
    for low, keys in seen.items():
        if len(keys) > 1:
            out.append(Issue("DUP_CASE", keys[0],
                             "대소문자만 다른 이름: " + ", ".join(repr(k) for k in keys), True))

    # 부분문자열 관계. guide_node._resolve 와 voice_node._match 는
    # "이름이 발화 안에 들어 있는가" 를 **YAML 순서대로** 본다. 짧은 이름이
    # 먼저 나오면 긴 이름은 영원히 안 뽑힌다.
    for i, a in enumerate(entries):
        for j, b in enumerate(entries):
            if i == j or not a.key or not b.key:
                continue
            if a.key in b.key and i < j:
                out.append(Issue("MATCH_SHADOW", a.key,
                                 f"'{a.key}' 가 '{b.key}' 의 일부이고 먼저 온다 "
                                 f"— 발화 매칭에서 '{b.key}' 는 절대 안 뽑힌다", True))

    # 좌표 근접·중복
    good = [e for e in entries if e.ok_geometry]
    for i in range(len(good)):
        for j in range(i + 1, len(good)):
            a, b = good[i], good[j]
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d <= SAME_POINT_M:
                out.append(Issue("GEOM_SAME", a.key,
                                 f"'{a.key}' 와 '{b.key}' 가 같은 지점이다 ({d*100:.0f}cm)", True))
            elif d < min_sep:
                out.append(Issue("GEOM_NEAR", a.key,
                                 f"'{a.key}' 와 '{b.key}' 가 {d:.2f}m 밖에 안 떨어졌다", True))
    return out


# ── 이관 ──────────────────────────────────────────────────────────────────
def migrate(entries: list[Entry], path: str, rename: bool) -> tuple[dict, list[Issue]]:
    """새 스키마로 옮긴 매핑과, 사람이 정해야 할 것들을 돌려준다.

    최상위는 지금과 같은 '이름 -> 항목' 매핑을 유지한다. 리스트로 바꾸면
    guide_node 의 `self._waypoints[name]` 이 전부 깨진다.
    """
    reviews: list[Issue] = []
    out: dict = {}
    used: set = set()

    # 이름의 '모양'. 숫자를 #, 글자를 A 로 바꾼 것이다. 깨진 이름을 복원할 때
    # 같은 층 형제들과 모양이 맞는 후보를 고르는 데 쓴다 — '11w' 가 있으면
    # '1111111111111m' 은 'm' 보다 '11m' 일 가능성이 높다.
    def shape(s: str) -> str:
        return re.sub(r"[0-9]", "#", re.sub(r"[A-Za-z]", "A", s))

    clean_keys = [e.key for e in entries if not name_issues(e.key)]
    sibling_shapes = {shape(k) for k in clean_keys}

    for e in entries:
        corrupt = bool(name_issues(e.key))
        cands = clean_candidates(e.key) if corrupt else [e.key]
        base = next((c for c in cands if shape(c) in sibling_shapes), cands[0] if cands else "")
        if corrupt and len(cands) > 1:
            reviews.append(Issue("REVIEW_AMBIGUOUS", e.key,
                                 "복원 후보가 여럿이다: "
                                 + ", ".join(repr(c) for c in cands)
                                 + f" — '{base}' 를 골랐다"
                                 + (" (같은 층 다른 이름과 모양이 같다)"
                                    if shape(base) in sibling_shapes else ""),
                                 True))
        e.suggested_id = base
        e.kind, e.display_name, e.confidence = infer(base or e.key)
        floor, why = infer_floor(base or e.key, path)

        key = e.key
        if corrupt:
            if not base:
                reviews.append(Issue("REVIEW_NAME", e.key,
                                     "이름을 복원할 수 없다 — 좌표만 보고 사람이 새로 지어야 한다",
                                     True))
            elif base in {x.key for x in entries} or base in used:
                reviews.append(Issue("REVIEW_COLLIDE", e.key,
                                     f"복원하면 '{base}' 인데 그 이름이 이미 있다 "
                                     f"— 같은 지점인지 다른 지점인지 사람이 판단해야 한다",
                                     True))
            else:
                reviews.append(Issue("REVIEW_RENAME", e.key,
                                     f"'{base}' 로 보인다" + ("" if rename else " (--rename-corrupt 로 반영)"),
                                     True))
                if rename:
                    key = base

        if e.confidence != "high":
            reviews.append(Issue("REVIEW_KIND", e.key,
                                 f"종류를 '{e.kind}' 로, 이름을 '{e.display_name}' 로 짐작했다 "
                                 f"(확신 {e.confidence})", True))
        if floor == 0:
            reviews.append(Issue("REVIEW_FLOOR", e.key, "층을 못 정했다", True))

        while key in used:                       # 이관 중에 항목을 잃지 않는다
            key += "_dup"
        used.add(key)

        item = dict(e.raw)                       # 원본 필드는 하나도 안 버린다
        item.setdefault("frame_id", "map")
        item["display_name"] = e.display_name or key
        item["kind"] = e.kind
        item["floor"] = floor
        if "aliases" not in item:
            item["aliases"] = []
        out[key] = item

    return out, reviews


# ── 보고 ──────────────────────────────────────────────────────────────────
def render(path: str, entries: list[Entry], issues: list[Issue], reviews: list[Issue]) -> str:
    L: list[str] = []
    L.append(f"# {path}")
    L.append(f"  지점 {len(entries)}개")
    dirty = {i.name for i in issues if i.code.startswith("NAME_")}
    L.append(f"  이름이 오염된 지점 {len(dirty)}개, 문제 {len(issues)}건")
    L.append("")

    if issues:
        L.append("## 발견된 문제")
        for code in sorted({i.code for i in issues}):
            L.append(f"  [{code}]")
            for i in issues:
                if i.code == code:
                    # '!' 는 도구가 못 정하는 것 — 사람이 답을 알아야 한다
                    L.append(f"   {'!' if i.needs_human else ' '} {i.name!r}: {i.text}")
        L.append("")

    L.append("## 이관 결과 (짐작한 값이 들어 있다)")
    for e in entries:
        mark = "!" if e.confidence != "high" else " "
        L.append(f"  {mark} {e.key!r:45s} -> id={e.suggested_id!r} "
                 f"kind={e.kind} name={e.display_name!r}")
    L.append("")

    if reviews:
        L.append("## 사람이 정해야 하는 것")
        for code in ("REVIEW_NAME", "REVIEW_COLLIDE", "REVIEW_AMBIGUOUS", "REVIEW_RENAME",
                     "REVIEW_KIND", "REVIEW_FLOOR"):
            got = [r for r in reviews if r.code == code]
            if not got:
                continue
            L.append(f"  [{code}] {len(got)}건")
            for r in got:
                L.append(f"    {r.name!r}: {r.text}")
        L.append("")
    return "\n".join(L)


def process(path: str, out_dir: str, min_sep: float, rename: bool) -> tuple[str, int, int]:
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"{path}: 최상위가 '이름 -> 항목' 매핑이 아니다")

    issues: list[Issue] = []
    entries: list[Entry] = []
    for key, raw in data.items():
        key = key if isinstance(key, str) else str(key)
        issues.extend(name_issues(key))
        entries.append(parse_entry(key, raw, issues))
    issues.extend(cross_checks(entries, min_sep))

    migrated, reviews = migrate(entries, path, rename)

    stem = os.path.splitext(os.path.basename(path))[0]
    os.makedirs(out_dir, exist_ok=True)
    out_yaml = os.path.join(out_dir, f"{stem}.poi.yaml")
    with open(out_yaml, "w") as f:
        f.write("# waypoint_doctor.py 가 이관한 파일. kind/display_name/floor 는 **짐작**이다.\n"
                "# 같은 디렉터리의 보고서에서 REVIEW 항목을 보고 손으로 고칠 것.\n")
        yaml.safe_dump(migrated, f, allow_unicode=True, sort_keys=False)

    report = render(path, entries, issues, reviews)
    return report + f"  이관 파일: {out_yaml}\n", len(issues), len(reviews)


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("paths", nargs="+", help="검사할 waypoint YAML (원본은 안 고친다)")
    p.add_argument("--out-dir", default="/tmp/waypoint_doctor", help="이관 파일과 보고서를 쓸 곳")
    p.add_argument("--min-separation", type=float, default=MIN_SEPARATION_M,
                   help=f"이보다 가까운 두 지점을 문제로 본다 (기본 {MIN_SEPARATION_M}m)")
    p.add_argument("--rename-corrupt", action="store_true",
                   help="오염된 이름을 복원한 이름으로 바꿔서 이관한다 (기본은 원본 키 유지)")
    p.add_argument("--check-only", action="store_true",
                   help="보고만 하고 이관 파일을 남기지 않는다. 문제가 있으면 종료코드 1")
    args = p.parse_args()

    reports, n_issue, n_review = [], 0, 0
    for path in args.paths:
        rep, ni, nr = process(path, args.out_dir, args.min_separation, args.rename_corrupt)
        reports.append(rep)
        n_issue += ni
        n_review += nr

    text = "\n".join(reports)
    text += f"\n합계: 문제 {n_issue}건, 사람 판단 필요 {n_review}건\n"
    print(text)

    if args.check_only:
        for path in args.paths:
            stem = os.path.splitext(os.path.basename(path))[0]
            f = os.path.join(args.out_dir, f"{stem}.poi.yaml")
            if os.path.exists(f):
                os.remove(f)
    else:
        os.makedirs(args.out_dir, exist_ok=True)
        rp = os.path.join(args.out_dir, "report.txt")
        with open(rp, "w") as f:
            f.write(text)
        print(f"보고서: {rp}")

    return 1 if n_issue else 0


if __name__ == "__main__":
    sys.exit(main())

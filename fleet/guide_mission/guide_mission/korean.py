#!/usr/bin/env python3
"""조사 붙이기. 화면과 음성에 나가는 문장이라 "11층 로 갑니다" 로 두면 안 된다.

층 이름도 waypoint 이름도 사람이 정하는 문자열이라 미리 못 정한다. 받침이
있으면 '으로', 없거나 ㄹ 받침이면 '로' 다. 한글이 아니면(숫자·영문) 읽는
소리의 끝소리로 정한다 — "11a" 는 "에이" 라 '로', "1004호" 는 '호' 라 '로'.
"""
from __future__ import annotations

# 숫자를 읽었을 때 받침이 남는가. 0 영, 1 일, 3 삼, 6 육, 7 칠, 8 팔 → 받침 있음
_DIGIT_FINAL = {"0": True, "1": True, "2": False, "3": True, "4": False,
                "5": False, "6": True, "7": True, "8": True, "9": False}


def _final(ch: str) -> tuple[bool, bool]:
    """(받침이 있는가, 그 받침이 ㄹ 인가)."""
    o = ord(ch)
    if 0xAC00 <= o <= 0xD7A3:            # 한글 음절
        jong = (o - 0xAC00) % 28
        return jong != 0, jong == 8      # 8 = ㄹ
    if ch.isdigit():
        return _DIGIT_FINAL[ch], ch == "1"   # 일 → ㄹ 받침
    return False, False                  # 영문·기호는 받침 없음으로 본다


def ro(word: str) -> str:
    """'로' / '으로' 를 붙인다. 빈 문자열이면 그대로."""
    if not word:
        return word
    has, is_r = _final(word.strip()[-1])
    return word + ("로" if (not has or is_r) else "으로")


def eul(word: str) -> str:
    """'을' / '를'."""
    if not word:
        return word
    has, _ = _final(word.strip()[-1])
    return word + ("을" if has else "를")


def i_ga(word: str) -> str:
    """'이' / '가'."""
    if not word:
        return word
    has, _ = _final(word.strip()[-1])
    return word + ("이" if has else "가")

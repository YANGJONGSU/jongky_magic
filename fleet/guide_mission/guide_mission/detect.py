#!/usr/bin/env python3
"""층 판정 — 지금 몇 층인가.

엘리베이터 문이 열렸을 때 로봇은 자기가 몇 층인지 모른다. 엔코더는 내내
"정지" 라고 했고 라이다는 금속 벽만 봤다. 그래서 밖에서 알아내야 한다.

[후보 셋]
  1. **SSID** — 건물이 층마다 다른 AP 를 쓴다 (`FASTCAMPUS_10F`/`11F`).
     공짜고 즉시 나온다. 그러나 **젯슨이 관제 노트북 핫스팟에 붙어 있으면
     건물 SSID 가 아예 안 보인다.** VLM 이 노트북에 남는 것이 확정돼서
     (brain.py 상단) 핫스팟은 선택이 아니라 11층 운용의 전제 조건이다.
     즉 **가장 필요한 순간에 이 방법이 안 되는 구성이 정상 구성이다.**
  2. **사람이 UI 에서 고른다** — 언제나 된다. 대신 사람이 틀릴 수 있고,
     틀리면 다른 층 지도로 주행한다.
  3. 기압계·층 표지판 OCR — 하드웨어/모델이 없다.

[골라 쓴 것: 1 + 2]
SSID 를 **제안**으로 쓰고 **확정은 사람**이 하되, SSID 가 확신 있게 맞으면
자동으로 넘어간다(policy="auto"). 핫스팟에 붙어 있거나 SSID 를 모르면
자동 판정이 **모른다고 말하고 멈춘다** — 조용히 한 층을 찍지 않는다.
policy="always" 로 두면 매번 사람에게 묻는다.

무선 조회는 두 가지를 차례로 시도한다. 컨테이너 안에는 둘 다 없을 수 있고,
그때도 예외가 아니라 "모름" 으로 떨어져야 한다.

    iwgetid -r
    nmcli -t -f active,ssid dev wifi     # "yes:FASTCAMPUS_10F"
"""
from __future__ import annotations

import subprocess
from dataclasses import dataclass

# 왜 모르는가. UI 가 사람에게 보여 줄 문구를 고르는 데 쓴다.
NO_TOOL = "no_tool"          # iwgetid·nmcli 둘 다 없다 (컨테이너)
NO_WIFI = "no_wifi"          # 무선에 안 붙어 있다
HOTSPOT = "hotspot"          # 노트북 핫스팟에 붙어 있어 건물 SSID 가 안 보인다
UNKNOWN_SSID = "unknown_ssid"  # 붙어 있는데 어느 층인지 모르는 이름이다
DISABLED = "disabled"        # 자동 판정을 꺼 뒀다
MATCHED = "matched"


@dataclass(frozen=True)
class FloorGuess:
    floor: str | None       # 층 키. None 이면 모름
    ssid: str | None
    confident: bool
    why: str                # 위 상수 중 하나
    reason: str             # 사람이 읽을 문장

    @property
    def known(self) -> bool:
        return self.floor is not None


def _run(argv: list[str], timeout: float = 2.0) -> tuple[int, str]:
    """명령 하나. 없거나 죽으면 (코드, "") 로 떨어진다 — 예외를 안 던진다."""
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or "").strip()
    except FileNotFoundError:
        return 127, ""
    except subprocess.TimeoutExpired:
        return 124, ""
    except Exception:
        return 1, ""


class FloorDetector:
    """SSID 로 층을 짚는다. 명령 실행기는 주입할 수 있다 (시험용)."""

    def __init__(self, book, policy: str = "auto", runner=_run):
        self._book = book
        self._runner = runner
        self.policy = policy      # auto | always | off

    # ── SSID ──────────────────────────────────────────────────────────────
    def ssid(self) -> tuple[str | None, str]:
        """(SSID, 못 얻었으면 이유코드). 둘 다 없으면 (None, NO_TOOL)."""
        code, out = self._runner(["iwgetid", "-r"])
        if code == 0 and out:
            return out.splitlines()[0].strip(), MATCHED
        tool_seen = code != 127

        code, out = self._runner(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"])
        if code == 0:
            tool_seen = True
            for line in out.splitlines():
                # "yes:FASTCAMPUS_10F" — SSID 에 콜론이 있으면 nmcli 가 \: 로 준다
                if line.startswith("yes:"):
                    name = line[4:].replace("\\:", ":").strip()
                    if name:
                        return name, MATCHED
        elif code != 127:
            tool_seen = True

        return None, (NO_WIFI if tool_seen else NO_TOOL)

    # ── 판정 ──────────────────────────────────────────────────────────────
    def guess(self) -> FloorGuess:
        if self.policy == "off":
            return FloorGuess(None, None, False, DISABLED,
                              "자동 층 판정이 꺼져 있습니다. 층을 골라 주세요")

        name, why = self.ssid()
        if name is None:
            if why == NO_TOOL:
                return FloorGuess(
                    None, None, False, NO_TOOL,
                    "무선 상태를 볼 수 없습니다(iwgetid·nmcli 둘 다 없음). 층을 골라 주세요")
            return FloorGuess(
                None, None, False, NO_WIFI,
                "무선에 붙어 있지 않아 층을 알 수 없습니다. 층을 골라 주세요")

        low = name.strip().lower()
        if any(low == h.strip().lower() for h in getattr(self._book, "hotspot_ssids", ())):
            # 이게 정상 구성이다. 사람에게 고장이라고 말하면 안 된다.
            return FloorGuess(
                None, name, False, HOTSPOT,
                f"관제 노트북 핫스팟('{name}')에 붙어 있어 건물 SSID 가 보이지 않습니다. "
                f"층을 직접 골라 주세요")

        fl = self._book.by_ssid(name)
        if fl is None:
            return FloorGuess(
                None, name, False, UNKNOWN_SSID,
                f"'{name}' 은 어느 층인지 모르는 무선입니다. 층을 골라 주세요")

        confident = self.policy == "auto"
        return FloorGuess(
            fl.key, name, confident, MATCHED,
            f"무선 '{name}' 으로 {fl.label} 으로 판정했습니다")

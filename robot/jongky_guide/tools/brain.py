#!/usr/bin/env python3
"""온보드 LLM/VLM. 젯슨에서 도는 ollama 에 물어본다.

두 가지 일을 한다.

  1. 목적지 해석 — "삼백사호 가고 싶어요" 처럼 문자열 매칭이 실패한 발화에서
     등록된 waypoint 중 하나를 고른다
  2. 돌발상황 판단 — 앞이 막혀 멈췄을 때 전면 카메라 한 장을 보고
     사람인지 물건인지, 어떻게 대응할지 정한다

[왜 온보드인가]
건물 10·11층이 격리된 서브넷이라 관제 노트북에 두면 층을 넘는 순간 끊긴다.
젯슨에서 돌면 네트워크와 무관해진다. ollama 는 호스트에 있고 컨테이너는
--network host 로 뜨므로 localhost:11434 로 그냥 닿는다.

[안전 규약]
**LLM 은 로봇을 직접 조종하지 않는다.** 정해진 행동 집합 중 하나를 고를 뿐이고,
실제 주행은 그대로 Nav2 가 한다. 모델이 이상한 소리를 해도 로봇이 벽으로
돌진하지 않는다. 파싱이 실패하면 가장 보수적인 행동(정지 후 대기)으로 떨어진다.

[지연]
E2B 라도 젯슨에서 응답에 몇 초 걸린다. 그래서 제어 루프에 안 넣는다 —
로봇은 이미 Nav2 가 세워 둔 상태이고, 그 다음에 물어본다.
"""
from __future__ import annotations

import base64
import json
import logging
import urllib.error
import urllib.request

log = logging.getLogger("brain")

OLLAMA_URL = "http://localhost:11434/api/generate"

# 돌발상황에서 고를 수 있는 행동. 이 밖의 것은 받지 않는다.
ACTIONS = {
    "wait": "사람이 비켜 주기를 기다린다",
    "ask_to_move": "지나가겠다고 말하고 기다린다",
    "reroute": "다른 경로로 돌아간다",
    "alert": "위급해 보이므로 멈추고 알린다",
    "resume": "길이 비었으니 계속 간다",
}


class Brain:
    def __init__(self, model: str = "gemma4:e2b", timeout: float = 30.0, url: str = OLLAMA_URL):
        self._model = model
        self._timeout = timeout
        self._url = url

    # ── 공통 ──────────────────────────────────────────────────────────────
    def _ask(self, prompt: str, images: list[str] | None = None) -> str | None:
        payload = {
            "model": self._model,
            "prompt": prompt,
            "stream": False,
            # Gemma 4 는 추론 모델이다. think 를 켠 채로 두면 내부 추론이
            # num_predict 를 다 써버려서 **response 가 빈 문자열로 돌아온다**
            # (done_reason 은 "length"). 목적지 하나 고르는 데 추론이 필요 없다.
            "think": False,
            # 짧고 일관되게. 창의성이 필요한 일이 아니다.
            "options": {"temperature": 0.1, "num_predict": 200},
        }
        if images:
            payload["images"] = images

        req = urllib.request.Request(
            self._url,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as res:
                return json.load(res).get("response", "").strip()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            log.warning(f"ollama 응답 없음 (건너뛴다): {exc}")
            return None

    @property
    def available(self) -> bool:
        return self._ask("ping") is not None

    # ── 1. 목적지 해석 ────────────────────────────────────────────────────
    def resolve_destination(self, utterance: str, destinations: list[str]) -> str | None:
        """발화에서 목적지를 고른다. 못 고르면 None.

        문자열 매칭이 실패한 뒤에만 부른다. 빠르고 확실한 경로를 먼저 쓰고
        애매할 때만 모델을 부르는 편이 지연도 오판도 적다.
        """
        if not destinations:
            return None

        listing = "\n".join(f"- {d}" for d in destinations)
        prompt = (
            "너는 건물 안내로봇이다. 사용자의 말을 듣고 아래 목록에서 갈 곳을 하나 고른다.\n\n"
            f"[갈 수 있는 곳]\n{listing}\n\n"
            f'[사용자] "{utterance}"\n\n'
            "목록에 있는 이름을 그대로 한 줄만 출력해라. "
            "해당하는 곳이 없으면 NONE 이라고만 써라. 설명하지 마라."
        )
        answer = self._ask(prompt)
        if not answer:
            return None

        # 모델이 목록에 없는 걸 지어내는 경우를 막는다
        first = answer.splitlines()[0].strip().strip("\"'`")
        for d in destinations:
            if d == first or d.replace(" ", "") == first.replace(" ", ""):
                return d
        log.info(f"목적지 해석 실패: 모델 응답 '{first}'")
        return None

    # 오디오 직접 입력은 지원하지 않는다.
    #
    # Gemma 4 E2B 자체는 오디오를 받지만 **ollama 가 아직 안 넘겨준다**
    # (ollama#11798 기능 요청 상태). 실측으로 확인했다 — 같은 프롬프트를
    # ①오디오 없이 ②audio 필드로 ③엉터리 필드명으로 보냈더니 세 응답이 전부
    # 같았다. 필드가 통째로 무시되고 모델은 텍스트만 보고 목록 첫 항목을 찍는다.
    #
    # 그래서 이 경로는 "근거 없이 그럴듯한 목적지" 를 만들어 낸다 — 실패하느니만
    # 못하다. ollama 가 오디오를 지원하면 그때 다시 넣을 것. 그때까지 음성 인식은
    # 온보드 Whisper 가 전담한다.

    # ── 2. 돌발상황 판단 ──────────────────────────────────────────────────
    def judge_obstacle(self, jpeg: bytes, stuck_seconds: float) -> tuple[str, str]:
        """앞이 막힌 장면을 보고 (행동, 할 말) 을 정한다.

        어떤 실패에서도 행동은 ACTIONS 안의 것이고, 못 정하면 wait 이다.
        """
        options = "\n".join(f"- {k}: {v}" for k, v in ACTIONS.items())
        prompt = (
            "너는 건물 복도를 다니는 안내로봇의 눈이다. 앞이 막혀 "
            f"{stuck_seconds:.0f}초째 멈춰 있다. 사진을 보고 무엇 때문인지 판단해라.\n\n"
            f"[고를 수 있는 행동]\n{options}\n\n"
            "JSON 한 줄로만 답해라. 형식:\n"
            '{"action": "위 중 하나", "say": "사람에게 할 말(없으면 빈 문자열)", "reason": "판단 근거 한 문장"}\n\n'
            # 환각 억제. 이걸 안 넣으면 아무것도 없는 회색 화면을 보고도
            # "사람이 쓰러져 있다" 며 alert 을 고른다 (실측). 있는 것만 근거로
            # 삼게 하고, 애매하면 무해한 쪽으로 떨어뜨린다.
            "규칙:\n"
            "- 사진에 **실제로 보이는 것만** 근거로 삼아라. 추측하지 마라.\n"
            "- 사람이 보이지 않으면 사람에 대한 판단을 하지 마라.\n"
            "- 무엇이 막고 있는지 분명하지 않으면 resume 을 골라라.\n"
            "- alert 은 사람이 바닥에 쓰러져 있는 것이 **분명히 보일 때만** 고른다."
        )
        answer = self._ask(prompt, images=[base64.b64encode(jpeg).decode()])
        if not answer:
            return "wait", ""

        # 모델이 코드펜스나 잡설을 붙이는 경우가 흔하다. JSON 만 건져낸다.
        try:
            start, end = answer.index("{"), answer.rindex("}") + 1
            data = json.loads(answer[start:end])
            action = str(data.get("action", "")).strip()
            if action not in ACTIONS:
                log.info(f"모르는 행동 '{action}' — wait 으로 떨어진다")
                return "wait", ""
            log.info(f"판단: {action} ({data.get('reason', '')})")
            return action, str(data.get("say", "")).strip()
        except (ValueError, json.JSONDecodeError):
            log.info(f"판단 파싱 실패 — wait 으로 떨어진다: {answer[:80]}")
            return "wait", ""

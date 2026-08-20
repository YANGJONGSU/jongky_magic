#!/usr/bin/env python3
"""온보드 LLM/VLM. 젯슨에서 도는 ollama 에 물어본다.

두 가지 일을 한다.

  1. 목적지 해석 — "삼백사호 가고 싶어요" 처럼 문자열 매칭이 실패한 발화에서
     등록된 waypoint 중 하나를 고른다
  2. 돌발상황 판단 — 앞이 막혀 멈췄을 때 전면 카메라 한 장을 보고
     사람인지 물건인지, 어떻게 대응할지 정한다

[왜 관제 노트북인가]  (2026-08-19 실측으로 뒤집힘)
원래는 층 격리(10·11층이 별도 서브넷) 때문에 온보드로 두려 했다.
그러나 실제로 쓰는 모델 `gemma4:e2b` 는 7.2GB 고, **젯슨에 로드하는 시도만으로
기기가 마비돼 전원 재시작이 필요했다.** 이미 Q4_K_M 이라 더 줄일 여지도 없다.

→ 관제 노트북(RTX 5080 16GB)에 두고 URL 로 가리킨다. 판단이 이벤트 기반이라
   네트워크가 끊겨도 주행은 nav2 로 그대로 돈다. 11층에서 쓰려면 노트북
   AP 핫스팟이 필요하다 — 선택지가 아니라 전제 조건이다.

굳이 온보드로 돌리려면 URL 을 명시적으로 localhost 로 주어야 한다.
기본값으로는 젯슨을 치지 않는다.

[안전 규약]
**LLM 은 로봇을 직접 조종하지 않는다.** 정해진 행동 집합 중 하나를 고를 뿐이고,
실제 주행은 그대로 Nav2 가 한다. 모델이 이상한 소리를 해도 로봇이 벽으로
돌진하지 않는다. 파싱이 실패하면 가장 보수적인 행동(정지 후 대기)으로 떨어진다.

[지연]
E2B 라도 응답에 몇 초 걸린다. 그래서 제어 루프에 안 넣는다 —
로봇은 이미 Nav2 가 세워 둔 상태이고, 그 다음에 물어본다.

[호출은 동기다 — 타임아웃이 곧 정지 시간이다]
guide_node 의 `_guide_loop` 워커가 `_handle_obstacle()` 을 **동기로** 부른다.
그래서 여기서 기다리는 시간이 그대로 주행 판단 루프가 멎는 시간이다.
관제 노트북에 못 닿을 때(층을 넘었거나 노트북이 꺼졌거나) 판단마다 통째로
멈추는데, `_guide_loop` 의 재판단 간격(COOLDOWN 15초)이 그보다 짧으면
정체가 이어지는 동안 사실상 상시 블로킹이 된다. 예전 타임아웃 30초가
정확히 그 상태였다.

그래서 두 가지를 둔다.

  1. 타임아웃을 짧게 (기본 4초). 응답이 늦는 것보다 늦는 걸 아는 게 중요하다
  2. 회로 차단기 — 연속으로 BREAKER_FAILS 회 실패하면 BREAKER_COOLDOWN_S
     동안 아예 시도하지 않고 즉시 None 을 돌린다. 못 닿는 게 확정된 망에서
     매번 4초씩 버리지 않는다. 차단이 풀리면 한 번 찔러 보고, 되면 복구한다

**LLM 이 없어도 안내는 계속된다.** _ask 가 None 이면 judge_obstacle 은 wait 을,
resolve_destination 은 None 을 돌리고, 주행은 그대로 Nav2 가 한다.
"""
from __future__ import annotations

import base64
import json
import logging
import os
import time
import urllib.error
import urllib.request

log = logging.getLogger("brain")

def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        log.warning(f"{name}='{raw}' 를 숫자로 못 읽는다 — 기본값 {default} 를 쓴다")
        return default


# 관제 노트북. 젯슨(localhost)이 기본이면 --llm-url 을 빠뜨렸을 때
# 7.2GB 모델을 온보드에 올리려다 기기가 멈춘다. 환경변수로 덮을 수 있다.
OLLAMA_URL = os.environ.get(
    "JONGKY_LLM_URL", "http://192.168.129.97:11434/api/generate"
)

# 한 번 물어보고 기다리는 최대 시간. **이 값이 그대로 주행 판단이 멎는
# 시간이다** (위 docstring 참조). 관제 노트북이 RTX 5080 이라 정상일 때는
# 1~2초면 온다. 현장에서 정말 모자라면 환경변수로 올릴 것 — 소스를 고치고
# 다시 배포하는 것보다 낫다. 다만 15초(guide_node 의 재판단 간격)에 가까워질수록
# 예전의 상시 블로킹으로 돌아간다는 걸 알고 올릴 것.
ASK_TIMEOUT_S = _env_float("JONGKY_LLM_TIMEOUT", 4.0)

# 회로 차단기. 연속 실패가 이만큼 쌓이면 한동안 아예 시도하지 않는다.
# 못 닿는 망에서 판단마다 ASK_TIMEOUT_S 씩 버리는 걸 막는다.
BREAKER_FAILS = 3
BREAKER_COOLDOWN_S = _env_float("JONGKY_LLM_BREAKER_S", 60.0)

# 돌발상황에서 고를 수 있는 행동. 이 밖의 것은 받지 않는다.
ACTIONS = {
    "wait": "사람이 비켜 주기를 기다린다",
    "ask_to_move": "지나가겠다고 말하고 기다린다",
    "reroute": "다른 경로로 돌아간다",
    "alert": "위급해 보이므로 멈추고 알린다",
    "resume": "길이 비었으니 계속 간다",
}


class Brain:
    def __init__(
        self,
        model: str = "gemma4:e2b",
        timeout: float = ASK_TIMEOUT_S,
        url: str = OLLAMA_URL,
        breaker_fails: int = BREAKER_FAILS,
        breaker_cooldown_s: float = BREAKER_COOLDOWN_S,
    ):
        self._model = model
        self._timeout = timeout
        self._url = url

        # 회로 차단기 상태.
        # _fails      연속 실패 횟수 (성공하면 0 으로 돌아간다)
        # _open_until 이 시각까지는 시도조차 안 한다 (monotonic)
        self._breaker_fails = max(1, breaker_fails)
        self._breaker_cooldown = breaker_cooldown_s
        self._fails = 0
        self._open_until = 0.0
        self._skipped = 0  # 차단 중 건너뛴 호출 수. 복구 로그에 같이 낸다

    @property
    def blocked(self) -> bool:
        """지금 회로가 열려 있나(=시도하지 않나). 진단용."""
        return time.monotonic() < self._open_until

    # ── 공통 ──────────────────────────────────────────────────────────────
    def _ask(self, prompt: str, images: list[str] | None = None) -> str | None:
        # 차단 중이면 즉시 포기한다. 여기서 로그를 남기면 안 된다 —
        # 정체가 이어지는 동안 guide_node 가 15초마다 부르므로 그대로 도배된다.
        now = time.monotonic()
        if now < self._open_until:
            self._skipped += 1
            return None

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
                answer = json.load(res).get("response", "").strip()
        # ValueError 는 ollama 가 JSON 이 아닌 것을 돌려준 경우다(프록시 오류
        # 페이지 등). 예전에는 이게 그대로 올라가 _guide_loop 워커를 죽였다 —
        # 판단 하나 때문에 안내가 통째로 멎으면 안 된다. 실패로 세고 넘어간다.
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            self._on_failure(exc)
            return None
        self._on_success()
        return answer

    def _on_success(self) -> None:
        if self._fails:
            log.info(
                f"ollama 복구됨 ({self._url}) — 연속 실패 {self._fails}회, "
                f"차단 중 건너뛴 판단 {self._skipped}회"
            )
        self._fails = 0
        self._skipped = 0
        self._open_until = 0.0

    def _on_failure(self, exc: Exception) -> None:
        """실패를 세고, 임계를 넘으면 회로를 연다.

        로그는 두 번만 낸다 — 첫 실패와 차단에 들어갈 때. 그 사이와 차단
        중에는 조용하다. 판단이 15초마다 반복되므로 매번 찍으면 정작 중요한
        주행 로그가 묻힌다. 반대로 아예 안 찍으면 "LLM 이 죽은 걸 아무도
        모르는" 상태가 되므로, 차단이 갱신될 때마다(쿨다운마다 한 번) 남긴다.
        """
        self._fails += 1
        if self._fails == 1:
            log.warning(f"ollama 응답 없음 (판단을 건너뛴다): {exc}")
        elif self._fails >= self._breaker_fails:
            self._open_until = time.monotonic() + self._breaker_cooldown
            log.warning(
                f"ollama({self._url}) 연속 {self._fails}회 실패 — "
                f"{self._breaker_cooldown:.0f}초 동안 묻지 않는다. "
                f"주행은 그대로 간다 (판단만 쉰다)"
            )

    @property
    def available(self) -> bool:
        """차단 중이면 찔러 보지 않고 바로 False 다."""
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

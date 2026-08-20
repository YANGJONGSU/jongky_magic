# ADR 002: Zenoh 브릿지 채택

- **Status**: **미결 (Proposed). 구현 0건.**
- **Date**: 문서화 2026-08-20

## Context

동기는 저장소에 한 줄로 남아 있다.

> Wi-Fi 너머로 이미지를 흘리는 구간은 DDS 만으로는 지연·지터가 크다.
> `comms/` 의 zenoh 브리지(ADR 002)가 그 대응이다.
> — `docker/README.md:82-84`

배경 조건도 실재한다. 컨테이너를 `--network host` 로 띄우는 이유가 DDS
멀티캐스트 디스커버리이고(`docker/README.md:74-76`), 학습 노트북과 통신하려면
양쪽 `ROS_DOMAIN_ID` 를 맞춰야 한다(`:78-79`). 건물 WiFi 는 **층마다 서브넷이
갈려서** 11층에 올라가면 SSH 조차 끊긴다(`jongky_bringup/README.md 의 "현장 맵핑 절차" 절`).

## Decision

**아직 없다.** 2026-08-20 확인 기준:

| | |
|---|---|
| `zenoh-bridge-ros2dds` 설정 | **0건** |
| 벤치마크 스크립트 | **0건** (`docs/benchmarks/README.md` 도 "(TODO)") |
| `comms/zenoh_bridge/` 의 내용물 | README 3줄이 전부 |
| DDS 지연·지터 실측값 | **없다. 확인 안 됨** — "크다" 는 서술만 있고 숫자가 없다 |

### 그리고 지금 로봇 밖으로 나가는 것은 DDS 가 아니다

실제로 기계 경계를 넘는 두 갈래는 **평문 HTTP** 다.

| | 어디로 | 무엇 |
|---|---|---|
| `brain.py 의 OLLAMA_URL 기본값` | 관제 노트북 `192.168.129.97:11434` | LLM/VLM (ollama `POST /api/generate`) |
| `follow_client.py 의 FOLLOW_URL 기본값` | 젯슨 **호스트** `localhost:8641` | 후면 사람 탐지 |

둘 다 ROS 를 안 쓴다. HTTP 로 간 이유는 성능이 아니라 각각 **모델 크기**
(7.2GB 를 젯슨에 올리면 기기가 마비된다, `brain.py 머리말 [왜 관제 노트북인가]`)와 **컨테이너 cv2 의
numpy ABI 충돌**(`follow_service.py 머리말 [왜 컨테이너가 아니라 호스트인가]`)이었다.

즉 **DDS 지연이 문제가 되는 지점에 도달하기 전에**, 이미 다른 이유로 HTTP 를
쓰고 있다. Zenoh 를 넣으려면 "무엇을 DDS 로 흘릴 것인가" 부터 정해야 한다.

## Consequences

- `comms/` 가 빈 채로 남는다. `docs/architecture.md` 의 L5 위 관제 계층이 없다
- 벤치마크가 없으므로 **"Zenoh 가 나았다" 를 나중에 주장할 수 없다.**
  브리지를 넣기 전에 DDS 기준선을 먼저 재야 한다
- **2026-08-24 시연 판정 대상이 아니다** — `docs/mission.md` 5절

## 다음에 이 문서를 채울 사람에게

1. **DDS 기준선을 먼저 잰다** — 젯슨↔노트북 사이 `/scan`·이미지 토픽의
   지연·지터·손실. 이 숫자 없이 Zenoh 를 넣으면 판정이 불가능하다
2. **층 격리를 Zenoh 가 푸는가** — 서브넷이 갈린 것은 라우팅 문제다.
   지금 계획된 대응은 관제 노트북 AP 핫스팟이고, 그건 **11층 운용의 전제
   조건**으로 이미 잡혀 있다(`brain.py 머리말 [왜 관제 노트북인가]`, `잔여-공정.html` N3, 작업 노트)
3. **`ROS_DOMAIN_ID` 로 충분한 구간과 아닌 구간을 나눈다**

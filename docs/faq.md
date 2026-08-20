# FAQ

> 통합 당번이 다른 두 사람에게 "질의부터 모터까지 어디를 거치나 / 왜 그렇게 정했나 / 뭐가 실패했나"를 묻고,
> 그 자리에서 막힌 답을 여기 기록합니다. [collaboration.md](collaboration.md)의 통합 당번 로테이션 참고.

---

## 상호 질의 기록

**아직 없다.** 통합 당번 로테이션이 아직 한 바퀴도 안 돌았다.
지어낸 문답으로 채우면 이 문서의 쓸모가 없어지므로 비워 둔다.

세 질문의 답은 지금 이 세 문서에 있다. 질의 전에 이쪽을 먼저 읽을 것.

| 질문 | 지금 답이 있는 곳 |
|---|---|
| 질의부터 모터까지 어디를 거치나 | [trace.md](trace.md) — 8홉 전부 파일:줄로 |
| 왜 그렇게 정했나 | [adr/](adr/) — 채택 3장, **미결 2장** |
| 뭐가 실패했나 | [troubleshooting.md](troubleshooting.md) — 문제당 네 줄 |

---

## 답이 아직 없는 질문 (2026-08-20 코드 정리 중에 나온 것)

**누가 물어본 것이 아니라, 코드를 읽다가 저장소에 답이 없다는 것이 확인된
질문들이다.** 다음 질의 세션에서 먼저 소진할 것.

| # | 질문 | 지금 상태 |
|---|---|---|
| Q1 | 층 전환은 L4(맵 교체)인가 L5(임무 상태 전이)인가 | **2026-08-20 저녁에 L5 로 답이 나왔다** — `fleet/guide_mission/guide_mission/transfer.py` 가 ROS 없이 전이만 돌리고 `guide_node.py::NavEffects` 가 행동을 옮긴다. 미커밋·실차 0회 |
| Q2 | VDA5050 을 왜 골랐나. Open-RMF 와 역할 분담은 | **근거 없음** — [adr/001](adr/001-vda5050.md) |
| Q3 | DDS 지연이 실제로 얼마인가. Zenoh 를 넣을 근거가 되나 | **측정 없음** — [adr/002](adr/002-zenoh.md). `docs/benchmarks/` 도 비어 있다 |
| Q4 | 라이다 평면(0.219 m) 아래 장애물을 무엇이 막나 | **아무것도 안 막는다.** 코스트맵 관측원이 `/scan` 하나뿐이고(`nav2_params.yaml:210-220`), VLM 은 이미 멈춘 뒤에 부르는 이벤트형이다. 시연에서는 통제 조건으로 다룬다 — `mission.md` 3절 |
| Q5 | `/guide/destination` 을 아무나 발행하면 로봇이 출발하는데 괜찮은가 | **검증 계층이 없다.** HTTP 도 `0.0.0.0` 무인증 (`guide_node.py::main()` 의 `ThreadingHTTPServer` 가 `0.0.0.0` 바인드) |
| Q6 | waypoint 이름 하나가 표시명·음성매칭·목적지ID·초기위치후보를 겸하는데 층 정보는 어디에 넣나 | **오늘 `Poi.msg` 가 답을 냈다** — `id`/`display_name`/`aliases`/`kind`/`floor` 분리. **아직 런타임에 안 붙었다** — `interfaces.md` 7절 |
| Q7 | 배터리가 얼마 남았는지 시연 중에 어떻게 아나 | **모른다.** 보드는 전압을 보고하는데 경고 로그만 남는다 (`jongky_system.cpp:220-226`) |
| Q8 | 아스트라 최소거리는 얼마인가 | **안 쟀다.** 도구는 있다 (`jongky_description/scripts/check_depth_min_range.py`). 시뮬 `clipping_range` 하한 0.1 m 는 가정값이다 |
| Q9 | 후면 카메라 거리 추정의 HFOV 62.2도는 실측인가 | **아니다. 렌즈 공칭값이다** (`follow_service.py 의 HFOV_DEG 상수`). 응답의 `calib.measured` 가 `false` 로 그 사실을 노출한다 |

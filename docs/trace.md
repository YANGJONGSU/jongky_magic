# Trace — 임무 한 건의 전 경로

> 통합 당번이 매 사이클 갱신. G0 이후 stub 기준으로 먼저 채우고, G5(통합) 이후 실제 경로로 교체.
> 이 문서를 안 보고 화이트보드에 그릴 수 있으면 면접 준비는 끝입니다.

각 홉마다: 노드/토픽/파일 + "여기서 실패하면 어떻게 되는가" 한 줄.

## 예시 형식

| # | 홉 | 노드 / 토픽 / 파일 | 실패 시 |
|---|---|---|---|
| 1 | 자연어 질의 입력 | TODO | TODO |
| 2 | POI 검증 | TODO (`fleet/guide_mission`) | TODO |
| 3 | 임무 상태머신 진입 | TODO | TODO |
| 4 | VDA5050/Nav2 목표 전달 | TODO | TODO |
| 5 | 층 전환 (필요 시) | TODO | TODO |
| 6 | Nav2 경로 추종 | TODO | TODO |
| 7 | ros2_control → CAN FD → 모터 | TODO | TODO |
| 8 | RobotStatus / GuideTask result 반환 | TODO | TODO |

---

TODO: G0 stub 기준 trace 채우기.

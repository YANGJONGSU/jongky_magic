# Architecture

> TODO: 계층 + 계약(interface contract) 정의

- robot/ — 로봇 단일 개체 (description, bringup, hardware, control, navigation)
- fleet/ — 다중 로봇 운영 (vda5050, rmf, mission)
- comms/ — 로봇 ↔ 상위 시스템 통신 브릿지 (zenoh)
- sim/ — 시뮬레이션 (gz sim, 월드)
- interfaces/ — 계층 간 계약 (msg/action/POI 스키마)

계층별 상세 계약은 [interfaces.md](interfaces.md), 좌표계/단위 규약은 [conventions.md](conventions.md) 참고.

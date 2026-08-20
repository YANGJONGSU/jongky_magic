# ADR — Architecture Decision Records

왜 그 선택을 했는지. **결정이 안 내려진 것은 "미결" 로 적는다** — 빈 칸을
그럴듯한 산문으로 덮으면 나중에 그게 결정이었던 줄 알게 된다.

| | 결정 | Status |
|---|---|---|
| [000](000-ros-distro.md) | ROS 2 **Jazzy**. 젯슨에서는 24.04 컨테이너로 | 채택 · 실물 검증 |
| [001](001-vda5050.md) | VDA5050 | **미결 · 구현 0건** |
| [002](002-zenoh.md) | Zenoh 브릿지 | **미결 · 구현 0건** |
| [003](003-dreamerv3-torch.md) | DreamerV3 를 RLlib 이 아니라 **dreamerv3-torch** 로 | 채택 · **학습 미실행** |
| [004](004-astra-driver.md) | 아스트라를 `orbbec_camera` 가 아니라 **`openni2_camera`** 로 | 채택 · 실물 검증 |
| [005](005-diff-drive-controller.md) | 야붐 보드에 **`diff_drive_controller`**(A안) | 채택 · 실물 검증 |

## 아직 ADR 이 없는 결정

코드에는 있는데 근거가 문서로 안 남은 것들. 채울 사람이 가져갈 것.

| 결정 | 코드 근거 | 왜 ADR 이 필요한가 |
|---|---|---|
| `odom->base_footprint` 를 **EKF** 가 낸다 | `jongky_controllers.yaml:85-93`, `robot.launch.py:64-77` | 지도를 두 겹으로 그리게 만든 사고가 정확히 이 구조였다 |
| Nav2 플래너 **SmacPlanner2D** · 컨트롤러 **RPP** | `nav2_params.yaml:133`, `:103` | 대안(NavFn/DWB/MPPI)과 비교한 기록이 없다 |
| **rplidar_ros 소스 빌드** (apt 판 제외) | `Dockerfile.robot:22`, `troubleshooting.md:18-42` | 이미 troubleshooting 에 다 적혀 있어 ADR 로 옮기기만 하면 된다 |
| LLM 을 **온보드가 아니라 관제 노트북**에 | `brain.py 머리말 [왜 관제 노트북인가]` | 2026-08-19 실측으로 원래 결정이 뒤집혔다. 뒤집힌 결정이야말로 ADR 감이다 |
| **층 전환**의 소속 계층 | `fleet/guide_mission/guide_mission/transfer.py`, `guide_node.py::NavEffects` | 2026-08-20 저녁에 코드로 답이 나왔다(L5, Effects 주입). **미커밋·실차 0회.** 자리 잡히면 ADR 로 남길 것 |

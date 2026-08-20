# jongky_magic

jongky는 특정 장소의 가이드를 수행하는 VLM 기반 자율주행 로봇입니다.

이 README는 프로젝트의 진입점입니다. 무엇을 하는 프로젝트인지, 저장소가 어떻게 나뉘어 있는지,
어떤 문서를 언제 봐야 하는지, 팀이 어떻게 협업하는지를 여기서 먼저 파악하세요.

## 문서 가이드

| 문서 | 언제 보나 |
|---|---|
| [docs/mission.md](docs/mission.md) | 임무 정의와 성공 기준이 궁금할 때 |
| [docs/architecture.md](docs/architecture.md) | 계층 구조와 계약(interface contract)을 볼 때 |
| [docs/conventions.md](docs/conventions.md) | 좌표계·단위·메쉬 규약 — **메쉬/CAD 작업 전 필독** |
| [docs/interfaces.md](docs/interfaces.md) | msg/action/POI 스키마 목록 |
| [docs/adr/](docs/adr/) | 왜 그 기술/설계를 선택했는지 (ROS distro, VDA5050, Zenoh) |
| [docs/collaboration.md](docs/collaboration.md) | 팀 역할 분담, 진척 게이트(G0~G7), 협업 규약 |
| [docs/trace.md](docs/trace.md) | 임무 한 건이 질의→모터까지 어떤 경로를 타는지 |
| [docs/troubleshooting.md](docs/troubleshooting.md) | 뭐가 안 됐고 어떻게 알아냈는지 |
| [docs/faq.md](docs/faq.md) | 팀원 간 상호 질의에서 나온 것들 |
| [docs/benchmarks/](docs/benchmarks/) | 성능/지연 벤치마크 결과 |

## 저장소 구조

```
jongky_magic/
├── docs/            # 위 문서 가이드 전체
├── interfaces/       # msg/action + POI 스키마 (변경 시 3인 승인 필수)
├── robot/            # 단일 로봇: description, bringup, hardware, control, navigation
├── fleet/            # 다중 로봇 운영: vda5050, rmf, mission
├── comms/            # zenoh-bridge-ros2dds
├── sim/              # Gazebo 시뮬레이션 + 월드
├── tools/            # 개발/운영 보조 스크립트
└── docker/           # 빌드/실행 컨테이너
```

각 패키지 디렉토리 안의 README.md에 해당 패키지의 역할이 짧게 적혀 있습니다.

## 팀 & 역할

3인 풀스택 체제로 운영합니다. 소유는 최종 책임이지 독점이 아닙니다 — 상세는 [docs/collaboration.md](docs/collaboration.md) 참고.

| | A — 플랫폼/주행 | B — 임무/관제 | C — 인식/인터랙션 |
|---|---|---|---|
| 소유 | `robot/`, `sim/` | `fleet/`, `comms/`, `interfaces/` | `robot/jongky_guide/`, POI, 월드 자산 |
| 핵심 | URDF, ros2_control, UART(야붐 보드), Nav2, SLAM | 임무 상태머신, VDA5050, Open-RMF, Zenoh | VLM, POI 매핑, 뎁스 카메라, 센서 배치 |

## 진행 상태

로봇 스택은 실물에서 돈다. URDF·ros2_control·야붐 보드 하드웨어 인터페이스·Nav2·
slam_toolbox·EKF 가 모두 실차에서 검증됐고, 10·11층 지도와 waypoint 를 실제로 찍었다.
안내 기능(웹 UI·TTS·STT·Nav2 디스패치·후면 사람 추종·VLM 판단)도 `robot/jongky_guide/`
에 구현돼 있다 — **다만 아직 한 번도 빌드·실행된 적이 없다.**

시뮬 쪽은 Isaac Lab 복도 env 와 DreamerV3 래퍼가 있고, SLAM 지도에서 복도 기하를
뽑아 세우는 파이프라인까지 있다. **학습은 아직 안 돌렸다.**

여전히 비어 있는 것: `interfaces/guide_interfaces`, `fleet/*`, `comms/zenoh_bridge`
(전부 README 3줄), CI 워크플로, 그리고 **층 전환**(SSID 판정 + `map_server` 의
`load_map`) — 코드에 흔적이 없다.

게이트 전체 흐름은 [docs/collaboration.md](docs/collaboration.md#4-진척-게이트)에서 확인하세요.

## 시작하기

```bash
# 목업 하드웨어 (보드 없이)
ros2 launch jongky_bringup robot.launch.py use_mock:=true

# 실물
ros2 launch jongky_bringup robot.launch.py

# 맵핑 (현장 스크립트 — 카메라·SLAM·RViz 까지 한 번에)
jmap 10f

# 안내 (지도와 waypoint 를 인자로 준다)
ros2 launch jongky_guide guide.launch.py \
    map:=/path/fastcampus_10f.yaml waypoints:=~/waypoints_10f.yaml
```

> `jongky_guide` 는 `jmap` 의 빌드 목록에 없다. 처음 쓸 때는
> `colcon build --packages-select jongky_guide` 를 따로 돌려야 한다.

## 기여 규칙

- `interfaces/` 변경은 A/B/C 3인 승인 없이 머지 금지
- 버그는 담당자가 아닌 사람이 먼저 봄 (디버그 우선권 역전) — [docs/collaboration.md](docs/collaboration.md#3-전원-풀스택-장치-3개)
- 메쉬/CAD 관련 작업 전 [docs/conventions.md](docs/conventions.md) 체크리스트 확인

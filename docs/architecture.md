# Architecture

계층과 계약. **README 에 적힌 구조가 아니라 저장소에 실제로 있는 것**을 적는다.

> **확인 시점 2026-08-20.** `robot/`·`fleet/`·`interfaces/` 를 다른 트랙이 같은
> 날 고치고 있어서, 그쪽 인용은 파일:줄 대신 심볼 이름으로 적었다.
> "오늘 들어왔다" 로 표시한 것은 전부 **미커밋(untracked/modified) 상태**다.

계층별 상세 계약은 [interfaces.md](interfaces.md), 좌표계/단위 규약은
[conventions.md](conventions.md), 임무 한 건의 전 경로는 [trace.md](trace.md).

---

## 1. 지금 서 있는 것 / 비어 있는 것

| 디렉터리 | 역할 | 상태 |
|---|---|---|
| `robot/jongky_description` | URDF/xacro, 메쉬, 프레임 | **실물 검증됨** |
| `robot/jongky_hardware` | `ros2_control` SystemInterface (C++), 야붐 보드 UART | **실물 검증됨** |
| `robot/jongky_control` | 컨트롤러 설정 + EKF | **실물 검증됨** |
| `robot/jongky_bringup` | 구동계·라이다 런치, 텔레옵, 현장 스크립트 | **실물 검증됨** |
| `robot/jongky_navigation` | SLAM Toolbox + Nav2 + 지도 저장 | **실물 검증됨** |
| `robot/jongky_guide` | 웹 UI · TTS/STT · Nav2 디스패치 · 추종 · VLM | 구현됨. **실행 이력 0건** |
| `sim/jongky_rl` | Isaac Lab 복도 env + DreamerV3 어댑터 | 구현됨. **학습 미실행** |
| `sim/jongky_gz_sim` · `sim/jongky_gz_worlds` | Gazebo 월드 | **빈 껍데기** (README 3줄) |
| `fleet/guide_mission` | 임무 상태머신 (층 전환 포함) | **작업 중.** 2026-08-20 저녁에 코드가 들어왔다 — 미커밋·미검증 |
| `fleet/guide_vda5050` | VDA5050 어댑터 | **빈 껍데기** — `adr/001` |
| `fleet/guide_rmf` | Open-RMF fleet adapter | **빈 껍데기** |
| `comms/zenoh_bridge` | zenoh-bridge-ros2dds | **빈 껍데기** — `adr/002` |
| `interfaces/guide_interfaces` | msg/action/srv + POI 스키마 | **정의는 들어왔다(2026-08-20). 쓰는 코드는 0건** |
| `docker/` | 젯슨 온보드 런타임 이미지 | 동작 중 (`jongky:jazzy`) |
| `tools/` · `deploy/` | 캘리브레이션·라이다 진단 / RunPod Cosmos 배치 | 사용 중 |

`.msg` · `.action` · `.srv` 정의가 오늘 `interfaces/guide_interfaces` 에
들어왔지만 **임포트하는 코드는 아직 0건이다.** 지금 런타임에 계층을 가로지르는
것은 여전히 기성 ROS 타입 아니면 HTTP JSON 이다 — [interfaces.md](interfaces.md).

---

## 2. 실제 계층 (아래에서 위로)

```
 야붐 Rosmaster 보드 ── UART 115200 8N1 ── /dev/yahboom
        ▲
 [L1] jongky_hardware        C++ SystemInterface. 부호·단위·역기구학
        │                    yahboom_board.{hpp,cpp} 는 ROS 를 모른다
        ▼
 [L2] ros2_control           diff_drive_controller · joint_state_broadcaster
        │                    imu_sensor_broadcaster
        │  /cmd_vel(TwistStamped) ↓   /odom /joint_states /imu/data ↑
        ▼
 [L3] robot_localization EKF  odom -> base_footprint 의 유일한 주인
        │
        ▼
 [L4] Nav2 / SLAM Toolbox     AMCL · SmacPlanner2D · RPP · costmap · velocity_smoother
        │  navigate_to_pose 액션 ↑
        ▼
 [L5] jongky_guide            BasicNavigator 디스패치 + 상태 + 웹서버
        │  HTTP :8080 ↕  /guide/destination ↓  /guide/status ↑
        ▼
 [L6] 사람                    7인치 터치스크린 · 마이크 · 스피커

 ─── 로봇 밖 (HTTP 로만 붙는다) ───────────────────────────
 관제 노트북 ollama  192.168.129.97:11434   brain.py  (LLM/VLM)
 젯슨 호스트  localhost:8641/follower       follow_service.py (컨테이너 밖)
```

**L5 위에 관제 계층이 없다.** 원래 `fleet/` 과 `comms/` 가 그 자리인데
`guide_vda5050` · `guide_rmf` · `zenoh_bridge` 가 전부 비어 있어서, 지금은
`jongky_guide` 가 최상위다. `fleet/guide_mission` 만 오늘 채워지기 시작했는데,
그것도 관제가 아니라 **L5 옆에 붙는 층 전환 상태머신**이다 — `guide_node` 가
`--floors` 로 임포트해서 같은 프로세스에서 돌린다.

### 계층 경계에서 지켜지는 것

| 경계 | 계약 | 왜 여기서 자르나 |
|---|---|---|
| L1 ↔ L2 | 관절 단위 속도 명령(rad/s) · 관절 위치/속도 상태 | **보드 부호 반전을 L1 안에서만 처리한다.** URDF·컨트롤러에 넣으면 목업/시뮬/실물이 서로 다른 인터페이스를 갖는다 (`jongky_hardware/README.md:49-50`) |
| L2 ↔ L3 | `/odom` 은 컨트롤러가, `odom->base_footprint` TF 는 EKF 가 | 둘 다 TF 를 쏘면 **에러 없이** 두 추정이 번갈아 이긴다 (`jongky_controllers.yaml:85-93`) |
| L3 ↔ L4 | `map -> odom` 은 AMCL(또는 SLAM Toolbox) | AMCL 이 초기 위치를 못 받으면 이 변환이 없고, 그러면 `map` 프레임 목표가 통째로 죽는다 (`guide_node.py::set_start() 앞 주석`) |
| L4 ↔ L5 | `navigate_to_pose` 액션 (BasicNavigator 경유) | 여기가 **VDA5050 이 들어올 자리였다.** 지금은 직결 |
| L5 ↔ L6 | HTTP JSON + `std_msgs/String` 토픽 | `.msg` 정의 없이 문자열로 넘긴다. 설계 중인 `guide_interfaces` 가 이 자리 |

---

## 3. 시뮬 트랙은 별도 계층이 아니라 **별도 스택**이다

`sim/jongky_rl` 은 ROS 패키지가 아니다. Isaac Lab venv 에서 직접 돈다
(`sim/jongky_rl/README.md:7`). 로봇 스택과 공유하는 것은 **URDF 와 실차
상수뿐**이다.

```
robot/jongky_description/urdf/robot.urdf.xacro
        │  xacro is_sim:=true  →  URDF  →  convert_urdf.py --merge-joints
        ▼
   jongky.usd  →  Isaac Lab DirectRLEnv  →  dreamerv3-torch
```

공유되는 실차 상수: `v_max` 0.40 · `omega_max` 1.50 · `a_max` 0.30 ·
`wheel_radius` 0.0335 · `wheel_separation` 0.11909 · 카메라 HFOV 57.86도
(`sim/jongky_rl/README.md:67-80`). **이 값이 어긋나면 sim2real 이 통째로
날아간다** — MARS 정책이 종키에 못 올라간 이유가 정확히 `MAX_VX` 1.5 대
0.40 이다.

Gazebo 트랙(`sim/jongky_gz_*`)은 **비어 있다.** Isaac Lab 으로 갔다.

---

## 4. 배포 형태

| | |
|---|---|
| 젯슨 호스트 | Ubuntu 22.04 (JetPack 6 / L4T R36.4). 라이다 드라이버·`follow_service.py` 가 여기서 돈다 |
| 컨테이너 `jongky:jazzy` | Ubuntu 24.04 + ROS 2 Jazzy + CUDA 12.8. ROS 스택 전부 (`docker/README.md:11`) |
| 관제 노트북 | Ubuntu 24.04 + Jazzy + RTX 5080. ollama · Isaac Lab · DreamerV3 |

호스트/컨테이너 분할은 ROS distro 결정에서 온다 — [adr/000](adr/000-ros-distro.md).
`follow_service.py` 가 컨테이너 밖인 것은 성능이 아니라 **컨테이너 cv2 의
numpy 2.0 ABI 충돌** 때문이다(`follow_service.py 머리말 [왜 컨테이너가 아니라 호스트인가]`). 16GB 이미지를
흔드는 것보다 HTTP 로 넘기는 편이 쌌다.

건물 WiFi 가 층마다 서브넷이 갈린다(`jongky_bringup/README.md 의 "현장 맵핑 절차" 절`).
그래서 11층에서 LLM 을 쓰려면 관제 노트북 AP 핫스팟이 **선택지가 아니라
전제 조건**이다. 끊겨도 주행은 Nav2 로 그대로 돌고 VLM 판단만 `wait` 으로
떨어진다(`brain.py 머리말 [왜 관제 노트북인가]`, `:89-91`).

---

## 5. 이 구조에서 아직 안 정해진 것

- **관제 계층 전체.** `fleet/guide_vda5050`·`guide_rmf`·`comms/` 가 비어
  있어서 로봇 여러 대를 어떻게 묶는지에 대한 결정이 저장소에 없다
- **층 전환의 소속 계층 — 2026-08-20 저녁에 코드로 답이 나왔다. L5 다.**
  `transfer.py` 가 ROS 를 임포트하지 않고 전이만 돌리고, `guide_node` 의
  `NavEffects` 가 그걸 Nav2·`load_map`·TTS 로 옮긴다. 맵 교체(L4)는 그
  상태머신이 부르는 한 동작일 뿐이다. **미커밋·실차 0회라 아직 뒤집힐 수 있다**
- **`guide_interfaces` 를 실제로 갈아끼우는 일.** 정의는 있고 쓰는 코드가
  없다 — `interfaces.md` 7절의 마이그레이션 4단계

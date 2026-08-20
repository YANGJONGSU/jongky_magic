# ADR 005: 야붐 보드에 `diff_drive_controller` 를 쓴다 (A안)

- **Status**: 채택 (Accepted). 실물에서 검증됨
- **Date**: 2026-08-11 (문서화 2026-08-20)

## Context

야붐 Rosmaster 보드는 **차체 속도를 직접 받는다** — `set_car_motion(v_x, v_y, v_z)`.
보드 펌웨어 안에 모터 PID 가 들어 있어서 좌우 바퀴 속도 배분과 폐루프 제어를
보드가 알아서 한다. **바퀴 각속도를 우리가 계산해 보낼 필요가 없다.**

그런데 `ros2_control` 은 반대 전제다. `diff_drive_controller` 가 `/cmd_vel` 을
받아 좌우 바퀴 각속도로 쪼개고, 하드웨어 인터페이스는 그 **관절 단위** 명령을
받는다. 층위가 한 겹 어긋난다.

## Decision

**A안 — `diff_drive_controller` 를 쓰고, 하드웨어 인터페이스에서 역산한다.**

```
/cmd_vel ─→ diff_drive_controller ─→ 좌우 바퀴 각속도 ─→ [역산] ─→ set_car_motion(vx, 0, vz)
                    │
                    └─→ /odom  (odom→base_footprint TF 는 EKF 가 낸다)
```

역산은 두 줄이고 (`jongky_system.cpp:412-415`), **바퀴가 두 개뿐이라 정보
손실이 없다.** 쪼갰다 합치는 왕복이지만 값은 보존된다.

### 버린 것 — B안: `/cmd_vel` 을 보드에 직결하는 단순 노드

코드는 훨씬 적지만 대가가 크다.

| | A안 | B안 |
|---|---|---|
| 오도메트리 | 컨트롤러가 만들어 줌 | **직접 구현** |
| 속도·가속도 제한 | 파라미터 한 줄 (`jongky_controllers.yaml:109-127`) | 직접 구현 |
| 트레드 보정 | 파라미터 한 줄 (`:59`) | 코드 수정 |
| mock 하드웨어 · 컨트롤러 교체 · 진단 | 전부 사용 가능 | 못 씀 |
| 시뮬 전환 | 하드웨어 플러그인만 교체 | 시뮬 경로를 따로 만들어야 함 |
| Nav2 연동 | 표준 경로 그대로 | 토픽은 맞출 수 있으나 관례 밖 |

> 비교표 원본은 저장소 **밖** 작업 노트
> `~/Documents/종키/hardware-interface-설계결정.md` 에 있다.
> 코드 쪽 흔적은 `jongky_system.cpp:410-411` 의 "설계 결정 A안" 주석뿐이다.

## Consequences

**이 결정이 만든 규약 — `jongky_hardware` 의 존재 이유가 여기서 나온다**

부호 변환을 **하드웨어 인터페이스 안에서만** 한다. 보드는 ROS 규약과 반대다
(`jongky_hardware/README.md:31-50`).

| 항목 | 보드 | 처리 |
|---|---|---|
| `+vx` | 물리적 **후진** | 보낼 때 부호 반전 (`jongky_system.cpp:422`) |
| `+vz` | 반시계 | REP-103 일치, 그대로 |
| 엔코더 증가(+) | 후진 | 읽을 때 반전 |
| 자이로 z(+) | 시계 | 읽을 때 반전 |

**이 반전을 URDF 나 컨트롤러 설정에 넣으면 안 된다.** 여기서만 처리해야
목업·시뮬·실물이 같은 인터페이스를 갖는다.

**대가로 지켜야 하는 것들**

- **`wheel_radius`·`wheel_separation` 이 세 곳에서 같아야 한다** — URDF xacro,
  `jongky_controllers.yaml:54-55`, 하드웨어 파라미터. xacro 프로퍼티에서
  자동으로 넘어가므로 손으로 적지 말 것 (`jongky_hardware/README.md:65-66`)
- **`enable_odom_tf: false`** — `odom->base_footprint` 의 주인은 EKF 하나다.
  둘 다 켜면 에러 한 줄 없이 두 추정이 번갈아 이긴다
  (`jongky_controllers.yaml:85-93`)
- **`base_frame_id: base_footprint`** — 기본값 `base_link` 로 두면 odom 이
  바퀴축 높이(33.5mm) 기준이 되어 지도가 그만큼 뜬다 (`:80-83`)
- **보드 속도 스케일 보정이 필요하다** — 보드가 자기 바퀴 크기 가정으로
  vx·vz 를 각속도로 바꾸는데 그게 우리 바퀴보다 커서, 비율(`board_vel_scale`
  1.391)만큼 키워 보낸다 (`jongky_system.cpp:419-421`)
- **`update_rate` 50 Hz 인데 보드 엔코더 보고는 25 Hz** 다. 위치 적분 odom 은
  정확하고 속도만 튀므로 `velocity_rolling_window_size` 로 평활한다
  (`jongky_controllers.yaml:9-13`, `:102`)

**검증 수단이 딸려 왔다** — `test/fake_board.py` 가 pty 로 가상 시리얼을 만들어
**부호 변환까지** 하드웨어 없이 검증한다 (`jongky_hardware/README.md:68-87`).

# jongky_control

종키 AMR의 `ros2_control` 컨트롤러 설정. 코드는 없고 설정과 런치뿐이다.

실제 하드웨어 통신은 `jongky_hardware`(예정)가 맡고, 로봇 모델은
`jongky_description`에 있다.

## 실행

```bash
# 하드웨어 없이 전 경로 검증 (목업)
ros2 launch jongky_control control.launch.py use_mock:=true

# RViz 같이
ros2 launch jongky_control control.launch.py use_mock:=true use_rviz:=true

# 실물 (jongky_hardware 구현 후)
ros2 launch jongky_control control.launch.py
```

## 구성

```
/cmd_vel (TwistStamped)
    │
    ▼
diff_drive_controller ──→ 좌우 바퀴 각속도 ──→ 하드웨어 인터페이스
    │                                              │
    │                                    use_mock  │  실물
    │                            GenericSystem ────┴──── jongky_hardware
    │
    └──→ /diff_drive_controller/odom  +  odom→base_footprint TF
```

컨트롤러 두 개가 올라간다.

| 컨트롤러 | 역할 |
|---|---|
| `joint_state_broadcaster` | 관절 상태를 `/joint_states` 로 발행 |
| `diff_drive_controller` | cmd_vel → 바퀴 속도, 엔코더 → 오도메트리 + TF |

## 알아둘 것 두 가지

**1. Jazzy 의 `cmd_vel` 은 `TwistStamped` 다.** 예전 `Twist` 가 아니다.
teleop 이나 Nav2 를 붙였는데 로봇이 안 움직이면 이것부터 확인할 것.

```bash
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.2}}}'
```

**2. 런치에서 `Command()` 결과는 `ParameterValue(value_type=str)` 로 감싼다.**
안 감싸면 launch 가 xacro 출력(XML)을 YAML 로 파싱하려다 죽는다.

```
Unable to parse the value of parameter robot_description as yaml.
```

## 설정 중 중요한 것

`config/jongky_controllers.yaml` 전체에 주석을 달아 두었다. 특히:

- **`base_frame_id: base_footprint`** — 기본값이 `base_link` 라 반드시 바꿔야 한다.
  안 바꾸면 odom 이 바퀴축 높이(33.5mm)를 기준으로 잡혀 지도가 그만큼 뜬다
- **`wheel_separation: 0.11625`** — 잠정값. 제자리 10바퀴 회전 후
  `/odom` yaw 와 실제 3600도를 비교해 보정한다.
  `wheel_separation_multiplier` 로 흡수하면 description 과의 일치를 안 깬다
- **가속 제한이 낮다** — 야붐 보드가 목표 속도까지 자체 램프를 태운다.
  `vx=0.15` 를 명령해도 1.5초 시점에 0.088 m/s 밖에 못 올라간다.
  컨트롤러가 그보다 급하게 명령해도 하드웨어가 못 따라오므로 낮게 잡았다
- **`update_rate: 50` vs 보드 리포트 25 Hz** — 절반의 주기는 갱신되지 않은
  값을 본다. 위치 적분으로 만드는 odom 포즈는 정확하고 속도만 튀는데,
  `velocity_rolling_window_size: 10` 으로 평활한다

## 부호는 여기서 다루지 않는다

야붐 보드는 `+vx` 가 물리적 후진이고 엔코더 증가도 후진이다.
**그 반전은 전부 `jongky_hardware` 안에서 처리한다.**

이 패키지와 URDF 의 `<ros2_control>` 인터페이스는 ROS 규약을 따른다 —
관절 속도 양수 = 전진. 그래야 목업·시뮬·실물이 같은 인터페이스를 갖고,
하드웨어를 갈아끼울 때 컨트롤러 설정을 안 건드린다.

## 검증 기록 (2026-08-11, 목업)

| 시험 | 명령 | 결과 |
|---|---|---|
| 직진 | `linear.x 0.2`, 3초 | odom x **0.3387 m**. 관절 10.11 rad × 0.0335 = 0.3387 로 일치 |
| 회전 | `angular.z 0.8`, 3초 | yaw **+68.4도**. 관절 좌 8.04 / 우 12.18 로 갈라짐 (반시계 시 왼쪽이 덜 도는 게 정상) |
| TF | | `odom → base_footprint` 발행 확인 |

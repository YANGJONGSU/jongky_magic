# jongky_description

종키 AMR의 xacro 로봇 모델. 메쉬 수령·검증 완료 상태입니다.

치수 근거, 실측값, 남은 확인 항목은 **[HANDOVER.md](HANDOVER.md)** 에 전부 정리되어 있습니다.
값을 고치기 전에 그 문서를 먼저 보세요.

## 빠르게 보기

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch jongky_description view_robot.launch.py
```

## 구조

```
urdf/
├── robot.urdf.xacro          진입점. 실행 인자 기본값만 정의
├── jongky.urdf.xacro         본체 정의 (치수 상수가 전부 파일 상단에 모여 있음)
└── common/insert_inertia.urdf.xacro   관성 텐서 매크로
meshes/visual/                base_link.stl, wheel.stl (mm 단위, URDF에서 0.001 스케일)
launch/view_robot.launch.py   RViz + joint_state_publisher_gui
scripts/                      실측 확인용 도구 (아래)
```

프레임은 `base_footprint` → `base_link` → 구동륜 2 + 센서 6, 총 10개입니다.

## 실행 인자

파일을 고치지 않고 실행 시 바꿀 수 있습니다.

| 인자 | 기본값 | 용도 |
|---|---|---|
| `laser_yaw_deg` | 180 | 라이다 0도를 로봇 정면에 맞추는 값 |
| `cam_tilt_deg` | 0 | 아스트라 숙임각 |
| `tof_pitch_deg` | 25 | TOF 숙임각 (근거리 바닥 감지) |
| `tof_yaw_deg` | 5 | TOF 좌우 벌어짐 |

## 실측 확인 도구

```bash
# 라이다 0도가 어느 쪽을 향하는지 (정면에 물체 놓고 실행)
ros2 run jongky_description check_laser_yaw.py

# 뎁스 카메라 최소 측정거리 실측
ros2 run jongky_description check_depth_min_range.py
```

## docs/conventions.md 와 다른 점

메쉬를 실제로 받아본 뒤 규약과 어긋난 부분이 생겼습니다. 원점 항목은 규약 문서를 고쳐 해소했고,
나머지 셋은 다음 메쉬 리비전에 적용할 목표로 남겨둡니다.

| 항목 | conventions.md | 현재 구현 |
|---|---|---|
| visual 포맷 | `.dae` (재질 포함) | `.stl` (수령본이 STL) |
| collision 메쉬 | `meshes/collision/*.stl` | 메쉬 없이 primitive (박스 1 + 구 4) |
| 링크 단위 분할 | 링크마다 파일 분리 | `base_link.stl` 하나에 판·젯슨·배터리·모터·캐스터 전부 |

- **원점**: conventions.md 쪽을 고쳤습니다 (2026-08). `base_footprint`가 바닥, `base_link`가 축 높이인 지금 구조가 ROS 관례이고 Nav2·SLAM이 기대하는 형태입니다.
- **collision**: conventions.md 스스로 "22cm급이면 원기둥이나 convex hull로 충분"이라고 했으니 primitive는 규약 취지에 맞습니다.
- **링크 분할**: 현재는 움직이는 부품이 바퀴뿐이라 실질 문제가 없습니다. 다만 서스펜션이나 팬틸트가 생기면 그때 재분할이 필요합니다.

## 남은 확인 항목

[HANDOVER.md](HANDOVER.md) 참고. 실물에서 확인해야 정해지는 것들입니다.

1. **라이다 yaw** — 제일 급합니다. 틀리면 지도가 안 나옵니다
2. 트레드 116.25 확정 (제자리 10바퀴 → `/odom`)
3. IMU 축 방향, 무게중심, 아스트라 최소 측정거리

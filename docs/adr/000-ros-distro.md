# ADR 000: ROS 2 Distro 선택

- **Status**: 채택 (Accepted). 실물에서 검증됨
- **Date**: 2026-08 (문서화 2026-08-20 — 결정은 이미 내려져 코드에 반영돼 있었다)

## Context

젯슨 Orin Nano Super 호스트가 **Ubuntu 22.04.5 / L4T R36.4 (JetPack 6.1)** 다
(`docker/README.md:10`). ROS 2 Jazzy 는 Ubuntu 24.04 를 요구하므로 호스트에
네이티브로 안 올라간다.

개발 노트북은 Ubuntu 24.04 다. 로봇과 노트북이 다른 distro 를 쓰면 같은
워크스페이스를 못 나누고, 시뮬↔실물 전환 때마다 다른 문제를 밟게 된다.

## Decision

**ROS 2 Jazzy 를 쓴다. 젯슨에서는 컨테이너 유저스페이스를 24.04 로 올려서
돌린다.** 컨테이너는 커널만 호스트 것을 공유하므로 이 조합이 성립한다
(`docker/README.md:6`).

```
dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04   베이스 (외부)
        │
ros_torch:jazzy                                   + PyTorch (직접 빌드)
        │
jongky:jazzy                                      + ros2_control 스택
```

- 컨테이너: Ubuntu 24.04 + ROS 2 Jazzy + CUDA 12.8 (`docker/README.md:11`)
- 개발 노트북: Ubuntu 24.04 + Jazzy — **컨테이너와 같은 환경** (`:12`)
- `ARG ROS_DISTRO=jazzy` (`docker/Dockerfile.robot:16`)

### 버린 것

- **호스트 네이티브 실행** — 22.04 이므로 Jazzy 가 안 된다
- **Humble(22.04 네이티브)** — 이 대안을 명시적으로 비교한 기록이 저장소에 없다.
  **확인 안 됨.** 근거로 남은 것은 "Jazzy 가 네이티브로 안 도니까 컨테이너를
  쓴다"는 서술뿐이다(`docker/README.md:3-4`)

## Consequences

**좋은 쪽**

- 로봇·노트북·시뮬이 같은 distro 라 launch 경로가 하나다
- CUDA 12.8 베이스라 온보드 추론 경로가 이미 열려 있다

**대가 — 전부 실제로 밟았다**

| | |
|---|---|
| `/cmd_vel` 이 `TwistStamped` 다 | Jazzy `diff_drive_controller` 기본값. 안 맞추면 **경로도 나오고 로그도 정상인데 바퀴만 안 돈다.** `nav2_params.yaml:81`·`:170`·`:273` 의 `enable_stamped_cmd_vel`, `teleop_key.py:5-7` |
| 표준 `teleop_twist_keyboard` 를 못 쓴다 | 같은 이유 — `Twist` 를 쏜다. `jongky_bringup/README.md 의 "키보드 텔레옵" 절` |
| apt 의 `ros-jazzy-rplidar-ros` 가 C1 을 모른다 | 2.1.0 / SDK 1.12.0 이라 스캔 시작이 실패한다. 소스 빌드로 갔고, 잘못된 버전이 조용히 잡히는 사고를 막으려 **이미지에서 아예 뺐다**(`docker/Dockerfile.robot:22`). `docs/troubleshooting.md:18-42` |
| `astra_camera` 의 Jazzy deb 이 없다 | 카메라 드라이버 결정으로 이어졌다 — [adr/004](004-astra-driver.md) |
| 베이스에 `ros-base` 만 있다 | `ros2_control` 계열이 통째로 없어서 `Dockerfile.robot:43-65` 가 얹는다 |
| 이미지가 16.1GB | 컨테이너를 건드리는 비용이 비싸다. `follow_service.py` 를 호스트로 뺀 판단의 배경이다(`follow_service.py 머리말 [왜 컨테이너가 아니라 호스트인가]`) |
| 컨테이너 안에 udev 심링크가 없다 | `--device` 가 심링크가 아니라 실제 노드를 요구한다. `run_robot.sh` 가 호스트에서 실경로로 풀어 환경변수로 넘긴다(`jongky_bringup/README.md 의 "포트 기본값은 환경변수에서 온다" 절`) |

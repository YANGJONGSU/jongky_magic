# docker

종키 온보드 실행 환경. 젯슨 호스트는 Ubuntu 22.04(JetPack 6)라 ROS 2 Jazzy가
네이티브로 안 돈다. 컨테이너 유저스페이스를 24.04로 써서 Jazzy를 올린다.

컨테이너는 커널만 호스트 것을 공유하므로 이 조합이 성립한다.

| | |
|---|---|
| 호스트 | Ubuntu 22.04.5, 커널 5.15.148-tegra, L4T R36.4 (JetPack 6.1) |
| 컨테이너 | Ubuntu 24.04 + ROS 2 Jazzy + CUDA 12.8 |
| 개발 노트북 | Ubuntu 24.04 + Jazzy — **컨테이너와 같은 환경** |

## 이미지

```
dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04   베이스 (외부)
        │
ros_torch:jazzy                                   + PyTorch (직접 빌드)
        │
jongky:jazzy                                      + ros2_control 스택  ← Dockerfile.robot
```

베이스에 `ros-base` 만 들어 있어 `ros2_control` · `controller_manager` ·
`diff_drive_controller` · `xacro` 가 전부 없다. `Dockerfile.robot` 이 그걸 얹는다.

## 빌드

젯슨에서:

```bash
cd ~/jongky_docker      # 또는 저장소의 docker/
docker build -f Dockerfile.robot -t jongky:jazzy .
```

## 실행

```bash
./run_robot.sh                                   # 대화형 셸
./run_robot.sh ros2 launch jongky_control control.launch.py
```

워크스페이스(`~/jongky_ws`)를 `/ws` 로 마운트한다. **소스를 이미지에 굽지
않는다** — 코드를 고칠 때마다 이미지를 다시 만들지 않기 위해서다.

```bash
# 컨테이너 안에서
cd /ws && colcon build --symlink-install && source install/setup.bash
```

## 시리얼 장치가 까다로운 지점

udev 심링크(`/dev/yahboom`)는 **컨테이너 안으로 그대로 넘어가지 않는다.**
`--device` 는 심링크가 아니라 실제 장치 노드를 요구한다.

`run_robot.sh` 는 호스트에서 심링크를 실경로로 풀어서 넘기고, 그 경로를
환경변수로 알려준다.

```bash
/dev/yahboom -> /dev/ttyUSB1     $JONGKY_YAHBOOM_PORT
/dev/rplidar -> /dev/ttyUSB0     $JONGKY_LIDAR_PORT
```

실행 시점에 다시 푸므로 **부팅 순서가 바뀌어도 안전하다** — udev 규칙의
안정성이 유지된다. `--privileged` 나 `-v /dev:/dev` 로 통째로 넘기는 방법도
있지만 필요 이상으로 권한을 준다.

컨테이너 안에서는 이렇게 쓴다:

```bash
ros2 launch jongky_control control.launch.py serial_port:=$JONGKY_YAHBOOM_PORT
```

## 네트워크

`--network host` 를 쓴다. ROS 2 DDS 는 멀티캐스트 디스커버리를 쓰는데
브리지 네트워크에서는 호스트 밖의 노드를 못 찾는다. `--ipc host` 와
`/dev/shm` 마운트는 같은 호스트 안 노드 간 공유메모리 전송을 위한 것이다.

학습 노트북(`192.168.129.97`)과 통신하려면 `ROS_DOMAIN_ID` 를 양쪽에서
맞춰야 한다.

> Wi-Fi 너머로 이미지를 흘리는 구간은 DDS 만으로는 지연·지터가 크다.
> `comms/` 의 zenoh 브리지(ADR 002)가 그 대응이다.

## 아직 안 넣은 것

- **라이다 드라이버** — `rplidar_ros` 는 의도적으로 뺐다 (Dockerfile.robot:22).
  호스트 쪽에서 돈다
- **GPU 런타임 플래그** — 지금은 구동 검증용이라 필요 없다.
  추론을 컨테이너에서 돌릴 때 `--runtime nvidia` 를 붙인다
- **`robot-localization`** — `robot.launch.py` 가 EKF 를 기본으로 띄우는데
  apt 목록에 없다. 젯슨에는 수동 설치돼 있어서 지금은 도는데, 이미지를
  새로 만들면 깨진다
- **`python3-pil`** — `guide_node.py:173` 의 비압축 이미지 폴백이 쓴다.
  `package.xml:25` 에는 선언돼 있으나 apt 목록에 없다

## 들어 있는 것 (예전에 위 목록에 있던 것들)

- **아스트라** — `openni2-camera` + 오르베 재배포 OpenNI2 덮어쓰기
  (`Dockerfile.robot:45, :76-83`). apt 판 `orbbec_camera` 로는 우리 카메라
  (PID `0401`)를 못 연다
- **Nav2 · slam_toolbox** — `Dockerfile.robot:43-44`

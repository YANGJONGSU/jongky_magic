# jongky_bringup

종키를 올리는 진입점. 구동계와 센서를 런치 하나로 묶는다.

```bash
ros2 launch jongky_bringup robot.launch.py
```

| 인자 | 기본값 | |
|---|---|---|
| `use_mock` | `false` | 목업 하드웨어. 보드 없이 전 경로 확인 |
| `use_rviz` | `false` | 젯슨에서는 보통 끄고 개발 PC 에서 따로 띄운다 |
| `use_lidar` | `true` | 라이다 빼고 주행만 볼 때 `false` |
| `serial_port` | `/dev/yahboom` | 야붐 제어보드 |
| `lidar_port` | `/dev/rplidar` | 라이다 |

두 포트 모두 udev 규칙이 만드는 심링크다. `ttyUSB0/1` 은 부팅 순서에 따라
바뀌므로 직접 쓰지 않는다.

## 올라오는 것 (실측)

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 50.07 Hz |
| `/odom` | `nav_msgs/Odometry` | 50.02 Hz |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 구독 |
| `/imu/data` | `sensor_msgs/Imu` | 50.03 Hz |
| `/scan` | `sensor_msgs/LaserScan` | 9.99 Hz, 721 포인트, 360도 |

```
TF   odom -> base_footprint -> base_link -> {laser, imu_link, wheels, ...}
```

`/cmd_vel` 이 **`TwistStamped`** 다. Jazzy 의 `diff_drive_controller` 기본값이고,
Nav2 쪽에서 맞춰 줘야 한다.

### 토픽 이름 리맵은 스포너에서 한다

컨트롤러는 `controller_manager` 프로세스 안에서 자기 노드를 만든다. 그래서
`ros2_control_node` 에 `remappings` 를 걸어도 **컨트롤러 토픽에는 안 먹는다.**
기본 이름은 `/diff_drive_controller/{cmd_vel,odom}` 과
`/imu_sensor_broadcaster/imu` 다. 관례 이름으로 내려고 스포너의
`--controller-ros-args` 를 쓴다 (`jongky_control/launch/control.launch.py`).

### 포트 기본값은 환경변수에서 온다

udev 심링크(`/dev/yahboom`, `/dev/rplidar`)는 **컨테이너 안에 존재하지 않는다** —
`--device` 가 심링크가 아니라 실제 노드를 요구하기 때문이다. `run_robot.sh` 가
호스트에서 실경로로 풀어 `JONGKY_YAHBOOM_PORT` / `JONGKY_LIDAR_PORT` 로 넘기고,
런치 기본값이 그 변수를 읽는다. 호스트에서 직접 돌릴 때는 변수가 없으므로
심링크로 떨어진다.

## 라이다 드라이버는 소스 빌드본이어야 한다

apt 의 `ros-jazzy-rplidar-ros` 는 **2.1.0 / SDK 1.12.0** 이고 **C1 을
모른다.** 스캔 시작이 `Cannot start scan: '80008002'` 로 실패한다. 그래서
Dockerfile 에서 그 패키지를 뺐다.

```bash
cd ~/jongky_ws
vcs import src < src/jongky_magic/robot/jongky_robot.repos
colcon build --packages-select rplidar_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

자세한 경위는 `docs/troubleshooting.md`.

## 키보드 텔레옵 (`tools/teleop_key.py`)

SLAM 맵핑 주행용. **표준 `teleop_twist_keyboard` 를 쓰면 안 된다** — 그건
`Twist` 를 쏘는데 Jazzy 의 `diff_drive_controller` 는 `TwistStamped` 를 받아서
**에러 없이 조용히 안 움직인다.**

```bash
ros2 run jongky_bringup teleop_key.py
ros2 run jongky_bringup teleop_key.py --speed 0.15 --out ~/waypoints_10f.yaml
```

| 키 | |
|---|---|
| `i` `,` `j` `l` `u` `o` | 전진·후진·좌회전·우회전·전진좌·전진우 |
| 스페이스 / `k` | 즉시 정지 |
| `z` `x` / `c` `v` | 최고 속도 / 각속도 ±10% |
| **`w`** | **지금 위치를 waypoint 로 저장** (이름을 물어본다) |
| `p` | 저장된 waypoint 목록 |
| `q` | 종료 (정지 명령을 내고 나간다) |

기본 속도는 **0.15 m/s** 다. 실차 한계는 0.40 이지만 맵핑은 천천히 돌아야
스캔이 촘촘히 쌓이고 루프도 잘 닫힌다. 어떤 값을 주든 실차 한계
(0.40 / 1.50)를 넘지 않게 자른다.

`w` 는 `map -> base_footprint` TF 를 읽으므로 **SLAM 이 떠 있어야** 한다.
강의장 앞에 세우고 찍으면 그대로 Nav2 목표점이 된다.

### 현장 맵핑 절차

`scripts/` 의 단축 명령을 쓴다 (`scripts/install.sh` 로 한 번 설치).
**7인치 터치스크린 + USB 키보드만으로 돌아간다** — 네트워크가 필요 없다.
건물 WiFi 는 층마다 서브넷이 갈려 있어서 11층에 올라가면 SSH 가 끊긴다.

```bash
jcheck        # 맵핑 전 점검. /map 이 없으면 주행해 봐야 안 쌓인다
jmap 10f      # SLAM + rosbag  (터미널 1)
jdrive 10f    # 텔레옵          (터미널 2)  ← w 로 강의장 앞 waypoint
jsave 10f     # 지도 저장       (터미널 3)
jstop         # 정리
```

컨테이너를 하나만 띄우고 나머지는 거기 붙는다. 터미널마다 따로 띄우면
장치를 서로 뺏고 `/map` 도 공유되지 않는다.

층마다 따로 해야 한다. `map` 프레임은 2D 평면이라 10층과 11층을 한 지도에
못 담는다.

bag 을 같이 받는 이유는 현장이 다시 가기 비싸기 때문이다. 파라미터를 바꿔
지도를 다시 뽑을 수 있고, 카메라 영상은 Cosmos 씨앗이 된다.

## 아직 이 런치에 없는 센서

| | 상태 |
|---|---|
| 아스트라 (뎁스) | **동작한다.** 단 `orbbec_camera` 가 아니라 `openni2_camera` + 오르베 재배포 OpenNI2 조합이다 (`docker/Dockerfile.robot` 주석 참조). 최소거리 실측은 미완 |
| IMX219 (CSI) | **동작한다.** `jetson-io` 로 `imx219-dual` 오버레이 적용 후 재부팅. CAM0 포트에 물려 있다 |
| TOF ×2 | 40핀 I2C. 노드가 없고, VL53L0X 주소가 둘 다 `0x29` 라 충돌부터 풀어야 한다 |

카메라 둘은 동작 확인만 됐고 아직 이 런치에 안 얹혀 있다.
`use_camera` / `use_tof` 인자로 붙일 것.

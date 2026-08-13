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

## 아직 없는 센서

| | 상태 |
|---|---|
| 아스트라 (뎁스) | `astra_camera` 가 이미지에 없다. 최소거리 실측도 미완 |
| IMX219 (CSI) | `/dev/video*` 자체가 안 잡힌다. `jetson-io` 로 디바이스 트리 설정 필요 |
| TOF ×2 | 40핀 I2C. 노드가 없고, VL53L0X 주소가 둘 다 `0x29` 라 충돌부터 풀어야 한다 |

셋 다 붙으면 이 런치에 `use_camera` / `use_tof` 로 얹는다.

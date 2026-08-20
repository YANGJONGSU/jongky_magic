# jongky_bringup

종키를 올리는 진입점. 구동계와 센서를 런치 하나로 묶는다.

```bash
ros2 launch jongky_bringup robot.launch.py
```

| 인자 | 기본값 | |
|---|---|---|
| `use_ekf` | `true` | **끄지 말 것.** `odom -> base_footprint` 의 유일한 주인이다 |
| `use_watchdog` | `true` | 바퀴와 자이로의 회전 불일치를 `/mapping_alert` 로 알린다 |
| `use_mock` | `false` | 목업 하드웨어. 보드 없이 전 경로 확인 |
| `use_rviz` | `false` | 젯슨에서는 보통 끄고 개발 PC 에서 따로 띄운다 |
| `use_lidar` | `true` | 라이다 빼고 주행만 볼 때 `false` |
| `serial_port` | `/dev/yahboom` | 야붐 제어보드 |
| `lidar_port` | `/dev/rplidar` | 라이다 |

두 포트 모두 udev 규칙이 만드는 심링크다. `ttyUSB0/1` 은 부팅 순서에 따라
바뀌므로 직접 쓰지 않는다.

### EKF 가 `odom -> base_footprint` 를 낸다

`jongky_controllers.yaml` 의 `diff_drive_controller` 는 `enable_odom_tf: false`
라 TF 를 내지 않는다. 그래서 **`robot_localization` 의 `ekf_node` 가 그 프레임의
유일한 주인**이다. 없으면 TF 트리가 `base_footprint` 위에서 끊겨 SLAM·Nav2·
waypoint 저장이 통째로 죽는다 — `use_ekf:=false` 로 끄든, 패키지가 안 깔려
있든 결과는 같다.

`ros-jazzy-robot-localization` 은 `docker/Dockerfile.robot` 의 apt 목록에 있다.
젯슨 호스트에는 수동으로도 깔려 있어서 예전에는 목록에서 빠진 것을 모르고
돌았다. 그 항목을 지우면 이미지를 새로 만드는 순간 TF 가 끊긴다.

`jcheck` 가 `/ekf_filter_node` 와 `/odometry/filtered` 를 확인하는 이유가 이것이다.

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
./fetch_models.sh --check   # 나가기 전, 인터넷 되는 자리에서. 탐지 가중치 확인
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

## IMX219 화각(HFOV) 실측

**아직 안 했다.** `jongky_guide/tools/follow_service.py` 는 지금 렌즈 공칭값
62.2도를 쓰고, 사람까지의 거리를 여기서 역산한다.

```
focal_px = (640/2) / tan(HFOV/2)          # 62.2도 → 530.5 px
distance = PERSON_HEIGHT_M * focal_px / bbox_높이
```

**거리 추정이 화각에 직접 비례한다.** 화각이 10% 틀리면 거리도 10% 틀리고,
"뒤처졌다" 판정 문턱이 그만큼 통째로 밀린다. 아스트라는 `camera_info` 의 K
행렬에서 57.86도를 실측해 썼지만 IMX219 는 그 경로가 없어서 직접 재야 한다.

값은 **소스에 안 박혀 있다.** 재고 나서 코드를 고칠 필요가 없다:

```bash
export JONGKY_HFOV_DEG=61.4            # 또는
python3 follow_service.py --hfov 61.4 --person-height 1.68
```

### 준비물

줄자, 폭을 정확히 아는 평평한 표적(A0 폼보드·화이트보드·문틀 — 폭 `W`),
그리고 로봇 자체(카메라를 손에 들지 말 것 — 장착 상태 그대로 재야 한다).

### 1) 프레임 한 장 뜨기

**서비스가 쓰는 파이프라인 그대로** 찍어야 한다. `nvarguscamerasrc` 는 센서
모드에 따라 크롭이 달라서, 해상도나 모드를 바꾸면 화각 자체가 바뀐다.

표적을 카메라 광축에 **수직**으로, 화면 **가운데**에 오게 세운다. 거리 `D` 는
줄자로 렌즈 앞면부터 잰다. 2m 를 권한다 — 너무 가까우면 왜곡이, 너무 멀면
픽셀 분해능이 문제가 된다.

```bash
python3 - <<'EOF'
import cv2
pipe = ("nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
        "video/x-raw,format=BGR,width=640,height=360 ! appsink")
cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
for _ in range(15):          # 노출·화이트밸런스가 잡힐 때까지 버린다
    ok, f = cap.read()
cv2.imwrite("/tmp/hfov.png", f)
print(ok, f.shape)           # (360, 640, 3) 이어야 한다
EOF
```

### 2) 표적의 좌우 끝 픽셀 읽기

`/tmp/hfov.png` 를 열어 표적 왼쪽 끝 `x1`, 오른쪽 끝 `x2` 를 읽는다
(GIMP·`eog` 는 커서 좌표를 표시한다). `p = x2 - x1`.

### 3) 계산

```bash
python3 -c '
import math
W, D, p = 1.20, 2.00, 318        # 표적 폭(m), 거리(m), 화면상 폭(px)
f = p * D / W
print(f"focal_px {f:.1f}   HFOV {2*math.degrees(math.atan(320/f)):.2f}도")'
```

위 예시 숫자는 공칭값(62.2도)을 되돌려 주는 값이라 계산이 맞는지 확인하는
데 쓸 수 있다 — `focal_px 530.0  HFOV 62.24도` 가 나온다.

거리를 바꿔(예: 1.5m·2.5m) 두세 번 재서 **화각이 같게 나오는지** 본다.
1도 넘게 흔들리면 표적이 광축에 수직이 아니거나 거리 측정이 틀린 것이다.

### 4) 사람 거리로 검증하고 기준 신장 보정

화각을 넣고 서비스를 띄운 뒤, 키를 아는 사람을 줄자로 잰 거리에 세운다.

```bash
JONGKY_HFOV_DEG=61.4 python3 follow_service.py
curl -s localhost:8641/follower       # distance_m 과 calib 를 함께 본다
```

`distance_m` 이 줄자와 어긋나면 남은 오차는 bbox 높이 쪽이다 — 후면 카메라가
낮게 달려 있어 가까운 사람은 머리나 발이 잘리고, 그만큼 멀게 나온다.
`PERSON_HEIGHT_M` 은 그 편향까지 흡수하는 **유효값**이라 실제 신장과 달라도 된다.

```
새 기준 신장 = 지금 기준 신장 × (줄자 거리 / 보고된 거리)
```

**실제로 쓰는 거리(2~4m)에서** 맞추면 된다. 전 구간에서 맞출 수는 없다.

### 5) 남기기

- `follow_service.py` 의 `DEFAULT_HFOV_DEG` / `DEFAULT_PERSON_HEIGHT_M` 을 고치고
  주석의 `(미실측)` 을 `(실측 YYYY-MM-DD)` 로 바꾼다. 안 그러면 다음 사람이
  공칭값을 실측값으로 착각한다
- 지금 도는 서비스가 어떤 값을 쓰는지는 `curl localhost:8641/follower` 의
  `calib` 필드에 나온다 (`measured: false` 면 아직 공칭값이다)

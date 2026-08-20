# jongky_hardware

종키 AMR 의 `ros2_control` 하드웨어 인터페이스. 야붐 Rosmaster 제어보드와
시리얼로 통신한다.

`diff_drive_controller` 가 주는 좌우 바퀴 각속도를 보드가 이해하는 차체 속도로
바꿔 보내고, 엔코더를 관절 위치·속도로 되돌린다.
**보드와 ROS 의 부호 차이를 흡수하는 것이 이 패키지의 핵심 역할이다.**

## 구조

```
diff_drive_controller
        │  좌우 바퀴 각속도 (rad/s, 전진이 양수)
        ▼
JongkySystemHardware      ← 부호 변환, 단위 변환, 역기구학
        │  vx, vz (보드 부호)
        ▼
YahboomBoard              ← 프레임 인코딩/디코딩, 시리얼, 수신 스레드
        │  115200 8N1
        ▼
   /dev/yahboom
```

| 파일 | 역할 |
|---|---|
| `yahboom_board.{hpp,cpp}` | 보드 프로토콜. ROS 를 모르고 부호도 다루지 않는다 |
| `jongky_system.{hpp,cpp}` | `ros2_control` 인터페이스. 부호·단위 변환은 전부 여기 |
| `test/fake_board.py` | 보드 시뮬레이터. 하드웨어 없이 전 경로 검증 |

## 부호 규약 — 제일 중요한 부분

야붐 보드는 ROS 규약과 반대다. 실물 계측으로 확정했고 교차검증 근거는
작업 노트의 `야붐보드-레퍼런스.md` 4절에 있다.

| 항목 | 보드 | 이 패키지의 처리 |
|---|---|---|
| `+vx` | 물리적 **후진** | 보낼 때 **부호 반전** |
| `+vz` | 반시계 | REP-103 일치, 그대로 |
| 엔코더 증가(+) | 후진 | 읽을 때 **부호 반전** |
| 자이로 z(+) | 시계 | 읽을 때 **부호 반전** |
| 엔코더 ch2 | **오른쪽** 바퀴 | |
| 엔코더 ch3 | **왼쪽** 바퀴 | |
| 엔코더 ch1 · ch4 | 미사용 (X3 4륜 차종의 남는 채널) | 무시 |

`vx` 는 뒤집는데 `vz` 는 안 뒤집는다. 보드의 채널↔좌우 배정이 물리 배선과
반대이고 모터 극성도 반대라, 두 반전이 회전에서만 상쇄되기 때문이다.

**이 반전을 URDF 나 컨트롤러 설정에 넣지 말 것.** 여기서만 처리해야
목업·시뮬·실물이 같은 인터페이스를 갖는다.

## 파라미터

URDF 의 `<ros2_control><hardware>` 안에 `<param>` 으로 준다.

| 이름 | 기본값 | 설명 |
|---|---|---|
| `serial_port` | `/dev/yahboom` | udev 심링크. `99-jongky.rules` 참조 |
| `baud_rate` | 115200 | |
| `car_type` | 1 | `CARTYPE_X3`. 틀리면 보드가 명령을 다르게 해석한다 |
| `counts_per_rev` | 1960 | **[확정]** 줄자 실측 1945 와 무부하 1973 의 절충 |
| `wheel_radius` | 0.0335 | `jongky_description` 의 값이 자동으로 넘어온다 |
| `wheel_separation` | 0.11909 | **[확정]** 제자리 5바퀴 ×3회. `/odom` 역산은 쓰지 말 것 — 그 값이 이 상수에서 나오므로 순환이다 |

`wheel_radius` 와 `wheel_separation` 은 xacro 프로퍼티에서 직접 넘어오므로
URDF 의 형상과 어긋날 수 없다. 손으로 적지 말 것.

## 하드웨어 없이 시험하기

보드 시뮬레이터가 pty 로 가상 시리얼 포트를 만든다. 실물과 같은 부호 규약으로
동작하므로 **부호 변환까지 검증된다.**

```bash
# 터미널 1 — 시뮬레이터. 첫 줄에 포트 경로가 찍힌다
ros2 run jongky_hardware fake_board.py
# /dev/pts/9

# 터미널 2 — 그 포트로 스택 기동
ros2 launch jongky_control control.launch.py serial_port:=/dev/pts/9

# 터미널 3 — 명령
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.2}}}'
```

시뮬레이터 쪽에 `RX MOTION vx=-0.2000` 이 찍히면 부호 변환이 맞는 것이다
(ROS 전진 `+0.2` → 보드 `-0.2`).

## 구현하며 밟은 지뢰

앞으로 이 코드를 고칠 사람이 같은 데서 넘어지지 않도록 남긴다.

### 1. 이름으로 접근하는 상태/명령 API 는 실시간 안전하지 않다

`get_command("joint/velocity")` 와 `set_state("joint/position", v)` 는
`wait_until_set = true` 로 동작한다. 헤더에도 *"not real-time safe"* 라고
적혀 있다. `read()` / `write()` 안에서 쓰면 **컨트롤러 매니저가 통째로 멈춘다** —
서비스 응답조차 안 되어 `list_controllers` 가 타임아웃 난다.

인터페이스 핸들을 `on_activate` 에서 한 번만 잡고, `read()`/`write()` 에서는
논블로킹 오버로드를 쓴다.

```cpp
// on_activate
left_cmd_handle_ = get_command_interface_handle(left_joint_ + "/velocity");

// write()
std::ignore = get_command<double>(left_cmd_handle_, wl, false);  // wait=false
```

### 2. 포트가 열렸다고 보드가 살아 있는 건 아니다

`last_rx_ns_` 를 `open()` 시각으로 초기화하면 프레임을 하나도 못 받아도
"살아 있음" 으로 판정된다. 보드 전원이 꺼져 있어도 연결 성공으로 뜬다.

`rx_count_` 를 따로 두어 **최초 프레임 수신 여부**(`has_data()`)와
**최근 통신 여부**(`is_alive()`)를 구분한다.

### 3. 프레임 길이 필드는 페이로드 + 3 이다

```
0xFF 0xFB [LEN] [TYPE] [페이로드…] [체크섬]
LEN = 페이로드 길이 + 3
수신자는 TYPE 뒤로 LEN-2 바이트를 읽고 마지막을 체크섬으로 본다
```

시뮬레이터를 처음에 `+2` 로 만들어 1바이트 어긋났고, 프레임이 하나도
파싱되지 않았다. 실물 계측으로 검증된 리더 로직과 대조해서 잡았다.

**송신과 수신은 device id 도 체크섬 계산법도 다르다.** 헷갈리기 쉬운 지점.

```
송신  0xFF 0xFC [LEN] [FUNC] …  CHECKSUM = (전체 합 + (257 - 0xFC)) & 0xFF
수신  0xFF 0xFB [LEN] [TYPE] …  CHECKSUM = (LEN + TYPE + 페이로드 합) % 256
```

### 4. pty 로 시험할 때는 raw 모드 필수

프레임 타입이 하필 `REPORT_SPEED = 0x0A`(LF), `REPORT_ENCODER = 0x0D`(CR) 라
pty 기본 모드의 개행 변환에 정통으로 걸린다. 시뮬레이터가 master·slave 양쪽에
`tty.setraw()` 를 건다.

### 5. 스포너 파일 락

`ros2 run controller_manager spawner` 가 `~/.ros/locks/` 에 파일 락을 쓴다.
스포너를 강제 종료하면 락이 남아 다음 실행이 20초씩 기다린다.

```
Failed to acquire lock in 20 seconds. Attempt 1 of 5 failed.
```

이게 보이면 남은 spawner 프로세스를 죽이면 된다.

## 안전

- `write()` 는 명령값이 유한하지 않으면 정지 명령을 보낸다
- `on_deactivate` 와 `close()` 에서 정지 명령을 여러 번 보낸다 (패킷 유실 대비)
- `read()` 는 통신 두절 시 `ERROR` 를 반환한다. 컨트롤러 매니저가 하드웨어를
  비활성화하므로 명령이 끊긴 채 계속 달리는 상황은 생기지 않는다
- 배터리가 3S 기준 10.5V 아래면 `on_configure` 에서 경고한다

## 아직 안 한 것

- **배터리 발행.** `sensor_msgs/BatteryState` 로 내보낼 것. 지금은
  `jongky_system.cpp:221-225` 가 10.5V 아래에서 경고 로그만 남긴다

## 끝난 것 (예전에 여기 "아직 안 한 것" 으로 적혀 있던 것들)

- **IMU 발행** — 된다. `jongky_system.cpp:243-249` 가 핸들을 잡고
  `:369-389` 가 orientation·gyro·accel 을 채운다.
  `jongky_controllers.yaml:21-42` 의 `imu_sensor_broadcaster` 가 실측 공분산과
  함께 발행하고, `control.launch.py:87-93` 이 `/imu/data` 로 리맵한다
- **실물 검증** — 됐다. 젯슨에서 빌드해 실차로 돌렸고, 엔코더 원점 버그를
  실차에서 잡았다(`docs/troubleshooting.md:44-61`). IMU 공분산도 10층 주행
  bag 에서 실측했다

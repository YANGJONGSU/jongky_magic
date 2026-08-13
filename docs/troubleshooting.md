# Troubleshooting

> 문제당 네 줄: 증상 / 원인 / 해결 / 알아낸 방법. 마지막 줄이 가장 변별력 있는 면접 질문의 답입니다.
> 통합 당번이 통합 사이클마다 갱신.

## 형식

```
### [짧은 제목]
- 증상:
- 원인:
- 해결:
- 알아낸 방법:
```

---

### RPLIDAR C1 이 Cannot start scan 으로 죽는다
- 증상: `rplidar_composition` 이 S/N·펌웨어·health 는 정상으로 읽고 `Start`
  까지 찍은 뒤 `Cannot start scan: '80008002'` → `Failed to set scan mode`
  로 종료. `scan_mode:=Standard` 를 주면 `'80008000'` 으로 바뀔 뿐이다.
- 원인: apt 의 `ros-jazzy-rplidar-ros` 는 2.1.0 이고 SDK 1.12.0 을 쓴다.
  패키지 설명이 "support rplidar A2/A1 and A3/S1" 이고 런치 파일도 a3·s1
  뿐이다 — **C1 이 나오기 전 버전이다.** 지원 모드 목록에 C1 이 없으니
  express scan 협상이 타임아웃(`80008002` = `RESULT_OPERATION_TIMEOUT`)
  하고, 모드 이름을 직접 주면 매칭에 실패한다(`80008000` =
  `RESULT_INVALID_DATA`).
- 해결: Slamtec `ros2` 브랜치를 소스로 받아 워크스페이스에서 빌드한다
  (SDK 2.1.0, `rplidar_c1_launch.py` 포함). `robot/jongky_robot.repos` 에
  커밋을 고정해 뒀다. Dockerfile 에서는 apt 판을 **뺐다** — 오버레이를
  안 잡으면 조용히 깨진 버전이 쓰이기 때문이다.
  ```bash
  ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/rplidar frame_id:=laser
  # → current scan mode: Standard, 5 Khz, max 16.0 m, 10.0 Hz
  ```
- 알아낸 방법: 파라미터를 의심해 보드레이트(256000→460800)와 `scan_mode` 를
  차례로 바꿔 봤지만 오류 코드만 바뀌었다. 방향을 돌린 건 **연결 자체는
  성공한다**는 점이다 — 시리얼·전원·권한이 다 정상인데 스캔 명령만 실패하면
  남는 건 프로토콜 협상이고, 그건 SDK 버전 문제다. 로그의
  `SDK Version: '1.12.0'` 과 `dpkg -l` 의 지원 모델 목록에서 확인했다.
  같은 장비를 순수 시리얼(`tools/lidar_yaw.py`)로는 계속 돌릴 수 있었던
  것도 "하드웨어는 멀쩡하다" 는 근거였다.

### 라이다가 전 방향 3~6cm 로 균일하게 나온다
- 증상: 스캔은 정상적으로 도는데 모든 각도의 거리가 34~63mm 로 비슷하다.
  사방이 막힌 것처럼 보이지만 실제로는 트인 공간이다.
- 원인: 구입 시 감겨 있던 **보호필름을 안 벗겼다.** 레이저가 필름에서 바로
  반사돼 그 너머를 못 본다.
- 해결: 필름 제거. 바로 228~2560mm 로 정상화.
- 알아낸 방법: 고장이면 값이 안 나오거나 튀는데, **값은 멀쩡하고 전부 작기만**
  했다. 거리 분포가 물리적으로 불가능(로봇이 5cm 상자 안에 있다는 뜻)하다는
  점에서 센서 앞을 의심했다.

### URDF 를 고쳤는데 TF 가 안 바뀐다
- 증상: xacro 를 수정하고 재배포했는데 `ros2 run tf2_ros tf2_echo` 결과가
  그대로다.
- 원인: 빠른 재배포가 `ros2_control_node` 만 죽였다. **TF 를 발행하는 건
  `robot_state_publisher`** 고, 그건 옛 URDF 를 들고 살아 있었다.
- 해결: URDF 를 고쳤으면 런치 그룹 전체를 죽이고 다시 띄운다.
  ```bash
  pkill -f ros2_control_node; pkill -f robot_state_publisher; pkill -f "ros2 launch"
  ```
- 알아낸 방법: 컨트롤러 YAML 만 고칠 때는 같은 절차로 잘 반영됐다는 점.
  차이는 "누가 그 값을 들고 있느냐" 였다.

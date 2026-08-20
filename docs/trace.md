# Trace — 임무 한 건의 전 경로

> 통합 당번이 매 사이클 갱신. G0 이후 stub 기준으로 먼저 채우고, G5(통합) 이후 실제 경로로 교체.
> 이 문서를 안 보고 화이트보드에 그릴 수 있으면 면접 준비는 끝입니다.

각 홉마다: 노드/토픽/파일 + "여기서 실패하면 어떻게 되는가" 한 줄.

> **2026-08-20 갱신.** stub 기준이 아니라 **저장소에 실제로 있는 코드**를 읽어서
> 채웠다. 아래 파일:줄은 전부 확인한 것이다. 다만 **이 경로 전체가 한 번도
> 끝까지 돌아 본 적이 없다** — `jongky_guide` 는 아직 빌드·실행 이력이 0건이다
> (`README.md:52-53`, `전체-작업계획.md` A1e). 그러니 이 표는 "코드가 이렇게
> 이어져 있다" 이지 "이렇게 동작하는 것을 봤다" 가 아니다.

---

## 8홉 요약

| # | 홉 | 노드 / 토픽 / 파일 | 실패 시 |
|---|---|---|---|
| 1 | 질의 입력 → 접수 | 터치: `web/index.html::loadDestinations() · go()` → `POST /api/go` · 음성: `voice_node.py:59-127` → `/guide/destination` · PTT: `guide_node.py::listen_and_go()` · `::_listen_loop()` / 접수는 `guide_node.py::do_POST()`, `::_on_destination_topic()` | UI 가 400 과 에러 문자열을 그대로 화면에 띄운다(`index.html::go() 의 실패 분기`). 토픽 경로는 `warn` 한 줄만 남고 조용히 버려진다(`guide_node.py::_on_destination_topic()`) |
| 2 | 목적지 해석 · POI 검증 | `guide_node.py::_resolve()` — ①공백 제거 부분일치 ②실패 시 `brain.py::resolve_destination()` LLM. 등록 여부는 `::start_guiding()` 이 판정한다 | 둘 다 실패하면 `None` → "어디로 갈지 알아듣지 못했습니다" 를 말하고 화면 선택으로 되돌린다(`guide_node.py::_listen_loop()`). **지어낸 목적지로 출발하지 않는다** — LLM 응답도 목록과 대조해 걸러낸다(`brain.py::resolve_destination() 의 목록 대조`) |
| 3 | 임무 상태머신 진입 | `guide_node.py::start_guiding()` → 워커 스레드 `::_guide_loop()`. 상태는 `GuideState` 의 status 문자열 | 미측위·미등록·이미 안내 중이면 각각 거절한다. 셋 다 400 + 사유 문자열이라 **조용한 실패가 없다** |
| 4 | 목표 전달 (VDA5050 / Nav2) | **VDA5050 경유 없음.** `guide_node.py::_to_pose()` → `BasicNavigator.goToPose()` → `navigate_to_pose` 액션 | AMCL 이 `map->odom` 을 안 내면 `frame_id: map` 목표를 변환 못 해 **접수된 것처럼 보이고 안 간다**(`guide_node.py::set_start()` 앞 주석). 그래서 `::set_start()` 가 waypoint 를 초기 위치로 박는다 |
| 5 | 층 전환 (필요 시) | **작업 중 — 방금 들어왔다.** `fleet/guide_mission/guide_mission/{floors,detect,transfer,effects}.py` + `guide_node.py::_mission_loop()` · `NavEffects` · `load_map()` · `_relocalize_at()` | 층 판정이 확신 없으면 **모른다고 말하고 멈춘다** (`detect.py` 머리말) — 조용히 한 층을 찍지 않는다. 엘리베이터 앞까지 못 가면 층 전환 자체를 접는다(`transfer.py` 전이표의 `nav_fail → fault`) |
| 6 | Nav2 경로 추종 | `nav2_params.yaml` — AMCL `:12-51`, SmacPlanner2D `:127-149`, RPP `:101-125`, 코스트맵 `:195-262`, `velocity_smoother` `:264-283` → `/cmd_vel` (`geometry_msgs/TwistStamped`) | 경로를 못 내면 `TaskResult` 가 `FAILED` 로 떨어지고 8홉으로 간다. **`enable_stamped_cmd_vel` 이 빠지면 경로도 나오고 로그도 정상인데 바퀴만 안 돈다** (`:76-81`, `:165-170`, `:268-273`) |
| 7 | ros2_control → UART(야붐 보드) → 모터 | `control.launch.py:75-85` 리맵 → `jongky_controllers.yaml:45-55` `diff_drive_controller` → `jongky_system.cpp:396-430` `write()` → `yahboom_board.cpp:163-...` `set_motion()` → `:159` `::write(fd_)` → **UART 115200 8N1** (`:32`, `:55-105`) → 야붐 Rosmaster 보드 | 전송 실패는 `ERROR` 반환(`jongky_system.cpp:422-427`), 통신 두절은 `read()` 가 `ERROR`(`:319`) → 컨트롤러 매니저가 하드웨어를 비활성화한다. **명령이 끊긴 채 계속 달리는 상황은 안 생긴다**(`jongky_hardware/README.md:154-160`) |
| 8 | 결과 반환 | `guide_node.py::_drive_to()` 의 `TaskResult` 판정 → `::_guide_loop()` → `speech.py:54-58` piper TTS + `::_publish_status()` → `/guide/status` (`std_msgs/String`, JSON) | TTS 가 없으면 로그만 남기고 넘어간다(`speech.py:42-46`) — **음성 실패가 주행을 막지 않는다.** `/guide/status` 는 지금 구독자가 없다 (UI 는 `/api/status` 폴링을 쓴다) |

---

## 홉별 상세

### 1. 질의 입력 → 접수

입구가 셋이고, 셋 다 같은 `start_guiding()` 으로 모인다.

| 입구 | 경로 | 비고 |
|---|---|---|
| 터치스크린 버튼 | `index.html::renderGrid()` → `:148-158` `POST /api/go` → `guide_node.py::do_POST()` 의 `/api/go` | 버튼 목록은 waypoint YAML 이 정한다(`index.html::loadDestinations() · renderGrid()`, `guide_node.py::do_GET()` 의 `/api/destinations`) |
| 상시 음성 | `voice_node.py:59-87` 볼륨 게이트 → `:89-116` Whisper → `:117-127` 문자열 매칭 → `:113` `/guide/destination` 발행 | 별도 프로세스다. 죽어도 UI 는 산다 |
| PTT 버튼 | `index.html 의 PTT 버튼 핸들러` → `POST /api/listen` → `guide_node.py::listen_and_go()` → `::_listen_loop()` → `::_resolve()` | `--mic` 를 줘야 켜진다(`main()` 의 `--mic` 인자) |

`/guide/destination` 은 `std_msgs/String` 이고 payload 가 waypoint 이름 문자열
그대로다. 검증 계층이 따로 없어서 **아무나 발행하면 로봇이 출발한다.**

### 2. 목적지 해석 · POI 검증

싼 경로부터 시도한다(`guide_node.py::_resolve() 주석` 주석).

1. Whisper 텍스트 + 공백 제거 부분일치 — 온보드, 네트워크 불필요
2. 실패하면 LLM (`brain.py::resolve_destination()`) — "삼백사호" 같은 표현용

원본 오디오를 LLM 에 직접 넘기는 3단계는 **의도적으로 뺐다.** ollama 가
오디오 필드를 통째로 무시해서 근거 없는 목적지가 나온다(`brain.py 의 오디오 입력 미지원 주석`,
실측 근거 포함). `_resolve()` 의 `wav` 인자는 그때를 대비한 빈 자리다.

LLM 은 관제 노트북(`192.168.129.97:11434`)에 있다(`brain.py 의 OLLAMA_URL 기본값`). 층이
갈리면 못 닿고, 그때는 1번 경로만 남는다.

### 3. 임무 상태머신 진입

**"상태머신" 이라고 부를 만한 것은 아직 없다.** 실제로 있는 것은
`GuideState.status` 문자열과 `_guide_loop()` 의
`while` 하나다.

```
idle → navigating → arrived        (정상)
              ↓  → waiting → navigating   (사람이 뒤처짐, :271-313)
              ↓  → alert            (VLM 이 위급 판정, :263-267)
              ↓  → failed           (경로 없음, :375-377)
```

층 전환·다단계 임무·재시도를 담을 자리(`fleet/guide_mission`)에 오늘
상태머신이 들어왔다 — 5번 홉 참조. `guide_node` 쪽 상태에도 `transfer` ·
`fault` 가 늘었다(`GuideState`). **다만 아직 미커밋이고 실차 검증 0회다.**

### 4. 목표 전달 — VDA5050 홉은 비어 있다

**저장소 전체에 VDA5050 코드가 0건이다.** `fleet/guide_vda5050/README.md:3`
이 "VDA5050 v3.0 어댑터 (MQTT/JSON). (TODO)" 세 줄이고, MQTT 클라이언트도
JSON 스키마도 없다. `docs/adr/001-vda5050.md` 참조.

지금 실제로 도는 것은 `BasicNavigator` 가 `navigate_to_pose` 액션을 직접
치는 것이다. 관제·플릿 계층을 거치지 않는다.

목표 pose 는 맵핑 주행 중 `teleop_key.py` 의 `w` 로 찍어 둔 YAML 에서
그대로 온다(`guide_node.py::_load_waypoints()` · `::_to_pose()`). 좌표계가 같은 `map` 이라 변환이 없다.

### 5. 층 전환 — 작업 중 (2026-08-20 저녁에 들어왔다)

**이 절은 오늘 오전까지 "코드 0건" 이었다.** 다른 트랙이 지금 넣고 있고,
아래는 **미커밋 상태(`git status` 기준 untracked)** 의 코드를 읽은 것이다.
아직 한 번도 실차에서 돌지 않았다.

| | |
|---|---|
| 층별 자원 대장 | `fleet/guide_mission/guide_mission/floors.py` — 층 ↔ 지도 ↔ waypoint 짝을 `floors.yaml` 에 적고 검증한다. **ROS 없이 도는 검사**(`check_floors.py`)라 현장 가기 전에 노트북에서 돌릴 수 있다 |
| 층 판정 | `detect.py` — SSID 를 **제안**으로, 확정은 사람이. 자동 판정은 확신 있을 때만 |
| 상태머신 | `transfer.py` — `idle → to_elevator → at_elevator → … → fault/aborted`. **ROS 를 임포트하지 않고** 행동을 `Effects` 로 주입해서 하드웨어 없이 전이를 시험한다 (`test/test_transfer.py`) |
| 로봇 쪽 결선 | `guide_node.py` 의 `NavEffects` · `_mission_loop()` · `load_map()` · `_relocalize_at()` · `transfer_event()`. `--floors` 를 안 주면 **지금까지처럼 한 층만 돈다** (`guide_node.py` 머리말) |

설계 판단 두 개가 코드 주석에 남아 있고, 둘 다 이 홉이 왜 어려운지를 정확히
집는다.

- **엘리베이터는 주행 구간이 아니라 상태 전이 구간이다.** 버튼을 못 누른다 ·
  좁은 금속 박스라 AMCL 이 어느 층 지도에도 안 맞는다 · 로봇은 수직으로
  움직이는데 엔코더는 정지라고 한다 (`transfer.py` 머리말)
- **SSID 판정이 가장 필요한 순간에 안 되는 구성이 정상 구성이다.** 젯슨이
  관제 노트북 핫스팟에 붙어 있으면 건물 SSID 가 안 보이는데, 그 핫스팟은
  11층에서 VLM 을 쓰기 위한 전제 조건이다 (`detect.py` 머리말)

**아직 확인 안 된 것**

- `fleet/guide_mission/README.md` 는 여전히 `(TODO)` 세 줄이다 — 코드가
  README 를 앞질렀다
- UI 결선은 같은 저녁에 따라 들어왔다 — `POST /api/transfer` 와
  `index.html::renderTransfer()` · `sendTransfer()`. `index.html` 이
  225줄에서 465줄로 늘었다
- **실차 검증 0회.** 이 절을 쓰는 동안에도 코드가 계속 바뀌었다

### 6. Nav2 경로 추종

```
navigate_to_pose  →  bt_navigator      (nav2_params.yaml:53-70)
                  →  planner_server    SmacPlanner2D      :127-149
                  →  controller_server RegulatedPurePursuit :101-125
                  →  velocity_smoother                    :264-283
                  →  /cmd_vel  (geometry_msgs/TwistStamped)
```

측위는 AMCL(`:12-51`), 코스트맵은 `/scan` 하나만 본다(`:207-220`, `:244-257`).
뎁스도 ToF 도 코스트맵에 안 들어간다 — **라이다 평면(0.21925 m,
`jongky.urdf.xacro:148`)에 안 걸리는 장애물은 Nav2 가 모른다.**

정체가 길어지면 `_guide_loop()` 이 VLM 에 사진 한 장을 물어보고
`wait`/`ask_to_move`/`reroute`/`alert`/`resume` 중 하나를 고른다
(`guide_node.py::_handle_obstacle()` → `brain.py::judge_obstacle()`). **LLM 이 로봇을 직접
조종하지는 않는다** — 정해진 행동 집합 중 하나를 고를 뿐이고 주행은 그대로
Nav2 가 한다(`brain.py 머리말 [안전 규약]`).

### 7. ros2_control → UART → 모터

```
/cmd_vel (TwistStamped)
   │  control.launch.py:75-85 이 /diff_drive_controller/cmd_vel 을 /cmd_vel 로 리맵
   ▼
diff_drive_controller            jongky_controllers.yaml:45-128
   │  좌우 바퀴 각속도 (rad/s, 전진이 양수)
   ▼
JongkySystemHardware::write()    jongky_system.cpp:396-430
   │  역기구학으로 vx, vz 복원 (:412-415) + 부호 반전 + 스케일 (:417-422)
   ▼
YahboomBoard::set_motion()       yahboom_board.cpp:163-
   │  프레임 인코딩 → ::write(fd_)  :159
   ▼
UART 115200 8N1                  yahboom_board.cpp:32, :55-105
   ▼
야붐 Rosmaster 보드 → 모터
```

**CAN 이 아니라 UART 다.** 저장소 전체에 CAN·SocketCAN·CAN FD 문자열이
0건이고(2026-08-20 확인), 실제 경로는 `/dev/yahboom` 심링크가 가리키는
USB 시리얼이다(`docker/README.md:60-61`).

부호 규약이 이 홉의 핵심이다 — 보드의 `+vx` 는 물리적 **후진**이라
보낼 때 뒤집는다(`jongky_system.cpp:417-422`,
`jongky_hardware/README.md:31-50`). 이 반전을 URDF 나 컨트롤러 설정에
넣으면 목업·시뮬·실물이 서로 다른 인터페이스를 갖게 된다.

### 8. 결과 반환

판정과 표현이 나뉘어 있다. `::_drive_to()` 가 Nav2 결과를 코드로 환산하고,
`::_guide_loop()` 이 그걸 화면·음성으로 옮긴다.

```
_drive_to()                          _guide_loop()
  TaskResult.SUCCEEDED → ok         → status="arrived",  TTS "도착했습니다"
  TaskResult.CANCELED  → "canceled" → status="idle"
  그 외                → "failed"   → status="failed",  TTS "경로를 찾지 못했습니다"
  VLM alert            → "alert"      ┐ 그 안에서 이미 알렸으므로
  사람 놓침            → "follower_lost" ┘ 여기서는 아무것도 안 한다
                                     → _publish_status()  →  /guide/status
```

`/guide/status` payload 는 `GuideState.snapshot()` 을 JSON 으로 굳힌 것이다 —
`status` · `destination` · `message` · `distance` · `follower_m`, 그리고
오늘 늘어난 `floor` · `floor_label` · `transfer`.
UI 는 이 토픽이 아니라 `GET /api/status` 를 폴링한다(`index.html::poll()`, `guide_node.py::do_GET()`).

---

## 이 경로에서 아직 안 채워진 것

| | 상태 |
|---|---|
| 4번 홉의 VDA5050 | **비었음.** 코드 0건 — `docs/adr/001-vda5050.md` |
| 5번 홉 층 전환 | **작업 중.** 코드가 오늘 들어왔다(미커밋). 실차 검증 0회, `fleet/guide_mission/README.md` 는 아직 `(TODO)` |
| 임무 계약(액션·POI 스키마) | **정의는 들어왔다.** `interfaces/guide_interfaces` 에 `Guide.action` · `GuideStatus.msg` · `Poi.msg` · `ListPois.srv` · `SetStart.srv` 가 오늘 들어왔다(미커밋). **아직 아무도 안 쓴다** — 지금 실제로 오가는 것은 `docs/interfaces.md` |
| 전 경로 1회 완주 | **확인 안 됨.** `jongky_guide` 빌드·실행 이력 0건 |
| 관제 계층(Zenoh) | **비었음** — `docs/adr/002-zenoh.md` |

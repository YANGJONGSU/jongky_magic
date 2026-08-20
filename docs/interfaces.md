# Interfaces

**지금 실제로 계층을 가로지르는 것**을 적는다. 런타임에 오가는 것은 아직
전부 기성 ROS 타입 아니면 HTTP JSON 이다.

> **2026-08-20 저녁 — `interfaces/guide_interfaces` 에 타입 정의가 들어왔다.**
> `action/Guide.action` · `msg/GuideStatus.msg` · `msg/Poi.msg` ·
> `srv/ListPois.srv` · `srv/SetStart.srv` · `schema/waypoints.example.yaml`.
> 설계 근거는 `interfaces/guide_interfaces/README.md` 에 상세히 적혀 있으니
> 여기서 되풀이하지 않는다.
>
> **다만 아직 아무도 안 쓴다.** `guide_interfaces` 를 임포트하는 코드가
> `robot/`·`fleet/` 전체에 0건이고(주석 한 줄 제외), 커밋도 안 됐다.
> 그래서 이 문서는 **여전히 "지금 오가는 것"** 을 적는다 — 아래를 계약이
> 바뀐 뒤의 모습으로 읽지 말 것. `interfaces/` 변경은 3인 승인 필수
> (`collaboration.md:26`).

---

## 1. ROS 토픽 — 안내 계층

| 토픽 | 타입 | 방향 | 정의 위치 |
|---|---|---|---|
| `/guide/destination` | `std_msgs/String` | `voice_node` → `guide_node` | 발행 `voice_node.py::VoiceNode.__init__` · `::_recognize_loop()` · 구독 `guide_node.py` 의 `create_subscription` · `::_on_destination_topic()` |
| `/guide/status` | `std_msgs/String` (JSON 문자열) | `guide_node` → (구독자 없음) | 발행 `guide_node.py` 의 `_status_pub` · `::_publish_status()` |

**`/guide/destination` 의 payload 는 waypoint 이름 문자열 그대로다.** 스키마도
검증 계층도 없다. 등록 여부만 `start_guiding()` 이 확인한다
(`guide_node.py::start_guiding()` 의 미등록 거절). 아무 노드나 발행하면 로봇이 출발한다.

**`/guide/status` 의 payload** — `GuideState.snapshot()` 을 JSON 으로 굳힌 것
(`guide_node.py::GuideState.snapshot()`).

```json
{"status": "navigating", "destination": "304호 강의장",
 "message": "304호 강의장 으로 안내합니다", "distance": 8.42, "follower_m": 1.6,
 "floor": "10f", "floor_label": "10층", "transfer": null}
```

| 필드 | 값 |
|---|---|
| `status` | `idle` \| `navigating` \| `waiting` \| `arrived` \| `failed` \| `alert` \| `listening` \| `transfer` \| `fault` |
| `destination` | waypoint 이름 |
| `message` | 사람에게 보여 줄 한 줄 |
| `distance` | Nav2 피드백의 `distance_remaining` (m, 소수 2자리) |
| `follower_m` | 뒤따르는 사람까지 거리. **`-1` = 모름** |
| `floor` · `floor_label` | 지금 층. 빈 문자열이면 모름 — **2026-08-20 추가** |
| `transfer` | 층 전환 중일 때만 채워진다. UI 가 이걸 보고 "사람이 할 일" 버튼을 그린다 — **2026-08-20 추가** |

> `listening` · `transfer` · `fault` 는 한때 `GuideState` 주석의 목록에서
> 빠져 있었다. 문자열 상태를 코드가 아니라 주석으로만 정의한 결과이고,
> 오늘 들어온 `GuideStatus.msg` 가 이걸 `uint8` 상수로 못 박는다 — 7절.

지금 이 토픽을 구독하는 노드가 없다. UI 는 `GET /api/status` 를 폴링한다.

## 2. ROS 토픽 — 구동·센서 (실측)

`jongky_bringup/README.md 의 "올라오는 것 (실측)" 절` 의 실측표. 여기가 L2~L4 경계다.

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/cmd_vel` | **`geometry_msgs/TwistStamped`** | 구독 |
| `/odom` | `nav_msgs/Odometry` | 50.02 Hz |
| `/joint_states` | `sensor_msgs/JointState` | 50.07 Hz |
| `/imu/data` | `sensor_msgs/Imu` | 50.03 Hz |
| `/scan` | `sensor_msgs/LaserScan` | 9.99 Hz, 721 포인트, 360도 |
| `/camera/rgb/image_raw`(+`/compressed`) | `sensor_msgs/Image`, `CompressedImage` | openni2 lazy 발행 |

```
TF   map -> odom -> base_footprint -> base_link -> {laser, imu_link, wheels, ...}
      AMCL   EKF        diff_drive_controller 는 이 TF 를 내지 않는다
```

> **`/cmd_vel` 이 `Twist` 가 아니라 `TwistStamped` 다.** Jazzy 의
> `diff_drive_controller` 기본값이고, 받는 쪽이 안 맞추면 **경로도 계획되고
> 로그도 정상인데 바퀴만 안 돈다.** Nav2 쪽은 `enable_stamped_cmd_vel: true`
> 세 곳(`nav2_params.yaml:81`, `:170`, `:273`), 텔레옵은
> `teleop_key.py:5-7`. 표준 `teleop_twist_keyboard` 는 못 쓴다.

토픽 이름은 스포너의 `--controller-ros-args` 로 리맵된다
(`control.launch.py:75-93`). `ros2_control_node` 에 `remappings` 를 걸어도
**컨트롤러 토픽에는 안 먹는다** (`jongky_bringup/README.md 의 "토픽 이름 리맵은 스포너에서 한다" 절`).

## 3. ROS 액션 · 서비스

| | | 쓰는 곳 |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `BasicNavigator.goToPose()` (`guide_node.py::_drive_to()` · `::_handle_obstacle()` · `::_wait_for_follower()`) |
| Nav2 lifecycle | `waitUntilNav2Active` | `guide_node.py::wait_for_nav2()` |
| 코스트맵 초기화 | `clearAllCostmaps()` | `guide_node.py::_handle_obstacle()` 의 reroute 분기 |
| 초기 위치 | `setInitialPose()` → `/initialpose` | `guide_node.py::set_start()` |
| `map_server/load_map` | `nav2_msgs/srv/LoadMap` | **오늘 들어왔다** — `guide_node.py::load_map()` (층 전환용, 미커밋·미검증) |

**런타임에 쓰이는 자체 정의 타입은 아직 없다.** 정의는 오늘
`interfaces/guide_interfaces` 에 들어왔지만 임포트하는 코드가 0건이다 — 7절.
`collaboration.md:52` 의 G0 산출물 "stub 노드가 NavigateToPose 를 받아
RobotStatus 를 발행" 은 여전히 코드에 없다.

## 4. HTTP — 터치스크린 UI (`guide_node.py::make_handler()`, 기본 포트 8080)

| 메서드 | 경로 | 요청 | 응답 | 코드 |
|---|---|---|---|---|
| GET | `/` `/index.html` | — | `web/index.html` | `do_GET()` |
| GET | `/api/destinations` | — | `catalog()` — `destinations` + `floor` · `floor_label` · `floors[]` · `multi_floor` | `do_GET()` |
| GET | `/api/status` | — | `snapshot()` + `can_listen`(bool) + `localized`(bool) | `do_GET()` |
| POST | `/api/go` | `{"destination": "...", "floor": ""}` | `{"ok": bool, "error": str}` · 실패 시 **400** | `do_POST()` → `start_guiding()` |
| POST | `/api/cancel` | `{}` | `{"ok": true}` | `do_POST()` → `cancel()` |
| POST | `/api/start-here` | `{"waypoint": "...", "floor": ""}` | `{"ok": bool, "error": str}` · 실패 시 **400** | `do_POST()` → `set_start()` |
| POST | `/api/transfer` | `{"event": "...", ...}` | `{"ok": bool, "error": str}` · 실패 시 **400** | `do_POST()` → `transfer_event()` |
| POST | `/api/listen` | `{}` | `{"ok": bool, "error": str}` · 실패 시 **400** | `do_POST()` → `listen_and_go()` |

> **`/api/transfer` 와 `floor` 인자는 2026-08-20 저녁에 들어왔다** (층 전환,
> 미커밋). 사람이 "탔습니다" 같은 버튼을 누르면 그 이벤트가 상태머신으로
> 넘어간다 — `trace.md` 5번 홉.

- 인증이 없다. `0.0.0.0` 바인드다(`guide_node.py::main()` 의 `ThreadingHTTPServer`) — 같은 서브넷 누구나 로봇을 출발시킬 수 있다
- 오류는 **한국어 문자열**로 그대로 내려가고 UI 가 그대로 띄운다
  (`index.html::go() 의 실패 분기`). 오류 코드 체계는 없다
- `/api/start-here` 가 AMCL 초기 위치를 잡는 유일한 현장 수단이다. 이걸 안 하면
  `map->odom` 이 없어 **어떤 목적지도 안 간다**(`guide_node.py::set_start() 앞 주석`)

## 5. HTTP — 로봇 밖으로 나가는 두 갈래

DDS 도 Zenoh 도 아니다. 둘 다 평문 HTTP 다.

### 5-1. 후면 사람 탐지 (`follow_service.py`, 젯슨 **호스트**, 포트 8641)

`GET /follower` → JSON. 컨테이너 밖에서 도는 이유는
컨테이너 cv2 의 numpy 2.0 ABI 충돌이다(`follow_service.py 머리말 [왜 컨테이너가 아니라 호스트인가]`).

```json
{"present": true, "score": 0.83, "bearing_deg": -4.1, "distance_m": 1.62,
 "bbox": [...], "stamp": 1755...,  "age_s": 0.21,
 "calib": {"hfov_deg": 62.2, "person_height_m": 1.7, "focal_px": ..., "measured": false}}
```

클라이언트 계약(`follow_client.py::FollowerState · Follower.poll()`)이 중요하다.

| 조건 | 결과 |
|---|---|
| 서비스에 못 닿음 · `error` 필드 있음 | `present=None` (**"모름"**) |
| `age_s < 0` 또는 `> 3.0` (낡은 프레임) | `present=None` |
| 정상 | `present`/`distance_m`/`bearing_deg`/`score`/`age_s` |

**"모름" 은 "없음" 이 아니다.** `_guide_loop()` 은 모르면 사람이 있다고 치고
계속 간다(`guide_node.py::_drive_to()` · `::_wait_for_follower()`) — 탐지기 장애로 복도 한복판에
멈춰 서는 편이 더 나쁘다고 판단했다.

`calib.measured` 가 `false` 면 거리 추정이 **렌즈 공칭 HFOV 62.2도** 위에서
돌고 있다는 뜻이다(`follow_service.py 의 HFOV_DEG 상수`). 계통 오차가 보이면 여기부터
의심한다.

### 5-2. LLM/VLM (`brain.py`, 관제 노트북 `192.168.129.97:11434`)

`POST /api/generate` (ollama). 두 용도 모두 **응답 형식을 계약으로 강제**한다.

| 용도 | 요청 | 응답 계약 | 실패 시 |
|---|---|---|---|
| 목적지 해석 | 프롬프트 + waypoint 목록 (`brain.py::resolve_destination() 의 프롬프트`) | 목록에 있는 이름 한 줄, 없으면 `NONE` | 목록과 대조해 못 맞추면 `None` (`:119-125`) — **지어낸 목적지가 못 빠져나간다** |
| 돌발상황 판단 | 프롬프트 + base64 JPEG (`brain.py::judge_obstacle() 의 프롬프트`) | `{"action","say","reason"}` 한 줄. `action` 은 `wait`/`ask_to_move`/`reroute`/`alert`/`resume` 다섯 중 하나 (`:50-56`) | 파싱 실패·모르는 action·응답 없음 → **전부 `wait`** (`:161-176`) |

고정 옵션: `think: false`, `temperature: 0.1`, `num_predict: 200`
(`brain.py::_ask() 의 payload`). `think` 를 켜면 내부 추론이 `num_predict` 를 다 써서
**응답이 빈 문자열로 돌아온다.**

**오디오 입력은 계약에 없다.** ollama 가 필드를 통째로 무시해서 근거 없는
답이 나온다 — 실측으로 확인하고 뺐다(`brain.py 의 오디오 입력 미지원 주석`).

## 6. POI 스키마 — waypoint YAML

**저장소에 없다.** 젯슨의 `~/waypoints_10f.yaml` 에 있고, 맵핑 주행 중
`teleop_key.py` 의 `w` 로 찍는다.

```yaml
304호 강의장:
  position: {x: 12.5, y: 0.3, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  frame_id: map          # 생략하면 "map" (guide_node.py::_to_pose)
```

| | |
|---|---|
| 읽는 곳 | `guide_node.py::_load_waypoints()` · `::_to_pose()`, `voice_node.py:142-143` |
| 키 = 표시 이름 = 음성 매칭 대상 = 목적지 ID | 넷을 겸한다. 별도 등록처가 없다 |
| UI 버튼 목록 | 이 파일이 그대로 정한다 (`index.html::loadDestinations() · renderGrid()`) |
| 초기 위치 후보 | 같은 목록을 쓴다 (`index.html::buildStart()`) |

**알려진 문제** — 키 하나가 네 역할을 겸해서 생기는 것들이다.

- 이름이 곧 음성 매칭 문자열이라 길게 지으면 안 걸린다
  (`jongky_guide/README.md:87-89`)
- `10a` · `ev1` · `m1` 같은 내부 코드가 그대로 버튼에 뜬다
  (`jongky_guide/README.md:114-115`)
- 저장키가 `w` 라 눌린 채 들어간 무효 항목이 있다 (`잔여-공정.html` N4, 작업 노트).
  **엘리베이터를 이름 규칙으로 못 찾는 이유**이기도 하다 — 실제 파일에
  `wwwwwwwwwwwwwwwwwev2` 가 있다 (`guide_interfaces/README.md`)
- **층 정보가 없다.** 10층 waypoint 와 11층 waypoint 가 형식상 구분되지 않는다.
  층 전환 스키마의 선행 조건이다

> 넷 다 오늘 들어온 `Poi.msg`(`display_name`/`kind`/`aliases`/`floor`)와
> `waypoint_doctor.py` 가 겨냥하는 것이다. **아직 런타임에 안 붙었다** — 7절.

## 7. 계약 현황 (2026-08-20 확인)

| | 상태 |
|---|---|
| `guide_interfaces` 의 msg/action/srv/POI 스키마 | **정의는 들어왔다. 쓰는 코드는 0건.** 미커밋 |
| `/api/listen` 대응 타입 | 새로 안 만들었다 — `std_srvs/Trigger` 로 충분하다는 판단 (`guide_interfaces/README.md`) |
| waypoint YAML 이관 도구 | `robot/jongky_bringup/tools/waypoint_doctor.py` 가 오늘 들어왔다. **런타임은 이름에서 종류를 추론하지 않고**, 이 도구가 한 번만 짐작해 파일에 적는다 |
| VDA5050 order/state JSON | **없음** — `adr/001`. `guide_interfaces` 도 이쪽 타입은 **일부러 안 만들었다** |
| Zenoh 브리지 설정 | **없음** — `adr/002` |
| 층 전환 계약 | 별도 타입을 안 만들고 `Poi.floor` · `Poi.kind` 로 처리하기로 했다. 상태머신 자체는 `fleet/guide_mission/` 에 오늘 들어왔다 (`trace.md` 5번 홉) |
| 배터리 (`sensor_msgs/BatteryState`) | **없음** — 보드는 전압을 보고하는데 경고 로그만 남는다 (`jongky_system.cpp:219-226`) |
| ToF ×2 | URDF 프레임만 있고 노드가 없다 |

**마이그레이션이 남았다.** 정의가 있다는 것과 계약이 바뀌었다는 것은 다르다.
아래 넷이 실제로 갈아끼워져야 이 문서의 1·4절이 바뀐다.

1. `/guide/destination` (`std_msgs/String`) → `Guide.action` goal
2. `/guide/status` (JSON 문자열) → `GuideStatus.msg`
3. `GET /api/destinations` → `ListPois.srv` · `POST /api/start-here` → `SetStart.srv`
4. waypoint YAML → `Poi` 필드(`display_name`/`kind`/`aliases`/`floor`) 채우기

새 필드는 전부 선택이라 **오늘 현장에 있는 YAML 이 한 글자도 안 고치고 그대로
돈다** (`guide_interfaces/README.md` 의 하위 호환 표). 그래서 4번은 로봇을
세우지 않고 진행할 수 있다.

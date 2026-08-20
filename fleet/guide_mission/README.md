# guide_mission — 층 전환 상태머신

안내로봇이 **엘리베이터를 건너 다른 층으로 넘어가는 구간**을 맡는다.
지도 교체·AMCL 재초기화·사람에게 시킬 일의 순서와, 각 단계가 실패했을 때
무엇을 하는지가 여기 다 들어 있다.

```
fleet/guide_mission/
├── guide_mission/
│   ├── floors.py        층별 지도·waypoint 대장과 짝 검증
│   ├── detect.py        층 판정 (SSID / 사람)
│   ├── transfer.py      엘리베이터 상태머신  ← 이 패키지의 본체
│   ├── effects.py       상태머신이 바깥에 하는 일의 경계면
│   ├── korean.py        조사 (로/으로). 화면·음성에 그대로 나간다
│   └── check_floors.py  현장 가기 전 대장 검사 CLI
├── config/floors.example.yaml
└── test/                로봇·Isaac·Nav2 없이 도는 단위 시험 76개
```

로봇 쪽 짝은 `robot/jongky_guide/tools/guide_node.py` 의 `NavEffects` 다.

---

## 1. 왜 상태머신인가

계획서(`전체-작업계획.md` 2절)가 이유를 이미 적어 뒀다.

> 버튼을 못 누른다. 좁은 금속 박스라 AMCL 이 어느 층 지도에도 안 맞는다.
> 로봇은 수직으로 움직이는데 엔코더는 정지라고 한다. 문이 열리면 다른 층이다.

그래서 엘리베이터는 **목표를 주고 도착을 기다리는 주행 구간이 아니라, 사람이
하는 일을 기다리는 상태 전이 구간**이다. 이 구간에서 로봇이 하는 일은 셋뿐이다.

1. 제자리에 서 있기 (Nav2 에 목표가 없다)
2. 다음에 무엇을 해 달라고 화면·음성으로 말하기
3. 문이 열린 뒤 **자기가 어디인지 다시 알아내기**

3번이 이 기능의 전부라고 해도 된다. 지도를 갈고, 초기 위치를 다시 주고,
정말 잡혔는지 확인한다. 하나라도 빠지면 로봇은 **다른 층 지도로 길을 찾는다.**

---

## 2. 상태 전이표

| 상태 | 로봇이 하는 일 | 사건 | 다음 |
|---|---|---|---|
| `idle` | — | `start` | `to_elevator` |
| `to_elevator` | 이 층 지도로 엘리베이터 앞까지 **자율주행** | `nav_ok` | `at_elevator` |
| | | `nav_fail` | **`fault`** |
| | | `abort` | `aborted` |
| `at_elevator` | 정지. "○층 버튼을 눌러 주세요" | `called` | `boarding` |
| | | `timeout` / `abort` | **`fault`** / `aborted` |
| `boarding` | 정지. **초기 위치를 버린다.** "로봇을 넣어 주세요" | `boarded` | `riding` |
| | | `timeout` / `abort` | **`fault`** / `aborted` |
| `riding` | 정지. Nav2 목표 없음. "○층에서 내려 주세요" | `arrived` | `exiting` |
| | | `timeout` / `abort` | **`fault`** / `aborted` |
| `exiting` | 정지. "로봇을 내려 주세요" | `exited` | `confirm_floor` |
| | | `timeout` / `abort` | **`fault`** / `aborted` |
| `confirm_floor` | 층 판정 (SSID → 안 되면 사람에게) | `floor_ok` | `swap_map` |
| | | `timeout` / `abort` | **`fault`** / `aborted` |
| `swap_map` | waypoint 재확인 → `map_server/load_map` | `map_ok` | `relocalize` |
| | | `map_fail` | **`fault`** |
| `relocalize` | 초기 위치 발행 + **잡혔는지 확인** + 코스트맵 초기화 | `pose_ok` | `landed` |
| | | `pose_fail` | **`fault`** |
| `landed` | 목표 층과 대조 | `same_floor` | `resume` |
| | | `wrong_floor` | `at_elevator` (다시 태워 달라) |
| | | `give_up` | **`fault`** (재탑승 한도 초과) |
| `resume` | 새 층에서 원래 목적지까지 안내 | `nav_ok` | `done` |
| | | `nav_fail` / `abort` | **`fault`** / `aborted` |

전이표는 코드 안에 `TRANSITIONS` 사전으로 **데이터로** 있고, 표에 없는
(상태, 사건) 조합은 예외가 되어 `fault` 로 떨어진다. 시험이 표 자체를 검사한다
(모든 상태 도달 가능, 모든 비종단 상태에 실패 출구 존재, 처리기 누락 없음).

### 핵심 불변식

> **`localized` 는 `boarding` 에 들어가는 순간 False 가 되고, `relocalize` 가
> 성공했을 때만 True 로 돌아온다.**

`guide_node` 는 `localized` 가 False 면 **어떤 목적지 요청도 거부한다.**
그래서 지도 교체가 실패했거나 AMCL 이 안 잡혔으면 로봇은 아예 안 움직인다.
`fault` 와 "탄 뒤의 취소" 는 False 인 채로 끝난다 — 사람이 화면에서 층과
위치를 다시 잡아 줘야 한다.

### 엉뚱한 층에 내렸을 때

사람이 버튼을 잘못 눌렀거나 엘리베이터가 다른 층에 섰다. SSID 는 그것을 안다.
이때 **지도는 실제로 서 있는 층으로 갈아 둔다** — 거짓 위치를 들고 있는 것보다
낫다. 그 다음 `at_elevator` 로 돌아가 다시 태워 달라고 한다. `max_rides`(기본
2회)를 넘으면 `fault`.

---

## 3. 층 판정 — SSID 를 제안으로, 확정은 사람

| 후보 | 되는가 | 판단 |
|---|---|---|
| **SSID** (`FASTCAMPUS_10F`/`11F`) | 대개 | **쓴다.** 공짜고 즉시 나온다 |
| **사람이 UI 에서 고름** | 언제나 | **쓴다.** SSID 가 안 될 때의 바닥 |
| 기압계 | — | 센서가 없다 |
| 층 표지판 OCR | — | 모델·카메라 각도 문제. 나중 일 |

SSID 하나로 못 끝내는 이유가 있다.

> **젯슨이 관제 노트북 핫스팟에 붙어 있으면 건물 SSID 가 아예 안 보인다.**
> 그리고 그게 **정상 구성**이다 — VLM(`gemma4:e2b`, 7.2GB)이 노트북에 남는
> 것이 실측으로 확정됐고(`brain.py` 상단), 11층에서 돌발상황 판단을 쓰려면
> 노트북 AP 핫스팟이 선택이 아니라 전제 조건이다(`전체-작업계획.md` 6절).
> 즉 **가장 필요한 순간에 SSID 가 안 되는 구성이 기본값이다.**

그래서 이렇게 갈랐다.

- SSID 가 대장에 있는 층과 맞으면 → **그대로 진행하고 근거를 화면에 남긴다**
- 핫스팟 SSID(`hotspot_ssids`)에 붙어 있으면 → **"핫스팟에 붙어 있어 층을 알 수
  없습니다. 층을 골라 주세요"**. 고장이 아니라 정상 구성이므로 그렇게 말한다
- 무선이 없거나(`iwgetid`·`nmcli` 둘 다 없는 컨테이너 포함) 모르는 SSID → 묻는다
- `--floor-detect always` → SSID 로 짚어 놓되 매번 사람 확인을 받는다
- `--floor-detect off` → 아예 안 본다

**어떤 경우에도 모르는 채로 한 층을 찍지 않는다.** 찍으면 그 다음이 다른 층
지도로 주행이다.

---

## 4. 실패했을 때 무엇을 하는가

모든 실패는 `fault` 한 곳으로 모인다. `fault` 는 세 가지를 반드시 한다.

1. **`hold()`** — Nav2 목표를 취소하고 제자리에 선다
2. **`set_localized(False)`** — 그 뒤의 모든 목적지 요청이 거부된다
3. **화면 + 음성** — 무엇이 잘못됐고 사람이 무엇을 하면 되는지 말한다

| 어디서 | 무엇이 실패하면 | 어떻게 되는가 |
|---|---|---|
| `to_elevator` | 엘리베이터 앞까지 경로를 못 찾음 | 층 전환 자체를 접는다. 지도는 그대로 |
| `at_elevator`·`boarding`·`riding`·`exiting` | 사람이 안 누름 (기본 5~10분) | 정지 + 화면 안내. 로봇은 이미 서 있으므로 위험 없음 |
| `confirm_floor` | 층을 모르고 사람도 안 고름 | 정지. **지도를 안 건드린다** |
| `swap_map` | 새 층 waypoint 를 못 읽음 | **`load_map` 을 부르기 전에** 멈춘다. 지도만 새 층이고 waypoint 는 옛 층인 상태를 안 만든다 |
| `swap_map` | `load_map` 결과 코드가 실패 | 정지. `RESULT_MAP_DOES_NOT_EXIST` 같은 코드를 사람이 읽을 문장으로 바꿔 보여 준다 |
| `swap_map` | 올라간 지도 크기가 대장과 다름 | 정지 (엉뚱한 파일이 올라갔다) |
| `swap_map` | `amcl.first_map_only` 가 true | 정지. 이건 지도를 갈아도 AMCL 만 옛 지도를 보는 설정이라 **성공한 것처럼 보이는 실패**다 |
| `relocalize` | 20초 안에 `amcl_pose` 가 안 옴 | 정지 |
| `relocalize` | AMCL 이 준 자리에서 2 m 넘게 떨어진 데 수렴 | 정지 (지도와 waypoint 의 층이 어긋난 신호) |
| `relocalize` | `map->odom` TF 가 안 나옴 | 정지 |
| `resume` | 새 층에 그 목적지가 없음 | 정지 |
| 어디서든 | 코드 예외 · 전이표에 없는 조합 | 정지 (예외를 밖으로 흘리지 않는다) |

### `changeMap()` 을 안 쓴 이유

`BasicNavigator.changeMap()` 은 이미 있고 `guide_node` 가 한 번도 안 불렀다.
그런데 그대로 쓰면 안 된다.

```python
# nav2_simple_commander/robot_navigator.py:647
def changeMap(self, map_filepath):
    while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):  # ← 무한 대기
        ...
    if status != LoadMap.Response().RESULT_SUCCESS:
        self.error('Change map request failed!')                        # ← 로그만
    return                                                              # ← 반환값 없음
```

**실패를 부른 쪽이 알 방법이 없고, 서비스가 안 뜨면 영원히 기다린다.**
그래서 클라이언트(`change_maps_srv`)만 빌려 쓰고 호출은 `guide_node.load_map()`
에서 직접 한다 — 결과 코드를 읽고, 시간을 걸고, 올라간 지도 크기까지 본다.

---

## 5. 층별 자원 — `floors.yaml`

지도와 waypoint 의 짝을 사람이 명령줄에 손으로 치는 것이 지금까지의 방식이었고
(`map:=fastcampus_10f.yaml waypoints:=~/waypoints_11f.yaml`), **틀려도 아무도
알려주지 않았다.** 그 짝을 파일 하나로 옮겼다.

```yaml
hotspot_ssids: [jongky]          # 여기 붙어 있으면 건물 SSID 가 안 보인다

floors:
  "10f":
    label: "10층"
    map: ~/jongky_ws/src/jongky_navigation/maps/fastcampus_10f.yaml
    waypoints: ~/waypoints_10f.yaml
    ssid: [FASTCAMPUS_10F]
    elevator: {board: ev1, exit: ev1}     # 타는 자리 / 내리는 자리
    labels: {ev1: "엘리베이터 앞", 10a: "1004호 강의장"}
```

`labels` 는 화면·음성에 쓸 이름이다. 실제 파일의 waypoint 이름은 `10a`·`ev1`
이고 맵핑 중 터미널 버퍼가 새어 `wwwwwwwwwwwwwwwwwev2`·`.,,` 같은 것도 섞여
있다(`teleop_key.py:228`). 사람이 그걸 보고 누를 수는 없다.

### 짝 검증

```bash
ros2 run guide_mission check_floors ~/floors.yaml     # 종료 코드 0 이면 띄워도 된다
python3 -m guide_mission.check_floors ~/floors.yaml   # ROS 없이도 돈다
```

지도 YAML 의 `origin`·`resolution` 과 이미지(pgm/png) 헤더의 픽셀 크기로 지도가
덮는 사각형을 구하고, waypoint 가 그 안에 있는지 본다. **다른 층 waypoint 를
얹으면 좌표가 통째로 지도 밖으로 나가므로 바로 걸린다.** 그 밖에 엘리베이터
지점 누락, 파일 없음, 오염된 이름을 잡는다.

- 전부 지도 밖 / 엘리베이터 지점이 밖·없음 → **오류**. 그 층으로는 못 간다
- 일부만 밖 → 경고. 그 지점만 못 쓴다
- 이름이 오염됨 → 경고. `labels` 로 표시 이름을 줄 것

한계: **같은 건물의 두 층은 크기가 비슷해서 안 걸릴 수 있다.** 그래서 층
판정과 `relocalize` 의 수렴 거리 검사가 따로 있는 것이다.

---

## 6. 시험 — 무엇을 덮고 무엇은 못 덮는가

```bash
python3 -m unittest discover -s fleet/guide_mission/test        # 76개, 0.1초
colcon test --packages-select guide_mission                     # 워크스페이스에서
```

**로봇도 Isaac 도 Nav2 도 필요 없다.** `transfer.py` 가 ROS 를 임포트하지 않고
모든 행동을 `Effects` 로 주입받기 때문이다.

덮는 것:

- 정상 경로 전체 (전이 순서, 지도 교체 → 초기 위치 순서, 층 갱신, 안내 재개)
- 각 단계의 실패가 **주행으로 이어지지 않는가** (지도 실패, AMCL 실패,
  waypoint 실패, 경로 실패, 사람 미응답)
- `localized` 불변식 — 탈 때 버리고 재초기화에서만 돌아온다
- 취소: 타기 전(위치 유지) / 탄 뒤(위치 상실 + 수동 복구 요구)
- 핫스팟·무선 없음·모르는 SSID 에서 **찍지 않고 묻는가**
- 엉뚱한 층: 실제 층으로 지도 교체 → 재탑승 → 한도 초과 시 정지
- 전이표 무결성 (도달 가능성, 실패 출구, 처리기 누락)
- 지도/waypoint 짝 검증, pgm·png 헤더 파싱, 조사(로/으로)
- **어댑터 계약** — `guide_node.py` 를 AST 로 읽어 `NavEffects` 가 `Effects`
  전부를 구현하는지, `mission.XXX` 로 부르는 이름이 실제로 있는지 본다
  (임포트가 아니라 파싱이라 rclpy 없이 돈다)

**현장에서만 확인되는 것** — 시험이 못 덮는다:

- 엘리베이터 안에서 라이다·AMCL 이 실제로 어떻게 망가지는가. 문이 닫힌 채
  파티클이 어디로 튀는지는 재현할 수 없다
- 새 층에서 `relocalize` 가 **정말로 수렴하는가**. 엘리베이터 앞은 대칭적인
  복도인 경우가 많아 AMCL 이 180도 돌아 앉을 수 있다. 2 m 검사는 위치만 보고
  방향은 안 본다
- `load_map` 뒤 global costmap 이 새 지도 크기로 제대로 다시 잡히는가
- SSID 가 정말 층마다 다른가, 엘리베이터 앞에서 옆 층 AP 가 더 세게 잡히지는
  않는가 (**층 판정이 틀리는 가장 현실적인 경로다**)
- 사람이 로봇을 태우고 내리는 데 실제로 몇 초 걸리는가 → 시간 초과값
- 터치스크린에서 버튼이 실제로 눌리는가 (장갑, 반사)

---

## 7. 패키지 형태 — ROS 패키지로 만들었다

`ament_python` 패키지 `guide_mission` 이다. `jongky_guide` 안의 모듈로 두지
않은 이유:

1. **`jongky_guide` 는 `ament_cmake` 이고 모듈을 `lib/jongky_guide` 에 납작한
   파일로 깐다** (`CMakeLists.txt` 의 `install(FILES ...)`). 이름공간도 시험
   디렉터리도 없다. `from brain import Brain` 처럼 전역 이름을 하나씩 늘리는
   구조라 여섯 개짜리 모듈 묶음을 넣을 자리가 아니다
2. **이번 작업 범위가 `jongky_guide/CMakeLists.txt`·`package.xml` 을 건드리지
   않는다.** 거기에 새 파일을 두면 설치 목록에 안 들어가 **빌드해도 안 깔린다**
3. 상태머신은 **ROS 없이 시험이 돌아야** 한다. 별도 패키지로 두니 `rclpy` 의존이
   구조적으로 불가능해진다 (`package.xml` 에 아예 없다)
4. `fleet/` 이 원래 그 층이다. 옆의 `guide_rmf`·`guide_vda5050` 도 결국 같은
   층 대장을 읽어야 한다

### 그래서 빌드가 하나 늘었다

`jmap:80` 의 colcon 목록에 `guide_mission` 이 **없다**(그 파일은 이번 범위 밖
이다). 처음 쓸 때 한 번 따로 빌드할 것.

```bash
colcon build --packages-select guide_mission jongky_guide --symlink-install
source install/setup.bash
```

`jongky_guide/package.xml` 에도 `<exec_depend>guide_mission</exec_depend>` 가
아직 없다 — 같은 이유(범위 밖)다. **런타임 임포트에는 지장이 없다**
(`install/setup.bash` 가 `PYTHONPATH` 를 잡는다). colcon 빌드 순서에만 쓰이는
값이고 순수 파이썬 런타임 의존이라 순서가 상관없다. 다음에 그 파일을 열 때
넣으면 된다.

### 겹치는 작업이 하나 있다 (같은 날 병행)

`interfaces/guide_interfaces/msg/Poi.msg` 와 `robot/jongky_bringup/tools/
waypoint_doctor.py` 가 같은 문제의 다른 절반을 건드리고 있다 — waypoint 에
`display_name`·`kind`(`KIND_ELEVATOR`)·`floor` 를 붙이는 일이다.

지금 이 패키지는 그것을 `floors.yaml` 에서 받는다.

| 여기 (`floors.yaml`) | 저기 (`Poi.msg`) |
|---|---|
| `labels: {ev1: "엘리베이터 앞"}` | `display_name` |
| `elevator: {board: ev1, exit: ev1}` | `kind == KIND_ELEVATOR` |
| 층별 파일 분리 | `floor` 필드 |

**둘을 지금 합치지 않았다.** 저쪽은 아직 메시지 정의뿐이고 `guide_node` 가 한
줄도 안 쓴다. 합칠 때는 이 방향이 맞다 — POI 가 들어오면 `floors.yaml` 은
**지도↔waypoint 짝과 SSID 만** 남기고, 이름과 종류는 POI 에서 읽는다.
`FloorBook.reload_waypoints()` 와 `Floor.label_of()` 두 군데만 바꾸면 된다.

`guide_node.py` 는 `guide_mission` 이 없으면 **한 층짜리로 그대로 돈다.**
다만 `--floors` 를 줬는데 임포트가 안 되면 **기동을 거부한다** — 조용히 한 층
짜리로 떨어지면 다른 층 버튼이 그냥 안 보이고 아무도 이유를 모른다.

---

## 8. 현장에서 처음 돌릴 때

### 0) 준비 — 층마다 맵핑이 끝나 있어야 한다

```bash
jmap 10f     # 주행 → Ctrl+C (지도 저장까지 자동)
jdrive 10f   # 다른 터미널. w 로 강의장 앞·엘리베이터 앞을 찍는다
jmap 11f ; jdrive 11f
```

**엘리베이터 앞 지점을 반드시 찍을 것.** 이름은 `floors.yaml` 의
`elevator.board`/`exit` 에 그대로 적는다.

### 1) 대장을 쓰고 검사한다 (로봇 없이, 노트북에서)

```bash
cp fleet/guide_mission/config/floors.example.yaml ~/floors.yaml
$EDITOR ~/floors.yaml
ros2 run guide_mission check_floors ~/floors.yaml
```

```
✓ 10f    10층   waypoint 6개  엘리베이터 ev1/ev1  SSID FASTCAMPUS_10F  지도 42×31 m @ 0.05 m/px
      · 1004호 강의장  (10a)
      · 1005호 강의장  (10b)
✓ 11f    11층   waypoint 5개  엘리베이터 ev1/ev1  SSID FASTCAMPUS_11F  지도 41×30 m @ 0.05 m/px
      · 1101호 강의장  (11a)

✓ 짝이 맞는다
```

`✗` 가 하나라도 나오면 여기서 고친다. 로봇을 켜기 전이다.

### 2) 띄운다

```bash
colcon build --packages-select guide_mission jongky_guide --symlink-install
source install/setup.bash

ros2 launch jongky_guide guide.launch.py \
    floors:=~/floors.yaml floor:=10f start_waypoint:=ev1 \
    tts_voice:=~/voices/ko_KR-xxx.onnx audio_device:=plughw:2,0
```

`map`·`waypoints` 를 따로 안 준다 — `floor:=10f` 로 대장에서 뽑는다.

콘솔에 이렇게 나와야 한다.

```
층 2개: 10층(10f), 11층(11f) · 지금 층 10f · 자동 판정 auto
무선으로 본 층: 10f — 무선 'FASTCAMPUS_10F' 으로 10층 으로 판정했습니다
지도 교체: /home/.../fastcampus_10f.yaml (842×627 @ 0.050)
초기 위치를 'ev1' 로 잡았다
```

`AMCL 이 20초 안에 amcl_pose 를 안 냈다` 가 나오면 Nav2 가 아직 안 뜬 것이다.
UI 의 **'여기서 시작'** 에서 다시 고르면 된다.

### 3) 화면 (7인치 터치스크린 `http://localhost:8080`)

오른쪽 위에 **`10층`** 배지가 늘 떠 있다. 목적지는 층별로 묶여 나오고,
다른 층 버튼에는 **🛗 엘리베이터** 꼬리표가 붙는다.

11층 강의장을 누르면 화면 전체가 층 이동 화면으로 바뀐다.

```
엘리베이터로 이동 중
11층으로 가기 위해 엘리베이터 앞으로 이동합니다        [취소]
────────────────────────────────────────────────
엘리베이터 앞
엘리베이터 앞에 도착했습니다. 11층 버튼을 눌러 주세요
로봇은 버튼을 누르지 못합니다. 눌러 주신 뒤 아래를 눌러 주세요
                    [ 엘리베이터를 불렀습니다 ]  [취소]
────────────────────────────────────────────────
탑승 대기
문이 열리면 로봇을 안으로 넣어 주세요. 11층으로 갑니다
로봇은 스스로 타지 않습니다. 밀어서 넣어 주세요
                    [ 로봇을 태웠습니다 ]  [취소]
────────────────────────────────────────────────
이동 중
11층으로 가는 중입니다. 도착하면 눌러 주세요
이동 중에는 로봇이 스스로 움직이지 않습니다
                    [ 도착했습니다 ]  [취소]
────────────────────────────────────────────────
하차
11층 입니다. 로봇을 엘리베이터 밖으로 내려 주세요
내려놓는 자리는 '엘리베이터 앞' 입니다
                    [ 로봇을 내려놨습니다 ]  [취소]
```

같은 문장이 piper TTS 로 나간다.

핫스팟에 붙어 있으면 (11층 운용의 기본값) 하차 뒤에 이게 뜬다.

```
층 확인
지금 몇 층인가요? 관제 노트북 핫스팟('jongky')에 붙어 있어
건물 SSID 가 보이지 않습니다. 층을 직접 골라 주세요
가려던 층은 11층 입니다
                    [ 10층 ]  [ 11층 ]        [취소]
```

고르면 `지도 교체` → `위치 재설정` 이 몇 초 지나가고 원래 목적지로 안내가
이어진다. 배지가 **`11층`** 으로 바뀌고 목적지 목록도 11층 것으로 바뀐다.

무엇이든 실패하면 화면이 붉어진다.

```
멈춤
층 이동을 멈췄습니다: 11층 지도를 불러오지 못했다:
지도 파일이 없다 (RESULT_MAP_DOES_NOT_EXIST)
화면에서 지금 층과 로봇 위치를 골라 주면 다시 움직입니다
                    [ 층·위치 다시 잡기 ]
```

이 상태에서 로봇은 **아무 목적지도 받지 않는다.** '층·위치 다시 잡기' 를
누르면 층 → 지점 순서로 고르는 화면이 나오고, 고르면 그 층 지도를 올리고
초기 위치를 잡은 뒤 평소 화면으로 돌아온다.

### 4) 현장에서 먼저 확인할 것

| 무엇 | 어떻게 | 안 되면 |
|---|---|---|
| 층마다 SSID 가 다른가 | 각 층에서 `iwgetid -r` | `ssid:` 를 비우고 `--floor-detect off`. 매번 사람이 고른다 |
| 엘리베이터 앞에서 옆 층 AP 가 잡히지 않는가 | 문 앞에서 `iwgetid -r` 여러 번 | `--floor-detect always` 로 매번 확인받는다 |
| 새 층에서 AMCL 이 수렴하는가 | 하차 뒤 화면이 `위치 재설정` 을 몇 초 만에 지나가는가 | 20초 뒤 붉은 화면. 엘리베이터 앞 waypoint 를 특징이 있는 자리로 다시 찍는다 |
| 사람이 태우고 내리는 시간 | 실제로 재 본다 | `transfer.Config` 의 시간 초과를 늘린다 |
| 핫스팟을 켜고도 도는가 | 노트북 핫스팟 붙인 채 한 번 | 층 확인 화면이 뜨는 게 정상이다 |


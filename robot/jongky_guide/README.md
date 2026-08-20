# jongky_guide

안내로봇의 사람 쪽 절반. 목적지를 받아 Nav2 로 데려가고 음성으로 안내한다.

```
[터치스크린 웹 UI] ─┐
                    ├→ guide_node ─→ Nav2 goToPose ─→ 주행
[음성 (Whisper)] ───┘        │
                             └→ piper TTS ─→ 스피커
```

| | |
|---|---|
| `tools/guide_node.py` | 본체. 웹서버 + Nav2 디스패치 + 상태 관리 |
| `tools/speech.py` | 오프라인 TTS (piper). 없으면 로그만 남기고 넘어간다 |
| `tools/voice_node.py` | Whisper STT. 별도 프로세스라 죽어도 UI 는 산다 |
| `tools/listen.py` | Whisper 래퍼. 마이크 녹음 + 무음 구간 판정 |
| `tools/brain.py` | LLM/VLM. 목적지 해석 폴백과 돌발상황 판단. ollama 에 물어본다 — 기본 주소가 **관제 노트북** `192.168.129.97:11434` 라 층이 갈리면 못 닿고, 그때는 `wait` 로 떨어진다 |
| `tools/follow_service.py` | 후면 카메라(IMX219) 사람 탐지. **젯슨 호스트**에서 도는 HTTP 서비스다 — 컨테이너 밖이다. 모델은 torchvision `ssdlite320_mobilenet_v3_large`(COCO), 가중치는 **첫 실행 때 인터넷에서 받는다** |
| `tools/follow_client.py` | 위 서비스를 부르는 쪽. 서비스가 없으면 `present=None` 로 넘어간다 |
| `web/index.html` | 7인치 터치스크린용 UI. 층 배지·층별 목적지·층 이동 화면 |
| `launch/guide.launch.py` | Nav2 + 카메라 + UI + 음성 |

층 전환(엘리베이터)은 별도 패키지다 — **`fleet/guide_mission`**. 상태머신과
층별 자원 대장이 거기 있고, 이 패키지의 `guide_node.py` 안 `NavEffects` 가
그것을 실제 로봇 동작에 옮긴다.

## 목적지는 맵핑 주행에서 나온다

`teleop_key.py` 가 `w` 로 찍어 둔 YAML 을 그대로 읽는다. 좌표계가 같은 `map`
이라 변환 없이 Nav2 goal 이 된다. **UI 버튼 목록도 이 파일이 정한다** —
따로 등록하는 곳이 없다.

```yaml
304호 강의장:
  position: {x: 12.5, y: 0.3, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  frame_id: map
```

## 실행

```bash
ros2 launch jongky_guide guide.launch.py \
    map:=~/maps/fastcampus_10f.yaml \
    waypoints:=~/waypoints_10f.yaml
```

층이 둘 이상이면 지도와 waypoint 를 손으로 짝지어 주지 말고 대장을 준다.

```bash
colcon build --packages-select guide_mission jongky_guide --symlink-install
source install/setup.bash

ros2 launch jongky_guide guide.launch.py \
    floors:=~/floors.yaml floor:=10f start_waypoint:=ev1
```

터치스크린에서 `http://localhost:8080`. 관제 노트북에서도 젯슨 IP 로 같은
화면을 볼 수 있다.

음성 입력은 기본으로 꺼져 있다. Whisper 가 젯슨 CPU 를 상당히 먹으므로
주행이 먼저 안정된 뒤에 켤 것.

```bash
    use_voice:=true voice_model:=tiny mic:=plughw:1,0
```

노드만 따로 띄울 수도 있다.

```bash
ros2 run jongky_guide guide_node.py --waypoints ~/waypoints_10f.yaml --port 8080
```

## 음성은 별도 설치가 필요하다

둘 다 apt 에 없다. **없어도 버튼 UI 와 주행은 그대로 된다** — 음성은 있으면
좋은 것이지 안전에 필요한 게 아니라서, 없으면 조용히 건너뛰도록 만들었다.

**TTS (piper)** — 모델은 이미지에 굽지 않고 실행 시 경로로 준다. 라이선스가
모델마다 다르고 크기도 수십 MB 다.

```bash
# huggingface.co/rhasspy/piper-voices/tree/main/ko/ko_KR 에서 .onnx 를 받는다
ros2 launch jongky_guide guide.launch.py ... tts_voice:=~/voices/ko_KR-xxx.onnx
```

**STT (whisper)** — `pip install openai-whisper`. 젯슨 홈에 torch 휠이 있으니
그걸 먼저 깔면 빌드를 피할 수 있다.

## 알아둘 것

**마이크를 지정할 것.** 아스트라에도 마이크가 있어서(`card 0`) 안 정하면
엉뚱한 장치를 잡는다. `arecord -l` 로 확인하고 USB PnP 쪽(보통 `card 1`)을 준다.
스피커도 마찬가지로 `aplay -l` 을 보고 `audio_device:=plughw:2,0` 식으로 준다.

**음성 인식은 상시가 아니다.** 볼륨이 임계를 넘으면 녹음을 시작하고 조용해지면
끊어서 그 구간만 Whisper 에 넘긴다. 상시로 돌리면 CPU 를 계속 먹고 복도 소음까지
전부 받아쓴다. 오작동하면 `--threshold` 를 올린다.

**목적지 매칭은 부분 일치다.** "304호 어디예요" 에서 "304호 강의장" 을 찾으려면
waypoint 이름이 발화에 들어 있어야 한다. Whisper 가 띄어쓰기를 제멋대로 넣으므로
공백을 지우고 비교한다. 이름을 너무 길게 지으면 안 걸린다.

**층마다 지도와 waypoint 가 따로다.** `map` 프레임은 2D 평면이라 10층과 11층을
한 지도에 못 담는다. `--floors` 를 주면 `map_server/load_map` 으로 런타임에
갈아끼운다 (아래 「층 전환」). 안 주면 예전처럼 층마다 런치를 다시 띄워야 한다.

## 층 전환 — 엘리베이터

**엘리베이터는 주행 구간이 아니라 상태 전이 구간이다.** 버튼을 못 누르고,
좁은 금속 박스라 AMCL 이 어느 층 지도에도 안 맞고, 로봇은 수직으로 움직이는데
엔코더는 정지라고 한다. 그래서 그 구간의 코드는 "목표를 주고 도착을 기다리는"
것이 아니라 **사람이 하는 일을 기다리는** 것이다.

```
안내중 → 엘리베이터앞이동 → 도착(버튼 요청) → 탑승대기 → 이동중(nav2 목표 없음)
      → 하차 → 층 판정·확정 → 지도 교체 → AMCL 재초기화 → 안내 재개
```

설계·전이표·실패 거동·현장 절차는 **`fleet/guide_mission/README.md`** 에 있다.
여기서는 이 패키지가 무엇을 맡는지만 적는다.

| 인자 | 뜻 |
|---|---|
| `floors:=~/floors.yaml` | 층별 지도·waypoint 대장. 주면 층 전환이 켜진다 |
| `floor:=10f` | 기동 층. `map`·`waypoints` 를 대장에서 뽑는다 |
| `floor_detect:=auto\|always\|off` | SSID 자동 판정 정책 |

`guide_node.py` 가 하는 일은 셋이다.

- **`load_map()`** — `map_server/load_map` 을 직접 부른다. `BasicNavigator.changeMap()`
  은 실패해도 로그만 찍고 아무것도 안 돌려주며 서비스가 없으면 무한히 기다린다
  (`robot_navigator.py:647`). 여기서는 결과 코드·시간 초과·올라간 지도 크기를
  전부 확인하고, `amcl.first_map_only` 가 켜져 있지 않은지도 한 번 물어본다
- **`_relocalize_at()`** — 초기 위치를 주고 **정말 잡혔는지 확인한다.**
  `amcl_pose` 가 발행 뒤에 새로 왔는지, 그 값이 준 자리 근처인지,
  `map->odom` TF 가 실제로 나오는지. 하나라도 안 되면 실패다
- **`NavEffects`** — 상태머신이 시키는 일(주행·정지·지도·위치·층 판정·안내
  재개)을 로봇 동작으로 옮긴다. 예외를 밖으로 던지지 않고 `(성공, 이유)` 로만
  답한다

**`localized` 가 False 면 어떤 목적지 요청도 거부한다.** 엘리베이터에 타는
순간 False 가 되고 재초기화가 성공해야 돌아온다. 지도 교체가 실패했는데 계속
주행하는 일이 구조적으로 안 생긴다.

## 아직 없는 것

- **이 패키지는 한 번도 빌드된 적이 없다.** `jmap:80` 의 colcon 목록에
  `jongky_guide` 가 없고 `build/`·`install/` 에도 항목이 없다. 처음 쓸 때는
  `colcon build --packages-select guide_mission jongky_guide` 를 따로 돌려야
  한다 (층 전환을 쓰면 `guide_mission` 도 같이). `guide_node.py` 상단의
  `get_package_share_directory("jongky_guide")` 가 **모듈 최상위**에서
  실행되므로, 빌드 없이 실행하면 `PackageNotFoundError` 로 즉사한다
- **`package.xml` 에 `guide_mission` 의존이 없다.** 런타임 임포트에는 지장이
  없지만(워크스페이스를 source 하면 `PYTHONPATH` 에 들어온다) 적어 두는 편이
  맞다. `jmap` 의 colcon 목록에도 `guide_mission` 이 없다 — 두 파일 다 이번
  작업 범위 밖이었다
- **waypoint 이름이 내부 코드다.** `10a` / `ev1` / `m1` 은 사람이 말하지도
  버튼에서 알아보지도 못한다. `floors.yaml` 의 `labels:` 로 표시 이름을 줄 수
  있게 했지만, **`--floors` 를 안 쓰면 여전히 날 이름이 그대로 보인다.**
  맵핑 중 터미널 버퍼가 새어 들어간 `wwwwwwwwwwwwwwwwwev2`·`.,,` 같은 이름도
  실제 파일에 있다 (`check_floors` 가 경고로 잡는다)
- **엘리베이터 버튼을 못 누른다.** 층 전환은 사람이 태우고 내려 주는 것을
  전제로 짰다. 팔도 없고 층 표시등을 읽는 눈도 없다
- **층 판정이 SSID 와 사람뿐이다.** 기압계도 층 표지판 OCR 도 없다. 젯슨이
  관제 노트북 핫스팟에 붙어 있으면(11층 운용의 전제 조건) 자동 판정이 늘
  "모른다" 로 떨어지고 매번 사람이 골라야 한다
- **안내도 표시** — 층별 피난안내도를 UI 에 띄우고 현재 위치를 찍어 주면
  좋겠지만, 도면과 SLAM 지도의 좌표 정합이 필요하다

### 최근에 메워진 것 (이 목록에 있었다)

- ~~초기 위치를 주는 곳이 없다~~ → `set_start()` 와 UI 의 '여기서 시작'.
  층 전환 뒤 재초기화도 같은 경로를 쓰고, **잡혔는지 확인까지 한다**
- ~~카메라를 아무도 안 띄운다~~ → `guide.launch.py` 가 `openni2_camera` 를
  포함한다 (`use_camera:=false` 로 끈다)
- ~~같은 노드를 두 스레드가 spin 한다~~ → `MultiThreadedExecutor`.
  워커에서는 `guide_node` 를 spin 하지 않는다 (`BasicNavigator` 는 별도 노드라
  그쪽을 도는 것은 그대로다)
- ~~층 전환이 코드에 아예 없다~~ → `fleet/guide_mission`

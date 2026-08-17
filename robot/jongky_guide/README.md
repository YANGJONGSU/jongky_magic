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
| `web/index.html` | 7인치 터치스크린용 UI |
| `launch/guide.launch.py` | Nav2 + UI + 음성 |

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
한 지도에 못 담는다. 층을 옮기면 런치를 다시 띄우거나 `map_server` 의
`load_map` 서비스로 갈아끼워야 한다.

## 아직 없는 것

- **후면 사람 추종** — 계획서의 "사람이 잘 따라오는지 뒤쪽 카메라로 확인" 이
  아직 없다. IMX219 는 동작하므로 YOLO-nano 급 탐지기를 붙이면 된다
- **층 전환** — 엘리베이터는 자율주행 구간이 아니라 상태 전이 구간으로 짜야
  한다. 버튼을 못 누르고, 좁은 금속 박스라 AMCL 이 어느 층 지도에도 안 맞고,
  로봇은 수직으로 움직이는데 엔코더는 정지라고 한다
- **안내도 표시** — 층별 피난안내도를 UI 에 띄우고 현재 위치를 찍어 주면
  좋겠지만, 도면과 SLAM 지도의 좌표 정합이 필요하다

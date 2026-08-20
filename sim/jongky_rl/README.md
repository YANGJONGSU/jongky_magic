# jongky_rl — Isaac Lab 복도 주행 학습 환경

종키프로 안내로봇의 주행 정책을 학습시키기 위한 Isaac Lab `DirectRLEnv`.
DreamerV3(`NM512/dreamerv3-torch`) 로 사전학습한 뒤 실물에서 파인튜닝하는 것이
목표다.

ROS 패키지가 아니다. Isaac Lab venv 에서 직접 돌린다.

## 구성

```
jongky_corridor_env.py     환경 정의 (관측/행동/보상/리셋)
tools/smoke_env.py         환경 인스턴스화 + 몇 스텝 굴려보기
tools/diag_drive.py        구동 진단. 램프·기구학을 우회하고 바퀴에 속도를 직접 꽂는다
tools/check_merged.py      USD 질량 합 · 센서 프레임 생존 확인
tools/inspect_collisions.py  충돌 prim 구조 훑기
dreamer_env.py             dreamerv3-torch 어댑터
train_dreamer.py           DreamerV3 학습 진입점
```

## 사전 준비 — URDF → USD

```bash
# 1) xacro 를 평문 URDF 로. 최상위는 robot.urdf.xacro 다.
#    jongky.urdf.xacro 는 매크로 정의만 있어 빈 <robot> 이 나온다.
xacro robot/jongky_description/urdf/robot.urdf.xacro use_mock:=true > jongky.urdf

# 2) package:// 참조를 상대경로로 바꾸고 STL 을 같이 둔다
sed -i 's|package://jongky_description/meshes/|meshes/|g' jongky.urdf

# 3) USD 변환 — --merge-joints 를 반드시 붙일 것 (아래 참조)
cd ~/isaac/IsaacLab
OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python \
  scripts/tools/convert_urdf.py ~/jongky_usd/jongky.urdf ~/jongky_usd_merged/jongky.usd \
  --merge-joints --joint-target-type velocity \
  --joint-stiffness 0.0 --joint-damping 100.0 --headless
```

> `isaaclab.sh` 는 venv 를 못 찾는다. venv 파이썬을 직접 부를 것.

## 실행

```bash
cd sim/jongky_rl
OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u \
  tools/smoke_env.py --headless --enable_cameras
```

- `--enable_cameras` 없으면 카메라 초기화에서 죽는다
- `-u` 없으면 stdout 버퍼링 때문에 print 가 유실된다

## 실차 일치 항목

이 값들이 어긋나면 sim2real 이 통째로 날아간다.

| | |
|---|---|
| `v_max` | 0.40 m/s |
| `omega_max` | 1.50 rad/s |
| `a_max` | 0.30 m/s² (액션 램프) |
| `wheel_radius` | 0.0335 m |
| `wheel_separation` | 0.11909 m |
| 카메라 HFOV | **57.86도** (`focal_length` 18.958) |

MARS `actor_phase15.pt` 가 종키에 못 올라가는 이유가 정확히 이 불일치다 —
`MAX_VX 1.5` 대 실차 `0.40`, 3.75배.

카메라는 `camera_link` prim 아래에 붙이므로 실측 장착 위치
(base_link 기준 x=0.07, z=0.1656)가 자동 반영된다.

화각은 아스트라가 발행하는 `camera_info` 의 K 행렬에서 뽑았다
(640x480, fx=fy=579.01 → HFOV 57.86도). 줄자로 재는 것보다 정확하다 —
드라이버가 장치에서 읽어온 공장 캘리브레이션 값이기 때문이다.
다시 재려면 이미지를 구독한 상태에서 `ros2 topic echo /camera/rgb/camera_info`
(스트림이 lazy 라 구독자가 없으면 camera_info 도 안 나온다).

## 밟은 지뢰

### 1. 팬텀 질량 — `--merge-joints` 를 반드시 쓸 것

URDF 의 프레임 전용 링크(`camera_link` · `imu_link` · `laser` ·
`rear_camera_link` · `tof_l/r_link` · `base_footprint`)에는 `<inertial>` 이
없다. ROS 에서는 정상이지만 **URDF 임포터가 이런 링크마다 기본 질량 1.0 kg 을
박는다.**

```
병합 전: [1.0, 2.503, 1.0, 1.0, 0.05, 1.0, 0.05, 1.0, 1.0, 1.0]  합계 9.6 kg
병합 후: [2.503, 0.05, 0.05]                                      합계 2.603 kg
```

2.603 kg 로봇이 **9.6 kg (3.7배)** 이 된다. 증상은 "바퀴 토크가 계속 포화되고
로봇이 제자리에서 진동하며 안 나간다". 병합해도 `camera_link` 같은 센서
프레임은 Xform prim 으로 남으므로 카메라를 붙이는 데 지장이 없다.

**변환 직후 `robot.data.default_mass` 합이 2.603 kg 인지 확인할 것**
(`tools/check_merged.py`).

> 변환 시 "Merging bodies with inertia is deprecated" 경고가 뜬다. 나중에
> 깨지면 URDF 프레임 링크에 미소 관성을 넣는 쪽으로 간다.

### 2. 액추에이터 게인 — `damping=100` 은 750배 과하다

바퀴는 0.05 kg 이다. 2.603 kg 을 0.30 m/s² 로 가속하는 데 필요한 바퀴 토크는
약 0.013 N·m 뿐이다. `damping=100` 이면 속도 오차 0.1 rad/s 에 10 N·m 를 때려
즉시 포화되고 토크가 +10 ↔ −10 으로 뒤집히는 뱅뱅 진동이 난다.

→ `damping=2.0`, `effort_limit_sim=0.5`

`effort_limit` · `velocity_limit` 은 deprecated 다. 특히 `velocity_limit` 은
implicit actuator 에서 **그냥 무시된다.** `*_sim` 접미사를 쓸 것.

### 3. 접촉 발산 — `armature` 로 잡는다

차체 2.503 kg 대 바퀴 0.05 kg, **질량비 50:1**. 가벼운 바퀴가 50배 무거운
몸통의 접촉 임펄스를 받으면 PhysX 솔버가 발산한다 — 바퀴가 목표 11.94 rad/s 를
벗어나 ±35 rad/s 로 날뛴다. 바닥 마찰을 올리면 오히려 심해진다.

→ `armature=1e-3` + `solver_velocity_iteration_count=4`

바퀴 고유 관성이 2.2e-5 kg·m² 이므로 1e-3 은 그보다 충분히 크다. armature 는
조인트 자유도에만 더해지는 수치적 항이라 **로봇의 실제 질량·관성은 그대로다**
(`default_mass` 는 여전히 2.603 kg). 바퀴 질량을 억지로 올리는 건 실측값
왜곡이라 다른 얘기다.

### 4. 뒤집힘 판정을 높이로 하면 안 된다

루트 링크가 `base_footprint` 다. 정의상 접지면이라 정상 주행 중에도 z 가
0 근처다. `root_pos_w[:,2] < 0.01` 로 판정하면 **매 스텝 종료된다.**

→ `projected_gravity_b[:,2] > -0.5`

### 5. exit code 를 믿지 말 것

`isaaclab.sh`, `convert_urdf.py`, 스모크 테스트 전부 **실패해도 exit code 가
0 이다.** 출력을 직접 볼 것.

### 6. 액션에 tanh 를 두 번 걸면 안 된다

`_pre_physics_step` 이 액션에 `torch.tanh` 를 걸고 나서 실차 한계를 곱했다.
그런데 **DreamerV3 의 연속 액터는 이미 tanh 로 [-1,1] 을 뱉는다.** 두 번
걸리면 실효 범위가 [-0.762, 0.762] 로 줄어든다.

| | 한계값 | 정책이 실제로 낼 수 있던 값 |
|---|---|---|
| `v` | 0.40 m/s | tanh(1)×0.40 = **0.305 m/s** |
| `omega` | 1.50 rad/s | tanh(1)×1.50 = **1.142 rad/s** |

sim2real 을 위해 가장 맞추기로 한 그 값(v_max 0.40)에 정책이 영원히 못 닿는다.
양 끝에서 기울기도 이중으로 눌린다.

더 나쁜 건 조용하다는 점이다. `train_dreamer.check_reachable` 은 20초 에피소드에
`V_MAX × 20 = 8.0 m` 를 갈 수 있다고 계산했지만 실제 최대 주행은 6.09 m 였다.
최악 필요 거리 6.0 m 와의 여유가 33% 가 아니라 **1.6%** 였고, 그래서 "여유 20%
미만" 경고가 뜨지 않았다. 시작 yaw ±20°·목표 y 랜덤까지 넣으면 가장 먼 목표는
아예 못 닿는다. **검사가 막으려던 실패 모드를 검사 자신이 놓쳤다.**

→ 스케일을 `jongky_corridor_env.scale_action()` 하나로 모으고 tanh 대신
`torch.clamp(actions, -1, 1)` 을 쓴다. 범위 밖 액션(tanh 를 안 쓰는 알고리즘,
탐색 노이즈)에는 clamp 가 하드 리밋으로 대비한다.

→ `check_reachable` 은 `V_MAX` 를 읽지 않고 `scale_action(torch.ones(1, 2))` 에
물어본다. 스케일을 또 손대도 검사가 따라오게 하려는 것이다 — 이 사고가 정확히
env 와 검사가 서로 다른 값을 믿어서 났다.

고친 뒤 여유는 **21%** 다 (가로 어긋남 1.0 m = 시작 y ±0.4 + 목표 y ±0.6 와
가속 램프 손실 0.27 m 까지 넣은 값. 축만 보면 22%). 20% 문턱을 겨우 넘으므로
회전 결손(아래)이 잡히기 전까지는 빠듯하다고 봐야 한다 — 더 벌려면
`goal_x_range` 상한 7.0 → 6.5 (28%) 나 `--episode-steps 720` (24초, 35%).

## 검증 결과

바퀴 11.94 rad/s 명령 → 정상속도 **v_body 0.398 m/s** (목표 0.40, 오차 0.5% 는
슬립), 3초간 유지, 0.888 m 주행. 가속 램프도 실차 `a_max` 대로 누적된다.

## TODO

- **목표 마커의 실물 대응** — 관측이 픽셀뿐이라 목표가 화면에 보이지 않으면
  정책이 복도 직진밖에 못 배운다. 그래서 목표 지점에 주황 원기둥을 세운다
  (충돌 없음, kinematic, 리셋 때 목표 위치로 이동). 실물 복도에서 강의장
  문·표지판이 이 역할을 해야 sim2real 이 성립한다 — 현장 확인 필요.
  저차원 상태(목표거리, sin/cos 방위, v, omega)는 `state_space` 로 빼 두었다
- **`clipping_range` 하한** — 지금 0.1m 가정. 아스트라 최소거리를
  `check_depth_min_range.py` 로 실측해 교체할 것
- **제자리 회전 결손** — 캐스터가 URDF 상 고정 구슬이라 시뮬에서 구르지 못하고
  끌린다. 최대 각속도 액션으로도 이론의 1/10 수준밖에 안 돈다 (정확한 배수는
  아래 "제자리 회전" 절 — 액션 이중 압착을 고쳤으므로 재측정이 필요하다).
  회전이 필요한 정책을 학습시키기 전에 URDF 쪽에서 잡아야 한다

**끝난 것** — "복도 폭 2.4 m 를 실측으로 교체" 와 "SLAM 지도에서 환경 기하
뽑아오기" 는 아래 "실측 지도 복도" 절에서 했다. `jongky_map_corridor_env.py` 가
점유 격자 지도에서 구간별 폭(사물함 1.20 m · 개방 1.69 m)을 뽑아 복도를 세운다.
`jongky_corridor_env.py` 의 스칼라 2.4 m 는 비교용으로 남겨 둔 것이다.

## DreamerV3 연결 — dreamerv3-torch 를 쓴다

**RLlib 은 못 쓴다.** 신 API 스택의 `SingleAgentEnvRunner` 가 `gym.make_vec()`
으로 **스스로 벡터화를 하는데**, Isaac Sim 은 한 프로세스에 하나만 뜨므로
env 생성을 넘겨줄 수가 없다.

```
TypeError: The environment must inherit from the gymnasium.Env class,
actual class: IsaacLabRLlibVecEnv
```

대신 `NM512/dreamerv3-torch` 를 쓴다 (계획서가 원래 후보로 적어 둔 것).
env 를 그냥 받아 쓰므로 이 마찰이 없다.

```bash
git clone --depth 1 https://github.com/NM512/dreamerv3-torch.git ~/dreamerv3-torch

cd sim/jongky_rl
OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u train_dreamer.py \
    --headless --enable_cameras --steps 50000
```

> **`requirements.txt` 를 그대로 깔지 말 것.** `torch==2.4.1` 핀이 Isaac Lab 의
> 2.7.0 을 깨뜨린다. 실제로 모자란 건 `ruamel.yaml` 하나뿐이다.

### 어댑터가 맞춰야 하는 것들

**구 gym API 다.** gymnasium 이 아니다 — `reset() -> obs`, `step() -> 4-튜플`.
obs 는 `image`(uint8 0~255) · `is_first` · `is_terminal` 을 가진 dict.

**`is_terminal` 은 진짜 종료만 참이다.** 시간초과(truncated)는 거짓이어야 한다.
시간이 다 됐다고 그 상태의 가치가 0 인 것은 아니다 — 뭉뚱그리면 크리틱이
에피소드 끝을 전부 실패로 배운다.

**정규화를 두 번 하면 안 된다.** dreamerv3-torch 가 내부에서 한다. env 의
`normalize_obs` 를 False 로 두고 uint8 을 그대로 넘긴다.

**env 는 싱글턴이다.** `dreamer.py` 는 train_envs 와 eval_envs 를 따로 만드는데
Isaac Sim 은 두 번째 생성에서 죽는다. 같은 인스턴스를 돌려준다.

**`envs=1`, `parallel=False`.** 서브프로세스를 띄우면 거기서 또 Isaac Sim 을
만든다. GPU 병렬성을 버리는 셈이지만 DreamerV3 는 sample-efficient 설계이고
DayDreamer 도 실물 1대로 학습했다.

### 검증 결과

1200 스텝 스모크 테스트에서 월드모델·액터·크리틱 손실이 모두 갱신되고
에피소드가 쌓이는 것을 확인했다 (`model_loss` 682 / `image_loss` 675 /
`actor_loss` 0.1 / `value_loss` 9.2, 7 에피소드). 학습 곡선을 논하기엔
이르지만 파이프라인은 뚫렸다.

---

## 실측 지도 복도 — 점유 격자에서 형상 뽑아 쓰기

`jongky_corridor_env.py` 는 복도를 `corridor_width = 2.4` 스칼라 하나로 세운다.
**실측은 개방 구간 1.68~1.70 m, 사물함 구간 1.20 m 다.** 2.4 m 는 개방 구간
대비 +42%, 사물함 구간 대비 +100% 다. 게다가 실제 복도는 한 숫자로 표현되지
않는다 — 사물함이 있는 구간만 좁아지는 이봉 구조다.

이걸 고치려고 점유 격자 지도(`.pgm`+`.yaml`)에서 실제 벽 배치를 뽑아
시뮬 복도를 세우는 경로를 만들었다. 기존 파일은 건드리지 않았다.

| 파일 | 하는 일 | Isaac 필요 |
|---|---|---|
| `map_geometry.py` | 지도 → 벽 선분 + 폭 프로파일 + **실측 대조 검증** | 아니오 |
| `corridor_L10b.json` / `corridor_L11.json` | 추출 결과 | — |
| `jongky_map_corridor_env.py` | 그 형상으로 복도를 세우는 env | 예 |
| `tools/check_corridor_fit.py` | 최협 구간 통과·판정선·회전 확인 | 예 |

```bash
# 1) 매핑 PC 에서 형상 추출 (numpy/scipy/opencv 만 있으면 된다)
python3 map_geometry.py /root/maps_local/L10b.yaml -o corridor_L10b.json \
        --debug-png L10b_debug.png

# 2) Isaac Lab 쪽에서 학습 — 기존 env 와 나란히 쓸 수 있다
JONGKY_CORRIDOR_JSON=corridor_L10b.json python -u train_dreamer.py
```

### 어떻게 뽑는가

1. **잡음 제거.** 유리창 너머로 라이다가 새어 나가 생기는 부채꼴은 두 얼굴이다.
   점유 셀 쪽에서는 작은 연결 성분(10 셀 미만 → 버린다), 자유 셀 쪽에서는
   가느다란 쐐기(반지름 0.30 m 원반 opening 으로 지운다)로 나타난다.
   opening 기준은 최협 실측 1.20 m 의 절반도 안 되므로 진짜 복도는 안 깎인다.
2. **중심선.** 세선화한 골격에서 그래프 지름(두 번 BFS)을 복도 척추로 잡는다.
3. **폭.** 척추를 따라 `2*거리변환` 으로 잰다.
4. **구간화.** 폭이 거의 일정한 구간으로 바텀업 병합한다 → 시뮬 벽 배치.

### 측정 편향을 먼저 잡았다

폭 추정치를 그냥 믿으면 안 돼서 **폭을 아는 합성 복도**로 두 추정치를 먼저 쟀다.

| 추정치 | 편향 | 회전 강건성 |
|---|---|---|
| `2*distanceTransform` | **0.000 m** | 30° 기울여도 유지 |
| 법선 레이캐스트 (원시) | **+0.050 m** (정확히 한 셀) | 격자축에서만 유효 |

1.20 / 1.68 / 1.70 / 2.40 m 합성 복도 전부에서 레이캐스트가 정확히 +0.050 m
치우쳤다. 그래서 레이캐스트는 `- resolution` 보정해서 **교차 검증용으로만**
쓰고, 주 추정치는 편향 0 인 `2*DT` 로 간다.

### 검증 결과 — 외부 실측 대조

**지도 내부 지표로는 판정하지 않았다.** 기준은 줄자 실측(1.68~1.70 / 1.20)과
실차 footprint 뿐이다.

무편향 KDE 봉우리 탐지(실측값을 넣지 않고 찾은 것):

| 지도 | 봉우리 1 | 봉우리 2 | 이봉 구조 |
|---|---|---|---|
| L10b | **1.120 m** | **1.620 m** | 나온다 |
| L11 | **1.660 m** | 2.140 m | 부분적 |

L10b 척추 36 m 의 구간별 폭 — 실측 이봉 구조가 그대로 보인다:

```
   s0     s1    len   width   wall%
 8.80  13.45   4.65   1.100   100%
13.45  15.00   1.55   1.400   100%
15.00  16.45   1.45   1.639   100%
16.45  19.55   3.10   1.200   100%   ← 사물함 구간, 실측 1.20 과 정확히 일치
19.55  23.65   4.10   2.100   100%   ← 교차로
23.65  31.00   7.35   1.620   100%   ← 개방 구간, 실측 1.68~1.70 대비 -4%
```

지도 20판 전체에서:

| 대역 | 추출 범위 | 실측 | 오차 |
|---|---|---|---|
| 개방 | 1.569 ~ 1.700 m | 1.68~1.70 | **-7.2% ~ +0.6%** |
| 사물함 | 1.079 ~ 1.400 m | 1.20 | **-10.1% ~ +16.7%** |

**개방 구간이 20판 전부에서 -5% 쯤 일관되게 낮다.** 판마다 흔들리는 것이
아니라 계통 편향이다. 원인은 점유 격자의 원리다 — 빔이 닿은 셀을 통째로
점유로 찍으므로 벽면이 복도 안쪽으로 반 셀~한 셀 먹고 들어간다. 5 cm 격자에서
1.69 m 를 재면 원리적으로 ±0.05 m (±3%) 아래로는 못 내려간다.
**즉 -4% 는 지도 오류가 아니라 격자 해상도의 바닥이다.**

그래서 env 의 `width_source` 기본값은 `"measured"` 다 — 실측값(1.69/1.20)에
0.15 m 안으로 붙는 구간은 실측값으로 스냅하고, **무엇을 갈아끼웠는지 로그로
남긴다**. 지도 값을 그대로 쓰려면 `"map"` 으로 두면 된다.

### 로봇 제원 대조

footprint 는 `nav2_params.yaml` 과 같은 직사각형이다
(`[[-0.14,-0.085],[-0.14,0.085],[0.08,0.085],[0.08,-0.085]]`).
직진은 전폭 0.17 m 만 보면 되지만 **복도에서 돌아서려면 외접원**이 기준이다.

| 항목 | 값 | 최협 1.20 m 구간에서 |
|---|---|---|
| 전폭 | 0.170 m | 측면 여유 0.515 m |
| 외접원 반지름 | 0.1638 m | 여유 0.436 m |

`tools/check_corridor_fit.py` 로 물리를 돌려서 확인한 것:

- **최협 1.20 m 구간 직진 통과** — 통과함. 통과 중 최소 측면 여유 0.384 m
- **종료 판정선** — 1.20 m 구간에서 `|y| > 0.510` 에 정확히 걸리고,
  **같은 y 가 2.10 m 구간에서는 안 걸린다**. 구간별 판정이 실제로 동작한다는
  뜻이고, 이게 스칼라 `corridor_width` 를 버린 이유 그 자체다

### 벽 기하는 밀폐되어 있다

폭이 바뀌는 경계에 연결 판(`step_*`)을 넣지 않으면 넓어지는 쪽에 벽이 통째로
비는 틈이 생긴다. 물리적으로도 틀리지만 관측이 픽셀이라 더 나쁘다 — 카메라가
그 틈으로 빈 공간을 보고 실제 복도엔 없는 특징을 정책이 학습한다.
1 cm 격자로 래스터화해 flood fill 로 확인했다: **경계 접촉 0 픽셀(밀폐)**,
5개 구간 전부 설계 폭과 1 cm 이내 일치.

### 알아낸 것 — 제자리 회전이 이론의 1/10 수준이다 (이 변경과 무관)

최대 각속도 액션(`act=1.0`)으로 20초를 돌려도 **3.1 rad(0.5 바퀴)** 밖에 안
돈다. 원인이 복도인지 로봇인지 갈라 보려고 대조군을 돌렸다:

| 조건 | 20초 회전량 |
|---|---|
| 최협 1.20 m 구간 | 3.12 rad |
| 넓은 2.10 m 구간 | 3.07 rad |
| **기존 2.4 m env (손 안 댐)** | **3.11 rad** |

셋이 같다. **복도 폭과 무관한 기존 로봇 동역학 문제**이고, 캐스터가 URDF 상
고정 구슬이라 구르지 못하고 끌리는 것(위 "밟은 지뢰" 참조)이 유력하다.
이 변경이 만든 것도 악화시킨 것도 아니지만, 회전이 필요한 정책을 학습시키기
전에 반드시 따로 잡아야 한다.

**이론값 대비 몇 배인지는 액션 이중 압착(지뢰 6번)을 고치기 전후가 다르다.**

| | `act=1.0` 의 각속도 | 20초 이론 회전량 | 측정 3.1 rad 대비 |
|---|---|---|---|
| 측정 당시 (tanh 두 겹) | 1.142 rad/s | 22.85 rad | **1/7.4** |
| tanh 걷어낸 지금 | 1.500 rad/s | 30.0 rad | 1/9.6 (추정) |

즉 위 표의 3.1 rad 은 **액션에 tanh 가 한 겹 더 걸려 있던 상태의 값**이고,
당시 이론값은 30 rad 이 아니라 22.85 rad 이었다. 지금은 `act=1.0` 이 진짜로
`OMEGA_MAX` 1.50 rad/s 를 내므로 이론값 30 rad 이 다시 맞다.

캐스터 끌림은 명령 크기와 별개로 걸리는 저항이라 절대 회전량이 크게 안 변할
공산이 크고, 그러면 결손은 1/9.6 쯤이 된다. **다만 이건 추정이니
`tools/check_corridor_fit.py` 를 다시 돌려 재측정할 것.** 그 스크립트는 이제
이론값을 상수로 박지 않고 액션 스케일러(`scale_action`)에서 뽑으므로,
어느 쪽 기준인지 헷갈릴 일이 없다.

대조군 결론(복도 탓이 아니다)은 그대로 유효하다 — 세 조건이 전부 같은 명령을
썼으므로 명령 크기가 얼마였든 셋이 같다는 사실은 변하지 않는다.

### 아직 안 한 것

- **복도를 곧게 폈다.** 중심선 호길이를 x 축으로 삼으므로 곡률이 사라진다.
  이번에 42% 틀렸던 것은 폭이지 곡률이 아니라서 폭부터 잡았다. 곡률이
  필요하면 JSON 의 `walls`(월드 좌표 벽 선분 280개)를 쓰면 된다
- **10층 지도의 복제 결함은 검증하지 않았다** (다른 사람이 보고 있다).
  다만 파이프라인은 어느 지도를 넣어도 돌고, 20판 결과를 위 표에 남겼다
- **학습은 돌리지 않았다.** 환경이 서고 로봇이 최협 구간을 통과하는 것까지만
  확인했다. 이 복도에서 정책이 실제로 학습되는지는 별개 문제다

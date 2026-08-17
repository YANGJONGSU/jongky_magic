# jongky_rl — Isaac Lab 복도 주행 학습 환경

종키프로 안내로봇의 주행 정책을 학습시키기 위한 Isaac Lab `DirectRLEnv`.
DreamerV3(Ray RLlib) 로 사전학습한 뒤 실물에서 파인튜닝하는 것이 목표다.

ROS 패키지가 아니다. Isaac Lab venv 에서 직접 돌린다.

## 구성

```
jongky_corridor_env.py     환경 정의 (관측/행동/보상/리셋)
tools/smoke_env.py         환경 인스턴스화 + 몇 스텝 굴려보기
tools/diag_drive.py        구동 진단. 램프·기구학을 우회하고 바퀴에 속도를 직접 꽂는다
tools/check_merged.py      USD 질량 합 · 센서 프레임 생존 확인
tools/inspect_collisions.py  충돌 prim 구조 훑기
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

MARS `actor_phase15.pt` 가 종키에 못 올라가는 이유가 정확히 이 불일치다 —
`MAX_VX 1.5` 대 실차 `0.40`, 3.75배.

카메라는 `camera_link` prim 아래에 붙이므로 실측 장착 위치
(base_link 기준 x=0.07, z=0.1656)가 자동 반영된다.

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

## 검증 결과

바퀴 11.94 rad/s 명령 → 정상속도 **v_body 0.398 m/s** (목표 0.40, 오차 0.5% 는
슬립), 3초간 유지, 0.888 m 주행. 가속 램프도 실차 `a_max` 대로 누적된다.

## TODO

- **목표 가시성** — 지금 목표는 좌표로만 존재하고 카메라에 안 보인다. 정책이
  복도 직진은 배워도 특정 강의장을 고를 수 없다. 시각 마커를 둘 것인지
  관측에 목표 벡터를 넣을 것인지 정해야 한다. 저차원 상태는 `state_space` 로
  빼 두었다 (목표거리, sin/cos 방위, v, omega)
- **카메라 FOV** — 지금 HFOV 60도 가정(`focal_length=18.15`). 아스트라 실측으로
  교체할 것. `clipping_range` 하한도 최소거리 실측 후 교체
- **복도 폭** — 지금 2.4 m. 현장 실측으로 교체
- **환경 기하** — SLAM 지도에서 복도를 뽑아오면 실제 건물과 맞출 수 있다
- **DreamerV3 래퍼** — RLlib 은 gymnasium 인터페이스를 받는다. 병렬 env 는
  4~16 개로 (Isaac Lab 예제는 PPO 용 수천 개지만 DreamerV3 는 sample-efficient
  설계라 그렇게 필요 없고, 카메라 렌더링 VRAM 도 16GB 에 그 정도까지만 들어간다)

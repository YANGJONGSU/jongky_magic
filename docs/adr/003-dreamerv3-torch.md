# ADR 003: DreamerV3 구현체 — RLlib 이 아니라 dreamerv3-torch

- **Status**: 채택 (Accepted). 배선은 끝났고 **학습은 아직 안 돌렸다**
- **Date**: 2026-08-17 (문서화 2026-08-20)

## Context

주행 정책을 Isaac Lab 에서 사전학습한 뒤 실물에서 파인튜닝하는 것이 시뮬
트랙의 목표다(`sim/jongky_rl/README.md:3-5`).

처음에는 **Ray RLlib 2.52.1 내장 DreamerV3** 로 잡았다. 예전 RLlib DreamerV3
가 TensorFlow 전용이었던 것이 2.52 에서 torch 전용으로 바뀌어, Blackwell
GPU 에 TF 를 올릴 걱정이 없어진 것이 이유였다. `dm-tree`·`ray[tune]` 을
설치하고 ray 버전을 `==2.52.1` 로 핀까지 잡았다.

## Decision

**`NM512/dreamerv3-torch` 를 쓴다. RLlib 은 버린다.**

RLlib 신 API 스택의 `SingleAgentEnvRunner` 가 **`gym.make_vec()` 으로 스스로
env 를 벡터화**하는데, Isaac Sim 은 한 프로세스에 하나만 뜨므로 env 생성을
넘겨줄 수가 없다.

```
TypeError: The environment must inherit from the gymnasium.Env class,
actual class: IsaacLabRLlibVecEnv
```

`sim/jongky_rl/README.md:209-221`, `train_dreamer.py:14-16`,
`dreamer_env.py:3-5`.

**TF 문제와는 무관한 이유다.** RLlib 을 버린 근거가 프레임워크가 아니라
**벡터화 소유권**이라는 점이 중요하다 — TF 가 고쳐져도 이 마찰은 남는다.

## Consequences

**좋은 쪽**

- dreamerv3-torch 는 env 를 그냥 받아 쓴다. Isaac Sim 싱글턴 제약과 안 부딪힌다

**대가 — 어댑터가 흡수해야 하는 것들** (`dreamer_env.py`)

| | |
|---|---|
| **구 gym API** 다. gymnasium 이 아니다 | `reset() -> obs`, `step() -> 4-튜플`. obs 는 `image`(uint8) · `is_first` · `is_terminal` dict (`sim/jongky_rl/README.md:236-237`) |
| `is_terminal` 은 **진짜 종료만** 참 | 시간초과를 뭉뚱그리면 크리틱이 에피소드 끝을 전부 실패로 배운다 (`sim/jongky_rl/README.md:239-241`) |
| 정규화를 두 번 하면 안 된다 | dreamerv3-torch 가 내부에서 한다. uint8 을 그대로 넘긴다 (`sim/jongky_rl/README.md:243-244`, `dreamer_env.py:152`) |
| env 는 싱글턴 | `dreamer.py` 가 train/eval env 를 따로 만드는데 두 번째 생성에서 죽는다. 같은 인스턴스를 돌려준다 (`sim/jongky_rl/README.md:246-247`) |
| `envs=1`, `parallel=False` | 서브프로세스를 띄우면 거기서 또 Isaac Sim 을 만든다 (`sim/jongky_rl/README.md:249-250`, `dreamer_env.py:20-24`) |
| `requirements.txt` 를 그대로 깔면 안 된다 | `torch==2.4.1` 핀이 Isaac Lab 의 2.7.0 을 깨뜨린다. 실제로 모자란 건 `ruamel.yaml` 하나 (`sim/jongky_rl/README.md:231-232`) |
| 저장소 밖 의존 | `~/dreamerv3-torch` 를 따로 clone 해야 한다 (`train_dreamer.py:50`, `:88`) |

**아직 안 된 것 — 학습을 한 번도 안 돌렸다**

이 ADR 은 "어느 구현체를 쓰나" 만 결정한다. 학습이 아직 안 돈 것은 별개 사유다
(`README.md:56`).

`전체-작업계획.md` C절(작업 노트)이 학습 전 선결 항목으로 셋을 적어 뒀는데,
**2026-08-20 코드 확인 기준 둘은 이미 고쳐져 있다.** 노트 쪽이 뒤처졌다.

| 노트의 항목 | 코드 현재 상태 |
|---|---|
| C3 tanh 이중 압착 (`v_max` 0.40 에 못 닿음) | **고쳐졌다.** `jongky_corridor_env.py:63-80` 이 `scale_action()` 에서 tanh 를 빼고 clamp 로 갔다. 왜 그런지까지 함수 docstring 에 남아 있다 |
| C4 학습 경로가 구식 scalar env 하드코딩 | **고쳐졌다.** `dreamer_env.py:124-139` 이 `_ENV_KINDS` 레지스트리를 두고 **`DEFAULT_ENV_KIND = "map"`**(`:128`). 없는 cfg 키에 조용히 `setattr` 되던 것도 `:169-174` 가 막는다 |
| C1 캐스터 회전 결함 | **남아 있다.** `sim/jongky_rl/README.md:199` 가 "제자리 회전 결손" 을 알려진 문제로 유지한다. 우회 경로는 `is_sim:=true` + `--joint-damping 0.0` 둘 **다** 주는 것이고, 하나만 빠지면 **에러 없이** 수정이 무효가 된다 (`sim/jongky_rl/README.md:43-52`) |
| C2 Isaac Lab 자동 리셋 오염 | 어댑터가 방어한다 — `dreamer_env.py:28-50` 이 `DirectRLEnv.step()` 순서를 근거로 종료 프레임을 따로 스냅샷한다. **"손실 곡선에도 안 나타나는" 종류라 이 방어 코드를 지우지 말 것** |

**안전망**: nav2 가 전 구간에서 baseline 으로 남는다. 이 트랙이 수렴하지
않아도 로봇은 굴러간다(`전체-작업계획.md` 8절).

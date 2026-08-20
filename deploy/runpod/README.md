# RunPod — Cosmos 합성 데이터

## 파드 설정

| 항목 | 값 |
|---|---|
| Template | `runpod/pytorch:2.2.0-py3.10-cuda12.1.1-devel-ubuntu22.04` |
| GPU | RTX 4090 / L40S / A100 (**sm_100 이상은 안 된다** — 아래 참고) |
| Network Volume | 100GB, `/workspace` 에 마운트 |
| Container Disk | 20GB |

**Network Volume 을 먼저 만들고 거기에 파드를 붙인다.** 파드를 버려도
`/workspace` 가 남아서, 호스트가 나쁘면 몇 번이고 다시 띄울 수 있다.

### 왜 이 템플릿인가

Cosmos1GP 는 python 3.10 을 전제한다 — `mmgp==3.1.2` 가 `python>=3.10`,
`torch>=2.1.0`. 이 이미지는 시스템 파이썬이 3.10, torch 2.2.0+cu121 이라
**venv 를 새로 만들 필요가 없다.**

다만 torch 는 2.4.1 로 올린다. mmgp 3.1.2 의 선언은 `torch>=2.1.0` 이지만
그게 틀렸다 — `safetensors2.py` 가 `torch.uint16` 을 쓰고 그 dtype 은 torch
2.3 에서 생겼다. CUDA 계열(cu121)은 그대로 두고 파이썬 쪽 버전만 올리는 것이라
드라이버 정합은 유지된다.

앞서 python 3.12 인 24.04 이미지로 시도했다가 uv 설치 → 3.10 venv 생성 →
torch 직접 선택으로 이어졌고, `uv not found` / `setuptools AssertionError` /
휠 불일치가 전부 거기서 나왔다. 템플릿 하나가 그 사슬을 만든다.

### GPU 제약

cu121 로 빌드된 torch 2.2 는 **sm_90 까지** 지원한다. RTX 5090(sm_120) 계열은
이 템플릿으로 GPU 를 아예 못 본다. `setup_cosmos.sh` 가 3단계에서 먼저 막는다.

## 순서

```bash
cd /workspace && git clone -b map-quality https://github.com/YANGJONGSU/jongky_magic
```
```bash
bash /workspace/jongky_magic/deploy/runpod/preflight.sh
```
```bash
bash /workspace/jongky_magic/deploy/runpod/setup_cosmos.sh
```
```bash
bash /workspace/jongky_magic/deploy/runpod/serve.sh
```

### preflight 를 먼저 돌리는 이유

체크포인트는 약 20GB 다. 지난 파드는 그걸 다 받고 환경을 다 만든 **뒤에야**
GPU 가 CUDA 컨텍스트를 못 만든다는 걸 알았고, 그건 파드 안에서 고칠 수 있는
종류가 아니었다.

`preflight.sh` 는 드라이버 API 를 한 호출씩 내려가며 어디서 죽는지 짚는다:

```
cuInit           -> 성공
cuDeviceGetCount -> 성공 · GPU 1개
cuDeviceGet      -> 성공
cuCtxCreate      -> DEVICE_UNAVAILABLE
```

torch 는 이 네 단계를 메시지 하나로 뭉쳐서 보고한다. 그래서 휠을 네 번 바꿔도
어느 층이 문제인지 안 보였다. 열거는 되고 컨텍스트만 안 서는 상태는
`/dev/nvidia-uvm` 이 없을 때 나오고, 그건 컨테이너 권한이라 파드 안에서 못
고친다 — RunPod 도 이 상태를 "그 워커가 망가진 것" 으로 보고 조치는 재배포다.

10초 만에 판정되고, 실패하면 아무것도 안 받은 상태라 잃는 게 없다.

## 규칙: 플랫폼이 넣어둔 torch 를 갈아치우지 않는다

이미지의 torch 는 RunPod 이 자기 드라이버에 맞춰 넣은 것이다. 다시 고르는 건
그 보증을 버리는 것이다.

`requirements.txt` 안의 `peft` / `transformers` / `optimum-quanto` 는 torch 를
의존성으로 갖기 때문에, 제약 없이 설치하면 pip 가 조용히 torch 를 올린다.
`setup_cosmos.sh` 는 torch 2.4.1 + numpy<2 로 맞춘 뒤 그 조합을 constraints 로
못박고, 설치 후에 **실제로 안 바뀌었는지 다시 확인한다.**

numpy 를 같이 묶는 이유: 이 torch 는 numpy 1.x 로 빌드됐는데 의존성으로 numpy 2.2
가 딸려 들어오면 `_ARRAY_API not found` 가 난다. 경고처럼 보이지만 그 상태의
torch 는 numpy 변환이 통째로 죽어 있다.

## 검증은 서버가 쓰는 심볼로

`import mmgp` 는 통과하는데 서버는 `from mmgp import offload` 에서 죽을 수 있다.
`offload.py` 가 `safetensors2` 를 끌고 들어가고 거기서 문제가 터지는데, 패키지
`__init__` 은 그 경로를 안 건드리기 때문이다. 실제로 그렇게 한 번 통과시킨 뒤
서버가 죽었다. 그래서 검증은 `gradio_server_v2w.py` 17번 줄과 같은 import 를 쓴다.

## 클립 생성

```bash
python3 deploy/runpod/gen_clip.py --seed-video deploy/runpod/seeds/corridor_10f.mp4 \
  --prompt-file deploy/runpod/seeds/prompt_v1.txt
```

한 번에 121프레임(약 5초)이다. 20분짜리를 넣어 20분을 받는 물건이 아니다.
범위는 길이가 아니라 **씨앗 지점 수**로 넓힌다 — 길게 뽑을수록 복도 폭과
카메라 높이가 흘러가고, 그러면 depth 와 액션의 관계가 깨져서 학습 신호가
거짓이 된다.

## 배치 생성

```bash
cd /workspace/Cosmos1GP
nohup python3 /workspace/jongky_magic/deploy/runpod/run_batch.py \
    --seeds-dir /workspace/jongky_magic/deploy/runpod/seeds \
    > /workspace/batch.log 2>&1 &
```

```bash
tail -f /workspace/batch.log
```

씨앗 14지점 × 조건 4가지 = 56개. A40 에서 클립당 약 8.7분이므로 **8시간**쯤 걸린다.

### 왜 이 설정인가

측정값: 1104x832 · 25스텝 · 121프레임 = **19.3분/클립** (A40, profile 1).
배치는 832x624 · 20스텝으로 돌린다 — 해상도가 0.57배, 스텝이 0.8배라 0.45배,
즉 8.7분이 된다.

832x624 는 화질 타협이 아니다. 아스트라가 640x480 이므로 오히려 **1104x832
보다 원본에 가깝다**. 4:3 이라 복도 폭·카메라 높이의 비율도 그대로다.

### 순서: 조건이 바깥, 씨앗이 안쪽

조건 1을 14지점 전부에 대해 먼저 끝내고 조건 2로 넘어간다. 중간에 멈춰도
노선 전체가 한 번은 덮이게 하기 위해서다. 반대로 돌면 한 지점만 조건 4개를
갖고 나머지 13지점은 아무것도 없다. 강화학습 데이터에서 중요한 건 한 곳의
깊이가 아니라 상황의 넓이다.

이미 만든 조합은 `/workspace/batch_manifest.jsonl` 을 보고 건너뛴다.
끊겼다 다시 켜도 이어서 간다.

### 씨앗을 고른 방법

`make_seed.py scan` 으로 15초마다 표본을 뽑아 159장을 만든 뒤, 아래 기준으로
14지점을 골랐다.

| 기준 | 값 | 왜 |
|---|---|---|
| 노출 | 날아간 화소 <5%, 어두운 화소 <25% | 천장 조명에 날아가면 구조가 안 보인다 |
| 선명도 | 라플라시안 분산 >60 | 흔들린 프레임 제외 |
| **원근선** | 수평/수직이 아닌 긴 직선 ≥2 | 복도인지 벽인지 가른다 |
| 간격 | ≥75초 | 한 구간에 몰리지 않게 |

원근선이 핵심이다. 밝기만 보면 벽만 찍힌 프레임과 엘리베이터 앞 역광
프레임이 통과한다 — 실제로 그렇게 두 개를 잘못 골랐다. 카메라가 바닥에서
25cm 라 대부분의 프레임은 바닥과 옆벽만 본다. 원근선 개수의 중앙값이 0~1 인
게 그 뜻이다. 복도가 뻗어 보이는 프레임 자체가 소수다.

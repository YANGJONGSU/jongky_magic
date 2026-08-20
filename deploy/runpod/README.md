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
**아무것도 새로 만들 필요가 없다.**

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
`setup_cosmos.sh` 는 설치 전에 현재 버전을 constraints 로 못박고, 설치 후에
**실제로 안 바뀌었는지 다시 확인한다.**

## 클립 생성

```bash
python3 deploy/runpod/gen_clip.py --seed-video deploy/runpod/seeds/corridor_10f.mp4 \
  --prompt-file deploy/runpod/seeds/prompt_v1.txt
```

한 번에 121프레임(약 5초)이다. 20분짜리를 넣어 20분을 받는 물건이 아니다.
범위는 길이가 아니라 **씨앗 지점 수**로 넓힌다 — 길게 뽑을수록 복도 폭과
카메라 높이가 흘러가고, 그러면 depth 와 액션의 관계가 깨져서 학습 신호가
거짓이 된다.

#!/usr/bin/env bash
# Cosmos1GP 설치 — runpod/pytorch:2.2.0-py3.10-cuda12.1.1-devel-ubuntu22.04 전용
#
#   bash setup_cosmos.sh
#
# ## 이 템플릿을 쓰는 이유
#
# Cosmos1GP 는 python 3.10 을 전제한다 (mmgp 3.1.2 가 `>=3.10`,
# transformers 4.45 조합). 이 이미지는 시스템 파이썬이 이미 3.10 이고
# torch 2.2.0+cu121 이 들어 있다. mmgp 요구가 `torch>=2.1.0` 이니 그대로 맞는다.
#
# 이전 판은 python 3.12 인 24.04 이미지를 골랐다. 그래서 uv 를 깔고 3.10 venv 를
# 새로 만들고 torch 를 직접 골라 넣어야 했고, 그 사슬에서 나온 실패가
# `uv not found` → `setuptools AssertionError` → 휠 불일치였다. 전부 첫 단추
# 하나에서 나왔다. 이 판은 그 사슬 자체를 없앤다.
#
# ## 규칙 하나: 플랫폼이 넣어둔 torch 를 갈아치우지 않는다
#
# 이미지의 torch 는 RunPod 이 자기 드라이버에 맞춰 넣은 것이다. 내가 다시 고르면
# 그 보증을 버리는 것이고, 지난 파드에서 그렇게 해서 하루를 날렸다.
# 그래서 아래 pip 설치는 전부 constraints 로 torch 를 못 건드리게 묶는다 —
# transformers·peft·optimum-quanto 중 하나가 의존성으로 torch 를 올려버리면
# 같은 자리로 돌아간다.
set -u

ROOT="${ROOT:-/workspace}"
REPO="$ROOT/Cosmos1GP"
CKPT="$REPO/checkpoints"
MODEL="${MODEL:-7B}"          # 7B 는 16GB 급에서도 돈다. A100/H100 이면 14B.
PY=python3
export HF_HOME="${HF_HOME:-$ROOT/.hf}"      # 캐시도 볼륨 안에 둬야 재배포에 안 날아간다
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$ROOT/.pip_cache}"
export DEBIAN_FRONTEND=noninteractive
CONS="$ROOT/torch_constraints.txt"

say(){ echo "[$(date +%H:%M:%S)] $*"; }
die(){ echo; echo "!! $*"; exit 1; }

mkdir -p "$ROOT" "$HF_HOME" "$PIP_CACHE_DIR"
say "ROOT=$ROOT MODEL=$MODEL"

# ── 1. 사전점검 ─────────────────────────────────────────────────────────────
# 체크포인트(int8 로 약 20GB)를 받기 **전에** GPU 가 실제로 컨텍스트를 만들 수
# 있는지 본다. 지난 파드는 순서가 거꾸로여서, 다 받은 뒤에야 파드 안에서는
# 못 고치는 결함이라는 걸 알았다. 여기서 막히면 잃는 게 없다.
if [ "${SKIP_PREFLIGHT:-0}" != "1" ]; then
  bash "$(dirname "$0")/preflight.sh" \
    || die "사전점검 실패. 아무것도 안 받았다. 파드를 Terminate 하고 다시 Deploy 할 것."
fi

# ── 2. 템플릿 확인 ──────────────────────────────────────────────────────────
# 다른 이미지로 이 스크립트를 돌리면 조용히 이상하게 깔린다. 먼저 세운다.
PYVER="$("$PY" -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
say "python $PYVER"
[ "$PYVER" = "3.10" ] || die "python 3.10 이 아니다 ($PYVER).
   템플릿을 runpod/pytorch:2.2.0-py3.10-cuda12.1.1-devel-ubuntu22.04 로 띄울 것."

# ── 3. GPU 세대 확인 (torch 를 쓰지 않는다) ──────────────────────────────────
# 여기서 torch 로 검사하면 안 된다. 이 단계는 4단계가 torch 를 고치기 **전**에
# 도는데, 앞선 실행이 torch 를 망가뜨려 놨으면 3단계가 먼저 죽어서 스크립트가
# 자기 자신을 고칠 수 없게 된다. 실제로 그렇게 막혔다 — 재실행이 안 되는
# 검사는 검사가 아니라 함정이다.
#
# compute capability 는 드라이버에서 바로 읽을 수 있다. 파이썬 패키지가 필요 없다.
say "GPU 세대 확인"
"$PY" - <<'PYEOF' || die "GPU 세대를 못 읽거나 이 템플릿으로 못 쓰는 GPU 다"
import ctypes, sys
cu = ctypes.CDLL("libcuda.so.1")
if cu.cuInit(0):
    sys.exit("  !! cuInit 실패 — preflight 를 먼저 볼 것")
dev = ctypes.c_int()
if cu.cuDeviceGet(ctypes.byref(dev), 0):
    sys.exit("  !! cuDeviceGet 실패")
maj, mnr = ctypes.c_int(), ctypes.c_int()
# CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR=75, MINOR=76
cu.cuDeviceGetAttribute(ctypes.byref(maj), 75, dev)
cu.cuDeviceGetAttribute(ctypes.byref(mnr), 76, dev)
name = ctypes.create_string_buffer(128)
cu.cuDeviceGetName(name, 128, dev)
print("  %s · sm_%d%d" % (name.value.decode(), maj.value, mnr.value))
# cu121 로 빌드된 torch 는 sm_90 까지다. 5090(sm_120) 급은 GPU 를 아예 못 본다.
if maj.value >= 10:
    sys.exit("  !! sm_%d%d 는 이 템플릿의 cu121 torch 가 지원하지 않는다.\n"
             "     GPU 를 4090/A40/A100/L40S 급으로 바꿀 것." % (maj.value, mnr.value))
PYEOF

# ── 4. torch/numpy 를 알려진 조합으로 맞춘다 ────────────────────────────────
# mmgp 3.1.2 는 메타데이터에 `torch>=2.1.0` 이라고 적어놨는데 그게 틀렸다.
# 실제로는 훨씬 높다. `safetensors2.py` 가 `torch.uint16`(2.3 에서 생김) 을,
# `offload.py` 가 `torch.nn.Buffer`(2.5 에서 생김) 를 쓴다. 선언만 보고 고르면
# 이런 게 실행 도중에 하나씩 터진다 — 실제로 두 번 그렇게 터졌다.
#
# 그래서 버전을 **배포 시점**으로 맞춘다. mmgp 3.1.2 는 2025-01-21 배포이고
# 그때 최신 torch 가 2.5.1 이었다 (2.6.0 은 8일 뒤). 그게 이 코드가 실제로
# 개발된 환경이다.
#
# 이건 "플랫폼 torch 를 갈아치우지 않는다" 와 어긋나 보이지만 아니다. 그 규칙의
# 이유는 드라이버 정합이었고 여기서는 CUDA 계열을 안 바꾼다: 이미지가 cu121,
# 드라이버가 12.8 까지 받으니 cu121 휠은 그대로 유효하다. 파이썬 쪽만 올린다.
#
# numpy 도 같이 묶는다. 이 torch 는 numpy 1.x 로 빌드됐는데 의존성으로 numpy 2.2
# 가 딸려 들어와 `_ARRAY_API not found` 가 났다. 경고처럼 보이지만 그 상태의
# torch 는 numpy 변환이 통째로 죽어 있다.
TORCH_V="${TORCH_V:-2.5.1}"; TV_V="${TV_V:-0.20.1}"; TA_V="${TA_V:-2.5.1}"
CU_IDX="https://download.pytorch.org/whl/cu121"

# 드라이버가 cu121 을 받아주는지부터 본다. 12.1 미만이면 이 조합을 못 쓴다.
DMAX="$(nvidia-smi 2>/dev/null | grep -o 'CUDA Version: [0-9.]*' | head -1 | awk '{print $3}')"
awk -v d="${DMAX:-0}" 'BEGIN{split(d,a,".");exit !(a[1]>12||(a[1]==12&&a[2]>=1))}' \
  || die "드라이버가 받아주는 CUDA 가 ${DMAX:-?} 라 cu121 을 쓸 수 없다. 파드를 바꿀 것."

say "torch $TORCH_V (cu121) 설치"
"$PY" -m pip install --no-input -q \
  "torch==$TORCH_V" "torchvision==$TV_V" "torchaudio==$TA_V" --index-url "$CU_IDX" \
  || die "torch 설치 실패"

# ── 4b. 제약 파일을 **다른 설치보다 먼저** 만든다 ──────────────────────────
# 순서가 중요하다. 앞서 numpy/opencv/quanto 를 제약 없이 깔았다가 quanto 가
# 의존성으로 torch 를 2.13 까지 끌어올렸고, 그 판은 이 드라이버(CUDA 12.8)로는
# 안 돈다. 제약을 쓸 거면 **첫 설치부터** 걸어야 한다.
#
# · torch 3종  : 방금 넣은 그 버전 그대로
# · numpy<2    : 이 torch 는 numpy 1.x 로 빌드됐다. 2.x 면 `_ARRAY_API not found`
# · opencv<5   : 5.0 은 numpy>=2 로 빌드돼서 배열을 주고받을 때 깨진다
# · quanto 0.2.6: 0.2.7 은 `torch>=2.6.0` 이라 2.4.1 과 같이 못 쓴다.
#                 그리고 파드에 깔려 있던 판은 state_dict 에서
#                 `weight_qtype` 을 pop 하는데 이 체크포인트에는 그 키가 없다
#                 (`weight._data` I8 + `weight._scale` F32 형식).
#                 pypi 의 0.2.4~0.2.7 중 그렇게 하는 판은 없다 — dist-info 는
#                 0.2.7 이라고 적혀 있는데 파일 내용이 달랐다. 그래서 버전
#                 표기를 믿지 않고 --force-reinstall 로 파일을 덮어쓴다.
"$PY" - > "$CONS" <<'PYEOF'
import importlib.metadata as md
for p in ("torch", "torchvision", "torchaudio"):
    print("%s==%s" % (p, md.version(p)))
print("numpy<2")
print("opencv-python<5")
print("optimum-quanto==0.2.6")
PYEOF
say "고정:"; sed 's/^/    /' "$CONS"
PIP=( "$PY" -m pip install --no-input -c "$CONS" )

# ── 4c. 나머지를 제약 아래에서 ──────────────────────────────────────────────
"${PIP[@]}" -q "numpy<2" "opencv-python<5" || die "numpy/opencv 설치 실패"
# quanto 는 --force-reinstall 로 파일을 갈아엎는다. 버전 표기가 맞는데 내용이
# 다른 상태였기 때문에, 보통 설치는 "이미 0.2.x 있음" 으로 건너뛴다.
# --no-deps 를 붙이는 이유는 이 호출이 torch 를 다시 건드리지 못하게 하기 위함이다.
"$PY" -m pip install --no-input -q --force-reinstall --no-deps \
  "optimum-quanto==0.2.6" || die "quanto 설치 실패"

# 이 조합이 실제로 쓸 수 있는지: 할당 + mmgp 가 필요로 하는 dtype
"$PY" - <<'PYEOF' || die "고정한 조합이 GPU 에서 안 돈다 — 위 traceback 을 그대로 가져올 것"
import torch, numpy, inspect
import importlib.metadata as md
assert hasattr(torch, "uint16"), "torch.uint16 이 없다 — mmgp 가 이걸 쓴다"
assert numpy.__version__.startswith("1."), "numpy 가 2.x 다: " + numpy.__version__
# 버전 문자열이 아니라 **실제 코드**를 본다. 파드에 0.2.7 이라고 적혀 있으면서
# 내용이 다른 판이 깔려 있었다.
import optimum.quanto.nn.qmodule as _q
src = inspect.getsource(_q.QModuleMixin._load_from_state_dict)
bad = 'state_dict.pop(prefix + "weight_qtype")'
print("  optimum-quanto", md.version("optimum-quanto"),
      "· weight_qtype pop:", bad in src)
assert bad not in src, (
    "이 quanto 판은 state_dict 에서 weight_qtype 을 요구한다. "
    "체크포인트에는 그 키가 없다.")
x = torch.randn(2000, 2000, device="cuda")
print("  torch", torch.__version__, "· numpy", numpy.__version__, "· 할당 OK")

# mmgp 가 참조하는 torch 속성이 이 버전에 다 있는지 미리 훑는다.
# 없는 게 있으면 실행 중에 AttributeError 로 터지는데, 그게 모델을 다 읽은
# 뒤라서 매번 몇 분씩 날아간다. 여기서 목록으로 한 번에 본다.
import os, re, importlib.util
_spec = importlib.util.find_spec("mmgp")
if _spec and _spec.submodule_search_locations:
    _d = list(_spec.submodule_search_locations)[0]
    _seen, _missing = set(), []
    _pat = re.compile(r"\btorch\.([A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)?)")
    for _f in os.listdir(_d):
        if not _f.endswith(".py"):
            continue
        for _m in _pat.finditer(open(os.path.join(_d, _f), encoding="utf-8").read()):
            _seen.add(_m.group(1))
    for _a in sorted(_seen):
        _o = torch
        for _part in _a.split("."):
            _o = getattr(_o, _part, None)
            if _o is None:
                _missing.append(_a); break
    if _missing:
        print("  경고: 이 torch 에 없는 속성을 mmgp 가 참조한다 →", ", ".join(_missing))
        print("        실행 중에 AttributeError 로 터질 수 있다.")
    else:
        print("  ok mmgp 가 쓰는 torch 속성 %d개 전부 존재" % len(_seen))
PYEOF

# ── 5. 시스템 패키지 ────────────────────────────────────────────────────────
# opencv 는 libGL 이, imageio[ffmpeg] 는 ffmpeg 가 없으면 import 에서 죽는다.
# 둘 다 실행 직전이 아니라 import 시점에 터져서 원인이 안 보인다.
say "apt 패키지"
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq git ffmpeg libgl1 libglib2.0-0 >/dev/null 2>&1 \
  || say "경고: apt 일부 실패 — cv2/imageio import 에서 걸리면 여기를 볼 것"

# ── 6. 저장소 ───────────────────────────────────────────────────────────────
if [ -d "$REPO/.git" ]; then
  say "Cosmos1GP 이미 있음 — 갱신"
  git -C "$REPO" pull --ff-only >/dev/null 2>&1 || say "  (pull 생략)"
else
  say "Cosmos1GP 클론"
  git clone --depth 1 https://github.com/deepbeepmeep/Cosmos1GP "$REPO" || die "클론 실패"
fi

# ── 7. 파이썬 의존성 ────────────────────────────────────────────────────────
say "의존성 설치 (5~10분)"
"${PIP[@]}" -q -r "$REPO/requirements.txt" || die "requirements 설치 실패"
# requirements 에 없지만 서버가 쓰는 것들
"${PIP[@]}" -q "huggingface_hub[hf_transfer]" || die "huggingface_hub 설치 실패"

# ── 8. torch 가 그대로인지 재확인 ────────────────────────────────────────────
# constraints 를 걸었어도 실제로 지켜졌는지 본다. 이 검사가 없으면 며칠 뒤에
# "왜 또 안 되지" 로 돌아온다.
say "설치 후 torch 재확인"
"$PY" - <<'PYEOF' || die "의존성 설치가 torch 나 numpy 를 건드렸다. constraints 가 안 먹었다."
import torch, numpy
assert hasattr(torch, "uint16")
assert numpy.__version__.startswith("1."), "numpy " + numpy.__version__
x = torch.randn(2000, 2000, device="cuda")
print("  torch", torch.__version__, "· numpy", numpy.__version__, "· 할당 OK")
PYEOF

# ── 9. 체크포인트 ───────────────────────────────────────────────────────────
# int8 판만 받는다: T5XXL 11B 약 11GB + transformer 약 7GB + 토크나이저 약 1GB.
if [ "$MODEL" = "14B" ]; then TF="cosmo1_14B_video2world_quanto_int8.safetensors"
else                          TF="cosmo1_7B_video2world_quanto_int8.safetensors"; fi
say "체크포인트: $TF"
export HF_HUB_ENABLE_HF_TRANSFER=1
"$PY" - <<PYEOF || die "체크포인트 다운로드 실패"
import os
from huggingface_hub import hf_hub_download, snapshot_download
repo, root = "DeepBeepMeep/Cosmos1GP", "$CKPT"
if not os.path.isdir(os.path.join(root, "Cosmos-1.0-Tokenizer-CV8x8x8")):
    snapshot_download(repo_id=repo, allow_patterns="Cosmos-1.0-Tokenizer-CV8x8x8/*", local_dir=root)
for sub, files in (("text_encoder", ["config.json", "spiece.model", "tokenizer.json",
                                     "T5XXLEncoder_11B_quanto_int8.safetensors"]),
                   ("transformer",  ["$TF"])):
    for f in files:
        if not os.path.isfile(os.path.join(root, sub, f)):
            print("받는 중:", sub, f, flush=True)
            hf_hub_download(repo_id=repo, filename=f, subfolder=sub, local_dir=root)
print("체크포인트 준비 완료")
PYEOF

# ── 10. 서버 설정 ───────────────────────────────────────────────────────────
# profile 1 은 오프로딩 없이 돌아 제일 빠르지만 VRAM 24GB + RAM 45GB 를 요구한다.
# 둘 중 하나라도 모자라면 2(RAM 오프로딩)로 간다.
VRAM_MB="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1)"
RAM_GB="$(free -g | awk '/^Mem:/{print $2}')"
if [ "${VRAM_MB:-0}" -ge 24000 ] && [ "${RAM_GB:-0}" -ge 45 ]; then PROFILE=1; else PROFILE=2; fi
say "VRAM ${VRAM_MB}MiB · RAM ${RAM_GB}GB → profile $PROFILE"
cat > "$REPO/gradio_config_v2w.json" <<JEOF
{"attention_mode": "sdpa", "transformer_filename": "$TF", "text_encoder_filename": "T5XXLEncoder_11B_quanto_int8.safetensors", "compile": "", "profile": $PROFILE}
JEOF

# ── 11. import 사슬 검증 ────────────────────────────────────────────────────
# 서버를 띄우고 5분 기다렸다가 import 하나 때문에 죽는 걸 여기서 먼저 잡는다.
#
# 껍데기만 import 하면 안 된다. 앞서 `import mmgp` 는 통과했는데 서버는
# `from mmgp import offload` 에서 죽었다 — offload.py 가 safetensors2 를 끌고
# 들어가고 거기서 torch.uint16 을 만지는데, 패키지 __init__ 은 그걸 안 건드린다.
# 그래서 **서버 파일이 실제로 쓰는 심볼을** 그대로 불러본다.
say "import 검증"
CKPT_DIR="$CKPT" T5_NAME="T5XXLEncoder_11B_quanto_int8.safetensors" \
"$PY" - <<'PYEOF' || die "import 검증 실패 — 위 traceback 을 그대로 가져올 것"
import importlib
for m in ("torch", "numpy", "transformers", "optimum.quanto", "gradio",
          "cv2", "imageio", "sentencepiece", "peft", "einops"):
    importlib.import_module(m)
    print("  ok", m)
# gradio_server_v2w.py 17번 줄과 같은 import
from mmgp import offload, profile_type
print("  ok mmgp.offload / profile_type")

# import 만으로는 부족하다. numpy ABI 가 어긋난 확장 모듈은 import 는 통과하고
# 배열을 실제로 주고받을 때 죽는다. 그래서 왕복을 시켜본다.
import numpy as np, cv2, torch
a = np.zeros((8, 8, 3), np.uint8)
assert cv2.cvtColor(a, cv2.COLOR_RGB2BGR).shape == (8, 8, 3)
assert torch.from_numpy(a).numpy().shape == (8, 8, 3)
print("  ok numpy 왕복 (cv2 · torch)")

# T5 인코더를 실제로 읽어본다. 두 번 다 여기서 죽었는데 두 번 다 import
# 검사는 통과했다 — 양자화 형식이 안 맞는 건 파일을 실제로 읽어야만 드러난다.
# 30초쯤 걸리지만, 서버를 띄우고 5분 기다렸다가 같은 자리에서 죽는 것보다 낫다.
import os
_p = os.path.join(os.environ.get("CKPT_DIR", "checkpoints"),
                  "text_encoder", os.environ["T5_NAME"])
from mmgp import offload as _off
_off.fast_load_transformers_model(_p)
print("  ok T5 인코더 실제 로딩")
PYEOF

echo
say "설치 완료"
echo "   다음:  bash $(dirname "$0")/serve.sh"

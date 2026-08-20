#!/usr/bin/env bash
# RunPod 에 Cosmos1GP(Video2World) 를 세운다. Ubuntu 24.04 컨테이너 기준.
#
#   nohup bash setup_cosmos.sh > /workspace/setup_cosmos.log 2>&1 &
#   tail -f /workspace/setup_cosmos.log
#
# 왜 스크립트인가 — 파드는 껐다 켜면 /workspace 밖이 전부 날아간다. 손으로 깔면
# 매번 30분씩 다시 깐다. 전부 /workspace 안에 두고 한 번에 재현되게 한다.
#
# 24.04 에서 걸리는 것 셋
#   1. 기본 파이썬이 3.12 인데 Cosmos1GP 는 3.10 을 전제한다 (transformers 4.45,
#      mmgp 3.1.2 조합). uv 로 3.10 venv 를 따로 만든다.
#   2. PEP 668 때문에 시스템 pip 설치가 막힌다. venv 밖에서 pip 하지 말 것.
#   3. torch 는 GPU 세대에 맞는 CUDA 빌드를 골라야 한다. Blackwell(RTX 5090,
#      sm_120)은 cu128 이상이 필요하고 cu124 는 GPU 를 아예 못 본다.
#      아래에서 실제 compute capability 를 읽어 자동으로 고른다.
set -u

ROOT="${ROOT:-/workspace}"
REPO="$ROOT/Cosmos1GP"
VENV="$ROOT/cosmos_venv310"
CKPT="$REPO/checkpoints"
# 7B 는 16GB 급에서도 돌고, 14B 는 40GB 이상이 편하다. A100/H100 이면 14B 를 권한다.
MODEL="${MODEL:-7B}"
export HF_HOME="${HF_HOME:-$ROOT/.hf}"          # 캐시도 볼륨 안에 둬야 재부팅에 안 날아간다
export UV_CACHE_DIR="${UV_CACHE_DIR:-$ROOT/.uv_cache}"
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-900}"
export DEBIAN_FRONTEND=noninteractive

say(){ echo "[$(date +%H:%M:%S)] $*"; }
have(){ command -v "$1" >/dev/null 2>&1; }

say "ROOT=$ROOT MODEL=$MODEL"
mkdir -p "$ROOT" "$HF_HOME" "$UV_CACHE_DIR"

# ── 1. 시스템 패키지 ────────────────────────────────────────────────────────
# ffmpeg 는 씨앗 영상을 읽고 결과를 쓰는 데 둘 다 필요하다. libgl/libglib 은
# opencv-python 이 import 만 해도 요구한다 — 없으면 런타임에야 터진다.
say "시스템 패키지"
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
  git git-lfs wget curl ca-certificates ffmpeg \
  libgl1 libglib2.0-0 build-essential python3-dev \
  || { say "!! apt 실패"; exit 1; }
git lfs install --skip-repo >/dev/null 2>&1 || true

# ── 2. uv ───────────────────────────────────────────────────────────────────
if ! have uv; then
  say "uv 설치"
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
  # 새 셸에서도 찾게 한다. 설치 후 따로 붙었을 때 uv: command not found 가 난다
  grep -q 'HOME/.local/bin' "$HOME/.bashrc" 2>/dev/null || \
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
fi
have uv || { say "!! uv 를 못 찾겠다. PATH=$PATH"; exit 1; }

# ── 3. 저장소 ───────────────────────────────────────────────────────────────
if [ ! -d "$REPO/.git" ]; then
  say "Cosmos1GP 클론"
  git clone --depth 1 https://github.com/deepbeepmeep/Cosmos1GP.git "$REPO" || exit 1
else
  say "저장소 이미 있음 — 건너뜀"
fi

# ── 4. 파이썬 3.10 venv ─────────────────────────────────────────────────────
if [ ! -x "$VENV/bin/python" ]; then
  say "python 3.10 venv 생성"
  uv venv --python 3.10 "$VENV" || exit 1
fi
PY="$VENV/bin/python"
PIP=( uv pip install --python "$PY" )

# uv 로 만든 venv 에는 setuptools 가 없다 (python -m venv 와 다르다).
# optimum-quanto 가 torch.utils.cpp_extension 을 통해 import 하므로, 없으면
# 서버가 뜨는 순간 ModuleNotFoundError 로 죽는다 — 설치 단계에서는 아무 티도 안 난다.
say "setuptools / wheel"
"${PIP[@]}" setuptools wheel || exit 1

# ── 5. torch — GPU 세대에 맞는 CUDA 빌드 ────────────────────────────────────
# compute capability 로 고른다. 12.x = Blackwell → cu128, 그 아래는 cu124 로 충분.
# 빈 문자열이면 GPU 가 하나도 안 보인다 (unset 과 다르다). RunPod 컨테이너가
# 이렇게 내보내는 경우가 있고, 그러면 torch 가 "device busy or unavailable" 로
# 죽는데 nvidia-smi 는 멀쩡해 보여서 원인을 엉뚱한 데서 찾게 된다.
if [ -z "${CUDA_VISIBLE_DEVICES:-x}" ]; then
  say "CUDA_VISIBLE_DEVICES 가 빈 문자열 — 0 으로 바로잡고 .bashrc 에도 남긴다"
  export CUDA_VISIBLE_DEVICES=0
  grep -q 'CUDA_VISIBLE_DEVICES' "$HOME/.bashrc" 2>/dev/null || \
    echo 'export CUDA_VISIBLE_DEVICES=0' >> "$HOME/.bashrc"
fi

CAP="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ')"
GPUNAME="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
MAJOR="${CAP%%.*}"
if [ -z "$CAP" ]; then
  say "!! nvidia-smi 가 GPU 를 못 본다. --gpus all 로 띄운 파드가 맞는지 확인할 것"; exit 1
fi
if [ "${MAJOR:-0}" -ge 12 ]; then IDX="https://download.pytorch.org/whl/cu128"; TAG="cu128"
else                                IDX="https://download.pytorch.org/whl/cu124"; TAG="cu124"; fi
say "GPU=$GPUNAME cap=$CAP → torch $TAG"

if ! "$PY" -c "import torch" 2>/dev/null; then
  say "torch 설치 ($TAG)"
  "${PIP[@]}" torch torchvision torchaudio --index-url "$IDX" || exit 1
fi

# ── 6. 나머지 의존성 ────────────────────────────────────────────────────────
# requirements.txt 에 git+ 소스 빌드(Pytorch_Retinaface)가 하나 있어서 네트워크가
# 흔들리면 실패한다. 3번 재시도한다.
say "의존성 설치"
for i in 1 2 3; do
  "${PIP[@]}" -r "$REPO/requirements.txt" && break
  say "  실패 — 재시도 $i/3"; sleep 20
done

# sage attention 은 30% 빠르고 리눅스에서는 쉽게 붙는다. 실패해도 진행한다
# (gradio_server 기본이 sdpa 라 없어도 돈다).
"${PIP[@]}" sageattention==1.0.6 >/dev/null 2>&1 && say "sageattention OK" || say "sageattention 건너뜀 (sdpa 로 돈다)"
"${PIP[@]}" "huggingface_hub[hf_transfer]" >/dev/null 2>&1 || true

# ── 7. 체크포인트 ───────────────────────────────────────────────────────────
# 텍스트 인코더(T5XXL 11B int8, 약 5GB)는 t2w/v2w 공용이다.
# transformer 는 v2w 만 받는다 — t2w 는 우리 용도에 안 쓴다.
if [ "$MODEL" = "14B" ]; then TF="cosmo1_14B_video2world_quanto_int8.safetensors"
else                          TF="cosmo1_7B_video2world_quanto_int8.safetensors"; fi
say "체크포인트: $TF"
export HF_HUB_ENABLE_HF_TRANSFER=1
"$PY" - <<PYEOF || { say "!! 체크포인트 다운로드 실패"; exit 1; }
import os
from huggingface_hub import hf_hub_download, snapshot_download
repo, root = "DeepBeepMeep/Cosmos1GP", "$CKPT"
if not os.path.isdir(os.path.join(root, "Cosmos-1.0-Tokenizer-CV8x8x8")):
    snapshot_download(repo_id=repo, allow_patterns="Cosmos-1.0-Tokenizer-CV8x8x8/*", local_dir=root)
for sub, files in (("text_encoder", ["config.json","spiece.model","tokenizer.json",
                                     "T5XXLEncoder_11B_quanto_int8.safetensors"]),
                   ("transformer",  ["$TF"])):
    for f in files:
        if not os.path.isfile(os.path.join(root, sub, f)):
            print("받는 중:", sub, f, flush=True)
            hf_hub_download(repo_id=repo, filename=f, subfolder=sub, local_dir=root)
print("체크포인트 준비 완료")
PYEOF

# ── 8. 서버 설정 ────────────────────────────────────────────────────────────
# transformer_choices 순서: [14B, 14B_int8, 7B, 7B_int8] → int8 을 쓴다.
# profile 은 VRAM 이 넉넉하면 1(가장 빠름), 아니면 2(RAM 으로 오프로딩).
VRAM_MB="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1)"
# profile 1 은 오프로딩 없이 돌아 가장 빠르지만 VRAM 24GB + RAM 48GB 를 요구한다.
# 둘 중 하나라도 모자라면 2(RAM 으로 오프로딩)로 간다.
RAM_GB="$(free -g | awk '/^Mem:/{print $2}')"
if [ "${VRAM_MB:-0}" -ge 24000 ] && [ "${RAM_GB:-0}" -ge 45 ]; then PROFILE=1; else PROFILE=2; fi
say "VRAM ${VRAM_MB}MiB · RAM ${RAM_GB}GB → profile $PROFILE"
cat > "$REPO/gradio_config_v2w.json" <<JEOF
{"attention_mode": "sdpa", "transformer_filename": "$TF", "text_encoder_filename": "T5XXLEncoder_11B_quanto_int8.safetensors", "compile": "", "profile": $PROFILE}
JEOF

mkdir -p "$REPO/seeds" "$REPO/outputs"

# ── 9. 검증 ─────────────────────────────────────────────────────────────────
say "검증"
"$PY" - <<'PYEOF'
import torch
print("  torch", torch.__version__, "cuda", torch.version.cuda)
print("  arch_list", torch.cuda.get_arch_list())
assert torch.cuda.is_available(), "CUDA 를 못 본다"
cap = torch.cuda.get_device_capability(0)
print("  device", torch.cuda.get_device_name(0), "cap", cap)
# CUDA 는 같은 major 세대 안에서 바이너리 호환이다 — sm_86 큐빈이 sm_89(Ada)
# 에서 그대로 돈다. RTX 4090/RTX 5000 Ada 가 sm_86 까지만 담긴 표준 휠로 도는
# 이유가 이것이다. 정확히 일치하는지 보면 멀쩡한 설치를 실패로 판정한다.
arch = torch.cuda.get_arch_list()
same_major = [a for a in arch if a.startswith("sm_%d" % cap[0])]
assert same_major or ("sm_%d%d" % cap) in arch, (
    "이 torch 빌드에 sm_%d.x 계열이 없다 (%s) — CUDA 빌드를 바꿔야 한다"
    % (cap[0], ", ".join(arch)))
a = torch.randn(4000, 4000, device="cuda"); (a @ a).sum().item()
print("  실연산 OK")

# torch 만 확인하면 부족하다. 서버는 optimum.quanto 를 통해
# torch.utils.cpp_extension -> setuptools 로 들어가는데, uv venv 에는
# setuptools 가 없어서 여기서만 죽는다. 설치 중에는 아무 티도 안 난다.
import importlib
for m in ("setuptools", "optimum.quanto", "gradio", "mmgp", "cv2", "imageio"):
    importlib.import_module(m)
    print("  import %s OK" % m)
PYEOF
[ $? -eq 0 ] || { say "!! 검증 실패"; exit 1; }

cat <<MSG

────────────────────────────────────────────────────────────
설치 완료.

  서버 띄우기 — 한 줄이면 된다. 기동·대기·판정을 다 한다:

    bash $REPO/../jongky_magic/deploy/runpod/serve.sh

  정지 / 로그:

    bash .../serve.sh stop
    bash .../serve.sh log

  씨앗 영상은 $REPO/seeds/ 에 올리고,
  생성은 gen_clip.py 로 한다 (같은 폴더의 스크립트 참조).
────────────────────────────────────────────────────────────
MSG
say "끝"

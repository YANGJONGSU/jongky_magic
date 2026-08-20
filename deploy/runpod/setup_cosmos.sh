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

"$PY" -c 'import torch' 2>/dev/null \
  || die "이미지에 torch 가 없다. 템플릿이 잘못됐다."

# ── 3. 이미지의 torch 가 실제로 할당되는지 ────────────────────────────────────
# is_available() 로는 부족하다. 지난 파드가 True 를 내면서 할당에서 죽었다.
say "torch 실할당 확인"
"$PY" - <<'PYEOF' || die "이미지의 torch 로 GPU 할당이 안 된다. 사전점검이 통과했는데 여기서 죽으면 torch 쪽이다 — 이 출력을 그대로 가져올 것."
import torch
print("  torch", torch.__version__, "· cuda", torch.version.cuda)
cap = torch.cuda.get_device_capability(0)
print("  GPU", torch.cuda.get_device_name(0), "· sm_%d%d" % cap)
# cu121 로 빌드된 torch 2.2 는 sm_90 까지다. 5090(sm_120) 급에서는 GPU 를 아예
# 못 본다. 그건 이 템플릿으로 해결되지 않으니 미리 세운다.
if cap[0] >= 10:
    raise SystemExit(
        "  !! sm_%d%d 는 이 템플릿의 cu121 torch 가 지원하지 않는다.\n"
        "     GPU 를 4090/A100/L40S 급으로 바꾸거나 다른 템플릿을 써야 한다." % cap)
x = torch.randn(4000, 4000, device="cuda")
print("  할당·실연산 OK", float((x @ x).sum()))
PYEOF

# ── 4. torch 를 못 건드리게 못박는다 ─────────────────────────────────────────
# 아래 requirements 안의 peft / transformers / optimum-quanto 는 torch 를
# 의존성으로 갖는다. 제약을 안 걸면 pip 가 조용히 올려버리고, 그러면 드라이버와
# 다시 어긋난다. 지금 깔려 있는 정확한 버전으로 고정한다.
"$PY" - > "$CONS" <<'PYEOF'
import importlib.metadata as md
for p in ("torch", "torchvision", "torchaudio"):
    try: print("%s==%s" % (p, md.version(p)))
    except md.PackageNotFoundError: pass
PYEOF
say "torch 고정:"; sed 's/^/    /' "$CONS"
PIP=( "$PY" -m pip install --no-input -c "$CONS" )

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
"$PY" - <<'PYEOF' || die "의존성 설치가 torch 를 건드렸다. constraints 가 안 먹었다."
import torch
x = torch.randn(2000, 2000, device="cuda")
print("  torch", torch.__version__, "· 할당 OK")
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
say "import 검증"
"$PY" - <<'PYEOF' || die "import 검증 실패 — 위 traceback 을 그대로 가져올 것"
import importlib
for m in ("torch", "transformers", "optimum.quanto", "mmgp", "gradio",
          "cv2", "imageio", "sentencepiece", "peft", "einops"):
    importlib.import_module(m)
    print("  ok", m)
PYEOF

echo
say "설치 완료"
echo "   다음:  bash $(dirname "$0")/serve.sh"

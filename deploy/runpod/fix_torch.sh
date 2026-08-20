#!/usr/bin/env bash
# 드라이버가 지원하는 CUDA 에 맞는 torch 로 갈아끼운다.
#
#   bash /workspace/jongky_magic/deploy/runpod/fix_torch.sh
#
# 왜 필요한가 — 휠을 GPU 세대로 고르면 안 된다. sm_89 는 cu124 로 빌드된
# 커널을 돌릴 수 있지만, 그건 **드라이버가 CUDA 12.4 런타임을 받아줄 때** 얘기다.
# 이 파드는 드라이버 590(CUDA 13.1)에 torch 는 cu124 였고, 그 조합에서
# `torch.cuda.is_available()` 은 True 를 내면서 실제 할당은
# "CUDA-capable device(s) is/are busy or unavailable" 로 죽었다.
# 열거는 되는데 컨텍스트 생성이 안 되는 상태라, 증상만 보면 GPU 를 누가
# 점유한 것처럼 보인다 — nvidia-smi 는 0% / 프로세스 없음으로 나온다.
set -o pipefail

ROOT="${ROOT:-/workspace}"
PY="${PY:-$ROOT/cosmos_venv310/bin/python}"
UV="$(command -v uv || echo "$HOME/.local/bin/uv")"

[ -x "$PY" ] || { echo "!! 파이썬이 없다: $PY"; exit 1; }
[ -x "$UV" ] || { echo "!! uv 가 없다: $UV"; exit 1; }

DRV_CUDA="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
# nvidia-smi 헤더의 "CUDA Version: 13.1" 이 드라이버가 받아주는 최대 런타임이다
MAX="$(nvidia-smi 2>/dev/null | grep -o 'CUDA Version: [0-9.]*' | head -1 | awk '{print $3}')"
echo "드라이버 $DRV_CUDA · 지원 CUDA 최대 ${MAX:-알수없음}"

MAJ="${MAX%%.*}"; MIN="${MAX#*.}"
if   [ "${MAJ:-0}" -ge 13 ]; then IDX="https://download.pytorch.org/whl/cu128"; TAG="cu128"
elif [ "${MAJ:-0}" -eq 12 ] && [ "${MIN:-0}" -ge 8 ]; then IDX="https://download.pytorch.org/whl/cu128"; TAG="cu128"
else IDX="https://download.pytorch.org/whl/cu124"; TAG="cu124"; fi
echo "→ torch $TAG 로 재설치"

"$UV" pip install --python "$PY" --reinstall torch torchvision torchaudio --index-url "$IDX" || exit 1

echo
echo "검증"
"$PY" - <<'PYEOF'
import torch
print("  torch", torch.__version__, "cuda", torch.version.cuda)
print("  available", torch.cuda.is_available())
x = torch.randn(2000, 2000, device="cuda")   # 열거가 아니라 실제 할당
print("  할당 OK", float((x @ x).sum()))
print("  device", torch.cuda.get_device_name(0))
PYEOF
rc=$?
[ $rc -eq 0 ] && echo "끝. serve.sh 로 서버를 띄우면 된다." || echo "!! 여전히 실패 — 드라이버/파드 문제일 수 있다"
exit $rc

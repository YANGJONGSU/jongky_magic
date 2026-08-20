#!/usr/bin/env bash
# 파드를 받자마자 **제일 먼저** 돌린다. 10초 걸린다.
#
#   bash preflight.sh
#
# 왜 제일 먼저인가 — 지난 파드에서 체크포인트 60GB 를 다 받고 venv 를 다 만든
# 뒤에야 GPU 가 컨텍스트를 못 만든다는 걸 알았다. 그건 파드를 바꾸는 것 말고
# 고칠 방법이 없는 종류였다. 순서가 거꾸로였다.
#
# 이 스크립트는 CUDA 스택을 **층별로** 내려가며 어디서 죽는지 짚는다.
# torch 는 안 쓴다 — torch 는 실패 층을 뭉개서 하나의 메시지로 보고한다.
# 드라이버 API 를 직접 부르면 어느 호출에서 무슨 코드가 나오는지 보인다.
set -u

echo "=================================================="
echo " 파드 사전점검"
echo "=================================================="

# --- 1. 장치 노드 -------------------------------------------------------
# nvidia-smi 는 /dev/nvidiactl + /dev/nvidia0 만 있어도 동작한다.
# 그런데 CUDA 컨텍스트 생성에는 /dev/nvidia-uvm 이 필요하다.
# uvm 이 없으면 "GPU 는 멀쩡히 보이는데 할당만 죽는" 정확히 그 증상이 난다.
echo
echo "[1] 장치 노드"
ls /dev/nvidia* 2>/dev/null | sed 's/^/    /' || echo "    (없음)"
UVM=ok
ls /dev/nvidia-uvm >/dev/null 2>&1 || {
  echo "    !! /dev/nvidia-uvm 이 없다 — 만들어 본다"
  # 컨테이너에 권한이 있으면 이걸로 생긴다. 없으면 못 고친다.
  nvidia-modprobe -u -c=0 2>&1 | sed 's/^/       /'
  ls /dev/nvidia-uvm >/dev/null 2>&1 && echo "    -> 생성됨" || { echo "    -> 실패"; UVM=no; }
}

# --- 2. 드라이버 --------------------------------------------------------
echo
echo "[2] 드라이버"
nvidia-smi --query-gpu=name,driver_version,memory.total,compute_mode --format=csv,noheader 2>&1 | sed 's/^/    /'
MAXCUDA="$(nvidia-smi 2>/dev/null | grep -o 'CUDA Version: [0-9.]*' | head -1 | awk '{print $3}')"
echo "    드라이버가 받아주는 CUDA 최대: ${MAXCUDA:-알수없음}"

# --- 3. 드라이버 API 를 층층이 --------------------------------------------
# 여기가 핵심이다. cuInit 은 되는데 cuCtxCreate 가 죽으면 그건 호스트/컨테이너
# 문제이고 파드 안에서 못 고친다. 코드까지 찍어야 판정이 된다.
echo
echo "[3] CUDA 드라이버 API"
python3 - <<'PY'
import ctypes, sys
ERR = {0:"성공", 3:"NOT_INITIALIZED", 46:"DEVICE_UNAVAILABLE",
       100:"NO_DEVICE", 101:"INVALID_DEVICE", 201:"INVALID_CONTEXT",
       802:"SYSTEM_NOT_READY", 803:"SYSTEM_DRIVER_MISMATCH",
       804:"COMPAT_NOT_SUPPORTED_ON_DEVICE", 999:"UNKNOWN"}
def name(r): return ERR.get(r, "코드 %d" % r)
try:
    cu = ctypes.CDLL("libcuda.so.1")
except OSError as e:
    print("    libcuda.so.1 을 못 연다:", e); sys.exit(3)

r = cu.cuInit(0)
print("    cuInit          ->", name(r))
if r: sys.exit(3)

n = ctypes.c_int()
r = cu.cuDeviceGetCount(ctypes.byref(n))
print("    cuDeviceGetCount->", name(r), "· GPU", n.value, "개")
if r or n.value == 0: sys.exit(3)

dev = ctypes.c_int()
r = cu.cuDeviceGet(ctypes.byref(dev), 0)
print("    cuDeviceGet     ->", name(r))
if r: sys.exit(3)

# 여기가 지난번에 죽던 자리다. 열거는 다 되고 컨텍스트만 안 섰다.
ctx = ctypes.c_void_p()
r = cu.cuCtxCreate_v2(ctypes.byref(ctx), 0, dev)
print("    cuCtxCreate     ->", name(r), "   <-- 지난 파드는 여기서 죽었다")
if r:
    print()
    print("    컨텍스트가 안 선다. torch 문제가 아니다.")
    sys.exit(4)
cu.cuCtxDestroy_v2(ctx)
print("    -> 컨텍스트 정상")
PY
CUDA_RC=$?

# --- 판정 ---------------------------------------------------------------
echo
echo "=================================================="
if [ $CUDA_RC -eq 0 ] && [ "$UVM" = ok ]; then
  echo " 통과 — 이 파드 쓸 수 있다"
  echo " 다음:  bash deploy/runpod/setup_cosmos.sh"
  exit 0
fi
echo " 실패 — 이 파드는 버린다"
echo
echo " 파드 안에서 고칠 수 있는 문제가 아니다. RunPod 는 이 상태를"
echo " '그 워커가 망가진 것' 으로 보고, 조치는 종료 후 재배포다."
echo
echo "  1. 파드 Terminate"
echo "  2. 같은 설정으로 새로 Deploy (다른 호스트에 뜬다)"
echo "  3. 다시 이 스크립트부터"
echo
echo " 체크포인트를 안 받은 상태라 잃는 게 없다. 그래서 이걸 먼저 돌린다."
echo "=================================================="
exit 1

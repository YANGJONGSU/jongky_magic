#!/usr/bin/env bash
# Cosmos Video2World 서버를 띄우고 준비될 때까지 기다린다.
#
#   bash /workspace/jongky_magic/deploy/runpod/serve.sh
#   bash .../serve.sh stop
#   bash .../serve.sh log
#
# 짧은 한 줄로 끝나야 하는 이유: 긴 명령을 터미널에 붙여넣으면 줄바꿈에서
# 잘려서 `nohup ... > log` 와 `2>&1 &` 가 따로 실행된다. 그러면 서버가
# 떴는지 안 떴는지도 헷갈린다.
set -o pipefail

ROOT="${ROOT:-/workspace}"
REPO="$ROOT/Cosmos1GP"
PY="${PY:-$ROOT/cosmos_venv310/bin/python}"
LOG="$ROOT/v2w.log"
PORT="${PORT:-7860}"

case "${1:-start}" in
  stop)
    pgrep -f "[g]radio_server_v2w" | xargs -r kill
    sleep 2
    pgrep -f "[g]radio_server_v2w" >/dev/null && echo "아직 살아 있다" || echo "정지됨"
    exit 0 ;;
  log)
    tail -f "$LOG"; exit 0 ;;
esac

# CUDA_VISIBLE_DEVICES 가 "없는 것" 과 "빈 문자열" 은 다르다. 빈 문자열은
# "GPU 를 하나도 보여주지 마라" 는 뜻이고, torch 는 그걸
# "CUDA-capable device(s) is/are busy or unavailable" 로 보고한다.
# nvidia-smi 는 이 변수를 안 보므로 GPU 가 멀쩡히 놀고 있는 것처럼 나온다 —
# 증상이 원인을 정반대로 가리키는 종류다. RunPod 컨테이너가 빈 값으로 내보낸다.
if [ -z "${CUDA_VISIBLE_DEVICES:-x}" ]; then
  echo "CUDA_VISIBLE_DEVICES 가 빈 문자열이다 — 0 으로 바로잡는다"
  export CUDA_VISIBLE_DEVICES=0
fi

[ -x "$PY" ] || { echo "!! 파이썬이 없다: $PY"; exit 1; }
[ -d "$REPO" ] || { echo "!! 저장소가 없다: $REPO"; exit 1; }

if pgrep -f "[g]radio_server_v2w" >/dev/null; then
  echo "이미 떠 있다 (PID $(pgrep -f '[g]radio_server_v2w' | head -1))"
else
  cd "$REPO" || exit 1
  # -u 가 없으면 파이썬이 블록 버퍼링을 해서 "Running on local URL" 이 로그에
  # 안 찍힌다. 서버가 멀쩡히 떠 있어도 안 뜬 것처럼 보인다.
  nohup "$PY" -u gradio_server_v2w.py > "$LOG" 2>&1 < /dev/null &
  echo "기동 (PID $!) · 로그 $LOG"
fi

echo "모델 로딩 대기 — 5~15분 걸린다. 로그가 아니라 포트로 판정한다."
for i in $(seq 1 120); do
  if ss -tln 2>/dev/null | grep -q ":$PORT "; then
    echo
    echo "준비됨 ($((i*10))초) — http://localhost:$PORT"
    grep -E "profile|reserved RAM|Running on" "$LOG" | tail -3
    exit 0
  fi
  if ! pgrep -f "[g]radio_server_v2w" >/dev/null; then
    echo
    echo "!! 서버가 죽었다. 로그 마지막:"
    tail -20 "$LOG"
    exit 1
  fi
  printf '.'
  sleep 10
done
echo
echo "!! 20분이 지나도 포트가 안 열린다"
tail -20 "$LOG"
exit 1

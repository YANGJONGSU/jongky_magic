#!/usr/bin/env bash
# 종키 온보드 컨테이너 실행.
#
#   ./docker/run_robot.sh                    대화형 셸
#   ./docker/run_robot.sh <명령...>          명령 실행 후 종료
#
# 시리얼 장치 처리
#   udev 심링크(/dev/yahboom)는 컨테이너 안에 그대로 넘어가지 않는다.
#   --device 는 심링크가 아니라 실제 장치 노드를 요구하기 때문이다.
#   그래서 호스트에서 심링크를 실경로로 풀어서 넘긴다. 부팅 순서가 바뀌어도
#   실행 시점에 다시 풀리므로 udev 규칙의 안정성은 유지된다.
#
#   컨테이너 안에서는 JONGKY_YAHBOOM_PORT / JONGKY_LIDAR_PORT 환경변수로
#   경로를 받는다.
#
# --cap-add SYS_NICE
#   없으면 controller_manager 가 FIFO 실시간 스케줄링을 못 잡는다.
#   지금은 50Hz 가 잘 나오지만 부하가 늘면 주기 지터가 생긴다.
set -euo pipefail

IMAGE="${JONGKY_IMAGE:-jongky:jazzy}"
WS="${JONGKY_WS:-$HOME/jongky_ws}"

DEVICE_ARGS=()
ENV_ARGS=()

add_device() {
  local link="$1" var="$2"
  if [ -e "$link" ]; then
    local real
    real="$(readlink -f "$link")"
    DEVICE_ARGS+=(--device "${real}:${real}")
    ENV_ARGS+=(-e "${var}=${real}")
    echo "장치: $link -> $real  (\$$var)"
  else
    echo "경고: $link 없음. 해당 장치 없이 실행한다" >&2
  fi
}

add_device /dev/yahboom JONGKY_YAHBOOM_PORT
add_device /dev/rplidar JONGKY_LIDAR_PORT

mkdir -p "$WS/src"

exec docker run -it --rm \
  --network host \
  --ipc host \
  --cap-add SYS_NICE \
  --pid host \
  -v "$WS:/ws" \
  -v /dev/shm:/dev/shm \
  "${DEVICE_ARGS[@]}" \
  "${ENV_ARGS[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  "$IMAGE" \
  "${@:-bash}"

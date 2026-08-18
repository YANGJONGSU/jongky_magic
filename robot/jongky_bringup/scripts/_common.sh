# 현장 스크립트 공통. 단독 실행용이 아니라 source 해서 쓴다.
#
# 컨테이너를 매번 새로 띄우는 대신 이미 떠 있으면 거기에 붙는다.
# 터치스크린에서 터미널을 여러 개 여는데, 각자 컨테이너를 띄우면
# 장치를 서로 뺏고 /map 도 공유가 안 된다.

IMAGE="${JONGKY_IMAGE:-jongky:jazzy}"
WS="${JONGKY_WS:-$HOME/jongky_ws}"
VOICES="${JONGKY_VOICES:-$HOME/voices}"
CONTAINER="${JONGKY_CONTAINER:-jongky_field}"

# 컨테이너가 살아 있으면 붙고, 없으면 띄운다.
# 터미널에서 직접 칠 때만 -it 를 붙인다. 파이프나 스크립트로 돌릴 때
# -it 를 주면 "cannot attach stdin to a TTY-enabled container" 로 죽는다.
_tty_flags() {
  if [ -t 0 ] && [ -t 1 ]; then echo "-it"; else echo "-i"; fi
}

# 키보드 입력이 필요한 명령(텔레옵)은 TTY 가 반드시 있어야 한다.
# JONGKY_FORCE_TTY=1 로 강제한다 — 자동 감지에 맡기면 파이프로 실행될 때
# termios.error: Inappropriate ioctl for device 로 죽는다.
jongky_exec() {
  local flags
  if [ "${JONGKY_FORCE_TTY:-0}" = "1" ]; then flags="-it"; else flags="$(_tty_flags)"; fi
  if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker exec $flags "$CONTAINER" bash -lc "$*"
    return $?
  fi

  echo "컨테이너를 새로 띄운다: $CONTAINER"
  local DEV=()
  for link in /dev/yahboom /dev/rplidar; do
    if [ -e "$link" ]; then
      real="$(readlink -f "$link")"
      DEV+=(--device "$real:$real")
    else
      echo "경고: $link 없음" >&2
    fi
  done
  [ -d /dev/snd ] && DEV+=(--device /dev/snd)
  [ -d "$VOICES" ] && DEV+=(-v "$VOICES:/voices:ro")

  # --privileged 가 필요하다. 아스트라(openni2_camera, libusb 백엔드)가
  # --device 만으로는 안 열린다 — USB 버스를 마운트해도 "Input/output error"
  # 로 실패한다. 단독 컨테이너로 --privileged 없이 --network host 없이 돌리면
  # 되던 게, jmap 이 만드는 컨테이너에서만 안 되는 걸로 이 문제를 잡았다.
  docker run -d --rm --name "$CONTAINER" \
    --network host --ipc host --cap-add SYS_NICE --privileged \
    -v "$WS:/ws" -v /dev/shm:/dev/shm -v /dev/bus/usb:/dev/bus/usb \
    -v "$HOME:/host" \
    "${DEV[@]}" \
    -e JONGKY_YAHBOOM_PORT="$(readlink -f /dev/yahboom 2>/dev/null)" \
    -e JONGKY_LIDAR_PORT="$(readlink -f /dev/rplidar 2>/dev/null)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "$IMAGE" sleep infinity >/dev/null

  sleep 2
  docker exec $flags "$CONTAINER" bash -lc "$*"
}

# 컨테이너 안에서 ROS 환경을 잡는 접두사
ROS_SETUP='source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash'

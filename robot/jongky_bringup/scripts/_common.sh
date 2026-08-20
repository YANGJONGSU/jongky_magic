# 현장 스크립트 공통. 단독 실행용이 아니라 source 해서 쓴다.
#
# 컨테이너를 매번 새로 띄우는 대신 이미 떠 있으면 거기에 붙는다.
# 터치스크린에서 터미널을 여러 개 여는데, 각자 컨테이너를 띄우면
# 장치를 서로 뺏고 /map 도 공유가 안 된다.

IMAGE="${JONGKY_IMAGE:-jongky:jazzy}"
WS="${JONGKY_WS:-$HOME/jongky_ws}"
VOICES="${JONGKY_VOICES:-$HOME/voices}"
CONTAINER="${JONGKY_CONTAINER:-jongky_field}"

# Fast DDS Discovery Server. 개발 PC 에서 지도가 쌓이는 걸 실시간으로 보려면
# 이게 있어야 한다. 로봇과 PC 가 같은 WiFi 에 있어도 멀티캐스트가 AP 에서
# 막혀 기본(Simple) 디스커버리로는 서로를 못 찾는다 — 실측으로 확인했다.
# 서버는 별도 컨테이너로 띄운다. 맵핑 컨테이너를 재기동해도 살아 있어야
# RViz 쪽 연결이 안 끊긴다.
DISCOVERY_CONTAINER="${JONGKY_DISCOVERY_CONTAINER:-jongky_discovery}"
DISCOVERY_PORT="${JONGKY_DISCOVERY_PORT:-11811}"

# 로봇 안의 노드는 루프백으로 붙는다. 젯슨 IP 가 DHCP 로 바뀌어도
# 로봇 쪽은 아무것도 안 고쳐도 된다 — 개발 PC 의 환경변수만 바꾸면 된다.
DISCOVERY_EP="127.0.0.1:$DISCOVERY_PORT"

jongky_lan_ip() {
  local ip
  ip="$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}')"
  [ -z "$ip" ] && ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "$ip"
}

# 서버가 없으면 띄운다. 이미 떠 있으면 아무것도 안 한다.
#
# 0.0.0.0 바인딩으로 충분하다 — 개발 PC 가 젯슨 LAN IP 로 붙어도 정상
# 동작하는 것을 실측으로 확인했다(서버 id 1 로 따로 띄워 대조 실험).
#
# 정작 걸린 건 다른 데였다: `ros2 topic list` 는 기본 spin-time 이 1초라
# WiFi + 디스커버리 서버 조합에서 그래프가 다 도착하기 전에 끝나 버린다.
# 개발 PC 에서는 데몬을 먼저 띄우고 10초쯤 기다리거나
# `--no-daemon --spin-time 15` 를 줄 것. 토픽이 "안 보이는" 게 아니라
# "아직 안 온" 것이다.
jongky_discovery_up() {
  if docker ps --format '{{.Names}}' | grep -qx "$DISCOVERY_CONTAINER"; then
    return 0
  fi
  docker rm -f "$DISCOVERY_CONTAINER" >/dev/null 2>&1 || true
  echo "디스커버리 서버를 띄운다: $DISCOVERY_CONTAINER (0.0.0.0:$DISCOVERY_PORT)"
  docker run -d --restart unless-stopped --name "$DISCOVERY_CONTAINER" \
    --network host "$IMAGE" \
    bash -lc "source /opt/ros/jazzy/setup.bash && exec fastdds discovery -i 0 -l 0.0.0.0 -p $DISCOVERY_PORT" >/dev/null
  sleep 3
  if docker ps --format '{{.Names}}' | grep -qx "$DISCOVERY_CONTAINER"; then
    echo "  개발 PC 에서:  export ROS_DISCOVERY_SERVER=$(jongky_lan_ip):$DISCOVERY_PORT ROS_SUPER_CLIENT=TRUE"
  else
    echo "경고: 디스커버리 서버 기동 실패 — 개발 PC 에서 토픽이 안 보인다" >&2
    docker logs "$DISCOVERY_CONTAINER" 2>&1 | tail -5 >&2
  fi
}

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
  jongky_discovery_up

  if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker exec $flags \
      -e ROS_DISCOVERY_SERVER="$DISCOVERY_EP" -e ROS_SUPER_CLIENT=TRUE \
      "$CONTAINER" bash -lc "$*"
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
  # RViz 를 로봇 화면에 띄우려면 X11 을 넘겨야 한다. 이게 없으면
  # use_rviz:=true 를 줘도 "cannot connect to display" 로 조용히 죽고,
  # 맵핑은 그대로 도니까 화면이 없는 걸 눈치채기 어렵다.
  local X11=()
  if [ -n "${DISPLAY:-}" ]; then
    X11=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
    [ -n "${XAUTHORITY:-}" ] && X11+=(-v "$XAUTHORITY:$XAUTHORITY:ro" -e "XAUTHORITY=$XAUTHORITY")
    xhost +local:root >/dev/null 2>&1 || true
  else
    echo "경고: DISPLAY 가 없다 — RViz 를 못 띄운다. 로봇 화면에서 실행할 것" >&2
  fi

  docker run -d --rm --name "$CONTAINER" \
    --network host --ipc host --cap-add SYS_NICE --privileged \
    -v "$WS:/ws" -v /dev/shm:/dev/shm -v /dev/bus/usb:/dev/bus/usb \
    -v "$HOME:/host" \
    "${X11[@]}" \
    "${DEV[@]}" \
    -e JONGKY_YAHBOOM_PORT="$(readlink -f /dev/yahboom 2>/dev/null)" \
    -e JONGKY_LIDAR_PORT="$(readlink -f /dev/rplidar 2>/dev/null)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e ROS_DISCOVERY_SERVER="$DISCOVERY_EP" \
    -e ROS_SUPER_CLIENT=TRUE \
    "$IMAGE" sleep infinity >/dev/null

  sleep 2
  docker exec $flags \
    -e ROS_DISCOVERY_SERVER="$DISCOVERY_EP" -e ROS_SUPER_CLIENT=TRUE \
    "$CONTAINER" bash -lc "$*"
}

# 컨테이너 안에서 ROS 환경을 잡는 접두사
ROS_SETUP='source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash'

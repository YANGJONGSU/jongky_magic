# 종키 온보드 런타임 — Jetson Orin Nano Super (JetPack 6 / L4T R36.4)
#
# 베이스는 dustynv 계열 ROS 2 Jazzy + PyTorch 이미지다. 호스트는 Ubuntu 22.04
# 이지만 컨테이너 유저스페이스가 24.04 라 Jazzy 가 네이티브로 돈다.
#
# 베이스에는 ros-base 만 들어 있어 ros2_control 계열이 없다. 여기서 얹는다.
#
# 빌드 (젯슨에서):
#   docker build -f docker/Dockerfile.robot -t jongky:jazzy .
#
# 실행은 docker/run_robot.sh 참조.

ARG BASE_IMAGE=ros_torch:jazzy
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# ros2_control 스택과 로봇 모델 도구.
#
# 라이다 드라이버는 여기에 없다. apt 의 ros-jazzy-rplidar-ros(2.1.0, SDK 1.12.0)
# 는 우리 C1 을 모른다 — 스캔 시작이 타임아웃으로 실패한다. 소스로 받아
# 워크스페이스에서 빌드한다: robot/jongky_robot.repos 참조.
# 그 잘못된 버전이 오버레이 없이 잡히는 사고를 막으려고 아예 안 깐다.
#
# vcs import 를 쓰려면 python3-vcstool 이 필요하다.
#
# 아스트라는 openni2_camera + 오르베 재배포 OpenNI2 로 붙인다.
# apt 판 orbbec_camera(SDK v2)로는 안 된다 — 아래 RUN 주석 참조.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-vcstool \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-diff-drive-controller \
      ros-${ROS_DISTRO}-joint-state-broadcaster \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-openni2-camera \
      python3-colcon-common-extensions \
      build-essential \
      git \
      alsa-utils \
      ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# ── 아스트라(Orbbec Astra, USB 2bc5:0401) ────────────────────────────────────
#
# 우리 카메라를 여는 조합을 찾기까지 세 갈래를 다 밟았다.
#
#   ros-jazzy-orbbec-camera (OrbbecSDK v2.7.6)
#       PID 0401 을 지원 목록에 안 갖고 있다. SDK 가 아는 PID 는
#       0x06xx / 0x08xx / 0x0Axx 대역뿐이라 이 장치는 열거 후보에서 아예 빠진다.
#       그래서 노드는 뜨는데 에러 한 줄 없이 /camera/device_status 만 나온다.
#
#   openni2_camera + 배포판 PS1080 드라이버
#       PS1080 은 PrimeSense VID 만 안다. "Found 0 devices".
#
#   openni2_camera + 오르베 재배포 OpenNI2      ← 이게 답이다
#       오르베가 자기 VID(0x2bc5)를 넣어 빌드한 OpenNI2 계층으로 갈아끼운다.
#       ROS 노드(openni2_camera)는 apt 판 그대로 쓴다.
#
# liborbbec.so 만 복사하면 안 된다. 같은 디렉터리의 orbbec.ini 를 읽고,
# libOpenNI2 본체도 오르베 빌드여야 한다.
#
# LD_LIBRARY_PATH 로는 안 된다 — 런치가 컴포넌트 컨테이너를 별도 프로세스로
# 띄우면서 환경이 유실된다. 시스템 libOpenNI2.so.0 을 직접 덮어써야 한다.
#
# 확인: ros2 run openni2_camera list_devices 가 장치 Uri 와 시리얼을 뱉으면 성공.
ARG ASTRA_REPO=https://github.com/orbbec/ros2_astra_camera.git
RUN git clone --depth 1 ${ASTRA_REPO} /tmp/astra_src \
    && ARCH_DIR=/tmp/astra_src/astra_camera/openni2_redist/$(uname -m | sed 's/aarch64/arm64/; s/x86_64/x64/') \
    && LIB_DIR=/lib/$(uname -m)-linux-gnu \
    && cp ${ARCH_DIR}/libOpenNI2.so ${LIB_DIR}/libOpenNI2.so.0 \
    && cp ${ARCH_DIR}/OpenNI2/Drivers/* /usr/lib/$(uname -m)-linux-gnu/OpenNI2/Drivers/ \
    && cp ${ARCH_DIR}/OpenNI.ini ${LIB_DIR}/ \
    && rm -rf /tmp/astra_src

# ── 안내 음성 (jongky_guide) ────────────────────────────────────────────────
#
# piper  : 오프라인 한국어 TTS. 층별 서브넷이 격리돼 있어 클라우드 TTS 는
#          층을 넘는 순간 끊긴다. 음성 모델(.onnx)은 여기 굽지 않는다 —
#          라이선스가 모델마다 다르고 교체 가능해야 해서 --voice 로 준다.
# whisper: STT. torch 는 이미 이미지에 있으므로 재설치 없이 얹힌다.
#
# --index-url 을 반드시 명시할 것. 안 주면 이 환경에서 pip 이 기본 인덱스를
# 해석하지 못해 "No matching distribution found" 로 죽는다. curl 로는 같은
# 주소가 200 을 받으므로 네트워크 문제가 아니다.
RUN pip install --no-cache-dir --index-url https://pypi.org/simple \
      piper-tts \
      openai-whisper

# whisper 모델은 첫 실행 때 받아 ~/.cache/whisper 에 캐시된다. 컨테이너를
# --rm 으로 띄우므로 미리 받아 두지 않으면 매번 다시 받는다. tiny 는 39MB 다.
RUN python3 -c "import whisper; whisper.load_model('tiny')"

# 워크스페이스는 실행 시 볼륨으로 마운트한다. 이미지에 굽지 않는 이유는
# 소스를 고칠 때마다 이미지를 다시 만들지 않기 위해서다.
WORKDIR /ws

# 컨테이너 안에서 대화형으로 쓸 때 ROS 환경이 자동으로 잡히도록.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

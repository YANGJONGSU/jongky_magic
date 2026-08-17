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
# 아스트라(astra_camera)는 별도 검증 후 추가한다.
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
      python3-colcon-common-extensions \
      build-essential \
    && rm -rf /var/lib/apt/lists/*

# 워크스페이스는 실행 시 볼륨으로 마운트한다. 이미지에 굽지 않는 이유는
# 소스를 고칠 때마다 이미지를 다시 만들지 않기 위해서다.
WORKDIR /ws

# 컨테이너 안에서 대화형으로 쓸 때 ROS 환경이 자동으로 잡히도록.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]

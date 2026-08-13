"""RPLIDAR C1 만 띄운다.

라이다만 따로 볼 때 쓰고, robot.launch.py 가 이걸 include 한다.

  ros2 launch jongky_bringup lidar.launch.py

드라이버는 apt 판이 아니라 소스 빌드본이어야 한다. apt 의
ros-jazzy-rplidar-ros 2.1.0(SDK 1.12.0)은 C1 이 나오기 전 버전이라
스캔 시작이 타임아웃으로 실패한다. robot/jongky_robot.repos 참조.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jongky_bringup')
    params = os.path.join(bringup_pkg, 'config', 'rplidar_c1.yaml')

    lidar_port = LaunchConfiguration('lidar_port')

    # 포트만 런치 인자로 받는다. 나머지는 YAML 이 들고 있다 —
    # 장비 특성값이 런치 파일과 설정 파일로 흩어지면 어느 쪽이 이겼는지
    # 추적하기 어렵다.
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[params, {'serial_port': lidar_port}],
        output='screen',
    )

    return LaunchDescription([
        # 기본값을 환경변수에서 받는다. udev 심링크(/dev/rplidar)는 컨테이너
        # 안에 존재하지 않는다 — --device 가 심링크가 아니라 실제 노드를
        # 요구해서, run_robot.sh 가 호스트에서 실경로로 풀어 JONGKY_LIDAR_PORT
        # 로 넘긴다. 호스트에서 직접 돌릴 때는 변수가 없으므로 심링크를 쓴다.
        DeclareLaunchArgument(
            'lidar_port',
            default_value=EnvironmentVariable(
                'JONGKY_LIDAR_PORT', default_value='/dev/rplidar'),
            description='라이다 시리얼 포트. 기본값은 $JONGKY_LIDAR_PORT'),
        lidar_node,
    ])

"""종키 진입점.

구동계(ros2_control)와 라이다를 한 번에 올린다.

  # 실물
  ros2 launch jongky_bringup robot.launch.py

  # 하드웨어 없이 전 경로만 (라이다도 뺀다)
  ros2 launch jongky_bringup robot.launch.py use_mock:=true use_lidar:=false

  # 라이다 없이 주행만
  ros2 launch jongky_bringup robot.launch.py use_lidar:=false

올라오는 것
  /joint_states  /odom  /imu/data  /scan  /odometry/filtered
  TF(odom -> base_footprint -> ...)  ← odom->base_footprint 는 EKF 가 낸다

아직 없는 것
  아스트라(astra_camera)가 이미지에 없다. IMX219 는 /dev/video* 자체가
  안 잡힌다(jetson-io 로 디바이스 트리 설정 필요). TOF 2개는 노드가 없다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jongky_bringup')
    ctrl_pkg = get_package_share_directory('jongky_control')

    use_mock = LaunchConfiguration('use_mock')
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    use_ekf = LaunchConfiguration('use_ekf')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'control.launch.py')),
        launch_arguments={
            'use_mock': use_mock,
            'use_rviz': use_rviz,
            'serial_port': serial_port,
        }.items(),
    )

    # 엔코더의 전진 거리와 자이로의 회전을 융합한다. 설계 의도는 ekf.yaml 참조.
    #
    # **이 노드가 odom -> base_footprint 의 주인이다.** 그래서
    # jongky_controllers.yaml 의 enable_odom_tf 가 false 다. EKF 를 끄면 그 TF 를
    #아무도 안 내고, 증상은 "노드는 다 떴는데 아무것도 안 움직인다" 로 나온다 —
    # 원인을 전혀 안 알려주는 종류다. 아래 경고가 그래서 있다.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(ctrl_pkg, 'config', 'ekf.yaml')],
        condition=IfCondition(use_ekf),
    )

    ekf_off_warning = LogInfo(
        msg=('[경고] use_ekf:=false 다. odom -> base_footprint 를 낼 노드가 없다. '
             'jongky_control/config/jongky_controllers.yaml 의 '
             'enable_odom_tf 를 true 로 되돌려야 주행한다.'),
        condition=UnlessCondition(use_ekf),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'lidar.launch.py')),
        launch_arguments={'lidar_port': lidar_port}.items(),
        condition=IfCondition(use_lidar),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description=('robot_localization EKF 로 오도메트리 + 자이로를 융합한다. '
                         'false 로 두면 enable_odom_tf 도 함께 되돌려야 한다'),
        ),
        DeclareLaunchArgument(
            'use_mock', default_value='false',
            description='true 면 목업 하드웨어. 보드 없이 돌려볼 때'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='RViz 동시 실행. 젯슨에서는 보통 false 로 두고 '
                        '개발 PC 에서 따로 띄운다'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='false 면 라이다를 빼고 구동계만'),
        # 포트 기본값은 환경변수에서 받는다. udev 심링크는 컨테이너 안에
        # 없다 — run_robot.sh 가 호스트에서 실경로로 풀어 넘긴다.
        DeclareLaunchArgument(
            'serial_port',
            default_value=EnvironmentVariable(
                'JONGKY_YAHBOOM_PORT', default_value='/dev/yahboom'),
            description='야붐 제어보드 포트. 기본값은 $JONGKY_YAHBOOM_PORT'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=EnvironmentVariable(
                'JONGKY_LIDAR_PORT', default_value='/dev/rplidar'),
            description='라이다 포트. 기본값은 $JONGKY_LIDAR_PORT'),

        control,
        ekf,
        ekf_off_warning,
        lidar,
    ])

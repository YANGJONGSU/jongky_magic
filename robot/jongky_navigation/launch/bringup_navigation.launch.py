"""종키 전체 통합 브링업 런치 파일 (로봇 구동계 + 라이다 + SLAM 또는 Nav2 자율주행).

  # 1) SLAM 맵핑 모드 (실물)
  ros2 launch jongky_navigation bringup_navigation.launch.py mode:=slam

  # 2) Nav2 자율주행 모드 (실물 + 지도 지정)
  ros2 launch jongky_navigation bringup_navigation.launch.py mode:=nav map:=/path/to/my_map.yaml

  # 3) 실시간 SLAM 자율주행 모드 (지도 없이 주행하며 맵핑)
  ros2 launch jongky_navigation bringup_navigation.launch.py mode:=slam_nav

  # 4) 목업 하드웨어 검증 (라이다 없이 구동계 목업 + 내비 스택)
  ros2 launch jongky_navigation bringup_navigation.launch.py use_mock:=true use_lidar:=false mode:=nav map:=/path/to/my_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description():
    bringup_pkg = get_package_share_directory('jongky_bringup')
    nav_pkg = get_package_share_directory('jongky_navigation')

    mode = LaunchConfiguration('mode')
    use_mock = LaunchConfiguration('use_mock')
    use_lidar = LaunchConfiguration('use_lidar')
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    default_nav_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox_params.yaml')

    # 1. 로봇 베이스 브링업 (ros2_control + lidar)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_mock': use_mock,
            'use_lidar': use_lidar,
            'use_rviz': 'false',  # RViz는 네비게이션 설정으로 띄움
            'serial_port': serial_port,
            'lidar_port': lidar_port,
        }.items(),
    )

    # 2. SLAM 단독 모드
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'params_file': default_slam_params,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'slam')),
    )

    # 3. Nav2 자율주행 모드 (지도 기반)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'nav')),
    )

    # 4. 실시간 SLAM + Nav2 모드
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation_slam.launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'slam_params_file': default_slam_params,
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(EqualsSubstitution(mode, 'slam_nav')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            description='동작 모드: slam (맵핑), nav (자율주행), slam_nav (실시간 SLAM 주행)',
        ),
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='true 면 하드웨어 없이 목업 인터페이스 사용',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='라이다 구동 여부',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='RViz2 시각화 동시 실행 여부 (젯슨에서는 보통 false)',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_pkg, 'maps', 'sample_map.yaml'),
            description='자율주행 모드(mode:=nav)에서 사용할 지도 YAML 경로',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav_params,
            description='Nav2 파라미터 YAML 경로',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=EnvironmentVariable(
                'JONGKY_YAHBOOM_PORT', default_value='/dev/yahboom'
            ),
            description='야붐 제어보드 포트',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=EnvironmentVariable(
                'JONGKY_LIDAR_PORT', default_value='/dev/rplidar'
            ),
            description='라이다 포트',
        ),
        robot_bringup,
        slam_launch,
        nav_launch,
        slam_nav_launch,
    ])

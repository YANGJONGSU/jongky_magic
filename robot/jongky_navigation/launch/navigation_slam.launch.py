"""종키 실시간 SLAM + Nav2 동시 구동 런치 파일 (사전 지도 없이 맵핑하며 자율주행).

  # 기본 실행
  ros2 launch jongky_navigation navigation_slam.launch.py

  # 개발 PC에서 RViz 포함 실행
  ros2 launch jongky_navigation navigation_slam.launch.py use_rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('jongky_navigation')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')

    default_nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox_params.yaml')
    default_rviz_config = os.path.join(nav_pkg, 'rviz', 'nav2.rviz')

    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이션 시간(/clock) 사용 여부',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Nav2 파라미터 YAML 파일 경로',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='SLAM Toolbox 파라미터 YAML 파일 경로',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Nav2 라이프사이클 노드 자동 활성화 여부',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='ROS 2 컴포넌트 컨테이너(합성 노드) 사용 여부',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='노드 크래시 시 자동 재시작 여부',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='로그 레벨 (info, debug, warn, error)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='RViz2 시각화 동시 실행 여부',
        ),
        nav2_slam_launch,
        rviz_node,
    ])

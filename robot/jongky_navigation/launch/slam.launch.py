"""종키 2D SLAM 맵핑 런치 파일 (SLAM Toolbox Online Async).

  # 기본 실행 (헤드리스)
  ros2 launch jongky_navigation slam.launch.py

  # RViz 포함 실행 (개발 PC)
  ros2 launch jongky_navigation slam.launch.py use_rviz:=true

  # 라이프사이클 자동 활성화 끄기 (수동 전이로 디버깅할 때)
  ros2 launch jongky_navigation slam.launch.py autostart:=false
"""
import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    nav_pkg = get_package_share_directory('jongky_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    default_params_file = os.path.join(nav_pkg, 'config', 'slam_toolbox_params.yaml')
    default_rviz_config = os.path.join(nav_pkg, 'rviz', 'slam.rviz')

    # slam_toolbox 2.8.5 는 스스로 라이프사이클 전이를 하지 않는다. 외부에서
    # configure/activate 를 걸어주지 않으면 unconfigured 로 멈춰 선다. 이때
    # 파라미터가 on_configure 에서 선언되므로 YAML 이 통째로 안 읽히고,
    # /map 도 map->odom TF 도 나오지 않는다. 노드는 멀쩡히 떠 있어서
    # `ros2 node list` 만으로는 정상으로 보인다 — 증상이 원인을 안 알려준다.
    #
    # mode:=slam_nav 는 nav2 bringup 이 자체 관리자를 갖고 있어 문제없다.
    # 순수 맵핑 모드만 전이를 직접 걸어줘야 한다.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # nav2_lifecycle_manager 로는 안 된다. Discovery Server + SUPER_CLIENT
    # 환경에서 관리자의 change_state 호출이 돌아오지 않고 "Configuring
    # slam_toolbox" 에서 영영 멈춘다. 같은 순간 CLI 로 같은 전이를 걸면 즉시
    # 성공하므로 노드 문제가 아니라 관리자 쪽 서비스 매칭 문제다.
    # 런치 자체 이벤트로 거는 편이 이 환경에서 확실하다.
    configure_event = TimerAction(
        period=5.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
        condition=IfCondition(autostart),
    )

    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
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
            default_value=default_params_file,
            description='SLAM Toolbox 파라미터 YAML 파일 경로',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='RViz2 시각화 동시 실행 여부 (젯슨에서는 보통 false)',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='slam_toolbox 라이프사이클 자동 활성화 여부',
        ),
        slam_node,
        activate_on_inactive,
        configure_event,
        rviz_node,
    ])

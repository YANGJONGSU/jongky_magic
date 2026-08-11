"""종키 컨트롤러 스택.

robot_state_publisher + controller_manager + 컨트롤러 2개를 띄운다.
jongky_bringup 이 생기면 이 런치를 include 해서 쓰면 된다.

  # 하드웨어 없이 전 경로 검증
  ros2 launch jongky_control control.launch.py use_mock:=true

  # 실물 (jongky_hardware 구현 후)
  ros2 launch jongky_control control.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('jongky_description')
    ctrl_pkg = get_package_share_directory('jongky_control')

    use_mock = LaunchConfiguration('use_mock')
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')

    # xacro 출력은 긴 XML 문자열이다. 감싸지 않으면 launch 가 YAML 로
    # 파싱하려다 실패한다.
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([desc_pkg, 'urdf', 'robot.urdf.xacro']),
            ' use_mock:=', use_mock,
            ' serial_port:=', serial_port,
        ]),
        value_type=str,
    )

    controllers_yaml = os.path.join(ctrl_pkg, 'config', 'jongky_controllers.yaml')

    # controller_manager 는 robot_description 을 파라미터로 직접 받는다.
    # (예전처럼 /robot_description 토픽을 구독하지 않는다)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
        output='screen',
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '-c', '/controller_manager'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='false',
            description='true 면 목업 하드웨어. 실물 없이 돌려볼 때 사용'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='RViz 동시 실행'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/yahboom',
            description='야붐 보드 시리얼 포트. 가상 포트로 시험할 때 바꾼다'),

        control_node,
        rsp_node,

        # 스포너는 순서가 있다. joint_state_broadcaster 가 먼저 활성화돼야
        # 관절 상태가 흐르고, 그 뒤에 diff_drive_controller 를 올린다.
        jsb_spawner,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[diff_spawner])),
        RegisterEventHandler(
            OnProcessExit(target_action=diff_spawner, on_exit=[rviz_node])),
    ])

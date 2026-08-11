import os
from launch import LaunchDescription
from launch.actions import Shutdown
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('jongky_description')

    # xacro가 .xacro 파일을 순수 URDF(XML)로 변환한 결과를
    # robot_state_publisher에게 통째로 넘긴다.
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg, 'urdf', 'robot.urdf.xacro']),
    ])

    # URDF를 읽어 TF를 발행한다. TF는 여기서 자동으로 생긴다.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # continuous 조인트(바퀴)를 슬라이더로 직접 돌려보는 창.
    # 실물 로봇에서는 엔코더가 이 역할을 대신한다.
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'view_robot.rviz')],
        on_exit=Shutdown(),
    )

    return LaunchDescription([rsp_node, jsp_gui_node, rviz_node])

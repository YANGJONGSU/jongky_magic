"""안내로봇 런치 — Nav2 자율주행 + UI + 음성.

    ros2 launch jongky_guide guide.launch.py \
        map:=/path/to/fastcampus_10f.yaml \
        waypoints:=~/waypoints_10f.yaml

음성은 기본으로 꺼져 있다. Whisper 가 젯슨 CPU 를 상당히 먹으므로
주행이 먼저 안정된 뒤에 켜는 것을 권한다.

    use_voice:=true voice_model:=tiny mic:=plughw:1,0
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
    nav_share = get_package_share_directory("jongky_navigation")

    args = [
        DeclareLaunchArgument("map", description="점유 격자 지도 YAML (층마다 다르다)"),
        DeclareLaunchArgument("waypoints", default_value="~/waypoints.yaml",
                              description="teleop_key.py 가 찍어 둔 목적지"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("use_voice", default_value="false"),
        DeclareLaunchArgument("voice_model", default_value="tiny"),
        DeclareLaunchArgument("mic", default_value=""),
        DeclareLaunchArgument("tts_voice", default_value="", description="piper onnx 경로"),
        DeclareLaunchArgument("audio_device", default_value="", description="aplay -D 값"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "bringup_navigation.launch.py")
        ),
        launch_arguments={
            "mode": "nav",
            "map": LaunchConfiguration("map"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    guide = Node(
        package="jongky_guide",
        executable="guide_node.py",
        name="jongky_guide",
        output="screen",
        arguments=[
            "--waypoints", LaunchConfiguration("waypoints"),
            "--port", LaunchConfiguration("port"),
            "--voice", LaunchConfiguration("tts_voice"),
            "--audio-device", LaunchConfiguration("audio_device"),
        ],
    )

    voice = Node(
        package="jongky_guide",
        executable="voice_node.py",
        name="jongky_voice",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_voice")),
        arguments=[
            "--model", LaunchConfiguration("voice_model"),
            "--mic", LaunchConfiguration("mic"),
            "--waypoints", LaunchConfiguration("waypoints"),
        ],
    )

    return LaunchDescription(args + [navigation, guide, voice])

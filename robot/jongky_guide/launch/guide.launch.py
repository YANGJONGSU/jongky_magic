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
        DeclareLaunchArgument("llm", default_value="", description="ollama 모델 (예: gemma4:e2b)"),
        DeclareLaunchArgument("llm_url", default_value="",
                              description="ollama 주소. 비우면 brain.py 기본값(관제 노트북)"),
        DeclareLaunchArgument("follow_url", default_value="",
                              description="후면 사람 탐지 서비스. 주면 뒤처진 사람을 기다린다 "
                                          "(예: http://localhost:8641/follower)"),
        DeclareLaunchArgument("tts_voice", default_value="", description="piper onnx 경로"),
        DeclareLaunchArgument("audio_device", default_value="", description="aplay -D 값"),
        DeclareLaunchArgument(
            "start_waypoint", default_value="",
            description="시작 위치로 쓸 waypoint. 안 주면 UI 의 '여기서 시작' 에서 고른다. "
                        "이걸 안 정하면 AMCL 이 map->odom 을 못 내서 어떤 목적지도 안 간다"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_camera", default_value="true",
            description="아스트라를 띄운다. 끄면 brain 의 돌발상황 판단이 항상 건너뛰어진다 "
                        "— 먹일 영상이 없기 때문이다"),
    ]

    # 카메라. 이 런치가 이걸 안 띄우면 guide_node 의 _latest_jpeg() 가 항상
    # None 이고, _handle_obstacle() 이 매번 판단을 건너뛴다. brain.py 와
    # judge_obstacle() 은 배선돼 있는데 **먹일 영상이 없어서** VLM 경로 전체가
    # dead code 였다.
    #
    # openni2 는 lazy 발행이라 구독자가 붙어야 스트림이 돈다. guide_node 가
    # 구독하므로 순서는 상관없다 (bag 을 뜨는 jmap 과 다른 점).
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openni2_camera"),
                "launch", "camera_only.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

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
            "--mic", LaunchConfiguration("mic"),
            "--llm", LaunchConfiguration("llm"),
            "--llm-url", LaunchConfiguration("llm_url"),
            "--follow-url", LaunchConfiguration("follow_url"),
            "--start-waypoint", LaunchConfiguration("start_waypoint"),
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

    return LaunchDescription(args + [camera, navigation, guide, voice])

cmake_minimum_required(VERSION 3.8)
project(guide_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Poi.msg
  msg/GuideStatus.msg
  srv/ListPois.srv
  srv/SetStart.srv
  action/Guide.action
  DEPENDENCIES geometry_msgs action_msgs
)

# waypoint YAML 스키마는 메시지가 아니라 디스크 포맷이라 rosidl 이 못 담는다.
# 실물 예제를 같이 깔아 두면 teleop_key.py 로 찍은 파일과 대조할 수 있다.
install(
  DIRECTORY schema
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

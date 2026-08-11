cmake_minimum_required(VERSION 3.8)
project(jongky_description)

find_package(ament_cmake REQUIRED)

# urdf, meshes, launch, rviz 폴더를 통째로 설치 위치에 복사한다.
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 실측 확인용 스크립트 — ros2 run jongky_description check_laser_yaw.py
install(
  PROGRAMS
    scripts/check_laser_yaw.py
    scripts/check_depth_min_range.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

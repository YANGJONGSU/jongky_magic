cmake_minimum_required(VERSION 3.8)
project(jongky_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch tools scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  tools/teleop_key.py
  tools/odom_imu_watch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

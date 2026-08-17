cmake_minimum_required(VERSION 3.10)
project(jongky_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz tools
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  tools/save_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

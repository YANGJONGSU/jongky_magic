cmake_minimum_required(VERSION 3.16)
project(jongky_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS THIS_PACKAGE_DEPS)
  find_package(${dep} REQUIRED)
endforeach()

add_library(jongky_hardware SHARED
  src/jongky_system.cpp
  src/yahboom_board.cpp
)
target_compile_features(jongky_hardware PUBLIC cxx_std_17)
target_include_directories(jongky_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/jongky_hardware>
)
ament_target_dependencies(jongky_hardware PUBLIC ${THIS_PACKAGE_DEPS})

pluginlib_export_plugin_description_file(hardware_interface jongky_hardware.xml)

install(DIRECTORY include/ DESTINATION include/jongky_hardware)
install(TARGETS jongky_hardware
  EXPORT export_jongky_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 수동 시험용 보드 시뮬레이터. 하드웨어 없이 전 경로를 돌릴 때 쓴다.
install(PROGRAMS test/fake_board.py DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_jongky_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_package()

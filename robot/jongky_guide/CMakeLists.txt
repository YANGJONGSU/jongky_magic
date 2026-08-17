cmake_minimum_required(VERSION 3.8)
project(jongky_guide)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch web tools
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  tools/guide_node.py
  tools/voice_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# speech.py 는 guide_node.py 가 import 한다. 실행 파일이 아니므로
# PROGRAMS 가 아니라 같은 디렉터리에 얹어 둔다.
install(
  FILES tools/speech.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

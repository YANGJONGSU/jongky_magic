# 지도(Maps) 디렉토리

종키 AMR 주행에 사용할 점유 격자 지도(`.yaml`, `.pgm`) 보관 장소입니다.

## 1. 새 지도 작성 및 저장 방법

1. **SLAM 맵핑 모드 실행**:
   ```bash
   # 실물 젯슨에서:
   ros2 launch jongky_navigation bringup_navigation.launch.py mode:=slam use_rviz:=false

   # 개발 PC에서 (RViz 모니터링):
   ros2 launch jongky_navigation slam.launch.py use_rviz:=true
   ```

2. **주행을 통해 공간 탐색 완료 후 지도 저장**:
   - 도구 스크립트 사용:
     ```bash
     ros2 run jongky_navigation save_map.py --name my_lab_map
     ```
   - 또는 표준 Nav2 CLI 직접 사용:
     ```bash
     ros2 run nav2_map_server map_saver_cli -f ~/jongky_ws/src/jongky_navigation/maps/my_lab_map
     ```

## 2. 저장된 지도로 자율주행 실행

```bash
ros2 launch jongky_navigation bringup_navigation.launch.py mode:=nav map:=/path/to/my_lab_map.yaml
```

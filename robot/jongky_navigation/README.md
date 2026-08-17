# jongky_navigation

종키(Jongky) AMR 2D SLAM 지도 작성 및 Nav2 자율 내비게이션 패키지.

## 주요 기능

1. **2D SLAM 맵핑 (SLAM Toolbox Online Async)**: RPLIDAR C1 및 오도메트리 기반 실시간 지도 작성
2. **Nav2 자율 내비게이션**: AMCL 위치 추정, SmacPlanner2D 전역 경로 계획, Regulated Pure Pursuit (RPP) 로컬 경로 추종
3. **실시간 SLAM 자율주행**: 사전 지도 없이 SLAM 맵핑과 Nav2 자율주행을 동시에 구동
4. **실차 특성 튜닝 완료**: 속도/가속도 제한($0.40\,\text{m/s}$, $0.30\,\text{m/s}^2$), 풋프린트($148\times 229\,\text{mm}$), C1 유효 거리 반영

---

## 1. 실행 방법

### A. SLAM 지도 작성 (맵핑 모드)

1. **로봇 구동계 + 라이다 + SLAM 실행 (젯슨 온보드)**:
   ```bash
   ros2 launch jongky_navigation bringup_navigation.launch.py mode:=slam use_rviz:=false
   ```

2. **개발 PC에서 RViz 모니터링 (선택 사항)**:
   ```bash
   ros2 launch jongky_navigation slam.launch.py use_rviz:=true
   ```

3. **공간 탐색 후 지도 저장**:
   ```bash
   ros2 run jongky_navigation save_map.py --name my_lab_map --dir ~/jongky_ws/src/jongky_navigation/maps
   ```

---

### B. Nav2 자율주행 (사전 작성된 지도 기반)

1. **로봇 구동계 + 라이다 + Nav2 실행**:
   ```bash
   ros2 launch jongky_navigation bringup_navigation.launch.py mode:=nav map:=/path/to/my_lab_map.yaml use_rviz:=false
   ```

2. **개발 PC에서 RViz2 실행 후 목적지 지정**:
   ```bash
   ros2 launch jongky_navigation navigation.launch.py map:=/path/to/my_lab_map.yaml use_rviz:=true
   ```
   - RViz 상단 툴바의 **2D Pose Estimate**를 클릭하여 지도의 로봇 실제 위치/방향 지정
   - **Nav2 Goal**을 클릭하여 목표 지점 및 방향 지정 $\rightarrow$ 자율주행 개시

---

### C. 실시간 SLAM 자율주행 (지도 없이 주행하며 맵핑)

```bash
ros2 launch jongky_navigation bringup_navigation.launch.py mode:=slam_nav
```

---

### D. 하드웨어 없이 시뮬레이션 / 목업 검증

```bash
ros2 launch jongky_navigation bringup_navigation.launch.py use_mock:=true use_lidar:=false mode:=nav
```

---

## 2. 주요 설정 파일

- `config/slam_toolbox_params.yaml`: SLAM Toolbox 설정 (해상도 0.05m, C1 16m 범위, Orin Nano 최적화)
- `config/nav2_params.yaml`: Nav2 전체 파라미터 (AMCL, RPP Controller, Smac2D Planner, Costmap Footprint, Velocity Smoother)
- `rviz/slam.rviz`: SLAM 전용 RViz 뷰어 설정
- `rviz/nav2.rviz`: Nav2 전용 RViz 뷰어 설정
- `tools/save_map.py`: 지도 저장 유틸리티 CLI

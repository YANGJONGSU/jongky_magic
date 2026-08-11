# Conventions — 좌표계 · 단위 · 네이밍 규약

> **이 문서는 메쉬(CAD) 담당자와 반드시 공유 후 진행합니다.**
> 여기 정의된 규약과 다르게 export되면 URDF/충돌/관성 전부 재작업이 필요합니다.
> 메쉬를 받기 **전에** 아래 체크리스트로 먼저 확인하세요.

---

## 1. 단위 (Units)

- ROS/URDF 표준 단위: **미터(m)**, 라디안(rad), 킬로그램(kg)
- CAD 툴은 보통 **mm** 단위 → export 시 스케일 규약을 명시적으로 정할 것
  - 방법 A: CAD에서 export 전에 모델을 m 단위로 변환 (권장, URDF에 스케일 코드 안 남음)
  - 방법 B: mm 그대로 export하고 URDF `<mesh scale="0.001 0.001 0.001"/>` 로 보정
  - **한 프로젝트 안에서 방법 A/B 중 하나로 통일**, 파일마다 다르게 섞지 않기
- 어떤 방법을 썼는지 파일명 또는 전달 시 명시 (예: `base_link.dae` 단위: m)

## 2. 원점 (Origin) — `base_footprint` 와 `base_link`

원점은 **두 개**다. 헷갈리기 쉬우니 역할을 나눠서 본다.

| 프레임 | 위치 | 쓰는 곳 |
|---|---|---|
| `base_footprint` | 좌우 구동륜 축 중심을 **바닥에 투영**한 점 | 내비게이션·SLAM의 기준 바닥 좌표 |
| **`base_link`** | 좌우 구동륜 **축 중심**(바닥에서 바퀴 반지름만큼 위) | 로봇 본체. **모든 부품이 여기 정렬** |

두 프레임은 `base_footprint → base_link` fixed joint로 z축으로만
`wheel_radius` 만큼 떨어져 있다. 종키 기준 33.5mm.

### CAD 담당자용 — 어디에 맞춰서 export 하나

**`base_link`, 즉 구동륜 축 중심에 맞춘다. 바닥으로 내리지 않는다.**

- 두 구동륜 축 중심을 잇는 선분의 중점을 CAD assembly 원점으로 잡는다
- 그 점의 **높이 그대로** 원점으로 쓴다 (z=0으로 투영하지 않는다)
- 결과적으로 모델 아래쪽이 음수 z로 내려간다. 바퀴 아랫면이 대략
  `−wheel_radius`, 캐스터가 조금 더 내려갈 수 있다. **이게 정상이다**
- 모든 부품 메쉬는 이 원점 기준 상대 위치로 정렬되어야 한다.
  원점이 다르면 URDF `<origin>` 보정값을 부품마다 따로 구해야 해서
  오차가 누적된다

### 왜 바닥이 아니라 축인가

ROS 관례이고 Nav2·SLAM·`robot_state_publisher`가 이 형태를 기대한다.
구동륜 joint의 회전축이 `base_link` 원점을 지나야 오도메트리 계산이
깔끔하고, 바닥 기준이 필요한 쪽은 `base_footprint`를 쓰면 된다.
`base_link`를 바닥에 두면 바퀴 joint마다 z 오프셋이 붙고 타이어가
닳아 반지름이 변할 때 전부 다시 잡아야 한다.

> 2026-08 변경: 이 절은 원래 "`base_link` = 바닥 투영점"이었다.
> 실제 수령 메쉬(`base_link.stl`)가 축 중심 기준으로 왔고 그게
> ROS 관례와도 맞아서, 문서를 구현에 맞춰 고쳤다.
> 자세한 배경은 `robot/jongky_description/README.md` 참고.

## 3. 축 방향 (Axis convention)

- ROS 표준 (REP-103), 오른손 좌표계:
  - **x = 전방(forward)**
  - **y = 좌측(left)**
  - **z = 상방(up)**
- CAD 툴 기본 축은 다른 경우가 많음 (예: z가 아니라 y가 상방인 툴, x/y가 뒤바뀐 경우 등)
- **Export 전 확인 필수.** 확인이 어려우면:
  - 로봇 사진/스케치에 "앞쪽", "좌측", "위쪽"을 화살표로 표시해서 CAD 담당자에게 함께 전달
  - CAD 원점의 각 축이 실제로 어느 방향을 향하는지 회신받기

## 4. Visual / Collision 분리

| 구분 | 용도 | 폴리곤 |
|---|---|---|
| **visual** | 실제 형상, 재질/텍스처 포함, 화면 표시용 | 제한 없음 (과도하게 무겁지만 않으면 됨) |
| **collision** | 충돌 검사용, 단순화된 형상 | **폴리곤 예산 상한을 두고 단순화** |

- 22cm급 소형 로봇 기준: collision은 **원기둥(cylinder) 또는 convex hull** 정도로 충분
- collision을 visual과 동일한 고폴리곤 메쉬로 쓰지 말 것 → 실시간 충돌 검사(Nav2, gz sim) 성능 저하 원인

## 5. 파일 포맷 · 경로

- **visual**: `.dae` (재질 포함)
- **collision**: `.stl`
- 저장 위치: `robot/jongky_description/meshes/` 아래 고정
  - `meshes/visual/` — `.dae`
  - `meshes/collision/` — `.stl`

```
robot/jongky_description/meshes/
├── visual/
│   ├── base_link.dae
│   ├── wheel_left.dae
│   ├── wheel_right.dae
│   └── lidar_mount.dae
└── collision/
    ├── base_link.stl
    ├── wheel_left.stl
    ├── wheel_right.stl
    └── lidar_mount.stl
```

## 6. 부품 분할 (링크 단위)

- 파일은 **링크 단위로 분리**해서 export: 예) `base`, `wheel_left`, `wheel_right`, `lidar_mount`
- 하나의 파일에 여러 링크를 합쳐서 export하지 말 것
  - URDF에서 링크별 `<origin>`, `<joint>` 지정이 불가능해짐
- 링크 이름은 URDF에서 쓸 이름과 1:1로 맞춰서 요청 (네이밍 통일)

## 7. 질량 정보 (Mass properties)

- CAD에서 추출 가능하면 **부품별로 함께 요청**:
  - 질량 (mass, kg)
  - 무게중심 (center of mass)
  - 관성 텐서 (inertia tensor: `ixx iyy izz ixy ixz iyz`)
- 요청 시 **기준 좌표계를 명시**할 것 (부품 로컬 원점 기준인지, global assembly 기준인지) — 안 그러면 URDF `<inertial>`에 그대로 못 씀

---

## 메쉬 요청 전 체크리스트

- [ ] 단위: mm → m 변환 방법 확정 (A: CAD단 변환 / B: URDF scale)
- [ ] 원점: 좌우 구동륜 **축 중심** = `base_link` 기준 정렬
      (바닥으로 투영하지 말 것. 바닥 투영점은 `base_footprint` 로 URDF가 따로 만든다)
- [ ] 축 방향: x=전방 / y=좌측 / z=상방 확인 (그림으로 전달)
- [ ] visual(.dae, 재질 포함) / collision(.stl, 단순화) 분리
- [ ] 링크 단위로 파일 분리 (base / wheel_left / wheel_right / lidar_mount ...)
- [ ] 부품별 질량 + 관성 텐서 (+ 기준 좌표계) 포함 여부

이 문서는 메쉬 규약이 바뀌거나 새 링크가 추가될 때마다 갱신합니다.

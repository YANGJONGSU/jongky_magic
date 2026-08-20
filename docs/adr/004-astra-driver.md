# ADR 004: 아스트라 뎁스 카메라 드라이버 — orbbec_camera 가 아니라 openni2_camera

- **Status**: 채택 (Accepted). 실물에서 검증됨
- **Date**: 2026-08-17 (문서화 2026-08-20)

## Context

전면 뎁스 카메라가 Orbbec Astra (USB `2bc5:0401`) 다. 용도는 DreamerV3 관측 +
Cosmos 씨앗이고, `jongky_guide` 의 돌발상황 판단도 이 카메라 영상을 쓴다
(`guide_node.py` 의 카메라 구독).

ROS 2 Jazzy 를 쓰기로 한 것([adr/000](000-ros-distro.md))이 여기로 이어진다 —
**`astra_camera` 는 Jazzy 용 deb 이 없다.**

## Decision

**`openni2_camera`(apt 판) + 오르베 재배포 OpenNI2 로 시스템 라이브러리를
덮어쓴다.**

세 갈래를 다 밟은 결과다 (`docker/Dockerfile.robot:67-89`).

| 시도 | 결과 | 왜 |
|---|---|---|
| `ros-jazzy-orbbec-camera` (OrbbecSDK v2.7.6) | 노드는 뜨는데 `/camera/device_status` 만 나옴 | SDK 가 아는 PID 가 `0x06xx`/`0x08xx`/`0x0Axx` 대역뿐이라 **`0401` 이 열거 후보에서 아예 빠진다. 에러 한 줄 없이 조용히 아무것도 안 한다** |
| `openni2_camera` + 배포판 PS1080 드라이버 | `Found 0 devices` | PS1080 은 PrimeSense VID 만 안다 |
| **`openni2_camera` + 오르베 재배포 OpenNI2** | **성공** | 오르베가 자기 VID(`0x2bc5`)를 넣어 빌드한 OpenNI2 계층 |

재배포본은 `github.com/orbbec/ros2_astra_camera` 의
`astra_camera/openni2_redist/{arm64,x64}/` 에 있다 (빌드 불필요).
`Dockerfile.robot:43-65`(apt `ros-${ROS_DISTRO}-openni2-camera`)과
`:67-90`(덮어쓰기)에 구워 두었다.

### 두 가지 함정 — 둘 다 밟았다

- **`liborbbec.so` 만 복사하면 안 된다.** 같은 디렉터리의 `orbbec.ini` 를 읽고,
  `libOpenNI2` 본체도 오르베 빌드여야 한다 (`Dockerfile.robot:83-84`)
- **`LD_LIBRARY_PATH` 로는 안 된다.** 런치가 컴포넌트 컨테이너를 별도
  프로세스로 띄우면서 환경이 유실된다. 시스템 `libOpenNI2.so.0` 을 직접
  덮어써야 한다 (`:86-87`). `list_devices` 는 되는데 런치만 안 되던 이유가 이것

확인법: `ros2 run openni2_camera list_devices` 가 장치 Uri 와 시리얼을 뱉으면
성공 (`:89`).

## Consequences

**토픽 이름이 `orbbec_camera` 와 다르다 — `color` 가 아니라 `rgb` 다.**

```
/camera/rgb/image_raw   /camera/depth/image   /camera/depth_raw/image   /camera/ir/image_raw
```

`guide_node.py` 가 `/camera/rgb/image_raw{,/compressed}` 를 구독하는
근거가 이것이다. 실측 RGB 640×480 @ 30.1 Hz.

**lazy 발행이다.** 구독자가 붙어야 스트림이 돈다. 그래서
`guide.launch.py:58-59` 가 "guide_node 가 구독하므로 순서는 상관없다" 고
적어 뒀다. 반대로 `camera_info` 도 구독자가 없으면 안 나온다 —
카메라 HFOV 57.86도를 K 행렬에서 뽑을 때 이 점이 걸린다
(`sim/jongky_rl/README.md:80-88`).

**컨테이너의 cv_bridge 는 여전히 못 쓴다.** numpy 2.0 ABI 충돌이라
`guide_node.py::_latest_jpeg()` 가 압축 토픽을 우선하고, 원본으로 떨어질 때만
PIL 로 굽는다. `python3-pil` 은 `Dockerfile.robot:59` 에 있다.

**아직 안 한 것**

- **최소거리 실측.** 도구(`jongky_description/scripts/check_depth_min_range.py`)는
  있고 안 쟀다. 시뮬 `clipping_range` 하한이 0.1 m **가정값**이라, 실제가
  0.4 m 면 시뮬이 실물에 없는 근거리 정보를 정책에 주고 있는 것이다
- **뎁스가 Nav2 코스트맵에 안 들어간다.** 관측원은 `/scan` 하나뿐이다
  (`nav2_params.yaml:210-220`). `docs/mission.md` 3절의 "라이다가 못 보는
  높이" 문제가 여기서 안 풀린다

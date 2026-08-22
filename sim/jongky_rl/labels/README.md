# 라벨 파이프라인 (depth · 액션 · 보상)

Cosmos 클립과 실주행 bag 을 DreamerV3 가 먹을 수 있는 `(관측, 액션, 보상)`
시퀀스로 만드는 단계들. 설계 근거는 `~user/Documents/종키/라벨-파이프라인.md`.

## 단계와 상태

| 단계 | 도구 | 입력 | 출력 | 상태 |
|---|---|---|---|---|
| 0 출처 인덱스 | `build_index.py` | cosmos_out | `index.json` (80클립 ↔ 씨앗 ↔ QC ↔ 갈래) | **완료** (71/3/6 = 검수결과 일치) |
| 1 실물 리라벨 | `relabel_bag.py` | 카메라 bag | 에피소드 npz (보상 재계산) | **완료 8/22** — 62GB(jetson_archive_20260819)에서 27에피소드 |
| 2 액션 라벨 | `action_labels.py seed` | 카메라 bag + index | 씨앗별 실측 액션·보상 npz | **완료 8/22** — 씨앗 16개, 60프레임 정렬. fragment 의미론 |
| 2' IDM 유사 라벨 | `action_labels.py idm-*` | 카메라 bag | 게이트 통과 시만 라벨 | 보류 — 리스타일(A) 결정 대기 |
| 3 depth 라벨 | `depth_labels.py` | 클립 mp4 | 프레임별 이격 json | **완료 8/22** — 74클립, 척도 100% |
| 4 버퍼 적재 | `pack_episodes.py` | index + 라벨 + mp4 | dreamer 에피소드 npz | 게이트 검증됨 — 리스타일(A) 배치가 나오면 개통 |

공용: `mcap_lite.py` (footer 없는 현장 bag 을 청크 단위로 읽는 리더 + CDR
디코더. TwistStamped/Odometry/LaserScan/Image — bags_0821 실데이터로 검증).

## 원칙 (월드모델-설계 7절 갈래와 1:1)

- **버퍼에는 실측 액션만.** 연속 생성 클립(현재 배치 전부)은 갈래 B
  (인코더 강건화) 전용이고 `pack_episodes.py` 가 적재를 거부한다.
- **보상은 한 곳에서.** `../reward_spec.py` 가 시뮬 env 와 리라벨러의 공용
  소스다. 계수를 바꾸려면 거기만 바꾼다.
- **IDM 은 게이트 뒤에.** 실주행 검증 MAE (v 0.05 m/s · ω 0.15 rad/s) 를
  못 넘으면 라벨이 아예 안 나온다. 넘어도 `--allow-idm` 없이는 적재 불가.

## 카메라 bag 이 오면 (62GB 원본)

```bash
# 1) 씨앗 구간 실측 라벨 — 프레임 수 경고가 없어야 정렬이 맞은 것
python3 action_labels.py seed --bag BAG.mcap --floor 10f
# 2) 실물 리플레이 에피소드 (미세조정용)
python3 relabel_bag.py BAG.mcap --odom-topic /odometry/filtered
#    (EKF 토픽이 bag 에 없으면 --odom-topic /odom)
# 3) depth 라벨 — 두 클립으로 척도 잔차부터
python3 depth_labels.py --limit 2
```

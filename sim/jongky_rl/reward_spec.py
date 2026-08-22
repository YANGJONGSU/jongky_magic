"""보상함수의 단일 소스.

시뮬(jongky_corridor_env)과 오프라인 리라벨러(labels/relabel_bag.py)가
**같은 상수, 같은 식**을 여기서 가져다 쓴다. DreamerV3 는 보상 헤드가
데이터에 적힌 보상을 회귀하므로, 시뮬 롤아웃과 실물 리플레이의 보상이
다른 식으로 계산되면 미세조정 때 보상 헤드가 두 분포 사이에서 찢어진다.
그래서 이 파일은 isaaclab 도 torch 도 import 하지 않는다 — 젯슨·노트북
어디서든 (numpy 만으로) 불러올 수 있어야 한다.

식은 xp 인자로 numpy / torch 를 받아 양쪽에서 동일하게 돈다.
둘의 일치 여부는 이 파일을 직접 실행하면 검사한다:

    python3 reward_spec.py        # numpy/torch 대조 + 형태 검사
"""

# ── 보상 계수 (jongky_corridor_env cfg 가 이 값을 그대로 쓴다) ────────────
REW_PROGRESS = 10.0     # 목표까지 거리 감소분에 곱함 (주 신호)
REW_GOAL = 50.0         # 도달 보너스
REW_COLLISION = -25.0   # 벽/장애물 접촉
REW_TIME = -0.02        # 스텝당 시간 패널티
REW_SPIN = -0.01        # |각속도| 에 곱하는 제자리 회전 억제

GOAL_RADIUS = 0.35      # 도달 판정 반경 [m]
ROBOT_HALF_WIDTH = 0.085  # footprint 실측 반폭 [m] (env cfg 는 여유 포함 0.09)

# ── 실차 한계 = 액션 정규화 스케일 ────────────────────────────────────────
# 정책 액션 [-1,1] × 이 값 = cmd_vel. 리플레이에 적는 액션도 같은 정규화를
# 거꾸로 쓴다 (cmd_vel ÷ 이 값). env 와 리라벨러가 여기서 같이 가져간다.
V_MAX = 0.40            # m/s
OMEGA_MAX = 1.50        # rad/s

# ── 근접 패널티 ───────────────────────────────────────────────────────────
# 충돌 항은 부딪혀야 켜진다. 실차에서는 스치기 직전이 이미 실패라서,
# 이격거리(clearance)에 연속으로 걸리는 항을 따로 둔다.
#
#   pen(c) = -PROX_K * clip((PROX_C0 - c) / PROX_C0, 0, 1)^2
#
#   · c >= C0 (0.35 m)  → 0. 일반 구간(폭 1.68) 중앙 주행은 건드리지 않고,
#     사물함 구간(폭 1.20) 중앙(이격 0.515)도 패널티 밖이다.
#   · c → 0 으로 갈수록 2차로 매끄럽게 커진다. 계단 함수면 경계에서
#     정책이 떨기 때문에 C1 연속인 형태를 쓴다.
#   · K = 0.4: 벽에서 0.10 m 로 붙으면 스텝당 -0.20 로, 최대 진행 보상
#     (v_max 0.40 × dt 1/30 × 10 = 0.133/스텝)을 넘는다. 즉 벽을 스치며
#     직진하는 해는 순보상이 음수가 되어 성립하지 않는다. c = 0.25 m 에서는
#     -0.033 으로 진행 보상의 25% — 스치지만 않으면 거의 안 보인다.
PROX_C0 = 0.35          # 이 이격 아래에서만 켜짐 [m]
PROX_K = 0.4            # 이격 0 일 때의 스텝당 패널티 크기

# ── 진행 항 클램프 ────────────────────────────────────────────────────────
# 스텝당 진행 보상의 물리 상한은 v_max × dt × k = 0.40 × (1/30) × 10 = 0.133.
# 그런데 corridor_50k 학습에서 한 스텝 −225 (거리 22.5 m 점프)가 1회 관측됐다
# — 충돌 직후 PhysX depenetration 이 로봇을 순간이동시킨 것으로, 42500 시점
# eval −742 이상치와 같은 서명이다. 물리적으로 불가능한 값만 자르도록
# 한계의 ~7배로 클램프한다. 정상 학습 신호는 건드리지 않는다.
PROGRESS_CLAMP = 1.0    # 스텝당 진행 보상 절대값 상한


def clamp_progress(progress, xp):
    return xp.clip(progress, -PROGRESS_CLAMP, PROGRESS_CLAMP)


# ── 실물 리라벨 전용 (시뮬은 기하 판정을 쓰므로 무관) ─────────────────────
SCAN_FRONT_HALF_ANGLE = 1.5708   # 전방 ±90° 만 본다 [rad]
SCAN_COLLISION_RANGE = 0.14      # 스캔 최소거리가 이 아래면 접촉으로 라벨 [m]


def proximity_penalty(clearance, xp):
    """이격거리 → 스텝당 패널티(음수). xp 는 numpy 또는 torch 모듈.

    clearance 는 '가장 가까운 장애물까지 거리 - 로봇 반폭'.
    시뮬은 복도 반폭 - |y| - 반폭(기하), 실물은 /scan 전방 최소거리 - 반폭.
    두 정의가 완전히 같지는 않다(시뮬은 측방만, 스캔은 전방도 본다).
    복도 환경에서는 벽이 지배적이라 차이가 작지만, 미세조정 로그에서
    보상 헤드 잔차가 크면 여기부터 의심할 것.
    """
    x = (PROX_C0 - clearance) / PROX_C0
    x = xp.clip(x, 0.0, 1.0)
    return -PROX_K * x * x


def _selftest():
    import numpy as np
    cs = np.array([0.0, 0.05, 0.10, 0.25, 0.35, 0.515, 1.0])
    pn = proximity_penalty(cs, np)
    assert pn[0] == -PROX_K, "이격 0 에서 -K"
    assert pn[4] == 0.0 and pn[5] == 0.0 and pn[6] == 0.0, "C0 이상은 0"
    assert all(pn[i] <= pn[i + 1] for i in range(len(cs) - 1)), "단조"
    # 벽 스치기(0.10 m)가 최대 진행 보상보다 아파야 설계가 성립한다
    assert -pn[2] > 0.40 * (1.0 / 30.0) * REW_PROGRESS, "0.10 m 에서 진행보상 초과"
    try:
        import torch
        pt = proximity_penalty(torch.tensor(cs), torch)
        assert np.allclose(pt.numpy(), pn), "torch/numpy 불일치"
        print("selftest OK (numpy+torch):", dict(zip(cs.tolist(), np.round(pn, 4).tolist())))
    except ImportError:
        print("selftest OK (numpy only):", dict(zip(cs.tolist(), np.round(pn, 4).tolist())))


if __name__ == "__main__":
    _selftest()

# 종키프로 복도 주행 환경 — 실측 지도에서 뽑은 복도 형상 판
#
# jongky_corridor_env.py 는 복도를 corridor_width = 2.4 스칼라 하나로 세운다.
# 실측은 개방 구간 1.68~1.70 m, 사물함 구간 1.20 m 다. 2.4 m 는 개방 구간
# 대비 +42%, 사물함 구간 대비 +100% 다. 게다가 실제 복도는 한 숫자가 아니라
# 구간마다 폭이 바뀌는 이봉 구조라 스칼라로는 표현 자체가 안 된다.
#
# 이 파일은 map_geometry.py 가 점유 격자 지도에서 뽑아 놓은 폭 프로파일
# (JSON) 을 읽어 구간별로 폭이 바뀌는 복도를 세운다. 기존 파일은 건드리지
# 않는다 — JongkyCorridorEnv 를 상속해서 씬 구성과 벽 충돌 판정만 갈아끼운다.
# 그래서 둘을 나란히 두고 비교 학습할 수 있다.
#
# ── 좌표계: 왜 복도를 펴서 쓰는가 ─────────────────────────────────────────
# 실제 복도는 굽어 있지만 여기서는 중심선 호길이를 x 축으로 삼아 곧게 편다.
#   · 상속하는 부모 환경의 목표 샘플링·리셋·보상이 전부 "x 가 전방" 을 전제한다
#   · 이번에 고치려는 것은 폭 프로파일이지 곡률이 아니다.
#     42% 틀린 것은 폭이었다
#   · 종키 복도는 실제로 거의 직선 구간이 길다 (L10b 척추 36 m 중
#     7.35 m 짜리 직선 개방 구간이 통째로 들어 있다)
# 곡률까지 필요해지면 sections 대신 walls (월드 좌표 벽 선분) 를 쓰면 된다.
# JSON 에 같이 들어 있다.
#
# ── 실행 ──────────────────────────────────────────────────────────────────
#   # 1) 매핑 PC 에서 형상 추출 (Isaac 필요 없음)
#   python3 map_geometry.py /root/maps_local/L10b.yaml -o corridor_L10b.json
#   # 2) Isaac Lab 쪽에서 학습 — 이 env 가 기본값이다
#   python3 train_dreamer.py --headless --enable_cameras
#   # 다른 지도로:
#   python3 train_dreamer.py --headless --enable_cameras --geometry-json corridor_L11.json
#
# 예전에 여기 적혀 있던 JONGKY_CORRIDOR_JSON=... 은 없앴다. 그 환경변수는 이
# 파일의 DEFAULT_JSON 에서만 읽혔고 train_dreamer.py 는 이 모듈을 import 조차
# 하지 않았다 — 그래서 저 명령은 **에러 없이 폭 2.4 m 복도로 학습을 돌렸다.**

from __future__ import annotations

import json
import os
from collections.abc import Sequence
from dataclasses import dataclass

import torch

import isaaclab.sim as sim_utils
from isaaclab.utils import configclass

from jongky_corridor_env import (
    JongkyCorridorEnv,
    JongkyCorridorEnvCfg,
)

# 외부 실측값 — map_geometry.py 와 같은 값을 쓴다.
MEASURED_OPEN = 1.69        # 개방 구간 1.68~1.70 의 중앙
MEASURED_LOCKER = 1.20      # 사물함 구간

DEFAULT_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "corridor_L10b.json")


# ── 지도별 학습 대역 ───────────────────────────────────────────────────────
# 어느 구간을 학습에 쓸지는 지도의 성질이 아니라 **커리큘럼 결정**이다
# (goal_x_range 3~7 m 와 에피소드 20초에 맞춰 고른다). 그래서 map_geometry.py
# 가 뽑는 JSON 에 넣지 않는다 — 그 JSON 은 추출을 다시 돌릴 때마다 덮어써진다.
# 대신 지도 이름(meta.map_name)으로 여기서 찾는다.
#
# 고르는 기준 셋:
#   1) wall_backed_frac >= 0.5 인 구간만으로 끊기지 않고 이어질 것
#   2) **실제로 주행하는 x 0 ~ goal_x_range[1] (=7 m) 안에** 개방(1.69) 대역과
#      사물함(1.20) 대역이 둘 다 들어올 것. 로봇은 x∈[0,1] 에서 출발해
#      x∈[3,7] 의 목표까지만 간다 — 복도 저 끝의 사물함 구간은 한 번도 안 겪는다
#   3) 가장 먼 목표 뒤로 벽이 더 남을 것 (카메라 화면이 복도로 유지되게)
TRAINING_S_RANGE = {
    # 사물함 1.20 구간(s 16.45~19.55)이 x 3.00~6.10 으로 목표 대역 한복판에 온다.
    "L10b": (13.45, 31.00),
    # s 7.65~11.00 은 폭 3.4~3.5 m 인데 2*DT 추정치와 레이캐스트가 0.95~0.99 m
    # 어긋나고 구간 내 p90-p10 이 1.10 m 다 (정상 복도 구간은 각각 0.03~0.07,
    # 0.10). 교차로/홀이지 복도가 아니다 — 거리변환은 열린 방에서 방 크기를
    # 재지 복도 폭을 재지 않는다. 그 뒤인 11.00 에서 시작하면 사물함 1.20
    # 구간(s 16~18)이 x 5.00~7.00 으로 목표 대역에 들어온다.
    # 4.20 이나 2.80 에서 시작하면 주행 구간 폭열이 1.69 → 3.40 → 3.54 로 끝나
    # **사물함 대역을 한 번도 못 만난다** — 그게 wall_backed 로만 고른 답이다.
    "L11": (11.00, 21.10),
}


def resolve_s_range(doc, s_range):
    """corridor_s_range 값을 실제 (lo, hi) 로 바꾼다.

    "auto"   지도 이름으로 TRAINING_S_RANGE 에서 찾는다. **없으면 에러다.**
             다른 지도의 대역을 조용히 적용하는 것이 이 파일의 원래 버그였다 —
             L10b 전용 (13.45, 31.0) 이 L11 에 그대로 걸리면 25.5 m 중
             7.65 m 만 남고 잘린다. 잘렸다는 것은 로그에도 안 나온다.
    None     척추 전체.
    (lo, hi) 그대로.
    """
    if s_range is None or isinstance(s_range, (tuple, list)):
        return tuple(s_range) if s_range is not None else None
    if s_range != "auto":
        raise ValueError(f"corridor_s_range 는 'auto' | None | (lo, hi) — 받은 값: {s_range!r}")
    name = doc.get("meta", {}).get("map_name")
    if name not in TRAINING_S_RANGE:
        raise RuntimeError(
            f"지도 '{name}' 의 학습 대역이 정해져 있지 않다.\n"
            f"  jongky_map_corridor_env.TRAINING_S_RANGE 에 추가하거나 "
            f"train_dreamer.py --s-range LO:HI 로 직접 줄 것 (전체는 --s-range full).\n"
            f"  아는 지도: {', '.join(sorted(TRAINING_S_RANGE))}"
        )
    return TRAINING_S_RANGE[name]


# ── 형상 로드 ──────────────────────────────────────────────────────────────
@dataclass
class Section:
    x0: float
    x1: float
    width: float
    source_width: float     # 지도에서 나온 원래 값 (스냅 전)
    snapped: bool


def load_sections(json_path: str, s_range="auto", width_source: str = "measured",
                  snap_tol: float = 0.15, min_width: float = 0.60):
    """추출 JSON → 시뮬 복도 구간 목록.

    width_source
      "map"      지도에서 나온 폭을 그대로 쓴다.
      "measured" 실측값(1.69 / 1.20) 에 snap_tol 안으로 가까운 구간은
                 실측값으로 스냅한다. 나머지는 지도 값을 쓴다.

    왜 "measured" 가 기본인가:
      지도는 5 cm 격자다. 1.69 m 를 5 cm 격자로 재면 원리적으로 ±0.05 m
      (±3%) 아래로는 못 내려간다. 게다가 점유 격자는 빔이 닿은 셀을
      점유로 찍기 때문에 벽면이 복도 안쪽으로 반 셀~한 셀 먹고 들어가서
      자유 폭이 계통적으로 좁게 나온다. 실제로 20개 지도판 전부에서
      개방 구간이 1.57~1.70 (중앙 약 1.60) 으로 -5% 쯤 일관되게 낮게 나왔다.
      줄자 실측이 5 cm 격자보다 정확하므로, 실측값이 있는 대역은
      실측값을 쓰는 것이 맞다. 다만 무엇을 갈아끼웠는지 로그로 남긴다.
    """
    with open(json_path) as f:
        doc = json.load(f)

    s_range = resolve_s_range(doc, s_range)
    secs = doc["sections"]
    if s_range is not None:
        lo, hi = s_range
        secs = [s for s in secs if s["s1"] > lo and s["s0"] < hi]
        secs = [dict(s, s0=max(s["s0"], lo), s1=min(s["s1"], hi)) for s in secs]

    # 벽이 뒷받침되지 않는 구간(미탐사 영역에 닿은 구간)은 폭이 아니라
    # "지도가 끝난 곳까지의 거리" 라서 복도 폭으로 쓰면 안 된다. 버린다.
    secs = [s for s in secs if s.get("wall_backed_frac", 1.0) >= 0.5]
    secs = [s for s in secs if s["width"] >= min_width]
    if not secs:
        raise RuntimeError(f"{json_path}: 쓸 수 있는 구간이 없다 (벽 뒷받침/최소폭 조건)")

    # 버려진 구간이 **가운데** 있으면 x 축에 구멍이 난다. 그 구간에는 벽이 안
    # 세워지는데 _half_width_at 은 bucketize 로 바로 앞 구간의 반폭을 그대로
    # 돌려준다 — "벽은 없는데 충돌 판정은 있는" 구간이 생기고, 카메라는 그리로
    # 빈 공간을 본다. L10b/L11 은 지금 버려지는 구간이 양 끝뿐이라 안 걸리지만,
    # 다음 지도에서 조용히 걸리면 원인을 찾기 어렵다.
    for _a, _b in zip(secs, secs[1:]):
        if _b["s0"] - _a["s1"] > 1e-6:
            raise RuntimeError(
                f"{json_path}: s {_a['s1']:.2f}~{_b['s0']:.2f} m 가 비었다 "
                f"(벽 뒷받침 없는 구간이 가운데 있다). s_range 를 그 앞이나 뒤로 좁힐 것."
            )

    x0 = secs[0]["s0"]
    out = []
    for s in secs:
        w = float(s["width"])
        snapped = False
        if width_source == "measured":
            for target in (MEASURED_LOCKER, MEASURED_OPEN):
                if abs(w - target) <= snap_tol:
                    w, snapped = target, True
                    break
        elif width_source != "map":
            raise ValueError(f"width_source 는 'map' 또는 'measured' — 받은 값: {width_source}")
        out.append(Section(x0=s["s0"] - x0, x1=s["s1"] - x0,
                           width=w, source_width=float(s["width"]), snapped=snapped))
    return out, doc


# ── 설정 ───────────────────────────────────────────────────────────────────
@configclass
class JongkyMapCorridorEnvCfg(JongkyCorridorEnvCfg):
    """실측 지도 형상으로 복도를 세우는 설정. 나머지는 부모와 동일하다."""

    # map_geometry.py 산출 JSON
    geometry_json: str = DEFAULT_JSON

    # 척추 호길이 중 어느 구간을 쓸지 (m).
    #   "auto"   지도 이름(meta.map_name)으로 TRAINING_S_RANGE 조회 — 기본값
    #   None     척추 전체
    #   (lo, hi) 직접 지정
    # 여기에 숫자를 직접 박으면 안 된다. 예전 기본값 (13.45, 31.0) 은 L10b
    # 전용 수치였는데 다른 지도에도 그대로 걸려서, L11 을 넣으면 25.5 m 중
    # 7.65 m 만 남기고 조용히 잘라 냈다.
    corridor_s_range: tuple | str | None = "auto"

    width_source: str = "measured"   # "measured" | "map"
    snap_tol: float = 0.15

    # 목표 y 를 복도 반폭에서 이만큼 안쪽으로 제한한다.
    # 1.20 m 구간의 반폭은 0.60 이고 로봇 반폭이 0.085 라서,
    # 부모의 goal_y_range (-0.6, 0.6) 를 그대로 쓰면 목표가 벽 안에 박힌다.
    goal_y_margin: float = 0.25

    # 벽 렌더링
    wall_height: float = 2.5
    wall_thickness: float = 0.10

    # ── 부모의 스칼라 복도 설정은 여기서 죽어 있다 ─────────────────────────
    # 부모는 복도를 corridor_length/corridor_width 두 스칼라로 세우는데, 그 두
    # 값을 읽는 곳은 부모의 _setup_scene 과 _get_dones 뿐이고 이 env 는 둘 다
    # 갈아끼운다. 상속만 하고 한 번도 안 읽는다.
    #
    # 주석만 달아 두면 다음 사람이 corridor_width 를 1.69 로 고쳐 놓고 왜 안
    # 바뀌냐고 헤맨다 — 주석은 그 사람이 이미 안 읽은 것이다. 그래서 값 자체를
    # None 으로 덮어쓴다. 혹시라도 읽는 코드가 생기면 조용히 2.4 m 복도를
    # 세우는 대신 그 자리에서 터진다.
    # 폭은 geometry_json 의 구간별 값에서, 길이는 마지막 구간 끝에서 온다.
    corridor_length: float | None = None
    corridor_width: float | None = None


# ── 환경 ───────────────────────────────────────────────────────────────────
class JongkyMapCorridorEnv(JongkyCorridorEnv):
    """폭이 구간마다 바뀌는 실측 복도.

    부모와 다른 것은 딱 두 가지다.
      _setup_scene  벽 두 장 → 구간별 벽 상자 여러 장
      _get_dones    스칼라 반폭 → x 위치별 반폭 조회
    관측·행동·보상·로봇 설정은 전부 부모 그대로다.
    """

    cfg: JongkyMapCorridorEnvCfg

    def __init__(self, cfg: JongkyMapCorridorEnvCfg, render_mode: str | None = None, **kwargs):
        self._sections, self._geom_doc = load_sections(
            cfg.geometry_json, cfg.corridor_s_range, cfg.width_source, cfg.snap_tol)
        self._corridor_length = self._sections[-1].x1

        # 목표가 복도 밖으로 나가지 않게 부모 설정을 좁힌다.
        narrowest = min(s.width for s in self._sections)
        y_lim = max(0.05, narrowest * 0.5 - cfg.goal_y_margin)
        cfg.goal_y_range = (-y_lim, y_lim)

        # 목표 x 는 복도 안에 있어야 한다. 부모 기본값 (3,7) 을 복도 길이로 자른다.
        gx0 = min(cfg.goal_x_range[0], self._corridor_length - 1.0)
        gx1 = min(cfg.goal_x_range[1], self._corridor_length - 0.5)
        cfg.goal_x_range = (gx0, gx1)

        # super().__init__ 전에 찍는다 — 씬 구성이 무거워서, 형상이 틀렸으면
        # Isaac 이 뜨기 전에 눈에 보이는 편이 낫다. 이 시점엔 self.cfg 가
        # 아직 없으므로 cfg 를 인자로 넘긴다.
        self._log_geometry(cfg)
        super().__init__(cfg, render_mode, **kwargs)

        # x → 구간 조회용 텐서 (경계, 반폭)
        self._sec_x0 = torch.tensor([s.x0 for s in self._sections], device=self.device)
        self._sec_half = torch.tensor([s.width * 0.5 for s in self._sections], device=self.device)

    # ── 로그 ───────────────────────────────────────────────────────────────
    def _log_geometry(self, cfg: JongkyMapCorridorEnvCfg):
        src = self._geom_doc.get("meta", {}).get("map_name", "?")
        print(f"[jongky] 복도 형상: {cfg.geometry_json}  (지도 {src})")
        print(f"[jongky]   길이 {self._corridor_length:.2f} m, 구간 {len(self._sections)} 개, "
              f"width_source={cfg.width_source}")
        for s in self._sections:
            tag = f"  ← 지도 {s.source_width:.3f} 에서 실측으로 스냅" if s.snapped else ""
            print(f"[jongky]   x {s.x0:6.2f}~{s.x1:6.2f} m   폭 {s.width:.3f} m{tag}")
        narrow = min(self._sections, key=lambda s: s.width)
        half = narrow.width * 0.5
        print(f"[jongky]   최협 {narrow.width:.3f} m → 로봇 반폭 {cfg.robot_half_width} 기준 "
              f"측면 여유 {half - cfg.robot_half_width:.3f} m")
        print(f"[jongky]   목표 범위 x{cfg.goal_x_range} y{cfg.goal_y_range}")
        print("[jongky]   (부모의 corridor_width/corridor_length 는 이 env 에서 안 쓴다 — None)")

    # ── 씬 구성 ────────────────────────────────────────────────────────────
    def _setup_scene(self):
        # 부모와 같은 순서로 만든다 (로봇 → 카메라 → 마커 → 바닥 → 벽 → clone)
        from isaaclab.assets import Articulation, RigidObject
        from isaaclab.sensors import TiledCamera

        self._robot = Articulation(self.cfg.robot_cfg)
        self._camera = TiledCamera(self.cfg.tiled_camera)
        self._marker = RigidObject(self.cfg.marker_cfg)

        ground = sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=self.cfg.ground_static_friction,
                dynamic_friction=self.cfg.ground_dynamic_friction,
                restitution=0.0,
            )
        )
        ground.func("/World/ground", ground)

        # 구간별 벽 상자. env 복제 전에 템플릿 경로(env_0)에 만들어야 같이 복제된다.
        th = self.cfg.wall_thickness
        h = self.cfg.wall_height
        for i, s in enumerate(self._sections):
            length = s.x1 - s.x0
            if length <= 0:
                continue
            cx = 0.5 * (s.x0 + s.x1)
            half = s.width * 0.5
            box = sim_utils.CuboidCfg(
                size=(length, th, h),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.7, 0.68)),
            )
            for side, y in (("l", half + th * 0.5), ("r", -(half + th * 0.5))):
                box.func(f"/World/envs/env_0/wall_{side}_{i:02d}", box,
                         translation=(cx, y, h * 0.5))

        # 폭이 바뀌는 경계를 막는 연결 판.
        #
        # 이게 없으면 넓어지는 쪽 경계에 벽이 통째로 비는 틈이 생긴다.
        # 물리적으로도 틀리지만(로봇이 그 틈으로 빠진다) 관측이 픽셀이라
        # 더 나쁘다 — 카메라가 그 틈으로 빈 공간(스카이박스)을 보게 되고
        # 실제 복도에는 없는 특징을 정책이 학습해 버린다.
        # 실제로 이 경계는 사물함 열이 시작/끝나는 면이라 막혀 있는 것이 맞다.
        for i in range(len(self._sections) - 1):
            a, b = self._sections[i], self._sections[i + 1]
            ha, hb = a.width * 0.5, b.width * 0.5
            if abs(ha - hb) < 1e-6:
                continue
            lo, hi = sorted((ha, hb))
            depth = (hi - lo) + th          # 얇은 쪽 벽 두께까지 덮어 틈을 없앤다
            cy = 0.5 * (lo + hi) + th * 0.5
            panel = sim_utils.CuboidCfg(
                size=(th, depth, h),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.66, 0.66, 0.64)),
            )
            for side, sgn in (("l", 1.0), ("r", -1.0)):
                panel.func(f"/World/envs/env_0/step_{side}_{i:02d}", panel,
                           translation=(a.x1, sgn * cy, h * 0.5))

        # 복도 양 끝을 막아 카메라 화면이 복도로 유지되게 한다.
        endw = max(s.width for s in self._sections) + 2 * th
        cap = sim_utils.CuboidCfg(
            size=(th, endw, h),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.65, 0.65, 0.63)),
        )
        for name, x in (("cap_back", -th * 0.5), ("cap_front", self._corridor_length + th * 0.5)):
            cap.func(f"/World/envs/env_0/{name}", cap, translation=(x, 0.0, h * 0.5))

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self.scene.articulations["robot"] = self._robot
        self.scene.sensors["front_cam"] = self._camera
        self.scene.rigid_objects["goal_marker"] = self._marker

        light_cfg = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.88))
        light_cfg.func("/World/Light", light_cfg)

    # ── 위치별 복도 반폭 ───────────────────────────────────────────────────
    def _half_width_at(self, x: torch.Tensor) -> torch.Tensor:
        """x (env 로컬) 에서의 복도 반폭. 복도 밖이면 양 끝 구간 값을 쓴다."""
        idx = torch.bucketize(x.contiguous(), self._sec_x0, right=True) - 1
        idx = idx.clamp(0, len(self._sections) - 1)
        return self._sec_half[idx]

    # ── 보상 ───────────────────────────────────────────────────────────────
    def _clearance(self) -> torch.Tensor:
        """부모는 복도 폭 스칼라 하나로 계산한다. 여기서는 로봇 x 위치의
        실제 반폭을 쓴다 — 사물함 구간에 들어가면 근접 패널티가 자동으로
        일찍 켜진다. _get_rewards 자체는 부모 것을 그대로 쓴다."""
        xy = self._robot_xy()
        return self._half_width_at(xy[:, 0]) - torch.abs(xy[:, 1]) - self.cfg.robot_half_width

    # ── 종료 ───────────────────────────────────────────────────────────────
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy = self._robot_xy()
        dist = torch.norm(self._goal - xy, dim=-1)

        # 부모는 corridor_width 스칼라 하나로 판정한다. 여기서는 로봇이 있는
        # x 위치의 실제 복도 반폭을 찾아서 판정한다 — 사물함 구간에 들어가면
        # 판정선이 자동으로 좁아진다.
        limit = self._half_width_at(xy[:, 0]) - self.cfg.robot_half_width
        self._collided = torch.abs(xy[:, 1]) > limit

        upside = self._robot.data.projected_gravity_b[:, 2] > -0.5
        terminated = self._collided | upside | (dist < self.cfg.goal_radius)
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    # ── 리셋 ───────────────────────────────────────────────────────────────
    def _reset_idx(self, env_ids: Sequence[int] | None):
        super()._reset_idx(env_ids)
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES

        # 부모가 뽑은 목표를 그 x 위치의 실제 복도 폭 안으로 다시 밀어 넣는다.
        # 부모는 복도가 어디서 좁아지는지 모르기 때문에 이 보정이 필요하다.
        gx = self._goal[env_ids, 0]
        lim = (self._half_width_at(gx) - self.cfg.goal_y_margin).clamp(min=0.05)
        self._goal[env_ids, 1] = torch.clamp(self._goal[env_ids, 1], -lim, lim)

        # 시작 위치도 마찬가지로 복도 안에 있어야 한다.
        root_xy = self._robot.data.root_pos_w[env_ids, :2] - self.scene.env_origins[env_ids, :2]
        slim = (self._half_width_at(root_xy[:, 0]) - self.cfg.robot_half_width - 0.05).clamp(min=0.02)
        new_y = torch.clamp(root_xy[:, 1], -slim, slim)
        if not torch.allclose(new_y, root_xy[:, 1]):
            pose = self._robot.data.root_state_w[env_ids, :7].clone()
            pose[:, 1] = new_y + self.scene.env_origins[env_ids, 1]
            self._robot.write_root_pose_to_sim(pose, env_ids)
            root_xy = root_xy.clone()
            root_xy[:, 1] = new_y

        # 목표 마커를 보정된 목표 위치로 다시 옮긴다.
        mk = self._marker.data.default_root_state[env_ids].clone()
        mk[:, 0] = self._goal[env_ids, 0]
        mk[:, 1] = self._goal[env_ids, 1]
        mk[:, 2] = self.cfg.marker_height * 0.5
        mk[:, :3] += self.scene.env_origins[env_ids]
        self._marker.write_root_pose_to_sim(mk[:, :7], env_ids)

        # 목표가 움직였으므로 진행 보상의 기준 거리를 다시 잡는다.
        # 이걸 빼먹으면 첫 스텝에 가짜 progress 보상이 튄다.
        self._prev_dist[env_ids] = torch.norm(self._goal[env_ids] - root_xy, dim=-1)

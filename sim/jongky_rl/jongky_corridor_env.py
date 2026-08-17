# 종키프로 복도 주행 환경 (Isaac Lab DirectRLEnv)
#
# 관측: 전면 아스트라 시점 RGB 64x64  (DreamerV3 표준 해상도)
# 행동: cmd_vel 연속 2차원 [v, omega] — 실차 한계값으로 스케일
# 보상: 목표까지 거리 감소 + 도달 보너스 - 충돌 - 시간
#
# [실차 일치 항목]  이 값들이 어긋나면 sim2real 이 통째로 날아간다.
#   v_max 0.40 m/s · omega_max 1.50 rad/s · a_max 0.30 m/s^2
#   wheel_radius 0.0335 m · wheel_separation 0.11909 m
#   MARS actor_phase15 가 종키에 못 올라간 이유가 정확히 이 불일치였다
#   (MAX_VX 1.5 대 실차 0.40, 3.75배).
#
# [카메라] URDF 변환 때 fixed joint 를 병합하지 않았으므로 camera_link 프레임이
#   USD 에 그대로 살아 있다. 카메라를 그 prim 아래에 붙이면 실측 장착 위치
#   (base_link 기준 x=0.07, z=0.1656)가 자동으로 맞는다.

from __future__ import annotations

import math
import os
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCamera, TiledCameraCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import sample_uniform

# 병합(--merge-joints) 버전을 쓴다.
#
# 병합 안 한 USD 는 프레임 전용 링크(camera_link·imu_link·laser·rear_camera_link·
# tof_l/r_link·base_footprint)에 <inertial> 이 없어서 임포터가 각각 1.0 kg 을
# 박는다 → 2.603 kg 로봇이 9.6 kg (3.7배) 이 되고, 바퀴 토크가 포화되며
# 제자리에서 진동한다. 모르고 지나가면 sim2real 이 통째로 날아가는 버그다.
#
# 병합하면 질량이 정확히 2.603 kg 이 되고 (base_footprint 2.503 + 바퀴 0.05x2),
# 강체도 10개 → 3개로 줄어 물리가 빨라진다. 그러면서
# camera_link 같은 센서 프레임은 Xform prim 으로 그대로 남아 카메라를 붙일 수 있다.
#
# 주의: 변환 시 "Merging bodies with inertia is deprecated" 경고가 뜬다.
#       Isaac Lab 을 올릴 때 깨지면 URDF 프레임 링크에 미소 관성을 넣는 쪽으로 간다.
#
# 경로는 JONGKY_USD 환경변수로 덮어쓸 수 있다.
JONGKY_USD = os.environ.get("JONGKY_USD", os.path.expanduser("~/jongky_usd_merged/jongky.usd"))

# ── 실차 상수 ──────────────────────────────────────────────────────────────
WHEEL_RADIUS = 0.0335       # m
WHEEL_SEPARATION = 0.11909  # m — 제자리 5바퀴 회전 3회 평균으로 확정한 값
V_MAX = 0.40                # m/s
OMEGA_MAX = 1.50            # rad/s
A_MAX = 0.30                # m/s^2  (액션 램프 제한)


@configclass
class JongkyCorridorEnvCfg(DirectRLEnvCfg):
    # ── 에피소드 ───────────────────────────────────────────────────────────
    decimation = 4
    episode_length_s = 60.0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # ── 로봇 ───────────────────────────────────────────────────────────────
    # 바퀴는 속도 제어. USD 변환 때 stiffness 0 / damping 100 으로 넣어 뒀고
    # 여기서 한 번 더 명시한다.
    # 목표 마커 (설정값은 아래 "목표 마커" 절 참조).
    # 충돌 프로퍼티를 안 주므로 순수 시각 물체이고, kinematic 이라 물리에 안 밀린다.
    marker_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/envs/env_.*/GoalMarker",
        spawn=sim_utils.CylinderCfg(
            radius=0.15,
            height=1.60,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.35, 0.05), emissive_color=(0.30, 0.10, 0.0)
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(10.0, 0.0, 0.80)),
    )

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=JONGKY_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                # 0 이면 속도 제약이 제대로 안 풀려 바퀴 속도가 진동한다.
                # Isaac 자체 경고에도 1~2 이상으로 올리라고 나온다.
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.05),
            joint_pos={"l_wheel_joint": 0.0, "r_wheel_joint": 0.0},
            joint_vel={"l_wheel_joint": 0.0, "r_wheel_joint": 0.0},
        ),
        actuators={
            # *_sim 접미사를 쓴다. effort_limit / velocity_limit 은 deprecated 이고
            # 특히 velocity_limit 은 implicit actuator 에서 그냥 무시된다.
            #
            # [게인] 바퀴는 0.05 kg 이다. 2.603 kg 을 0.30 m/s^2 로 가속하는 데
            #   필요한 바퀴 토크는 약 0.013 N·m 뿐이다. damping 을 100 으로 두면
            #   속도 오차 0.1 rad/s 에 10 N·m 를 때려 즉시 포화되고, 토크가
            #   +10 <-> -10 으로 뒤집히는 뱅뱅 진동이 난다. 실제 모터 규모에 맞춘다.
            # [armature] 차체 2.503 kg 대 바퀴 0.05 kg — 질량비 50:1 이다.
            #   가벼운 바퀴가 50배 무거운 몸통의 접촉 임펄스를 받으면 PhysX
            #   솔버가 발산한다 (바퀴가 목표 11.94 를 벗어나 ±35 rad/s 로 날뛴다).
            #   armature 는 조인트 자유도에 가상 관성을 더해 이걸 잡는다.
            #   로봇의 실제 질량은 안 건드리므로 sim2real 에 영향이 없다.
            #   바퀴 고유 관성이 2.2e-5 kg·m^2 이므로 1e-3 은 그보다 충분히 크다.
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["l_wheel_joint", "r_wheel_joint"],
                effort_limit_sim=0.5,
                velocity_limit_sim=V_MAX / WHEEL_RADIUS * 2.0,
                stiffness=0.0,
                damping=2.0,
                armature=1e-3,
            ),
        },
    )

    # ── 카메라 ─────────────────────────────────────────────────────────────
    # camera_link 프레임 아래에 붙인다 → 실측 장착 위치가 자동 반영된다.
    # convention="ros" 로 두어 REP-103 (x 전방) 과 맞춘다.
    #
    # spawn 을 주어 camera_link 의 자식으로 새로 만든다. 부모 변환을 상속하므로
    # offset 은 0 이면 되고, 장착 위치는 URDF 실측값이 그대로 반영된다.
    #
    # TODO: focal_length 를 아스트라 실측 FOV 로 교체할 것. 아래는 HFOV 60도 가정
    #       (f = (aperture/2) / tan(HFOV/2) = 10.4775 / tan(30도)).
    #       clipping_range 하한도 최소거리 실측(check_depth_min_range.py) 후 교체.
    tiled_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="/World/envs/env_.*/Robot/base_footprint/base_link/camera_link/front_cam",
        offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.15,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
        width=64,
        height=64,
    )

    # ── 공간 ───────────────────────────────────────────────────────────────
    action_space = 2                                   # [v, omega], 각각 [-1, 1]
    observation_space = [64, 64, 3]                    # DreamerV3 표준
    state_space = 5                                    # 크리틱/디버그용 (목표거리, sin/cos 방위, v, omega)

    # ── 씬 ─────────────────────────────────────────────────────────────────
    # DreamerV3 는 sample-efficient 설계라 env 를 많이 안 띄운다.
    # Isaac Lab 기본 예제(PPO)는 수천 개지만 여기서는 4~16 이면 충분하고,
    # 카메라 렌더링 VRAM 도 16GB 에 그 정도까지만 들어간다.
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=8, env_spacing=30.0, replicate_physics=True)

    viewer = ViewerCfg(eye=(6.0, 6.0, 5.0), lookat=(6.0, 0.0, 0.0))

    # ── 바닥 마찰 ──────────────────────────────────────────────────────────
    # 캐스터가 URDF 상 base_link 에 고정된 구슬이라 시뮬에서 구르지 못하고
    # 미끄러진다. 마찰을 올리면 구동륜 접지는 좋아지지만 캐스터 저항도 커진다.
    # 두 효과 중 뭐가 지배적인지 실험으로 정한 값이 들어가야 한다.
    ground_static_friction = 1.0
    ground_dynamic_friction = 1.0

    # ── 복도 기하 ──────────────────────────────────────────────────────────
    corridor_length = 24.0
    corridor_width = 2.4        # "꽤 넓음" — 실측으로 교체할 것
    wall_height = 2.5
    wall_thickness = 0.1
    robot_half_width = 0.09     # footprint 반폭 0.085 에 여유

    # 목표 지점 (강의장 앞) 샘플 범위
    goal_x_range = (8.0, 20.0)
    goal_y_range = (-0.6, 0.6)
    goal_radius = 0.35          # 도달 판정. MARS warehouse_env 와 같은 값

    # ── 목표 마커 ──────────────────────────────────────────────────────────
    # 관측이 픽셀뿐이라 목표가 화면에 보이지 않으면 정책이 복도 직진밖에 못 배운다.
    # 실제 시나리오에서 강의장 문·표지판이 하는 역할을 시뮬에서 대신한다.
    #
    # 충돌을 끄고 kinematic 으로 둔다 — 로봇이 밀거나 부딪혀 넘어뜨리면
    # 안 되고, 도달 판정은 어차피 거리로 한다.
    #
    # 크기·색은 위 marker_cfg 의 spawn 에 있다. 여기 높이는 리셋 때 마커를
    # 바닥에 세우는 계산에만 쓰므로 marker_cfg 의 height 와 같이 바꿀 것.
    marker_height = 1.60        # 64x64 화면에서 멀리서도 보이도록 문 높이쯤

    # ── 보상 계수 ──────────────────────────────────────────────────────────
    rew_progress = 10.0         # 목표까지 거리 감소분에 곱함 (주 신호)
    rew_goal = 50.0             # 도달 보너스
    rew_collision = -25.0       # 벽 충돌
    rew_time = -0.02            # 스텝당 시간 패널티
    rew_spin = -0.01            # 제자리 회전 억제


class JongkyCorridorEnv(DirectRLEnv):
    """종키프로 복도 주행 — 전면 카메라 픽셀로 목표까지 간다."""

    cfg: JongkyCorridorEnvCfg

    def __init__(self, cfg: JongkyCorridorEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._wheel_idx, _ = self._robot.find_joints(["l_wheel_joint", "r_wheel_joint"])

        # 목표 지점 (env 로컬 좌표)
        self._goal = torch.zeros(self.num_envs, 2, device=self.device)
        self._prev_dist = torch.zeros(self.num_envs, device=self.device)
        # 가속 제한을 걸기 위해 직전 명령을 들고 있는다
        self._prev_cmd = torch.zeros(self.num_envs, 2, device=self.device)
        self._collided = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    # ── 씬 구성 ────────────────────────────────────────────────────────────
    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot_cfg)
        self._camera = TiledCamera(self.cfg.tiled_camera)
        self._marker = RigidObject(self.cfg.marker_cfg)

        # 바닥
        ground = sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=self.cfg.ground_static_friction,
                dynamic_friction=self.cfg.ground_dynamic_friction,
                restitution=0.0,
            )
        )
        ground.func("/World/ground", ground)

        # 복도 벽 두 장. env 복제 전에 템플릿 경로에 만들어야 같이 복제된다.
        half_w = self.cfg.corridor_width * 0.5
        wall = sim_utils.CuboidCfg(
            size=(self.cfg.corridor_length, self.cfg.wall_thickness, self.cfg.wall_height),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.7, 0.7, 0.68)),
        )
        for name, y in (("wall_l", half_w), ("wall_r", -half_w)):
            wall.func(
                f"/World/envs/env_0/{name}",
                wall,
                translation=(self.cfg.corridor_length * 0.5 - 2.0, y, self.cfg.wall_height * 0.5),
            )

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self.scene.articulations["robot"] = self._robot
        self.scene.sensors["front_cam"] = self._camera
        self.scene.rigid_objects["goal_marker"] = self._marker

        light_cfg = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.88))
        light_cfg.func("/World/Light", light_cfg)

    # ── 행동 ───────────────────────────────────────────────────────────────
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """[-1,1] 정규화 액션 → 실차 한계 안의 (v, omega) → 좌우 바퀴 각속도."""
        cmd = torch.tanh(actions.clone()) * torch.tensor([V_MAX, OMEGA_MAX], device=self.device)

        # 가속 제한. 실차 보드가 램프를 걸기 때문에 시뮬에서도 같이 건다.
        dv_max = A_MAX * self.cfg.sim.dt * self.cfg.decimation
        dv = torch.clamp(cmd[:, 0] - self._prev_cmd[:, 0], -dv_max, dv_max)
        cmd[:, 0] = self._prev_cmd[:, 0] + dv
        self._prev_cmd = cmd

        v, omega = cmd[:, 0], cmd[:, 1]
        half_l = WHEEL_SEPARATION * 0.5
        self._wheel_target = torch.stack(
            [(v - omega * half_l) / WHEEL_RADIUS, (v + omega * half_l) / WHEEL_RADIUS], dim=-1
        )

    def _apply_action(self) -> None:
        self._robot.set_joint_velocity_target(self._wheel_target, joint_ids=self._wheel_idx)

    # ── 관측 ───────────────────────────────────────────────────────────────
    def _get_observations(self) -> dict:
        rgb = self._camera.data.output["rgb"] / 255.0
        rgb = rgb - torch.mean(rgb, dim=(1, 2), keepdim=True)  # 채널별 평균 제거
        return {"policy": rgb.clone(), "critic": self._compute_state()}

    def _compute_state(self) -> torch.Tensor:
        """크리틱/디버그용 저차원 상태. 정책은 이걸 안 본다."""
        to_goal = self._goal - self._robot_xy()
        dist = torch.norm(to_goal, dim=-1)
        yaw = self._robot_yaw()
        bearing = torch.atan2(to_goal[:, 1], to_goal[:, 0]) - yaw
        lin = self._robot.data.root_lin_vel_b[:, 0]
        ang = self._robot.data.root_ang_vel_b[:, 2]
        return torch.stack([dist, torch.sin(bearing), torch.cos(bearing), lin, ang], dim=-1)

    def _robot_xy(self) -> torch.Tensor:
        return self._robot.data.root_pos_w[:, :2] - self.scene.env_origins[:, :2]

    def _robot_yaw(self) -> torch.Tensor:
        q = self._robot.data.root_quat_w  # (w, x, y, z)
        return torch.atan2(
            2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
            1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2),
        )

    # ── 보상 ───────────────────────────────────────────────────────────────
    def _get_rewards(self) -> torch.Tensor:
        dist = torch.norm(self._goal - self._robot_xy(), dim=-1)

        progress = (self._prev_dist - dist) * self.cfg.rew_progress
        self._prev_dist = dist

        reached = (dist < self.cfg.goal_radius).float() * self.cfg.rew_goal
        collision = self._collided.float() * self.cfg.rew_collision
        spin = torch.abs(self._robot.data.root_ang_vel_b[:, 2]) * self.cfg.rew_spin

        return progress + reached + collision + spin + self.cfg.rew_time

    # ── 종료 ───────────────────────────────────────────────────────────────
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy = self._robot_xy()
        dist = torch.norm(self._goal - xy, dim=-1)

        # 벽 충돌: 복도 반폭에서 로봇 반폭을 뺀 선을 넘으면 닿은 것으로 본다.
        # ContactSensor 대신 기하 판정 — 복도라 정확하고 훨씬 싸다.
        limit = self.cfg.corridor_width * 0.5 - self.cfg.robot_half_width
        self._collided = torch.abs(xy[:, 1]) > limit

        # 뒤집힘 판정.
        # 높이로 보면 안 된다 — 루트 링크가 base_footprint 라 정상 주행 중에도
        # z 가 0 근처다. 투영 중력의 z 성분을 본다 (똑바로 서 있으면 -1).
        upside = self._robot.data.projected_gravity_b[:, 2] > -0.5

        terminated = self._collided | upside | (dist < self.cfg.goal_radius)
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, time_out

    # ── 리셋 ───────────────────────────────────────────────────────────────
    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES
        super()._reset_idx(env_ids)
        n = len(env_ids)

        root = self._robot.data.default_root_state[env_ids].clone()
        root[:, 0] = sample_uniform(0.0, 1.0, (n,), self.device)
        root[:, 1] = sample_uniform(-0.4, 0.4, (n,), self.device)
        root[:, :3] += self.scene.env_origins[env_ids]

        # 시작 방향을 살짝 흔든다 (복도 축 ±20도)
        yaw = sample_uniform(-0.35, 0.35, (n,), self.device)
        root[:, 3] = torch.cos(yaw * 0.5)
        root[:, 4:6] = 0.0
        root[:, 6] = torch.sin(yaw * 0.5)

        self._robot.write_root_pose_to_sim(root[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(torch.zeros(n, 6, device=self.device), env_ids)

        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        self._goal[env_ids, 0] = sample_uniform(*self.cfg.goal_x_range, (n,), self.device)
        self._goal[env_ids, 1] = sample_uniform(*self.cfg.goal_y_range, (n,), self.device)

        # 목표 마커를 새 목표 위치로 옮긴다. 이게 있어야 정책이 픽셀만 보고도
        # 어디로 가야 하는지 알 수 있다.
        mk = self._marker.data.default_root_state[env_ids].clone()
        mk[:, 0] = self._goal[env_ids, 0]
        mk[:, 1] = self._goal[env_ids, 1]
        mk[:, 2] = self.cfg.marker_height * 0.5
        mk[:, :3] += self.scene.env_origins[env_ids]
        self._marker.write_root_pose_to_sim(mk[:, :7], env_ids)
        self._marker.write_root_velocity_to_sim(torch.zeros(n, 6, device=self.device), env_ids)

        self._prev_dist[env_ids] = torch.norm(
            self._goal[env_ids] - (root[:, :2] - self.scene.env_origins[env_ids, :2]), dim=-1
        )
        self._prev_cmd[env_ids] = 0.0
        self._collided[env_ids] = False

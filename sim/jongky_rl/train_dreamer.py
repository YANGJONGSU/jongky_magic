"""dreamerv3-torch(NM512) 로 종키 복도 주행 정책을 학습한다.

    cd sim/jongky_rl
    OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u train_dreamer.py \
        --headless --enable_cameras --steps 50000

dreamerv3-torch 를 먼저 받아 둘 것 (기본 경로 ~/dreamerv3-torch):

    git clone --depth 1 https://github.com/NM512/dreamerv3-torch.git

requirements.txt 를 그대로 깔면 안 된다 — torch==2.4.1 핀이 Isaac Lab 의
2.7.0 을 깨뜨린다. 실제로 모자란 건 ruamel.yaml 하나다.

[RLlib 이 아닌 이유]
RLlib 신 API 스택은 gym.make_vec() 으로 스스로 벡터화를 하는데 Isaac Sim 은
한 프로세스에 하나만 뜨므로 env 생성을 넘겨줄 수가 없다. dreamerv3-torch 는
env 를 그냥 받아 쓴다. 자세한 경위는 README "DreamerV3 연결".

[강제되는 설정]
  envs=1      Isaac Sim 이 하나뿐이다
  parallel=False  서브프로세스를 띄우면 거기서 또 Isaac Sim 을 만든다
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--dreamer-repo", default="~/dreamerv3-torch")
parser.add_argument("--logdir", default="~/jongky_dreamer_runs/corridor")
parser.add_argument("--steps", type=int, default=50000, help="환경 스텝 수")
parser.add_argument("--config", default="dmc_vision", help="configs.yaml 의 프리셋")
parser.add_argument(
    "--episode-steps", type=int, default=600,
    help="에피소드 길이(env 스텝, 30Hz). 기본 600 = 20초. configs.yaml 의 --time_limit 은 "
         "여기서 덮어쓰므로 이 값을 쓸 것 — check_reachable 이 이 값으로 도달성을 잰다",
)
AppLauncher.add_app_launcher_args(parser)
args, remaining = parser.parse_known_args()

# 시뮬레이터를 먼저 띄운다. isaaclab 관련 임포트는 이 뒤여야 한다.
simulation_app = AppLauncher(args).app

REPO = pathlib.Path(os.path.expanduser(args.dreamer_repo))
if not REPO.exists():
    raise SystemExit(f"dreamerv3-torch 가 없다: {REPO}\n  git clone https://github.com/NM512/dreamerv3-torch.git")
sys.path.insert(0, str(REPO))

import ruamel.yaml as yaml  # noqa: E402

import dreamer as dreamer_main  # noqa: E402  (dreamerv3-torch 의 dreamer.py)
import tools as dreamer_tools  # noqa: E402

from dreamer_env import get_env  # noqa: E402


def build_config():
    """configs.yaml 을 읽고 종키에 맞게 덮어쓴다."""
    configs = yaml.YAML(typ="safe").load((REPO / "configs.yaml").read_text())

    merged = {}
    for name in ("defaults", args.config):
        for k, v in configs[name].items():
            if isinstance(v, dict) and k in merged:
                merged[k].update(v)
            else:
                merged[k] = v

    # 남은 CLI 인자를 configs.yaml 키로 받는다 (--batch_size 같은 것)
    p = argparse.ArgumentParser()
    for k, v in sorted(merged.items()):
        t = dreamer_tools.args_type(v)
        p.add_argument(f"--{k}", type=t, default=t(v))
    cfg = p.parse_args(remaining)

    cfg.task = "jongky_corridor"
    cfg.logdir = os.path.expanduser(args.logdir)
    cfg.steps = args.steps
    cfg.envs = 1            # Isaac Sim 은 하나뿐이다
    cfg.parallel = False    # 서브프로세스는 거기서 또 Isaac Sim 을 만든다
    cfg.size = (64, 64)
    # 에피소드 기본 20초(600 스텝 @30Hz). dmc_vision 기본 500 스텝(16.7초)으로는
    # 목표까지 못 간다 — check_reachable 참조. action_repeat 로 나뉘므로 곱해 둔다.
    cfg.time_limit = args.episode_steps * cfg.action_repeat
    return cfg


def check_reachable(cfg) -> None:
    """에피소드 시간 안에 목표에 닿을 수 있는지 본다.

    닿을 수 없으면 도달 보너스를 한 번도 못 받아 정책이 "도착" 을 못 배운다.
    그런데 학습은 멀쩡히 도는 것처럼 보이므로, 몇 시간을 태우고 나서야
    이상하다는 걸 알게 된다. 그래서 시작 전에 잰다.

    [실효 속도를 액션 파이프라인에 직접 물어보는 이유]
    예전에는 여기서 V_MAX(0.40) 를 그대로 곱했는데, env 의 _pre_physics_step 이
    액션에 tanh 를 한 겹 더 걸고 있어서 실효 상한이 tanh(1)*0.40 = 0.305 m/s
    였다. 그래서 이 검사는 8.0 m 를 갈 수 있다고 계산했지만 실제로는 6.09 m
    였고, 필요 거리 6.0 m 와의 진짜 여유는 33% 가 아니라 1.6% 였다 — 아래
    20% 경고가 떠야 했는데 조용히 통과했다. 정확히 이 검사가 막으려던 실패
    모드를 검사 자신이 놓친 것이다. 그래서 이제는 상수를 믿지 않고
    scale_action() 에 물어본다. 스케일을 다시 손대도 검사가 따라온다.
    """
    import math

    import torch

    from jongky_corridor_env import A_MAX, JongkyCorridorEnvCfg, scale_action

    env_cfg = JongkyCorridorEnvCfg()
    dt = env_cfg.sim.dt * env_cfg.decimation           # env 한 스텝의 실제 시간
    # dreamer 는 time_limit 을 action_repeat 로 나눈 뒤 쓴다 (main 에서 처리됨)
    horizon_s = cfg.time_limit * dt

    # act=1.0 을 실제 스케일러에 넣어 실효 상한을 뽑는다
    v_eff, w_eff = (float(x) for x in scale_action(torch.ones(1, 2))[0].abs())

    # 정지에서 출발하므로 가속 램프(a_max)에서 v_eff^2/(2*a) 만큼 손해를 본다
    ramp_loss_m = v_eff * v_eff / (2.0 * A_MAX)
    reach_m = v_eff * horizon_s - ramp_loss_m

    # 최악의 경우 거리. 복도 축(x) 뿐 아니라 가로 어긋남(y)도 있다 —
    # _reset_idx 는 시작 x 를 [0,1], 시작 y 를 [-0.4,0.4] 에서 뽑고
    # 목표 y 는 goal_y_range 에서 뽑으므로 둘이 반대편일 수 있다.
    start_x_max, start_y_max = 1.0, 0.4                 # _reset_idx 의 시작 범위
    need_m = math.hypot(
        env_cfg.goal_x_range[1] - start_x_max,
        abs(env_cfg.goal_y_range[1]) + start_y_max,
    )
    margin = 1.0 - need_m / reach_m if reach_m > 0 else -1.0

    print(
        f"도달 검사: 에피소드 {horizon_s:.1f}초 · 실효 v_max {v_eff:.3f} m/s "
        f"(omega {w_eff:.3f} rad/s) · 최대주행 {reach_m:.2f}m · 최원거리 {need_m:.2f}m "
        f"· 여유 {margin * 100:.0f}%"
    )
    if need_m > reach_m:
        need_steps = int(math.ceil((need_m + ramp_loss_m) / v_eff / dt)) + 200
        raise SystemExit(
            f"목표가 에피소드 시간 안에 닿지 않는다 ({need_m:.2f}m > {reach_m:.2f}m).\n"
            f"  goal_x_range 를 줄이거나 --episode-steps 를 {need_steps} 이상으로 줄 것."
        )
    if margin < 0.2:
        print("  경고: 여유가 20% 미만이다. 회전·감속을 감안하면 빠듯하다")
    # 회전 비용은 여기 안 들어가 있다. 시작 yaw 가 ±0.35 rad 흔들리므로 정책은
    # 매 에피소드 방향부터 맞춰야 하는데, 캐스터가 URDF 상 고정 구슬이라
    # 시뮬 제자리 회전이 명령의 1/10 수준이다 (README "제자리 회전" 절).
    # 그게 잡히기 전에는 위 여유가 실제보다 낙관적이라고 봐야 한다.


def main() -> None:
    cfg = build_config()
    # dreamer.main 이 time_limit 을 action_repeat 로 나누므로 여기서도 맞춘다
    cfg.time_limit //= cfg.action_repeat
    check_reachable(cfg)
    cfg.time_limit *= cfg.action_repeat

    # make_env 를 우리 것으로 바꾼다. dreamer.py 는 task 이름 앞자리로
    # 분기하는데 거기에 우리 env 가 없다.
    def make_env(config, mode, id):
        import envs.wrappers as wrappers

        env = get_env(size=tuple(config.size))
        env = wrappers.TimeLimit(env, config.time_limit)
        env = wrappers.SelectAction(env, key="action")
        env = wrappers.UUID(env)
        return env

    dreamer_main.make_env = make_env

    print(f"logdir : {cfg.logdir}")
    print(f"steps  : {cfg.steps}")
    print(f"프리셋 : {args.config}")
    dreamer_main.main(cfg)
    simulation_app.close()


if __name__ == "__main__":
    main()

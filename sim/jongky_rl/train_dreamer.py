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
    return cfg


def main() -> None:
    cfg = build_config()

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

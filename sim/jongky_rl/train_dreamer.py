"""DreamerV3 로 종키 복도 주행 정책을 학습한다.

    cd sim/jongky_rl
    OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u train_dreamer.py \
        --headless --enable_cameras --num-envs 8 --iters 100

[왜 num_env_runners=0 인가]
Isaac Sim 은 한 프로세스에 하나만 뜬다. RLlib 이 env 를 여러 프로세스로
띄우려 하면 거기서 죽는다. 대신 Isaac Lab 이 GPU 안에서 num_envs 개를
병렬로 돌리므로 병렬성은 이미 확보돼 있다.

[env 개수]
DreamerV3 는 sample-efficient 설계라 env 를 많이 안 띄운다. Isaac Lab 예제는
PPO 용이라 수천 개지만 여기서는 4~16 이면 충분하고, 카메라 렌더링 VRAM 도
16GB 에 그 정도까지만 들어간다.

[모델 크기]
model_size 는 XS 부터 시작할 것. 관측이 64x64 RGB 한 장이고 행동이 2차원인
단순한 과제라 큰 모델이 필요 없고, 실물에 올릴 액터도 작아야 한다.
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--num-envs", type=int, default=8)
parser.add_argument("--iters", type=int, default=100)
parser.add_argument("--model-size", type=str, default="XS", help="XS/S/M/L/XL")
parser.add_argument("--checkpoint-dir", type=str, default="~/jongky_rl_runs")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

# 시뮬레이터를 먼저 띄운다. isaaclab 임포트가 이 뒤여야 한다.
simulation_app = AppLauncher(args).app

import os  # noqa: E402

import ray  # noqa: E402
from ray.rllib.algorithms.dreamerv3 import DreamerV3Config  # noqa: E402
from ray.tune.registry import register_env  # noqa: E402

from jongky_corridor_env import JongkyCorridorEnv, JongkyCorridorEnvCfg  # noqa: E402
from rllib_wrapper import IsaacLabRLlibVecEnv  # noqa: E402

# Isaac Sim 인스턴스는 하나뿐이므로 env 도 하나만 만들어 재사용한다.
_ENV = None


def make_env(_cfg=None):
    global _ENV
    if _ENV is None:
        cfg = JongkyCorridorEnvCfg()
        cfg.scene.num_envs = args.num_envs
        _ENV = IsaacLabRLlibVecEnv(JongkyCorridorEnv(cfg))
    return _ENV


def main() -> None:
    ray.init(local_mode=True, ignore_reinit_error=True)
    register_env("jongky_corridor", make_env)

    config = (
        DreamerV3Config()
        .environment(env="jongky_corridor")
        .framework("torch")
        # env 를 RLlib 이 만들지 않게 한다 — Isaac Sim 은 이 프로세스에만 있다.
        .env_runners(num_env_runners=0, num_envs_per_env_runner=args.num_envs)
        .learners(num_learners=0)
        .training(
            model_size=args.model_size,
            training_ratio=512,
            batch_size_B=16,
            batch_length_T=64,
        )
    )

    algo = config.build_algo()
    ckpt_dir = os.path.expanduser(args.checkpoint_dir)
    os.makedirs(ckpt_dir, exist_ok=True)

    for i in range(args.iters):
        result = algo.train()
        env_steps = result.get("num_env_steps_sampled_lifetime", 0)
        reward = (
            result.get("env_runners", {})
            .get("episode_return_mean")
        )
        print(f"[{i + 1}/{args.iters}] steps={env_steps} return={reward}")

        # 중간에 죽어도 이어갈 수 있게 주기적으로 남긴다.
        if (i + 1) % 10 == 0:
            path = algo.save(ckpt_dir)
            print(f"  체크포인트: {path}")

    algo.save(ckpt_dir)
    algo.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()

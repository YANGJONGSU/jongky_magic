"""PPO 베이스라인 (Stable-Baselines3) — D1 표본 효율 비교군.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaac/env_isaaclab/bin/python -u train_ppo.py \
        --headless --enable_cameras --steps 1000000

DreamerV3 와의 공정 비교 조건:
  · 같은 env (JongkyMapCorridorEnv, L10b 실측 형상) · 같은 보상 (reward_spec)
  · 같은 관측 — 이미지 64x64 uint8 + proprio [v/V_MAX, ω/Ω_MAX]
    (proprio 는 dreamer_env._proprio 와 동일하게 critic state 의 뒤 두 값)
  · x축 = 환경 스텝 합계. PPO 는 8 병렬 env 를 쓴다 — 그것도 PPO 방식의
    일부이므로 스텝 합계로 세면 공정하다 (Dreamer 는 env 1개, train_dreamer 참조)

의도적으로 다른 것 (알고리즘 고유):
  · PPO 는 on-policy 라 리플레이가 없다. 이게 비교의 요점이다
  · 하이퍼파라미터는 SB3 표준값 근처 (γ 만 dreamer 의 0.997 로 맞춤).
    베이스라인을 고문하지도, 튜닝을 퍼붓지도 않는다

정직 고지: 시간초과(truncation) 시 SB3 부트스트랩에 넘기는
terminal_observation 이 Isaac 자동 리셋 뒤 프레임이다 (dreamer_env 의
스냅샷 기법은 단일 env 전용이라 8병렬에 안 얹었다). 충돌·도달(진짜 종료)은
부트스트랩을 안 하므로 영향이 없고, 시간초과의 가치 추정만 약간 문다.
"""
from __future__ import annotations

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=1_000_000, help="환경 스텝 합계")
parser.add_argument("--num-envs", type=int, default=8)
parser.add_argument("--logdir", default="~/jongky_ppo_runs/corridor")
parser.add_argument("--geometry-json", default=None)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
simulation_app = AppLauncher(args).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import reward_spec  # noqa: E402
from jongky_map_corridor_env import JongkyMapCorridorEnv, JongkyMapCorridorEnvCfg  # noqa: E402

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.vec_env import VecEnv, VecMonitor  # noqa: E402

cfg = JongkyMapCorridorEnvCfg()
cfg.scene.num_envs = args.num_envs
cfg.normalize_obs = False              # uint8 그대로 — SB3 CnnPolicy 가 /255 한다
if args.geometry_json:
    cfg.geometry_json = os.path.abspath(args.geometry_json)
env = JongkyMapCorridorEnv(cfg)


class IsaacVecEnv(VecEnv):
    """Isaac DirectRLEnv(자동 리셋) → SB3 VecEnv."""

    def __init__(self, isaac):
        self._env = isaac
        obs_space = gym.spaces.Dict({
            "image": gym.spaces.Box(0, 255, (64, 64, 3), dtype=np.uint8),
            "proprio": gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32),
        })
        act_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
        super().__init__(isaac.num_envs, obs_space, act_space)
        self._actions = None

    def _obs(self, obs_dict):
        img = obs_dict["policy"].detach().cpu().numpy().astype(np.uint8)
        st = obs_dict["critic"].detach().cpu().numpy()
        prop = np.clip(st[:, 3:5] / np.array([reward_spec.V_MAX, reward_spec.OMEGA_MAX],
                                             dtype=np.float32), -1, 1).astype(np.float32)
        return {"image": img, "proprio": prop}

    def reset(self):
        obs_dict, _ = self._env.reset()
        return self._obs(obs_dict)

    def step_async(self, actions):
        self._actions = torch.as_tensor(actions, dtype=torch.float32,
                                        device=self._env.device)

    def step_wait(self):
        obs_dict, rew, term, trunc, _ = self._env.step(self._actions)
        obs = self._obs(obs_dict)
        rew_np = rew.detach().cpu().numpy().astype(np.float32)
        term_np = term.detach().cpu().numpy()
        trunc_np = trunc.detach().cpu().numpy()
        done = term_np | trunc_np
        infos = []
        for i in range(self.num_envs):
            info = {}
            if done[i]:
                # Isaac 은 이미 리셋했으므로 여기 관측이 곧 '리셋 후'다.
                # SB3 규약상 terminal_observation 을 넣어야 하고, 시간초과
                # 부트스트랩에만 쓰인다 (파일 머리의 정직 고지 참조).
                info["terminal_observation"] = {k: v[i] for k, v in obs.items()}
                if trunc_np[i] and not term_np[i]:
                    info["TimeLimit.truncated"] = True
                # 도달 = 진짜 종료 + 큰 양수 보상 (도달 +50 이 지배)
                info["is_success"] = bool(term_np[i] and rew_np[i] > 20.0)
            infos.append(info)
        return obs, rew_np, done, infos

    # SB3 추상 메서드 구색
    def close(self):
        self._env.close()

    def get_attr(self, name, indices=None):
        return [getattr(self._env, name, None)] * self.num_envs

    def set_attr(self, name, value, indices=None):
        raise NotImplementedError

    def env_method(self, name, *a, indices=None, **kw):
        raise NotImplementedError

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs


logdir = os.path.expanduser(args.logdir)
os.makedirs(logdir, exist_ok=True)
from stable_baselines3.common.vec_env import VecNormalize  # noqa: E402
venv = VecMonitor(IsaacVecEnv(env), filename=os.path.join(logdir, "monitor"),
                  info_keywords=("is_success",))
# 보상 정규화 + 클립. 1차 시도(lr 3e-4, 정규화 없음)가 149k 스텝·성공 69%
# 에서 정책 NaN 으로 발산했다 — 도달 +50/충돌 −25 스파이크 보상의 전형적
# PPO 발산이라 표준 처방을 쓴다. D1 지표는 성공률-스텝이므로 보상 정규화는
# 비교 공정성을 해치지 않는다 (관측·env·보상 정의는 그대로다).
venv = VecNormalize(venv, norm_obs=False, norm_reward=True, clip_reward=10.0,
                    gamma=0.997)

model = PPO(
    "MultiInputPolicy",
    venv,
    n_steps=256,               # ×8 env = 롤아웃 2048 스텝
    batch_size=512,
    n_epochs=5,
    learning_rate=1e-4,        # 3e-4 는 발산 (위 주석)
    gamma=0.997,               # dreamer(dmc_vision) discount 와 동일
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.005,
    verbose=1,
    tensorboard_log=logdir,
    device="cuda" if torch.cuda.is_available() else "cpu",
)
print("관측:", model.observation_space)
print("환경 스텝 목표: %d (env %d개 합산)" % (args.steps, args.num_envs))
model.learn(total_timesteps=args.steps, progress_bar=False)
model.save(os.path.join(logdir, "ppo_final"))
print("저장:", os.path.join(logdir, "ppo_final.zip"))
env.close()
simulation_app.close()

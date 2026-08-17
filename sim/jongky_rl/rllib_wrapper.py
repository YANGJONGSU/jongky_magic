"""Isaac Lab DirectRLEnv → RLlib(DreamerV3) 다리.

Isaac Lab 에는 sb3·skrl·rsl_rl·rl_games 래퍼만 있고 RLlib 용이 없어서 직접 쓴다.

[왜 VectorEnv 인가]
RLlib 은 보통 env 를 자기가 여러 개 만들어 돌린다. 그런데 **Isaac Sim 은 한
프로세스에 하나만 뜬다** — 포크도 복제도 안 된다. 대신 Isaac Lab 이 이미
num_envs 개를 GPU 안에서 병렬로 돌리고 있으므로, "이미 벡터화된 env" 로
넘겨주고 RLlib 에는 추가 생성을 시키지 않는다 (num_env_runners=0).

[자동 리셋 규약 — 조심할 것]
Isaac Lab 은 step() 안에서 끝난 env 를 스스로 리셋하고, 돌려주는 관측은
**리셋된 뒤의 것**이다. gymnasium 의 autoreset 은 종료 스텝에 마지막 관측을
주고 그다음 스텝에서 리셋 관측을 준다 — 한 스텝 어긋난다.

월드모델은 (관측, 행동, 다음관측) 의 연결을 학습하므로 여기가 어긋나면
에피소드 경계마다 "순간이동" 을 배운다. 그래서 종료 스텝의 관측을
final_observation 으로 따로 실어 보낸다.
"""

from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch
from isaaclab.envs import DirectRLEnv


class IsaacLabRLlibVecEnv(gym.vector.VectorEnv):
    """이미 벡터화된 Isaac Lab env 를 gymnasium VectorEnv 로 보이게 한다."""

    metadata = {"render_modes": []}

    def __init__(self, env: DirectRLEnv):
        self._env = env
        self.num_envs = env.num_envs

        # DirectRLEnv 의 space 는 배치 차원이 붙어 있다. 낱개 space 를 복원한다.
        obs_shape = tuple(env.cfg.observation_space)
        self.single_observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=obs_shape, dtype=np.float32
        )
        self.single_action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(env.cfg.action_space,), dtype=np.float32
        )
        self.observation_space = gym.vector.utils.batch_space(
            self.single_observation_space, self.num_envs
        )
        self.action_space = gym.vector.utils.batch_space(self.single_action_space, self.num_envs)

    # ── 변환 ──────────────────────────────────────────────────────────────
    @staticmethod
    def _to_numpy(x: torch.Tensor) -> np.ndarray:
        return x.detach().cpu().numpy()

    def _obs(self, obs_dict: dict) -> np.ndarray:
        return self._to_numpy(obs_dict["policy"]).astype(np.float32)

    # ── gymnasium VectorEnv ───────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        obs_dict, extras = self._env.reset(seed=seed, options=options)
        return self._obs(obs_dict), dict(extras)

    def step(self, actions: np.ndarray):
        act = torch.as_tensor(actions, dtype=torch.float32, device=self._env.device)
        obs_dict, rew, terminated, truncated, extras = self._env.step(act)

        obs = self._obs(obs_dict)
        infos = dict(extras)

        # 위 주석의 자동 리셋 규약. Isaac Lab 이 이미 리셋한 관측을 돌려주므로
        # 종료 직전 관측을 따로 실어 보내지 않으면 월드모델이 에피소드 경계에서
        # 잘못된 전이를 배운다.
        done = self._to_numpy(terminated | truncated)
        if done.any() and "final_observation" not in infos:
            infos["final_observation"] = obs.copy()

        return (
            obs,
            self._to_numpy(rew).astype(np.float32),
            self._to_numpy(terminated).astype(bool),
            self._to_numpy(truncated).astype(bool),
            infos,
        )

    def close(self):
        self._env.close()

    def render(self):
        return None

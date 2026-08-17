"""Isaac Lab 복도 env → dreamerv3-torch(NM512) 어댑터.

RLlib 대신 이쪽을 쓴다. RLlib 신 API 스택은 `gym.make_vec()` 으로 스스로
벡터화를 하는데 **Isaac Sim 은 한 프로세스에 하나만 뜨므로** env 생성을
넘겨줄 수가 없다 (README "DreamerV3 연결" 참조). dreamerv3-torch 는 env 를
그냥 받아 쓰므로 이 마찰이 없다.

[인터페이스]
dreamerv3-torch 는 **구 gym API** 다. gymnasium 이 아니다.

    reset()       -> obs                      (5-튜플 아님)
    step(action)  -> (obs, reward, done, info)  (4-튜플)

obs 는 dict 이고 다음 키가 필요하다.

    image        (H, W, 3) uint8 0~255   — 내부에서 정규화한다
    is_first     bool                    — 에피소드 첫 스텝
    is_terminal  bool                    — 진짜 종료(시간초과는 False)

[num_envs = 1]
dreamerv3-torch 는 env 를 여러 개 만들 때 서브프로세스(Parallel)를 쓰는데,
Isaac Sim 은 프로세스마다 하나씩 뜰 수 없다. 그래서 Isaac Lab 을 num_envs=1
로 두고 단일 env 로 쓴다. DreamerV3 는 sample-efficient 설계이고 DayDreamer 도
실물 1대로 학습했으므로 이 자체가 막다른 길은 아니다.
"""

from __future__ import annotations

import gym
import numpy as np

from jongky_corridor_env import JongkyCorridorEnv, JongkyCorridorEnvCfg


class JongkyDreamerEnv:
    """Isaac Lab 단일 env 를 dreamerv3-torch 가 기대하는 모양으로 감싼다."""

    metadata = {}

    def __init__(self, size: tuple[int, int] = (64, 64), **cfg_overrides):
        cfg = JongkyCorridorEnvCfg()
        cfg.scene.num_envs = 1
        # dreamerv3-torch 가 uint8 을 받아 내부에서 정규화한다. 두 번 하면 안 된다.
        cfg.normalize_obs = False
        for k, v in cfg_overrides.items():
            setattr(cfg, k, v)

        self._env = JongkyCorridorEnv(cfg)
        self._size = size
        self._first = True
        self.reward_range = [-np.inf, np.inf]

    # ── space ─────────────────────────────────────────────────────────────
    @property
    def observation_space(self):
        return gym.spaces.Dict(
            {
                "image": gym.spaces.Box(0, 255, self._size + (3,), dtype=np.uint8),
                "is_first": gym.spaces.Box(0, 1, (1,), dtype=np.uint8),
                "is_terminal": gym.spaces.Box(0, 1, (1,), dtype=np.uint8),
            }
        )

    @property
    def action_space(self):
        n = self._env.cfg.action_space
        space = gym.spaces.Box(-1.0, 1.0, (n,), dtype=np.float32)
        space.discrete = False
        return space

    # ── 변환 ──────────────────────────────────────────────────────────────
    def _image(self, obs_dict) -> np.ndarray:
        # (1, H, W, 3) 에서 배치 차원을 뗀다
        img = obs_dict["policy"][0].detach().cpu().numpy()
        return img.astype(np.uint8)

    def _obs(self, obs_dict, is_terminal: bool) -> dict:
        return {
            "image": self._image(obs_dict),
            "is_first": self._first,
            "is_terminal": is_terminal,
        }

    # ── 구 gym API ────────────────────────────────────────────────────────
    def reset(self):
        obs_dict, _ = self._env.reset()
        self._first = True
        obs = self._obs(obs_dict, is_terminal=False)
        self._first = False
        return obs

    def step(self, action):
        import torch

        act = torch.as_tensor(
            np.asarray(action, dtype=np.float32).reshape(1, -1), device=self._env.device
        )
        obs_dict, rew, terminated, truncated, _ = self._env.step(act)

        term = bool(terminated[0].item())
        trunc = bool(truncated[0].item())

        # is_terminal 은 **진짜 끝**만 참이다. 시간초과(truncated)는 거짓이어야
        # 한다 — 시간이 다 됐다고 해서 그 상태의 가치가 0 인 것은 아니기 때문이다.
        # 여기를 뭉뚱그리면 크리틱이 에피소드 끝을 전부 실패로 배운다.
        obs = self._obs(obs_dict, is_terminal=term)
        self._first = False

        return obs, float(rew[0].item()), term or trunc, {}

    def close(self):
        self._env.close()


# ── 싱글턴 ────────────────────────────────────────────────────────────────
#
# dreamer.py 는 train_envs 와 eval_envs 를 따로 만든다. Isaac Sim 은 프로세스당
# 하나뿐이라 두 번째 생성이 실패한다. 그래서 같은 인스턴스를 돌려준다.
#
# 학습용과 평가용 env 를 공유하는 것이 RL 위생상 이상적이지는 않지만,
# dreamer.py 는 둘을 동시에 돌리지 않고 평가는 매번 reset() 으로 시작하므로
# 실질적인 오염은 없다.
_SINGLETON: JongkyDreamerEnv | None = None


def get_env(**kwargs) -> JongkyDreamerEnv:
    global _SINGLETON
    if _SINGLETON is None:
        _SINGLETON = JongkyDreamerEnv(**kwargs)
    return _SINGLETON

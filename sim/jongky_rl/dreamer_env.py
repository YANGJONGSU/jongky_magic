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

================================================================================
[에피소드 경계 오염 — 이 파일이 방어하는 것]           (조사 2026-08, 확인 완료)
================================================================================

**이 절의 방어 코드를 지우지 말 것.** 여기서 막는 버그는 학습을 멈추지 않고,
손실 곡선에도 안 나타난다. 배치가 batch_length=64 짜리 조각인데 에피소드
경계는 600 스텝에 한 번뿐이라, 경계 프레임이 통째로 틀려도 재구성/KL 손실에
섞이는 비중이 1% 미만이다. 곡선은 평소처럼 내려가고, 월드모델만 "끝나는
순간" 을 영원히 잘못 배운다. 증상은 몇 시간 뒤 정책이 목표 근처에서 이상하게
구는 것으로만 나온다. 그래서 코드를 봐도 없어도 될 것처럼 보인다. 아니다.

원인 — Isaac Lab `DirectRLEnv.step()` 의 순서 (v0.54.2,
source/isaaclab/isaaclab/envs/direct_rl_env.py):

    391  self.reset_terminated[:], self.reset_time_outs[:] = self._get_dones()
    392  self.reset_buf = self.reset_terminated | self.reset_time_outs
    393  self.reward_buf = self._get_rewards()
    396  reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
    397  if len(reset_env_ids) > 0:
    398      self._reset_idx(reset_env_ids)                      # <-- 여기서 리셋
    400      if self.sim.has_rtx_sensors() and self.cfg.num_rerenders_on_reset > 0:
    401          for _ in range(self.cfg.num_rerenders_on_reset):
    402              self.sim.render()                           # <-- 새 상태를 렌더
    410  self.obs_buf = self._get_observations()                 # <-- 리셋된 뒤에 관측
    418  return self.obs_buf, self.reward_buf, self.reset_terminated, ...

즉 **종료 스텝이 돌려주는 관측은 이미 리셋된 상태에서 만들어진다.** 우리
`_get_observations()` 는 `self._camera.data.output["rgb"]` 를 그때 읽으므로,
그 프레임이 종료 직전 프레임인지 다음 에피소드 첫 프레임인지가
`num_rerenders_on_reset` 에 달려 있다.

  num_rerenders_on_reset == 0 (Isaac 기본값):
      리셋 뒤 렌더가 없다 → 카메라 애노테이터에 직전 렌더가 그대로 남아
      **종료 프레임**이 나온다. 우연히 맞는다. 대신 `reset()` 쪽이 깨진다 —
      `DirectRLEnv.reset()` 도 315 줄에서 `_reset_idx`, 322 줄에서 같은
      `num_rerenders_on_reset > 0` 가드, 331 줄에서 관측이라, 렌더가 없으면
      **직전 에피소드 마지막 프레임**을 `is_first=True` 로 내보낸다.
  num_rerenders_on_reset >= 1:
      `reset()` 은 맞아지고, 대신 종료 스텝이 **다음 에피소드 첫 프레임**을
      `done=True` 와 함께 내보낸다.

어느 쪽으로 두든 에피소드 경계의 두 프레임 중 하나가 틀린다. 그래서 여기서
둘 다 잡는다.

  (1) `num_rerenders_on_reset` 을 1 이상으로 강제한다 → `reset()` 이 진짜
      리셋된 화면을 돌려준다. 그래도 꺼져 있으면(cfg_overrides 로 되돌렸거나
      필드 이름이 바뀌었으면) `reset()` 에서 `sim.render()` 를 직접 한 번
      부르고 관측을 다시 읽는다.
  (2) `_reset_idx` 를 감싸서 **리셋이 상태를 지우기 직전의 관측을 떠 둔다**
      → 종료 스텝은 그 스냅샷을 돌려준다. (2) 가 (1) 의 부작용을 덮으므로
      두 프레임이 동시에 맞는다.

스냅샷 훅이 안 걸렸거나(Isaac 버전 변경) 종료 스텝에 `_reset_idx` 가 안
불렸으면 — 즉 자동 리셋이 없는 구현이면 — 스냅샷 없이 Isaac 이 돌려준 관측을
그대로 쓴다. 어느 쪽이든 안전하다.

[리셋 2회에 대하여]
dreamerv3-torch 의 드라이버(tools.simulate:152-165)는 `done` 을 보면
`envs[i].reset()` 을 부른다. Isaac 이 이미 자동 리셋을 했으므로 에피소드마다
리셋이 2회 돈다. 이걸 없애려면 자동 리셋 뒤의 화면을 첫 프레임으로 재활용해야
하는데, `step()` 의 리셋 경로에는 `reset()` 에 있는 `sim.forward()`
(318-319 줄, 관절 kinematics 반영) 가 없다. 그 화면이 새 자세를 반영한다는
보장이 없어서 재활용하지 않는다. 남는 비용은 에피소드당 리셋 1회 + 렌더 1회
(600 스텝 기준 0.2% 미만) 이고, 정확성에는 영향이 없다 — 두 번째 리셋이
첫 번째를 덮어쓸 뿐이다. 비용보다 첫 프레임 정확성이 중요하다고 판단했다.
`autoreset_detected` 로 자동 리셋이 실제로 도는지 런타임에 확인할 수 있다.

[검증] Isaac 없이 도는 단위 시험이 있다. 이 파일을 고치면 반드시 돌릴 것.

    cd sim/jongky_rl && python3 tools/check_reset.py

자동 리셋 O/X × 리셋 후 재렌더 O/X 네 조합에서 종료 프레임과 첫 프레임이
모두 맞는지 본다. 위 방어 코드를 지우면 이 시험이 즉시 빨개진다.
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

        # 리셋 뒤 재렌더를 강제한다 (위 (1)). Isaac 기본값은 0 이고, 0 이면
        # reset() 이 직전 에피소드 마지막 프레임을 is_first=True 로 내보낸다.
        # 종료 프레임 쪽은 아래 _reset_idx 스냅샷이 따로 지킨다.
        if hasattr(cfg, "num_rerenders_on_reset"):
            if not cfg.num_rerenders_on_reset:
                cfg.num_rerenders_on_reset = 1
        elif hasattr(cfg, "rerender_on_reset"):  # 2.3.1 이전 이름
            cfg.rerender_on_reset = True

        for k, v in cfg_overrides.items():
            setattr(cfg, k, v)

        self._env = JongkyCorridorEnv(cfg)
        self._size = size
        self.reward_range = [-np.inf, np.inf]

        # `_first` 같은 상태 플래그는 두지 않는다. is_first 는 "reset() 이
        # 돌려주는 관측만 True" 라는 규칙이 전부인데, 플래그로 들고 있으면
        # Isaac 의 자동 리셋(에피소드마다 리셋이 2회 돈다)과 어긋날 여지가
        # 생긴다. 아래 두 메서드에서 리터럴로 박는다.

        # 에피소드 경계 방어용 상태
        self._pre_reset_image: np.ndarray | None = None  # 리셋 직전 스냅샷
        self._reset_idx_fired = False                    # 이번 step 에서 자동 리셋이 돌았나
        self.autoreset_detected: bool | None = None      # 진단용 (None = 아직 모름)
        self._hook_installed = self._install_pre_reset_hook()

    # ── 에피소드 경계 방어 ────────────────────────────────────────────────
    def _install_pre_reset_hook(self) -> bool:
        """`_reset_idx` 를 감싸 **리셋 직전 관측**을 떠 두게 한다.

        `DirectRLEnv.step()` 은 `_reset_idx()` 를 부른 뒤에 `_get_observations()`
        를 부른다 (398 줄 vs 410 줄). 그 사이에 끼어들 지점이 `_reset_idx`
        뿐이다. 인스턴스 속성으로 덮으므로 `self._reset_idx(...)` 호출이
        여기로 온다 (파이썬은 인스턴스 속성을 클래스보다 먼저 본다).

        Isaac 이 이름이나 호출 순서를 바꾸면 훅이 안 걸리거나 안 불린다.
        그때는 스냅샷이 None 으로 남고 step() 이 Isaac 관측을 그대로 쓴다.
        """
        env = self._env
        inner = env._reset_idx

        def _hooked_reset_idx(env_ids, *args, **kwargs):
            # 리셋이 상태를 지우기 전에 관측을 뜬다. _get_observations() 는
            # 카메라 애노테이터를 읽을 뿐 상태를 바꾸지 않으므로 안전하다.
            # (에피소드당 1회라 비용도 무시할 수 있다)
            try:
                self._pre_reset_image = self._image(env._get_observations())
            except Exception as exc:  # 관측을 못 떠도 학습을 멈추지는 않는다
                self._pre_reset_image = None
                print(f"[dreamer_env] 경고: 리셋 직전 관측 스냅샷 실패 — {exc}")
            self._reset_idx_fired = True
            return inner(env_ids, *args, **kwargs)

        try:
            env._reset_idx = _hooked_reset_idx
        except Exception as exc:
            print(
                "[dreamer_env] 경고: _reset_idx 훅을 못 걸었다 — "
                f"{exc}\n  종료 스텝 관측이 다음 에피소드 첫 프레임일 수 있다 "
                "(파일 상단 '에피소드 경계 오염' 참조)."
            )
            return False
        return True

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

    def _obs(self, image: np.ndarray, is_first: bool, is_terminal: bool) -> dict:
        return {
            "image": image,
            "is_first": is_first,
            "is_terminal": is_terminal,
        }

    def _rerender_is_on(self) -> bool:
        """리셋 뒤 재렌더가 Isaac 쪽에서 켜져 있나."""
        cfg = self._env.cfg
        return bool(getattr(cfg, "num_rerenders_on_reset", 0)) or bool(
            getattr(cfg, "rerender_on_reset", False)
        )

    # ── 구 gym API ────────────────────────────────────────────────────────
    def reset(self):
        # 드라이버가 done 뒤에 여기를 부른다. Isaac 은 이미 자동 리셋을 했지만
        # 한 번 더 리셋한다 — 파일 상단 "[리셋 2회에 대하여]" 참조.
        self._reset_idx_fired = False
        self._pre_reset_image = None

        obs_dict, _ = self._env.reset()

        # 재렌더가 꺼져 있으면 (cfg_overrides 로 되돌렸거나 Isaac 이 필드 이름을
        # 바꿨거나) `DirectRLEnv.reset()` 이 돌려주는 화면은 **직전 에피소드
        # 마지막 프레임**이다 (direct_rl_env.py:322 가드가 False 라 렌더가 없다).
        # 그 프레임을 is_first=True 로 내보내면 월드모델이 에피소드 시작을
        # 잘못 배운다 — 종료 프레임 오염과 같은 병이고, 마찬가지로 손실 곡선에
        # 안 나타난다. 직접 한 번 렌더해서 다시 읽는다 (에피소드당 1회).
        if not self._rerender_is_on():
            try:
                self._env.sim.render()
                obs_dict = self._env._get_observations()
            except Exception as exc:
                print(f"[dreamer_env] 경고: 리셋 후 수동 렌더 실패 — {exc}")

        # reset() 안에서도 훅이 돌아 스냅샷이 남는다. 그건 직전 에피소드의
        # 마지막 프레임이므로 여기서 버린다. 안 버리면 다음 종료 스텝이
        # 남은 찌꺼기를 쓸 수 있다.
        self._reset_idx_fired = False
        self._pre_reset_image = None

        return self._obs(self._image(obs_dict), is_first=True, is_terminal=False)

    def step(self, action):
        import torch

        act = torch.as_tensor(
            np.asarray(action, dtype=np.float32).reshape(1, -1), device=self._env.device
        )

        self._reset_idx_fired = False
        self._pre_reset_image = None

        obs_dict, rew, terminated, truncated, _ = self._env.step(act)

        term = bool(terminated[0].item())
        trunc = bool(truncated[0].item())
        done = term or trunc

        # 종료 스텝이면 Isaac 이 돌려준 관측은 **이미 리셋된 상태**의 것이다.
        # 리셋 직전에 떠 둔 스냅샷으로 갈아끼운다. 스냅샷이 없으면(훅 실패,
        # 혹은 자동 리셋을 안 하는 구현이면) Isaac 관측이 곧 종료 관측이므로
        # 그대로 쓴다 — 어느 쪽이든 안전하다.
        if done and self._reset_idx_fired and self._pre_reset_image is not None:
            image = self._pre_reset_image
        else:
            image = self._image(obs_dict)

        if done and self.autoreset_detected is None:
            # 첫 에피소드 끝에서 한 번만, 자동 리셋이 실제로 도는지 남긴다.
            self.autoreset_detected = bool(self._reset_idx_fired)
            print(
                "[dreamer_env] Isaac 자동 리셋 "
                + ("감지됨 — 종료 관측을 리셋 직전 스냅샷으로 교체한다."
                   if self.autoreset_detected
                   else "없음 — Isaac 관측을 그대로 종료 관측으로 쓴다.")
            )

        # is_terminal 은 **진짜 끝**만 참이다. 시간초과(truncated)는 거짓이어야
        # 한다 — 시간이 다 됐다고 해서 그 상태의 가치가 0 인 것은 아니기 때문이다.
        # 여기를 뭉뚱그리면 크리틱이 에피소드 끝을 전부 실패로 배운다.
        # (models.py:191 에서 cont = 1 - is_terminal 로 continuation head 를 학습한다)
        obs = self._obs(image, is_first=False, is_terminal=term)
        return obs, float(rew[0].item()), done, {}

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

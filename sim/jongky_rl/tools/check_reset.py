#!/usr/bin/env python3
"""에피소드 경계 관측이 오염되지 않는지 검사한다. **Isaac 없이 돈다.**

    python3 tools/check_reset.py

무엇을 보는가
-------------
Isaac Lab `DirectRLEnv.step()` 은 종료 판정 직후 `_reset_idx()` 를 부르고
**그 뒤에** `_get_observations()` 를 부른다 (v0.54.2, direct_rl_env.py:398 과
410). 그래서 `done=True` 와 함께 나오는 관측이 종료 직전 화면인지 다음
에피소드 첫 화면인지가 `num_rerenders_on_reset` 설정에 따라 뒤집힌다.
`dreamer_env.JongkyDreamerEnv` 는 `_reset_idx` 를 감싸 리셋 직전 관측을 떠
두는 방식으로 양쪽을 다 막는다. 여기서는 그 방어가 실제로 작동하는지를
Isaac 을 흉내낸 가짜 env 로 확인한다.

이 검사가 필요한 이유: 이 버그는 학습을 멈추지 않고 **손실 곡선에도 안
나타난다**. batch_length 조각 안에서 에피소드 경계는 수백 스텝에 한 번뿐이라
경계 프레임이 통째로 틀려도 손실 기여가 1% 미만이다. 사람이 눈으로 잡을 수
없으니 기계가 잡아야 한다.

가짜 env 는 네 가지 조합을 재현한다.
    자동 리셋 O / X   ×   리셋 후 재렌더 O / X
어느 조합에서도 어댑터가 내보내는 프레임이 맞아야 한다.
"""

from __future__ import annotations

import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # dreamer_env.py 가 있는 곳


# ── 가짜 텐서 ──────────────────────────────────────────────────────────────
class FakeTensor:
    """`obs["policy"][0].detach().cpu().numpy()` 만 흉내내면 된다."""

    def __init__(self, arr):
        self.arr = np.asarray(arr)

    def __getitem__(self, i):
        return FakeTensor(self.arr[i])

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.arr


# ── 프레임 인코딩 ──────────────────────────────────────────────────────────
# 화면 내용을 (에피소드 번호, 에피소드 내 스텝) 으로 식별한다. 픽셀 0,0 의
# R 채널에 에피소드, G 채널에 스텝을 박아 두고 나중에 되읽는다.
def encode(ep: int, t: int) -> np.ndarray:
    img = np.zeros((1, 4, 4, 3), dtype=np.uint8)
    img[0, :, :, 0] = ep % 256
    img[0, :, :, 1] = t % 256
    return img


def decode(img: np.ndarray) -> tuple[int, int]:
    return int(img[0, 0, 0]), int(img[0, 0, 1])


# ── Isaac Lab 흉내 ─────────────────────────────────────────────────────────
class FakeCfg:
    """JongkyCorridorEnvCfg 중 어댑터가 만지는 필드만."""

    def __init__(self):
        self.scene = types.SimpleNamespace(num_envs=8)
        self.normalize_obs = True
        self.num_rerenders_on_reset = 0   # Isaac 기본값
        self.rerender_on_reset = False
        self.action_space = 2


class FakeIsaacEnv:
    """DirectRLEnv 의 스텝 순서를 그대로 흉내낸다.

    핵심은 카메라 애노테이터를 `self._rendered` 하나로 모델링한 것이다.
    렌더가 돌 때만 갱신되므로, 리셋 후 재렌더가 없으면 직전 프레임이 남는다 —
    실제 TiledCamera 가 그렇게 동작한다.
    """

    def __init__(self, cfg, *, autoreset=True, term_at=5, truncate=False):
        self.cfg = cfg
        self.device = "cpu"
        self._autoreset = autoreset
        self._term_at = term_at
        self._truncate = truncate

        self.episode = 0
        self.t = 0
        self._rendered = (self.episode, self.t)
        # DirectRLEnv.sim — 어댑터가 재렌더가 꺼져 있을 때 직접 부른다
        self.sim = types.SimpleNamespace(render=self._render)
        self.manual_renders = 0

        self.reset_idx_calls = 0
        self.true_terminal_frame: tuple[int, int] | None = None  # 정답 (검사용)

    # -- 내부 --
    def _render(self):
        self._rendered = (self.episode, self.t)

    def _get_observations(self):
        ep, t = self._rendered
        # critic 은 [dist, sin, cos, v, omega] 5차원이다. dreamer_env 가 뒤 두
        # 개(v, omega)를 proprio 로 슬라이스한다. 프레임 값에서 만들어 두면
        # 스냅샷이 뒤바뀌었을 때 proprio 도 같이 틀어져서 시험이 잡아낸다.
        return {
            "policy": FakeTensor(encode(ep, t)),
            "critic": FakeTensor(np.array([[3.0, 0.0, 1.0, 0.01 * t, 0.001 * ep]])),
        }

    def _reset_idx(self, env_ids):
        self.reset_idx_calls += 1
        self.episode += 1
        self.t = 0

    # -- DirectRLEnv 공개 API --
    def reset(self):
        self._reset_idx([0])                      # direct_rl_env.py:315
        if self.cfg.num_rerenders_on_reset > 0:   # :322
            self._render()
        return self._get_observations(), {}       # :331

    def step(self, action):
        self.t += 1
        self._render()                            # 물리 루프 안의 sim.render()

        term = (not self._truncate) and self.t >= self._term_at
        trunc = self._truncate and self.t >= self._term_at
        done = term or trunc
        if done:
            self.true_terminal_frame = self._rendered

        if done and self._autoreset:              # :397-402
            self._reset_idx([0])
            if self.cfg.num_rerenders_on_reset > 0:
                self._render()

        obs = self._get_observations()            # :410 — 리셋된 뒤에 관측
        rew = np.array([1.0], dtype=np.float32)
        return obs, rew, np.array([term]), np.array([trunc]), {}

    def close(self):
        pass


# ── 모듈 스텁 ──────────────────────────────────────────────────────────────
def install_stubs(env_factory):
    """dreamer_env 가 import 하는 것들을 가짜로 채운다."""
    mod = types.ModuleType("jongky_corridor_env")
    mod.JongkyCorridorEnvCfg = FakeCfg
    mod.JongkyCorridorEnv = env_factory
    # dreamer_env 가 proprio 를 [-1,1] 로 정규화할 때 쓴다. 실제 값과 같게 둔다.
    mod.V_MAX = 0.40
    mod.OMEGA_MAX = 1.50
    sys.modules["jongky_corridor_env"] = mod

    # 지도 env 는 이 시험의 관심사가 아니다 (리셋 경계만 본다). dreamer_env 가
    # 기본값으로 그쪽을 고르므로 같은 가짜를 물려 준다.
    mapmod = types.ModuleType("jongky_map_corridor_env")
    mapmod.JongkyMapCorridorEnvCfg = FakeCfg
    mapmod.JongkyMapCorridorEnv = env_factory
    sys.modules["jongky_map_corridor_env"] = mapmod

    try:
        import gym  # noqa: F401
    except ImportError:
        gym = types.ModuleType("gym")
        spaces = types.ModuleType("gym.spaces")

        class _Space:
            def __init__(self, *a, **kw):
                self.args, self.kwargs = a, kw

        spaces.Box = _Space
        spaces.Dict = _Space
        gym.spaces = spaces
        sys.modules["gym"] = gym
        sys.modules["gym.spaces"] = spaces

    try:
        import torch  # noqa: F401
    except ImportError:
        torch = types.ModuleType("torch")
        torch.as_tensor = lambda x, device=None: x
        sys.modules["torch"] = torch


# ── 검사 ───────────────────────────────────────────────────────────────────
FAILURES: list[str] = []


def check(cond: bool, label: str, detail: str = "") -> None:
    if cond:
        print(f"  OK   {label}")
    else:
        print(f"  FAIL {label}" + (f"  — {detail}" if detail else ""))
        FAILURES.append(label)


def run_case(*, autoreset: bool, rerender: int, truncate: bool) -> None:
    name = (
        f"자동리셋={'O' if autoreset else 'X'} "
        f"재렌더={rerender} "
        f"종료={'시간초과' if truncate else '진짜종료'}"
    )
    print(f"\n[{name}]")

    made = {}

    def factory(cfg):
        # 어댑터가 강제한 재렌더 설정을 케이스에 맞게 되돌려, "어댑터가 켜도
        # Isaac 쪽이 꺼져 있는" 최악의 조합까지 검사한다.
        cfg.num_rerenders_on_reset = rerender
        env = FakeIsaacEnv(cfg, autoreset=autoreset, term_at=5, truncate=truncate)
        made["cfg"] = cfg
        made["env"] = env
        return env

    install_stubs(factory)
    import dreamer_env
    import importlib

    importlib.reload(dreamer_env)
    dreamer_env._SINGLETON = None

    wrapper = dreamer_env.JongkyDreamerEnv(size=(4, 4))
    inner: FakeIsaacEnv = made["env"]

    check(getattr(wrapper, "_hook_installed", False), "_reset_idx 훅이 걸렸다")

    # -- 첫 리셋 --
    obs = wrapper.reset()
    ep0, t0 = decode(obs["image"])
    check(obs["is_first"] is True, "reset() 의 is_first 가 True")
    check(obs["is_terminal"] is False, "reset() 의 is_terminal 이 False")
    check((ep0, t0) == (inner.episode, 0),
          "reset() 이 리셋된 화면을 돌려준다", f"{(ep0, t0)} vs {(inner.episode, 0)}")

    # -- 종료까지 진행 --
    for i in range(1, 6):
        obs, rew, done, _ = wrapper.step(np.zeros(2, dtype=np.float32))
        ep, t = decode(obs["image"])
        if i < 5:
            check(not done, f"스텝 {i} 은 done 이 아니다")
            check(obs["is_first"] is False, f"스텝 {i} 의 is_first 가 False")
            check((ep, t) == (inner.episode, i),
                  f"스텝 {i} 관측이 현재 프레임", f"{(ep, t)} vs {(inner.episode, i)}")
        else:
            check(done, "마지막 스텝에서 done")
            check(obs["is_terminal"] is (not truncate),
                  "is_terminal 은 진짜 종료에서만 True",
                  f"is_terminal={obs['is_terminal']} truncate={truncate}")
            # ★ 핵심 검사 — 종료 관측은 리셋 **직전** 프레임이어야 한다
            check((ep, t) == inner.true_terminal_frame,
                  "★ 종료 관측이 리셋 직전 프레임이다",
                  f"받은={(ep, t)} 정답={inner.true_terminal_frame}")
            check(t == 5, "★ 종료 관측이 다음 에피소드 첫 프레임(t=0)이 아니다",
                  f"t={t}")

    detected = getattr(wrapper, "autoreset_detected", None)
    check(detected is autoreset,
          "자동 리셋 감지 결과가 실제와 맞다",
          f"detected={detected} 실제={autoreset}")

    # -- 드라이버가 done 뒤에 부르는 reset --
    resets_before = inner.reset_idx_calls
    obs = wrapper.reset()
    ep2, t2 = decode(obs["image"])
    check(obs["is_first"] is True, "새 에피소드 첫 관측의 is_first 가 True")
    check(obs["is_terminal"] is False, "새 에피소드 첫 관측의 is_terminal 이 False")
    check(inner.reset_idx_calls == resets_before + 1,
          "드라이버 reset() 이 Isaac 리셋을 한 번 부른다")
    check((ep2, t2) == (inner.episode, 0),
          "★ 새 에피소드 첫 관측이 리셋된 화면 (직전 에피소드 잔상 아님)",
          f"{(ep2, t2)} vs {(inner.episode, 0)}")
    check((ep2, t2) != inner.true_terminal_frame,
          "새 에피소드 첫 관측이 직전 종료 프레임이 아니다",
          f"{(ep2, t2)} == {inner.true_terminal_frame}")

    # -- 스냅샷 찌꺼기가 다음 스텝으로 새지 않는지 --
    obs, _, done, _ = wrapper.step(np.zeros(2, dtype=np.float32))
    ep3, t3 = decode(obs["image"])
    check(not done and (ep3, t3) == (inner.episode, 1),
          "새 에피소드 첫 스텝이 살아 있는 관측을 쓴다",
          f"{(ep3, t3)} vs {(inner.episode, 1)}")


def check_rerender_forced() -> None:
    """어댑터가 num_rerenders_on_reset 을 켜는지 (리셋 화면 stale 방지)."""
    print("\n[cfg 강제]")
    made = {}

    def factory(cfg):
        made["cfg"] = cfg
        return FakeIsaacEnv(cfg)

    install_stubs(factory)
    import importlib

    import dreamer_env

    importlib.reload(dreamer_env)
    dreamer_env._SINGLETON = None
    dreamer_env.JongkyDreamerEnv(size=(4, 4))
    cfg = made["cfg"]
    check(cfg.scene.num_envs == 1, "num_envs 를 1 로 내린다")
    check(cfg.normalize_obs is False, "normalize_obs 를 끈다 (dreamer 가 정규화한다)")
    check(cfg.num_rerenders_on_reset >= 1,
          "num_rerenders_on_reset 을 1 이상으로 올린다",
          f"={cfg.num_rerenders_on_reset}")


def main() -> int:
    check_rerender_forced()
    for autoreset in (True, False):
        for rerender in (0, 1):
            for truncate in (False, True):
                run_case(autoreset=autoreset, rerender=rerender, truncate=truncate)

    print()
    if FAILURES:
        print(f"실패 {len(FAILURES)}건:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("전부 통과 — 에피소드 경계 관측이 오염되지 않는다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

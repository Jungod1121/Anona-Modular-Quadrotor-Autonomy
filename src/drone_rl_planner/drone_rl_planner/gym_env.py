"""Gymnasium wrapper for LocalNavEnv (Stable-Baselines3)."""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from drone_rl_planner.local_nav_env import EnvConfig, LocalNavEnv


class LocalNavGymEnv(gym.Env):
    """Continuous local navigation: rays + goal → desired XY velocity."""

    metadata = {"render_modes": []}

    def __init__(self, cfg: Optional[EnvConfig] = None, seed: int = 0) -> None:
        super().__init__()
        self._core = LocalNavEnv(cfg=cfg, seed=seed)
        self._seed = seed
        d = self._core.obs_dim
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(d,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        if seed is not None:
            self._core.rng = np.random.default_rng(seed)
        obs = self._core.reset().astype(np.float32)
        return obs, {}

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        obs, reward, done, info = self._core.step(action)
        # Gymnasium semantics: a pure time-out is `truncated` only — setting
        # both flags made SB3 zero the bootstrap on non-terminal episodes.
        timed_out = self._core.t >= self._core.cfg.max_steps and not info.get("success")
        truncated = bool(timed_out)
        terminated = bool(done and not timed_out)
        return obs.astype(np.float32), float(reward), terminated, truncated, info


def make_env(cfg: Optional[EnvConfig] = None, rank: int = 0):
    """Factory for SubprocVecEnv."""
    def _init() -> gym.Env:
        env = LocalNavGymEnv(cfg=cfg, seed=rank)
        return gym.wrappers.RecordEpisodeStatistics(env)
    return _init

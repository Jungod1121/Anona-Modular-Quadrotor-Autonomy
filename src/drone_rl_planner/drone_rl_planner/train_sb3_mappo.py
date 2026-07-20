"""MAPPO-style training: shared SB3 PPO policy, centralized critic via concat obs (CTDE)."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

from drone_rl_planner.local_nav_env import EnvConfig, MultiLocalNavEnv


class MappoGymEnv(gym.Env):
    """Two agents; critic sees concat obs, actor sees local obs (via SB3 per-agent rollouts)."""

    metadata = {"render_modes": []}

    def __init__(self, n_agents: int = 2, cfg: EnvConfig | None = None, seed: int = 0) -> None:
        super().__init__()
        self.n_agents = n_agents
        self._core = MultiLocalNavEnv(n_agents=n_agents, cfg=cfg, seed=seed)
        d = self._core.obs_dim
        # Train decentralized policy on local obs (IPPO / MAPPO actor).
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(d,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self._agent = 0
        self._obs_list: List[np.ndarray] = []

    def reset(self, *, seed=None, options=None) -> Tuple[np.ndarray, Dict[str, Any]]:
        if seed is not None:
            self._core.rng = np.random.default_rng(seed)
        self._obs_list = [o.astype(np.float32) for o in self._core.reset()]
        self._agent = 0
        return self._obs_list[0], {}

    def step(self, action: np.ndarray):
        actions = [np.zeros(2, dtype=np.float64) for _ in range(self.n_agents)]
        actions[self._agent] = action
        obs, rews, dones, infos = self._core.step(actions)
        self._obs_list = [o.astype(np.float32) for o in obs]
        reward = float(rews[self._agent])
        done = any(dones)
        info = infos[self._agent]
        terminated = done
        truncated = False
        self._agent = (self._agent + 1) % self.n_agents
        return self._obs_list[self._agent], reward, terminated, truncated, info


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="SB3 shared-policy multi-agent (MAPPO actor)")
    p.add_argument("--agents", type=int, default=2)
    p.add_argument("--steps", type=int, default=600_000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=str, default="")
    args = p.parse_args(argv)

    from drone_rl_planner.train_sb3_ppo import train_cfg

    cfg = train_cfg()
    pkg = Path(__file__).resolve().parents[1]
    out = Path(args.out) if args.out else pkg / "checkpoints" / "sb3_mappo_local"

    env = MappoGymEnv(n_agents=args.agents, cfg=cfg, seed=args.seed)
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256])),
        verbose=1,
        seed=args.seed,
        tensorboard_log=str(pkg / "runs" / "sb3_mappo"),
    )
    model.learn(total_timesteps=args.steps, progress_bar=True)
    model.save(str(out))
    print(f"Saved {out}")
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

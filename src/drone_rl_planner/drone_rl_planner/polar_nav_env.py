"""Plant-matched 2D env for Path H polar SAC training.

Dynamics approximate cascade-PID limits:
  max_vel, max_acc, first-order velocity lag (tracking delay).
Action = (heading_offset, lookahead, speed) decoded via polar_sensing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from drone_rl_planner.bezier_path import build_bezier_path
from drone_rl_planner.local_nav_env import LocalNavEnv, EnvConfig
from drone_rl_planner.polar_sensing import (
    POLAR_DEFAULTS,
    build_polar_image,
    build_polar_vector,
    decode_action,
)


@dataclass
class PolarEnvConfig:
    n_rings: int = POLAR_DEFAULTS['n_rings']
    n_sectors: int = POLAR_DEFAULTS['n_sectors']
    ray_max: float = POLAR_DEFAULTS['ray_max']
    robot_r: float = POLAR_DEFAULTS['robot_r']
    safety: float = POLAR_DEFAULTS['safety']
    max_speed: float = POLAR_DEFAULTS['max_speed']
    max_acc: float = POLAR_DEFAULTS['max_acc']
    world_scale: float = POLAR_DEFAULTS['world_scale']
    dt: float = 0.1
    vel_lag: float = 0.45  # blend toward commanded vel each step
    max_steps: int = 450
    goal_tol: float = 0.70
    collide_penalty: float = -10.0
    step_penalty: float = -0.005
    progress_scale: float = 4.0
    goal_bonus: float = 28.0
    unsafe_penalty: float = -0.8
    cruise_z: float = 1.5


class PolarNavEnv:
    """Observation = dict(image=(2,R,S), vector=(8,)). Action = [-1,1]^3."""

    def __init__(
        self,
        cfg: Optional[PolarEnvConfig] = None,
        seed: int = 0,
        forest_heavy: bool = False,
        dense_heavy: bool = False,
    ) -> None:
        self.cfg = cfg or PolarEnvConfig()
        # Reuse scenario spawner from LocalNavEnv
        base = EnvConfig(
            max_speed=self.cfg.max_speed,
            robot_r=self.cfg.robot_r,
            ray_max=self.cfg.ray_max,
            world_scale=self.cfg.world_scale,
            cruise_z=self.cfg.cruise_z,
            goal_tol=self.cfg.goal_tol,
            max_steps=self.cfg.max_steps,
            n_obstacles=55 if forest_heavy else (70 if dense_heavy else 40),
        )
        if forest_heavy:
            # Bias toward official_forest-like scenes
            base.p_forest = 0.55
            base.p_dense = 0.20
            base.p_gate = 0.08
            base.p_corridor = 0.07
            base.p_maze = 0.05
            base.world_size = 30.0
        elif dense_heavy:
            # Match drone_map DENSE_FIELD pillar density ≈ 80 / (24×14 m²)
            # and radius range from map_dense.yaml — prior 75/30m was far too sparse,
            # so reported success (~90%+) did not transfer to the real map.
            area = 36.0 * 36.0
            catalog_n = int(round(80.0 * area / (24.0 * 14.0)))
            base.p_dense = 0.90
            base.p_forest = 0.05
            base.p_gate = 0.03
            base.p_corridor = 0.01
            base.p_maze = 0.01
            base.n_obstacles = max(160, catalog_n)
            base.world_size = 36.0
            base.obstacle_r = (0.12, 0.30)
            base.max_steps = max(base.max_steps, 650)
            self.cfg.max_steps = max(self.cfg.max_steps, 650)
        self._base = LocalNavEnv(base, seed=seed)
        self.rng = self._base.rng
        self.prev_action = np.zeros(3, dtype=np.float64)
        self.cmd_vel = np.zeros(2, dtype=np.float64)

    @property
    def image_shape(self) -> Tuple[int, int, int]:
        return (2, self.cfg.n_rings, self.cfg.n_sectors)

    @property
    def vec_dim(self) -> int:
        return POLAR_DEFAULTS['vec_dim']

    @property
    def act_dim(self) -> int:
        return 3

    def reset(self) -> Dict[str, np.ndarray]:
        self._base.reset()
        self.prev_action[:] = 0.0
        self.cmd_vel[:] = 0.0
        self._base.vel[:] = 0.0
        return self.observation()

    def observation(self) -> Dict[str, np.ndarray]:
        c = self.cfg
        b = self._base
        img = build_polar_image(
            b.pos, b.goal, b.cloud,
            n_rings=c.n_rings, n_sectors=c.n_sectors,
            ray_max=c.ray_max, robot_r=c.robot_r, safety=c.safety,
            cruise_z=b.cruise_z,
        )
        vec = build_polar_vector(
            b.pos, b.vel, b.goal, self.prev_action,
            max_speed=c.max_speed, world_scale=c.world_scale,
        )
        return {'image': img, 'vector': vec}

    def step(self, action: np.ndarray) -> Tuple[Dict[str, np.ndarray], float, bool, dict]:
        c = self.cfg
        b = self._base
        a = np.clip(np.asarray(action, dtype=np.float64).ravel()[:3], -1.0, 1.0)
        heading, look, speed = decode_action(a, b.goal, b.pos)

        # Desired velocity along heading, clipped by plant max_vel
        desired = np.array([np.cos(heading), np.sin(heading)]) * speed
        # Accel limit
        delta = desired - self.cmd_vel
        max_dv = c.max_acc * c.dt
        n = float(np.linalg.norm(delta))
        if n > max_dv:
            delta = delta / n * max_dv
        self.cmd_vel = self.cmd_vel + delta
        # First-order lag toward cmd (PID tracking delay)
        b.vel = (1.0 - c.vel_lag) * b.vel + c.vel_lag * self.cmd_vel
        spd = float(np.linalg.norm(b.vel))
        if spd > c.max_speed:
            b.vel *= c.max_speed / spd
        b.pos = b.pos + b.vel * c.dt
        b.t += 1
        self.prev_action = a.copy()

        # Safety check on proposed Bézier
        cloud_xy = b.cloud[:, :2] if b.cloud.size else None
        path, safe, min_clear = build_bezier_path(
            b.pos, heading, look, b.goal, cloud_xy,
            robot_r=c.robot_r, safety=c.safety,
        )

        half = 0.5 * b.cfg.world_size
        out = bool(np.any(np.abs(b.pos) > half))
        hit = b._collision(b.pos, c.robot_r) or out
        dist = float(np.linalg.norm(b.goal - b.pos))
        progress = b._prev_dist - dist
        b._prev_dist = dist

        reward = c.step_penalty + c.progress_scale * progress
        if not safe:
            reward += c.unsafe_penalty
        reward += 0.05 * min(1.0, min_clear / c.ray_max)

        # Prefer goal alignment when clear
        if spd > 0.1:
            align = float(np.dot(b.vel / spd, (b.goal - b.pos) / max(dist, 1e-6)))
            if min_clear > 1.5:
                reward += 0.10 * align
            else:
                reward += 0.03 * max(0.0, align)

        done = False
        info = {
            'success': False,
            'collision': False,
            'scenario': b.scenario,
            'safe': safe,
            'min_clear': min_clear,
            'heading': heading,
            'lookahead': look,
            'speed': speed,
            'path': path,
        }
        if hit:
            reward += c.collide_penalty
            done = True
            info['collision'] = True
        elif dist < c.goal_tol:
            reward += c.goal_bonus
            done = True
            info['success'] = True
        elif b.t >= c.max_steps:
            done = True
        return self.observation(), float(reward), done, info

"""2D local navigation env — velocity actions, catalog-scale domain mix."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from drone_rl_planner.sensing import (
    OBS_DEFAULTS,
    build_observation,
    circles_to_cloud,
    voxel_downsample,
    walls_to_cloud,
)


@dataclass
class EnvConfig:
    n_rays: int = OBS_DEFAULTS['n_rays']
    ray_max: float = OBS_DEFAULTS['ray_max']
    dt: float = 0.1
    max_speed: float = OBS_DEFAULTS['max_speed']
    max_steps: int = 500
    world_size: float = 28.0
    world_scale: float = OBS_DEFAULTS['world_scale']
    n_obstacles: int = 40
    obstacle_r: Tuple[float, float] = (0.18, 0.45)
    robot_r: float = OBS_DEFAULTS['robot_r']
    goal_tol: float = 0.70
    collide_penalty: float = -8.0
    step_penalty: float = -0.004
    progress_scale: float = 3.5
    goal_bonus: float = 25.0
    # Domain mix (must sum ≤ 1; remainder = sparse open)
    p_dense: float = 0.28      # dense_field-like
    p_forest: float = 0.22     # official_forest lane (clear_y)
    p_corridor: float = 0.12   # parallel walls
    p_gate: float = 0.18       # narrow_corridor gate
    p_maze: float = 0.15
    cruise_z: float = 1.5
    cruise_z_jitter: float = 0.25  # randomize ∈ [cruise_z±jitter]
    use_voxel: bool = True
    # dense_field catalog has no perimeter cage — keep False for Path H dense.
    add_boundary_walls: bool = True
    # Min center-to-center spacing hint for dense pillars (map uses ~0.9 m).
    min_obstacle_spacing: float = 0.0


class LocalNavEnv:
    """Single-agent avoid-and-reach. Action = desired XY velocity / max_speed."""

    def __init__(self, cfg: Optional[EnvConfig] = None, seed: int = 0) -> None:
        self.cfg = cfg or EnvConfig()
        self.rng = np.random.default_rng(seed)
        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.goal = np.zeros(2)
        self.obs_xy: np.ndarray = np.zeros((0, 2))
        self.obs_r: np.ndarray = np.zeros(0)
        self.wall_segs: np.ndarray = np.zeros((0, 4))
        self.cloud: np.ndarray = np.zeros((0, 3))
        self.t = 0
        self._prev_dist = 0.0
        self.scenario = 'dense'
        self.cruise_z = self.cfg.cruise_z

    @property
    def obs_dim(self) -> int:
        return self.cfg.n_rays + 5

    @property
    def act_dim(self) -> int:
        return 2

    def reset(self) -> np.ndarray:
        c = self.cfg
        jz = float(c.cruise_z_jitter)
        self.cruise_z = float(np.clip(
            c.cruise_z + self.rng.uniform(-jz, jz), 1.0, 1.8))
        r = float(self.rng.random())
        cum = 0.0
        for name, p in (
            ('dense', c.p_dense),
            ('forest', c.p_forest),
            ('corridor', c.p_corridor),
            ('gate', c.p_gate),
            ('maze', c.p_maze),
        ):
            cum += p
            if r < cum:
                self.scenario = name
                getattr(self, f'_spawn_{name}')()
                break
        else:
            self.scenario = 'sparse'
            self._spawn_dense(n_scale=0.35)
        self._rebuild_cloud()
        self.vel[:] = 0.0
        self.t = 0
        self._prev_dist = float(np.linalg.norm(self.goal - self.pos))
        return self.observation()

    def _boundary_box(self, half: float, margin: float = 0.5) -> np.ndarray:
        h = half - margin
        return np.array([
            [-h, -h, h, -h],
            [-h, h, h, h],
            [-h, -h, -h, h],
            [h, -h, h, h],
        ], dtype=np.float64)

    def _spawn_dense(self, n_scale: float = 1.0) -> None:
        """dense_field-like: many cylinders, long E–W goals (catalog style)."""
        c = self.cfg
        half = 0.5 * c.world_size
        n = max(20, int(c.n_obstacles * n_scale * self.rng.uniform(0.85, 1.15)))
        spacing = float(getattr(c, 'min_obstacle_spacing', 0.0) or 0.0)
        pts: list = []
        radii: list = []
        # Rejection sample so pillars aren't fused tighter than the catalog.
        for _ in range(n * 40):
            if len(pts) >= n:
                break
            p = self.rng.uniform(-half + 2.0, half - 2.0, size=2)
            r = float(self.rng.uniform(c.obstacle_r[0], c.obstacle_r[1]))
            if spacing > 0 and pts:
                d = np.linalg.norm(np.asarray(pts) - p[None, :], axis=1)
                if np.any(d < spacing + r):
                    continue
            pts.append(p)
            radii.append(r)
        self.obs_xy = np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 2))
        self.obs_r = np.asarray(radii, dtype=np.float64) if radii else np.zeros(0)
        if getattr(c, 'add_boundary_walls', True):
            self.wall_segs = self._boundary_box(half)
        else:
            # Match map_dense.yaml: add_boundary_walls: false
            self.wall_segs = np.zeros((0, 4), dtype=np.float64)
        # Prefer E–W crossings like catalog poses (start~west → goal~east)
        y_lane = float(self.rng.uniform(-half * 0.25, half * 0.25))
        self.pos = np.array([-half + 2.5, y_lane])
        self.goal = np.array([half - 2.5, y_lane + float(self.rng.uniform(-1.5, 1.5))])
        if self._collision(self.pos, c.robot_r + 0.3):
            self.pos = self._sample_free()
        if self._collision(self.goal, c.robot_r + 0.3):
            self.goal = self._sample_free(min_sep=max(8.0, 0.4 * c.world_size))

    def _spawn_forest(self) -> None:
        """official_forest-like: dense trees + optional rings, clear E–W band."""
        c = self.cfg
        half = 0.5 * c.world_size
        # Match random_forest clear_y ≈ 1.6 and ~60 trunks (+ rings as fat trees)
        clear_y = float(self.rng.uniform(1.4, 2.0))
        n = max(50, int(c.n_obstacles * 1.6))
        pts = []
        radii = []
        for _ in range(n * 4):
            if len(pts) >= n:
                break
            p = self.rng.uniform(-half + 1.5, half - 1.5, size=2)
            if abs(float(p[1])) < clear_y * 0.55:
                continue  # keep a free lane
            pts.append(p)
            # Mix thin trunks and thicker "circle" obstacles
            if self.rng.random() < 0.25:
                radii.append(float(self.rng.uniform(0.45, 0.70)))
            else:
                radii.append(float(self.rng.uniform(0.28, 0.50)))
        self.obs_xy = np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 2))
        self.obs_r = np.asarray(radii, dtype=np.float64) if radii else np.zeros(0)
        self.wall_segs = np.zeros((0, 4))
        # Long crossing like (−15,0)→(15,0)
        self.pos = np.array([-half + 1.5, 0.0])
        self.goal = np.array([half - 1.5, float(self.rng.uniform(-0.8, 0.8))])

    def _spawn_corridor(self) -> None:
        """Parallel-wall corridor with optional side posts."""
        c = self.cfg
        half = 0.5 * c.world_size
        width = float(self.rng.uniform(2.4, 3.8))
        y0 = float(self.rng.uniform(-2.0, 2.0))
        self.wall_segs = np.array([
            [-half + 0.5, y0 - width * 0.5, half - 0.5, y0 - width * 0.5],
            [-half + 0.5, y0 + width * 0.5, half - 0.5, y0 + width * 0.5],
        ], dtype=np.float64)
        n = int(self.rng.integers(2, 6))
        xs = self.rng.uniform(-half + 4.0, half - 4.0, size=n)
        side = self.rng.choice([-1.0, 1.0], size=n)
        ys = y0 + side * (width * 0.30)
        self.obs_xy = np.stack([xs, ys], axis=1)
        self.obs_r = self.rng.uniform(0.16, 0.30, size=n)
        self.pos = np.array([-half + 2.5, y0])
        self.goal = np.array([half - 2.5, y0])

    def _spawn_gate(self) -> None:
        """narrow_corridor-like: N–S wall with a gap + side clutter."""
        c = self.cfg
        half = 0.5 * c.world_size
        gate_x = float(self.rng.uniform(-2.0, 2.0))
        gap = float(self.rng.uniform(1.6, 2.6))
        gap_y = float(self.rng.uniform(-1.0, 1.0))
        y_lo, y_hi = -half + 0.5, half - 0.5
        # Two wall segments leaving a gap
        segs = [
            [gate_x, y_lo, gate_x, gap_y - gap * 0.5],
            [gate_x, gap_y + gap * 0.5, gate_x, y_hi],
        ]
        # Soft outer bounds
        segs.extend(self._boundary_box(half).tolist())
        self.wall_segs = np.asarray(segs, dtype=np.float64)
        # Dense side bays (away from centerline path)
        n = max(15, int(c.n_obstacles * 0.6))
        pts, radii = [], []
        for _ in range(n * 4):
            if len(pts) >= n:
                break
            p = self.rng.uniform(-half + 2.0, half - 2.0, size=2)
            # Keep approach corridor somewhat clear
            if abs(float(p[1]) - gap_y) < gap * 0.55 and abs(float(p[0]) - gate_x) < 3.0:
                continue
            pts.append(p)
            radii.append(float(self.rng.uniform(0.18, 0.40)))
        self.obs_xy = np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 2))
        self.obs_r = np.asarray(radii, dtype=np.float64) if radii else np.zeros(0)
        self.pos = np.array([-half + 2.5, gap_y])
        self.goal = np.array([half - 2.5, gap_y + float(self.rng.uniform(-0.5, 0.5))])

    def _spawn_maze(self) -> None:
        """Sparse axis-aligned wall maze."""
        c = self.cfg
        half = 0.5 * c.world_size
        segs = self._boundary_box(half).tolist()
        n_walls = int(self.rng.integers(5, 11))
        for _ in range(n_walls):
            if self.rng.random() < 0.5:
                x = float(self.rng.uniform(-half + 2, half - 2))
                y0 = float(self.rng.uniform(-half + 1, half - 4))
                length = float(self.rng.uniform(3.0, half * 0.55))
                segs.append([x, y0, x, min(y0 + length, half - 1)])
            else:
                y = float(self.rng.uniform(-half + 2, half - 2))
                x0 = float(self.rng.uniform(-half + 1, half - 4))
                length = float(self.rng.uniform(3.0, half * 0.55))
                segs.append([x0, y, min(x0 + length, half - 1), y])
        self.wall_segs = np.asarray(segs, dtype=np.float64)
        self.obs_xy = np.zeros((0, 2))
        self.obs_r = np.zeros(0)
        self.pos = self._sample_free()
        self.goal = self._sample_free(min_sep=max(8.0, 0.4 * c.world_size))

    def _rebuild_cloud(self) -> None:
        c = self.cfg
        parts = []
        if self.obs_xy.size:
            parts.append(circles_to_cloud(self.obs_xy, self.obs_r, z=self.cruise_z))
        if self.wall_segs.size:
            parts.append(walls_to_cloud(self.wall_segs, z=self.cruise_z))
        raw = np.concatenate(parts, axis=0) if parts else np.zeros((0, 3))
        if c.use_voxel and raw.size:
            self.cloud = voxel_downsample(raw, voxel=0.25, max_pts=60000)
        else:
            self.cloud = raw

    def _sample_free(self, min_sep: float = 0.0) -> np.ndarray:
        c = self.cfg
        half = 0.5 * c.world_size - 1.5
        for _ in range(400):
            p = self.rng.uniform(-half, half, size=2)
            if self._collision(p, c.robot_r + 0.25):
                continue
            if min_sep > 0 and float(np.linalg.norm(p - self.pos)) < min_sep:
                continue
            return p
        return np.zeros(2)

    def _point_to_seg_dist(self, p: np.ndarray, seg: np.ndarray) -> float:
        x0, y0, x1, y1 = seg
        a = np.array([x0, y0])
        b = np.array([x1, y1])
        ab = b - a
        t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0))
        closest = a + t * ab
        return float(np.linalg.norm(p - closest))

    def _collision(self, p: np.ndarray, margin: float) -> bool:
        if self.obs_xy.size:
            d = np.linalg.norm(self.obs_xy - p[None, :], axis=1)
            if bool(np.any(d < self.obs_r + margin)):
                return True
        wall_thick = 0.12
        for seg in self.wall_segs:
            if self._point_to_seg_dist(p, seg) < wall_thick + margin:
                return True
        return False

    def observation(self) -> np.ndarray:
        c = self.cfg
        pos3 = np.array([self.pos[0], self.pos[1], self.cruise_z])
        goal3 = np.array([self.goal[0], self.goal[1], self.cruise_z])
        return build_observation(
            pos3, self.vel, goal3, self.cloud,
            n_rays=c.n_rays, ray_max=c.ray_max,
            max_speed=c.max_speed, world_scale=c.world_scale,
            cruise_z=self.cruise_z, robot_r=c.robot_r,
        )

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        c = self.cfg
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        desired = a * c.max_speed
        self.vel = 0.65 * self.vel + 0.35 * desired
        self.vel = np.clip(self.vel, -c.max_speed, c.max_speed)
        self.pos = self.pos + self.vel * c.dt
        self.t += 1

        half = 0.5 * c.world_size
        out = bool(np.any(np.abs(self.pos) > half))
        hit = self._collision(self.pos, c.robot_r) or out
        dist = float(np.linalg.norm(self.goal - self.pos))
        progress = self._prev_dist - dist
        self._prev_dist = dist
        reward = c.step_penalty + c.progress_scale * progress

        obs = self.observation()
        rays = obs[: c.n_rays]
        min_ray = float(np.min(rays))
        # Early proximity cost (normalized): start caring ~3 m out, not only at collision
        # With ray_max=6, ray=0.5 ≈ 3 m; ray=0.25 ≈ 1.5 m
        if min_ray < 0.55:
            reward -= 1.8 * (0.55 - min_ray) ** 2
        if min_ray < 0.28:
            reward -= 3.5 * (0.28 - min_ray)

        # Prefer heading toward the goal when the way is clear (cuts big loops)
        goal_dir = (self.goal - self.pos) / max(dist, 1e-6)
        speed = float(np.linalg.norm(self.vel))
        if speed > 0.15:
            vel_dir = self.vel / speed
            align = float(np.dot(vel_dir, goal_dir))
            if min_ray > 0.45:
                reward += 0.08 * align
            else:
                # Near obstacles: small bonus for any progress, not pure alignment
                reward += 0.02 * max(0.0, align)
            # Penalize pure sideways circling when far from obstacles
            if min_ray > 0.50 and align < 0.0:
                reward -= 0.12 * abs(align)

        # Mild clearance bonus
        reward += 0.04 * min_ray

        done = False
        info = {'success': False, 'collision': False, 'scenario': self.scenario}
        if hit:
            reward += c.collide_penalty
            done = True
            info['collision'] = True
        elif dist < c.goal_tol:
            reward += c.goal_bonus
            done = True
            info['success'] = True
        elif self.t >= c.max_steps:
            done = True
        return obs, float(reward), done, info


class MultiLocalNavEnv:
    """Independent robots, shared obstacle field — MAPPO training."""

    def __init__(
        self,
        n_agents: int = 2,
        cfg: Optional[EnvConfig] = None,
        seed: int = 0,
    ) -> None:
        self.n_agents = n_agents
        self.cfg = cfg or EnvConfig()
        self.rng = np.random.default_rng(seed)
        self.envs = [LocalNavEnv(self.cfg, seed=seed + i) for i in range(n_agents)]

    @property
    def obs_dim(self) -> int:
        return self.envs[0].obs_dim

    @property
    def act_dim(self) -> int:
        return self.envs[0].act_dim

    def reset(self) -> List[np.ndarray]:
        o0 = self.envs[0]
        o0.reset()
        for i in range(1, self.n_agents):
            e = self.envs[i]
            e.obs_xy = o0.obs_xy.copy()
            e.obs_r = o0.obs_r.copy()
            e.wall_segs = o0.wall_segs.copy()
            e.cloud = o0.cloud.copy()
            e.scenario = o0.scenario
            e.cruise_z = o0.cruise_z
            e.pos = o0._sample_free(min_sep=2.0)
            e.goal = o0._sample_free(min_sep=3.0)
            e.vel[:] = 0.0
            e.t = 0
            e._prev_dist = float(np.linalg.norm(e.goal - e.pos))
            for j in range(i):
                if np.linalg.norm(e.pos - self.envs[j].pos) < 1.0:
                    e.pos = o0._sample_free(min_sep=2.0)
        return [e.observation() for e in self.envs]

    def step(
        self, actions: List[np.ndarray]
    ) -> Tuple[List[np.ndarray], List[float], List[bool], List[dict]]:
        obs, rews, dones, infos = [], [], [], []
        for i, (e, a) in enumerate(zip(self.envs, actions)):
            o, r, d, info = e.step(a)
            for j, other in enumerate(self.envs):
                if i == j:
                    continue
                if np.linalg.norm(e.pos - other.pos) < 2.0 * e.cfg.robot_r + 0.15:
                    r += -4.0
                    info['collision'] = True
                    d = True
            obs.append(o)
            rews.append(r)
            dones.append(d)
            infos.append(info)
        return obs, rews, dones, infos

    def global_state(self) -> np.ndarray:
        return np.concatenate([e.observation() for e in self.envs], axis=0)

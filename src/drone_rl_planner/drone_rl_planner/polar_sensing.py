"""Polar occupancy image for Path H (SACPlanner-style state).

Channels:
  0 — clearance (1 = free to ray_max, 0 = blocked)
  1 — goal bearing peak (gaussian bump on goal sector)
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from drone_rl_planner.sensing import voxel_downsample

POLAR_DEFAULTS = {
    'n_rings': 16,
    'n_sectors': 36,
    'ray_max': 6.0,
    'robot_r': 0.28,
    'safety': 0.30,
    'max_speed': 1.0,
    'max_acc': 1.5,
    'world_scale': 40.0,
    'vec_dim': 8,  # goal_dist, goal_cos, goal_sin, vx, vy, prev_a0..2
}


def build_polar_image(
    pos_xy: np.ndarray,
    goal_xy: np.ndarray,
    cloud_xyz: Optional[np.ndarray],
    n_rings: int = POLAR_DEFAULTS['n_rings'],
    n_sectors: int = POLAR_DEFAULTS['n_sectors'],
    ray_max: float = POLAR_DEFAULTS['ray_max'],
    robot_r: float = POLAR_DEFAULTS['robot_r'],
    safety: float = POLAR_DEFAULTS['safety'],
    cruise_z: float = 1.5,
    z_half: float = 1.4,
) -> np.ndarray:
    """Return float32 image (2, n_rings, n_sectors) in [0, 1]."""
    clear = np.ones((n_rings, n_sectors), dtype=np.float32)
    goal_ch = np.zeros((n_rings, n_sectors), dtype=np.float32)

    if cloud_xyz is not None and cloud_xyz.size:
        pts = np.asarray(cloud_xyz, dtype=np.float64)
        band = np.abs(pts[:, 2] - cruise_z) <= z_half
        xy = pts[band][:, :2] if np.any(band) else pts[:, :2]
        rel = xy - pos_xy[None, :]
        dist = np.linalg.norm(rel, axis=1)
        near = dist < (ray_max + 1.0)
        if np.any(near):
            rel = rel[near]
            dist = dist[near]
            ang = np.arctan2(rel[:, 1], rel[:, 0])
            inflated = np.maximum(0.05, dist - robot_r - safety)
            sector = ((ang + np.pi) / (2.0 * np.pi) * n_sectors).astype(np.int64)
            sector = np.clip(sector, 0, n_sectors - 1)
            ring = np.floor(inflated / ray_max * n_rings).astype(np.int64)
            ring = np.clip(ring, 0, n_rings - 1)
            # Mark occupied cells and all cells beyond first hit in that sector
            hit = np.full(n_sectors, ray_max, dtype=np.float64)
            for s, d in zip(sector, inflated):
                if d < hit[s]:
                    hit[s] = d
            for s in range(n_sectors):
                r_hit = hit[s]
                if r_hit >= ray_max:
                    continue
                r0 = int(np.floor(r_hit / ray_max * n_rings))
                r0 = min(max(r0, 0), n_rings - 1)
                # Occupancy rises from far=1 to near=0
                for r in range(r0, n_rings):
                    # cells at/after first obstacle
                    frac = 1.0 - (r_hit / ray_max)
                    clear[r, s] = min(clear[r, s], max(0.0, 1.0 - frac - 0.15 * (r - r0)))
                # ring of the hit itself darker
                clear[r0, s] = min(clear[r0, s], max(0.0, r_hit / ray_max))

    # Goal bearing channel: soft peak on goal sector across all rings
    to_g = goal_xy - pos_xy
    g_ang = float(np.arctan2(to_g[1], to_g[0]))
    g_sec = (g_ang + np.pi) / (2.0 * np.pi) * n_sectors
    for s in range(n_sectors):
        d = abs(((s + 0.5) - g_sec + n_sectors / 2) % n_sectors - n_sectors / 2)
        w = float(np.exp(-0.5 * (d / 1.8) ** 2))
        goal_ch[:, s] = w

    return np.stack([clear, goal_ch], axis=0).astype(np.float32)


def build_polar_vector(
    pos_xy: np.ndarray,
    vel_xy: np.ndarray,
    goal_xy: np.ndarray,
    prev_action: np.ndarray,
    max_speed: float = POLAR_DEFAULTS['max_speed'],
    world_scale: float = POLAR_DEFAULTS['world_scale'],
) -> np.ndarray:
    """Low-dim proprio + goal + previous action. float32."""
    rel = goal_xy - pos_xy
    dist = float(np.linalg.norm(rel))
    if dist < 1e-6:
        cos_g, sin_g = 1.0, 0.0
    else:
        cos_g = float(rel[0] / dist)
        sin_g = float(rel[1] / dist)
    a = np.clip(np.asarray(prev_action, dtype=np.float64).ravel()[:3], -1.0, 1.0)
    if a.size < 3:
        a = np.pad(a, (0, 3 - a.size))
    return np.array([
        np.clip(dist / world_scale, 0.0, 1.0),
        cos_g,
        sin_g,
        np.clip(vel_xy[0] / max_speed, -1.0, 1.0),
        np.clip(vel_xy[1] / max_speed, -1.0, 1.0),
        float(a[0]),
        float(a[1]),
        float(a[2]),
    ], dtype=np.float32)


def decode_action(
    action: np.ndarray,
    goal_xy: np.ndarray,
    pos_xy: np.ndarray,
    look_min: float = 0.9,
    look_max: float = 2.4,
    speed_min: float = 0.35,
    speed_max: float = 1.0,
) -> Tuple[float, float, float]:
    """Map a∈[-1,1]^3 → (heading_rad, lookahead_m, speed_mps).

    a[0]: heading offset from goal bearing (±90°)
    a[1]: lookahead
    a[2]: cruise speed
    """
    a = np.clip(np.asarray(action, dtype=np.float64).ravel()[:3], -1.0, 1.0)
    to_g = goal_xy - pos_xy
    goal_ang = float(np.arctan2(to_g[1], to_g[0]))
    heading = goal_ang + float(a[0]) * (0.5 * np.pi)
    look = look_min + 0.5 * (float(a[1]) + 1.0) * (look_max - look_min)
    speed = speed_min + 0.5 * (float(a[2]) + 1.0) * (speed_max - speed_min)
    return heading, look, speed


def random_shift(img: np.ndarray, pad: int = 4, rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """DrQ-style random shift on (C,H,W) polar image (circular in sector axis)."""
    rng = rng or np.random.default_rng()
    c, h, w = img.shape
    # Pad rings with edge; wrap sectors
    padded = np.pad(img, ((0, 0), (pad, pad), (0, 0)), mode='edge')
    # Circular pad sectors
    padded = np.concatenate([padded[:, :, -pad:], padded, padded[:, :, :pad]], axis=2)
    dy = int(rng.integers(0, 2 * pad + 1))
    dx = int(rng.integers(0, 2 * pad + 1))
    return padded[:, dy:dy + h, dx:dx + w].astype(np.float32)


def downsample_cloud(cloud_xyz: Optional[np.ndarray]) -> np.ndarray:
    if cloud_xyz is None or cloud_xyz.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return voxel_downsample(cloud_xyz, voxel=0.25, max_pts=60000)

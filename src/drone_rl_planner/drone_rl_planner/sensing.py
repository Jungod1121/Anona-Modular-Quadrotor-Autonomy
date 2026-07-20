"""Unified observation builder — same code for gym training and ROS inference."""

from __future__ import annotations

from typing import Optional

import numpy as np

# Defaults baked into the trained policy. Keep train + ROS identical.
OBS_DEFAULTS = {
    'n_rays': 36,
    'ray_max': 6.0,   # see obstacles earlier (~6 m) — needs matching checkpoint
    'max_speed': 1.2,
    'world_scale': 40.0,
    'robot_r': 0.22,
    'ray_width': 0.40,
}


def voxel_downsample(cloud_xyz: np.ndarray, voxel: float = 0.25, max_pts: int = 80000) -> np.ndarray:
    """Keep one point per voxel (avoids stride holes on walls)."""
    if cloud_xyz is None or cloud_xyz.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    pts = np.asarray(cloud_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return np.zeros((0, 3), dtype=np.float64)
    keys = np.floor(pts[:, :3] / max(voxel, 1e-3)).astype(np.int64)
    flat = keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791
    _, idx = np.unique(flat, return_index=True)
    out = pts[idx]
    if out.shape[0] > max_pts:
        # Random subsample preserves wall coverage better than stride
        sel = np.random.default_rng(0).choice(out.shape[0], size=max_pts, replace=False)
        out = out[sel]
    return out


def circles_to_cloud(
    centers: np.ndarray,
    radii: np.ndarray,
    z: float = 1.5,
    n_per: int = 24,
) -> np.ndarray:
    """Sample circle surfaces as a point cloud (matches ROS ray caster)."""
    if centers.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    angs = np.linspace(0.0, 2.0 * np.pi, n_per, endpoint=False)
    pts = []
    for c, r in zip(centers, radii):
        xs = c[0] + r * np.cos(angs)
        ys = c[1] + r * np.sin(angs)
        zs = np.full_like(xs, z)
        pts.append(np.stack([xs, ys, zs], axis=1))
        # Extra rings for thickness
        for scale in (0.85, 1.0):
            xs2 = c[0] + scale * r * np.cos(angs)
            ys2 = c[1] + scale * r * np.sin(angs)
            pts.append(np.stack([xs2, ys2, zs], axis=1))
    return np.concatenate(pts, axis=0)


def walls_to_cloud(
    segments: np.ndarray,
    z: float = 1.5,
    spacing: float = 0.15,
) -> np.ndarray:
    """segments: (N, 4) as x0,y0,x1,y1 axis-aligned or diagonal wall edges."""
    if segments is None or len(segments) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    pts = []
    for x0, y0, x1, y1 in segments:
        length = float(np.hypot(x1 - x0, y1 - y0))
        n = max(2, int(length / spacing) + 1)
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        zs = np.full(n, z)
        pts.append(np.stack([xs, ys, zs], axis=1))
        # Slight thickness
        nx, ny = -(y1 - y0), (x1 - x0)
        nrm = max(np.hypot(nx, ny), 1e-6)
        nx, ny = 0.08 * nx / nrm, 0.08 * ny / nrm
        pts.append(np.stack([xs + nx, ys + ny, zs], axis=1))
        pts.append(np.stack([xs - nx, ys - ny, zs], axis=1))
    return np.concatenate(pts, axis=0)


def cast_rays_from_cloud(
    origin_xy: np.ndarray,
    cloud_xyz: Optional[np.ndarray],
    n_rays: int = OBS_DEFAULTS['n_rays'],
    ray_max: float = OBS_DEFAULTS['ray_max'],
    z_half: float = 1.2,
    cruise_z: float = 1.5,
    robot_r: float = OBS_DEFAULTS['robot_r'],
    ray_width: float = OBS_DEFAULTS['ray_width'],
) -> np.ndarray:
    """Normalized ray hits in [0,1] (1 = clear to ray_max). Inflates by robot_r."""
    hits = np.full(n_rays, ray_max, dtype=np.float64)
    if cloud_xyz is None or cloud_xyz.size == 0:
        return hits / ray_max

    band = np.abs(cloud_xyz[:, 2] - cruise_z) <= z_half
    pts = cloud_xyz[band][:, :2]
    if pts.size == 0:
        pts = cloud_xyz[:, :2]

    # Local crop for speed
    ox, oy = float(origin_xy[0]), float(origin_xy[1])
    rel_all = pts - np.array([ox, oy])
    near = np.linalg.norm(rel_all, axis=1) < (ray_max + 1.0)
    pts = pts[near]
    if pts.size == 0:
        return hits / ray_max

    angles = np.linspace(-np.pi, np.pi, n_rays, endpoint=False)
    for i, ang in enumerate(angles):
        dx, dy = np.cos(ang), np.sin(ang)
        relx = pts[:, 0] - ox
        rely = pts[:, 1] - oy
        proj = relx * dx + rely * dy
        cross = np.abs(relx * dy - rely * dx)
        mask = (proj > 0.05) & (proj < ray_max) & (cross < ray_width)
        if np.any(mask):
            # Inflate: report surface minus robot radius (clamp ≥ 0.05)
            hits[i] = max(0.05, float(np.min(proj[mask])) - robot_r)
    return hits / ray_max


def build_observation(
    pos_xyz: np.ndarray,
    vel_xy: np.ndarray,
    goal_xyz: np.ndarray,
    cloud_xyz: Optional[np.ndarray],
    n_rays: int = OBS_DEFAULTS['n_rays'],
    ray_max: float = OBS_DEFAULTS['ray_max'],
    max_speed: float = OBS_DEFAULTS['max_speed'],
    world_scale: float = OBS_DEFAULTS['world_scale'],
    cruise_z: Optional[float] = None,
    robot_r: float = OBS_DEFAULTS['robot_r'],
) -> np.ndarray:
    """Shared obs vector for training and ROS. dtype float32 for SB3."""
    cz = float(cruise_z) if cruise_z is not None else (
        float(pos_xyz[2]) if len(pos_xyz) > 2 else 1.5)
    rays = cast_rays_from_cloud(
        pos_xyz[:2], cloud_xyz,
        n_rays=n_rays, ray_max=ray_max, cruise_z=cz, robot_r=robot_r)
    rel = goal_xyz[:2] - pos_xyz[:2]
    dist = float(np.linalg.norm(rel))
    rel_n = rel / max(dist, 1e-6)
    return np.concatenate([
        rays,
        rel_n,
        [np.clip(dist / world_scale, 0.0, 1.0)],
        np.clip(vel_xy / max_speed, -1.0, 1.0),
    ]).astype(np.float32)

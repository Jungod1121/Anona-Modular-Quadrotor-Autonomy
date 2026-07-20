"""Collision-checked short Bézier / rolled path for Path H."""

from __future__ import annotations

from typing import Callable, List, Optional, Tuple

import numpy as np


def _point_clearance(xy: np.ndarray, cloud_xy: Optional[np.ndarray], ray_max: float = 6.0) -> float:
    if cloud_xy is None or cloud_xy.size == 0:
        return ray_max
    d = np.linalg.norm(cloud_xy - xy[None, :], axis=1)
    return float(np.min(d)) if d.size else ray_max


def corridor_clearance(
    pts: np.ndarray,
    cloud_xy: Optional[np.ndarray],
    half_width: float = 0.45,
    ray_max: float = 6.0,
) -> float:
    """Min distance to obstacles *inside* a tube around the polyline.

    Trees beside the path (outside half_width) do not count — critical in forests.
    """
    if cloud_xy is None or cloud_xy.size == 0 or pts.shape[0] < 2:
        return ray_max
    min_c = ray_max
    for i in range(len(pts) - 1):
        a = pts[i]
        b = pts[i + 1]
        ab = b - a
        length = float(np.linalg.norm(ab))
        if length < 1e-6:
            continue
        t = np.clip(((cloud_xy - a) @ ab) / (length * length), 0.0, 1.0)
        closest = a[None, :] + t[:, None] * ab[None, :]
        lat = np.linalg.norm(cloud_xy - closest, axis=1)
        near = lat <= half_width
        if np.any(near):
            # Along-track remaining distance to obstacle projection
            along = np.linalg.norm(closest[near] - a[None, :], axis=1)
            # Prefer lateral penetration depth as clearance proxy
            min_c = min(min_c, float(np.min(lat[near])))
            _ = along  # reserved for future shaping
    return float(min_c)


def quadratic_bezier(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, n: int) -> np.ndarray:
    t = np.linspace(0.0, 1.0, n)[:, None]
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2


def build_bezier_path(
    pos_xy: np.ndarray,
    heading: float,
    lookahead: float,
    goal_xy: np.ndarray,
    cloud_xy: Optional[np.ndarray] = None,
    robot_r: float = 0.28,
    safety: float = 0.35,
    n_pts: int = 12,
    cruise_z: float = 1.5,
    corridor_half: float = 0.50,
) -> Tuple[np.ndarray, bool, float]:
    """Return (Nx2 path, is_safe, min_corridor_clearance)."""
    d_hat = np.array([np.cos(heading), np.sin(heading)], dtype=np.float64)
    p0 = pos_xy.astype(np.float64)
    p1 = p0 + d_hat * lookahead
    to_g = goal_xy - p1
    dg = float(np.linalg.norm(to_g))
    if dg > 1e-3:
        p2 = p1 + 0.55 * to_g / dg * min(lookahead, dg)
    else:
        p2 = goal_xy.astype(np.float64)

    if float(np.linalg.norm(goal_xy - p0)) < lookahead * 1.2:
        p2 = goal_xy.astype(np.float64)

    pts = quadratic_bezier(p0, p1, p2, n_pts)
    half = max(corridor_half, robot_r + 0.15)
    min_clear = corridor_clearance(pts, cloud_xy, half_width=half)
    margin = robot_r + safety * 0.55  # softer than raw nearest-point
    safe = min_clear >= margin
    return pts, safe, float(min_clear)


def build_rolled_path(
    pos_xy: np.ndarray,
    heading0: float,
    goal_xy: np.ndarray,
    heading_fn: Callable[[np.ndarray, float, float], float],
    cloud_xy: Optional[np.ndarray] = None,
    step_m: float = 0.45,
    horizon_m: float = 8.0,
    robot_r: float = 0.28,
    safety: float = 0.30,
    goal_tol: float = 0.70,
) -> Tuple[np.ndarray, float, float]:
    """Roll a polyline, re-picking heading along the way. Returns path, final_h, min_clear."""
    cur = pos_xy.astype(np.float64).copy()
    h = float(heading0)
    pts = [cur.copy()]
    traveled = 0.0
    min_clear = 6.0
    while traveled < horizon_m:
        to_g = goal_xy - cur
        dist_g = float(np.linalg.norm(to_g))
        if dist_g < goal_tol:
            pts.append(goal_xy.astype(np.float64))
            break
        goal_ang = float(np.arctan2(to_g[1], to_g[0]))
        h = heading_fn(cur, goal_ang, h)
        # Forward clearance estimate
        tip = cur + np.array([np.cos(h), np.sin(h)]) * step_m
        seg = np.stack([cur, tip], axis=0)
        c = corridor_clearance(seg, cloud_xy, half_width=robot_r + safety)
        min_clear = min(min_clear, c)
        step = min(step_m, max(0.25, c - 0.25), dist_g)
        cur = cur + np.array([np.cos(h), np.sin(h)]) * step
        traveled += step
        pts.append(cur.copy())
    else:
        pts.append(goal_xy.astype(np.float64))
    return np.asarray(pts, dtype=np.float64), h, float(min_clear)

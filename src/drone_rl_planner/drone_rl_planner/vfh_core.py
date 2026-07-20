"""Shared VFH+ core (Path G + adapter-level safety supervisor).

Classical Vector Field Histogram+ helpers — no ROS dependency.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def polar_clearance(
    cloud_xy: Optional[np.ndarray],
    pos_xy: np.ndarray,
    *,
    n_sectors: int = 72,
    ray_max: float = 6.0,
    robot_r: float = 0.28,
    safety: float = 0.35,
    hist_smooth: int = 3,
) -> np.ndarray:
    """Per-sector min distance (m) to obstacles. ray_max = free."""
    clear = np.full(n_sectors, ray_max, dtype=np.float64)
    if cloud_xy is None or cloud_xy.size == 0:
        return clear
    pts = np.asarray(cloud_xy, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 2:
        return clear
    rel = pts[:, :2] - pos_xy[None, :]
    dist = np.linalg.norm(rel, axis=1)
    near = dist < (ray_max + 1.0)
    if not np.any(near):
        return clear
    rel = rel[near]
    dist = dist[near]
    ang = np.arctan2(rel[:, 1], rel[:, 0])
    inflated = np.maximum(0.05, dist - robot_r - safety)
    sector = ((ang + np.pi) / (2 * np.pi) * n_sectors).astype(np.int64)
    sector = np.clip(sector, 0, n_sectors - 1)
    for s, d in zip(sector, inflated):
        if d < clear[s]:
            clear[s] = d
    k = max(1, int(hist_smooth))
    if k > 1:
        pad = np.concatenate([clear[-k:], clear, clear[:k]])
        kernel = np.ones(2 * k + 1) / (2 * k + 1)
        clear = np.convolve(pad, kernel, mode='valid')
    return clear


def select_heading(
    clear: np.ndarray,
    goal_ang: float,
    cur_heading: float,
    *,
    ray_max: float = 6.0,
    clear_threshold: float = 0.42,
    goal_weight: float = 1.2,
) -> float:
    """Pick sector: maximize clearance, bias toward goal, continuity."""
    n_sectors = int(clear.shape[0])
    thresh = clear_threshold * ray_max
    angles = -np.pi + (np.arange(n_sectors) + 0.5) * (2 * np.pi / n_sectors)
    best_score = -1e18
    best_ang = goal_ang
    for i, (ang, c) in enumerate(zip(angles, clear)):
        if c < thresh * 0.55:
            continue
        d_goal = abs(((ang - goal_ang + np.pi) % (2 * np.pi)) - np.pi)
        d_cur = abs(((ang - cur_heading + np.pi) % (2 * np.pi)) - np.pi)
        score = (
            (c / ray_max) * 2.0
            + goal_weight * np.cos(d_goal)
            - 0.35 * d_cur
        )
        left = clear[(i - 1) % n_sectors]
        right = clear[(i + 1) % n_sectors]
        if left > thresh and right > thresh:
            score += 0.35
        if score > best_score:
            best_score = score
            best_ang = float(ang)
    if best_score < -1e17:
        i = int(np.argmax(clear))
        best_ang = float(angles[i])
    return best_ang


def sector_clearance(clear: np.ndarray, heading: float) -> float:
    n_sectors = int(clear.shape[0])
    idx = int(((heading + np.pi) / (2 * np.pi) * n_sectors)) % n_sectors
    vals = [clear[(idx + k) % n_sectors] for k in (-1, 0, 1)]
    return float(np.min(vals))


def vfh_heading(
    cloud_xy: Optional[np.ndarray],
    pos_xy: np.ndarray,
    goal_xy: np.ndarray,
    cur_heading: float = 0.0,
    **kwargs,
) -> Tuple[float, np.ndarray]:
    """Return (heading, clearance_histogram)."""
    clear = polar_clearance(cloud_xy, pos_xy, **{
        k: kwargs[k] for k in ('n_sectors', 'ray_max', 'robot_r', 'safety', 'hist_smooth')
        if k in kwargs
    })
    to_goal = goal_xy - pos_xy
    if float(np.linalg.norm(to_goal)) < 1e-6:
        return cur_heading, clear
    goal_ang = float(np.arctan2(to_goal[1], to_goal[0]))
    h = select_heading(
        clear, goal_ang, cur_heading,
        ray_max=float(kwargs.get('ray_max', 6.0)),
        clear_threshold=float(kwargs.get('clear_threshold', 0.42)),
        goal_weight=float(kwargs.get('goal_weight', 1.2)),
    )
    return h, clear

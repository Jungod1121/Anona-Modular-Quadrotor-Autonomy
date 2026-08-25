"""Per-map planner parameter overrides — single source of truth.

These tables ARE the tuning contract between maps (MAPS.md) and planners.
Launch helpers consume them; do not duplicate values in launch files.

Historically these lived inside maps_catalog.py and one consumer
(planner_node) never received them at all — the "dead overrides" bug class
this extraction prevents. Ownership: drone_bringup maintainers.
"""

from __future__ import annotations

from typing import Any, Callable, Dict

# Normalizer injected by maps_catalog at import time (avoids a circular
# import: normalization validates against the MAPS catalog, while these
# override tables are consumed by launches through maps_catalog wrappers).
_normalize_fn: Callable[[str], str] = lambda mid: (mid or '').strip().lower()


def bind_map_normalizer(fn: Callable[..., str]) -> None:
    global _normalize_fn
    _normalize_fn = fn


def _norm(map_id: str, planner: str) -> str:
    try:
        return _normalize_fn(map_id, planner=planner)
    except TypeError:
        return _normalize_fn(map_id)


def homemade_planner_overrides(map_id: str, planner: str = 'homemade') -> Dict[str, Any]:
    """Light Path A hints. Grid AABB + inflate now auto-fit from the cloud."""
    mid = _norm(map_id, planner)
    # Keep cruise / speed hints; do not pin map_origin/size (auto_map_fit owns that).
    if mid == 'narrow_corridor' or mid == 'tier_medium_corridor':
        # S-bend gates are horizontal; true-3D can thread sparse wall voxels in Z.
        return {
            'cruise_z': 1.5,
            'true_3d_astar': False,
            'z_band': 0.45,
            'vertical_cost_scale': 4.0,
            'max_vel': 0.65,
            'resolution': 0.15,
            'auto_inflate': True,
            'auto_inflate_min': 0.24,
            'auto_inflate_max': 0.32,
            'local_goal_lookahead': 0.40,
            'emergency_clearance': 0.16,
            'seal_boundary_layers': 1,
        }
    if mid in ('ego_maze2d_port', 'tier_extreme_maze'):
        return {
            'cruise_z': 1.5,
            'auto_inflate_max': 0.22,
        }
    if MAPS[mid]['family'] != 'official':
        return {}
    overrides: Dict[str, Any] = {
        'cruise_z': 1.0,
        'auto_map_margin': 3.0,
        'true_3d_astar': True,
        'z_band': 1.5,
        'vertical_cost_scale': 1.25,
    }
    if mid == 'official_perlin':
        overrides['cruise_z'] = 1.0
        overrides['z_band'] = 0.8
    if mid in ('official_maze2d', 'official_maze3d'):
        overrides.update({
            'cruise_z': 1.0,
            'true_3d_astar': True,
            'z_band': 2.5 if mid == 'official_maze3d' else 0.6,
            'vertical_cost_scale': 1.25 if mid == 'official_maze3d' else 2.0,
            'max_vel': 0.85,
            'resolution': 0.15 if mid == 'official_maze3d' else 0.2,
            'auto_inflate_max': 0.18 if mid == 'official_maze3d' else 0.14,
            'seal_boundary_layers': 1,
            'local_goal_lookahead': 0.55,
        })
    if mid == 'official_posts':
        overrides['auto_inflate_max'] = 0.22
    return overrides



def ego_planner_overrides(map_id: str) -> Dict[str, Any]:
    """Per-map Path B inflation / clearance / grid extent."""
    mid = _norm(map_id, 'ego')
    if mid == 'dense_field':
        return {
            'grid_map/obstacles_inflation': 0.16,
            'optimization/dist0': 0.55,
        }
    if mid == 'official_perlin':
        # Cover start lanes west of the ±25×±13 Perlin box (swarm x≈±27).
        return {
            'grid_map/obstacles_inflation': 0.08,
            'optimization/dist0': 0.35,
            'grid_map/map_size_x': 60.0,
            'grid_map/map_size_y': 32.0,
            'grid_map/map_size_z': 5.0,
            'grid_map/local_update_range_x': 5.5,
            'grid_map/local_update_range_y': 5.5,
            'grid_map/local_update_range_z': 4.5,
            'grid_map/virtual_ceil_height': 3.5,
            'grid_map/visualization_truncate_height': 2.5,
        }
    if mid in ('official_maze2d', 'official_maze3d'):
        return {
            'grid_map/obstacles_inflation': 0.05,
            'optimization/dist0': 0.25,
        }
    if mid == 'official_forest':
        # Acceptance measures >=0.30 m clearance. Defaults (inflation 0.12)
        # tracked ~0.22 m from trunks; funnel-corner replans clipped ~0.11
        # with 0.28, while 0.40 made sensed obstacles "appear suddenly" and
        # latched EMERGENCY_STOP mid-mission. 0.32 + slower planned speed
        # keeps margin without starving the planner.
        return {
            # 0.32 starved A* in the thinned forest (repeated 'traj 1 failed'
            # -> parked beside trunks); 0.28 keeps hard clearance without
            # over-constraining the 1.6 m-spaced corridors.
            'grid_map/obstacles_inflation': 0.28,
            'optimization/dist0': 0.55,
            # 0.65 cleared with huge margin (min 0.61 m) but ran the 280 s
            # eval clock out on the last funnel leg; 0.75 trades back time.
            # Slow-and-steady profile: replans keep up with the vehicle,
            # avoiding both corner grazes and post-failure extrapolation
            # runaways. Mission fits the 420 s eval window.
            'optimization/max_vel': 0.60,
            'optimization/max_acc': 1.0,
            'manager/max_vel': 0.60,
        }
    if mid == 'official_posts':
        return {
            'grid_map/obstacles_inflation': 0.08,
            'optimization/dist0': 0.35,
            'grid_map/map_size_x': 30.0,
            'grid_map/map_size_y': 30.0,
        }
    return {}



def gcopter_planner_overrides(map_id: str) -> Dict[str, Any]:
    """Per-map Path C DilateRadius / MapBound."""
    mid = _norm(map_id, 'gcopter')
    if mid == 'official_forest':
        return {
            'DilateRadius': 0.22,
            'MapBound': [-18.0, 18.0, -12.0, 12.0, 0.0, 4.0],
        }
    if mid == 'official_perlin':
        return {
            'DilateRadius': 0.18,
            'MapBound': [-26.0, 26.0, -15.0, 15.0, 0.0, 5.0],
        }
    if mid == 'official_posts':
        return {
            'DilateRadius': 0.15,
            'MapBound': [-12.0, 12.0, -12.0, 12.0, 0.0, 4.0],
        }
    if mid == 'official_maze2d':
        return {
            'DilateRadius': 0.10,
            'MapBound': [-12.0, 12.0, -12.0, 12.0, 0.0, 4.0],
        }
    if mid == 'official_maze3d':
        # Fine voxels + light dilate so Voronoi plate holes stay open for 3D A*.
        return {
            'DilateRadius': 0.05,
            'VoxelWidth': 0.12,
            'MapBound': [-12.0, 12.0, -12.0, 12.0, 0.0, 5.5],
            'CruiseHeight': 1.0,
        }
    # Homemade dense fills a wide RViz frame (no perimeter walls).
    if mid == 'dense_field':
        return {
            'DilateRadius': 0.14,
            'VoxelWidth': 0.18,
            'MapBound': [-8.0, 48.0, -8.0, 32.0, 0.0, 4.0],
        }
    if mid in ('narrow_corridor', 'ego_maze2d_port', 'ego_forest_port'):
        return {
            'DilateRadius': 0.12,
            'MapBound': [-2.0, 22.0, -2.0, 12.0, 0.0, 4.0],
        }
    if mid == 'sparse':
        return {
            'DilateRadius': 0.20,
            'MapBound': [-8.0, 10.0, -8.0, 10.0, 0.0, 4.0],
        }
    return {}


# Maps shown in the web dashboard (hidden ones remain launchable via CLI).
DASHBOARD_MAP_IDS = [
    'official_forest',
    'official_perlin',
    'official_posts',
    'official_maze2d',
    'official_maze3d',
    'dense_field',
    'narrow_corridor',
]

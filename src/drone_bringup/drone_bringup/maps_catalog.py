"""Catalog of portable obstacle maps for drone_bringup + dashboard.

Families
--------
homemade_*  — drone_map (cylinders / walls / EGO-style ports) → /map/obstacles
official_*  — EGO random_forest or mockamap → /map_generator/global_cloud

A cloud_bridge republishes to the alternate topic so any planner can consume any map.
"""

from __future__ import annotations

import math
from typing import Any, Dict

# Poses used when a map does not override start/goal.
POSE_HOMEMADE = {
    'init_x': 2.0, 'init_y': 12.0, 'init_z': 1.5,
    'goal_x': 40.0, 'goal_y': 12.0, 'goal_z': 1.5,
}
# Acceptance scenario 4 (static avoidance) on official random forest.
# Stays inside catalog envelope (~±17 x, ±11 y) with flyable gaps among cylinders/rings.
#
# Lap 1 — axis-aligned rectangle (NW→NE→SE→SW):
# Seed-1 forest: outer rectangle along proven open corridors (±8×±6).
# Smaller inward squares put Path C spawn inside DilateRadius ("Start nudged").
OFFICIAL_FOREST_RECT_WAYPOINTS = (
    (-8.0, 6.0, 1.0),
    (8.0, 6.0, 1.0),
    (8.0, -6.0, 1.0),
    (-8.0, -6.0, 1.0),
)
# Lap 2 — funnel / hourglass: diagonal → wide → diagonal → wide.
# After finishing the rectangle at SW (-8,-6):
#   diagonal SW→NE, wide along north, diagonal NW→SE, wide along south.
OFFICIAL_FOREST_FUNNEL_WAYPOINTS = (
    (8.0, 6.0, 1.0),    # diagonal to NE
    (-8.0, 6.0, 1.0),   # wide west along north edge
    (8.0, -6.0, 1.0),   # diagonal to SE
    (-8.0, -6.0, 1.0),  # wide west along south edge
)
# Back-compat alias: rectangle only (mission builder appends funnel for cycle≥2).
OFFICIAL_FOREST_LOOP_WAYPOINTS = OFFICIAL_FOREST_RECT_WAYPOINTS


def official_forest_mission_waypoints(cycles: int = 2):
    """Build scenario-4 mission: 1× rectangle, then (cycles-1)× funnel laps."""
    n = max(1, int(cycles))
    mission = list(OFFICIAL_FOREST_RECT_WAYPOINTS)
    if n >= 2:
        mission.extend(OFFICIAL_FOREST_FUNNEL_WAYPOINTS * (n - 1))
    return mission

# Legacy dense_field loop (kept for scripts / older notes).
# Slightly inward of the prior dense square; keeps Path C/FP goals free.
DENSE_FIELD_LOOP_WAYPOINTS = (
    (15.0, 11.0, 1.5),
    (32.0, 11.0, 1.5),
    (32.0, 15.0, 1.5),
    (15.0, 15.0, 1.5),
)

# Planner comparative benchmark: one closed square per map (spawn at corner 0).
# Forest envelope ~±17×±11; dense_field span ~48×32 — sizes must differ.
BENCHMARK_SQUARE_CORNERS = {
    'official_forest': OFFICIAL_FOREST_RECT_WAYPOINTS,
    'dense_field': DENSE_FIELD_LOOP_WAYPOINTS,
}


def benchmark_square_corners(map_id: str):
    """Four corners of the benchmark square for a map (CCW / CW consistent)."""
    mid = normalize_map_id(map_id)
    corners = BENCHMARK_SQUARE_CORNERS.get(mid)
    if corners is None:
        raise KeyError(
            f'no benchmark square for map={mid!r}; '
            f'known={sorted(BENCHMARK_SQUARE_CORNERS)}')
    return corners


def benchmark_square_waypoints(map_id: str):
    """Goals that close the square when the vehicle already spawns at corner 0."""
    corners = benchmark_square_corners(map_id)
    # Publish corners 1→2→3→0 so the flown path is four edges, not a diagonal.
    return [corners[1], corners[2], corners[3], corners[0]]


def pose_for_benchmark_square(map_id: str) -> Dict[str, float]:
    """Spawn on corner 0; final goal is return to corner 0 after the square."""
    c0 = benchmark_square_corners(map_id)[0]
    return {
        'init_x': float(c0[0]),
        'init_y': float(c0[1]),
        'init_z': float(c0[2]),
        'goal_x': float(c0[0]),
        'goal_y': float(c0[1]),
        'goal_z': float(c0[2]),
    }
POSE_OFFICIAL = {
    'init_x': -15.0, 'init_y': 0.0, 'init_z': 1.0,
    'goal_x': 15.0, 'goal_y': 0.0, 'goal_z': 1.0,
}
POSE_SPARSE = {
    'init_x': 0.0, 'init_y': 0.0, 'init_z': 1.5,
    'goal_x': 2.0, 'goal_y': 1.0, 'goal_z': 1.5,
}
POSE_MAZE = {
    'init_x': -9.0, 'init_y': 0.0, 'init_z': 1.0,
    'goal_x': 9.0, 'goal_y': 0.0, 'goal_z': 1.0,
}
POSE_PERLIN = {
    'init_x': -22.0, 'init_y': 0.0, 'init_z': 1.0,
    'goal_x': 22.0, 'goal_y': 0.0, 'goal_z': 1.0,
}
POSE_POSTS = {
    'init_x': -8.0, 'init_y': 0.0, 'init_z': 1.0,
    'goal_x': 8.0, 'goal_y': 0.0, 'goal_z': 1.0,
}
POSE_MAZE3D = {
    'init_x': -6.0, 'init_y': 0.0, 'init_z': 1.0,
    'goal_x': 6.0, 'goal_y': 0.0, 'goal_z': 1.0,
}
POSE_ASYMMETRIC = {
    'init_x': 2.0, 'init_y': 6.0, 'init_z': 1.5,
    'goal_x': 40.0, 'goal_y': 24.0, 'goal_z': 1.5,
}

DIFFICULTIES = frozenset({'simple', 'medium', 'complex', 'extreme'})


def bounds_dict(
    xmin: float, ymin: float, zmin: float,
    xmax: float, ymax: float, zmax: float,
) -> Dict[str, float]:
    return {
        'xmin': float(xmin), 'ymin': float(ymin), 'zmin': float(zmin),
        'xmax': float(xmax), 'ymax': float(ymax), 'zmax': float(zmax),
    }


def bounds_from_box(box: tuple) -> Dict[str, float]:
    return bounds_dict(box[0], box[1], box[2], box[3], box[4], box[5])

# Path D starts inside / on the free edge of each map so fog sensing sees
# obstacles immediately (nav goals are frontiers — catalog "goal_*" unused).
FUEL_EXPLORE_POSE: Dict[str, Dict[str, float]] = {
    'dense_field': dict(POSE_HOMEMADE),
    'sparse': dict(POSE_SPARSE),
    'narrow_corridor': {
            'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
            'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
        },
    'ego_maze2d_port': {
        'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
        'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
    },
    'ego_forest_port': {
        'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
        'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
    },
    # Inside clear_y corridor (not outside at x=-15).
    'official_forest': {
        'init_x': -12.0, 'init_y': 0.0, 'init_z': 1.0,
        'goal_x': 12.0, 'goal_y': 0.0, 'goal_z': 1.0,
    },
    'official_perlin': {
        'init_x': -20.0, 'init_y': 0.0, 'init_z': 1.2,
        'goal_x': 20.0, 'goal_y': 0.0, 'goal_z': 1.2,
    },
    'official_posts': {
        'init_x': -7.0, 'init_y': 0.0, 'init_z': 1.0,
        'goal_x': 7.0, 'goal_y': 0.0, 'goal_z': 1.0,
    },
    'official_maze2d': {
        'init_x': -8.0, 'init_y': 0.0, 'init_z': 1.0,
        'goal_x': 8.0, 'goal_y': 0.0, 'goal_z': 1.0,
    },
    'official_maze3d': {
        'init_x': -5.0, 'init_y': 0.0, 'init_z': 1.0,
        'goal_x': 5.0, 'goal_y': 0.0, 'goal_z': 1.0,
    },
    'tier_simple_open': dict(POSE_SPARSE),
    'tier_medium_corridor': dict(POSE_HOMEMADE),
    'tier_complex_forest': {
        'init_x': -12.0, 'init_y': 0.0, 'init_z': 1.0,
        'goal_x': 12.0, 'goal_y': 0.0, 'goal_z': 1.0,
    },
    'tier_extreme_maze': {
        'init_x': -8.0, 'init_y': 0.0, 'init_z': 1.0,
        'goal_x': 8.0, 'goal_y': 0.0, 'goal_z': 1.0,
    },
    'forest_wide': dict(POSE_HOMEMADE),
    'dense_asymmetric': dict(POSE_ASYMMETRIC),
}

MAPS: Dict[str, Dict[str, Any]] = {
    # ---- homemade / Path A frames -------------------------------------------------
    'dense_field': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_dense.yaml',
        'pose': POSE_HOMEMADE,
        'label_en': 'Dense field',
        'label_zh': '密集场',
        'desc_en': 'Cylinders + spheres (no border walls), wide field',
        'desc_zh': '圆柱 / 球体（无边界墙），大场地',
        'source': 'drone_map · map_mode=dense_field',
        'difficulty': 'complex',
        'seed': 42,
        'obstacle_family': 'cylinders',
        'safety_radius': 0.4,
        # Match map_generator DENSE_FIELD envelope (no perimeter walls).
        'bounds': bounds_dict(-8.0, -8.0, 0.0, 48.0, 32.0, 4.0),
    },
    'sparse': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_sparse.yaml',
        'pose': POSE_SPARSE,
        'label_en': 'Sparse field',
        'label_zh': '稀疏场',
        'desc_en': 'Open field with few obstacles',
        'desc_zh': '开阔场地，少量障碍',
        'source': 'drone_map · map_mode=sparse',
        'difficulty': 'simple',
        'seed': 42,
        'obstacle_family': 'sparse_cylinders',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-6.0, -6.0, 0.0, 8.0, 8.0, 3.0),
    },
    'narrow_corridor': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_narrow.yaml',
        # Match map_narrow.yaml start/goal and map_generator NARROW_CORRIDOR envelope.
        'pose': {
            'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
            'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
        },
        'label_en': 'Narrow corridor',
        'label_zh': '狭窄通道',
        'desc_en': 'S-bend: 3×1.6 m staggered doors + side clutter (PLAN §5.3)',
        'desc_zh': 'S 弯：五道 1.6 m 错位门缝 + 侧湾障碍（PLAN §5.3）',
        'source': 'drone_map · map_mode=narrow_corridor',
        'difficulty': 'medium',
        'seed': 42,
        'obstacle_family': 's_bend_gates',
        'safety_radius': 0.4,
        # Full flyable envelope from MapGenerator::boundsForMode(NARROW_CORRIDOR).
        'bounds': bounds_dict(-2.0, -2.0, 0.0, 22.0, 12.0, 4.0),
    },
    'ego_maze2d_port': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_ego_narrow.yaml',
        # Match map_ego_narrow.yaml start/goal (not dense-field POSE_HOMEMADE).
        'pose': {
            'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
            'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
        },
        'label_en': 'Maze2D (drone_map)',
        'label_zh': '迷宫2D（drone_map）',
        'desc_en': 'Recursive-division maze in drone_map frame',
        'desc_zh': '递归分割迷宫（drone_map 坐标系）',
        'source': 'drone_map · map_mode=ego_maze2d',
        'difficulty': 'extreme',
        'seed': 510,
        'obstacle_family': 'maze2d',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-1.0, -1.0, 0.0, 21.0, 11.0, 3.0),
    },
    'ego_forest_port': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_ego_dense.yaml',
        # Match map_ego_dense.yaml start/goal (not dense-field POSE_HOMEMADE).
        'pose': {
            'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
            'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
        },
        'label_en': 'Random forest (drone_map)',
        'label_zh': '随机森林（drone_map）',
        'desc_en': 'Cylinders + rings in drone_map frame',
        'desc_zh': '圆柱 + 圆环（drone_map 坐标系）',
        'source': 'drone_map · map_mode=ego_dense_forest',
        'difficulty': 'complex',
        'seed': 42,
        'obstacle_family': 'forest_cylinders',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-1.0, -1.0, 0.0, 21.0, 11.0, 3.0),
    },
    # ---- map_generator / mockamap ------------------------------------------------
    'official_forest': {
        'family': 'official',
        'backend': 'random_forest',
        'pose': POSE_OFFICIAL,
        'label_en': 'Random forest',
        'label_zh': '随机森林',
        'desc_en': 'Cylinders + rings (Path B/C default)',
        'desc_zh': '圆柱 + 圆环（路径 B/C 默认）',
        'source': 'map_generator · random_forest',
        'difficulty': 'complex',
        'seed': 1,
        'obstacle_family': 'forest_cylinders',
        'safety_radius': 0.35,
        'bounds': bounds_dict(-17.0, -11.0, 0.0, 17.0, 11.0, 3.5),
    },
    'official_perlin': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 1,
        'pose': POSE_PERLIN,
        'label_en': 'Perlin3D',
        'label_zh': 'Perlin3D',
        'desc_en': 'Fractal Perlin voxel blobs (start outside box)',
        'desc_zh': '分形 Perlin 体素（起点在障碍盒外）',
        'source': 'mockamap · type=1',
        'difficulty': 'complex',
        'seed': 511,
        'obstacle_family': 'perlin_voxels',
        'safety_radius': 0.35,
        'bounds': bounds_dict(-26.0, -14.0, 0.0, 26.0, 14.0, 5.0),
    },
    'official_posts': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 2,
        'pose': POSE_POSTS,
        'label_en': 'Posts',
        'label_zh': '立柱',
        'desc_en': 'Hollow rectangular pillars (thinned)',
        'desc_zh': '空心矩形立柱（稀疏化）',
        'source': 'mockamap · type=2',
        'difficulty': 'medium',
        'seed': 511,
        'obstacle_family': 'pillars',
        'safety_radius': 0.35,
        'bounds': bounds_dict(-11.0, -11.0, 0.0, 11.0, 11.0, 4.0),
    },
    'official_maze2d': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 3,
        'pose': POSE_MAZE,
        'label_en': 'Maze2D',
        'label_zh': '迷宫2D',
        'desc_en': 'Recursive-division 2D maze (wider roads)',
        'desc_zh': '递归分割二维迷宫（加宽通道）',
        'source': 'mockamap · type=3',
        'difficulty': 'extreme',
        'seed': 510,
        'obstacle_family': 'maze2d',
        'safety_radius': 0.30,
        'bounds': bounds_dict(-11.0, -11.0, 0.0, 11.0, 11.0, 2.5),
    },
    'official_maze3d': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 4,
        'pose': POSE_MAZE3D,
        'label_en': 'Maze3D',
        'label_zh': '迷宫3D',
        'desc_en': 'Voronoi-like 3D maze (Z in [0,z], wider passages)',
        'desc_zh': 'Voronoi 三维迷宫（Z≥0，加宽通道）',
        'source': 'mockamap · type=4',
        'difficulty': 'extreme',
        'seed': 1,
        'obstacle_family': 'maze3d',
        'safety_radius': 0.30,
        'bounds': bounds_dict(-11.0, -11.0, 0.0, 11.0, 11.0, 4.0),
    },
    # ---- tier presets (review dataset) --------------------------------------------
    'tier_simple_open': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_sparse.yaml',
        'pose': POSE_SPARSE,
        'based_on': 'sparse',
        'label_en': 'Tier — simple open',
        'label_zh': '难度档 — 开阔',
        'desc_en': 'Preset: sparse open field (tier simple)',
        'desc_zh': '预设：稀疏开阔场（简单档）',
        'source': 'tier preset · based on sparse',
        'difficulty': 'simple',
        'seed': 42,
        'obstacle_family': 'sparse_cylinders',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-6.0, -6.0, 0.0, 8.0, 8.0, 3.0),
    },
    'tier_medium_corridor': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_narrow.yaml',
        'pose': {
            'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
            'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
        },
        'based_on': 'narrow_corridor',
        'label_en': 'Tier — medium corridor',
        'label_zh': '难度档 — 通道',
        'desc_en': 'Preset: narrow gate corridor (tier medium)',
        'desc_zh': '预设：狭窄门洞通道（中等档）',
        'source': 'tier preset · based on narrow_corridor',
        'difficulty': 'medium',
        'seed': 42,
        'obstacle_family': 'gate_walls',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-2.0, -2.0, 0.0, 22.0, 12.0, 4.0),
    },
    'tier_complex_forest': {
        'family': 'official',
        'backend': 'random_forest',
        'pose': POSE_OFFICIAL,
        'based_on': 'official_forest',
        'label_en': 'Tier — complex forest',
        'label_zh': '难度档 — 森林',
        'desc_en': 'Preset: EGO random forest (tier complex)',
        'desc_zh': '预设：EGO 随机森林（复杂档）',
        'source': 'tier preset · based on official_forest',
        'difficulty': 'complex',
        'seed': 1,
        'obstacle_family': 'forest_cylinders',
        'safety_radius': 0.35,
        'bounds': bounds_dict(-17.0, -11.0, 0.0, 17.0, 11.0, 3.5),
    },
    'tier_extreme_maze': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 3,
        'pose': POSE_MAZE,
        'based_on': 'official_maze2d',
        'label_en': 'Tier — extreme maze',
        'label_zh': '难度档 — 迷宫',
        'desc_en': 'Preset: recursive 2D maze (tier extreme)',
        'desc_zh': '预设：递归二维迷宫（极限档）',
        'source': 'tier preset · based on official_maze2d',
        'difficulty': 'extreme',
        'seed': 510,
        'obstacle_family': 'maze2d',
        'safety_radius': 0.30,
        'bounds': bounds_dict(-11.0, -11.0, 0.0, 11.0, 11.0, 2.5),
    },
    # ---- optional larger variants ---------------------------------------------------
    'forest_wide': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_forest_wide.yaml',
        'pose': POSE_HOMEMADE,
        'based_on': 'ego_forest_port',
        'label_en': 'Wide random forest',
        'label_zh': '加宽随机森林',
        'desc_en': 'Denser ego-style forest in drone_map frame',
        'desc_zh': '更密的 ego 风格森林（drone_map 坐标系）',
        'source': 'drone_map · map_mode=ego_dense_forest (wide)',
        'difficulty': 'complex',
        'seed': 43,
        'obstacle_family': 'forest_cylinders',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-1.0, -1.0, 0.0, 21.0, 11.0, 3.0),
    },
    'dense_asymmetric': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_dense_asymmetric.yaml',
        'pose': POSE_ASYMMETRIC,
        'based_on': 'dense_field',
        'label_en': 'Dense asymmetric',
        'label_zh': '密集非对称',
        'desc_en': 'Dense field with diagonal start–goal bias',
        'desc_zh': '密集场 + 对角起终点偏置',
        'source': 'drone_map · map_mode=dense_field (asymmetric)',
        'difficulty': 'complex',
        'seed': 77,
        'obstacle_family': 'cylinders',
        'safety_radius': 0.4,
        'bounds': bounds_dict(-8.0, -8.0, 0.0, 48.0, 32.0, 4.0),
    },
}

DEFAULT_MAP_BY_PLANNER = {
    'homemade': 'dense_field',
    'ego': 'official_forest',
    'gcopter': 'official_forest',
    'fuel_explore': 'narrow_corridor',
    'mighty': 'official_forest',
    'fast_planner': 'official_forest',
    'rl': 'dense_field',
    'sac': 'dense_field',
}

# Path D fog/frontier AABB (map frame): (min_x, min_y, min_z, max_x, max_y, max_z).
# Sized from generator extents + start/goal margin (see MAPS.md).
EXPLORE_BOX: Dict[str, tuple] = {
    'dense_field': (-8.0, -8.0, 0.0, 48.0, 32.0, 4.0),
    'sparse': (-6.0, -6.0, 0.0, 8.0, 8.0, 3.0),
    'narrow_corridor': (-2.0, -2.0, 0.0, 22.0, 12.0, 4.0),
    'ego_maze2d_port': (-1.0, -1.0, 0.0, 21.0, 11.0, 3.0),
    'ego_forest_port': (-1.0, -1.0, 0.0, 21.0, 11.0, 3.0),
    # official_forest cloud ~26×20; init x=-15 sits outside.
    'official_forest': (-17.0, -11.0, 0.0, 17.0, 11.0, 3.5),
    # mockamap lengths are full box sizes centered near origin.
    'official_perlin': (-26.0, -14.0, 0.0, 26.0, 14.0, 5.0),
    'official_posts': (-11.0, -11.0, 0.0, 11.0, 11.0, 4.0),
    'official_maze2d': (-11.0, -11.0, 0.0, 11.0, 11.0, 2.5),
    'official_maze3d': (-11.0, -11.0, 0.0, 11.0, 11.0, 4.0),
    'tier_simple_open': (-6.0, -6.0, 0.0, 8.0, 8.0, 3.0),
    'tier_medium_corridor': (-2.0, -2.0, 0.0, 22.0, 12.0, 4.0),
    'tier_complex_forest': (-17.0, -11.0, 0.0, 17.0, 11.0, 3.5),
    'tier_extreme_maze': (-11.0, -11.0, 0.0, 11.0, 11.0, 2.5),
    'forest_wide': (-1.0, -1.0, 0.0, 21.0, 11.0, 3.0),
    'dense_asymmetric': (-8.0, -8.0, 0.0, 48.0, 32.0, 4.0),
}

ALIASES = {
    'dense': 'dense_field',
    'homemade': 'dense_field',
    'homemade_dense': 'dense_field',
    'narrow': 'narrow_corridor',
    'forest': 'official_forest',
    'ego_forest': 'official_forest',
    'random_forest': 'official_forest',
    'perlin': 'official_perlin',
    'perlin3d': 'official_perlin',
    'posts': 'official_posts',
    'maze': 'official_maze2d',
    'maze2d': 'official_maze2d',
    'maze3d': 'official_maze3d',
    'ego_maze': 'ego_maze2d_port',
    'ego_maze2d': 'ego_maze2d_port',
    'tier_simple': 'tier_simple_open',
    'tier_medium': 'tier_medium_corridor',
    'tier_complex': 'tier_complex_forest',
    'tier_extreme': 'tier_extreme_maze',
    'simple_open': 'tier_simple_open',
    'medium_corridor': 'tier_medium_corridor',
    'complex_forest': 'tier_complex_forest',
    'extreme_maze': 'tier_extreme_maze',
}


def resolve_map_id(map_id: str, planner: str = '') -> str:
    """Canonical map id, following optional based_on for override lookup."""
    mid = normalize_map_id(map_id, planner=planner)
    return MAPS[mid].get('based_on', mid)


def normalize_map_id(map_id: str, planner: str = '') -> str:
    raw = (map_id or '').strip().lower()
    if not raw or raw in ('auto', 'default'):
        return DEFAULT_MAP_BY_PLANNER.get(planner, 'official_forest')
    mapped = ALIASES.get(raw, raw)
    if mapped not in MAPS:
        raise ValueError(
            f"Unknown map='{map_id}'. Choose one of: {', '.join(sorted(MAPS))}")
    return mapped


def catalog_metadata(map_id: str, seed: int | None = None) -> Dict[str, Any]:
    """JSON-friendly metadata for /map/metadata and tests."""
    mid = normalize_map_id(map_id)
    meta = MAPS[mid]
    pose = meta['pose']
    effective_seed = int(seed) if seed is not None else int(meta['seed'])
    return {
        'id': mid,
        'family': meta['family'],
        'backend': meta['backend'],
        'difficulty': meta['difficulty'],
        'seed': effective_seed,
        'obstacle_family': meta['obstacle_family'],
        'safety_radius': float(meta['safety_radius']),
        'bounds': dict(meta['bounds']),
        'init_x': pose['init_x'],
        'init_y': pose['init_y'],
        'init_z': pose['init_z'],
        'goal_x': pose['goal_x'],
        'goal_y': pose['goal_y'],
        'goal_z': pose['goal_z'],
        'based_on': meta.get('based_on'),
    }


def homemade_connectivity_sanity(map_id: str, margin: float = 0.35) -> Dict[str, Any]:
    """Lightweight start/goal checks for homemade maps (no full path planning)."""
    mid = normalize_map_id(map_id)
    meta = MAPS[mid]
    if meta['family'] != 'homemade':
        return {'ok': True, 'skipped': True, 'reason': 'not homemade'}
    pose = meta['pose']
    bounds = meta['bounds']
    ix, iy, iz = pose['init_x'], pose['init_y'], pose['init_z']
    gx, gy, gz = pose['goal_x'], pose['goal_y'], pose['goal_z']
    m = float(margin)

    def _inside(x: float, y: float, z: float) -> bool:
        return (
            bounds['xmin'] + m <= x <= bounds['xmax'] - m
            and bounds['ymin'] + m <= y <= bounds['ymax'] - m
            and bounds['zmin'] <= z <= bounds['zmax']
        )

    sep = math.hypot(gx - ix, gy - iy)
    ok = _inside(ix, iy, iz) and _inside(gx, gy, gz) and sep >= 1.0
    return {
        'ok': ok,
        'skipped': False,
        'map_id': mid,
        'separation_m': sep,
        'init_inside': _inside(ix, iy, iz),
        'goal_inside': _inside(gx, gy, gz),
    }


def pose_for_map(map_id: str, planner: str = '') -> Dict[str, float]:
    """Init/goal pose for a map, with Path D overrides when planner is fuel_explore."""
    mid = normalize_map_id(map_id, planner=planner)
    pose = dict(MAPS[mid]['pose'])
    pl = (planner or '').strip().lower()
    if pl in ('fuel_explore', 'fuel', 'd', 'path_d'):
        override = FUEL_EXPLORE_POSE.get(mid)
        if override:
            pose.update(override)
    return pose


def map_public_info() -> Dict[str, Dict[str, Any]]:
    """JSON-friendly catalog for the dashboard API (curated list only)."""
    out = {}
    for key in DASHBOARD_MAP_IDS:
        meta = MAPS.get(key)
        if meta is None:
            continue
        pose = meta['pose']
        fuel = FUEL_EXPLORE_POSE.get(key, pose)
        out[key] = {
            'id': key,
            'family': meta['family'],
            'backend': meta['backend'],
            'label_en': meta['label_en'],
            'label_zh': meta['label_zh'],
            'desc_en': meta['desc_en'],
            'desc_zh': meta['desc_zh'],
            'source': meta['source'],
            'difficulty': meta['difficulty'],
            'seed': meta['seed'],
            'obstacle_family': meta['obstacle_family'],
            'safety_radius': meta['safety_radius'],
            'bounds': dict(meta['bounds']),
            'goal_x': pose['goal_x'],
            'goal_y': pose['goal_y'],
            'goal_z': pose['goal_z'],
            'init_x': pose['init_x'],
            'init_y': pose['init_y'],
            'init_z': pose['init_z'],
            # Path D trigger / spawn (near free start — not the far nav goal).
            'explore_init_x': fuel['init_x'],
            'explore_init_y': fuel['init_y'],
            'explore_init_z': fuel['init_z'],
        }
    return out


def explore_box_params(map_id: str, planner: str = 'fuel_explore') -> Dict[str, float]:
    """ROS parameters for Path D local_sensing / exploration AABB."""
    mid = normalize_map_id(map_id, planner=planner)
    box = EXPLORE_BOX.get(mid, (-12.0, -12.0, 0.0, 12.0, 12.0, 3.5))
    return {
        'box_min_x': float(box[0]),
        'box_min_y': float(box[1]),
        'box_min_z': float(box[2]),
        'box_max_x': float(box[3]),
        'box_max_y': float(box[4]),
        'box_max_z': float(box[5]),
    }


def fuel_explore_params(map_id: str) -> Dict[str, Any]:
    """Per-map Path D FSM / sensing tuning (not FUEL upstream warehouse)."""
    mid = normalize_map_id(map_id, planner='fuel_explore')
    pose = pose_for_map(mid, planner='fuel_explore')
    dx = float(pose['goal_x'] - pose['init_x'])
    dy = float(pose['goal_y'] - pose['init_y'])
    span = max(math.hypot(dx, dy), 1e-3)
    base: Dict[str, Any] = {
        'explore_dir_x': dx / span,
        'explore_dir_y': dy / span,
        'seed_viewpoints': True,
        'arrive_tol': 0.70,
        'min_goal_sep': 2.5,
        'min_travel_m': 1.5,
        'goal_cooldown': 4.0,
        'prefer_forward_weight': 2.5,
        'prefer_size_weight': 0.12,
        'frontier_min_size': 4,
    }
    if mid == 'narrow_corridor' or mid == 'tier_medium_corridor':
        base.update({
            'prefer_forward_weight': 4.5,
            'min_goal_sep': 3.0,
            'min_travel_m': 2.0,
            'goal_cooldown': 5.0,
            'frontier_min_size': 5,
            'sensing_radius': 3.5,
            'resolution': 0.35,
        })
    elif mid == 'dense_field' or mid == 'dense_asymmetric':
        base.update({
            'prefer_forward_weight': 3.0,
            'min_travel_m': 1.8,
        })
    elif mid.startswith('official') or mid == 'tier_complex_forest':
        base.update({
            'prefer_forward_weight': 3.5,
            'min_travel_m': 2.0,
        })
    return base


def homemade_planner_overrides(map_id: str, planner: str = 'homemade') -> Dict[str, Any]:
    """Light Path A hints. Grid AABB + inflate now auto-fit from the cloud."""
    mid = normalize_map_id(map_id, planner=planner)
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
    mid = normalize_map_id(map_id, planner='ego')
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
    mid = normalize_map_id(map_id, planner='gcopter')
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

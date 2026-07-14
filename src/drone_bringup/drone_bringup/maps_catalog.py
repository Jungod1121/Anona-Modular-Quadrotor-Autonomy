"""Catalog of portable obstacle maps for drone_bringup + dashboard.

Families
--------
homemade_*  — drone_map (cylinders / walls / EGO-style ports) → /map/obstacles
official_*  — EGO random_forest or mockamap → /map_generator/global_cloud

A cloud_bridge republishes to the alternate topic so any planner can consume any map.
"""

from __future__ import annotations

from typing import Any, Dict

# Poses used when a map does not override start/goal.
POSE_HOMEMADE = {
    'init_x': 1.0, 'init_y': 5.0, 'init_z': 1.5,
    'goal_x': 17.0, 'goal_y': 5.0, 'goal_z': 1.5,
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

MAPS: Dict[str, Dict[str, Any]] = {
    # ---- homemade / Path A frames -------------------------------------------------
    'dense_field': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_dense.yaml',
        'pose': POSE_HOMEMADE,
        'label_en': 'Homemade dense field',
        'label_zh': '自研密集场地',
        'desc_en': 'Cylinders + spheres + boundary walls (early project map)',
        'desc_zh': '圆柱/球体 + 边界墙（项目初期地图）',
        'source': 'drone_map · map_mode=dense_field',
    },
    'sparse': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_sparse.yaml',
        'pose': POSE_SPARSE,
        'label_en': 'Homemade sparse',
        'label_zh': '自研稀疏场地',
        'desc_en': 'Open field with few obstacles',
        'desc_zh': '开阔场地，少量障碍',
        'source': 'drone_map · map_mode=sparse',
    },
    'narrow_corridor': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_narrow.yaml',
        'pose': POSE_HOMEMADE,
        'label_en': 'Homemade narrow corridor',
        'label_zh': '自研狭窄通道',
        'desc_en': 'Gate walls + side clutter',
        'desc_zh': '门洞墙体 + 侧湾障碍',
        'source': 'drone_map · map_mode=narrow_corridor',
    },
    'ego_maze2d_port': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_ego_narrow.yaml',
        'pose': POSE_HOMEMADE,
        'label_en': 'EGO maze2D (homemade port)',
        'label_zh': 'EGO 迷宫2D（自研移植）',
        'desc_en': 'Recursive-division maze ported into drone_map frame',
        'desc_zh': '递归分割迷宫，迁入自研坐标系',
        'source': 'drone_map · map_mode=ego_maze2d',
    },
    'ego_forest_port': {
        'family': 'homemade',
        'backend': 'drone_map',
        'config': 'map_ego_dense.yaml',
        'pose': POSE_HOMEMADE,
        'label_en': 'EGO forest (homemade port)',
        'label_zh': 'EGO 森林（自研移植）',
        'desc_en': 'Cylinders + rings ported into drone_map frame',
        'desc_zh': '圆柱 + 圆环，迁入自研坐标系',
        'source': 'drone_map · map_mode=ego_dense_forest',
    },
    # ---- official EGO ------------------------------------------------------------
    'official_forest': {
        'family': 'official',
        'backend': 'random_forest',
        'pose': POSE_OFFICIAL,
        'label_en': 'Official EGO forest',
        'label_zh': '官方 EGO 森林',
        'desc_en': 'Cylinders + rings (best completeness; Path B/C default)',
        'desc_zh': '圆柱 + 圆环（场景最完备；Path B/C 默认）',
        'source': 'map_generator · random_forest',
    },
    'official_perlin': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 1,
        'pose': POSE_PERLIN,
        'label_en': 'Official mockamap Perlin3D',
        'label_zh': '官方 mockamap Perlin3D',
        'desc_en': 'Fractal Perlin voxel blobs (start outside box)',
        'desc_zh': '分形 Perlin 体素（起点在障碍盒外）',
        'source': 'mockamap · type=1',
    },
    'official_posts': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 2,
        'pose': POSE_POSTS,
        'label_en': 'Official mockamap posts',
        'label_zh': '官方 mockamap 立柱',
        'desc_en': 'Hollow rectangular pillars (thinned)',
        'desc_zh': '空心矩形立柱（稀疏化）',
        'source': 'mockamap · type=2',
    },
    'official_maze2d': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 3,
        'pose': POSE_MAZE,
        'label_en': 'Official mockamap maze2D',
        'label_zh': '官方 mockamap 迷宫2D',
        'desc_en': 'Recursive-division 2D maze (wider roads)',
        'desc_zh': '递归分割二维迷宫（加宽通道）',
        'source': 'mockamap · type=3',
    },
    'official_maze3d': {
        'family': 'official',
        'backend': 'mockamap',
        'mock_type': 4,
        'pose': POSE_MAZE3D,
        'label_en': 'Official mockamap maze3D',
        'label_zh': '官方 mockamap 迷宫3D',
        'desc_en': 'Voronoi-like 3D maze (Z in [0,z], wider passages)',
        'desc_zh': 'Voronoi 三维迷宫（Z≥0，加宽通道）',
        'source': 'mockamap · type=4',
    },
}

DEFAULT_MAP_BY_PLANNER = {
    'homemade': 'dense_field',
    'ego': 'official_forest',
    'gcopter': 'official_forest',
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
}


def normalize_map_id(map_id: str, planner: str = '') -> str:
    raw = (map_id or '').strip().lower()
    if not raw or raw in ('auto', 'default'):
        return DEFAULT_MAP_BY_PLANNER.get(planner, 'official_forest')
    mapped = ALIASES.get(raw, raw)
    if mapped not in MAPS:
        raise ValueError(
            f"Unknown map='{map_id}'. Choose one of: {', '.join(sorted(MAPS))}")
    return mapped


def map_public_info() -> Dict[str, Dict[str, Any]]:
    """JSON-friendly catalog for the dashboard API."""
    out = {}
    for key, meta in MAPS.items():
        out[key] = {
            'id': key,
            'family': meta['family'],
            'backend': meta['backend'],
            'label_en': meta['label_en'],
            'label_zh': meta['label_zh'],
            'desc_en': meta['desc_en'],
            'desc_zh': meta['desc_zh'],
            'source': meta['source'],
            'goal_x': meta['pose']['goal_x'],
            'goal_y': meta['pose']['goal_y'],
            'goal_z': meta['pose']['goal_z'],
            'init_x': meta['pose']['init_x'],
            'init_y': meta['pose']['init_y'],
            'init_z': meta['pose']['init_z'],
        }
    return out


def homemade_planner_overrides(map_id: str, planner: str = 'homemade') -> Dict[str, Any]:
    """Expand Path A occupancy grid so official poses/maps fit."""
    mid = normalize_map_id(map_id, planner=planner)
    if MAPS[mid]['family'] != 'official':
        return {}
    return {
        'map_origin_x': -25.0,
        'map_origin_y': -15.0,
        'map_origin_z': 0.0,
        'map_size_x': 50.0,
        'map_size_y': 30.0,
        'map_size_z': 4.0,
        'resolution': 0.25,
        'inflate_radius': 0.35,
        'cruise_z': 1.0,
    }


def ego_planner_overrides(map_id: str) -> Dict[str, Any]:
    """Per-map Path B inflation / clearance."""
    mid = normalize_map_id(map_id, planner='ego')
    if mid in ('official_maze2d', 'official_maze3d'):
        return {
            'grid_map/obstacles_inflation': 0.05,
            'optimization/dist0': 0.25,
        }
    if mid == 'official_posts':
        return {
            'grid_map/obstacles_inflation': 0.08,
            'optimization/dist0': 0.35,
        }
    return {}


def gcopter_planner_overrides(map_id: str) -> Dict[str, Any]:
    """Per-map Path C DilateRadius / MapBound."""
    mid = normalize_map_id(map_id, planner='gcopter')
    if mid == 'official_perlin':
        return {
            'DilateRadius': 0.25,
            'MapBound': [-26.0, 26.0, -15.0, 15.0, 0.0, 5.0],
        }
    if mid == 'official_posts':
        return {
            'DilateRadius': 0.20,
            'MapBound': [-12.0, 12.0, -12.0, 12.0, 0.0, 4.0],
        }
    if mid in ('official_maze2d', 'official_maze3d'):
        return {
            'DilateRadius': 0.15,
            'MapBound': [-12.0, 12.0, -12.0, 12.0, 0.0, 4.0],
        }
    return {}

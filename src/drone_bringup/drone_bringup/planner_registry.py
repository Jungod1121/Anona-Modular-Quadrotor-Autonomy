"""Canonical planner / topic / rate contracts for drone_bringup.

Single source of truth for dashboard, launches, and evaluation manifests.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

# ---------------------------------------------------------------------------
# Rates (fair comparison)
# ---------------------------------------------------------------------------
RATES = {
    'dynamics_integration_hz': 500.0,
    'state_publish_hz': 100.0,
    'control_hz': 100.0,
    'planner_default_hz': 20.0,
}

# ---------------------------------------------------------------------------
# Topics / frames
# ---------------------------------------------------------------------------
TOPICS = {
    'odom': '/drone/odom',
    'imu': '/drone/imu',
    'path_flown': '/drone/path',
    'goal': '/drone/goal',
    'motor_rpm_cmd': '/drone/motor_rpm_cmd',
    'map_obstacles': '/map/obstacles',
    'map_occupancy': '/map/occupancy',
    'map_metadata': '/map/metadata',
    'map_generator_cloud': '/map_generator/global_cloud',
    'local_goal': '/planner/local_goal',
    'trajectory_cmd': '/planner/trajectory_cmd',
    'trajectory': '/planner/trajectory',
    'status': '/planner/status',
    'diagnostics': '/planner/diagnostics',
    'fallback_active': '/planner/fallback_active',
}

FRAMES = {
    'map': 'map',
    'body': 'base_link',
}

# ---------------------------------------------------------------------------
# Planner registry
# ---------------------------------------------------------------------------
# class: weak | strong | mode | multi | optional
# status: active | experimental | optional | deprecated

PLANNERS: Dict[str, Dict[str, Any]] = {
    'homemade': {
        'id': 'homemade',
        'aliases': ['a', 'path_a', 'dynastar'],
        'label_en': 'Path A — Grid A* + B-spline',
        'label_zh': '路径 A — 栅格 A* + B 样条',
        'class': 'weak',
        'status': 'active',
        'family': 'search',
        'launch': 'homemade_avoidance.launch.py',
        'package': 'drone_planner',
        'principle': 'Deterministic grid A* search with B-spline smoothing',
        'desc_en': 'Dyn-A* search + B-spline optimize on occupancy grid',
        'desc_zh': '动态 A* 搜索 + B 样条优化',
        'publish_rate_hz': 10.0,
        'local_goal_timeout': 2.0,
        'capabilities': ['global_path', 'local_goal', 'trajectory_viz'],
    },
    'ego': {
        'id': 'ego',
        'aliases': ['b', 'path_b'],
        'label_en': 'Path B — EGO rebound B-spline',
        'label_zh': '路径 B — EGO 反弹 B 样条',
        'class': 'strong',
        'status': 'active',
        'family': 'ego',
        'launch': 'ego_avoidance.launch.py',
        'package': 'ego_planner',
        'principle': 'EGO-Planner local rebound B-spline optimization',
        'desc_en': 'ego_planner + map_generator',
        'desc_zh': 'EGO-Planner + 地图生成',
        'publish_rate_hz': 50.0,
        'local_goal_timeout': 0.5,
        'capabilities': ['trajectory_cmd', 'trajectory_viz', 'bridge'],
    },
    'gcopter': {
        'id': 'gcopter',
        'aliases': ['c', 'path_c', 'minco'],
        'label_en': 'Path C — GCOPTER / MINCO',
        'label_zh': '路径 C — GCOPTER / MINCO',
        'class': 'strong',
        'status': 'active',
        'family': 'gcopter',
        'launch': 'gcopter_avoidance.launch.py',
        'package': 'gcopter',
        'principle': 'MINCO / GCOPTER corridor trajectory optimization',
        'desc_en': 'GCOPTER / MINCO trajectory optimization',
        'desc_zh': 'GCOPTER / MINCO 走廊优化',
        'publish_rate_hz': 20.0,
        'local_goal_timeout': 1.0,
        'capabilities': ['trajectory_cmd', 'trajectory_viz'],
    },
    'fuel_explore': {
        'id': 'fuel_explore',
        'aliases': ['d', 'path_d', 'fuel'],
        'label_en': 'Mode D — Frontier exploration (EGO)',
        'label_zh': '模式 D — 边界探索（EGO）',
        'class': 'mode',
        'status': 'active',
        'family': 'exploration',
        'launch': 'fuel_explore.launch.py',
        'package': 'drone_exploration',
        'principle': 'Frontier FSM mission mode using EGO as trajectory backend',
        'desc_en': 'Fog sensing + frontier FSM + EGO backend (not a standalone planner)',
        'desc_zh': '未知区感知 + 边界前沿状态机 + EGO 轨迹后端（非独立规划器）',
        'publish_rate_hz': 10.0,
        'local_goal_timeout': 0.5,
        'capabilities': ['mission', 'uses_ego'],
    },
    'mighty': {
        'id': 'mighty',
        'aliases': ['e', 'path_e'],
        'label_en': 'Path E — MIGHTY HGP',
        'label_zh': '路径 E — MIGHTY HGP',
        'class': 'strong',
        'status': 'active',
        'family': 'mighty',
        'launch': 'mighty_avoidance.launch.py',
        'package': 'mighty',
        'principle': 'MIGHTY HGP / Hermite-LBFGS trajectory optimization',
        'desc_en': 'upstream mit-acl/mighty + mighty_cmd_bridge',
        'desc_zh': '上游 MIGHTY + 指令桥',
        'publish_rate_hz': 50.0,
        'local_goal_timeout': 0.5,
        'capabilities': ['trajectory_cmd', 'trajectory_viz', 'bridge'],
    },
    'fast_planner': {
        'id': 'fast_planner',
        'aliases': ['f', 'path_f', 'fast'],
        'label_en': 'Optional F — Fast-Planner kino (lineage)',
        'label_zh': '可选 F — Fast-Planner 动力学（对照）',
        'class': 'optional',
        'status': 'optional',
        'family': 'ego_lineage',
        'launch': 'fast_planner_avoidance.launch.py',
        'package': 'plan_manage',
        'principle': 'Fast-Planner kino_replan (EGO/Fast-Planner lineage benchmark)',
        'desc_en': 'Optional lineage benchmark — not in canonical strong/weak matrix',
        'desc_zh': '可选算法对照，不计入标准强弱矩阵',
        'publish_rate_hz': 50.0,
        'local_goal_timeout': 0.5,
        'capabilities': ['trajectory_cmd', 'bridge', 'lineage_benchmark'],
    },
    'vfh': {
        'id': 'vfh',
        'aliases': ['g', 'path_g', 'rl', 'ppo', 'mappo'],
        'label_en': 'Path G — VFH+ histogram',
        'label_zh': '路径 G — VFH+ 直方图',
        'class': 'weak',
        'status': 'active',
        'family': 'reactive',
        'launch': 'rl_avoidance.launch.py',
        'package': 'drone_rl_planner',
        'principle': 'Classical Vector Field Histogram+ reactive avoidance',
        'desc_en': 'Polar-histogram avoider → smooth yellow path (PPO optional via backend:=rl)',
        'desc_zh': '极坐标直方图避障（可选 PPO 后端）',
        'publish_rate_hz': 20.0,
        'local_goal_timeout': 2.0,
        'capabilities': ['local_goal', 'trajectory_viz', 'reactive'],
    },
    'sac': {
        'id': 'sac',
        'aliases': ['h', 'path_h', 'drq_sac'],
        'label_en': 'Path H — Polar DrQ-SAC',
        'label_zh': '路径 H — Polar DrQ-SAC',
        'class': 'strong',
        'status': 'active',
        'family': 'learning',
        'launch': 'sac_avoidance.launch.py',
        'package': 'drone_rl_planner',
        'principle': 'Polar occupancy image + Soft Actor-Critic (DrQ) path generation',
        'desc_en': 'SAC policy → rolled path; adapter-level VFH safety supervisor',
        'desc_zh': 'SAC 策略出路径；适配层 VFH 安全监督',
        'publish_rate_hz': 20.0,
        'local_goal_timeout': 2.0,
        'capabilities': ['local_goal', 'trajectory_viz', 'learning', 'uses_supervisor'],
    },
}

MULTI_MODES: Dict[str, Dict[str, Any]] = {
    'ego_swarm': {
        'label_en': 'EGO-Swarm',
        'label_zh': 'EGO-Swarm',
        'desc_en': 'broadcast_bspline swarm (2–20 drones)',
        'desc_zh': 'broadcast_bspline 集群（2–20 机）',
        'launch': 'ego_swarm.launch.py',
        'class': 'multi',
    },
    'shared_field': {
        'label_en': 'Shared field',
        'label_zh': '共享空域避障',
        'desc_en': 'Homemade planners + dense_field + peer keep-out (2 drones)',
        'desc_zh': '自研规划 + 密集场景 + 机间禁入区（固定 2 机）',
        'launch': 'shared_field.launch.py',
        'class': 'multi',
    },
    'formation': {
        'label_en': 'Formation',
        'label_zh': '编队',
        'desc_en': 'Leader + 2 followers: line / column / V on dense_field',
        'desc_zh': '领机 + 2 从机：横排 / 纵列 / V 形（密集场）',
        'launch': 'formation.launch.py',
        'class': 'multi',
    },
}


def normalize_planner_id(raw: str) -> str:
    key = (raw or '').strip().lower()
    if key in PLANNERS:
        return key
    # Compatibility: old 'rl' key maps to vfh
    if key == 'rl':
        return 'vfh'
    for pid, meta in PLANNERS.items():
        if key in meta.get('aliases', []):
            return pid
    return key


def planner_public_info(lang: str = 'en') -> List[Dict[str, Any]]:
    out = []
    for pid, meta in PLANNERS.items():
        out.append({
            'id': pid,
            'label': meta.get(f'label_{lang}', meta.get('label_en', pid)),
            'desc': meta.get(f'desc_{lang}', meta.get('desc_en', '')),
            'class': meta['class'],
            'status': meta['status'],
            'family': meta.get('family', ''),
            'launch': meta['launch'],
            'principle': meta.get('principle', ''),
            'aliases': list(meta.get('aliases', [])),
            'capabilities': list(meta.get('capabilities', [])),
        })
    return out


def canonical_comparison_ids() -> List[str]:
    """Planners in the fair strong/weak matrix (excludes mode/optional/multi)."""
    return [
        pid for pid, m in PLANNERS.items()
        if m['status'] == 'active' and m['class'] in ('weak', 'strong')
    ]


def controller_timeout_for(planner_id: str) -> float:
    meta = PLANNERS.get(normalize_planner_id(planner_id), {})
    return float(meta.get('local_goal_timeout', 2.0))


def dashboard_planners_legacy() -> Dict[str, Dict[str, Any]]:
    """Shape expected by older dashboard frontends (id → label/launch/desc)."""
    out: Dict[str, Dict[str, Any]] = {}
    for pid, meta in PLANNERS.items():
        out[pid] = {
            'label': meta.get('label_en', pid),
            'launch': meta['launch'],
            'via': 'planner_sim',
            'desc': meta.get('desc_en', ''),
            'class': meta['class'],
            'status': meta['status'],
        }
    # Keep 'rl' alias key for old UI selections
    if 'vfh' in out:
        out['rl'] = dict(out['vfh'])
        out['rl']['label'] = out['vfh']['label'] + ' (alias rl)'
    return out

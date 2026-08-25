"""Shared helpers for drone_bringup launch files."""

import os
from typing import Any, Dict, Iterable, List, Optional, Tuple

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

from drone_bringup.maps_catalog import MAPS, normalize_map_id, pose_for_map


def workspace_root() -> str:
    """Best-effort path to drone_ws root (parent of src/)."""
    prefix = get_package_prefix('drone_bringup')
    # install prefix is .../drone_ws/install/drone_bringup
    return os.path.abspath(os.path.join(prefix, '..', '..'))


def script_path(name: str) -> str:
    """Path to repo-root scripts/ (e.g. scripts/evaluate.py)."""
    return os.path.join(workspace_root(), 'scripts', name)


def config_path(filename: str) -> str:
    pkg_share = get_package_share_directory('drone_bringup')
    return os.path.join(pkg_share, 'config', filename)


def libexec(name: str) -> str:
    """Path to installed console_script under lib/drone_bringup/."""
    return os.path.join(get_package_prefix('drone_bringup'), 'lib', 'drone_bringup', name)


def rviz_config_path(filename: str = 'drone.rviz') -> str:
    pkg_share = get_package_share_directory('drone_visualization')
    return os.path.join(pkg_share, 'rviz', filename)


def dynamics_node(
    extra_params: Optional[dict] = None,
    param_files: Optional[Iterable[str]] = None,
    namespace: str = '',
) -> Node:
    params: List = []
    for f in param_files or ['dynamics.yaml']:
        params.append(config_path(f))
    merged = {}
    if namespace:
        merged['namespace'] = namespace
    if extra_params:
        merged.update(extra_params)
    if merged:
        params.append(merged)
    return Node(
        package='drone_dynamics',
        executable='dynamics_node',
        name=('drone_dynamics_' + namespace) if namespace else 'drone_dynamics',
        output='screen',
        parameters=params,
    )


def controller_node(namespace: str = '', extra_params: Optional[dict] = None) -> Node:
    params: List = [config_path('controller.yaml')]
    merged = {}
    if namespace:
        merged['namespace'] = namespace
    if extra_params:
        merged.update(extra_params)
    if merged:
        params.append(merged)
    return Node(
        package='drone_controller',
        executable='controller_node',
        name=('drone_controller_' + namespace) if namespace else 'drone_controller',
        output='screen',
        parameters=params,
    )


def map_node(
    map_config: str,
    extra_params: Optional[dict] = None,
    namespace: str = '',
) -> Node:
    params: List = [config_path(map_config)]
    merged = {}
    if namespace:
        merged['namespace'] = namespace
    if extra_params:
        merged.update(extra_params)
    if merged:
        params.append(merged)
    return Node(
        package='drone_map',
        executable='map_node',
        name=('drone_map_' + namespace) if namespace else 'drone_map',
        output='screen',
        parameters=params,
    )


def map_adapter_node(input_topic: str, map_id: str, seed: int = 1, planner: str = '') -> Node:
    mid = normalize_map_id(map_id, planner=planner)
    meta = MAPS[mid]
    pose = pose_for_map(mid, planner=planner)
    return Node(
        package='drone_bringup',
        executable='map_adapter',
        name='map_adapter',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topics': [
                '/map/obstacles',
                '/map_generator/global_cloud',
            ],
            'frame_id': 'map',
            'map_id': mid,
            'seed': int(seed),
            'cruise_z': float(pose['init_z']),
            'z_band': 0.55,
            'grid_resolution': 0.25,
            'ensure_boundary': meta['family'] == 'official',
            'boundary_resolution': 0.15,
        }],
    )


def cloud_bridge_node(input_topic: str) -> Node:
    return Node(
        package='drone_bringup',
        executable='cloud_bridge',
        name='cloud_bridge',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topics': [
                '/map/obstacles',
                '/map_generator/global_cloud',
            ],
            'frame_id': 'map',
        }],
    )


def _random_forest_node(seed: int) -> Node:
    return Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[{
            'map/x_size': 26.0,
            'map/y_size': 20.0,
            'map/z_size': 3.0,
            'map/resolution': 0.1,
            'ObstacleShape/seed': int(seed) if seed else 1,
            'map/obs_num': 52,
            'ObstacleShape/lower_rad': 0.30,
            'ObstacleShape/upper_rad': 0.50,
            'ObstacleShape/lower_hei': 0.0,
            'ObstacleShape/upper_hei': 3.0,
            # Fewer suspended rings: they force vertical weaving that
            # destabilizes corner replans under load; trunks carry the
            # avoidance intent of this scenario.
            'map/circle_num': 12,
            'ObstacleShape/radius_l': 0.7,
            'ObstacleShape/radius_h': 0.5,
            'ObstacleShape/z_l': 0.7,
            'ObstacleShape/z_h': 0.8,
            'ObstacleShape/theta': 0.5,
            'pub_rate': 1.0,
            # Wider trunk spacing keeps inter-tree corridors passable with
            # the inflated-grid clearance the acceptance criteria measure.
            'min_distance': 1.6,
            'map/clear_y': 1.6,
        }],
    )


def _mockamap_node(mock_type: int, seed: int) -> Node:
    # Presets aligned with official mockamap/*.launch.py
    common = {
        'seed': int(seed) if seed else 511,
        'update_freq': 1.0,
        'resolution': 0.1,
        'type': int(mock_type),
    }
    if mock_type == 1:  # Perlin3D — box larger than start/goal (±22)
        params = {
            **common,
            'resolution': 0.15,
            'x_length': 50,
            'y_length': 26,
            'z_length': 5,
            'complexity': 0.05,
            'fill': 0.05,
            'fractal': 1,
            'attenuation': 0.1,
        }
    elif mock_type == 2:  # posts — thinned for flyable corridors
        params = {
            **common,
            'update_freq': 0.1,
            'x_length': 20,
            'y_length': 20,
            'z_length': 4,
            'width_min': 0.6,
            'width_max': 1.2,
            'obstacle_number': 12,
        }
    elif mock_type == 3:  # maze2D — roads wide enough for inflate/dist0
        params = {
            **common,
            'seed': int(seed) if seed else 510,
            'x_length': 20,
            'y_length': 20,
            'z_length': 2,
            # 1.5 m lanes leave margin after Path A inflate / EGO dist0.
            'road_width': 1.5,
            'add_wall_x': 0,
            'add_wall_y': 0,
            'maze_type': 1,
        }
    else:  # maze3D (type 4) — wider passages; Z remapped to [0,z] in maps.cpp
        params = {
            **common,
            'seed': int(seed) if seed else 1,
            'x_length': 20,
            'y_length': 20,
            'z_length': 4,
            'numNodes': 48,
            # Higher connectivity + larger road/node radii so Path C 3D A*
            # can thread (-6,0)→(6,0) without sealing plate holes.
            'connectivity': 0.7,
            'roadRad': 12,
            'nodeRad': 7,
        }
    return Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        parameters=[params],
    )


def map_stack(
    map_id: str,
    seed: int = 1,
    planner: str = '',
    map_extra: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Any], Dict[str, float]]:
    """Build obstacle map nodes + recommended init/goal pose for map_id."""
    mid = normalize_map_id(map_id, planner=planner)
    meta = MAPS[mid]
    pose = pose_for_map(mid, planner=planner)
    nodes: List[Any] = []

    backend = meta['backend']
    if backend == 'drone_map':
        map_params: Dict[str, Any] = {'seed': int(seed)}
        if map_extra:
            map_params.update(map_extra)
        nodes.append(map_node(
            meta['config'],
            extra_params=map_params,
        ))
        nodes.append(map_adapter_node('/map/obstacles', mid, seed=seed, planner=planner))
    elif backend == 'random_forest':
        nodes.append(_random_forest_node(int(seed)))
        nodes.append(map_adapter_node('/map_generator/global_cloud', mid, seed=seed, planner=planner))
    elif backend == 'mockamap':
        nodes.append(_mockamap_node(int(meta['mock_type']), int(seed)))
        nodes.append(map_adapter_node('/mock_map', mid, seed=seed, planner=planner))
    else:
        raise RuntimeError(f'Unsupported map backend: {backend}')

    return nodes, pose


def planner_node(
    extra_params: Optional[dict] = None,
    namespace: str = '',
    map_id: str = '',
) -> Node:
    params: List = [config_path('planner.yaml')]
    merged = {}
    if namespace:
        merged['namespace'] = namespace
    if map_id:
        # Per-map Path A hints (cruise/inflate/3D policy) — without this the
        # registry overrides never reach the node and launches fly defaults.
        from drone_bringup.maps_catalog import homemade_planner_overrides
        merged.update(homemade_planner_overrides(map_id))
    if extra_params:
        merged.update(extra_params)
    if merged:
        params.append(merged)
    return Node(
        package='drone_planner',
        executable='planner_node',
        name=('drone_planner_' + namespace) if namespace else 'drone_planner',
        output='screen',
        parameters=params,
    )


def visualization_node(namespace: str = '', extra_params: Optional[dict] = None) -> Node:
    params = [{'arm_length': 0.18}]
    if namespace:
        params[0]['namespace'] = namespace
    if extra_params:
        params[0].update(extra_params)
    return Node(
        package='drone_visualization',
        executable='viz_node',
        name=('drone_visualization_' + namespace) if namespace else 'drone_visualization',
        output='screen',
        parameters=params,
    )


def rviz_node(condition=None, config: str = 'drone.rviz') -> Node:
    kwargs = dict(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path(config)],
    )
    if condition is not None:
        kwargs['condition'] = condition
    return Node(**kwargs)


def send_goal_process(
    x: float,
    y: float,
    z: float,
    yaw: float = 0.0,
    delay_sec: float = 3.0,
    topic: str = '/drone/goal',
    repeats: int = 1,
) -> TimerAction:
    cmd = [
        libexec('send_goal'),
        '--x', str(x),
        '--y', str(y),
        '--z', str(z),
        '--yaw', str(yaw),
        '--topic', topic,
        '--repeats', str(max(1, repeats)),
    ]
    return TimerAction(
        period=delay_sec,
        actions=[ExecuteProcess(cmd=cmd, output='screen')],
    )


def waypoint_publisher_process(
    pattern: str = 'square',
    delay_sec: float = 5.0,
    extra_args: Optional[List[str]] = None,
    hold_sec: float = 8.0,
) -> TimerAction:
    cmd = [
        libexec('waypoint_publisher'),
        '--pattern', pattern,
        '--hold', str(hold_sec),
    ]
    if extra_args:
        cmd.extend(extra_args)
    return TimerAction(
        period=delay_sec,
        actions=[ExecuteProcess(cmd=cmd, output='screen')],
    )


def wp_list_arg(waypoints) -> str:
    """Serialize (x,y,z) tuples for waypoint_publisher --list=…."""
    return ';'.join(f'{float(x)},{float(y)},{float(z)}' for x, y, z in waypoints)


def square_mission_process(
    waypoints,
    delay_sec: float = 8.0,
    arrival_tol: float = 1.2,
    max_hold: float = 60.0,
) -> TimerAction:
    """Publish a sequential closed square (or any list) on /drone/goal."""
    return waypoint_publisher_process(
        pattern='list',
        delay_sec=delay_sec,
        hold_sec=1.0,
        extra_args=[
            f'--list={wp_list_arg(waypoints)}',
            '--cycles', '1',
            '--wait-arrival',
            '--arrival-tol', str(arrival_tol),
            '--max-hold', str(max_hold),
        ],
    )


def resolve_mission_pose(map_id: str, pose: dict, mission: str) -> dict:
    """Override spawn/goal when mission:=square; otherwise keep catalog pose."""
    if str(mission or 'catalog').strip().lower() != 'square':
        return dict(pose)
    from drone_bringup.maps_catalog import pose_for_benchmark_square
    out = dict(pose)
    out.update(pose_for_benchmark_square(map_id))
    return out


# Closed-square comparative benchmark: lower speed so tracking/clearance holds.
SQUARE_MISSION_MAX_VEL = 0.65
SQUARE_MISSION_MAX_ACC = 1.1
# Planners that still overshoot / fail square missions need a stricter plant cap.
SQUARE_MISSION_MAX_VEL_STRICT = 0.50
SQUARE_MISSION_MAX_ACC_STRICT = 0.85


def square_speed_params(mission: str, strict: bool = False) -> Dict[str, float]:
    """Controller speed caps used when mission:=square (empty for catalog)."""
    if str(mission or 'catalog').strip().lower() != 'square':
        return {}
    if strict:
        return {
            'max_vel': float(SQUARE_MISSION_MAX_VEL_STRICT),
            'max_acc': float(SQUARE_MISSION_MAX_ACC_STRICT),
        }
    return {
        'max_vel': float(SQUARE_MISSION_MAX_VEL),
        'max_acc': float(SQUARE_MISSION_MAX_ACC),
    }


def square_planner_speed_params(mission: str) -> Dict[str, float]:
    """Planner-side speed caps aligned with the square-mission controller."""
    if str(mission or 'catalog').strip().lower() != 'square':
        return {}
    v = float(SQUARE_MISSION_MAX_VEL)
    a = float(SQUARE_MISSION_MAX_ACC)
    return {
        'max_vel': v,
        'max_acc': a,
        'manager/max_vel': v,
        'manager/max_acc': a,
        'optimization/max_vel': v,
        'optimization/max_acc': a,
        'bspline/limit_vel': v,
        'bspline/limit_acc': a,
        'search/max_vel': v,
        'search/max_acc': a,
    }

def evaluate_process(
    delay_sec: float = 8.0,
    duration_sec: float = 60.0,
    output_dir: Optional[str] = None,
    goal: Optional[tuple] = None,
) -> TimerAction:
    out = output_dir or os.path.expanduser('~/drone_ws/scripts/output')
    os.makedirs(out, exist_ok=True)
    cmd = [
        libexec('evaluate_drone'),
        '--duration', str(duration_sec),
        '--output-dir', out,
    ]
    if goal is not None:
        cmd.extend(['--goal-x', str(goal[0]), '--goal-y', str(goal[1]), '--goal-z', str(goal[2])])
    return TimerAction(
        period=delay_sec,
        actions=[ExecuteProcess(cmd=cmd, output='screen')],
    )


def interference_monitor_process(
    delay_sec: float = 2.0,
    goal: Optional[tuple] = None,
    hover_limit_m: float = 0.3,
) -> TimerAction:
    """Floating Tk panel for scenario-6 wind/IMU status (screen-capture friendly)."""
    cmd = [libexec('interference_monitor')]
    g = goal or (0.0, 0.0, 1.5)
    # ROS2 Python nodes: pass params via --ros-args
    cmd.extend([
        '--ros-args',
        '-p', f'goal_x:={g[0]}',
        '-p', f'goal_y:={g[1]}',
        '-p', f'goal_z:={g[2]}',
        '-p', f'hover_limit_m:={hover_limit_m}',
    ])
    return TimerAction(
        period=delay_sec,
        actions=[ExecuteProcess(cmd=cmd, output='screen')],
    )


def rl_planner_root() -> Tuple[str, bool]:
    """Return (package root, is_ament_installed). Prefer src/ for live checkpoints."""
    src = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    try:
        share = get_package_share_directory('drone_rl_planner')
        # Prefer src when it has a newer / existing checkpoint
        src_ckpt = os.path.join(src, 'checkpoints', 'sb3_ppo_local.zip')
        share_ckpt = os.path.join(share, 'checkpoints', 'sb3_ppo_local.zip')
        if os.path.isfile(src_ckpt):
            if (not os.path.isfile(share_ckpt)
                    or os.path.getmtime(src_ckpt) >= os.path.getmtime(share_ckpt)):
                return src, False
        return share, True
    except Exception:
        return src, False


def resolve_rl_checkpoint(root: str) -> str:
    """Best checkpoint path for SB3 PPO (with or without .zip suffix)."""
    # Always also check workspace src/
    src_root = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    for base in (root, src_root):
        ckpt = os.path.join(base, 'checkpoints', 'sb3_ppo_local')
        for cand in (ckpt + '.zip', ckpt, os.path.join(base, 'checkpoints', 'ppo_local.npz')):
            if os.path.isfile(cand):
                return cand.replace('.zip', '') if cand.endswith('.zip') else cand
    return os.path.join(root, 'checkpoints', 'sb3_ppo_local')


def rl_planner_node(
    cruise_z: float,
    checkpoint: str = '',
    extra_params: Optional[Dict[str, Any]] = None,
) -> Node | ExecuteProcess:
    """Legacy PPO node (optional). Prefer vfh_planner_node for Path G."""
    root, installed = rl_planner_root()
    src_root = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    yaml_cfg = os.path.join(src_root if os.path.isdir(src_root) else root, 'config', 'rl_planner.yaml')
    ckpt = checkpoint or resolve_rl_checkpoint(root)
    params: Dict[str, Any] = {
        'checkpoint': ckpt if (os.path.isfile(ckpt) or os.path.isfile(ckpt + '.zip')) else '',
        'cruise_z': float(cruise_z),
        'map_topic': '/map/obstacles',
        'max_speed': 1.2,
        'world_scale': 40.0,
        'ray_max': 6.0,
        'lookahead_m': 1.4,
        'action_ema': 0.55,
        'cmd_speed_scale': 0.65,
        'pred_horizon_m': 4.5,
        'dir_rate_limit': 1.8,
        'goal_tol': 0.70,
        'control_hz': 20.0,
    }
    if extra_params:
        params.update(extra_params)
    if installed:
        return Node(
            package='drone_rl_planner',
            executable='rl_planner_node',
            name='rl_planner_node',
            output='screen',
            parameters=[yaml_cfg, params],
        )
    script = os.path.join(root, 'drone_rl_planner', 'rl_planner_node.py')
    return _python_node_process(script, root, yaml_cfg, params)


def vfh_planner_node(
    cruise_z: float,
    extra_params: Optional[Dict[str, Any]] = None,
) -> Node | ExecuteProcess:
    """Path G default: PX4-style VFH+ local avoider (classical, deterministic)."""
    root, installed = rl_planner_root()
    src_root = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    yaml_cfg = os.path.join(
        src_root if os.path.isdir(src_root) else root, 'config', 'vfh_planner.yaml')
    params: Dict[str, Any] = {
        'cruise_z': float(cruise_z),
        'map_topic': '/map/obstacles',
        'n_sectors': 72,
        'ray_max': 6.0,
        'lookahead_m': 1.6,
        'path_horizon_m': 7.0,
        'goal_tol': 0.85,
        'approach_m': 2.8,
        'hold_exit_m': 2.0,
        'control_hz': 20.0,
        'max_vel_hint': 0.85,
    }
    if extra_params:
        params.update(extra_params)
    # Always prefer src/ so Path G stop-at-goal fixes apply without a rebuild race.
    script = os.path.join(
        src_root if os.path.isdir(src_root) else root,
        'drone_rl_planner', 'vfh_planner_node.py')
    if os.path.isfile(script) and os.path.isdir(src_root):
        return _python_node_process(
            script, src_root,
            yaml_cfg if os.path.isfile(yaml_cfg) else None, params)
    if installed and os.path.isfile(
            os.path.join(get_package_prefix('drone_rl_planner'),
                         'lib', 'drone_rl_planner', 'vfh_planner_node')):
        return Node(
            package='drone_rl_planner',
            executable='vfh_planner_node',
            name='vfh_planner_node',
            output='screen',
            parameters=[yaml_cfg, params] if os.path.isfile(yaml_cfg) else [params],
        )
    return _python_node_process(
        script, src_root if os.path.isdir(src_root) else root,
        yaml_cfg if os.path.isfile(yaml_cfg) else None, params)


def resolve_sac_checkpoint(root: str) -> str:
    """Best Path H SAC checkpoint (.pt)."""
    src_root = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    for base in (root, src_root):
        for name in ('sac_polar_local_best.pt', 'sac_polar_local.pt'):
            cand = os.path.join(base, 'checkpoints', name)
            if os.path.isfile(cand):
                return cand
    return os.path.join(root, 'checkpoints', 'sac_polar_local.pt')


def sac_planner_node(
    cruise_z: float,
    checkpoint: str = '',
    extra_params: Optional[Dict[str, Any]] = None,
) -> Node | ExecuteProcess:
    """Path H: Polar DrQ-SAC → Bézier path with VFH safety fallback."""
    root, installed = rl_planner_root()
    src_root = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    yaml_cfg = os.path.join(
        src_root if os.path.isdir(src_root) else root, 'config', 'sac_planner.yaml')
    ckpt = checkpoint or resolve_sac_checkpoint(root)
    params: Dict[str, Any] = {
        'checkpoint': ckpt if os.path.isfile(ckpt) else '',
        'cruise_z': float(cruise_z),
        'map_topic': '/map/obstacles',
        'n_rings': 16,
        'n_sectors': 36,
        'ray_max': 6.0,
        'max_speed': 1.0,
        'goal_tol': 0.70,
        'control_hz': 20.0,
        'action_ema': 0.40,
        'fallback_clear_m': 0.40,
        'path_horizon_m': 8.0,
        'blend_clear_m': 1.4,
    }
    if extra_params:
        params.update(extra_params)
    script = os.path.join(
        src_root if os.path.isdir(src_root) else root,
        'drone_rl_planner', 'sac_planner_node.py')
    if installed and os.path.isfile(
            os.path.join(get_package_prefix('drone_rl_planner'),
                         'lib', 'drone_rl_planner', 'sac_planner_node')):
        return Node(
            package='drone_rl_planner',
            executable='sac_planner_node',
            name='sac_planner_node',
            output='screen',
            parameters=[yaml_cfg, params] if os.path.isfile(yaml_cfg) else [params],
        )
    return _python_node_process(
        script, src_root if os.path.isdir(src_root) else root,
        yaml_cfg if os.path.isfile(yaml_cfg) else None, params)


def safety_supervisor_node(
    cruise_z: float,
    extra_params: Optional[Dict[str, Any]] = None,
) -> Node | ExecuteProcess:
    """Adapter-level VFH safety supervisor for Path H."""
    root, installed = rl_planner_root()
    src_root = os.path.join(workspace_root(), 'src', 'drone_rl_planner')
    params: Dict[str, Any] = {
        'cruise_z': float(cruise_z),
        'map_topic': '/map/obstacles',
        'fallback_clear_m': 0.40,
        'blend_clear_m': 1.4,
        'path_horizon_m': 8.0,
        'enable_fallback': True,
        'planner_id': 'sac',
        'control_hz': 20.0,
    }
    if extra_params:
        params.update(extra_params)
    script = os.path.join(
        src_root if os.path.isdir(src_root) else root,
        'drone_rl_planner', 'safety_supervisor_node.py')
    if installed and os.path.isfile(
            os.path.join(get_package_prefix('drone_rl_planner'),
                         'lib', 'drone_rl_planner', 'safety_supervisor_node')):
        return Node(
            package='drone_rl_planner',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[params],
        )
    return _python_node_process(
        script, src_root if os.path.isdir(src_root) else root, None, params)


def _python_node_process(
    script: str,
    pkg_root: str,
    yaml_cfg: Optional[str],
    params: Dict[str, Any],
) -> ExecuteProcess:
    cmd = ['python3', script, '--ros-args']
    if yaml_cfg and os.path.isfile(yaml_cfg):
        cmd.extend(['--params-file', yaml_cfg])
    for key, val in params.items():
        if isinstance(val, bool):
            cmd.extend(['-p', f'{key}:={"true" if val else "false"}'])
        elif isinstance(val, (int, float)):
            cmd.extend(['-p', f'{key}:={val}'])
        else:
            cmd.extend(['-p', f'{key}:={val}'])
    py_path = os.pathsep.join(
        p for p in (pkg_root, os.environ.get('PYTHONPATH', '')) if p)
    return ExecuteProcess(
        cmd=cmd,
        additional_env={'PYTHONPATH': py_path},
        output='screen',
    )


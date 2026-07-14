"""Shared helpers for drone_bringup launch files."""

import os
from typing import Any, Dict, Iterable, List, Optional, Tuple

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

from drone_bringup.maps_catalog import MAPS, normalize_map_id


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
            'map/obs_num': 60,
            'ObstacleShape/lower_rad': 0.30,
            'ObstacleShape/upper_rad': 0.50,
            'ObstacleShape/lower_hei': 0.0,
            'ObstacleShape/upper_hei': 3.0,
            'map/circle_num': 25,
            'ObstacleShape/radius_l': 0.7,
            'ObstacleShape/radius_h': 0.5,
            'ObstacleShape/z_l': 0.7,
            'ObstacleShape/z_h': 0.8,
            'ObstacleShape/theta': 0.5,
            'pub_rate': 1.0,
            'min_distance': 1.0,
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
            'road_width': 1.2,
            'add_wall_x': 0,
            'add_wall_y': 0,
            'maze_type': 1,
        }
    else:  # maze3D (type 4) — wider passages; Z remapped to [0,z] in maps.cpp
        params = {
            **common,
            'seed': int(seed) if seed else 510,
            'x_length': 20,
            'y_length': 20,
            'z_length': 4,
            'numNodes': 64,
            'connectivity': 0.5,
            'roadRad': 8,
            'nodeRad': 5,
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
) -> Tuple[List[Any], Dict[str, float]]:
    """Build obstacle map nodes + recommended init/goal pose for map_id."""
    mid = normalize_map_id(map_id, planner=planner)
    meta = MAPS[mid]
    pose = dict(meta['pose'])
    nodes: List[Any] = []

    backend = meta['backend']
    if backend == 'drone_map':
        nodes.append(map_node(
            meta['config'],
            extra_params={'seed': int(seed)},
        ))
        nodes.append(cloud_bridge_node('/map/obstacles'))
    elif backend == 'random_forest':
        nodes.append(_random_forest_node(int(seed)))
        nodes.append(cloud_bridge_node('/map_generator/global_cloud'))
    elif backend == 'mockamap':
        nodes.append(_mockamap_node(int(meta['mock_type']), int(seed)))
        nodes.append(cloud_bridge_node('/mock_map'))
    else:
        raise RuntimeError(f'Unsupported map backend: {backend}')

    return nodes, pose


def planner_node(
    extra_params: Optional[dict] = None,
    namespace: str = '',
) -> Node:
    params: List = [config_path('planner.yaml')]
    merged = {}
    if namespace:
        merged['namespace'] = namespace
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


def visualization_node(namespace: str = '') -> Node:
    params = [{'arm_length': 0.18}]
    if namespace:
        params[0]['namespace'] = namespace
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
) -> TimerAction:
    cmd = [
        libexec('waypoint_publisher'),
        '--pattern', pattern,
        '--hold', '8.0',
    ]
    if extra_args:
        cmd.extend(extra_args)
    return TimerAction(
        period=delay_sec,
        actions=[ExecuteProcess(cmd=cmd, output='screen')],
    )


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

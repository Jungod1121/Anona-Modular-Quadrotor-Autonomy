"""Path E: upstream MIGHTY (mit-acl) + plant via mighty_cmd_bridge."""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_bringup.maps_catalog import benchmark_square_waypoints
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    resolve_mission_pose,
    rviz_node,
    send_goal_process,
    square_mission_process,
    square_speed_params,
    visualization_node,
)


def _load_mighty_params(yaml_path: str) -> dict:
    """Flatten mighty.yaml like onboard_mighty.launch.py.

    Params-file keys under ``mighty_node:`` do NOT apply to ``/NX01/mighty_node``;
    passing a flat dict does.
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    return dict(raw['mighty_node']['ros__parameters'])


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)
    mission = LaunchConfiguration('mission').perform(context).strip().lower()

    map_nodes, pose = map_stack(map_id, seed=seed, planner='mighty')
    pose = resolve_mission_pose(map_id, pose, mission)
    goal_x, goal_y, goal_z = pose['goal_x'], pose['goal_y'], pose['goal_z']

    mighty_yaml = os.path.join(
        get_package_share_directory('mighty'), 'config', 'mighty.yaml')
    mighty_params = _load_mighty_params(mighty_yaml)

    # Plant-facing overrides (after yaml so they win).
    cruise_z = max(float(goal_z), 1.2)
    speed = square_speed_params(mission)
    mighty_params.update({
        'sim_env': 'fake_sim',
        'use_hardware': False,
        'force_goal_z': True,
        'default_goal_z': cruise_z,
        'mass': 1.0,
        'v_max': speed.get('max_vel', 1.5),
        'a_max': speed.get('max_acc', 3.0),
        'drone_bbox': [0.4, 0.4, 0.3],
        'share_traj': False,
        'visual_level': 1,
        # Explicit floors — declare defaults are fopt=0.1 / iters=30 when YAML missed.
        'fopt_threshold': float(mighty_params.get('fopt_threshold', 1.0e5)),
        'max_iterations': int(mighty_params.get('max_iterations', 1000)),
        # Plant hover ~1.0; yaml z_min=1.0 rejects z≈0.999 as "out of the map".
        'z_min': 0.0,
        'z_max': 5.0,
        # Local window a bit larger for forest corridors.
        'local_box_size': [5.0, 5.0, 3.0],
        'min_wdx': 20.0,
        'min_wdy': 20.0,
        'min_wdz': 3.0,
    })

    ctrl_extra = {
        'trajectory_cmd_timeout': 0.50,
        'local_goal_timeout': 1.5,
        'max_vel': 1.2,
        'max_acc': 1.8,
        'max_tilt': 0.40,
        'use_drone_goal_fallback': False,
    }
    ctrl_extra.update(speed)

    actions = list(map_nodes)
    actions.extend([
        dynamics_node(
            extra_params={
                'init_x': pose['init_x'],
                'init_y': pose['init_y'],
                'init_z': cruise_z,
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(extra_params=ctrl_extra),
        Node(
            package='mighty',
            executable='convert_odom_to_state',
            name='convert_odom_to_state',
            namespace='NX01',
            output='screen',
            remappings=[
                ('odom', '/drone/odom'),
                ('state', 'state'),
            ],
        ),
        Node(
            package='mighty',
            executable='mighty',
            name='mighty_node',
            namespace='NX01',
            output='screen',
            parameters=[mighty_params],
            remappings=[
                ('sensor_point_cloud', '/map_generator/global_cloud'),
                ('term_goal', '/drone/goal'),
            ],
        ),
        Node(
            package='drone_bringup',
            executable='mighty_cmd_bridge',
            name='mighty_cmd_bridge',
            output='screen',
            parameters=[{
                'goal_cmd_topic': '/NX01/goal',
                'trajectory_topic': '/NX01/trajectory',
                'cruise_height': cruise_z,
            }],
        ),
        visualization_node(),
    ])
    if mission == 'square':
        actions.append(square_mission_process(
            benchmark_square_waypoints(map_id), delay_sec=10.0))
    else:
        actions.append(send_goal_process(
            goal_x, goal_y, cruise_z, yaw=0.0, delay_sec=12.0,
            topic='/drone/goal', repeats=4))
    actions.append(
        rviz_node(condition=IfCondition(use_rviz), config='ego_avoidance.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('map', default_value='official_forest'),
        DeclareLaunchArgument(
            'mission', default_value='catalog',
            description='catalog = single catalog goal; square = map-specific square'),
        OpaqueFunction(function=launch_setup),
    ])

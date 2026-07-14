"""Path B multi: official EGO-Swarm (broadcast_bspline) + our plant (no SO3).

Spawns N namespaced drones (uav0..uav{N-1}), each with:
  dynamics + controller + ego_planner + traj_server + ego_cmd_bridge

Inter-drone avoidance uses official /broadcast_bspline (EGO-Swarm core).
Shared obstacle cloud via map_stack.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    rviz_node,
    visualization_node,
)


def _crossing_pose(drone_id: int, num: int, map_family: str):
    """EGO-Swarm classic: start west, goal east (or mirrored lanes)."""
    # Spread in y; opposite half mirrors goal sign for crossing traffic.
    half = max(num - 1, 1)
    y = -4.0 + 8.0 * drone_id / half
    z = 1.0
    if map_family == 'homemade':
        # dense_field frame: ~x in [1,17]
        return {
            'init': (1.0, y + 5.0, 1.5),
            'goal': (17.0, 5.0 - y, 1.5),
        }
    # official forest centered at origin
    return {
        'init': (-15.0, y, z),
        'goal': (15.0, -y, z),
    }


def _ego_params(drone_id: int, goal, swarm_clearance: float, map_id: str = 'official_forest') -> dict:
    from drone_bringup.maps_catalog import ego_planner_overrides
    params = {
        'fsm/flight_type': 2,  # PRESET_TARGET — per-drone waypoints
        'fsm/thresh_replan_time': 3.0,
        'fsm/thresh_no_replan_meter': 2.0,
        'fsm/planning_horizon': 7.5,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,
        'fsm/cruise_height': 1.0,
        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': float(goal[0]),
        'fsm/waypoint0_y': float(goal[1]),
        'fsm/waypoint0_z': float(goal[2]),
        'grid_map/resolution': 0.15,
        'grid_map/map_size_x': 50.0,
        'grid_map/map_size_y': 30.0,
        'grid_map/map_size_z': 4.0,
        'grid_map/local_update_range_x': 20.0,
        'grid_map/local_update_range_y': 14.0,
        'grid_map/local_update_range_z': 3.0,
        'grid_map/obstacles_inflation': 0.12,
        'grid_map/local_map_margin': 10,
        'grid_map/ground_height': -0.01,
        'grid_map/cx': 321.0,
        'grid_map/cy': 243.0,
        'grid_map/fx': 387.0,
        'grid_map/fy': 387.0,
        'grid_map/use_depth_filter': False,
        'grid_map/depth_filter_tolerance': 0.15,
        'grid_map/depth_filter_maxdist': 5.0,
        'grid_map/depth_filter_mindist': 0.2,
        'grid_map/depth_filter_margin': 2,
        'grid_map/k_depth_scaling_factor': 1000.0,
        'grid_map/skip_pixel': 2,
        'grid_map/p_hit': 0.65,
        'grid_map/p_miss': 0.35,
        'grid_map/p_min': 0.12,
        'grid_map/p_max': 0.90,
        'grid_map/p_occ': 0.80,
        'grid_map/min_ray_length': 0.1,
        'grid_map/max_ray_length': 20.0,
        'grid_map/virtual_ceil_height': 2.8,
        'grid_map/visualization_truncate_height': 2.5,
        'grid_map/show_occ_time': False,
        'grid_map/pose_type': 2,
        'grid_map/frame_id': 'map',
        'manager/max_vel': 1.5,
        'manager/max_acc': 2.0,
        'manager/max_jerk': 4.0,
        'manager/control_points_distance': 0.4,
        'manager/feasibility_tolerance': 0.05,
        'manager/planning_horizon': 7.5,
        'manager/use_distinctive_trajs': True,
        'manager/drone_id': int(drone_id),
        'optimization/lambda_smooth': 1.0,
        'optimization/lambda_collision': 0.5,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.5,
        'optimization/swarm_clearance': float(swarm_clearance),
        'optimization/max_vel': 1.5,
        'optimization/max_acc': 2.0,
        'bspline/limit_vel': 1.5,
        'bspline/limit_acc': 2.0,
        'bspline/limit_ratio': 1.1,
        'prediction/obj_num': 0,
        'prediction/lambda': 1.0,
        'prediction/predict_rate': 1.0,
    }
    params.update(ego_planner_overrides(map_id))
    return params


def _one_drone(drone_id: int, ns: str, init, goal, swarm_clearance: float, map_id: str = 'official_forest'):
    odom = f'/{ns}/drone/odom'
    actions = [
        dynamics_node(
            namespace=ns,
            extra_params={
                'init_x': init[0],
                'init_y': init[1],
                'init_z': init[2],
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(namespace=ns, extra_params={
            'trajectory_cmd_timeout': 0.40,
            'local_goal_timeout': 1.5,
            'max_vel': 1.2,
            'max_acc': 1.8,
            'max_tilt': 0.40,
        }),
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name=f'drone_{drone_id}_ego_planner_node',
            output='screen',
            remappings=[
                ('odom_world', odom),
                ('planning/bspline', f'/drone_{drone_id}_planning/bspline'),
                ('planning/data_display', f'/drone_{drone_id}_planning/data_display'),
                ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
                ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
                ('goal', f'/{ns}/drone/goal'),
                ('goal_point', f'/drone_{drone_id}_plan_vis/goal_point'),
                ('global_list', f'/drone_{drone_id}_plan_vis/global_list'),
                ('init_list', f'/drone_{drone_id}_plan_vis/init_list'),
                ('optimal_list', f'/drone_{drone_id}_plan_vis/optimal_list'),
                ('a_star_list', f'/drone_{drone_id}_plan_vis/a_star_list'),
                ('grid_map/odom', odom),
                ('grid_map/cloud', '/map_generator/global_cloud'),
                ('grid_map/occupancy_inflate',
                 f'/drone_{drone_id}_grid/grid_map/occupancy_inflate'),
            ],
            parameters=[_ego_params(drone_id, goal, swarm_clearance, map_id)],
        ),
        Node(
            package='ego_planner',
            executable='traj_server',
            name=f'drone_{drone_id}_traj_server',
            output='screen',
            remappings=[
                ('planning/bspline', f'/drone_{drone_id}_planning/bspline'),
                ('/position_cmd', f'/drone_{drone_id}_planning/pos_cmd'),
            ],
            parameters=[{'traj_server/time_forward': 1.0}],
        ),
        Node(
            package='drone_bringup',
            executable='ego_cmd_bridge',
            name=f'ego_cmd_bridge_{ns}',
            output='screen',
            parameters=[{
                'namespace': ns,
                'drone_id': drone_id,
                # PRESET waypoints — no auto interactive goal.
                'auto_goal_enable': False,
                'cruise_height': 1.0,
            }],
        ),
        visualization_node(namespace=ns),
    ]
    return actions


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)
    num = int(LaunchConfiguration('num_drones').perform(context))
    clearance = float(LaunchConfiguration('swarm_clearance').perform(context))
    num = max(2, min(num, 3))  # keep 2–3 for plant CPU

    map_nodes, pose_meta = map_stack(map_id, seed=seed, planner='ego')
    from drone_bringup.maps_catalog import MAPS, normalize_map_id
    mid = normalize_map_id(map_id, planner='ego')
    family = MAPS[mid]['family']

    actions = list(map_nodes)
    for i in range(num):
        ns = f'uav{i}'
        poses = _crossing_pose(i, num, family)
        actions.extend(_one_drone(
            i, ns, poses['init'], poses['goal'], clearance, mid))

    # Multi-drone RViz: forest cloud + per-uav markers (reuse ego config + note).
    actions.append(
        rviz_node(condition=IfCondition(use_rviz), config='ego_swarm.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('map', default_value='official_forest'),
        DeclareLaunchArgument('num_drones', default_value='2'),
        DeclareLaunchArgument('swarm_clearance', default_value='0.7'),
        OpaqueFunction(function=launch_setup),
    ])

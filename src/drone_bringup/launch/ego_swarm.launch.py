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


def _crossing_pose(drone_id: int, num: int, map_id: str, map_family: str):
    """EGO-Swarm classic: start west, goal east (crossing lanes).

    Lanes are clamped per map so edge UAVs are not born inside / past the
    obstacle volume (Perlin3D especially: cloud ≈ ±25×±13; forest layout
    put outer drones at y=±11 with x=-18 *inside* the Perlin box).
    """
    half = max(num - 1, 1)
    # Default corridor (random forest).
    x0, x1 = -18.0, 18.0
    y_cap = 9.0 if num <= 5 else (11.0 if num <= 10 else min(14.0, 1.15 * half))
    z_base = 1.0

    if map_family == 'homemade':
        # Expanded dense_field ≈ x∈[-8,48], y∈[-8,32], no border walls.
        y_span = min(6.0, max(1.5, 1.2 * half))
        y = -y_span + (2.0 * y_span) * drone_id / half
        return {
            'init': (2.0, 12.0 + y, 1.5),
            'goal': (40.0, 12.0 - y, 1.5),
        }

    if map_id == 'official_perlin':
        # Start/goal clear of the Perlin box; keep Y with inflate margin.
        x0, x1 = -27.0, 27.0
        y_cap = min(y_cap, 7.0 if num <= 6 else 8.5)
        z_base = 1.2
    elif map_id == 'official_posts':
        x0, x1 = -12.0, 12.0
        y_cap = min(y_cap, 6.5)
    elif map_id in ('official_maze2d', 'official_maze3d'):
        x0, x1 = -10.5, 10.5
        y_cap = min(y_cap, 6.0)

    y = -y_cap + (2.0 * y_cap) * drone_id / half
    z = z_base + (0.15 * (drone_id % 3) if num >= 6 else 0.0)
    return {
        'init': (x0, y, z),
        'goal': (x1, -y, z),
    }


def _ego_params(drone_id: int, goal, swarm_clearance: float, map_id: str = 'official_forest',
                num_drones: int = 2) -> dict:
    from drone_bringup.maps_catalog import ego_planner_overrides
    # Large swarms: slower + shorter horizon keeps plans feasible on one PC.
    vel = 1.5 if num_drones <= 5 else (1.15 if num_drones <= 10 else 0.95)
    horizon = 7.5 if num_drones <= 5 else 6.0
    params = {
        # flight_type 1 = goal topic (safe under multi-node). Type 2 PRESET used
        # nested spin_some and SIGSEGV'd ego_planner when N is large.
        'fsm/flight_type': 1,
        # Parallel takeoff: do not wait for MultiBsplines chain (broke after ~2 UAVs).
        'fsm/skip_sequential_start': True,
        'fsm/thresh_replan_time': 3.0,
        'fsm/thresh_no_replan_meter': 2.0,
        'fsm/planning_horizon': horizon,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,
        'fsm/cruise_height': float(goal[2]),
        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': float(goal[0]),
        'fsm/waypoint0_y': float(goal[1]),
        'fsm/waypoint0_z': float(goal[2]),
        'grid_map/resolution': 0.15,
        'grid_map/map_size_x': 50.0,
        'grid_map/map_size_y': 30.0,
        'grid_map/map_size_z': 4.0,
        'grid_map/local_update_range_x': 5.5,
        'grid_map/local_update_range_y': 5.5,
        'grid_map/local_update_range_z': 4.5,
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
        'grid_map/max_ray_length': 4.5,
        'grid_map/virtual_ceil_height': 2.8,
        'grid_map/visualization_truncate_height': 1.8,
        'grid_map/show_occ_time': False,
        'grid_map/pose_type': 2,
        'grid_map/frame_id': 'map',
        'manager/max_vel': vel,
        'manager/max_acc': min(2.0, vel + 0.5),
        'manager/max_jerk': 4.0,
        'manager/control_points_distance': 0.4,
        'manager/feasibility_tolerance': 0.05,
        'manager/planning_horizon': horizon,
        'manager/use_distinctive_trajs': True,
        'manager/drone_id': int(drone_id),
        'optimization/lambda_smooth': 1.0,
        'optimization/lambda_collision': 0.5,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.5,
        'optimization/swarm_clearance': float(swarm_clearance),
        'optimization/max_vel': vel,
        'optimization/max_acc': min(2.0, vel + 0.5),
        'bspline/limit_vel': vel,
        'bspline/limit_acc': min(2.0, vel + 0.5),
        'bspline/limit_ratio': 1.1,
        'prediction/obj_num': 0,
        'prediction/lambda': 1.0,
        'prediction/predict_rate': 1.0,
    }
    params.update(ego_planner_overrides(map_id))
    return params


def _one_drone(drone_id: int, ns: str, init, goal, swarm_clearance: float,
               map_id: str = 'official_forest', num_drones: int = 2):
    odom = f'/{ns}/drone/odom'
    # Simultaneous for N≤5; mild stagger for N≥6 cuts planning CPU spikes.
    goal_delay = 8.0 + (0.25 * drone_id if num_drones >= 6 else 0.0)
    ctrl_vel = 1.2 if num_drones <= 5 else (0.95 if num_drones <= 10 else 0.8)
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
            'max_vel': ctrl_vel,
            'max_acc': min(1.8, ctrl_vel + 0.6),
            'max_tilt': 0.40,
            # Never ballistic-fly to the EGO goal — wait for trajectory_cmd.
            'use_drone_goal_fallback': False,
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
                # Keep EGO goals off /drone/goal so the plant cannot chase the
                # final pose before SEQUENTIAL_START finishes (looked like
                # "no obstacle avoidance").
                ('goal', f'/{ns}/ego/goal'),
                ('goal_point', f'/drone_{drone_id}_plan_vis/goal_point'),
                ('global_list', f'/drone_{drone_id}_plan_vis/global_list'),
                ('init_list', f'/drone_{drone_id}_plan_vis/init_list'),
                ('optimal_list', f'/drone_{drone_id}_plan_vis/optimal_list'),
                ('a_star_list', f'/drone_{drone_id}_plan_vis/a_star_list'),
                ('grid_map/odom', odom),
                ('grid_map/cloud', f'/drone_{drone_id}_pcl_render_node/cloud'),
                ('grid_map/occupancy',
                 f'/drone_{drone_id}_grid/grid_map/occupancy'),
                ('grid_map/occupancy_inflate',
                 f'/drone_{drone_id}_grid/grid_map/occupancy_inflate'),
            ],
            parameters=[_ego_params(
                drone_id, goal, swarm_clearance, map_id, num_drones)],
        ),
        Node(
            package='drone_bringup',
            executable='local_sense_cloud',
            name=f'drone_{drone_id}_local_sense_cloud',
            output='screen',
            parameters=[{
                'global_cloud_topic': '/map_generator/global_cloud',
                'odom_topic': odom,
                'local_cloud_topic': f'/drone_{drone_id}_pcl_render_node/cloud',
                'sensing_horizon': 5.0,
                'sensing_rate': 10.0,
                'frame_id': 'map',
            }],
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
                # Interactive goals arrive on the standard contract topic
                # (/uavN/drone/goal — what the dashboard and docs use) and are
                # forwarded to this agent's EGO goal topic.
                'goal_in_topic': f'/{ns}/drone/goal',
                'goal_out_topic': f'/{ns}/ego/goal',
                'auto_goal_enable': True,
                'auto_goal_x': float(goal[0]),
                'auto_goal_y': float(goal[1]),
                'auto_goal_z': float(goal[2]),
                'auto_goal_delay': goal_delay,
                'auto_goal_repeats': 2,
                'auto_goal_period': 0.5,
                'publish_move_base_simple': False,
                'cruise_height': float(goal[2]),
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
    num = max(2, min(num, 20))  # soft cap; large N is heavy on one PC
    if num >= 6 and clearance < 0.85:
        clearance = 0.85

    map_nodes, pose_meta = map_stack(map_id, seed=seed, planner='ego')
    from drone_bringup.maps_catalog import MAPS, normalize_map_id
    mid = normalize_map_id(map_id, planner='ego')
    family = MAPS[mid]['family']

    actions = list(map_nodes)
    for i in range(num):
        ns = f'uav{i}'
        poses = _crossing_pose(i, num, mid, family)
        actions.extend(_one_drone(
            i, ns, poses['init'], poses['goal'], clearance, mid, num))

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

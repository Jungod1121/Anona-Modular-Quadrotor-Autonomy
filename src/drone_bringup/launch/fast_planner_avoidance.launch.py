"""Path F: upstream Fast-Planner (kino_replan) + plant via ego_cmd_bridge."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_bringup.maps_catalog import (
    benchmark_square_waypoints,
    gcopter_planner_overrides,
)
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
    workspace_root,
)


def _nlopt_lib_dir() -> str:
    """nlopt built out-of-tree under <ws>/third_party; empty when absent."""
    path = os.path.join(workspace_root(), 'third_party', 'nlopt_install', 'lib')
    return path if os.path.isdir(path) else ''


def _narrow_s_bend_waypoints(
    init_x: float,
    init_y: float,
    goal_x: float,
    goal_y: float,
    cruise_z: float,
    n_gates: int = 5,
    gap: float = 2.0,
) -> list[tuple[float, float, float]]:
    """Door centers + final goal — must match MapGenerator::narrowPassageWaypoints."""
    offset = max(2.0, gap + 0.7)
    y_mid = 0.5 * (init_y + goal_y)
    span = max(1.0, goal_x - init_x)
    wps: list[tuple[float, float, float]] = []
    for i in range(n_gates):
        t = (i + 1.0) / (n_gates + 1.0)
        x = init_x + t * span
        sign = 1.0 if (i % 2 == 0) else -1.0
        wps.append((x, y_mid + sign * offset, cruise_z))
    wps.append((goal_x, goal_y, cruise_z))
    return wps


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)
    mission = LaunchConfiguration('mission').perform(context).strip().lower()

    map_nodes, pose = map_stack(map_id, seed=seed, planner='fast_planner')
    pose = resolve_mission_pose(map_id, pose, mission)
    init_x, init_y, init_z = pose['init_x'], pose['init_y'], pose['init_z']
    goal_x, goal_y, goal_z = pose['goal_x'], pose['goal_y'], pose['goal_z']
    square_wps = (
        benchmark_square_waypoints(map_id) if mission == 'square' else [])
    first_goal = square_wps[0] if square_wps else (goal_x, goal_y, goal_z)

    map_size_x, map_size_y, map_size_z = 50.0, 30.0, 4.0
    gb = gcopter_planner_overrides(map_id)
    # MapBound is absolute corners [xmin, xmax, ymin, ymax, zmin, zmax].
    # SDFMap centers the grid at the origin (origin = -size/2), so we must
    # convert extents → a size whose centered AABB still covers MapBound.
    # Using (xmax-xmin) alone truncates asymmetric maps (e.g. narrow goal x=17).
    if 'MapBound' in gb and len(gb['MapBound']) >= 6:
        b = [float(v) for v in gb['MapBound']]
        sx = 2.0 * max(abs(b[0]), abs(b[1]))
        sy = 2.0 * max(abs(b[2]), abs(b[3]))
        sz = max(abs(b[5] - b[4]), 3.0)
        # Homemade corridor is small (~20×10); keep the SDF tight. Forest still
        # uses the larger default so official maps are not clipped.
        if map_id in ('narrow_corridor', 'tier_medium_corridor', 'narrow',
                      'dense_field', 'ego_maze2d_port', 'ego_forest_port'):
            map_size_x = max(sx, 24.0)
            map_size_y = max(sy, 14.0)
            map_size_z = max(sz, 4.0)
        else:
            map_size_x = max(map_size_x, sx)
            map_size_y = max(map_size_y, sy)
            map_size_z = max(map_size_z, sz)

    cruise_z = max(float(first_goal[2]), float(goal_z), 1.0)
    # Match plant tracking bandwidth — planner 2 m/s finishes horizon before our PID arrives.
    max_vel, max_acc = 1.2, 1.8
    local_rx, local_ry, local_rz = 12.0, 12.0, 3.5
    inflation = 0.12
    dist0 = 0.4
    search_horizon = 7.0
    # Keep cruise under this slab so tall walls cannot be hopped (doors only).
    virtual_ceil = 2.8
    flight_type = 1
    waypoint_num = 1
    waypoints = [(float(first_goal[0]), float(first_goal[1]), cruise_z)]
    goal_repeats = 4
    # Narrow S-bend: doors are offset from the start→goal line. A single far
    # goal makes kinodynamic A* aim straight through walls → EXEC↔REPLAN forever.
    # Chain preset door centers (flight_type=2) so each hop threads one gate.
    if mission != 'square' and map_id in (
            'narrow_corridor', 'tier_medium_corridor', 'narrow'):
        max_vel, max_acc = 0.65, 1.0
        local_rx, local_ry, local_rz = 10.0, 8.0, 3.5
        inflation = 0.06
        dist0 = 0.18
        search_horizon = 6.0
        virtual_ceil = 3.0
        cruise_z = min(cruise_z, 1.5)
        flight_type = 2
        waypoints = _narrow_s_bend_waypoints(
            float(init_x), float(init_y), float(goal_x), float(goal_y),
            cruise_z, n_gates=5, gap=1.6)
        waypoint_num = len(waypoints)
        goal_repeats = 1

    speed = square_speed_params(mission, strict=True)
    if speed:
        max_vel = float(speed['max_vel'])
        max_acc = float(speed['max_acc'])
        # Square: soft inflation so spawn/goals are not occupied (seed-1 forest
        # previously: "open set empty, no path!"). Keep flight_type=1 + sequential
        # /drone/goal — flight_type=2 + live goal republish caused EXEC↔REPLAN
        # collisions and off-map runaways.
        inflation = 0.04
        dist0 = 0.18
        search_horizon = 9.0
        local_rx, local_ry, local_rz = 16.0, 16.0, 3.5
        flight_type = 1
        waypoints = [(float(first_goal[0]), float(first_goal[1]), cruise_z)]
        waypoint_num = 1
        fp_params_square_extra = {
            'search/margin': 0.08,
            'manager/clearance_threshold': 0.10,
            'optimization/lambda2': 10.0,
            'manager/control_points_distance': 0.35,
            'fsm/thresh_replan': 0.8,
            'fsm/thresh_no_replan': 1.5,
        }
    else:
        fp_params_square_extra = {}

    fp_params = {
        'planner_node/planner': 1,
        'fsm/flight_type': flight_type,
        'fsm/thresh_replan': 1.0,
        'fsm/thresh_no_replan': 1.2,
        'fsm/waypoint_num': waypoint_num,
    }
    for i, (wx, wy, wz) in enumerate(waypoints[:7]):
        fp_params[f'fsm/waypoint{i}_x'] = float(wx)
        fp_params[f'fsm/waypoint{i}_y'] = float(wy)
        fp_params[f'fsm/waypoint{i}_z'] = float(wz)

    fp_params.update({
        'sdf_map/resolution': 0.15,
        'sdf_map/map_size_x': map_size_x,
        'sdf_map/map_size_y': map_size_y,
        'sdf_map/map_size_z': map_size_z,
        'sdf_map/local_update_range_x': local_rx,
        'sdf_map/local_update_range_y': local_ry,
        'sdf_map/local_update_range_z': local_rz,
        'sdf_map/obstacles_inflation': inflation,
        'sdf_map/local_bound_inflate': 0.0,
        'sdf_map/local_map_margin': 50,
        'sdf_map/ground_height': -0.01,
        'sdf_map/cx': 321.0,
        'sdf_map/cy': 243.0,
        'sdf_map/fx': 387.0,
        'sdf_map/fy': 387.0,
        'sdf_map/use_depth_filter': False,
        'sdf_map/skip_pixel': 2,
        'sdf_map/depth_filter_margin': 2,
        'sdf_map/k_depth_scaling_factor': 1000.0,
        'sdf_map/p_hit': 0.65,
        'sdf_map/p_miss': 0.35,
        'sdf_map/p_min': 0.12,
        'sdf_map/p_max': 0.90,
        'sdf_map/p_occ': 0.80,
        'sdf_map/min_ray_length': 0.1,
        'sdf_map/max_ray_length': 20.0,
        'sdf_map/esdf_slice_height': 0.3,
        'sdf_map/visualization_truncate_height': max(virtual_ceil - 0.05, cruise_z + 0.3),
        'sdf_map/virtual_ceil_height': virtual_ceil,
        'sdf_map/show_occ_time': False,
        'sdf_map/show_esdf_time': False,
        'sdf_map/pose_type': 1,
        'sdf_map/frame_id': 'map',
        'manager/max_vel': max_vel,
        'manager/max_acc': max_acc,
        'manager/max_jerk': 4.0,
        'manager/dynamic_environment': 0,
        'manager/local_segment_length': 6.0,
        'manager/clearance_threshold': 0.2,
        'manager/control_points_distance': 0.5,
        'manager/use_geometric_path': False,
        'manager/use_kinodynamic_path': True,
        'manager/use_topo_path': False,
        'manager/use_optimization': True,
        'search/max_tau': 0.6,
        'search/init_max_tau': 0.8,
        'search/max_vel': max_vel,
        'search/max_acc': max_acc,
        'search/w_time': 10.0,
        'search/horizon': search_horizon,
        'search/lambda_heu': 5.0,
        'search/resolution_astar': 0.1,
        'search/time_resolution': 0.8,
        'search/margin': 0.2,
        'search/allocate_num': 100000,
        'search/check_num': 5,
        'optimization/lambda1': 10.0,
        'optimization/lambda2': 5.0,
        'optimization/lambda3': 0.00001,
        'optimization/lambda4': 0.01,
        'optimization/lambda5': 0.0,
        'optimization/lambda6': 0.0,
        'optimization/lambda7': 100.0,
        'optimization/dist0': dist0,
        'optimization/max_vel': max_vel,
        'optimization/max_acc': max_acc,
        'optimization/algorithm1': 15,
        'optimization/algorithm2': 11,
        'optimization/max_iteration_num1': 2,
        'optimization/max_iteration_num2': 300,
        'optimization/max_iteration_num3': 200,
        'optimization/max_iteration_num4': 200,
        'optimization/max_iteration_time1': 0.0001,
        'optimization/max_iteration_time2': 0.005,
        'optimization/max_iteration_time3': 0.003,
        'optimization/max_iteration_time4': 0.003,
        'optimization/order': 3,
        'bspline/limit_vel': max_vel,
        'bspline/limit_acc': max_acc,
        'bspline/limit_ratio': 1.1,
    })
    fp_params.update(fp_params_square_extra)

    if map_id in ('narrow_corridor', 'tier_medium_corridor', 'narrow'):
        # Stronger obstacle cost so B-spline does not straighten through gates.
        fp_params['optimization/lambda2'] = 12.0
        fp_params['search/margin'] = 0.12
        fp_params['search/lambda_heu'] = 3.0
        fp_params['manager/control_points_distance'] = 0.35

    actions = list(map_nodes)
    actions.extend([
        dynamics_node(
            extra_params={
                'init_x': init_x,
                'init_y': init_y,
                'init_z': cruise_z,
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(extra_params={
            'trajectory_cmd_timeout': 1.20 if speed else 0.50,
            'local_goal_timeout': 3.0 if speed else 1.5,
            # Match planner speed — 1.4 vs planner 0.7 overshoots and blows the plant.
            'max_vel': (
                float(speed['max_vel']) if speed
                else min(1.4, max_vel + 0.15)),
            'max_acc': (
                float(speed['max_acc']) if speed
                else min(2.0, max_acc + 0.3)),
            'max_tilt': 0.35 if speed else 0.45,
            # Keep off for square — ballistic fallback after traj-in-collision
            # was driving FP off-map.
            'use_drone_goal_fallback': False,
        }),
        Node(
            package='plan_manage',
            executable='fast_planner_node',
            name='fast_planner_node',
            output='screen',
            remappings=[
                ('/odom_world', '/drone/odom'),
                ('/sdf_map/odom', '/drone/odom'),
                # Homemade maps bridge into this topic; official forests publish natively.
                ('/sdf_map/cloud', '/map_generator/global_cloud'),
            ],
            parameters=[fp_params],
        ),
        Node(
            package='plan_manage',
            executable='traj_server',
            name='traj_server',
            output='screen',
            remappings=[
                ('/position_cmd', '/planning/pos_cmd'),
                ('/odom_world', '/drone/odom'),
            ],
            parameters=[{'traj_server/time_forward': 1.5}],
        ),
        Node(
            package='drone_bringup',
            executable='pose_to_path_goal',
            name='pose_to_path_goal',
            output='screen',
            parameters=[{
                'pose_topic': '/drone/goal',
                'path_topic': '/waypoint_generator/waypoints',
                'force_z': cruise_z,
            }],
        ),
        Node(
            package='drone_bringup',
            executable='ego_cmd_bridge',
            name='ego_cmd_bridge',
            output='screen',
            parameters=[{
                'cmd_topic': '/planning/pos_cmd',
                # FP publishes Marker on /planning_vis/trajectory (not EGO optimal_list).
                'optimal_topic': '/planning_vis/trajectory',
                'publish_move_base_simple': False,
                'auto_goal_enable': False,
                'cruise_height': cruise_z,
            }],
        ),
        visualization_node(),
    ])
    if mission == 'square':
        # flight_type=2 already owns the four corners; still publish /drone/goal
        # so pose_to_path_goal can re-trigger if the FSM stalls, and so the
        # controller goal-fallback has a live target.
        actions.append(square_mission_process(
            square_wps,
            delay_sec=12.0,
            arrival_tol=2.0,
            max_hold=100.0,
        ))
    else:
        actions.append(send_goal_process(
            goal_x, goal_y, cruise_z, yaw=0.0, delay_sec=10.0,
            topic='/drone/goal', repeats=goal_repeats))
    # Homemade maps: solid cylinder/wall markers + outlines (not EGO gray cloud).
    actions.append(rviz_node(
        condition=IfCondition(use_rviz), config='fast_planner_avoidance.rviz'))
    return actions


def generate_launch_description():
    nlopt_lib = _nlopt_lib_dir()
    ld = os.environ.get('LD_LIBRARY_PATH', '')
    actions: list = []
    if nlopt_lib:
        nlopt_ld = f'{nlopt_lib}:{ld}' if ld else nlopt_lib
        actions.append(SetEnvironmentVariable('LD_LIBRARY_PATH', nlopt_ld))
    else:
        print('[fast_planner_avoidance] third_party/nlopt_install/lib not found — '
              'assuming system nlopt')
    actions.extend([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('map', default_value='official_forest'),
        DeclareLaunchArgument(
            'mission', default_value='catalog',
            description='catalog = single catalog goal; square = map-specific square'),
        OpaqueFunction(function=launch_setup),
    ])
    return LaunchDescription(actions)

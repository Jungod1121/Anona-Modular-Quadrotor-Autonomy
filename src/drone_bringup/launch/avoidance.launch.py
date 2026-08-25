"""Acceptance scenario 4 — static avoidance protocol (Path B / EGO).

Built-in workflow (no Mission Console knobs required):
  - Planner: EGO-Planner rebound B-spline (Path B)
  - Map: official_forest (EGO random_forest cylinders + rings)
  - Mission: lap 1 rectangle, lap 2+ funnel (diagonal → wide → diagonal → wide)
  - Metrics: still collected by scripts/run_acceptance.py (evaluate_drone)

Path A demo: ``homemade_avoidance.launch.py``.
Generic EGO single-goal demo: ``ego_avoidance.launch.py``.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_bringup.maps_catalog import (
    official_forest_mission_waypoints,
    ego_planner_overrides,
)
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    rviz_node,
    visualization_node,
    waypoint_publisher_process,
)


def _wp_list_arg(waypoints) -> str:
    return ';'.join(f'{x},{y},{z}' for x, y, z in waypoints)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)
    cycles = int(float(LaunchConfiguration('cycles').perform(context)))
    cruise_z = float(LaunchConfiguration('cruise_z').perform(context))

    map_nodes, pose = map_stack(map_id, seed=seed, planner='ego')
    init_x, init_y, init_z = pose['init_x'], pose['init_y'], pose['init_z']
    cruise_z = max(cruise_z, float(pose.get('init_z', 1.0)), 1.0)

    # Flatten rect + funnel into one list; publisher runs once (cycles=1).
    mission_wps = [
        (float(x), float(y), cruise_z)
        for x, y, _z in official_forest_mission_waypoints(cycles)
    ]
    first_wp = mission_wps[0]

    ego_params = {
        'fsm/flight_type': 1,
        'fsm/thresh_replan_time': 3.0,
        'fsm/thresh_no_replan_meter': 2.0,
        'fsm/planning_horizon': 7.5,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,
        'fsm/cruise_height': cruise_z,
        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': float(first_wp[0]),
        'fsm/waypoint0_y': float(first_wp[1]),
        'fsm/waypoint0_z': float(first_wp[2]),
        'grid_map/resolution': 0.15,
        'grid_map/map_size_x': 50.0,
        'grid_map/map_size_y': 30.0,
        'grid_map/map_size_z': 4.0,
        'grid_map/local_update_range_x': 7.0,
        'grid_map/local_update_range_y': 7.0,
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
        # Hard floor barrier: EGO's rebound optimizer occasionally plans a
        # ground-strike trajectory when corner replans degrade; make the
        # below-0.25 m band physically unroutable instead of relying on the
        # controller's soft z-clamp.
        'grid_map/virtual_floor_height': 0.22,
        'grid_map/visualization_truncate_height': 1.8,
        'grid_map/show_occ_time': False,
        'grid_map/pose_type': 2,
        'grid_map/frame_id': 'map',
        'manager/max_vel': 0.8,
        'manager/max_acc': 1.8,
        'manager/max_jerk': 4.0,
        'manager/control_points_distance': 0.4,
        'manager/feasibility_tolerance': 0.05,
        'manager/planning_horizon': 7.5,
        'manager/use_distinctive_trajs': True,
        'manager/drone_id': 0,
        'optimization/lambda_smooth': 1.0,
        'optimization/lambda_collision': 0.5,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.5,
        'optimization/swarm_clearance': 0.5,
        'optimization/max_vel': 0.8,
        'optimization/max_acc': 1.8,
        'bspline/limit_vel': 0.8,
        'bspline/limit_acc': 1.8,
        'bspline/limit_ratio': 1.1,
        'prediction/obj_num': 0,
        'prediction/lambda': 1.0,
        'prediction/predict_rate': 1.0,
    }
    ego_params.update(ego_planner_overrides(map_id))

    ego_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='drone_0_ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', '/drone/odom'),
            # Hear /drone/goal directly (waypoint_publisher). Avoid depending only on
            # bridge fan-out to /move_base_simple/goal, which can miss one-shot goals
            # when Mission Console / other DDS participants are also online.
            # NOTE: upstream subscribes the ABSOLUTE "/move_base_simple/goal";
            # a relative 'goal' rule alone never matches it.
            ('goal', '/drone/goal'),
            ('/move_base_simple/goal', '/drone/goal'),
            ('planning/bspline', '/drone_0_planning/bspline'),
            ('planning/data_display', '/drone_0_planning/data_display'),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            ('goal_point', '/drone_0_plan_vis/goal_point'),
            ('global_list', '/drone_0_plan_vis/global_list'),
            ('init_list', '/drone_0_plan_vis/init_list'),
            ('optimal_list', '/drone_0_plan_vis/optimal_list'),
            ('a_star_list', '/drone_0_plan_vis/a_star_list'),
            ('grid_map/odom', '/drone/odom'),
            ('grid_map/cloud', '/drone_0_pcl_render_node/cloud'),
            ('grid_map/occupancy', '/drone_0_grid/grid_map/occupancy'),
            ('grid_map/occupancy_inflate', '/drone_0_grid/grid_map/occupancy_inflate'),
        ],
        parameters=[ego_params],
    )

    local_sense = Node(
        package='drone_bringup',
        executable='local_sense_cloud',
        name='drone_0_local_sense_cloud',
        output='screen',
        parameters=[{
            'global_cloud_topic': '/map_generator/global_cloud',
            'odom_topic': '/drone/odom',
            'local_cloud_topic': '/drone_0_pcl_render_node/cloud',
            # See obstacles early enough that EGO picks its replan branch
            # instead of the "suddenly discovered" emergency-stop latch.
            'sensing_horizon': 7.0,
            'sensing_rate': 10.0,
            'frame_id': 'map',
        }],
    )

    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='drone_0_traj_server',
        output='screen',
        remappings=[
            ('planning/bspline', '/drone_0_planning/bspline'),
            ('/position_cmd', '/drone_0_planning/pos_cmd'),
        ],
        parameters=[{'traj_server/time_forward': 1.0}],
    )

    # Cmd/path bridge only. Goals: waypoint_publisher → /drone/goal (EGO remapped).
    # Do NOT also fan-out to /move_base_simple/goal (would double-trigger FSM).
    bridge = Node(
        package='drone_bringup',
        executable='ego_cmd_bridge',
        name='ego_cmd_bridge',
        output='screen',
        parameters=[{
            'cmd_topic': '/drone_0_planning/pos_cmd',
            'auto_goal_enable': False,
            'cruise_height': cruise_z,
            'publish_move_base_simple': False,
            'goal_out_topic': '/drone/goal_bridge_unused',
        }],
    )

    actions = list(map_nodes)
    actions.extend([
        dynamics_node(
            extra_params={
                'init_x': init_x,
                'init_y': init_y,
                'init_z': max(float(init_z), cruise_z),
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(extra_params={
            'trajectory_cmd_timeout': 0.40,
            'local_goal_timeout': 1.5,
            'max_vel': 0.8,
            'max_acc': 1.8,
            'max_tilt': 0.40,
            'use_drone_goal_fallback': False,
        }),
        ego_node,
        local_sense,
        traj_server,
        bridge,
        visualization_node(),
        waypoint_publisher_process(
            pattern='list',
            delay_sec=10.0,
            hold_sec=1.0,
            extra_args=[
                # Use --list=... so a leading '-' in the first WP is not parsed as a flag.
                f'--list={_wp_list_arg(mission_wps)}',
                # Mission already encodes lap1 rect + lap2+ funnel; do not re-loop.
                '--cycles', '1',
                '--wait-arrival',
                '--arrival-tol', '1.2',
                '--max-hold', '60.0',
                '--z', str(cruise_z),
            ],
        ),
        rviz_node(condition=IfCondition(use_rviz), config='ego_avoidance.rviz'),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument(
            'map', default_value='official_forest',
            description='Acceptance default: official_forest (EGO random_forest)'),
        DeclareLaunchArgument(
            'cycles', default_value='2',
            description='Lap count: 1=rectangle only; 2+=rectangle then funnel laps'),
        DeclareLaunchArgument('cruise_z', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])

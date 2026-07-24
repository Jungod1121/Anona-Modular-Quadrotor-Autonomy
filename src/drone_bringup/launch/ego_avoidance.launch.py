"""Path B: official EGO planner + selectable map + our dynamics/controller.

Does NOT use drone_planner or so3/fake_drone. Path A remains homemade_avoidance.launch.py.
Default map is official_forest; switch with map:=… (see MAPS.md).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from drone_bringup.maps_catalog import (
    benchmark_square_waypoints,
    ego_planner_overrides,
)
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    resolve_mission_pose,
    rviz_node,
    send_goal_process,
    square_mission_process,
    square_planner_speed_params,
    square_speed_params,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)
    mission = LaunchConfiguration('mission').perform(context).strip().lower()

    map_nodes, pose = map_stack(map_id, seed=seed, planner='ego')
    pose = resolve_mission_pose(map_id, pose, mission)
    init_x, init_y, init_z = pose['init_x'], pose['init_y'], pose['init_z']
    goal_x, goal_y, goal_z = pose['goal_x'], pose['goal_y'], pose['goal_z']
    square_wps = (
        benchmark_square_waypoints(map_id) if mission == 'square' else [])
    first_goal = square_wps[0] if square_wps else (goal_x, goal_y, goal_z)

    ego_params = {
        'fsm/flight_type': 1,
        'fsm/thresh_replan_time': 3.0,
        'fsm/thresh_no_replan_meter': 2.0,
        'fsm/planning_horizon': 7.5,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,
        'fsm/cruise_height': 1.0,
        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': float(first_goal[0]),
        'fsm/waypoint0_y': float(first_goal[1]),
        'fsm/waypoint0_z': float(first_goal[2]),
        'grid_map/resolution': 0.15,
        'grid_map/map_size_x': 50.0,
        'grid_map/map_size_y': 30.0,
        'grid_map/map_size_z': 4.0,
        # Official advanced_param (with local sensing, not full-map dump).
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
        'manager/max_vel': 1.5,
        'manager/max_acc': 2.0,
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
        'optimization/max_vel': 1.5,
        'optimization/max_acc': 2.0,
        'bspline/limit_vel': 1.5,
        'bspline/limit_acc': 2.0,
        'bspline/limit_ratio': 1.1,
        'prediction/obj_num': 0,
        'prediction/lambda': 1.0,
        'prediction/predict_rate': 1.0,
    }
    ego_params.update(ego_planner_overrides(map_id))
    ego_params.update(square_planner_speed_params(mission))
    ctrl_extra = {
        'trajectory_cmd_timeout': 0.40,
        'local_goal_timeout': 1.5,
        'max_vel': 1.2,
        'max_acc': 1.8,
        'max_tilt': 0.40,
    }
    ctrl_extra.update(square_speed_params(mission))

    ego_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='drone_0_ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', '/drone/odom'),
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
            # Official: local scan from sensing_horizon (same maps, cropped).
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
            'sensing_horizon': 5.0,
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

    use_auto_goal = mission != 'square'
    bridge = Node(
        package='drone_bringup',
        executable='ego_cmd_bridge',
        name='ego_cmd_bridge',
        output='screen',
        parameters=[{
            'cmd_topic': '/drone_0_planning/pos_cmd',
            'auto_goal_enable': use_auto_goal,
            'auto_goal_x': float(first_goal[0]),
            'auto_goal_y': float(first_goal[1]),
            'auto_goal_z': float(first_goal[2]),
            'auto_goal_delay': 8.0,
            'auto_goal_repeats': 1,
            'auto_goal_period': 0.5,
            'cruise_height': 1.0,
        }],
    )

    actions = list(map_nodes)
    actions.extend([
        dynamics_node(
            extra_params={
                'init_x': init_x,
                'init_y': init_y,
                'init_z': init_z,
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(extra_params=ctrl_extra),
        ego_node,
        local_sense,
        traj_server,
        bridge,
        visualization_node(),
    ])
    if mission == 'square':
        actions.append(square_mission_process(square_wps, delay_sec=9.0))
    else:
        actions.append(send_goal_process(
            goal_x, goal_y, goal_z, yaw=0.0, delay_sec=9.0,
            topic='/drone/goal', repeats=1))
    actions.append(
        rviz_node(condition=IfCondition(use_rviz), config='ego_avoidance.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument(
            'map', default_value='official_forest',
            description='Map id — see MAPS.md / maps_catalog.py'),
        DeclareLaunchArgument(
            'mission', default_value='catalog',
            description='catalog = single catalog goal; square = map-specific square'),
        OpaqueFunction(function=launch_setup),
    ])

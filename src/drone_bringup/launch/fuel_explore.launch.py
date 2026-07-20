"""Path D — FUEL-style exploration with Path B (EGO) trajectory backend.

Fog sensing + frontier FSM (trigger on /drone/goal) publish sequential
/exploration/nav_goal → ego_cmd_bridge → EGO. Plant stays ours (no SO3).

Frontiers / local sensing use /map/obstacles_local (fog). EGO collision
map uses bridged /map_generator/global_cloud for stability — same plant
contract as Path B otherwise.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from drone_bringup.maps_catalog import (
    ego_planner_overrides,
    explore_box_params,
    fuel_explore_params,
    normalize_map_id,
)
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    rviz_node,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = normalize_map_id(
        LaunchConfiguration('map').perform(context), planner='fuel_explore')

    # Path D poses (inside free space) + same cloud wiring as Path B.
    map_nodes, pose = map_stack(map_id, seed=seed, planner='fuel_explore')
    cruise_z = float(pose.get('init_z', 1.0))

    explore_yaml = os.path.join(
        get_package_share_directory('drone_exploration'),
        'config', 'exploration.yaml')
    box = explore_box_params(map_id, planner='fuel_explore')
    fsm_extra = fuel_explore_params(map_id)
    sense_r = float(fsm_extra.pop('sensing_radius', 5.0 if map_id.startswith('official') else 4.0))
    sense_res = float(fsm_extra.pop('resolution', 0.4))
    frontier_min = int(fsm_extra.pop('frontier_min_size', 4))

    ego_params = {
        'fsm/flight_type': 1,
        'fsm/thresh_replan_time': 2.0,
        'fsm/thresh_no_replan_meter': 1.5,
        'fsm/planning_horizon': 7.5,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,
        'fsm/cruise_height': cruise_z,
        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': float(pose['goal_x']),
        'fsm/waypoint0_y': float(pose['goal_y']),
        'fsm/waypoint0_z': cruise_z,
        'grid_map/resolution': 0.15,
        'grid_map/map_size_x': 50.0,
        'grid_map/map_size_y': 30.0,
        'grid_map/map_size_z': 5.0 if map_id == 'official_perlin' else 4.0,
        'grid_map/local_update_range_x': 20.0,
        'grid_map/local_update_range_y': 14.0,
        'grid_map/local_update_range_z': 3.0,
        # Slightly thicker inflate than Path B default — plant + dense cylinders.
        'grid_map/obstacles_inflation': 0.16,
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
        'grid_map/virtual_ceil_height': max(2.8, cruise_z + 1.5),
        'grid_map/visualization_truncate_height': max(2.5, cruise_z + 1.2),
        'grid_map/show_occ_time': False,
        'grid_map/pose_type': 2,
        'grid_map/frame_id': 'map',
        'manager/max_vel': 1.2,
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
        'optimization/dist0': 0.55,
        'optimization/swarm_clearance': 0.5,
        'optimization/max_vel': 1.2,
        'optimization/max_acc': 1.8,
        'bspline/limit_vel': 1.2,
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
            # Fog-of-war for frontiers; give EGO the denser bridged cloud so
            # grid_map stays stable (full /map_generator/global_cloud).
            ('grid_map/cloud', '/map_generator/global_cloud'),
            ('grid_map/occupancy_inflate', '/drone_0_grid/grid_map/occupancy_inflate'),
        ],
        parameters=[ego_params],
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

    # Sequential explore goals → EGO; do NOT consume /drone/goal (that is trigger only).
    bridge = Node(
        package='drone_bringup',
        executable='ego_cmd_bridge',
        name='ego_cmd_bridge',
        output='screen',
        parameters=[{
            'cmd_topic': '/drone_0_planning/pos_cmd',
            'goal_in_topic': '/exploration/nav_goal',
            'goal_out_topic': '/move_base_simple/goal',
            'auto_goal_enable': False,
            'cruise_height': cruise_z,
        }],
    )

    actions = list(map_nodes)
    actions.extend([
        dynamics_node(
            extra_params={
                'init_x': pose['init_x'],
                'init_y': pose['init_y'],
                'init_z': pose['init_z'],
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(extra_params={
            'trajectory_cmd_timeout': 0.40,
            'local_goal_timeout': 1.5,
            'max_vel': 1.0,
            'max_acc': 1.6,
            'max_tilt': 0.40,
            # Critical: /drone/goal is explore *trigger* only. With fallback on,
            # Start Explore used to send the far catalog goal and fly straight
            # through obstacles.
            'use_drone_goal_fallback': False,
        }),
        Node(
            package='drone_exploration',
            executable='local_sensing',
            name='local_sensing',
            output='screen',
            parameters=[explore_yaml, {
                **box,
                'resolution': sense_res,
                'frontier_min_size': frontier_min,
                'global_cloud_topic': '/map/obstacles',
                'local_cloud_topic': '/map/obstacles_local',
                'odom_topic': '/drone/odom',
                'frontiers_topic': '/exploration/frontiers',
                'cruise_z': cruise_z,
                'sensing_radius': sense_r,
            }],
        ),
        Node(
            package='drone_exploration',
            executable='exploration_fsm',
            name='exploration_fsm',
            output='screen',
            parameters=[explore_yaml, {
                **box,
                **fsm_extra,
                'cruise_z': cruise_z,
                'trigger_topic': '/drone/goal',
                'nav_goal_topic': '/exploration/nav_goal',
                'frontiers_topic': '/exploration/frontiers',
                'odom_topic': '/drone/odom',
                'replan_period': 1.5,
            }],
        ),
        ego_node,
        traj_server,
        bridge,
        visualization_node(),
        rviz_node(condition=IfCondition(use_rviz), config='ego_avoidance.rviz'),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument(
            'map', default_value='narrow_corridor',
            description='Any MAPS.md id — Path D sensing box is per-map.'),
        OpaqueFunction(function=launch_setup),
    ])

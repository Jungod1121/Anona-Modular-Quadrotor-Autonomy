"""Path C: GCOPTER/MINCO + selectable map + our dynamics/controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

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
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)
    mission = LaunchConfiguration('mission').perform(context).strip().lower()

    map_nodes, pose = map_stack(map_id, seed=seed, planner='gcopter')
    pose = resolve_mission_pose(map_id, pose, mission)
    goal_x, goal_y, goal_z = pose['goal_x'], pose['goal_y'], pose['goal_z']

    gcopter_yaml = os.path.join(
        get_package_share_directory('gcopter'), 'config', 'global_planning.yaml')
    gcopter_extra = {
        'CruiseHeight': 1.0,
        **gcopter_planner_overrides(map_id),
    }
    speed = square_speed_params(mission, strict=True)
    if speed:
        # Flat overrides on top of yaml — keep square flights trackable.
        # Soft dilate: forest catalog 0.22 marks spawn/goals occupied and
        # nudges z→0.5; then tracking diverges off-map.
        gcopter_extra['MaxVelMag'] = speed['max_vel']
        gcopter_extra['MaxBdrMag'] = max(0.8, speed['max_acc'])
        gcopter_extra['DilateRadius'] = 0.08
        gcopter_extra['CruiseHeight'] = float(pose['init_z'])
        gcopter_extra['WeightT'] = 20.0
        gcopter_extra['TimeoutRRT'] = 0.08

    gcopter_node = Node(
        package='gcopter',
        executable='global_planning_node',
        name='global_planning_node',
        output='screen',
        parameters=[gcopter_yaml, gcopter_extra],
    )

    ctrl_extra = {
        'trajectory_cmd_timeout': 1.20,
        'local_goal_timeout': 3.0,
        'max_vel': float(speed['max_vel']) if speed else 1.2,
        'max_acc': float(speed['max_acc']) if speed else 1.8,
        'max_tilt': 0.35,
        # Square: allow gentle goal fallback if traj tracking is lost mid-lap.
        'use_drone_goal_fallback': bool(speed),
    }
    if speed:
        ctrl_extra.update(speed)

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
        controller_node(extra_params=ctrl_extra),
        gcopter_node,
        visualization_node(),
    ])
    if mission == 'square':
        actions.append(square_mission_process(
            benchmark_square_waypoints(map_id),
            delay_sec=20.0,
            arrival_tol=1.8,
            max_hold=95.0,
        ))
    else:
        actions.append(send_goal_process(
            goal_x, goal_y, goal_z, yaw=0.0, delay_sec=15.0,
            topic='/drone/goal', repeats=4))
    # Path C must not use ego_avoidance.rviz (InflatedOcc is EGO-only and
    # looks like a blank "wallpaper"). Use Obstacles on /map/obstacles.
    actions.append(
        rviz_node(condition=IfCondition(use_rviz), config='gcopter_avoidance.rviz'))
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

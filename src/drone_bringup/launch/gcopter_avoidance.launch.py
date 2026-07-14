"""Path C: GCOPTER/MINCO + selectable map + our dynamics/controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from drone_bringup.maps_catalog import gcopter_planner_overrides
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    rviz_node,
    send_goal_process,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(float(LaunchConfiguration('seed').perform(context)))
    map_id = LaunchConfiguration('map').perform(context)

    map_nodes, pose = map_stack(map_id, seed=seed, planner='gcopter')
    goal_x, goal_y, goal_z = pose['goal_x'], pose['goal_y'], pose['goal_z']

    gcopter_yaml = os.path.join(
        get_package_share_directory('gcopter'), 'config', 'global_planning.yaml')
    gcopter_extra = {
        'CruiseHeight': 1.0,
        **gcopter_planner_overrides(map_id),
    }

    gcopter_node = Node(
        package='gcopter',
        executable='global_planning_node',
        name='global_planning_node',
        output='screen',
        parameters=[gcopter_yaml, gcopter_extra],
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
            'max_vel': 1.2,
            'max_acc': 1.8,
            'max_tilt': 0.40,
        }),
        gcopter_node,
        visualization_node(),
        send_goal_process(
            goal_x, goal_y, goal_z, yaw=0.0, delay_sec=8.0,
            topic='/drone/goal', repeats=1),
        rviz_node(condition=IfCondition(use_rviz), config='ego_avoidance.rviz'),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument(
            'map', default_value='official_forest',
            description='Map id — see MAPS.md / maps_catalog.py'),
        OpaqueFunction(function=launch_setup),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.maps_catalog import homemade_planner_overrides
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    planner_node,
    rviz_node,
    send_goal_process,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(LaunchConfiguration('seed').perform(context))
    map_id = LaunchConfiguration('map').perform(context)

    map_nodes, pose = map_stack(map_id, seed=seed, planner='homemade')
    planner_extra = homemade_planner_overrides(map_id, planner='homemade')

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
        controller_node(),
        planner_node(extra_params=planner_extra or None),
        visualization_node(),
        send_goal_process(
            pose['goal_x'], pose['goal_y'], pose['goal_z'],
            yaw=0.0, delay_sec=4.0),
        rviz_node(condition=IfCondition(use_rviz)),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument(
            'map', default_value='dense_field',
            description='Map id — see MAPS.md / maps_catalog.py'),
        OpaqueFunction(function=launch_setup),
    ])

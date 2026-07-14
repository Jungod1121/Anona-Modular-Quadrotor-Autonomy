from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_node,
    rviz_node,
    visualization_node,
    waypoint_publisher_process,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    pattern = LaunchConfiguration('pattern')

    actions = [
        dynamics_node(
            extra_params={
                'init_x': 0.0,
                'init_y': 0.0,
                'init_z': 0.0,
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(),
        map_node('map_sparse.yaml'),
        visualization_node(),
        waypoint_publisher_process(
            pattern=pattern.perform(context),
            delay_sec=5.0,
            extra_args=['--z', '1.5', '--side', '2.0'],
        ),
    ]

    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('pattern', default_value='square'),
        OpaqueFunction(function=launch_setup),
    ])

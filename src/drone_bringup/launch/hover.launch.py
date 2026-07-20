from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_node,
    rviz_node,
    send_goal_process,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')

    dynamics = dynamics_node(
        extra_params={
            'init_x': 0.0,
            'init_y': 0.0,
            'init_z': 0.15,
            'init_yaw': 0.0,
        },
        param_files=['dynamics.yaml'],
    )

    actions = [
        dynamics,
        controller_node(),
        map_node('map_sparse.yaml'),
        visualization_node(),
        send_goal_process(0.0, 0.0, 1.5, yaw=0.0, delay_sec=3.0),
    ]

    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])

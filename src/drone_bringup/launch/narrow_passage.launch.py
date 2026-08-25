from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_node,
    planner_node,
    rviz_node,
    send_goal_process,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')

    actions = [
        dynamics_node(
            extra_params={
                'init_x': 1.0,
                'init_y': 5.0,
                'init_z': 1.5,
            },
            param_files=['dynamics.yaml'],
        ),
        controller_node(),
        map_node('map_narrow.yaml'),
        # map_id merges the narrow_corridor Path A hints (2D A* at cruise_z,
        # auto-inflate 0.24–0.32, max_vel 0.65) — without them the planner
        # ran 3D A* and dove under floating clutter, clipping gate frames.
        planner_node(map_id='narrow_corridor'),
        visualization_node(),
        send_goal_process(17.0, 5.0, 1.5, yaw=0.0, delay_sec=4.0),
    ]

    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])

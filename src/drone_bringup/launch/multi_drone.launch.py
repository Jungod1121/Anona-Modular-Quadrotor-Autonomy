"""Two-drone (bonus) demo with namespace parameterization."""

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


def _drone_stack(ns: str, init_xy, goal_xy, z: float = 1.5):
    """One independent dynamics+controller+map+planner stack under `ns`."""
    return [
        dynamics_node(
            namespace=ns,
            extra_params={
                'init_x': init_xy[0],
                'init_y': init_xy[1],
                'init_z': z,
            },
        ),
        controller_node(namespace=ns),
        map_node(
            'map_sparse.yaml',
            namespace=ns,
            extra_params={
                'start_x': init_xy[0],
                'start_y': init_xy[1],
                'start_z': z,
                'goal_x': goal_xy[0],
                'goal_y': goal_xy[1],
                'goal_z': z,
            },
        ),
        planner_node(namespace=ns),
        visualization_node(namespace=ns),
        send_goal_process(
            goal_xy[0], goal_xy[1], z, yaw=0.0, delay_sec=4.0,
            topic=f'/{ns}/drone/goal',
        ),
    ]


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    actions = []
    # uav0: (0,0) → (2,1); uav1: (0,3) → (2,4) — no path collision in sparse map
    actions.extend(_drone_stack('uav0', (0.0, 0.0), (2.0, 1.0)))
    actions.extend(_drone_stack('uav1', (0.0, 3.0), (2.0, 4.0)))
    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])

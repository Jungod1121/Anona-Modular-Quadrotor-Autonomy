"""Shared-field avoidance: one dense map, two drones, inter-drone collision avoidance."""

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


def _drone(ns: str, init, goal, peers: str):
    z = 1.5
    return [
        dynamics_node(
            namespace=ns,
            extra_params={
                'init_x': init[0],
                'init_y': init[1],
                'init_z': z,
            },
        ),
        controller_node(namespace=ns),
        planner_node(
            namespace=ns,
            extra_params={
                'map_topic': '/map/obstacles',
                'peer_namespaces': peers,
                'peer_radius': 0.75,
                'peer_replan_dist': 1.8,
                # Faster / more reliable for two concurrent planners.
                'use_dyn_astar': False,
                'enable_bspline_opt': False,
            },
        ),
        visualization_node(namespace=ns),
        send_goal_process(
            goal[0], goal[1], z, yaw=0.0, delay_sec=5.0,
            topic=f'/{ns}/drone/goal',
        ),
    ]


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(LaunchConfiguration('seed').perform(context))

    # Opposite-direction parallel lanes on dense_field (same static map).
    # Starts on the same clear end so neither goal sits under a peer at t=0.
    uav0_init, uav0_goal = (1.0, 4.0), (17.0, 6.0)
    uav1_init, uav1_goal = (1.0, 6.0), (17.0, 4.0)

    actions = [
        # Shared world map (no namespace) — both planners subscribe via map_topic.
        map_node(
            'map_dense.yaml',
            extra_params={
                'seed': seed,
                'start_x': 1.0,
                'start_y': 5.0,
                'goal_x': 17.0,
                'goal_y': 5.0,
                'local_sense_radius': 0.0,  # no per-drone local sense on shared node
            },
        ),
    ]
    actions.extend(_drone('uav0', uav0_init, uav0_goal, peers='uav1'))
    actions.extend(_drone('uav1', uav1_init, uav1_goal, peers='uav0'))
    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        OpaqueFunction(function=launch_setup),
    ])

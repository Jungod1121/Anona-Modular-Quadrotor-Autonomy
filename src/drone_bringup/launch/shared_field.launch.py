"""Shared-field avoidance: one dense map, two drones, inter-drone collision avoidance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
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
        controller_node(namespace=ns, extra_params={
            # Stay on planner local_goal — do not ballistic-fly through pillars.
            'use_drone_goal_fallback': False,
            'local_goal_timeout': 2.5,
            'max_vel': 1.2,
        }),
        planner_node(
            namespace=ns,
            extra_params={
                'map_topic': '/map/obstacles',
                'peer_namespaces': peers,
                'peer_radius': 0.70,
                'peer_replan_dist': 1.8,
                'use_dyn_astar': False,
                'enable_bspline_opt': False,
                # Start sits near pillars: do not seal AABB edges; snap farther.
                'seal_boundary_layers': 0,
                'free_snap_radius': 20,
                'auto_inflate_max': 0.28,
                'execution_safety_enable': False,
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

    # Tight clearance so start is next to pillars, still free enough to take off.
    map_nodes, _pose = map_stack(
        'dense_field',
        seed=seed,
        planner='homemade',
        map_extra={
            'start_x': 2.0,
            'start_y': 12.0,
            'goal_x': 40.0,
            'goal_y': 12.0,
            'clearance_radius': 0.55,
            'add_boundary_walls': False,
        },
    )

    # Parallel west→east lanes through the dense field (near obstacles).
    uav0_init, uav0_goal = (2.0, 9.0), (40.0, 18.0)
    uav1_init, uav1_goal = (2.0, 15.0), (40.0, 6.0)

    actions = list(map_nodes)
    actions.extend(_drone('uav0', uav0_init, uav0_goal, peers='uav1'))
    actions.extend(_drone('uav1', uav1_init, uav1_goal, peers='uav0'))
    actions.append(
        rviz_node(condition=IfCondition(use_rviz), config='multi_homemade.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        OpaqueFunction(function=launch_setup),
    ])

"""Path A (homemade Dyn-A* + B-spline) static avoidance demo.

Acceptance scenario 4 now uses Fast Planner via ``avoidance.launch.py``.
Use this launch (or ``planner_sim.launch.py planner:=homemade``) for Path A.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.maps_catalog import (
    benchmark_square_waypoints,
    homemade_planner_overrides,
)
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    planner_node,
    resolve_mission_pose,
    rviz_node,
    send_goal_process,
    square_mission_process,
    square_speed_params,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(LaunchConfiguration('seed').perform(context))
    map_id = LaunchConfiguration('map').perform(context)
    mission = LaunchConfiguration('mission').perform(context).strip().lower()

    map_nodes, pose = map_stack(map_id, seed=seed, planner='homemade')
    pose = resolve_mission_pose(map_id, pose, mission)
    planner_extra = homemade_planner_overrides(map_id, planner='homemade') or {}
    speed = square_speed_params(mission)
    if speed:
        planner_extra = {**planner_extra, **speed}

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
        controller_node(extra_params=speed or None),
        planner_node(extra_params=planner_extra or None),
        visualization_node(),
    ])
    if mission == 'square':
        actions.append(square_mission_process(
            benchmark_square_waypoints(map_id), delay_sec=5.0))
    else:
        actions.append(send_goal_process(
            pose['goal_x'], pose['goal_y'], pose['goal_z'],
            yaw=0.0, delay_sec=4.0))
    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument(
            'map', default_value='dense_field',
            description='Map id — see MAPS.md / maps_catalog.py'),
        DeclareLaunchArgument(
            'mission', default_value='catalog',
            description='catalog = single catalog goal; square = map-specific square'),
        OpaqueFunction(function=launch_setup),
    ])

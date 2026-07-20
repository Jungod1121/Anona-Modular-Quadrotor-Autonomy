from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    rl_planner_node,
    rviz_node,
    send_goal_process,
    vfh_planner_node,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(LaunchConfiguration('seed').perform(context))
    map_id = LaunchConfiguration('map').perform(context)
    backend = LaunchConfiguration('backend').perform(context).strip().lower()

    map_nodes, pose = map_stack(map_id, seed=seed, planner='homemade')
    cruise_z = float(pose.get('init_z', 1.5))

    # Default Path G = classical VFH+ (PX4-Avoidance style). RL is opt-in.
    if backend in ('rl', 'ppo', 'learning'):
        planner = rl_planner_node(cruise_z=cruise_z)
        max_vel = 0.75
    else:
        planner = vfh_planner_node(cruise_z=cruise_z)
        max_vel = 0.9

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
            'use_drone_goal_fallback': False,
            'max_vel': max_vel,
            'goal_slowdown_dist': 2.5,
        }),
        planner,
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
            description='Any catalog map'),
        DeclareLaunchArgument(
            'backend', default_value='vfh',
            description='Path G backend: vfh (default, classical) | rl (PPO research)'),
        OpaqueFunction(function=launch_setup),
    ])

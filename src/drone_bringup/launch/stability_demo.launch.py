import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    evaluate_process,
    interference_monitor_process,
    map_node,
    rviz_node,
    script_path,
    send_goal_process,
    visualization_node,
)


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    run_eval = LaunchConfiguration('run_eval')
    use_interference_panel = LaunchConfiguration('use_interference_panel')
    mode = LaunchConfiguration('mode')

    mode_val = mode.perform(context)
    if mode_val == 'single_goal':
        goal = (2.0, 1.0, 1.5)
        map_cfg = 'map_sparse.yaml'
        map_extra = {
            'goal_x': goal[0],
            'goal_y': goal[1],
            'goal_z': goal[2],
        }
        init = {'init_x': 0.0, 'init_y': 0.0, 'init_z': 0.0}
    else:
        goal = (0.0, 0.0, 1.5)
        map_cfg = 'map_sparse.yaml'
        map_extra = {}
        init = {'init_x': 0.0, 'init_y': 0.0, 'init_z': 0.0}

    dynamics_params = {
        **init,
        'wind_enable': True,
        # Milder wind — with I+DOB recovery, hover stays well under 0.3 m.
        'wind_const_x': 0.22,
        'wind_const_y': -0.16,
        'wind_sin_amp': 0.08,
        'wind_sin_freq': 0.2,
        'imu_noise_enable': True,
        'imu_accel_noise_std': 0.015,
        'imu_gyro_noise_std': 0.008,
    }

    actions = [
        dynamics_node(
            extra_params=dynamics_params,
            param_files=['dynamics.yaml', 'imu.yaml'],
        ),
        controller_node(),
        map_node(map_cfg, extra_params=map_extra),
        visualization_node(),
        send_goal_process(goal[0], goal[1], goal[2], yaw=0.0, delay_sec=3.0),
    ]

    # Dedicated interference HUD — only for this scenario (not S1–S5).
    if use_interference_panel.perform(context).lower() in ('1', 'true', 'yes'):
        actions.append(interference_monitor_process(delay_sec=2.5, goal=goal))

    output_dir = os.path.join(os.path.dirname(script_path('evaluate.py')), 'output')
    if run_eval.perform(context).lower() in ('1', 'true', 'yes'):
        actions.append(
            evaluate_process(
                delay_sec=8.0,
                duration_sec=90.0,
                output_dir=output_dir,
                goal=goal,
            ),
        )

    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('run_eval', default_value='true',
                              description='Start scripts/evaluate.py for CSV+plots'),
        DeclareLaunchArgument(
            'use_interference_panel', default_value='true',
            description='Open floating Interference Monitor for wind/IMU status'),
        DeclareLaunchArgument('mode', default_value='hover',
                              description='hover or single_goal'),
        OpaqueFunction(function=launch_setup),
    ])

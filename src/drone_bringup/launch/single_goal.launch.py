from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
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
    rviz_on = use_rviz.perform(context).lower() in ('true', '1', 'yes')
    goal_delay_raw = LaunchConfiguration('goal_delay').perform(context).strip()

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
        map_node('map_sparse.yaml', extra_params={
            'start_x': 0.0,
            'start_y': 0.0,
            'start_z': 0.0,
            'goal_x': 2.0,
            'goal_y': 1.0,
            'goal_z': 1.5,
        }),
        visualization_node(extra_params={
            'show_mission_endpoints': True,
            'mission_start_x': 0.0,
            'mission_start_y': 0.0,
            'mission_start_z': 0.0,
            'mission_goal_x': 2.0,
            'mission_goal_y': 1.0,
            'mission_goal_z': 1.5,
        }),
    ]

    rviz = rviz_node(condition=IfCondition(use_rviz))
    actions.append(rviz)

    if goal_delay_raw:
        # Explicit override retains fixed-delay behavior for scripted runs.
        actions.append(send_goal_process(
            2.0, 1.0, 1.5, yaw=0.0,
            delay_sec=float(goal_delay_raw),
        ))
    elif not rviz_on:
        actions.append(send_goal_process(
            2.0, 1.0, 1.5, yaw=0.0,
            delay_sec=3.0,
        ))
    else:
        goal_started = [False]

        def start_goal_when_rviz_ready(event):
            if goal_started[0] or b'OpenGl version:' not in event.text:
                return None
            goal_started[0] = True
            # Give RViz one second to finish constructing displays after its
            # OpenGL context is ready, then start the visible flight.
            return send_goal_process(
                2.0, 1.0, 1.5, yaw=0.0,
                delay_sec=1.0,
            )

        actions.append(RegisterEventHandler(OnProcessIO(
            target_action=rviz,
            on_stdout=start_goal_when_rviz_ready,
            on_stderr=start_goal_when_rviz_ready,
        )))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'goal_delay', default_value='',
            description='Fixed goal delay override; empty waits for RViz readiness'),
        OpaqueFunction(function=launch_setup),
    ])

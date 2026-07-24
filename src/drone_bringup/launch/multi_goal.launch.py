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
    pattern_name = pattern.perform(context)

    actions = [
        dynamics_node(
            extra_params={
                'init_x': 0.0,
                'init_y': 0.0,
                'init_z': 0.0,
            },
            param_files=['dynamics.yaml'],
        ),
        # Scenario 3 uses direct waypoint tracking.  Give the horizontal loop
        # extra damping so each corner settles instead of drawing a hook.
        controller_node(extra_params={
            'pos_kd.x': 2.2,
            'pos_kd.y': 2.2,
            'pos_ki.x': 0.0,
            'pos_ki.y': 0.0,
            'disturbance_reject_enable': False,
            'max_vel': 0.8,
            'max_acc': 1.2,
        }),
        map_node('map_sparse.yaml'),
        visualization_node(),
    ]

    if pattern_name == 'square':
        # Start at the vehicle's XY origin, then trace four edges.  The old
        # square was centred on the origin, so the recorded path first drew a
        # diagonal from the centre to (1,1), making the result look non-square.
        # The current XY=(0,0) is the implicit first vertex.
        square = '2,0,1.5;2,2,1.5;0,2,1.5;0,0,1.5'
        actions.append(waypoint_publisher_process(
            pattern='list',
            delay_sec=5.0,
            extra_args=[
                f'--list={square}',
                '--wait-arrival',
                '--arrival-tol', '0.12',
                '--max-hold', '25',
            ],
        ))
    else:
        actions.append(waypoint_publisher_process(
            pattern=pattern_name,
            delay_sec=5.0,
            extra_args=['--z', '1.5', '--side', '2.0'],
        ))

    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('pattern', default_value='square'),
        OpaqueFunction(function=launch_setup),
    ])

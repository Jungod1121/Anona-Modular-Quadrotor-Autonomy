"""Formation flight demo (sparse field): 1 leader + 2 followers, several shapes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    libexec,
    map_node,
    planner_node,
    rviz_node,
    send_goal_process,
    visualization_node,
)


def _drone(ns: str, init_xy, peers: str = ''):
    z = 1.5
    extra_planner = {
        'map_topic': '/map/obstacles',
    }
    if peers:
        extra_planner['peer_namespaces'] = peers
        extra_planner['peer_radius'] = 0.7
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
        planner_node(namespace=ns, extra_params=extra_planner),
        visualization_node(namespace=ns),
    ]


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    formation = LaunchConfiguration('formation').perform(context)
    spacing = float(LaunchConfiguration('spacing').perform(context))

    # Initial spatulae matching rough formation so takeoff is calm.
    inits = {
        'line': [(0.0, 0.0), (0.0, spacing), (0.0, -spacing)],
        'column': [(0.0, 0.0), (-spacing, 0.0), (-2.0 * spacing, 0.0)],
        'v': [(0.0, 0.0), (-spacing, 0.75 * spacing), (-spacing, -0.75 * spacing)],
        'triangle': [(0.0, 0.0), (-spacing, 0.9 * spacing), (-spacing, -0.9 * spacing)],
        'diamond': [(0.0, 0.0), (-spacing, 0.0), (0.0, spacing)],
    }
    poses = inits.get(formation, inits['v'])

    actions = [
        map_node(
            'map_sparse.yaml',
            extra_params={'local_sense_radius': 0.0},
        ),
    ]
    actions.extend(_drone('uav0', poses[0], peers='uav1,uav2'))
    actions.extend(_drone('uav1', poses[1], peers='uav0,uav2'))
    actions.extend(_drone('uav2', poses[2], peers='uav0,uav1'))

    # Leader flies a square-ish tour; followers track formation offsets.
    actions.append(
        send_goal_process(3.0, 0.0, 1.5, delay_sec=4.0, topic='/uav0/drone/goal'))
    actions.append(
        send_goal_process(3.0, 3.0, 1.5, delay_sec=18.0, topic='/uav0/drone/goal'))
    actions.append(
        send_goal_process(0.0, 3.0, 1.5, delay_sec=32.0, topic='/uav0/drone/goal'))
    actions.append(
        send_goal_process(0.0, 0.0, 1.5, delay_sec=46.0, topic='/uav0/drone/goal'))

    form_cmd = [
        libexec('formation_coordinator'),
        '--leader', 'uav0',
        '--followers', 'uav1,uav2',
        '--formation', formation,
        '--spacing', str(spacing),
        '--z', '1.5',
        '--rate', '5',
    ]
    actions.append(
        TimerAction(
            period=3.0,
            actions=[ExecuteProcess(cmd=form_cmd, output='screen')],
        )
    )
    actions.append(rviz_node(condition=IfCondition(use_rviz)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'formation', default_value='v',
            description='line | column | v | triangle | diamond'),
        DeclareLaunchArgument('spacing', default_value='1.5'),
        OpaqueFunction(function=launch_setup),
    ])

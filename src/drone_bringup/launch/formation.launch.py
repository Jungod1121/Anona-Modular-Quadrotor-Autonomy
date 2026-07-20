"""Formation flight demo on dense_field: 1 leader + 2 followers, several shapes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    libexec,
    map_stack,
    planner_node,
    rviz_node,
    send_goal_process,
    visualization_node,
)


def _drone(ns: str, init_xy, peers: str = ''):
    z = 1.5
    extra_planner = {
        'map_topic': '/map/obstacles',
        'seal_boundary_layers': 0,
        'free_snap_radius': 20,
        'auto_inflate_max': 0.28,
        'execution_safety_enable': False,
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
        controller_node(namespace=ns, extra_params={
            'use_drone_goal_fallback': False,
            'local_goal_timeout': 2.5,
        }),
        planner_node(namespace=ns, extra_params=extra_planner),
        visualization_node(namespace=ns),
    ]


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    formation = LaunchConfiguration('formation').perform(context)
    spacing = float(LaunchConfiguration('spacing').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))

    # Spawn poses for line / column / V (3 UAVs). Triangle/diamond removed.
    ax, ay = 3.0, 12.0
    inits = {
        'line': [(ax, ay), (ax, ay + spacing), (ax, ay - spacing)],
        'column': [(ax, ay), (ax - spacing, ay), (ax - 2.0 * spacing, ay)],
        'v': [(ax, ay), (ax - spacing, ay + 0.75 * spacing),
              (ax - spacing, ay - 0.75 * spacing)],
    }
    # Legacy triangle/diamond → nearest distinct shape.
    if formation in ('triangle', 'wedge'):
        formation = 'v'
    elif formation == 'diamond':
        formation = 'column'
    poses = inits.get(formation, inits['v'])

    map_nodes, _pose = map_stack(
        'dense_field',
        seed=seed,
        planner='homemade',
        map_extra={
            'start_x': ax,
            'start_y': ay,
            'goal_x': 36.0,
            'goal_y': 12.0,
            'clearance_radius': 0.55,
            'add_boundary_walls': False,
        },
    )

    actions = list(map_nodes)
    actions.extend(_drone('uav0', poses[0], peers='uav1,uav2'))
    actions.extend(_drone('uav1', poses[1], peers='uav0,uav2'))
    actions.extend(_drone('uav2', poses[2], peers='uav0,uav1'))

    actions.append(
        send_goal_process(36.0, 12.0, 1.5, delay_sec=4.0, topic='/uav0/drone/goal'))
    actions.append(
        send_goal_process(36.0, 20.0, 1.5, delay_sec=18.0, topic='/uav0/drone/goal'))
    actions.append(
        send_goal_process(20.0, 20.0, 1.5, delay_sec=32.0, topic='/uav0/drone/goal'))
    actions.append(
        send_goal_process(20.0, 12.0, 1.5, delay_sec=46.0, topic='/uav0/drone/goal'))

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
    actions.append(
        rviz_node(condition=IfCondition(use_rviz), config='multi_homemade.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'formation', default_value='v',
            description='line | column | v  (3-UAV demos)'),
        DeclareLaunchArgument('spacing', default_value='1.5'),
        DeclareLaunchArgument('seed', default_value='42'),
        OpaqueFunction(function=launch_setup),
    ])

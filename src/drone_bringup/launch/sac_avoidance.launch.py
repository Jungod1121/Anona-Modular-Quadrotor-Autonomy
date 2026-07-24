from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from drone_bringup.maps_catalog import benchmark_square_waypoints
from drone_bringup.launch_utils import (
    controller_node,
    dynamics_node,
    map_stack,
    resolve_mission_pose,
    rviz_node,
    sac_planner_node,
    safety_supervisor_node,
    send_goal_process,
    square_mission_process,
    square_speed_params,
    visualization_node,
)

# Maps that need slower flight + earlier VFH takeover (forest-trained SAC).
_DENSE_MAPS = frozenset({
    'dense_field', 'dense_asymmetric', 'sparse', 'homemade', 'dense',
})


def _is_dense_map(map_id: str) -> bool:
    mid = (map_id or '').strip().lower()
    return mid in _DENSE_MAPS or mid.startswith('dense')


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    seed = int(LaunchConfiguration('seed').perform(context))
    map_id = LaunchConfiguration('map').perform(context)
    enable_fb = LaunchConfiguration('enable_fallback').perform(context).lower() in (
        '1', 'true', 'yes', 'on')
    mission = LaunchConfiguration('mission').perform(context).strip().lower()
    dense = _is_dense_map(map_id)

    map_nodes, pose = map_stack(map_id, seed=seed, planner='sac')
    pose = resolve_mission_pose(map_id, pose, mission)
    cruise_z = float(pose.get('init_z', 1.5))

    # Forest-trained / under-dense SAC + dense pillars: slow plant, VFH-first.
    if dense:
        ctrl = {
            'use_drone_goal_fallback': False,
            'max_vel': 0.45,
            'goal_slowdown_dist': 3.5,
        }
        sac_extra = {
            'direct_plant': False,
            'action_ema': 0.62,
            'path_horizon_m': 4.5,
            'path_step_m': 0.30,
            'approach_m': 3.2,
            'fallback_clear_m': 0.90,
            'blend_clear_m': 2.2,
        }
        sup_extra = {
            'enable_fallback': enable_fb,
            'fallback_clear_m': 0.90,
            'blend_clear_m': 2.2,
            'path_horizon_m': 4.5,
            'path_step_m': 0.30,
            'safety': 0.42,
            'vfh_prefer_delta_m': 0.0,
            'emergency_clear_m': 0.35,
            'prefer_vfh': True,
            'fallback_hold_ticks': 25,
        }
    else:
        ctrl = {
            'use_drone_goal_fallback': False,
            'max_vel': 0.85,
            'goal_slowdown_dist': 2.5,
        }
        sac_extra = {'direct_plant': False}
        sup_extra = {'enable_fallback': enable_fb}

    ctrl.update(square_speed_params(mission))

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
        controller_node(extra_params=ctrl),
        # Pure SAC on /planner/sac_*; supervisor publishes plant contract.
        sac_planner_node(cruise_z=cruise_z, extra_params=sac_extra),
        safety_supervisor_node(cruise_z=cruise_z, extra_params=sup_extra),
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
            description='Catalog map (dense_field uses slower + earlier VFH profile)'),
        DeclareLaunchArgument(
            'enable_fallback', default_value='true',
            description='Adapter-level VFH safety supervisor'),
        DeclareLaunchArgument(
            'mission', default_value='catalog',
            description='catalog = single catalog goal; square = map-specific square'),
        OpaqueFunction(function=launch_setup),
    ])

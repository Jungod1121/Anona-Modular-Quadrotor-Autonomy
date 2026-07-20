"""Unified planner + map switch: homemade | ego | gcopter × MAPS.md ids.

Keeps per-path launches intact; forwards map:=… / seed / use_rviz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from drone_bringup.maps_catalog import DEFAULT_MAP_BY_PLANNER, normalize_map_id


def launch_setup(context, *args, **kwargs):
    planner = LaunchConfiguration('planner').perform(context).strip().lower()
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    map_raw = LaunchConfiguration('map').perform(context)

    share = get_package_share_directory('drone_bringup')
    mapping = {
        'homemade': 'avoidance.launch.py',
        'a': 'avoidance.launch.py',
        'path_a': 'avoidance.launch.py',
        'ego': 'ego_avoidance.launch.py',
        'b': 'ego_avoidance.launch.py',
        'path_b': 'ego_avoidance.launch.py',
        'gcopter': 'gcopter_avoidance.launch.py',
        'c': 'gcopter_avoidance.launch.py',
        'path_c': 'gcopter_avoidance.launch.py',
        'minco': 'gcopter_avoidance.launch.py',
        'fuel_explore': 'fuel_explore.launch.py',
        'fuel': 'fuel_explore.launch.py',
        'd': 'fuel_explore.launch.py',
        'path_d': 'fuel_explore.launch.py',
        'mighty': 'mighty_avoidance.launch.py',
        'e': 'mighty_avoidance.launch.py',
        'path_e': 'mighty_avoidance.launch.py',
        'fast_planner': 'fast_planner_avoidance.launch.py',
        'fast': 'fast_planner_avoidance.launch.py',
        'f': 'fast_planner_avoidance.launch.py',
        'path_f': 'fast_planner_avoidance.launch.py',
        'rl': 'rl_avoidance.launch.py',
        'ppo': 'rl_avoidance.launch.py',
        'mappo': 'rl_avoidance.launch.py',
        'g': 'rl_avoidance.launch.py',
        'path_g': 'rl_avoidance.launch.py',
        'vfh': 'rl_avoidance.launch.py',
        'sac': 'sac_avoidance.launch.py',
        'h': 'sac_avoidance.launch.py',
        'path_h': 'sac_avoidance.launch.py',
        'drq_sac': 'sac_avoidance.launch.py',
    }
    planner_key = {
        'homemade': 'homemade', 'a': 'homemade', 'path_a': 'homemade',
        'ego': 'ego', 'b': 'ego', 'path_b': 'ego',
        'gcopter': 'gcopter', 'c': 'gcopter', 'path_c': 'gcopter', 'minco': 'gcopter',
        'fuel_explore': 'fuel_explore', 'fuel': 'fuel_explore',
        'd': 'fuel_explore', 'path_d': 'fuel_explore',
        'mighty': 'mighty', 'e': 'mighty', 'path_e': 'mighty',
        'fast_planner': 'fast_planner', 'fast': 'fast_planner',
        'f': 'fast_planner', 'path_f': 'fast_planner',
        'rl': 'rl', 'ppo': 'rl', 'mappo': 'rl', 'g': 'rl', 'path_g': 'rl', 'vfh': 'rl',
        'sac': 'sac', 'h': 'sac', 'path_h': 'sac', 'drq_sac': 'sac',
    }.get(planner)
    launch_file = mapping.get(planner)
    if launch_file is None or planner_key is None:
        raise RuntimeError(
            f"Unknown planner='{planner}'. Use homemade|ego|gcopter|fuel_explore|"
            f"mighty|fast_planner|rl|vfh|sac")

    # Resolve auto/default aliases for pass-through clarity.
    if not map_raw.strip() or map_raw.strip().lower() in ('auto', 'default'):
        map_id = DEFAULT_MAP_BY_PLANNER[planner_key]
    else:
        map_id = normalize_map_id(map_raw, planner=planner_key)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file)),
            launch_arguments={
                'use_rviz': use_rviz,
                'seed': seed,
                'map': map_id,
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner', default_value='homemade',
            description='Planner backend: homemade | ego | gcopter | fuel_explore | mighty | fast_planner | rl'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument(
            'map', default_value='auto',
            description='Map id (auto = planner default). See MAPS.md'),
        OpaqueFunction(function=launch_setup),
    ])

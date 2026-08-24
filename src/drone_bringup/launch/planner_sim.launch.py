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
from drone_bringup.planner_registry import PLANNERS, normalize_planner_id


def launch_setup(context, *args, **kwargs):
    planner = LaunchConfiguration('planner').perform(context).strip().lower()
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    map_raw = LaunchConfiguration('map').perform(context)
    mission = LaunchConfiguration('mission').perform(context)

    share = get_package_share_directory('drone_bringup')
    # Single alias source: planner_registry (PLANNERS.md stays in sync).
    meta = PLANNERS.get(normalize_planner_id(planner))
    if meta is None:
        raise RuntimeError(
            f"Unknown planner='{planner}'. Use homemade|ego|gcopter|fuel_explore|"
            f"mighty|fast_planner|rl|vfh|sac")
    launch_file = meta['launch']
    canon = meta['id']
    # Map-default table and the included rl launches key off the legacy 'rl'.
    map_key = 'rl' if canon == 'vfh' else canon
    pass_through = 'rl' if canon == 'vfh' else canon

    # Resolve auto/default aliases for pass-through clarity.
    if not map_raw.strip() or map_raw.strip().lower() in ('auto', 'default'):
        map_id = DEFAULT_MAP_BY_PLANNER[map_key]
    else:
        map_id = normalize_map_id(map_raw, planner=map_key)

    launch_args = {
        'use_rviz': use_rviz,
        'seed': seed,
        'map': map_id,
    }
    # Exploration Path D has no catalog/square goal switch.
    if canon != 'fuel_explore':
        launch_args['mission'] = mission

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file)),
            launch_arguments=launch_args.items(),
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
        DeclareLaunchArgument(
            'mission', default_value='catalog',
            description='catalog = single catalog goal; square = map-specific square'),
        OpaqueFunction(function=launch_setup),
    ])

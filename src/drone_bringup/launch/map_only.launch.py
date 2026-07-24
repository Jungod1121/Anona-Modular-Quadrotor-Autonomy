#!/usr/bin/env python3
"""Map-only stack for top-down rendering / debugging.

  ros2 launch drone_bringup map_only.launch.py map:=official_forest
  ros2 launch drone_bringup map_only.launch.py map:=dense_field seed:=42
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from drone_bringup.launch_utils import map_stack
from drone_bringup.maps_catalog import normalize_map_id


def _setup(context, *args, **kwargs):
    map_id = normalize_map_id(LaunchConfiguration('map').perform(context))
    seed = int(float(LaunchConfiguration('seed').perform(context) or '1'))
    nodes, _pose = map_stack(map_id, seed=seed)
    return list(nodes)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='official_forest'),
        DeclareLaunchArgument('seed', default_value='1'),
        OpaqueFunction(function=_setup),
    ])

# GCOPTER Path C vendor notes

Source: `yuwei-wu/GCOPTER`, branch `ros2` (MIT), based on ZJU FAST-Lab GCOPTER/MINCO.

Workspace adaptations:

- Removed the external OMPL dependency and replaced only the route frontend with
  a small 2D grid A* over GCOPTER's voxel map.
- Kept GCOPTER's FIRI safe-flight-corridor generation, MINCO trajectory
  optimization, flatness mapping, and visualization.
- Added `/drone/odom` as the initial state.
- Added direct outputs on `/planner/local_goal`,
  `/planner/trajectory_cmd`, and `/planner/trajectory`.
- Uses the official EGO `map_generator/global_cloud`.
- Does not include or run an SO3 controller or simulator.

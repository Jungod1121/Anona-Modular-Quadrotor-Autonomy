# Planner backends (Path A / B / C)

All planners drive the **same plant**. Do not change controller topic names per
backend — only remap inputs and bridge outputs.

## Contract (stable)

| Direction | Topic | Type |
|-----------|--------|------|
| in | `/drone/odom` | `nav_msgs/Odometry` |
| in | `/drone/goal` | `geometry_msgs/PoseStamped` |
| in | obstacle cloud | `sensor_msgs/PointCloud2` |
| out | `/planner/local_goal` | `geometry_msgs/PoseStamped` |
| out | `/planner/trajectory_cmd` | `drone_msgs/TrajectoryCommand` (p/v/a/yaw) |
| out | `/planner/trajectory` | `nav_msgs/Path` (RViz yellow) |

**Goals:** use RViz **2D Goal Pose** (official EGO does the same). Message z from
RViz is usually ~0 — Path B uses `fsm/cruise_height` (default 1.0), Path A
`cruise_z`, Path C `CruiseHeight`. Yellow planned path is `/planner/trajectory`
(blue flown path is `/drone/path`). EGO markers use frame `map`.

Maps differ only by topic (bridged so both names always exist):

- Homemade generators: `/map/obstacles` from `drone_map`
- Official EGO generators: `/map_generator/global_cloud` from `random_forest` / `mockamap`

Full map catalog: [`MAPS.md`](MAPS.md).

## Switch

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=homemade
ros2 launch drone_bringup planner_sim.launch.py planner:=ego map:=official_forest
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter map:=official_maze2d
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter map:=dense_field
```

Or call each launch directly: `avoidance.launch.py`, `ego_avoidance.launch.py`,
`gcopter_avoidance.launch.py` (both accept `map:=…`).

Web UI (planner + map Start/Stop):

```bash
ros2 run drone_bringup dashboard   # http://127.0.0.1:8765/
```

Multi-drone (EGO-Swarm core): see [`SWARM.md`](SWARM.md).

```bash
ros2 launch drone_bringup ego_swarm.launch.py num_drones:=2 map:=official_forest
```

## Backends

| ID | Package | Notes |
|----|---------|--------|
| homemade | `drone_planner` | Self-developed EGO-style; publishes contract natively |
| ego | `ego_vendor/ego_planner` | Official EGO; [`ego_cmd_bridge`](drone_bringup/ego_cmd_bridge.py) converts `PositionCommand` |
| gcopter | `gcopter_vendor/gcopter` | MINCO/GCOPTER; node publishes contract directly (no SO3 plant) |

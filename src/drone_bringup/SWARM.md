# EGO-Swarm multi-drone (Path B)

Official **EGO-Swarm** planner cores (`broadcast_bspline` + `drone_id` +
`swarm_clearance`) drive N custom plants. No SO3 / fake_drone / local_sensing.

```bash
ros2 launch drone_bringup ego_swarm.launch.py num_drones:=2 map:=official_forest
ros2 launch drone_bringup ego_swarm.launch.py num_drones:=10 map:=official_forest seed:=1 use_rviz:=false
ros2 launch drone_bringup ego_swarm.launch.py num_drones:=20 map:=official_forest use_rviz:=false
```

Dashboard → **Multi** mode → `EGO-Swarm`.

## Architecture

| Piece | Role |
|-------|------|
| Shared map (`map_stack`) | One cloud → `/map_generator/global_cloud` (+ bridge) |
| `uav{i}` plant | `drone_dynamics` + `drone_controller` + `viz` |
| `ego_planner_node` | Official swarm planner, `manager/drone_id:=i` |
| `/broadcast_bspline` | Inter-drone trajectory sharing (EGO-Swarm core) |
| `traj_server` | → `/drone_{i}_planning/pos_cmd` |
| `ego_cmd_bridge` | → `/uav{i}/planner/local_goal` + `trajectory_cmd` |

Default mission: **crossing lanes** (`fsm/flight_type:=2` presets), same idea as
vendor `swarm.launch.py` / `swarm_large.launch.py`, with our plant (2–20 drones).

## Homemade multi (kept)

Still available and also on the dashboard Multi pane:

| Launch | Planner | Avoidance |
|--------|---------|-----------|
| `shared_field.launch.py` | homemade | odom peer keep-out |
| `formation.launch.py` | homemade + coordinator | peer keep-out |

## Notes

- Cap `num_drones` at 20 in this launch. Large N is CPU-heavy (full plant × N); prefer RViz off on the dashboard.
- RViz `ego_swarm.rviz`: gray `ForestCloud` + per-drone rainbow inflate
  (`InflatedOcc0`…`InflatedOcc19` on `/drone_{i}_grid/grid_map/occupancy_inflate`,
  same as official EGO warehouse `default.rviz`) plus body/path/optimal for
  `uav0`…`uav19`.
- Prefer the **web dashboard** (Multi → EGO-Swarm). After ~8s all UAVs get goals at once.
- Planning is **CPU-only** (GPU does not accelerate this stack).
- Startup uses EGO `SEQUENTIAL_START` + `/drone_{i}_planning/swarm_trajs` chaining.
- Interactive goals: FSM listens to remappable relative `goal` **and**
  `/move_base_simple/goal` (single RViz).
- Sources: `src/ego_vendor/ego_planner` (symlink to reference swarm packages).

# MIGHTY Path E vendor notes

Pins (cloned 2026-07-15):

- mighty: `0c2e9d772997cfef1ea56743b45a01ef5ad00a7e` ([mit-acl/mighty](https://github.com/mit-acl/mighty))
- dynus_interfaces / DecompROS2: see parent clone notes

## Integration strategy in drone_ws

**Path E launch uses upstream `mighty` node** (not the thin `drone_mighty` adapter).

Plant wiring (`mighty_avoidance.launch.py`):

1. `convert_odom_to_state` — `/drone/odom` → `state` (ns `NX01`)
2. `mighty` (`/NX01/mighty_node`) — `sim_env:=fake_sim`, remaps:
   - `sensor_point_cloud` ← `/map_generator/global_cloud`
   - `term_goal` ← `/drone/goal`
3. `mighty_cmd_bridge` — `/NX01/goal` + `/NX01/trajectory` → `/planner/*`

**Params:** do **not** pass `config/mighty.yaml` as a params-file under
`namespace:=NX01` — keys under `mighty_node:` only match `/mighty_node`.
Launch flattens `mighty_node.ros__parameters` into a dict (same as
`onboard_mighty.launch.py`). Otherwise declare-defaults win
(`fopt_threshold=0.1`, `max_iterations=30`) and every replan hits
`Local Optimization Failed` with L-BFGS status `1001` (max iterations).

Humble adaptations in `mighty/`:

- Plant-only CMake (no Gazebo)
- `tf2_geometry_msgs.hpp`
- Local stubs for slim `pcl_ros` / include paths as needed

Adapter package `drone_mighty` remains available for offline experiments but is
not the default Path E entry.

# Fast-Planner Path F vendor notes

Source pin: RohitPawar2406/Fast-Planner-ROS2 `1600fc8f42675f7e5840a0ff8920a38069725ec9`
(Foxy-era port of HKUST Fast-Planner). Original `LiHaojie07/fast_planner_ros2` URL is 404.

## Integration strategy (upstream on plant)

Ament package names that collide with `ego_vendor` are renamed under this tree:

| Upstream name | Workspace package |
|---|---|
| plan_env | `fp_plan_env` |
| path_searching | `fp_path_searching` |
| bspline_opt | `fp_bspline_opt` |
| traj_utils | `fp_traj_utils` |

Unique: `poly_traj`, `bspline`, `fast_planner`, `plan_manage`.

### Build notes

- Eigen 3.4: `polynomial_traj` jerk-matrix loops use `int` indices (not `double`).
- NLopt: build into `third_party/nlopt_install` (no sudo); CMake and launch prepend that prefix.
- `quadrotor_msgs/msg/Bspline.msg` added under `ego_vendor/quadrotor_msgs` for plan_manage ↔ traj_server.

### Plant wiring (`fast_planner_avoidance.launch.py`)

1. `plan_manage/fast_planner_node` — odom `/drone/odom`, cloud `/map_generator/global_cloud`
2. `plan_manage/traj_server` → `/planning/pos_cmd` (`PositionCommand`)
3. `pose_to_path_goal` — `/drone/goal` → `/waypoint_generator/waypoints`
4. `ego_cmd_bridge` — `PositionCommand` → `/planner/local_goal` + `/planner/trajectory_cmd`

Plant adaptations in `kino_replan_fsm.cpp`: when a local B-spline finishes but
odometry is still farther than `fsm/thresh_no_replan` from the goal, regenerate
(`GEN_NEW_TRAJ`) instead of dropping to `WAIT_TARGET` (upstream assumes a
tighter SO3 tracker). Planner `max_vel` is capped near the cascade PID limit.

`sdf_map/map_size_*` must cover the scenario when the grid is origin-centered.
Do **not** set size to GCOPTER `MapBound` width alone (`xmax-xmin`); for asymmetric
homemade maps (narrow corridor goal at x≈17) that truncates the eastern half.
Convert absolute corners with `size = 2 * max(|min|, |max|)` (see launch).

Cloud QoS must be `transient_local`+`reliable` to match latched
`/map_generator/global_cloud`. Default volatile never receives the map → empty
SDF → straight line through walls. Also cache the cloud until odom arrives.

`virtual_ceil_height` must be applied on the **cloud** ingest path (not only depth
raycast). Without it, kinodynamic search climbs over tall homemade walls instead
of threading door slits. Narrow corridor uses ceil≈2.6 m (cruise 1.5, walls 4 m).
Also seal a **virtual floor** near z=0 — otherwise B-splines dive and REPLAN
restarts from underground starts (logged as `start z≈-2`).

Gate walls must be **solid-filled** in the point cloud (thickness ≥~0.3 m). Face-
only sampling of 0.18 m slabs leaves holes at SDF res 0.15 → straight flight
through the wall at y=mid.

Adapter package `drone_fast_planner` is retained but no longer the default Path F launch.

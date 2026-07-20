# RViz visualization

Canonical config: [`rviz/drone.rviz`](rviz/drone.rviz) (installed with `drone_visualization`).

Launches pass it via `drone_bringup` `rviz_node()` when `use_rviz:=true`.

## Required displays (contract topics)

| Display | Plugin | Topic | Notes |
|--------|--------|-------|--------|
| Obstacles | PointCloud2 | `/map/obstacles` | Path A global cloud |
| Actual path | Path | `/drone/path` | Blue flown trajectory |
| Planned path | Path | `/planner/trajectory` | Yellow planner output |
| Local goal | Marker | `/planner/local_goal_marker` | Rolling local goal arrow |
| Mission goal | Pose | `/drone/goal` | RViz 2D Goal Pose target |
| Drone body | MarkerArray | `/drone/body_markers` | Optional but default-on |
| Odometry | Odometry | `/drone/odom` | Pose arrow |
| TF | TF | — | `map` → `base_link` |

**Fixed frame:** `map` (ENU). **Goal tool:** 2D Goal Pose publishes to `/drone/goal`.

Path B / EGO-Swarm inflate uses **rainbow by height** (AxisColor Z) in
`ego_avoidance.rviz` / `ego_swarm.rviz` on
`/drone_{i}_grid/grid_map/occupancy_inflate` — matches paper/demo “rainbow highland”
look (warehouse `default.rviz` is solid blue FlatColor instead). See [`RVIZ_COLOR.md`](RVIZ_COLOR.md).

## Per-scenario configs

| Launch family | RViz file |
|---------------|-----------|
| Default / Path A / RL | `drone.rviz` |
| Path B EGO | `ego_avoidance.rviz` |
| Path C GCOPTER | `gcopter_avoidance.rviz` |
| Path F Fast-Planner | `fast_planner_avoidance.rviz` (solid markers + outlines; not EGO gray cloud) |
| EGO-Swarm | `ego_swarm.rviz` |

## Manual setup

If `drone.rviz` fails to load after an RViz upgrade, recreate the required displays
above in RViz2, set Fixed Frame to `map`, and save. Path colors in the canonical file:
actual = blue `(30,180,255)`, planned = yellow `(255,200,40)`.

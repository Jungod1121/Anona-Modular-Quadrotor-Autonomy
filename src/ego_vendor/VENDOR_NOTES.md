# ego_vendor — vendoring notes

Path B (EGO-Planner) stack, vendored from
[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm).

## Provenance

| Item | Value |
|---|---|
| Upstream commit | `23a8d5a191711dd65633df689bd00f55d4dea8f9` ("Update Readme.md") |
| Vendored on | 2026-08-25 |
| Method | Byte-faithful copy (`cp -a`) of the referenced subdirectories; verified with `diff -rq` against the upstream checkout |

All eight subpackages are **real directories in this repository** (no
symlinks into `reference_repos/`). Earlier the tree was symlinked and an
upstream re-clone silently wiped `quadrotor_msgs/msg/Bspline.msg`,
breaking clean rebuilds of `fast_planner_vendor/plan_manage` — that class
of surprise is what pinning the copies here prevents.

Update procedure: bump the upstream checkout under `reference_repos/`,
re-diff each subpackage, review the delta (especially anything touching
message definitions), copy over, and refresh this table.

## Subpackages

| Directory | Upstream path | Notes |
|---|---|---|
| `bspline_opt` | `src/planner/bspline_opt` | B-spline optimizer |
| `path_searching` | `src/planner/path_searching` | A*/kinodynamic search |
| `plan_env` | `src/planner/plan_env` | Grid map / obstacle env |
| `traj_utils` | `src/planner/traj_utils` | Trajectory msgs + visualization |
| `ego_planner` | `src/planner/plan_manage` | FSM + planner node (renamed from upstream `plan_manage` to avoid collisions) |
| `cmake_utils` | `src/uav_simulator/Utils/cmake_utils` | CMake helpers consumed by the above |
| `map_generator` | standalone ROS 2 port | Latched global cloud for the plant |
| `mockamap` | standalone ROS 2 port | mockamap maze2D / perlin maps |
| `quadrotor_msgs` | `src/uav_simulator/Utils/quadrotor_msgs` | + restored `Bspline.msg` (see fast_planner_vendor/VENDOR_NOTES.md) |

## Local modifications vs upstream

- `map_generator`, `mockamap`, `quadrotor_msgs`: full ROS 2 ports maintained in-tree.
- `random_forest_sensing.cpp`: stamps the latched global cloud with the node
  clock; unreachable local-cloud dead code removed.
- `ego_replan_fsm.cpp`: odometry twist rotated body->world (REP-105; the
  plant publishes body-frame twist and the kinodynamic search seeds plus
  fail-safe velocity check need world-frame velocity).
- Everything under `bspline_opt` / `path_searching` / `plan_env` /
  `traj_utils` / `cmake_utils`: currently byte-identical to upstream `23a8d5a`.

## Known upstream limitation (follow-up)

Under sustained CPU load EGO's rebound optimizer can exhaust escape
directions near forest corners ("Failed to generate direction") and plan a
degenerate trajectory; plant-side guards (controller z-fence,
bridge runaway-command filter, extended sensing horizon) contain this but
scenario 4 remains load-sensitive. Proper fix belongs in EGO's
checkCollisionAndRebound/optimizer — tracked as follow-up work.

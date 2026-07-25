# GCOPTER Path C vendor notes

Source: [`yuwei-wu/GCOPTER`](https://github.com/yuwei-wu/GCOPTER), branch `ros2` (MIT),
based on ZJU FAST-Lab GCOPTER/MINCO.

**Upstream pin (refresh reference):** `b625819c1913ae3e71f705ba3696bb739ff4b881`
(`setup ros2`, shallow clone 2026-07-15). Upstream still depends on **OMPL**
InformedRRT*; this tree keeps the workspace adaptations below instead of a
raw mirror.

## Workspace adaptations

- Removed the external OMPL dependency and replaced only the route frontend with
  a 3D grid A* over GCOPTER's voxel map (`include/gcopter/sfc_gen.hpp`).
  Fail-closed: if A* finds no free path, do **not** fall back to a straight
  start→goal segment (that previously made maze3d fly through plates).
- Kept GCOPTER's FIRI safe-flight-corridor generation, MINCO trajectory
  optimization, flatness mapping, and visualization (headers match upstream
  except `sfc_gen.hpp`).
- Added `/drone/odom` as the initial state.
- Added direct outputs on `/planner/local_goal`,
  `/planner/trajectory_cmd`, and `/planner/trajectory`.
- Default map topic: `/map_generator/global_cloud` (EGO / mockamap bridged).
- Does not include or run an SO3 controller or simulator.

## Refresh policy

When syncing from upstream `ros2`:

1. Diff `include/gcopter/{minco,gcopter,firi,flatness,voxel_*}.hpp` and pull
   non-conflicting bugfixes.
2. Never reintroduce OMPL into `sfc_gen.hpp` / `CMakeLists.txt`.
3. Keep plant publishers and `OdomTopic` in `src/global_planning.cpp`.

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

---

## Deep-audit findings register (2026-08-25)

Three parallel audits covered `mighty_node.cpp` (full 4030 L), the planner core
(`mighty.cpp`, `lbfgs_solver.cpp` full), and the HGP/frontier stack. ~60 findings;
the actionable-and-safe subset is fixed (below). Everything else is listed here so
it is not re-discovered — fix upstream-style, one topic per PR.

### Fixed in-tree

| Where | Fix |
|---|---|
| `hgp/hgp_planner.cpp` | `GraphSearch::plan()` return value now respected (`!search_ok || path.size()<2` fails) — unreachable goals no longer masquerade as plans; empty-map guard returns `false` not `-1` |
| `hgp/hgp_manager.cpp` + `map_util.hpp` | `map_initialized_` flag actually set after `readMap()` (free-space helpers were permanent no-ops); public `setMapInitialized()` added |
| `mighty_node.cpp` | `stateCallback` moved off the Reentrant group onto its own MutuallyExclusive group (was racing itself over `actual_traj_hist_` at 100 Hz) |
| `mighty_node.cpp` | benchmark CSV missing comma between columns 13/14 |
| `lbfgs_solver.cpp` | safe-path builder pushed the same matrix twice via `std::move` (second push received an empty plane set) |

### Deferred — threading (mighty_node)

- `frontier_manager_` called from three callback groups though it documents
  "same-group only" (replan: `markInvalidated`, goal-reached Reentrant:
  `markVisited`, occ2D map group: iterate+update). Containment: add an internal
  mutex to FrontierManager, or route all calls through `cb_group_map_`.
- Exploration/command flags (`exploration_active_`, `manual_goal_active_`,
  `current_explore_id_`, ...) are plain members mutated from four groups.
- `esdf_grid_` shared_ptr reassigned on the map group while read at 100 Hz on
  the replan group (torn-read class).
- Function-local `debug_log_stream()` ofstream written from >=3 groups.

### Deferred — planner core

- Homogeneous transform applied to *every* PWP coefficient when
  `use_hardware && provide_goal_in_global_frame` (adds t*(1+u+u²+u³) instead of
  translation once) — mighty.cpp:2074-2118.
- lbfgs line-search failure leaves `fx` = rejected trial while `x = xp`
  (inconsistent pair consumed by fopt gating) — lbfgs.hpp:570-579.
- Published plan truncates up to one `dc` short of trajectory end with forced
  zero terminal derivatives ("BUG #1", self-documented) — lbfgs_solver.cpp:1516.
- Dynamic-obstacle cost samples Quintic obstacles beyond their horizon
  (unbounded polynomial extrapolation) — lbfgs_solver.cpp:2532-2620.
- Initial-guess time inf/NaN on zero-length seams / U-turn apex (V_min_=0);
  LDLT NaN kills the whole replan — lbfgs_solver.cpp:1326-1385.
- `parameters` struct has ~40 uninitialized PODs consumed straight from YAML.
- Map cloud pointers passed to `hgp_manager_.updateMap` after releasing their
  mutex (mighty.cpp:1953-2006).
- Hot-loop churn: per-obstacle/per-sample vectors, mem_size=256 dense buffers
  allocated per optimize() call.

### Deferred — HGP/frontier

- Partial-path recovery reports success when the open set empties (goal
  provably unreachable) — graph_search.cpp:194-199. Deliberate exploration
  behaviour; needs product decision before changing.
- 2D-mode search clobbers its projected map pointer with the 3D voxel array
  mid-search — graph_search.cpp:169.
- Soft-cost pass-through grants zero-cost obstacle traversal for `sastar` /
  heat_weight=0 — graph_search.cpp:593-650.
- Weighted A* never reopens closed nodes (heuristic eps>1 is inconsistent).
- World<->grid conversion uses three different rounding conventions
  (map_util.hpp floatToInt vs readMap vs legacy read_map.hpp).
- `w_align` / `decay_len_cells` / `w_side` params accepted but never wired into
  GraphSearch.
- Per-replan deep copy of VoxelMapUtil + O(map) hm_/seen_ resets every plan().
- Frontier manager: greedy matching can emit duplicate overlapping goals;
  records on UNKNOWN cells stay ACTIVE until pursuit timeout.
- hgp/utils.cpp sphere helpers index at -1 / uninitialized on degenerate input;
  `color(int)` falls off its switch for BLACK_TRANS; two-cloud pclptr_to_vec
  writes through a reserve-only vector (dead code, hazardous).

### Deferred — node-level

- `communication_delay` computed against unstamped peer headers (~1e9 s);
  feature currently inert (no consumer).
- `use_benchmark_` pushes a 15-tuple per replan tick and rewrites the whole
  CSV each call (unbounded growth ~360k rows/h).
- Stale `manual_goal_active_` after rejected user goals disables exploration
  until the next successful goal.
- `traj.bbox[0..2]` indexed unchecked (unbounded float32[] in DynTraj.msg).
- HW TF lookup hardcodes frame "map"; actual_traj markers hardcode "map".
- Dead members/includes: cloud_callback_mutex_, p_points_, start_yaw param,
  pub_p_points_, getNextGoal() side effect evaluated before cheap guard.

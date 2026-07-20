# Planner backends (Path A–H)

Canonical registry: [`drone_bringup/planner_registry.py`](drone_bringup/planner_registry.py)
(dashboard, launches, batch matrix).

All planners drive the **same plant** (`drone_dynamics` + `drone_controller`). Do not
rename controller topics per backend — only remap planner inputs and bridge outputs.

## Contract (stable)

| Direction | Topic | Type |
|-----------|--------|------|
| in | `/drone/odom` | `nav_msgs/Odometry` |
| in | `/drone/goal` | `geometry_msgs/PoseStamped` |
| in | obstacle cloud | `sensor_msgs/PointCloud2` (`/map/obstacles` or bridged `/map_generator/global_cloud`) |
| out | `/planner/local_goal` | `geometry_msgs/PoseStamped` |
| out | `/planner/trajectory_cmd` | `drone_msgs/TrajectoryCommand` (p/v/a/yaw) |
| out | `/planner/trajectory` | `nav_msgs/Path` (RViz yellow) |
| out | `/planner/local_goal_marker` | `visualization_msgs/Marker` (rolling local goal) |

**Goals:** RViz **2D Goal Pose** → `/drone/goal` (official EGO does the same). RViz z is
usually ~0 — Path B uses `fsm/cruise_height` (default 1.0), Path A `cruise_z`, Path C
`CruiseHeight`. Yellow planned path is `/planner/trajectory`; blue flown path is
`/drone/path`.

Maps differ only by topic (bridged so both names exist when `map_adapter` runs):

- Homemade generators: `/map/obstacles` from `drone_map`
- Official EGO generators: `/map_generator/global_cloud` from `random_forest` / `mockamap`

Full map catalog (including four-tier presets): [`MAPS.md`](MAPS.md).

## Classification

| Class | Meaning | In fair strong/weak matrix? |
|-------|---------|----------------------------|
| **weak** | Classical / reactive baseline | yes |
| **strong** | Optimization or learning planner | yes |
| **mode** | Mission FSM using another planner as backend | no |
| **optional** | Lineage benchmark, not in canonical matrix | no |

`canonical_comparison_ids()` = active planners with class `weak` or `strong` only
(homemade, ego, gcopter, mighty, vfh, sac).

## Switch (`planner_sim.launch.py`)

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=homemade
ros2 launch drone_bringup planner_sim.launch.py planner:=ego map:=official_forest
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter map:=official_maze2d
ros2 launch drone_bringup planner_sim.launch.py planner:=fuel_explore map:=narrow_corridor
ros2 launch drone_bringup planner_sim.launch.py planner:=mighty map:=official_forest
ros2 launch drone_bringup planner_sim.launch.py planner:=fast_planner map:=official_forest
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=dense_field
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
```

Or call each launch directly: `avoidance.launch.py`, `ego_avoidance.launch.py`,
`gcopter_avoidance.launch.py`, `fuel_explore.launch.py`, `mighty_avoidance.launch.py`,
`fast_planner_avoidance.launch.py`, `rl_avoidance.launch.py`, `sac_avoidance.launch.py`
(all accept `map:=…`).

Web UI (planner + map Start/Stop):

```bash
ros2 run drone_bringup dashboard   # http://127.0.0.1:8765/
```

Multi-drone (EGO-Swarm core): see [`SWARM.md`](SWARM.md).

```bash
ros2 launch drone_bringup ego_swarm.launch.py num_drones:=2 map:=official_forest
```

## Backends

| ID | Path | Class | Package | Launch | Aliases |
|----|------|-------|---------|--------|---------|
| `homemade` | A | weak | `drone_planner` | `avoidance.launch.py` | `a`, `path_a`, `dynastar` |
| `ego` | B | strong | `ego_vendor/ego_planner` | `ego_avoidance.launch.py` | `b`, `path_b` |
| `gcopter` | C | strong | `gcopter_vendor/gcopter` | `gcopter_avoidance.launch.py` | `c`, `path_c`, `minco` |
| `fuel_explore` | D | **mode** | `drone_exploration` + EGO | `fuel_explore.launch.py` | `d`, `path_d`, `fuel` |
| `mighty` | E | strong | `mighty` (`mighty_vendor`) | `mighty_avoidance.launch.py` | `e`, `path_e` |
| `fast_planner` | F | **optional** | `plan_manage` (`fast_planner_vendor`) | `fast_planner_avoidance.launch.py` | `f`, `path_f`, `fast` |
| `vfh` | G | weak | `drone_rl_planner` | `rl_avoidance.launch.py` | `g`, `path_g`, `rl`, `ppo`, `mappo` |
| `sac` | H | strong | `drone_rl_planner` | `sac_avoidance.launch.py` | `h`, `path_h`, `drq_sac` |

### Path A — `homemade` (weak)

Dyn-A* + B-spline on an occupancy grid; auto-fits grid/inflate from the obstacle cloud.
Optional local mapping via `local_mapping_enable` (default off).

### Path B — `ego` (strong)

EGO-Planner rebound B-spline; [`ego_cmd_bridge`](drone_bringup/ego_cmd_bridge.py) converts
`PositionCommand` → plant contract. Official-style local sensing
(`local_sense_cloud`, horizon 5 m) crops the **same** global map cloud for
`grid_map` — not a different map. RViz: gray ForestCloud + gray occupancy +
rainbow inflate (`ego_avoidance.rviz`).

### Path C — `gcopter` (strong)

MINCO/GCOPTER corridor optimization (`yuwei-wu/GCOPTER@ros2` pin); publishes contract
directly (no SO3 plant).

### Mode D — `fuel_explore` (mode, not a standalone planner)

Frontier exploration **mission mode**: fog sensing + frontier FSM triggered by
`/drone/goal`; sequential nav goals feed **Path B (EGO)** for trajectories. Not upstream
FUEL warehouse stack — see [`MAPS.md`](MAPS.md).

### Path E — `mighty` (strong)

MIGHTY HGP / Hermite-LBFGS; `mighty_cmd_bridge` + upstream `mit-acl/mighty` (`fake_sim`
+ shared plant).

### Optional F — `fast_planner` (optional lineage)

Fast-Planner `kino_replan` benchmark in the EGO/Fast-Planner lineage. Optional
lineage对照 — excluded from the canonical strong/weak comparison matrix. Uses
`ego_cmd_bridge`; colliding vendor packages are prefixed `fp_*`.

### Path G — `vfh` (weak)

Classical **VFH+** polar histogram reactive avoidance → smooth yellow path on
`/planner/trajectory`. Optional PPO backend via `backend:=rl` in `rl_avoidance.launch.py`.
Registry id is `vfh`; `planner:=rl` remains a compatibility alias.

### Path H — `sac` (strong)

Polar occupancy image + **DrQ-SAC** policy rolls a Bézier path. Plant contract is
published by an external **`safety_supervisor_node`** (adapter-level VFH fallback), not
directly by the SAC node (`direct_plant:=false` in `sac_avoidance.launch.py`).

```bash
ros2 launch drone_bringup sac_avoidance.launch.py enable_fallback:=true
```

## Rates (fair comparison)

From `planner_registry.RATES`: dynamics 500 Hz integration, state 100 Hz, control
100 Hz; per-planner publish rates in registry (`homemade` 10 Hz, `ego`/`mighty` 50 Hz,
`vfh`/`sac` 20 Hz, etc.).

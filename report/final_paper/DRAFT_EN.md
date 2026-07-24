# Report depth rewrite — English working draft

Terminology: *plant* = dynamics + controller layer. Chinese: **动力学与控制层**. Never 「植物层」.

---

## Abstract (EN → to be translated)

Quadrotor autonomy stacks often couple rigid-body dynamics, low-level control, and motion planning in one repository tree. Swapping a planner then forces changes to the execution chain and evaluation scripts, which undermines fair comparison. This report describes a modular ROS 2 Humble simulation platform (engineering name Anona, codebase `drone_ws`) that keeps a fixed dynamics-and-control layer—self-developed X-layout rigid-body dynamics with cascade PID and mixer—and attaches Path A–H planning backends through a single topic contract. Obstacle maps are procedural point clouds from homemade generators or official EGO-lineage generators, unified by a map adapter. Six one-click acceptance scenarios and a planner×map square-mission benchmark share the same plant rates and safety definitions.

Relative to the two repositories designated by the supervisor—pengyu_sim and MARSIM—the dynamics and controller are implemented independently in ROS 2. Design ideas such as RPM-to-wrench allocation and asymmetric motor lag were cross-checked conceptually; neither codebase is executed or thinly wrapped. Official planners (EGO, GCOPTER, MIGHTY, Fast-Planner) are vendored or bridged so that SO3 / fake_drone plants are not used as the shared plant.

Acceptance runs report 6/6 PASS. The seven-planner × two-map matrix shows large performance gaps under staged corner-dwell success. Path H (polar DrQ-SAC) does not count as policy success on `dense_field` (FAIL under the delivery convention; FALLBACK to VFH safety supervision must not be read as pure SAC success). The single-UAV demo loop uses random forest (`forest.gif`), not dense field. The body gives full dynamics/control equations, map catalog, per-planner origin/principles/category/integration, experiments, and an explicit open-source relationship chapter.

**Keywords:** ROS 2; quadrotor dynamics; cascade control; motion planning; simulation maps; open-source comparison; obstacle avoidance

---

## Maps (summary for ZH chapter)

All maps: procedural point clouds, no PCD scene files. `map_adapter` republishes `/map/obstacles` and `/map_generator/global_cloud`, plus occupancy slice and metadata.

### Official EGO resources (`ego_vendor/map_generator`, `mockamap`)
- official_forest: random_forest cylinders + yawed rings; 26×20×3; start outside box
- official_perlin: mockamap type=1 Perlin blobs; 50×26×5
- official_posts: mockamap type=2 hollow pillars; 20×20×4
- official_maze2d: recursive-division walls; road_width 1.2
- official_maze3d: Voronoi 3D maze; Z≥0 patch

### Homemade (`drone_map`)
- dense_field: dense cylinders/spheres + walls
- sparse: open field
- narrow_corridor: S-bend doors
- ego_maze2d_port / ego_forest_port: ports into homemade frame

### Tier presets
- tier_simple_open ← sparse; tier_medium_corridor ← narrow; tier_complex_forest ← official_forest; tier_extreme_maze ← official_maze2d

### Used in this report
- Acceptance: sparse/hover goals, avoidance=EGO+official_forest, narrow, stability
- Benchmark: official_forest + dense_field
- Demo GIF: forest, not dense

---

## Path A–H (four blocks each)

### A homemade — weak; grid search + optional B-spline
1. Source: in-house `drone_planner`; B-spline/A* ideas referenced from EGO lineage, rewritten; NOT ego launch wrapper.
2. Principles: inflate occupancy from cloud → Dyn-A* / Grid A* at cruise → optional shortcut → optional B-spline (often off in dense) → local_goal + trajectory.
3. Category: classical global/local search on grid; weak baseline in fair matrix.
4. Integration: direct plant contract; `homemade_avoidance.launch.py`; default map dense_field.

### B ego — strong; ESDF-free rebound B-spline local planner
1. Source: https://github.com/ZJU-FAST-Lab/ego-planner-swarm (ROS2); paper EGO-Planner RA-L; evolution of Fast-Planner lineage.
2. Principles: ESDF-free gradient optimization on B-spline; rebound when colliding; ~ms planning; local sensing crops global cloud.
3. Category: optimization-based local replanning; strong.
4. Integration: `ego_vendor` + `ego_cmd_bridge` (PositionCommand→trajectory_cmd); no SO3 plant; acceptance scenario 4.

### C gcopter — strong; MINCO / geometrically constrained traj opt
1. Source: ZJU-FAST-Lab/GCOPTER; pin yuwei-wu/GCOPTER@ros2.
2. Principles: sparse MINCO polynomial + corridor / geometric constraints; L-BFGS; T-RO paper.
3. Category: global/local trajectory optimization; strong.
4. Integration: `gcopter_vendor`; publishes contract directly; OMPL→voxel A* patches; DilateRadius tuned.

### D fuel_explore — mode
1. Source: FUEL-style only; not upstream ZJU-FAST-Lab/FUEL warehouse stack.
2. Principles: fog + frontier FSM → sequential nav goals → EGO trajectories.
3. Category: exploration mission mode (not a planner algorithm class in matrix).
4. Integration: `drone_exploration` + Path B plant bridge.

### E mighty — strong; Hermite spline HGP
1. Source: https://github.com/mit-acl/mighty ; RA-L / arXiv 2511.10822.
2. Principles: Hermite-spline efficient traj planning; LBFGS-family; multi-agent capable upstream.
3. Category: trajectory optimization; strong.
4. Integration: `mighty_vendor` + `mighty_cmd_bridge` + odom→state; shared plant instead of fake_sim alone.

### F fast_planner — optional lineage
1. Source: HKUST Fast-Planner; ROS2 pin via community fork; packages prefixed `fp_*`.
2. Principles: kinodynamic replanning (kino_replan) in Fast-Planner/EGO ancestry.
3. Category: optional lineage benchmark; not in fair matrix.
4. Integration: `fast_planner_vendor` + `ego_cmd_bridge`.

### G vfh — weak; VFH+
1. Source: classical VFH+ (Ulrich et al. lineage); implemented in `drone_rl_planner`; no vendor tree.
2. Principles: polar histogram → clear sector toward goal → short polyline → track local_goal.
3. Category: reactive local avoidance; weak.
4. Integration: `vfh_planner_node`; optional PPO train card.

### H sac — strong; polar DrQ-SAC
1. Source: method refs denisyarats/drq, facebookresearch/drqv2; SACPlanner paper only (no full train repo); code in `drone_rl_planner`.
2. Principles: polar occupancy image → DrQ-SAC policy → Bézier rollouts; n-step=3, shared encoder.
3. Category: learning-based local planning; strong.
4. Integration: `safety_supervisor_node` publishes plant + VFH fallback; dense_field staged policy FAIL; demo uses forest.

---

## Open-source relations (supervisor point 7)

### Designated
- pengyu_sim (gitee.com/potato77/pengyu_sim): conceptual RPM→wrench, asymmetric motor τ, viz organization; independent rewrite.
- MARSIM (github.com/hku-mars/MARSIM): system organization / sensing ideas; do NOT wrap mars_drone_sim.
Rationale: fixed shared plant for fair A/B; supervisor forbids shell wrapping.

### Planner matrix — see ZH tables
### Maps: official generators vendored vs drone_map rewritten; adapter unifies topics
### Layout: arxiv-cjk-preprint fonts/header only

---
name: review-led architecture upgrade
overview: "Apply a review-gated upgrade: independently improve or rewrite the ROS2 plant where justified by comparison with pengyu_sim/MARSIM, consolidate the adapter contract, reduce the active planner set to orthogonal families, add standardized maps/evaluation/visualization, upgrade the web dashboard, and preserve all six acceptance scenarios."
todos:
  - id: baseline-audit
    content: Rebuild current tree, snapshot interfaces, and rerun the six acceptance scenarios before edits
    status: completed
  - id: plant-tests
    content: Compare dynamics/controller with pengyu_sim and MARSIM, then selectively improve or independently rewrite proven gaps with full formula/interface tests
    status: completed
  - id: adapter-contract
    content: Consolidate universal planner/map/plant adapters, registry, timestamps, parameters, and process lifecycle
    status: completed
  - id: planner-orthogonality
    content: Reclassify/remove overlapping active planners and externalize SAC→VFH safety supervision
    status: completed
  - id: map-dataset
    content: Add bounded four-tier reproducible maps and standardized cloud/occupancy/metadata outputs
    status: completed
  - id: evaluation-framework
    content: Implement complete metrics, diagnostics, batch matrix, pairwise comparison, and ablation runners
    status: completed
  - id: visualization-docs
    content: Unify RViz presentation and update README plus planner/map architecture documentation
    status: completed
  - id: web-dashboard
    content: Redesign and extend the web dashboard for planner control, telemetry, diagnostics, training, experiments, and results
    status: completed
  - id: final-regression
    content: Run full tests and six-scenario acceptance, then archive report-ready evidence
    status: completed
isProject: false
---

# Review-led UAV Simulation and Multi-Planner Upgrade

## Review verdict and non-negotiable baseline
- The current [`src/drone_dynamics`](#/home/jungod/drone_ws/src/drone_dynamics) and [`src/drone_controller`](#/home/jungod/drone_ws/src/drone_controller) are largely compliant, but they are **not frozen**. Compare their architecture, equations, parameters, and node boundaries against [pengyu_sim](https://gitee.com/potato77/pengyu_sim) and [MARSIM](https://github.com/hku-mars/MARSIM); modify, reorganize, or independently rewrite any part when the review shows a correctness, clarity, maintainability, or validation gap.
- Any rewritten plant code must remain an original ROS2 implementation: no direct source copy, runtime wrapping, shell execution, ROS1 compatibility shim, fake MAVROS layer, or use of either repository's dynamics/controller binary. The required 6-DoF equations, RPM interface, cascaded PID pipeline, and saturation behavior remain acceptance constraints.
- Treat the existing `6/6` report in [`report/acceptance_report.md`](#/home/jungod/drone_ws/report/acceptance_report.md) as historical, not current: first rebuild and rerun the six scenarios because the working tree has substantial newer uncommitted changes. Preserve all current user work; do not reset or overwrite unrelated files.
- Keep the qualified foundations unchanged where possible: [`maps_catalog.py`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/maps_catalog.py), `map_stack()`/plant factories in [`launch_utils.py`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/launch_utils.py), [`cloud_bridge.py`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/cloud_bridge.py), the native plant messages, and controller goal arbitration.

## 1. Review and selectively improve the plant
- Build a source-to-design comparison for pengyu_sim, MARSIM, and this project: state definition, motor model, thrust/moment allocation, integration, controller loops, ROS node separation, timing, noise/disturbance handling, and visualization. Record what is referenced conceptually and what is independently implemented.
- Add focused tests for motor first-order response, `F=k_Fω²`, X-allocation signs, rotational gyroscopic term, quaternion norm, RPM limits, PID acceleration/tilt limits, and mixer round trip in [`src/drone_dynamics/test`](#/home/jungod/drone_ws/src/drone_dynamics/test) and [`src/drone_controller/test`](#/home/jungod/drone_ws/src/drone_controller/test).
- Add ROS interface smoke assertions for `/drone/motor_rpm_cmd`, `/drone/odom`, `/drone/imu`, `/tf`, `/drone/path`, and `/drone/goal`; document evidence that no pengyu_sim/MARSIM dynamics or controller source is executed or wrapped.
- Retain compliant code, but allow partial refactoring or a clean-room rewrite where evidence justifies it. A replacement must preserve the specified topics/messages, full physical equations, cascaded PID architecture, and six-scenario behavior; compare old/new outputs before removing the old implementation.

## 2. Make `drone_bringup` the universal adapter layer
- Add a small contract/registry layer under [`src/drone_bringup/drone_bringup`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup): canonical topics, frame/QoS rules, planner capabilities, strong/weak class, publish rate, timeout, and active/experimental status.
- Consolidate triplicated EGO parameter builders and per-planner controller timeout/fallback settings into shared helpers; keep per-path launches thin.
- Normalize bridge semantics: ensure `TrajectoryCommand.trajectory_ready` is correct in [`mighty_cmd_bridge.py`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/mighty_cmd_bridge.py), make local-goal-only planners conform through a tested adapter, preserve source timestamps by default, and remove the source-tree `_python_node_process` fallback once installed ROS nodes are reliable.
- Replace broad `pkill` lifecycle handling with process-group ownership and fix stale process names in [`scripts/smoke_maps_planners.py`](#/home/jungod/drone_ws/scripts/smoke_maps_planners.py).

## 3. Enforce an orthogonal active planner matrix
- Canonical weak planners: Path A as pure Grid A* deterministic search; Path G renamed `vfh` as pure reactive histogram. Preserve `rl`, `g`, and old path aliases for compatibility.
- Canonical strong planners: EGO rebound B-spline, GCOPTER/MINCO corridor optimization, MIGHTY HGP/Hermite-LBFGS, and Polar DrQ-SAC learning.
- Reclassify `fuel_explore` as a mission/exploration mode using EGO, not an independent trajectory planner. Keep swarm/formation under multi-agent modes only.
- Remove Path F/Fast-Planner from the canonical comparison because it is an EGO/Fast-Planner lineage derivative; retain it only as an explicitly optional lineage benchmark if required. Delete the unused duplicate adapter packages [`src/drone_fast_planner`](#/home/jungod/drone_ws/src/drone_fast_planner) and [`src/drone_mighty`](#/home/jungod/drone_ws/src/drone_mighty) after a reference scan proves no production launch uses them.
- Separate Path H’s SAC solver from VFH: extract VFH into one shared core used by Path G and an adapter-level safety supervisor. Path H remains a pure SAC planner; the supervisor performs the required abnormal-condition switch and publishes fallback diagnostics.

## 4. Standardize and expand the map dataset without breaking legacy scenarios
- Preserve existing acceptance map IDs and geometry unchanged. Add dataset metadata (`bounds`, `difficulty`, `seed`, obstacle family, safety radius) to the catalog and create simple/medium/complex/extreme presets as new entries.
- Extend the map adapter to publish canonical `/map/obstacles` plus `/map/occupancy` and map bounds/metadata. Add physical boundary clouds to every new dataset map, including official/vendor generators, at the adapter boundary rather than patching vendor algorithms.
- Add larger tiled/stretched forest, dense-field, corridor, and maze variants with asymmetric clutter, local occlusion, and no guaranteed empty center lane; verify fixed-seed reproducibility and start-goal connectivity.
- Add map unit tests for deterministic seeds, bounds, closed walls, gate width, connectivity, and difficulty metadata.

## 5. Build fair evaluation and batch comparison
- Extend [`evaluate.py`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/evaluate.py) to record odometry, planned path/command, planner diagnostics, and fallback events, then compute collision rate, path smoothness/jerk, planned-vs-flown tracking error, detour ratio, failure rate, solve/replan timing, and fallback trigger rate.
- Add a `PlannerDiagnostics` message rather than overloading status strings; preserve all existing `PlannerStatus` fields and topics.
- Upgrade [`scripts/run_acceptance.py`](#/home/jungod/drone_ws/scripts/run_acceptance.py) with trajectory-overlap and hold-at-goal checks while keeping the original six pass criteria. Add a separate batch matrix runner for planner × map tier × seed, pairwise strong/weak comparisons, and SAC-with/without-shield ablation.
- Centralize experiment rates (dynamics 500 Hz, state/control 100 Hz, planner observation rate) and record them in every result manifest for fair comparisons.

## 6. Unify visualization and technical documentation
- Standardize `/planner/trajectory`, `/planner/local_goal_marker`, `/drone/path`, goal, obstacles, status, and fallback visualization in one canonical RViz configuration; keep vendor-specific debug displays optional.
- Correct stale A–F documentation in [`README.md`](#/home/jungod/drone_ws/README.md), update [`PLANNERS.md`](#/home/jungod/drone_ws/src/drone_bringup/PLANNERS.md) and [`MAPS.md`](#/home/jungod/drone_ws/src/drone_bringup/MAPS.md), and document the independent ROS2 relationship to pengyu_sim/MARSIM. Do **not** create an AI-development document.

## 7. Improve the web dashboard design and functionality
- Review the existing static UI and server APIs in [`dashboard_static`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/dashboard_static) and [`dashboard_server.py`](#/home/jungod/drone_ws/src/drone_bringup/drone_bringup/dashboard_server.py); retain working behavior and remove duplicated planner/map metadata by rendering from the canonical registry.
- Redesign the information hierarchy for desktop and smaller screens: clear run-state prominence, strong/weak/mode/experimental planner grouping, map difficulty and seed controls, compact live telemetry, explicit warnings, and accessible keyboard/focus/contrast/reduced-motion behavior.
- Add functional panels for planner diagnostics and fallback state, training progress, batch experiment setup, live acceptance/evaluation progress, result history, metric plots, artifact download/open actions, and safe start/stop/restart feedback.
- Keep motion restrained, responsive, and interruptible; use system typography and translucent depth only where it clarifies hierarchy. Preserve existing API compatibility while consolidating endpoints where duplication is proven.
- Add server/API tests and browser-level smoke coverage for planner selection, map selection, process lifecycle, goal publication, training state, evaluation state, error display, and refresh/reconnect behavior.

## 8. Regression gates and delivery evidence
- After each phase, run unit tests, launch/interface smoke tests, and affected planner-map smoke cases; stop and repair regressions before continuing.
- At completion, rerun all six minimum scenarios headless and archive current CSV/JSON/plots/logs. Require hover/goal hold ≤0.3 m, obstacle clearance > configured safety distance, no target orbiting, and unchanged topic/message contracts.
- Produce a reproducible comparison manifest and report-ready plots for strong vs weak, traditional vs learning, four map difficulties, and fallback ablation; clearly separate canonical planners, mission modes, multi-agent modes, and optional lineage benchmarks.
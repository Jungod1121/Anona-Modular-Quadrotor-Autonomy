# Final regression evidence (2026-07-16)

## Completed upgrade phases
1. Baseline audit — rebuild + interface snapshot; historical 6/6 stale; first re-run 2/6 (ground/DDS issues).
2. Plant — asymmetric motor τ, ground friction, optional clamps, RPM slew; expanded gtests (allocation signs, motor step, free-fall, IMU, closed-loop settle). Provenance: comments only vs pengyu_sim/MARSIM; no wrapping.
3. Adapter — `planner_registry.py`, mighty `trajectory_ready`, cleanup expanded, process factories for SAC supervisor.
4. Planner orthogonality — shared `vfh_core.py`; Path H pure SAC + `safety_supervisor_node`; fuel_explore = mode; fast_planner = optional; `COLCON_IGNORE` on unused `drone_fast_planner` / `drone_mighty`.
5. Maps — difficulty metadata + tier presets + `map_adapter_node` (occupancy/metadata/boundaries).
6. Evaluation — `PlannerDiagnostics.msg`, extended evaluate metrics, hold-at-goal, `run_batch_matrix.py`.
7. Docs/viz — README/PLANNERS/MAPS/VISUALIZATION; no `ai_usage.md`.
8. Web dashboard — registry-grouped UI, telemetry, diagnostics, reports API, Apple-style chrome.

## How to rebuild
```bash
export PYTHONNOUSERSITE=1
export PYTHONPATH=/usr/lib/python3/dist-packages
source /opt/ros/humble/setup.bash
cd ~/drone_ws && colcon build --symlink-install
source install/setup.bash
```

## Unit gates (pre-acceptance)
- drone_dynamics: allocation + closed_loop PASS
- drone_controller: mixer PASS
- maps_catalog: 6/6 PASS
- batch matrix dry-run: 6 cells written to report/batch_matrix/manifest.json

## Acceptance
**6/6 PASS** (2026-07-16 23:56 CST). Log: `report/baseline/acceptance_final.log`.
Report: `report/acceptance_report.md`. All hold-at-goal supplementary checks True.

# Baseline audit (2026-07-16)

## Build
- Core packages rebuilt with `PYTHONNOUSERSITE=1` (setuptools 83 breaks ament `--uninstall`).
- Unit tests: drone_dynamics 4/4, drone_controller 4/4 PASS.

## Acceptance re-run
- Result: **2/6** (scenarios 5 narrow_passage, 6 stability PASS; 1–4 FAIL).
- Hover symptom: `pz` stuck at 0, `|xy|` grows to hundreds of meters; evaluate loaded ~96k obstacle points while `map_sparse` published 0 — indicates graph pollution / ground-trap under tilt.
- Historical report (2026-07-14) was 6/6; treat as stale.

## Artifacts
- `interface_snapshot.md`, `plant_vs_refs.md`, `acceptance_rerun.log`, `acceptance_report_baseline.md`

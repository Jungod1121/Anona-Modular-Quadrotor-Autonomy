# Seven-planner comparative benchmark

- Updated: `2026-07-24T17:56:32+08:00`
- Matrix: `7 planners × 2 maps = 14 cases`
- Completed latest cases: `14/14`
- Maps: `Random Forest`, `Dense Obstacle Field`
- **Single independent runs merge into this report** by planner×map key (they replace only that cell; other cells stay).

## Metric definitions

- **Mission**: map-specific closed square (Random Forest ≈16×12 m; Dense Field ≈23×10 m).
- **Max speed (square mission)**: controller/planner capped at **0.65 m/s** (acc 1.1 m/s²) to improve tracking and clearance.
- **Success (staged square)**: visit corners **in order**; at each corner dwell ≥ **1.0 s** inside the waypoint tolerance with mean speed ≤ **1.20 m/s** during that window (stage gate — brief skims do not count). The return corner (spawn) only counts after prior stages. After the last dwell, remain within **3.0 m** of home and finish inside the waypoint tolerance — post-lap flyaways fail.
- **Time to goal**: elapsed time from first 0.15 m displacement to completion of the final corner dwell; launch delay is excluded.
- **Path efficiency**: ideal square perimeter divided by flown distance (higher is better).
- **Safety**: minimum obstacle-cloud clearance ≥ **0.08 m** (benchmark floor; catalog/acceptance radii 0.35–0.40 m are stricter and fail almost all dense square flights).
- **Smoothness**: mean finite-difference jerk during active travel (lower is better).
- **Overall score**: arrival 30%, clearance 20%, final accuracy 15%, path efficiency 15%, time 10%, smoothness 10%.

## Aggregate comparison charts

![overview](charts/overview.png)

![success and safety](charts/success_safety_heatmap.png)

## Per-trial trajectory charts

One figure per planner×map trial (obstacles overlaid when `obstacles.csv` is present).

![Path A — Grid A* + B-spline × Random Forest](charts/trials/homemade__official_forest.png)

![Path A — Grid A* + B-spline × Dense Obstacle Field](charts/trials/homemade__dense_field.png)

![Path B — EGO rebound B-spline × Random Forest](charts/trials/ego__official_forest.png)

![Path B — EGO rebound B-spline × Dense Obstacle Field](charts/trials/ego__dense_field.png)

![Path C — GCOPTER / MINCO × Random Forest](charts/trials/gcopter__official_forest.png)

![Path C — GCOPTER / MINCO × Dense Obstacle Field](charts/trials/gcopter__dense_field.png)

![Path E — MIGHTY HGP × Random Forest](charts/trials/mighty__official_forest.png)

![Path E — MIGHTY HGP × Dense Obstacle Field](charts/trials/mighty__dense_field.png)

![Optional F — Fast-Planner kino (lineage) × Random Forest](charts/trials/fast_planner__official_forest.png)

![Optional F — Fast-Planner kino (lineage) × Dense Obstacle Field](charts/trials/fast_planner__dense_field.png)

![Path G — VFH+ histogram × Random Forest](charts/trials/vfh__official_forest.png)

![Path G — VFH+ histogram × Dense Obstacle Field](charts/trials/vfh__dense_field.png)

![Path H — Polar DrQ-SAC × Random Forest](charts/trials/sac__official_forest.png)

![Path H — Polar DrQ-SAC × Dense Obstacle Field](charts/trials/sac__dense_field.png)

## Overall ranking

| Rank | Planner | Cases | Success | Safety | Score | Avg time [s] | Efficiency | Avg clearance [m] |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | Path A — Grid A* + B-spline | 2/2 | 50% | 50% | 91.6 | 40.51 | 1.000 | 0.192 |
| 2 | Path C — GCOPTER / MINCO | 2/2 | 50% | 100% | 51.8 | 124.55 | 0.530 | 0.176 |
| 3 | Path H — Polar DrQ-SAC | 2/2 | 50% | 50% | 48.5 | 53.02 | 0.480 | 0.103 |
| 4 | Optional F — Fast-Planner kino (lineage) | 2/2 | 0% | 100% | 28.4 | — | 0.517 | 0.231 |
| 5 | Path G — VFH+ histogram | 2/2 | 0% | 0% | 25.4 | — | 0.670 | 0.056 |
| 6 | Path B — EGO rebound B-spline | 2/2 | 0% | 50% | 20.8 | — | 0.284 | 0.080 |
| 7 | Path E — MIGHTY HGP | 2/2 | 0% | 50% | 18.1 | — | 0.383 | 0.061 |

## Per-case results

| Planner | Map | Status | Success | Safety | Score | Time [s] | Final error [m] | Clearance [m] | Path [m] | Efficiency | Mean speed [m/s] | Jerk [m/s³] |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Path A — Grid A* + B-spline | Random Forest | fail | FAIL | FAIL | — | — | — | — | — | — | — | — |
| Path A — Grid A* + B-spline | Dense Obstacle Field | ok | PASS | PASS | 91.6 | 40.51 | 0.002 | 0.192 | 41.16 | 1.000 | 1.02 | 52.60 |
| Path B — EGO rebound B-spline | Random Forest | ok | FAIL | PASS | 19.3 | — | 247.158 | 0.090 | 746.68 | 0.075 | 4.35 | 22.45 |
| Path B — EGO rebound B-spline | Dense Obstacle Field | ok | FAIL | FAIL | 22.4 | — | 0.001 | 0.070 | 85.28 | 0.492 | 0.50 | 19.70 |
| Path C — GCOPTER / MINCO | Random Forest | ok | PASS | PASS | 86.2 | 124.55 | 0.066 | 0.256 | 56.19 | 0.997 | 0.45 | 30.79 |
| Path C — GCOPTER / MINCO | Dense Obstacle Field | ok | FAIL | PASS | 17.3 | — | 116.103 | 0.096 | 653.00 | 0.064 | 4.13 | 254.47 |
| Path E — MIGHTY HGP | Random Forest | ok | FAIL | FAIL | 7.3 | — | 122.522 | 0.030 | 598.48 | 0.094 | 3.53 | 110.44 |
| Path E — MIGHTY HGP | Dense Obstacle Field | ok | FAIL | PASS | 29.0 | — | 3.926 | 0.093 | 62.53 | 0.672 | 0.34 | 19.19 |
| Optional F — Fast-Planner kino (lineage) | Random Forest | ok | FAIL | PASS | 35.8 | — | 16.879 | 0.281 | 46.06 | 1.000 | 0.29 | 111.41 |
| Optional F — Fast-Planner kino (lineage) | Dense Obstacle Field | ok | FAIL | PASS | 20.9 | — | 916.484 | 0.181 | 1206.21 | 0.035 | 7.24 | 235.93 |
| Path G — VFH+ histogram | Random Forest | ok | FAIL | FAIL | 27.6 | — | 0.831 | 0.063 | 89.12 | 0.628 | 0.51 | 2.97 |
| Path G — VFH+ histogram | Dense Obstacle Field | ok | FAIL | FAIL | 23.2 | — | 0.816 | 0.050 | 59.08 | 0.711 | 0.34 | 13.14 |
| Path H — Polar DrQ-SAC | Random Forest | ok | FAIL | FAIL | 9.6 | — | 916.064 | 0.043 | 1043.72 | 0.054 | 5.92 | 52.07 |
| Path H — Polar DrQ-SAC | Dense Obstacle Field | ok | PASS | PASS | 87.3 | 53.02 | 0.381 | 0.163 | 46.33 | 0.907 | 0.88 | 11.74 |

## Artifacts

- `latest_results.json`: latest result for every planner/map cell (single-cell runs upsert one key and keep the rest)
- `comparison_results.csv`: flat comparison table
- `history.jsonl`: append-only history of every trial
- `charts/overview.png` / `charts/success_safety_heatmap.png`: aggregate summary only
- `charts/trials/<planner>__<map>.png`: one trajectory figure per trial
- `runs/<planner>__<map>/<timestamp>/`: raw CSV, logs, summaries, `trajectory.png`, `obstacles.csv`

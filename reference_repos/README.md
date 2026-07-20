# Reference repositories (not built by colcon)

All trees here stay behind `COLCON_IGNORE` (folder + optional per-repo ignore).
Never `colcon build` these packages; copy ideas into `src/` only.

- `ego-planner-swarm/` — official `ros2_version` (algorithm reference for drone_planner / Path B).
- `GCOPTER/` — upstream reference for Path C (buildable copy lives under `src/gcopter_vendor/`).
- `FUEL/` — HKUST [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) ROS1 tree (read-only). Ideas for Path D FUEL-style exploration (`drone_exploration`). **Do not** ship `uav_simulator` / SO3 plant.

External (not linked here to avoid colcon package discovery):
- `/home/jungod/reference_repos/MARSIM`
- `/home/jungod/reference_repos/pengyu_sim`
- `/home/jungod/drone_sim/reference_repos/EGO-Planner` (older local copy)

See `../notes/reference_repos_notes.md`.

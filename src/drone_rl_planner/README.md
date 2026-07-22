# Anona RL planners — Path G (VFH) & Path H (Polar DrQ-SAC)

**English** · [中文](README_zh.md)

Local planners for Anona backends **G** and **H**. Both honor the plant contract
(`/planner/local_goal`, `/planner/trajectory`, `/planner/status`). Training is
independent of a live sim and can be started from the Mission Console train card.

## Path H — Polar DrQ-SAC

Learns heading / lookahead / speed from a **polar occupancy image**, rolls a short
Bézier path, and falls back to an external **`safety_supervisor_node`** (VFH) when
clearance is unsafe.

Algo notes (from open-source **DrQ / DrQ-v2** code, not SACPlanner — paper only):
shared encoder, actor on detached features, DrQ K-aug Q targets, **n-step=3**.

### Run

```bash
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
```

### Curriculum train

Prefer curriculum over jumping straight to full dense:

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH

python3 -m drone_rl_planner.train_sac_polar --fast --device cuda          # easy
bash src/drone_rl_planner/checkpoints/launch_curriculum.sh 2             # medium
bash src/drone_rl_planner/checkpoints/launch_curriculum.sh 4             # 15% dense mix
python3 -m drone_rl_planner.train_sac_polar --stage5 --device cuda       # 30% dense mix
python3 -m drone_rl_planner.train_sac_polar --stage6 --device cuda       # 50% dense mix
```

Auto ramp (stage5 → stage6) + decline guard:

```bash
bash src/drone_rl_planner/checkpoints/sac_mix_ramp_supervisor.sh
```

Live GUI monitor:

```bash
bash src/drone_rl_planner/checkpoints/launch_train_monitor.sh
# or: python3 -m drone_rl_planner.train_sac_monitor
```

**Latest stage numbers:** [`CURRICULUM_RESULTS.md`](CURRICULUM_RESULTS.md).

| Knob | Notes |
|------|--------|
| `--mix-mid-dense` / `--mix-dense-p` | Per-episode medium↔dense mix |
| `--nstep` | DrQ-v2 n-step (default 3) |
| `--reset-buffer` | Fresh replay on domain shift |
| `--eval-episodes` | Prefer ≥50–60 |

Artifacts (gitignored): `checkpoints/sac_polar_*_best.pt`, `*_replay/`,
`sac_training_status.json`, train logs.

### Direct dense (hard)

```bash
python3 -m drone_rl_planner.train_sac_polar \
  --steps 2000000 --eval-every 5000 --eval-episodes 60 \
  --device cuda --dense-heavy --target 0.80 \
  --resume src/drone_rl_planner/checkpoints/sac_polar_mixd_best.pt \
  --reset-buffer --finetune-lr 1e-4
```

Full dense needs **much longer** budgets than easy/medium (order of **millions** of
env steps). Short runs often plateau ~50–65%.

## Path G — VFH+ / optional PPO

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=dense_field
PYTHONPATH=src/drone_rl_planner \
  python3 -m drone_rl_planner.train_sb3_ppo --steps 500000 --target 0.95 --n-envs 8
```

## Mission Console

Single-UAV → Path **G** or **H** → train card (Start / Stop / success / steps /
checkpoint). Status from `training_status.json` (G) or `sac_training_status.json` (H).

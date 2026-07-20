# drone_rl_planner — Path G (VFH) + Path H (Polar DrQ-SAC)

Local planners for `drone_bringup` Paths **G** and **H**. Both publish the plant
contract (`/planner/local_goal`, `/planner/trajectory`, `/planner/status`) used by
the cascade-PID stack. Training is **independent** of a live sim run and can be
started from the Flight Deck dashboard when Path G or H is selected.

## Path H — Polar DrQ-SAC

Learns heading / lookahead / speed from a **polar occupancy image**, rolls a short
Bézier path (yellow), and uses an external **`safety_supervisor_node`** (VFH) when
clearance is unsafe.

### Run (sim)

```bash
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
# or: ros2 launch drone_bringup sac_avoidance.launch.py
# Dashboard: 单机 → Path H → 密集场 → 启动
```

On **`dense_field`**, the launch profile slows the plant (~0.45 m/s) and prefers
VFH earlier (sticky hold) so a forest-biased policy does not chatter into obstacles.

### Train (dense catalog density)

Training env density matches `drone_map` DENSE_FIELD ≈ `80 / (24×14 m²)` pillars
(not the old sparse 75/30 m gym). Eval uses **≥60 episodes** and a score that
**penalizes collisions** (raw success% alone is misleading).

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH

# Fresh dense-heavy train (recommended defaults on CUDA)
python3 -m drone_rl_planner.train_sac_polar \
  --steps 1000000 --eval-every 5000 --eval-episodes 60 \
  --device cuda --dense-heavy --target 0.95 \
  --batch-size 128 --updates-per-step 2 --n-envs 2

# Continue from best (overnight / dashboard “开始训练”)
python3 -m drone_rl_planner.train_sac_polar \
  --steps 800000 --dense-heavy --target 0.95 --device cuda \
  --resume src/drone_rl_planner/checkpoints/sac_polar_local_best.pt
```

| Knob | Recommended | Notes |
|------|-------------|--------|
| `--dense-heavy` | on for Path H dense | Catalog-like pillar density |
| `--target` | `0.95` | Early-stop when best success ≥ target |
| `--eval-episodes` | `60` | 25 eps was too noisy |
| `--batch-size` | `128` | Larger (256) + many updates/step slows **env-steps/hour** |
| `--updates-per-step` | `2` | `12` saturates GPU but ~8× fewer env steps/hour |
| `--n-envs` | `2` | Parallel CPU rollouts |

Checkpoints (gitignored `*.pt`):

- `checkpoints/sac_polar_local_best.pt` — best by eval **score**
- `checkpoints/sac_polar_local.pt` — latest
- `checkpoints/sac_training_status.json` — dashboard progress

**Resume safety:** when `--resume` is set, historical `best_success` / `best_score`
are preserved so a weaker mid-chunk eval cannot overwrite a stronger `*_best.pt`.

Overnight helper (optional):

```bash
bash src/drone_rl_planner/checkpoints/launch_overnight.sh
# Logs: checkpoints/sac_overnight_supervisor.log
# Default: target 95%, continue chunks of 800k until target or max rounds
```

| Component | Detail |
|-----------|--------|
| Algo | Soft Actor-Critic + DrQ (GPU random-shift) |
| Obs | Polar image `(2,16,36)` + vector `(8,)` |
| Action | heading offset, lookahead, speed → Bézier |
| Safety | `safety_supervisor_node` VFH fallback |
| Plant match | max_vel / max_acc / velocity lag ≈ cascade PID |

## Path G — VFH+ (classical) / optional PPO

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=dense_field
# alias: planner:=rl

# Optional research PPO (dashboard Path G train card)
PYTHONPATH=src/drone_rl_planner \
  python3 -m drone_rl_planner.train_sb3_ppo --steps 500000 --target 0.95 --n-envs 8
```

| Component | Entry |
|-----------|--------|
| Default | `vfh_planner_node` |
| Optional PPO | `train_sb3_ppo` → `sb3_ppo_local.zip` |
| Path H | `sac_planner_node` + `train_sac_polar` + `safety_supervisor_node` |

## Dashboard

Flight Deck → **单机** → select Path **G** or **H** → train card docks under the
mission panel (Start / Stop / success / steps / checkpoint). Status is polled from
`training_status.json` (G) or `sac_training_status.json` (H).

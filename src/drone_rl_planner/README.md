# Anona RL planners — Path G (VFH) & Path H (Polar DrQ-SAC)

**English** · [中文](#中文)

Local planners for Anona backends **G** and **H**. Both honor the plant contract
(`/planner/local_goal`, `/planner/trajectory`, `/planner/status`). Training is
independent of a live sim and can be started from the Mission Console train card.

<!-- Optional: ![Path H dense](../../docs/media/dense-field.gif) -->
<!-- Optional: ![SAC training](../../docs/media/sac-training.gif) -->

## Path H — Polar DrQ-SAC

Learns heading / lookahead / speed from a **polar occupancy image**, rolls a short
Bézier path, and falls back to an external **`safety_supervisor_node`** (VFH) when
clearance is unsafe.

### Run

```bash
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
# Mission Console: Single → Path H → dense_field → Start
```

On **`dense_field`**, launch slows the plant (~0.45 m/s) and prefers VFH earlier
(sticky hold) to reduce chatter into obstacles.

### Train (dense catalog density)

Training density matches `drone_map` DENSE_FIELD ≈ `80 / (24×14 m²)` pillars.
Eval uses **≥60 episodes** and a score that **penalizes collisions**.

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH

python3 -m drone_rl_planner.train_sac_polar \
  --steps 1000000 --eval-every 5000 --eval-episodes 60 \
  --device cuda --dense-heavy --target 0.95 \
  --batch-size 128 --updates-per-step 2 --n-envs 2

# Continue from best
python3 -m drone_rl_planner.train_sac_polar \
  --steps 800000 --dense-heavy --target 0.95 --device cuda \
  --resume src/drone_rl_planner/checkpoints/sac_polar_local_best.pt
```

| Knob | Recommended | Notes |
|------|-------------|--------|
| `--dense-heavy` | on for dense Path H | Catalog-like pillar density |
| `--target` | `0.95` | Early-stop on best success |
| `--eval-episodes` | `60` | 25 eps is too noisy |
| `--batch-size` | `128` | Larger batches can cut env-steps/hour |
| `--updates-per-step` | `2` | `12` fills GPU but slows rollouts |
| `--n-envs` | `2` | Parallel CPU envs |

Artifacts (gitignored): `checkpoints/sac_polar_local_best.pt`, `sac_polar_local.pt`,
`sac_training_status.json`.

**Resume safety:** `--resume` preserves prior `best_success` / `best_score` so a
weaker mid-chunk eval cannot overwrite a stronger `*_best.pt`.

Overnight:

```bash
bash src/drone_rl_planner/checkpoints/launch_overnight.sh
```

| Component | Detail |
|-----------|--------|
| Algo | Soft Actor-Critic + DrQ |
| Obs | Polar `(2,16,36)` + vector `(8,)` |
| Action | heading offset, lookahead, speed → Bézier |
| Safety | VFH supervisor |
| Plant match | max_vel / max_acc / lag ≈ cascade PID |

## Path G — VFH+ / optional PPO

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=dense_field

PYTHONPATH=src/drone_rl_planner \
  python3 -m drone_rl_planner.train_sb3_ppo --steps 500000 --target 0.95 --n-envs 8
```

## Mission Console

Single-UAV → Path **G** or **H** → train card under the mission panel (Start / Stop /
success / steps / checkpoint). Status from `training_status.json` (G) or
`sac_training_status.json` (H).

---

<a id="中文"></a>

# Anona 强化学习规划（中文）

路径 **G**（VFH+）与路径 **H**（极坐标 DrQ-SAC）本地规划包，输出与植物层契约一致。
训练与仿真 Start **解耦**，可在任务控制台训练卡启动。

## 路径 H

- 观测：极坐标占用图 + 向量特征  
- 动作：航向偏置 / 前瞻 / 速度 → 短 Bézier  
- 安全：外置 `safety_supervisor_node`（VFH）  
- 训练：`--dense-heavy` 对齐 catalog 密集场；评测 ≥60 局并惩罚碰撞  
- 续训：`--resume` 保留历史 best，避免弱评测覆盖强权重  

```bash
python3 -m drone_rl_planner.train_sac_polar \
  --steps 1000000 --dense-heavy --target 0.95 --device cuda \
  --batch-size 128 --updates-per-step 2 --n-envs 2
```

推荐吞吐默认：`updates-per-step=2`、`batch=128`、`n-envs=2`（过大更新步数会抬高 GPU% 但降低环境步/小时）。

## 路径 G

默认 `vfh_planner_node`；可选 PPO 训练卡见上文英文命令。

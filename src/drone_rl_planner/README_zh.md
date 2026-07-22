# Anona 强化学习规划器 — 路径 G（VFH）与路径 H（极坐标 DrQ-SAC）

[English](README.md) · **中文**

面向 Anona 后端 **G** 与 **H** 的本地规划器。二者均遵守植物层契约
（`/planner/local_goal`、`/planner/trajectory`、`/planner/status`）。训练与在线
仿真解耦，可从任务控制台的训练卡启动。

## 路径 H — 极坐标 DrQ-SAC

从**极坐标占用图**学习航向偏置 / 前瞻 / 速度，滚动生成短 Bézier 路径；当空隙
不安全时回退到外置 **`safety_supervisor_node`**（VFH）。

算法说明（来自开源 **DrQ / DrQ-v2 代码**，而非 SACPlanner — 后者仅有论文）：
共享 encoder、actor 使用 detach 特征、DrQ 的 K 次增强平均 Q 目标、**n-step=3**。

### 运行

```bash
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
```

### 课程训练

优先使用课程，避免直接跳到满 dense：

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH

python3 -m drone_rl_planner.train_sac_polar --fast --device cuda          # easy
bash src/drone_rl_planner/checkpoints/launch_curriculum.sh 2             # medium
bash src/drone_rl_planner/checkpoints/launch_curriculum.sh 4             # 15% dense 混合
python3 -m drone_rl_planner.train_sac_polar --stage5 --device cuda       # 30% dense 混合
python3 -m drone_rl_planner.train_sac_polar --stage6 --device cuda       # 50% dense 混合
```

自动爬坡（stage5 → stage6）+ 衰退保护：

```bash
bash src/drone_rl_planner/checkpoints/sac_mix_ramp_supervisor.sh
```

实时 GUI 监视器：

```bash
bash src/drone_rl_planner/checkpoints/launch_train_monitor.sh
# 或：python3 -m drone_rl_planner.train_sac_monitor
```

**最新阶段成绩：** [`CURRICULUM_RESULTS_zh.md`](CURRICULUM_RESULTS_zh.md)。

| 参数 | 说明 |
|------|------|
| `--mix-mid-dense` / `--mix-dense-p` | 按 episode 在 medium↔dense 间混合 |
| `--nstep` | DrQ-v2 n-step（默认 3） |
| `--reset-buffer` | 域切换时清空 replay |
| `--eval-episodes` | 建议 ≥50–60 |

产物（已被 gitignore）：`checkpoints/sac_polar_*_best.pt`、`*_replay/`、
`sac_training_status.json`、训练日志。

### 直接满 dense（困难）

```bash
python3 -m drone_rl_planner.train_sac_polar \
  --steps 2000000 --eval-every 5000 --eval-episodes 60 \
  --device cuda --dense-heavy --target 0.80 \
  --resume src/drone_rl_planner/checkpoints/sac_polar_mixd_best.pt \
  --reset-buffer --finetune-lr 1e-4
```

满 dense 需要的预算远大于 easy / medium（量级为**数百万**环境步）。短训往往
会在约 50–65% 平台期徘徊。

## 路径 G — VFH+ / 可选 PPO

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=dense_field
PYTHONPATH=src/drone_rl_planner \
  python3 -m drone_rl_planner.train_sb3_ppo --steps 500000 --target 0.95 --n-envs 8
```

## 任务控制台

单机 → 路径 **G** 或 **H** → 训练卡（开始 / 停止 / 成功率 / 步数 / 检查点）。
状态来自 `training_status.json`（G）或 `sac_training_status.json`（H）。

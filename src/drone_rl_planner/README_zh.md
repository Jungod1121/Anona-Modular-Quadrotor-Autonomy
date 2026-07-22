# Anona 强化学习规划 — 路径 G（VFH）与路径 H（极坐标 DrQ-SAC）

[English](README.md) · **中文**

路径 **G** / **H** 本地规划包，输出符合植物层话题契约。训练与仿真 Start **解耦**，可在任务控制台训练卡启动。

## 路径 H — 极坐标 DrQ-SAC

从极坐标占用图学习航向偏置 / 前瞻 / 速度，滚动短 Bézier；空隙不安全时由外置
`safety_supervisor_node`（VFH）接管。

算法对齐开源 **DrQ / DrQ-v2 代码**（SACPlanner 仅有论文、无完整训练仓库）：共享
encoder、actor 使用 detach 特征、DrQ K 次增强平均 Q、**n-step=3**。

### 仿真运行

```bash
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
```

### 课程训练（推荐）

不要一上来就满 dense：

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH

python3 -m drone_rl_planner.train_sac_polar --fast --device cuda
bash src/drone_rl_planner/checkpoints/launch_curriculum.sh 2   # medium
bash src/drone_rl_planner/checkpoints/launch_curriculum.sh 4   # 15% dense 混合
python3 -m drone_rl_planner.train_sac_polar --stage5 --device cuda
python3 -m drone_rl_planner.train_sac_polar --stage6 --device cuda
```

自动晋级与衰退保护：

```bash
bash src/drone_rl_planner/checkpoints/sac_mix_ramp_supervisor.sh
```

训练面板：

```bash
bash src/drone_rl_planner/checkpoints/launch_train_monitor.sh
```

**当前各阶段成绩：** [`CURRICULUM_RESULTS_zh.md`](CURRICULUM_RESULTS_zh.md)。

### 直接上满 dense（难）

密集场通常需要 **百万级** 环境步；短训很容易在约 50–65% 徘徊。

```bash
python3 -m drone_rl_planner.train_sac_polar \
  --steps 2000000 --dense-heavy --target 0.80 --device cuda \
  --resume src/drone_rl_planner/checkpoints/sac_polar_mixd_best.pt \
  --reset-buffer --finetune-lr 1e-4
```

权重 `*.pt` 与 replay 目录已被 gitignore。

## 路径 G

默认 `vfh_planner_node`；可选 PPO 见英文 README。

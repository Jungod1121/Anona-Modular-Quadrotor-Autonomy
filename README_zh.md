# Anona

### 模块化四旋翼自主仿真工作台

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

[English README](README.md) · **中文**

> 同一植物层。多种规划器。公平对照。

---

<!-- 媒体：将文件放入 docs/media/（见 docs/media/README.md） -->

<!-- ![Anona banner](docs/media/banner.png) -->
<p align="center">
  <em>横幅占位 — 请添加 <code>docs/media/banner.png</code>（建议 1600×480）</em>
</p>

<!-- [![Watch the demo](docs/media/demo-poster.png)](docs/media/demo.mp4) -->
<p align="center">
  <em>演示视频占位 — 请添加 <code>docs/media/demo.mp4</code> + <code>demo-poster.png</code></em>
</p>

<!-- ![Dense-field flight](docs/media/hero-dense-field.gif) -->
<p align="center">
  <em>动图占位 — 请添加 <code>docs/media/hero-dense-field.gif</code></em>
</p>

---

## 亮点

| | |
|---|---|
| **统一植物层** | 原生 ROS 2 刚体动力学 + 级联 PID + 混控器 — 不是 SO3 / MAVROS / fake_drone 的薄封装 |
| **规划矩阵** | 六种活跃后端（弱 / 强）共用**同一**话题契约，便于公平 A/B 对照 |
| **学习栈** | 路径 H：极坐标 DrQ-SAC + 外置 VFH 安全监督；课程式逼近密集场 |
| **任务控制台** | 浏览器 / Linux 原生应用：单机与多机、地图、启停、板载 RL 训练卡 |
| **多机** | EGO-Swarm、同场密集场、编队（一字 / 纵队 / V） |
| **验收** | 六项一键场景 + 规划器×地图批量矩阵 + 报告 |

---

## 图库

素材就绪后，替换 [`docs/media/`](docs/media/) 下的占位文件。

| 槽位 | 文件 | 建议内容 |
|------|------|----------|
| 架构 | `architecture.png` | 植物层 ↔ 规划器契约示意 |
| 控制台 UI | `console-ui.png` / `console-ui.gif` | 任务控制台截图 |
| 密集场 | `dense-field.gif` / `.mp4` | 路径 H 在 `dense_field` 中飞行 |
| 集群 | `swarm.gif` / `.mp4` | EGO-Swarm 或编队 |
| RViz | `rviz-overview.png` | 规范 RViz 布局 |
| 训练 | `sac-training.gif` | 训练卡 / 评测曲线 |

---

## Anona 是什么？

**Anona**（仓库名 `drone_ws`）是一套运行于 **Ubuntu 22.04 + ROS 2 Humble** 的模块化四旋翼仿真工作台。

它将**固定植物层**（动力学 + 控制器）与**可替换规划后端**分离，使经典搜索、轨迹优化与学习型规划器能在相同执行、感知话题与评测脚本下对照。

| 层级 | 作用 | 软件包 |
|------|------|--------|
| 植物层 | 里程计、IMU、电机转速 | `drone_dynamics`, `drone_controller` |
| 世界 | 可复现点云地图 | `drone_map`, `map_adapter` |
| 规划 | 路径 A–H 后端 | `drone_planner`、各 vendor、`drone_rl_planner` 等 |
| 运维 | 启动、控制台、验收 | `drone_bringup`、`scripts/`、任务控制台 |

---

## 规划后端

规范注册表：[`PLANNERS.md`](src/drone_bringup/PLANNERS.md)。

| ID | 后端 | 类别 | 摘要 |
|----|------|------|------|
| **A** `homemade` | Dyn-A* + B 样条 | 弱 | 自研搜索基线 |
| **B** `ego` | EGO-Planner | 强 | 官方谱系局部重规划 |
| **C** `gcopter` | GCOPTER / MINCO | 强 | 稀疏多项式轨迹 |
| **D** `fuel_explore` | 前沿 FSM | 模式 | 探索任务（不进公平矩阵） |
| **E** `mighty` | MIGHTY / Hermite | 强 | 适配后的强后端 |
| **F** `fast_planner` | Fast-Planner kino | 可选 | 仅作谱系参考 |
| **G** `vfh` | VFH+ | 弱 | 反应式直方图；可选 PPO 训练卡 |
| **H** `sac` | 极坐标 DrQ-SAC | 强 | 学习 + VFH 安全监督 |

**公平对照集合：** A / B / C / E / G / H。

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
ros2 launch drone_bringup planner_sim.launch.py planner:=ego map:=official_forest
```

地图目录：[`MAPS.md`](src/drone_bringup/MAPS.md)。

---

## 快速开始

### 依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-tf2-ros \
  ros-humble-pcl-conversions \
  libpcl-dev \
  python3-matplotlib \
  python3-numpy
```

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws
export PYTHONNOUSERSITE=1
colcon build --symlink-install
source install/setup.bash
```

### 任务控制台

```bash
# Linux 原生应用
bash packaging/linux/install.sh

# 浏览器
ros2 run drone_bringup dashboard   # http://127.0.0.1:8765/
```

---

## 场景

| # | 场景 | 启动文件 |
|---|------|----------|
| 1 | 悬停 | `hover.launch.py` |
| 2 | 单目标 | `single_goal.launch.py` |
| 3 | 多航点 | `multi_goal.launch.py` |
| 4 | 静态避障 | `avoidance.launch.py`（Path B · official_forest · 矩形后漏斗） |
| 5 | 窄通道 | `narrow_passage.launch.py` |
| 6 | 稳定性（+ 风扰 / IMU 噪声） | `stability_demo.launch.py` |

---

## 学习 · 路径 H（极坐标 DrQ-SAC）

课程（easy → medium → 更密混合）与最新阶段结果：

- 中文结果：[`src/drone_rl_planner/CURRICULUM_RESULTS_zh.md`](src/drone_rl_planner/CURRICULUM_RESULTS_zh.md)
- 训练说明：[`src/drone_rl_planner/README_zh.md`](src/drone_rl_planner/README_zh.md)

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
# 示例：轻量混合爬坡阶段
python3 -m drone_rl_planner.train_sac_polar --stage4 --device cuda
```

检查点（`*.pt`）已被 gitignore。辅助脚本位于 `src/drone_rl_planner/checkpoints/`。

---

## 话题契约

| 话题 | 类型 | 作用 |
|------|------|------|
| `/drone/motor_rpm_cmd` | `drone_msgs/MotorCommand` | 控制器 → 动力学 |
| `/drone/odom` | `nav_msgs/Odometry` | 状态 |
| `/drone/imu` | `sensor_msgs/Imu` | 感知 |
| `/drone/goal` | `geometry_msgs/PoseStamped` | 任务目标 |
| `/map/obstacles` | `sensor_msgs/PointCloud2` | 全局障碍 |
| `/planner/local_goal` | `geometry_msgs/PoseStamped` | 滚动局部目标 |
| `/planner/trajectory_cmd` | `drone_msgs/TrajectoryCommand` | 植物层指令 |
| `/planner/trajectory` | `nav_msgs/Path` | 规划路径（RViz） |
| `/planner/status` | `drone_msgs/PlannerStatus` | 规划器状态 |

坐标系：`map`（ENU）→ `base_link`（x 前，y 左，z 上）。

---

## 仓库结构

```
drone_ws/
├── src/                 # 植物层、规划器、bringup、RL
├── docs/media/          # 截图 / 视频 / GIF
├── packaging/           # Linux 原生控制台
├── scripts/             # 目标、评测、验收
└── report/              # 验收与基线报告
```

---

## 已知问题

1. `evaluate.py` 中密集障碍最近距离计算在大地图上可能较慢。
2. 悬停指标需用 `evaluate.py` / `stability_demo`，不能仅靠 RViz。
3. 后端 A 默认 `enable_bspline_opt: false`，以便在密集场景得到稳定 A* 折线。
4. 用户级 setuptools 83+ → 在 `colcon build` 前设置 `PYTHONNOUSERSITE=1`。

---

## 许可证

MIT

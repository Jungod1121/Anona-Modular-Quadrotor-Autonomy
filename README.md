# Anona

### Modular Quadrotor Autonomy Workbench · 模块化四旋翼自主仿真工作台

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**English** · [中文](#中文)

> One plant. Many planners. Fair comparison.  
> 同一植物层，多种规划后端，可公平对照。

---

<!-- ═══════════════════════════════════════════════════════════════
     MEDIA — drop files into docs/media/ (see docs/media/README.md)
     ═══════════════════════════════════════════════════════════════ -->

<!-- Banner / 横幅 -->
<!-- ![Anona banner](docs/media/banner.png) -->
<p align="center">
  <em>📷 Banner placeholder — add <code>docs/media/banner.png</code> (recommended 1600×480)</em>
</p>

<!-- Demo video / 演示视频 -->
<!-- [![Watch the demo](docs/media/demo-poster.png)](docs/media/demo.mp4) -->
<p align="center">
  <em>🎬 Demo video placeholder — add <code>docs/media/demo.mp4</code> + <code>demo-poster.png</code></em>
</p>

<!-- Hero animation / 主视觉动图 -->
<!-- ![Dense-field flight](docs/media/hero-dense-field.gif) -->
<p align="center">
  <em>✨ Animation placeholder — add <code>docs/media/hero-dense-field.gif</code></em>
</p>

---

## Highlights · 项目亮点

| | English | 中文 |
|---|---------|------|
| **Unified plant** | Native ROS 2 rigid-body dynamics + cascade PID + mixer — not a thin wrapper around SO3 / MAVROS / fake_drone | 自研原生 ROS 2 刚体动力学与级联 PID，非 SO3 / MAVROS / fake_drone 薄封装 |
| **Planner matrix** | Six active backends (weak & strong) on the **same** topic contract for fair A/B comparison | 六种强弱规划后端共用同一话题契约，便于公平对照 |
| **Learning stack** | Path H: Polar DrQ-SAC + external VFH safety supervisor; dense-catalog training with honest eval | 路径 H：极坐标 DrQ-SAC + 外置 VFH 安全监督；对齐密集场的诚实评测训练 |
| **Mission Console** | Browser / native Linux app: single & multi-UAV, maps, Start/Stop, onboard RL train cards | 浏览器或 Linux 原生「任务控制台」：单机/多机、地图、启停、板载 RL 训练卡 |
| **Multi-agent** | EGO-Swarm, shared dense field, formation (line / column / V) | EGO 集群、同场密集避障、编队（一字 / 纵队 / V） |
| **Acceptance** | Six one-click scenarios + batch planner×map matrix + reports | 六项一键验收 + 规划器×地图批量矩阵 + 报告 |

---

## Gallery · 图库 / 视频 / 动画

Replace the placeholders below when assets are ready. Paths are under [`docs/media/`](docs/media/).

| Slot | File | Suggested content |
|------|------|-------------------|
| Architecture | `architecture.png` | Plant ↔ planner contract diagram |
| Console UI | `console-ui.png` / `console-ui.gif` | Mission Console screenshots or walkthrough |
| Dense field | `dense-field.gif` / `.mp4` | Path H (or G) in `dense_field` |
| Swarm | `swarm.gif` / `.mp4` | EGO-Swarm or formation fly-through |
| RViz | `rviz-overview.png` | Canonical RViz layout |
| Training | `sac-training.gif` | Train-card progress / eval curve |

```text
docs/media/
├── banner.png              # Repo header
├── demo.mp4                # 30–90 s overview reel
├── demo-poster.png         # Video cover frame
├── hero-dense-field.gif    # Short loop for GitHub
├── architecture.png
├── console-ui.png
├── dense-field.gif
├── swarm.gif
├── rviz-overview.png
└── sac-training.gif
```

<!-- Uncomment when files exist:

### Architecture
![System architecture](docs/media/architecture.png)

### Mission Console
![Mission Console](docs/media/console-ui.png)

### Dense-field autonomy
![Dense field](docs/media/dense-field.gif)

### Multi-UAV
![Swarm](docs/media/swarm.gif)

-->

---

## What is Anona?

**Anona** (`drone_ws`) is a modular quadrotor simulation workbench on **Ubuntu 22.04 + ROS 2 Humble**.

It separates a **fixed plant** (dynamics + controller) from **swappable planning backends**, so classical search, optimization, and learning planners can be compared under identical actuation, sensing topics, and evaluation scripts.

| Layer | Role | Packages |
|-------|------|----------|
| Plant | Odometry, IMU, motor RPM | `drone_dynamics`, `drone_controller` |
| World | Seeded point-cloud maps | `drone_map`, `map_adapter` |
| Planning | Path A–H backends | `drone_planner`, vendors, `drone_rl_planner`, … |
| Ops | Launch, dashboard, acceptance | `drone_bringup`, `scripts/`, Mission Console |

---

## Planner backends · 规划后端

Canonical registry: [`PLANNERS.md`](src/drone_bringup/PLANNERS.md).

| ID | Backend | Class | Summary |
|----|---------|-------|---------|
| **A** `homemade` | Dyn-A* + B-spline | weak | In-house search baseline |
| **B** `ego` | EGO-Planner | strong | Official-lineage local replanner |
| **C** `gcopter` | GCOPTER / MINCO | strong | Sparse polynomial trajectory |
| **D** `fuel_explore` | Frontier FSM | mode | Exploration mission (not in fair matrix) |
| **E** `mighty` | MIGHTY / Hermite | strong | Adapted strong backend |
| **F** `fast_planner` | Fast-Planner kino | optional | Lineage reference only |
| **G** `vfh` | VFH+ | weak | Reactive histogram; optional PPO train card |
| **H** `sac` | Polar DrQ-SAC | strong | Learning + VFH safety supervisor |

**Fair comparison set:** A / B / C / E / G / H (`weak` ∪ `strong`, active).

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
ros2 launch drone_bringup planner_sim.launch.py planner:=ego map:=official_forest
```

Maps catalog: [`MAPS.md`](src/drone_bringup/MAPS.md). Any planner × any map via `map_adapter`.

---

## Quick start · 快速开始

### Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-tf2-ros \
  ros-humble-pcl-conversions \
  libpcl-dev \
  python3-matplotlib \
  python3-numpy
# Optional for some vendors: libnlopt-dev libarmadillo-dev
```

### Build

If user-level **setuptools 83+** breaks ament uninstall, isolate user site-packages:

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws
export PYTHONNOUSERSITE=1
colcon build --symlink-install
source install/setup.bash
```

Vendor planners (B / C / E / F):

```bash
export PYTHONNOUSERSITE=1
colcon build --symlink-install --packages-up-to map_generator ego_planner gcopter mighty drone_bringup
source install/setup.bash
```

### Mission Console (recommended)

**Native Linux app**

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && colcon build --symlink-install --packages-select drone_bringup
bash packaging/linux/install.sh
# Activities → “Drone WS Console” / 「无人机仿真控制台」
# After pull: drone-ws-update
```

**Browser**

```bash
source /opt/ros/humble/setup.bash && source ~/drone_ws/install/setup.bash
ros2 run drone_bringup dashboard   # http://127.0.0.1:8765/
```

Select a single-UAV backend (A–H) or multi-UAV mission (EGO-Swarm / shared field / formation), then Start. Path **G** / **H** expose a training card independent of the sim process.

---

## Scenarios · 验收与演示场景

| # | Scenario | Launch |
|---|----------|--------|
| 1 | Hover | `hover.launch.py` |
| 2 | Single goal | `single_goal.launch.py` |
| 3 | Multi-waypoint | `multi_goal.launch.py` |
| 4 | Static avoidance | `avoidance.launch.py` |
| 5 | Narrow passage | `narrow_passage.launch.py` |
| 6 | Stability (+ wind / IMU noise) | `stability_demo.launch.py` |
| + | Shared dense field (multi) | `shared_field.launch.py` |
| + | Formation (line / column / V) | `formation.launch.py formation:=v` |
| + | EGO-Swarm | `ego_swarm.launch.py num_drones:=2` |

```bash
ros2 launch drone_bringup hover.launch.py
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=dense_field
```

---

## Learning · Path H (Polar DrQ-SAC)

Dense training matches catalog `dense_field` density. Eval uses **≥60 episodes** and a score that **penalizes collisions**. Details: [`src/drone_rl_planner/README.md`](src/drone_rl_planner/README.md).

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
python3 -m drone_rl_planner.train_sac_polar \
  --steps 1000000 --dense-heavy --target 0.95 --device cuda \
  --batch-size 128 --updates-per-step 2 --n-envs 2
```

Checkpoints (`*.pt`) are gitignored. Overnight helper: `src/drone_rl_planner/checkpoints/launch_overnight.sh`.

---

## Topic contract · 话题契约

| Topic | Type | Role |
|-------|------|------|
| `/drone/motor_rpm_cmd` | `drone_msgs/MotorCommand` | Controller → dynamics |
| `/drone/odom` | `nav_msgs/Odometry` | State |
| `/drone/imu` | `sensor_msgs/Imu` | Sensing |
| `/drone/goal` | `geometry_msgs/PoseStamped` | Mission goal |
| `/map/obstacles` | `sensor_msgs/PointCloud2` | Global obstacles |
| `/planner/local_goal` | `geometry_msgs/PoseStamped` | Rolling local goal |
| `/planner/trajectory_cmd` | `drone_msgs/TrajectoryCommand` | Plant command |
| `/planner/trajectory` | `nav_msgs/Path` | Planned path (RViz) |
| `/planner/status` | `drone_msgs/PlannerStatus` | Planner status |

Frames: `map` (ENU) → `base_link` (x forward, y left, z up).

---

## Repository layout · 仓库结构

```
drone_ws/                    # Anona workbench
├── src/
│   ├── drone_msgs/
│   ├── drone_dynamics/      # Native plant
│   ├── drone_controller/    # Cascade PID + mixer
│   ├── drone_map/
│   ├── drone_planner/       # Backend A
│   ├── drone_rl_planner/    # Backends G & H
│   ├── drone_exploration/   # Mode D
│   ├── ego_vendor/          # Backend B
│   ├── gcopter_vendor/      # Backend C
│   ├── mighty_vendor/       # Backend E
│   ├── fast_planner_vendor/ # Optional F
│   ├── drone_visualization/
│   └── drone_bringup/       # Launches + Mission Console
├── docs/media/              # Screenshots, video, GIFs (you add)
├── packaging/               # Native Linux / desktop installers
├── scripts/                 # Goals, eval, acceptance, batch matrix
└── report/                  # Acceptance & baseline reports
```

---

## Tools · 工具

```bash
python3 scripts/send_goal.py --x 2 --y 1 --z 1.5
python3 scripts/waypoint_publisher.py --pattern square --side 2.0 --z 1.5
python3 scripts/evaluate.py --duration 60 --goal-x 0 --goal-y 0 --goal-z 1.5
python3 scripts/run_acceptance.py
python3 scripts/run_batch_matrix.py --dry-run
```

---

## Plant parameters · 植物层参数

Dynamics and controller must stay consistent (`src/drone_bringup/config/`):

| Parameter | Value |
|-----------|-------|
| mass | 1.0 kg |
| arm_length | 0.18 m |
| k_F / k_M | 1.5e-6 / 2.5e-8 |
| Ixx = Iyy / Izz | 0.01 / 0.02 |

---

## Relation to reference code · 与参考仓库

The **plant** is an independent native ROS 2 implementation. Topic layering ideas draw from MARSIM / pengyu_sim; EGO planning code is vendored without the original SO3 plant. See `notes/reference_repos_notes.md`.

---

## Known issues · 已知问题

1. Dense obstacle nearest-distance in `evaluate.py` can be slow on large maps.
2. Hover metrics need `evaluate.py` / `stability_demo`, not RViz alone.
3. Backend A defaults `enable_bspline_opt: false` for stable dense A* polylines.
4. User setuptools 83+ → set `PYTHONNOUSERSITE=1` before `colcon build`.

---

## License

MIT

---

<a id="中文"></a>

# Anona（中文）

### 模块化四旋翼自主仿真工作台

**Anona**（仓库名 `drone_ws`）运行于 Ubuntu 22.04 + ROS 2 Humble：用**固定植物层**（自研动力学 + 级联 PID）对接**可替换规划后端**，在同一话题契约与评测脚本下对比经典搜索、轨迹优化与强化学习方法。

## 亮点

- **统一植物层**：原生 ROS 2 实现，非 SO3 / MAVROS / fake_drone 封装。
- **规划矩阵**：弱/强六类活跃后端（A/B/C/E/G/H）公平对照；探索模式 D、谱系对照 F 不进主矩阵。
- **学习路径 H**：极坐标占用图 + DrQ-SAC + 外置 VFH 安全监督；训练密度对齐 `dense_field`，评测惩罚碰撞。
- **任务控制台**：浏览器或 Linux 原生应用；单机 / 多机 / 地图 / 启停；G/H 训练卡片与仿真进程解耦。
- **多机能力**：EGO-Swarm、同场密集避障、编队（一字 / 纵队 / V）。
- **验收体系**：六场景一键启动、批量矩阵与报告产物。

## 媒体素材

请将截图、演示视频与 GIF 放入 [`docs/media/`](docs/media/)，并按该目录说明取消 README 顶部注释即可展示。建议至少准备：横幅、30–90 秒总览视频、密集场飞行动图、控制台 UI、架构图。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && export PYTHONNOUSERSITE=1
colcon build --symlink-install && source install/setup.bash

# 任务控制台
ros2 run drone_bringup dashboard          # http://127.0.0.1:8765/
# 或：bash packaging/linux/install.sh

# 示例：路径 H + 密集场
ros2 launch drone_bringup planner_sim.launch.py planner:=sac map:=dense_field
```

规划与地图细节见 [`PLANNERS.md`](src/drone_bringup/PLANNERS.md)、[`MAPS.md`](src/drone_bringup/MAPS.md)；Path H 训练见 [`drone_rl_planner/README.md`](src/drone_rl_planner/README.md)。

## 文档导航

| Doc | Content |
|-----|---------|
| [`PLANNERS.md`](src/drone_bringup/PLANNERS.md) | Backend registry & contract |
| [`MAPS.md`](src/drone_bringup/MAPS.md) | Map catalog & tiers |
| [`SWARM.md`](src/drone_bringup/SWARM.md) | Multi-UAV |
| [`docs/media/README.md`](docs/media/README.md) | Media asset guide |
| [`packaging/linux/README.md`](packaging/linux/README.md) | Native console install |
| [`VISUALIZATION.md`](src/drone_visualization/VISUALIZATION.md) | RViz layout |

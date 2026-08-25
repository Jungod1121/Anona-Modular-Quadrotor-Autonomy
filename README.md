# Anona

### Modular Quadrotor Autonomy Workbench

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![介绍页](https://img.shields.io/badge/%E4%BB%8B%E7%BB%8D%E9%A1%B5-Apple%20%E9%A3%8E-0071E3)](https://jungod1121.github.io/Anona-Modular-Quadrotor-Autonomy/)

**English** · [中文版 README](README_zh.md)

> One plant. Many planners. Fair comparison.

<p align="center">
  <a href="https://www.bilibili.com/video/BV11Dgi6gE9A/">
    <img src="docs/media/demo-poster.png" alt="Anona demo video" width="720"/>
  </a>
  <br/>
  <em><a href="https://www.bilibili.com/video/BV11Dgi6gE9A/">Watch demo on Bilibili · Anona: Modular Quadrotor Autonomy</a></em>
</p>

<p align="center">
  <img src="docs/media/forest.gif" alt="Single-UAV random forest loop" width="720"/>
</p>
<p align="center">
  <img src="docs/media/swarm-or-formation.gif" alt="Multi-UAV formation" width="720"/>
</p>

---

## Highlights

| | |
|---|---|
| **Unified plant** | Self-developed ROS 2 dynamics + cascade PID + mixer |
| **Many planners** | Path A–H on one topic contract for fair comparison |
| **Diverse maps** | Process-generated forests, mazes, dense fields, narrow corridors |
| **Mission Console** | Browser / native app: maps, planners, single & multi-UAV |
| **Acceptance** | Six one-click scenarios (6/6 PASS) + [formal report](report/final_paper/) |

---

<p align="center">
  <img src="docs/media/architecture.png" alt="Architecture" width="640"/>
  <br/>
  <img src="docs/media/topic-dataflow.png" alt="Topic contract" width="640"/>
</p>
<p align="center">
  <img src="docs/media/console-ui.png" alt="Mission console" width="420"/>
  &nbsp;
  <img src="docs/media/rviz-overview.png" alt="RViz" width="420"/>
</p>

---

## What is Anona?

**Anona** is a modular quadrotor simulation workbench on **Ubuntu 22.04 + ROS 2 Humble**. A fixed plant (dynamics + controller) drives swappable planners under identical topics and evaluation scripts.

| Layer | Role | Packages |
|-------|------|----------|
| Plant | Odometry, IMU, motor RPM | `drone_dynamics`, `drone_controller` |
| World | Seeded point-cloud maps | `drone_map`, `map_adapter` |
| Planning | Path A–H backends | `drone_planner`, vendors, `drone_rl_planner`, … |
| Ops | Launch, dashboard, acceptance | `drone_bringup`, Mission Console |

Registry: [`PLANNERS.md`](src/drone_bringup/PLANNERS.md) · Maps: [`MAPS.md`](src/drone_bringup/MAPS.md)

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=ego map:=official_forest
```

---

## Quick start

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop ros-humble-tf2-ros ros-humble-pcl-conversions \
  libpcl-dev python3-matplotlib python3-numpy

source /opt/ros/humble/setup.bash
cd ~/drone_ws
export PYTHONNOUSERSITE=1
colcon build --symlink-install
source install/setup.bash

# Mission Console
bash packaging/linux/install.sh          # native
ros2 run drone_bringup dashboard         # http://127.0.0.1:8765/
```

---

## Acceptance (6/6)

| # | Scenario | Launch |
|---|----------|--------|
| 1 | Hover | `ros2 launch drone_bringup hover.launch.py` |
| 2 | Single goal | `ros2 launch drone_bringup single_goal.launch.py` |
| 3 | Multi-waypoint | `ros2 launch drone_bringup multi_goal.launch.py` |
| 4 | Static avoidance | `ros2 launch drone_bringup avoidance.launch.py` |
| 5 | Narrow passage | `ros2 launch drone_bringup narrow_passage.launch.py` |
| 6 | Stability | `ros2 launch drone_bringup stability_demo.launch.py` |

```bash
python3 scripts/run_acceptance.py          # six scenarios (~30 min; scenario 4
                                           #   budgets one retry for planner variance)
python3 scripts/run_conformance.py --planner ego   # single-planner contract check
```

Scenario 4 flies the official forest at a deliberately conservative speed with a
hard grid-inflation margin — its acceptance window is 420 s and the min-obstacle
criterion is 0.30 m.

<p align="center">
  <img src="docs/media/scenario_04_avoid_rviz.png" alt="Avoidance" width="360"/>
  &nbsp;
  <img src="docs/media/scenario_05_narrow_rviz.png" alt="Narrow passage" width="360"/>
</p>

Report: [`report/acceptance_report.md`](report/acceptance_report.md) · Benchmark: [`report/planner_benchmark/comparison_report.md`](report/planner_benchmark/comparison_report.md)

---

## Topic contract

| Topic | Role |
|-------|------|
| `/drone/goal` | Mission goal |
| `/map/obstacles` | Obstacle cloud |
| `/planner/local_goal`, `/planner/trajectory_cmd` | Planner → plant |
| `/drone/motor_rpm_cmd` | Controller → dynamics |
| `/drone/odom` | State feedback |

Frames: `map` (ENU) → `base_link`.

---

## Citations

- Repo: <https://github.com/Jungod1121/Anona-Modular-Quadrotor-Autonomy>
- [EGO-Planner Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) · [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) · [MIGHTY](https://github.com/mit-acl/mighty) · [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) · [DrQ](https://github.com/denisyarats/drq) / [DrQ-v2](https://github.com/facebookresearch/drqv2)

## License

MIT

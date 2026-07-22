# Anona

### 模块化四旋翼自主仿真工作台

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

[English README](README.md) · **中文**

> 同一植物层，多种规划后端，可公平对照。

---

<!-- 媒体：放入 docs/media/，说明见 docs/media/README.md -->

<p align="center">
  <em>横幅占位 — 请添加 <code>docs/media/banner.png</code>（建议 1600×480）</em>
</p>

<p align="center">
  <em>演示视频占位 — 请添加 <code>docs/media/demo.mp4</code> + <code>demo-poster.png</code></em>
</p>

<p align="center">
  <em>动图占位 — 请添加 <code>docs/media/hero-dense-field.gif</code></em>
</p>

---

## 亮点

- **统一植物层**：原生 ROS 2 刚体动力学与级联 PID，非 SO3 / MAVROS / fake_drone 薄封装。
- **规划矩阵**：弱/强六类活跃后端（A/B/C/E/G/H）共用同一话题契约，便于公平对照。
- **学习路径 H**：极坐标 DrQ-SAC + 外置 VFH 安全监督；课程式逼近密集场。
- **任务控制台**：浏览器或 Linux 原生应用；单机/多机、地图、启停；G/H 训练卡与仿真解耦。
- **多机能力**：EGO-Swarm、同场密集避障、编队（一字 / 纵队 / V）。
- **验收体系**：六场景一键启动、批量矩阵与报告。

---

## Anona 是什么？

**Anona**（仓库名 `drone_ws`）运行于 **Ubuntu 22.04 + ROS 2 Humble**：用**固定植物层**对接**可替换规划后端**，在同一话题契约与评测脚本下对比经典搜索、轨迹优化与强化学习方法。

| 层级 | 作用 | 包 |
|------|------|-----|
| 植物层 | 里程计、IMU、电机转速 | `drone_dynamics`, `drone_controller` |
| 世界 | 可复现点云地图 | `drone_map`, `map_adapter` |
| 规划 | 路径 A–H | `drone_planner`, 各 vendor, `drone_rl_planner` |
| 运维 | 启动、控制台、验收 | `drone_bringup`, `scripts/` |

规划注册表：[`PLANNERS.md`](src/drone_bringup/PLANNERS.md)。地图目录：[`MAPS.md`](src/drone_bringup/MAPS.md)。

---

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

---

## 路径 H 学习 / 课程结果

- 中文阶段结果：[`src/drone_rl_planner/CURRICULUM_RESULTS_zh.md`](src/drone_rl_planner/CURRICULUM_RESULTS_zh.md)
- 训练说明：[`src/drone_rl_planner/README_zh.md`](src/drone_rl_planner/README_zh.md)

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
python3 -m drone_rl_planner.train_sac_polar --stage4 --device cuda
```

权重文件 `*.pt` 已被 gitignore，需在本机训练或自行备份。

---

## 文档导航

| 文档 | 内容 |
|------|------|
| [`PLANNERS.md`](src/drone_bringup/PLANNERS.md) | 后端注册与契约 |
| [`MAPS.md`](src/drone_bringup/MAPS.md) | 地图目录 |
| [`SWARM.md`](src/drone_bringup/SWARM.md) | 多机 |
| [`docs/media/README.md`](docs/media/README.md) | 媒体素材说明 |
| [`packaging/linux/README.md`](packaging/linux/README.md) | 原生控制台安装 |

---

## 许可证

MIT

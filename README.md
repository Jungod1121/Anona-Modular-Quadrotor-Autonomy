# drone_ws — ROS2 四旋翼仿真工程

Ubuntu 22.04 + ROS2 Humble 下的模块化四旋翼仿真工作区：自研动力学/控制器（原生 ROS2，非参考仓库封装）、点云地图、多后端规划器（移植或自实现）、RViz2 可视化、Web 控制面板，以及六个验收场景的一键启动。

## 依赖

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

可选（规划器移植）：`libnlopt-dev`、`libarmadillo-dev`

## 编译

若本机用户目录安装了较新的 **setuptools 83+**，会打断 ament_python 的 `--uninstall`。
编译 Python 包前请隔离用户 site-packages：

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws
export PYTHONNOUSERSITE=1
colcon build --symlink-install
source install/setup.bash
```

## 规划路径（Path A–H）

统一契约、分类与切换说明见 [`src/drone_bringup/PLANNERS.md`](src/drone_bringup/PLANNERS.md)（与 [`planner_registry.py`](src/drone_bringup/drone_bringup/planner_registry.py) 同步）。

| 路径 | 分类 | 含义 | 启动命令 |
|------|------|------|----------|
| **Path A** | weak | Dyn-A* + B样条 (`drone_planner`) | `ros2 launch drone_bringup avoidance.launch.py` |
| **Path B** | strong | EGO-Planner + map_generator | `ros2 launch drone_bringup ego_avoidance.launch.py` |
| **Path C** | strong | GCOPTER/MINCO | `ros2 launch drone_bringup gcopter_avoidance.launch.py` |
| **Mode D** | mode | 前沿探索任务模式（EGO 轨迹后端，非独立规划器） | `ros2 launch drone_bringup fuel_explore.launch.py` |
| **Path E** | strong | MIGHTY / Hermite 适配 | `ros2 launch drone_bringup mighty_avoidance.launch.py` |
| **Optional F** | optional | Fast-Planner kino 谱系对照（不计入标准强弱矩阵） | `ros2 launch drone_bringup fast_planner_avoidance.launch.py` |
| **Path G** | weak | VFH+ 直方图避障 (`vfh`，`rl` 为兼容别名) | `ros2 launch drone_bringup rl_avoidance.launch.py` |
| **Path H** | strong | Polar DrQ-SAC + 外部 `safety_supervisor` | `ros2 launch drone_bringup sac_avoidance.launch.py` |

一键切换：

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=homemade      # Path A
ros2 launch drone_bringup planner_sim.launch.py planner:=ego           # Path B
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter       # Path C
ros2 launch drone_bringup planner_sim.launch.py planner:=fuel_explore  # Mode D
ros2 launch drone_bringup planner_sim.launch.py planner:=mighty        # Path E
ros2 launch drone_bringup planner_sim.launch.py planner:=fast_planner  # Optional F
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh           # Path G (alias: rl)
ros2 launch drone_bringup planner_sim.launch.py planner:=sac           # Path H
```

**公平对比矩阵**（`canonical_comparison_ids`）：Path A / B / C / E / G / H 中 class 为 `weak` 或 `strong` 且 `active` 的后端；Mode D 与 Optional F 不在矩阵内。

所有规划器经统一契约发布 `/planner/local_goal` + `/planner/trajectory_cmd`（及 `/planner/trajectory` 可视化）；**植物层**（`drone_dynamics` + `drone_controller`）为自写原生 ROS2 节点，不依赖 SO3 / fake_drone / MAVROS 封装。

地图可独立切换（官方 EGO 场景 + 自研地图 + 四档 tier 预设），见 [`src/drone_bringup/MAPS.md`](src/drone_bringup/MAPS.md)。`map_adapter` 同时提供 `/map/obstacles` 与 `/map_generator/global_cloud`，任意规划器可配任意地图。

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter map:=official_maze2d
ros2 launch drone_bringup planner_sim.launch.py planner:=vfh map:=tier_medium_corridor
```

编译含 Path B/C/E/F 等 vendor 包：

```bash
export PYTHONNOUSERSITE=1
colcon build --symlink-install --packages-up-to map_generator ego_planner gcopter mighty drone_bringup
source install/setup.bash
```

**为何选 GCOPTER 作 Path C**：MINCO 是 EGO-B 样条之后更快的稀疏多项式表示（T-RO 2022）；yuwei-wu ROS2 移植可落地。Fast-Planner（ROS1 过重）保留为 Optional F 谱系对照，非主验收路径。

## 六个验收场景（一键启动）

| # | 场景 | 命令 |
|---|------|------|
| 1 | 悬停 (0,0,1.5) | `ros2 launch drone_bringup hover.launch.py` |
| 2 | 单目标点 (2,1,1.5) | `ros2 launch drone_bringup single_goal.launch.py` |
| 3 | 多目标点（正方形 4 点） | `ros2 launch drone_bringup multi_goal.launch.py` |
| 4 | 静态避障（dense_field, seed=42） | `ros2 launch drone_bringup avoidance.launch.py` |
| 5 | 狭窄通道绕行 | `ros2 launch drone_bringup narrow_passage.launch.py` |
| 6 | 稳定性展示（风扰+IMU噪声+评测） | `ros2 launch drone_bringup stability_demo.launch.py` |
| + | 双机空旷演示 | `ros2 launch drone_bringup multi_drone.launch.py` |
| + | **同场避障**（共享 dense 地图 + 机间 keep-out） | `ros2 launch drone_bringup shared_field.launch.py` |
| + | **编队** line / column / V（3 机） | `ros2 launch drone_bringup formation.launch.py formation:=v` |
| + | **Path B 官方 EGO 避障** | `ros2 launch drone_bringup ego_avoidance.launch.py` |
| + | **Path B EGO-Swarm 多机** | `ros2 launch drone_bringup ego_swarm.launch.py num_drones:=2` |
| + | **Path C GCOPTER/MINCO 避障** | `ros2 launch drone_bringup gcopter_avoidance.launch.py` |
| + | **Mode D 前沿探索** | `ros2 launch drone_bringup fuel_explore.launch.py` |
| + | **Path G VFH+** | `ros2 launch drone_bringup rl_avoidance.launch.py` |
| + | **Path H SAC** | `ros2 launch drone_bringup sac_avoidance.launch.py` |
| + | **规划器切换** | `ros2 launch drone_bringup planner_sim.launch.py planner:=ego` |

通用参数：

```bash
# 不启动 RViz
ros2 launch drone_bringup hover.launch.py use_rviz:=false

# 多目标：圆形 / 8 字轨迹
ros2 launch drone_bringup multi_goal.launch.py pattern:=circle
ros2 launch drone_bringup multi_goal.launch.py pattern:=eight

# 稳定性场景：单目标 + 关闭自动评测
ros2 launch drone_bringup stability_demo.launch.py mode:=single_goal run_eval:=false
```

## 话题契约（无 namespace）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/drone/motor_rpm_cmd` | `drone_msgs/MotorCommand` | 控制器 → 动力学 |
| `/drone/odom` | `nav_msgs/Odometry` | 动力学发布 |
| `/drone/imu` | `sensor_msgs/Imu` | 动力学发布 |
| `/drone/path` | `nav_msgs/Path` | 实际轨迹 |
| `/drone/goal` | `geometry_msgs/PoseStamped` | RViz Goal / 脚本 |
| `/map/obstacles` | `sensor_msgs/PointCloud2` | 全局障碍点云 |
| `/map/obstacles_markers` | `visualization_msgs/MarkerArray` | 障碍 RViz 标记 |
| `/planner/trajectory` | `nav_msgs/Path` | 规划路径 |
| `/planner/local_goal` | `geometry_msgs/PoseStamped` | 滚动局部目标 |
| `/planner/local_goal_marker` | `visualization_msgs/Marker` | 局部目标可视化 |
| `/planner/status` | `drone_msgs/PlannerStatus` | 规划器状态 |

坐标系：`map`（ENU）→ `base_link`（x 前 y 左 z 上）。

## Web 控制面板（推荐）

本地浏览器面板：单机 Path A–H、**多机 EGO-Swarm / 同场避障 / 编队**、地图选择、Start/Stop。

**Path G / H 训练卡片：** 在单机页选中路径 G（VFH/PPO）或路径 H（SAC）后，中间会出现「训练进度」面板（与仿真 Start 独立）。Path H 默认 **dense-heavy**、目标成功率 **≥95%**、诚实评测 ≥60 局；详见 [`src/drone_rl_planner/README.md`](src/drone_rl_planner/README.md)。

**做成 Ubuntu / Linux 原生应用（推荐）：**

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && colcon build --symlink-install --packages-select drone_bringup
bash packaging/linux/install.sh
# 活动区搜索「无人机仿真控制台」；更新：drone-ws-update
# 说明见 packaging/linux/README.md
```

浏览器方式仍可用：

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && source install/setup.bash
ros2 run drone_bringup dashboard
# 浏览器打开 http://127.0.0.1:8765/
# 可选参数：ros2 run drone_bringup dashboard -- --port 8765
```

等价于面板里预览的 `planner_sim.launch.py`；Stop 会结束该 launch 进程组。规划器列表来自 `planner_registry`（含 class / 别名）。

**多机注意：** 请在「多机」页选择任务后再 Start；命令预览应为 `ego_swarm` / `shared_field` / `formation` 的 launch，而不是单机 `planner_sim`。编队形状现为 **line / column / V**（3 机）。共享空域与编队固定 **dense_field**。

## Path H 密集场训练（摘要）

```bash
cd ~/drone_ws
export PYTHONPATH=src/drone_rl_planner:$PYTHONPATH
python3 -m drone_rl_planner.train_sac_polar \
  --steps 1000000 --dense-heavy --target 0.95 --device cuda \
  --batch-size 128 --updates-per-step 2 --n-envs 2
```

- 训练密度对齐 catalog `dense_field`（勿用过稀的 gym 成功率当「已练好」）。
- `updates-per-step` 过大（如 12）会抬高 GPU% 但 **大幅降低环境步/小时**；推荐 2。
- 续训会重置本轮步数计数；`--resume` 时保留历史 best，避免弱评测覆盖更强的 `*_best.pt`。
- 检查点 `*.pt` 默认不入库（见 `.gitignore`）。

## 工具脚本

仓库根目录 `scripts/`（编译后亦安装到 `share/drone_bringup/scripts/`）：

```bash
# 手动发目标
python3 scripts/send_goal.py --x 2 --y 1 --z 1.5

# 航点序列（square / circle / eight / list）
python3 scripts/waypoint_publisher.py --pattern square --side 2.0 --z 1.5

# 评测：导出 CSV + 图到 scripts/output/
python3 scripts/evaluate.py --duration 60 --goal-x 0 --goal-y 0 --goal-z 1.5

# 六项验收（PLAN.md）→ report/acceptance_report.md
python3 scripts/run_acceptance.py

# 规划器 × 地图 tier × seed 小矩阵（dry-run 只写 manifest）
python3 scripts/run_batch_matrix.py --dry-run
python3 scripts/run_batch_matrix.py --duration 45
python3 scripts/run_batch_matrix.py --planners homemade,vfh,ego --tiers tier_simple_open,tier_medium_corridor
```

`evaluate_drone` 在话题可用时会额外记录 `/planner/trajectory`、`/planner/diagnostics` 等，并导出 detour ratio、jerk、fallback 触发率等到 `summary.txt`。批量矩阵默认 `homemade` / `vfh` / `ego` × tier 预设 × seed 42；manifest 见 `report/batch_matrix/manifest.json`。

## 包结构

```
drone_ws/
├── src/
│   ├── drone_msgs/           # 自定义消息
│   ├── drone_dynamics/       # 刚体动力学（自研，原生 ROS2）
│   ├── drone_controller/     # 级联 PID + mixer（自研，原生 ROS2）
│   ├── drone_map/            # 固定种子点云地图
│   ├── drone_planner/        # Path A：自研 EGO 风格规划器
│   ├── drone_exploration/    # Mode D：前沿探索 FSM
│   ├── drone_rl_planner/     # Path G VFH+ / Path H SAC + safety_supervisor
│   ├── ego_vendor/           # Path B：官方 ego_planner / map_generator 等
│   ├── gcopter_vendor/       # Path C：GCOPTER/MINCO（无 SO3 plant）
│   ├── mighty_vendor/        # Path E：MIGHTY
│   ├── fast_planner_vendor/  # Optional F：Fast-Planner 谱系
│   ├── drone_visualization/  # 机体 Marker + RViz 配置
│   └── drone_bringup/        # 场景 launch + planner_sim / dashboard
├── scripts/                  # 轨迹发生器、评测、批量矩阵
├── report/                   # 报告与 batch_matrix manifest
└── reference_repos/          # MARSIM / pengyu_sim / ego-planner 参考（非运行依赖）
```

## 物理参数（dynamics 与 controller 必须一致）

| 参数 | 值 |
|------|-----|
| mass | 1.0 kg |
| arm_length | 0.18 m |
| k_F | 1.5e-6 |
| k_M | 2.5e-8 |
| Ixx = Iyy | 0.01 |
| Izz | 0.02 |

配置文件位于 `src/drone_bringup/config/`。

## 与参考仓库的关系（概念借鉴，独立 ROS2 实现）

本工作区的 **植物层**（`drone_dynamics` + `drone_controller`）为从零编写的 **原生 ROS2 节点**，不是对 pengyu_sim、MARSIM 或 EGO 仿真器的 wrapper/薄封装；话题契约与分层思想有对照，代码与依赖完全独立。

- **MARSIM**：借鉴点云地图锁存发布与仿真器/算法解耦接口；**不参考**其简化动力学。
- **pengyu_sim（ROS1）**：借鉴 `quadrotor_dynamics` ↔ `uav_pid` 话题分层与 `map_generator` 发布模式；动力学/控制器自写；不接入 MAVROS 伪装层。
- **EGO-Planner（ego-planner-swarm `ros2_version`）**：仅移植/参考 `bspline_opt` + `path_searching` 规划管线；**弃用**原版 `so3_control` 与 `so3_quadrotor_simulator`。若环境依赖受阻，允许按论文公式自实现简化版（须在代码注释中说明）。

详见 `notes/reference_repos_notes.md` 与 `PLAN.md`。

## 已知问题

1. **评测脚本**：`evaluate.py` 对障碍点云做全量最近距离，大地图（dense_field 80 障碍）时可能较慢；输出目录默认为 `scripts/output/`。
2. **悬停硬指标**：位置误差 ≤ 0.3 m 需在场景跑通后用 `scripts/evaluate.py` 或 `stability_demo.launch.py` 核对，仅目视 RViz 不够。
3. **规划器（Path A）**：默认 `enable_bspline_opt: false`（稳定 dense A* 折线）；可把 `planner.yaml` 中该项改回 `true` 试 B-spline。
4. **多机 namespace**：`ros2 launch drone_bringup multi_drone.launch.py` 启动 `uav0`/`uav1` 两套节点；单机验收仍用无 namespace 的六场景 launch。
5. **代理克隆**：`reference_repos/ego-planner-swarm` 需本机代理（如 Clash `127.0.0.1:7897`）方可 `git clone`；仅作参考，不参与 colcon 编译。官方包已 vendor 到 `src/ego_vendor/`。
6. **抗扰加分**：控制器默认开启位置积分 + 扰动观测器（DOB）与 IMU 陀螺 LPF 融合；`stability_demo` 开风扰+IMU 噪声时误差应优于旧版（ki=0）设置。
7. **setuptools**：用户级 setuptools 83+ 需 `PYTHONNOUSERSITE=1` 再 `colcon build`（见上文编译节）。

## RViz

`drone_visualization` 提供 canonical 配置 [`rviz/drone.rviz`](src/drone_visualization/rviz/drone.rviz)，详见 [`VISUALIZATION.md`](src/drone_visualization/VISUALIZATION.md)：

- Grid / 障碍点云 `/map/obstacles` / 障碍 Marker
- 机体 Marker `/drone/body_markers`
- 实际轨迹 `/drone/path`、规划轨迹 `/planner/trajectory`
- 局部目标 `/planner/local_goal_marker`、任务目标 `/drone/goal`
- Odometry、TF
- **2D Goal Pose** 工具发布到 `/drone/goal`

## License

MIT（课程作业工程）

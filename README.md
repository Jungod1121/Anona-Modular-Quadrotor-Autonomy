# drone_ws — ROS2 四旋翼仿真工程

Ubuntu 22.04 + ROS2 Humble 下的模块化四旋翼仿真工作区：自研动力学/控制器、点云地图、EGO-Planner 风格规划器（移植或自实现）、RViz2 可视化，以及六个验收场景的一键启动。

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

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
```

## 三个规划路径（Path A / B / C）

统一契约与切换说明见 [`src/drone_bringup/PLANNERS.md`](src/drone_bringup/PLANNERS.md)。

| 路径 | 含义 | 启动命令 |
|------|------|----------|
| **Path A** | 自研 `drone_planner` + 自研 `drone_map` | `ros2 launch drone_bringup avoidance.launch.py` |
| **Path B** | 官方 `ego_planner` + 官方 `map_generator` + 自研动力学/控制器 | `ros2 launch drone_bringup ego_avoidance.launch.py` |
| **Path C** | **GCOPTER/MINCO**（较新的多项式轨迹优化）+ 官方地图 + 自研动力学/控制器 | `ros2 launch drone_bringup gcopter_avoidance.launch.py` |

一键切换：

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=homemade   # Path A
ros2 launch drone_bringup planner_sim.launch.py planner:=ego        # Path B
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter    # Path C
```

所有规划器都只发布 `/planner/local_goal` + `/planner/trajectory_cmd`；植物（动力学/控制器）不变，不用 SO3 / fake_drone。

地图可独立切换（官方 EGO 场景 + 自研地图），见 [`src/drone_bringup/MAPS.md`](src/drone_bringup/MAPS.md)。默认：Path A → `dense_field`，Path B/C → `official_forest`。`cloud_bridge` 同时提供 `/map/obstacles` 与 `/map_generator/global_cloud`，任意规划器可配任意地图。

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter map:=official_maze2d
ros2 launch drone_bringup planner_sim.launch.py planner:=ego map:=dense_field
```

编译含 Path C：

```bash
colcon build --symlink-install --packages-up-to map_generator ego_planner gcopter drone_bringup
source install/setup.bash
```

**为何选 GCOPTER 作 Path C**：MINCO 是 EGO-B 样条之后更快的稀疏多项式表示（T-RO 2022）；yuwei-wu ROS2 移植可落地。未整库搬 EGO-v2 Swarm Playground / Fast-Planner（ROS1 过重）/ RL 规划器。

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
| + | **编队** line/column/v/triangle/diamond | `ros2 launch drone_bringup formation.launch.py formation:=v` |
| + | **Path B 官方 EGO 避障** | `ros2 launch drone_bringup ego_avoidance.launch.py` |
| + | **Path B EGO-Swarm 多机** | `ros2 launch drone_bringup ego_swarm.launch.py num_drones:=2` |
| + | **Path C GCOPTER/MINCO 避障** | `ros2 launch drone_bringup gcopter_avoidance.launch.py` |
| + | **规划器切换** homemade/ego/gcopter | `ros2 launch drone_bringup planner_sim.launch.py planner:=ego` |

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
| `/planner/status` | `drone_msgs/PlannerStatus` | 规划器状态 |

坐标系：`map`（ENU）→ `base_link`（x 前 y 左 z 上）。

## Web 控制面板（推荐）

本地浏览器面板：单机 Path A/B/C、**多机 EGO-Swarm / 同场 / 编队**、地图选择、Start/Stop。

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && source install/setup.bash
ros2 run drone_bringup dashboard
# 浏览器打开 http://127.0.0.1:8765/
# 可选参数：ros2 run drone_bringup dashboard -- --port 8765
```

等价于面板里预览的 `planner_sim.launch.py`；Stop 会结束该 launch 进程组。

## 工具脚本

仓库根目录 `scripts/`（编译后亦安装到 `share/drone_bringup/scripts/`）：

```bash
# 手动发目标
python3 scripts/send_goal.py --x 2 --y 1 --z 1.5

# 航点序列（square / circle / eight / list）
python3 scripts/waypoint_publisher.py --pattern square --side 2.0 --z 1.5

# 评测：导出 CSV + 图到 scripts/output/
python3 scripts/evaluate.py --duration 60 --goal-x 0 --goal-y 0 --goal-z 1.5
```

## 包结构

```
drone_ws/
├── src/
│   ├── drone_msgs/           # 自定义消息
│   ├── drone_dynamics/       # 刚体动力学（自研）
│   ├── drone_controller/     # 级联 PID + mixer（自研）
│   ├── drone_map/            # 固定种子点云地图
│   ├── drone_planner/        # Path A：自研 EGO 风格规划器
│   ├── ego_vendor/           # Path B：官方 ego_planner / map_generator 等
│   ├── gcopter_vendor/       # Path C：GCOPTER/MINCO（无 SO3 plant）
│   ├── drone_visualization/  # 机体 Marker + RViz 配置
│   └── drone_bringup/        # 六场景 + planner_sim / ego / gcopter launch
├── scripts/                  # 轨迹发生器、评测脚本
├── report/                   # 报告占位
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

## 与参考仓库的关系

- **MARSIM**：借鉴点云地图锁存发布与仿真器/算法解耦接口；**不参考**其简化动力学。
- **pengyu_sim（ROS1）**：借鉴 `quadrotor_dynamics` ↔ `uav_pid` 话题分层与 `map_generator` 发布模式；动力学/控制器自写；不接入 MAVROS 伪装层。
- **EGO-Planner（ego-planner-swarm `ros2_version`）**：仅移植/参考 `bspline_opt` + `path_searching` 规划管线；**弃用**原版 `so3_control` 与 `so3_quadrotor_simulator`。若环境依赖受阻，允许按论文公式自实现简化版（须在代码注释中说明）。

详见 `notes/reference_repos_notes.md` 与 `PLAN.md`。

## 已知问题

1. **评测脚本**：`evaluate.py` 对障碍点云做全量最近距离，大地图（dense_field 80 障碍）时可能较慢；输出目录默认为 `scripts/output/`。
2. **悬停硬指标**：位置误差 ≤ 0.3 m 需在场景跑通后用 `scripts/evaluate.py` 或 `stability_demo.launch.py` 核对，仅目视 RViz 不够。
3. **规划器（双路径）**：**Path A** 默认 `enable_bspline_opt: false`（稳定 dense A* 折线）；可把 `planner.yaml` 中该项改回 `true` 试 B-spline。**Path B** 跑官方 EGO（见上文）。
4. **多机 namespace**：`ros2 launch drone_bringup multi_drone.launch.py` 启动 `uav0`/`uav1` 两套节点；单机验收仍用无 namespace 的六场景 launch。
5. **代理克隆**：`reference_repos/ego-planner-swarm` 需本机代理（如 Clash `127.0.0.1:7897`）方可 `git clone`；仅作参考，不参与 colcon 编译。官方包已 vendor 到 `src/ego_vendor/`（`ego_planner`、`map_generator` 等）。
6. **抗扰加分**：控制器默认开启位置积分 + 扰动观测器（DOB）与 IMU 陀螺 LPF 融合；`stability_demo` 开风扰+IMU 噪声时误差应优于旧版（ki=0）设置。

## RViz

`drone_visualization` 提供 `rviz/drone.rviz`，显示：

- Grid / 障碍点云 `/map/obstacles` / 障碍 Marker
- 机体 Marker `/drone/body_markers`
- 实际轨迹 `/drone/path`、规划轨迹 `/planner/trajectory`
- Odometry、TF
- **2D Goal Pose** 工具发布到 `/drone/goal`

## License

MIT（课程作业工程）

# 参考仓库精读笔记（PLAN §0.5）

> 在建任何 ROS2 包之前完成。供报告「与参考仓库关系」章节使用。
> 精读路径：`reference_repos/MARSIM`、`reference_repos/pengyu_sim`，以及 EGO-Planner（`ros2_version` / 本地副本）。

---

## a) MARSIM（https://github.com/hku-mars/MARSIM）

### 定位
- **不是规划器**，是 **LiDAR 点云级真实感仿真器**。
- 核心技巧：用已有真实点云地图做碰撞/扫描仿真，而非手搓几何体 mesh。
- 论文明确：无人机运动仿真相对简单直接（simplified）——**不要参考其动力学实现**；作业要求动力学要素齐全，正是要补全 MARSIM 简化掉的部分。

### 可借鉴
| 模块 | 启示 |
|------|------|
| `map_generator` | PointCloud2 全局点云锁存发布、降采样参数；作业改为固定种子自生成障碍点云 |
| `local_sensing` (`pcl_render_node`) | 全局点云 + 位姿 → 局部可见点云；实现加分项「局部感知」 |
| `test_interface` / 话题解耦 | RViz Goal → 控制；仿真器发 odom；外部规划/SLAM 插拔——与我们 `/drone/goal`+`/drone/odom` 契约同构 |
| launch 拼装 | 地图 / 感知 / 动力学 / 控制各自独立节点 |

### 不要参考
- `mars_drone_sim` 动力学、`cascadePID` 整条 RPM 链路
- 用仓库真实 PCD 顶替作业自生成地图
- 整包 OpenGL/GPU LiDAR 图案（过重）

### 关键路径
- `map_generator/src/map_publisher.cpp`
- `local_sensing/src/pointcloud_render_node.cpp`
- `test_interface/`、`mars_drone_sim/`（边界认知，不照搬）

---

## b) pengyu_sim（https://gitee.com/potato77/pengyu_sim）

### 定位
- **ROS1 catkin** 工程，给实验室 `Sunray_v2` 当仿真替身（可伪装 MAVROS/PX4）。
- 只学架构思路，**不能照抄 ROS1 语法/catkin**。

### README TODO 关键结论
> `map_generator` 和 `local_sensing` 目前主要从 marsim 复制过来，**动力学部分已经重写**。

说明：地图/感知层移植 MARSIM 是被认可的正常操作；红线卡在 **动力学与控制器**。

### 模块映射

| pengyu_sim | 本工程 | 参考强度 |
|------------|--------|----------|
| `quadrotor_dynamics_node` | `drone_dynamics` | 话题契约对照；公式自写 |
| `uav_pid_controller` | `drone_controller` | 参数拆分方式；**必须自写** |
| `map_generator` | `drone_map` | 点云发布/降采样 |
| `local_sensing` | `drone_map` 局部感知加分 | 可简化为半径裁剪 |
| `waypoint_generator` | `scripts/` | 圆/8字/航点方向确认 |
| `test_interface` | scripts + RViz Goal | 解耦思路 |
| （无规划包） | `drone_planner`（EGO） | 规划是我们补上的 |

### 不要移植
- `fake_mavros_bridge_node`、`px4_control_sim_node`（MAVROS 伪装层，与作业无关）

### yaml 拆分启示
- `dynamics.yaml` + `imu.yaml`（动力学）与 `controller.yaml`（增益/限幅）分文件
- dynamics 与 controller 的物理参数（mass、arm_length、kF/kM）必须一致

---

## c) EGO-Planner（ZJU-FAST-Lab / ego-planner-swarm `ros2_version`）

### 本地副本
- 已克隆：`reference_repos/ego-planner-swarm`（分支 `ros2_version`，仅作参考，不直接 roslaunch 整仓）。
- 克隆命令：`git clone -b ros2_version https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git`

### 为什么选它
- 官方有 ROS2 分支，移植成本低于 Fast-Planner / EGO-v2。
- **红线**：弃用其 `so3_control`、`fake_drone`/`so3_quadrotor_simulator`。
- 可移植：`bspline_opt`、`path_searching`、`plan_env`（对接我们点云）、FSM 思路重写。

### 本工程策略
1. 优先参考/移植算法核心进 `drone_planner`（namespace/话题按本工程契约）。
2. 若依赖/编译困难 → 按论文自实现简化版（kinodynamic A* + B 样条梯度优化 + LBFGS-Lite），注释说明原因。
3. 对外话题严格：`/drone/goal`、`/drone/odom`、`/map/obstacles` → `/planner/trajectory`、`/planner/local_goal`、`/planner/status`。

---

## 对本工程的总体决策（留痕）

1. **动力学 + 控制器**：纯自研 C++，参数 yaml 化。
2. **地图**：自研固定种子点云生成（sparse / dense_field / narrow_corridor）+ 连通性检测；发布形态参考 map_generator。
3. **局部感知**：距离半径裁剪（加分），不强制完整 pcl_render。
4. **规划**：EGO 风格前端搜索 + B 样条优化，对接自有话题；不用原仿真器/控制器。
5. **坐标系**：ENU，`map` → `base_link`。

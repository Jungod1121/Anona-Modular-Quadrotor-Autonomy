---
name: FUEL-style exploration
overview: 在不改动植物端与现有 Path A/B/C 源码的前提下，把官方 FUEL 只读克隆到 reference_repos，并新增一条可开关的「FUEL 式探索」Path D：雾面感知 + frontier/视点 FSM + 复用 homemade 轨迹契约；为后续真移植官方核预留入口。
todos: []
isProject: false
---

# FUEL 式探索（安全接入）

## 默认范围（按「试试 + 别坏源码」）

**不做**完整 Fast-Planner/FUEL ROS2 移植（官方 [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) 仅为 Melodic/Noetic + 自带 SO3 仿真）。

**做**分阶段 Path D：

1. 官方树只读放进 `reference_repos/FUEL/` + `COLCON_IGNORE`（与 [reference_repos/README.md](reference_repos/README.md) 同策），**禁止**改官方文件。
2. 在 `src/` 新增你们自己的探索包（新代码），驱动**现有** `/drone/*` + `/planner/*` 契约；**零改动** [drone_dynamics](src/drone_dynamics)、[drone_controller](src/drone_controller)、已有 ego/gcopter vendor 算法文件。

若以后要真 port `fuel_planner/`，再另开 vendor，不碰本阶段已锁死的植物。

## 架构

```mermaid
flowchart LR
  subgraph plant [Unchanged plant]
    Dyn[dynamics_node]
    Ctl[controller_node]
  end
  subgraph fuel_d [New Path D]
    Map[drone_map or official cloud]
    Sense[local_sensing fog]
    Expl[exploration_fsm]
    Plan[homemade planner_node]
  end
  Map --> Sense
  Sense -->|"/map/obstacles local"| Plan
  Expl -->|"sequential /drone/goal"| Plan
  Plan --> "/planner/trajectory_cmd"
  Dyn --> "/drone/odom"
  "/drone/odom" --> Expl
  "/drone/odom" --> Plan
  Ctl --> Dyn
```

语义对齐官方 FUEL：RViz **2D Goal 只作「开始探索」触发**，后续目标由 frontier/视点选择器生成，而不是单次点到点。

## 实现要点

### 1. 只读参考仓
- `git clone https://github.com/HKUST-Aerial-Robotics/FUEL.git reference_repos/FUEL`
- 确保 `reference_repos/COLCON_IGNORE` 已在（已有则不动）
- 更新 [reference_repos/README.md](reference_repos/README.md) + [notes/reference_repos_notes.md](notes/reference_repos_notes.md) 一小节：定位、可借鉴模块（FIS/TSP 视点）、**禁止**接 SO3/`uav_simulator`

### 2. 新包 `drone_exploration`（仅新增）
建议路径：`src/drone_exploration/`（ament_python 或 C++ 轻节点均可；优先 Python FSM 便于调试）

| 模块 | 作用 |
|------|------|
| `local_sensing_node` | 输入全局点云 + `/drone/odom`，输出机体周围球体/锥形「已观测」障碍云到 `/map/obstacles_local`（或桥到 planner 的 map topic）；未观测体积保持 unknown——这是探索前提，不能直接灌满图 latched 全局云给 planner |
| `frontier_extractor` | 在局部占据栅格上提 frontier 聚类（体素级），近似 FUEL active_perception 思路，算法自写，可读参考仓对照 |
| `exploration_fsm` | IDLE → TRIGGERED（收 `/drone/goal`）→ PICK_VIEW → SEND_GOAL → WAIT_ARRIVE → … → FINISH；发布连续 `/drone/goal`；状态 `/planner/status` 或自建 `ExplorationStatus` |
| 地图边界 | 参数 `box_min/max`（对齐 FUEL exploration.launch 的探索盒） |

**规划后端**：默认挂 **homemade** `planner_node`（已原生发契约）；local map 通过 launch remap `map_topic:=/map/obstacles_local`。不改编译过的 ego/gcopter 源。

### 3. Launch / 路由（只加文件）
- 新 [src/drone_bringup/launch/fuel_explore.launch.py](src/drone_bringup/launch/fuel_explore.launch.py)：`map_stack`（全局云仅给 sensing）+ sensing + exploration_fsm + dynamics + controller + homemade planner（local map）+ 可选 RViz
- 扩展 [planner_sim.launch.py](src/drone_bringup/launch/planner_sim.launch.py)：`planner:=fuel_explore`
- Dashboard：[dashboard_server.py](src/drone_bringup/drone_bringup/dashboard_server.py) + UI 增加 Path D 选项；地图默认 `dense_field` / office 式自制图

### 4. 明文不动清单
- 不修改 `reference_repos/FUEL/**`
- 不改 `src/ego_vendor/**` 算法、`src/gcopter_vendor/**` 优化核（除非仅 docs）
- 不启 `so3_*` / `fake_drone` / FUEL `uav_simulator`
- 植物话题名保持 [PLANNERS.md](src/drone_bringup/PLANNERS.md) 契约

### 5. 验收
- `ros2 launch drone_bringup planner_sim.launch.py planner:=fuel_explore use_rviz:=true`
- RViz 点一次 2D Goal → 连续换视点探索，黄线/`trajectory_cmd` 有输出，飞控跟随
- 跑完后 `bash scripts/cleanup_sim.sh`；确认 A/B/C 烟雾仍可用（可选抽 1 组 `homemade×dense_field`）

## 后续（本计划不实施）
官方 `exploration_manager` + Fast-Planner 整包 ROS2 vendor 与 NLopt/LKH 依赖，单独里程碑；本阶段笔记写清差异，避免报告宣称「已接入官方 FUEL 二进制」。

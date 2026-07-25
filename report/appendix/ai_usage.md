# AI 使用说明

按课程要求如实记录 AI 辅助过程。完整 Plan 原文见本附件 [`ai_plans/`](ai_plans/)（自 `ai_usage_records/00`–`08` 拷贝）及仓库 [`ai_usage_records/`](../../ai_usage_records/)。

---

## 1. 使用了哪些 AI 工具

| 工具 | 模型 | 用途阶段 |
|------|------|----------|
| **Claude Code** | （Claude Code 会话） | 早期工程执行计划：Ubuntu 22.04 + ROS 2 Humble 下搭动力学/控制/地图/规划接入红线（见 `00_workspace_PLAN_md`） |
| **Cursor** | **Grok 4.5**（Cursor Grok 4.5） | 后续 Plan / Agent 联调：双规划器、多后端切换、地图与控制台、Path D–F、架构升级、控制台 UI、报告修改（见 `01`–`08` 及近期 Agent） |

下文「Cursor」均指 **Cursor + Grok 4.5**，不单独省略模型名。

---

## 2. 关键 prompt / 交互摘要（≥8 条）

以下 9 条直接整理自各目录 `plan.md`（用户意图 + 定稿决策），不另编造。全文路径：`report/appendix/ai_plans/NN_…/plan.md`。

1. **Claude Code / `00_workspace_PLAN_md`**  
   **意图：** 按作业要求从零搭可编译可运行的 ROS2 四旋翼仿真，覆盖硬性项与尽量多的加分项。  
   **结果：** 定稿执行计划——C++ plant、点云地图、EGO 只移植规划核并禁止 SO3/fake_drone；动力学与控制器必须自研。

2. **Cursor Grok 4.5 / `01_dual_planner_ego_map`**  
   **意图：** 保留原 `drone_planner` 一条 launch，再加第二条：官方 EGO 规划 + 官方 map_generator，仍用本仓动力学与级联 PID。  
   **结果：** Path A（homemade）与 Path B（ego）双路径，plant 话题契约不变。

3. **Cursor Grok 4.5 / `02_multi_planner_switch`**  
   **意图：** plant 钉死，用 launch 切换规划后端；接入较新且有 ROS2 路径的 GCOPTER/MINCO。  
   **结果：** 形式化 backend 切换，Path C（gcopter）接到同一 `/planner/local_goal` 与 `trajectory_cmd`。

4. **Cursor Grok 4.5 / `03_maps_goals_dashboard`**  
   **意图：** 修 mockamap 起终点/密度、自研地图选不中、RViz 只剩蓝线；目标以 RViz 为主并支持 cruise_height；网页控制台改简洁风格。  
   **结果：** 地图目录与控制台可用，黄蓝轨迹与 RViz 2D Goal 对齐。

5. **Cursor Grok 4.5 / `04_fuel_style_exploration`**  
   **意图：** 不改 plant 与现有 A/B/C 源码，增加可开关的 FUEL 风格探索路径。  
   **结果：** Path D：雾面感知 + frontier/视点 FSM + 复用 homemade 轨迹契约；官方 FUEL 只读参考。

6. **Cursor Grok 4.5 / `05_three_planner_integration`**  
   **意图：** 在同一 plant+map 契约下继续加规划器。  
   **结果：** 刷新 Path C；加 Path E（MIGHTY）与 Path F（Fast-Planner 移植端口），经桥接进本仓控制器。

7. **Cursor Grok 4.5 / `06_review_led_architecture_upgrade`**  
   **意图：** 对照 pengyu_sim/MARSIM 做审查式升级：巩固适配契约、正交化规划族、标准化地图/评测/可视化与仪表盘。  
   **结果：** 架构分层与评测脚本加固，六项验收场景保留。

8. **Cursor Grok 4.5 / `07_glass_revert_xy_track`**  
   **意图：** 控制台去掉 WebGL LiquidGlass，改回 CSS 磨砂玻璃；每页常显俯视 XY 轨迹。  
   **结果：** 仪表盘可读性恢复，轨迹面板依赖现有 odom 轮询。

9. **Cursor Grok 4.5 / `08_map_heading_layout_fix`**  
   **意图：** 修点云投影导致障碍/轨迹消失、侧栏把主界面顶下去、地图可缩放，并改为航向朝上。  
   **结果：** 地图舞台可拖拽缩放，障碍与轨迹重新可见，yaw 驱动朝向。

---

## 3. AI 帮完成了哪些模块

| 模块 | AI 角色 | 说明 |
|------|---------|------|
| Launch / `drone_bringup` 注册表 | Cursor Grok 4.5 辅助编写与联调 | `planner_sim`、各 Path launch、地图 catalog |
| 第三方规划桥接 | 辅助移植与话题 remap | EGO / GCOPTER / MIGHTY / Fast-Planner；剥 SO3/fake_drone |
| 地图适配 `map_adapter` | 辅助 | 双话题统一、种子可复现 |
| 任务控制台（Web/原生） | 辅助 UI 与地图页 | 见 `03`/`07`/`08` |
| 评测脚本与报告排版 | 辅助 | acceptance、批量矩阵、LaTeX |
| **动力学方程 / 混控 / 级联 PID** | **不以 AI 外包实现为准** | 公式与节点以本仓 `drone_dynamics` / `drone_controller` 为准；AI 仅查阅与改写草稿，最终由人工确认 |

---

## 4. 自己确认或修改的核心公式与接口

- **Plant 话题契约：** `/drone/odom`、`/drone/goal`、`/drone/motor_rpm_cmd`、`/drone/imu`、`/drone/path`、`map→base_link`；规划出 `/planner/local_goal`、`/planner/trajectory_cmd`、`/planner/trajectory`。
- **推力/力矩：** $F_i=k_F\omega_i^2$，X 型分配矩阵 $A$，平动与 $I\dot\omega=\tau-\omega\times(I\omega)$，电机一阶滞后与 RPM 限幅。
- **控制器：** 位置→加速度（PD/I + 限速限加速度）→倾转受限推力方向 → 姿态/角速度 PD → mixer 求四路 RPM。
- **接入红线：** 第三方规划可引用，禁止用上游 SO3 / fake_drone 替换本仓动力学与控制层。
- **安全阈：** 验收避障约 $0.30\,\mathrm{m}$，窄通道约 $0.35\,\mathrm{m}$；批量矩阵 $\ge0.08\,\mathrm{m}$。

---

## 5. AI 生成过的错误与如何修正（实录）

1. **B 样条几乎必拒（Path A / 早期联调）**  
   AI/草稿把 B 样条验收放在与 A* 相同的膨胀栅格上，稠密图弦切即失败，总回退折线。  
   **修正：** 验收改看 raw 障碍 + 机体半径；引导点加密。对话见 `01_dual_planner_ego_map/dialogue.md`。

2. **GCOPTER 前端失败却走直线穿墙**  
   2D A* 找不到路径时仍 `push_back(start/goal)`，优化器当成功轨迹跟踪。  
   **修正：** 前端 fail-closed（无解则清空路径并失败），并调迷宫等地图膨胀参数。

3. **install 与 source 目录不一致的假故障**  
   AI 改 catalog 后未同步 install，出现「同一 launch 名、起飞点不同」。  
   **修正：** `colcon build` + 重新 `source install/setup.bash`，核对窄通道等姿态覆盖。

评测表中的 FAIL / 人工「差」予以保留，不要求 AI「把结果写好看」。

---

## 6. 如何验证动力学、控制器与 ROS 2 topic

1. **编译与单元测试：** `colcon build`；控制器 mixer 等 gtest（`drone_controller`）。
2. **六项验收：** `scripts/run_acceptance.py` / 对应 launch；终误 $\le0.3\,\mathrm{m}$，避障间隙大于设定阈（见 `acceptance_report.md`）。
3. **话题抽查：** `ros2 topic echo` / `hz` 检查 `/drone/odom`、`/drone/motor_rpm_cmd`、`/planner/local_goal`、`/planner/trajectory_cmd`。
4. **RViz：** 黄线规划、蓝线飞过、障碍云、2D Goal Pose → `/drone/goal`。
5. **矩阵对照：** 批量脚本 + 人工场景表（见 `planner_batch_comparison.md`）；失败格与附件一致。

---

## 归档位置

- 摘要本文：`report/appendix/ai_usage.md`
- Plan 拷贝：`report/appendix/ai_plans/`（[`INDEX.md`](ai_plans/INDEX.md)）
- 完整归档：`ai_usage_records/`（含 `meta.json`、原始 `.plan.md`）

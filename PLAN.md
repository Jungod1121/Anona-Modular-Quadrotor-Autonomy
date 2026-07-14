# 无人机仿真作业 — 执行计划（供本地 Claude Code / Ubuntu22.04+ROS2 Humble 使用）

> 本文档是给 Claude Code 的**执行说明**，不是报告。目标：搭建一个可编译、可运行、满足全部硬性要求+尽可能多加分项的 ROS2 四旋翼仿真工程。报告(PDF)、`ai_usage.md`、演示视频不在本轮任务范围内，但过程中的关键决策请留痕（commit message / 代码注释 / TODO），方便以后写报告和 ai_usage.md。

---

## 0. 作业原始要求摘要（对照检查用，见完整版 `无人机仿真作业要求.md`）

- 参考 pengyu_sim / MARSIM，用 ROS2 + AI辅助编程**从零搭建**，禁止简单运行/包装原仓库。
- 必须模块：① 无人机动力学节点 ② 控制器节点 ③ 可视化（RViz2 或地面站）。
- 选做但加分/建议做：④ 地图模块 ⑤ 避障/规划模块。
- 六个验收场景：悬停 / 单目标点 / 多目标点(3~4点航线) / 静态避障(≥5障碍物) / 狭窄通道绕行(展示规划路径+实际轨迹) / 稳定性展示(误差曲线等)。
- 硬指标：悬停误差 ≤0.3m；避障时实际轨迹与障碍物最小距离 > 安全距离(0.3~0.5m)。
- 工程结构要模块清楚、能一键启动。
- 十项加分项（见原文件第六节），本次全部纳入范围。
- 最终提交：Git仓库 + PDF报告(6~10页) + 演示视频 + `ai_usage.md`（≥8条关键prompt摘要等）—— **这三项本轮不做**，只做代码工程本身。

---

## 0.5 动手前必做：精读两个参考仓库（新增，真正的第一步，先于所有代码）

在建任何ROS2包之前，Claude Code 必须先做这一步，并把读后笔记记在 `notes/reference_repos_notes.md` 里，方便以后写报告"与参考仓库关系"章节：

### a) MARSIM（`https://github.com/hku-mars/MARSIM`）—— 已核实信息，可直接参考
- **不是规划器，是LiDAR真实感仿真器**。核心技巧：直接用**已有真实点云地图**做碰撞/扫描仿真，而不是手搓几何体——`drone_map`可以借鉴这个思路：不一定非要手写圆柱/球体采样器，也可以直接生成/加载点云数据作为障碍物真值。
- 官方论文明确说它的**无人机运动仿真部分是"相对简单直接"（simplified）的**，不是它的重点。这反过来印证了本次作业为什么专门强调"动力学节点要素齐全"——这正是MARSIM简化掉、我们要补全做扎实的部分。**不要参考它的动力学实现**。
- 值得学的是**接口解耦设计**：`test_interface`包用RViz 3D Goal工具发目标点、仿真器发里程计，外部SLAM/规划模块直接对接——跟我们`/drone/goal`+`/drone/odom`的契约设计思路一致，重点学"仿真器与算法解耦"的做法，不是照抄话题名。

### b) pengyu_sim（`https://gitee.com/potato77/pengyu_sim`）—— 已拿到README，核实后的结论

- **是ROS1(catkin_make)工程**，不是ROS2。定位是给该实验室自己的控制框架`Sunray_v2`当"仿真替身"（伪装成MAVROS/PX4接口）。移植时只学架构思路，不能照抄ROS1语法/catkin写法。
- **不需要学、不要搬**：`px4_control_simulator`里的`fake_mavros_bridge_node`、`px4_control_sim_node`——这是伪装MAVROS接口给`Sunray_v2`用的适配层，跟本作业无关，接了反而显得画蛇添足。
- **关键结论（修正了此前的保守判断）**：仓库TODO里明说"`map_generator`和`local_sensing`包目前主要从`marsim`中复制过来，动力学部分已经重写"——说明该实验室的习惯做法是**地图/感知层可以直接移植MARSIM的，但动力学必须自己重写**。这意味着"不能简单包装原仓库"这条红线主要卡在**动力学**（以及控制器），地图/感知层移植MARSIM是被认可的正常操作，不算偷懒。
- **可以直接对照参考的模块映射**（帮助校准我们自己的设计）：
  - `quadrotor_dynamics_node`（订阅电机RPM→发布odom/imu）↔ 我们的`drone_dynamics`，话题契约思路一致，可交叉验证设计是否合理。
  - `uav_pid_controller`（位置/姿态目标→电机RPM）↔ 我们的`drone_controller`，**必须自己写**，只能参考其参数拆分方式（yaml分dynamics/imu/pid等）。
  - `map_generator`(读PCD发布全局点云) + `local_sensing`(`pcl_render_node`，全局图+位姿→局部可见点云/深度图，支持动态障碍物、多机互相渲染) ↔ 我们要做的地图+局部感知（详见第2.2节调整后的策略）。
  - `waypoint_generator`(点/圆/8字/分段航点) ↔ 加分项"轨迹输入"，方向确认无误。
  - `test_interface`(简单目标点/轨迹→控制输入) ↔ `scripts/`轨迹发生器思路，方向确认无误。
  - **它自己没有规划/避障模块**，规划是交给`Sunray_v2/planning/sunray_planning`做的——证明"规划器"从来不是这类仿真工作区自带的东西，我们自己接EGO-Planner这个决策不冲突、是必须做的补充。

---

## 1. 本轮确定的技术选型

| 决策项 | 选择 |
|---|---|
| 核心语言 | **C++**（动力学/控制器/规划器），Python 仅做工具脚本（轨迹发生器、评测、绘图） |
| 地图形式 | **PointCloud2 点云地图**，固定随机种子生成 ≥5 个障碍物簇，贴近 MARSIM 风格 |
| 规划器 | **移植/复现浙大 FAST-Lab 高飞团队 EGO-Planner**（ESDF-free 梯度优化局部规划器） |
| 可视化 | **RViz2** 标准方案 |
| ROS2 版本 | Ubuntu 22.04 + ROS2 Humble |

---

## 2. EGO-Planner 的处理方式（重点，需要谨慎设计避免"简单包装"）

### 2.0 为什么选EGO-Planner（合适性说明 + 主流方法对比）

**当前四旋翼局部规划主流脉络**：
1. 采样类(RRT*/PRM)：无需建模但轨迹质量差、非时间最优，现多做全局粗规划或对比基线，不适合作主力。
2. **"前端搜索+后端优化"两段式（当前主流）**：Fast-Planner(2019,ESDF-based) → **EGO-Planner(2021 RA-L,ESDF-free)** → TGK-Planner/RAPTOR(拓扑引导/感知主动性改进) → MINCO/EGO-Planner-v2(2022起,轨迹表示换成MINCO多项式,优化更快)。
3. 安全走廊+凸优化：全局最优性强但走廊构建在密集障碍场景开销大，工程复杂度高。
4. 学习式方法(2024-2025新兴，如NavRL/You-Only-Plan-Once/SAGA)：能处理动态障碍物但训练成本高、可解释性差，与作业要求"自己确认核心公式"这一条冲突。

**方案对比**：

| 方案 | 优点 | 问题 |
|---|---|---|
| Fast-Planner | 经典、文档全 | 无官方ROS2版本，ESDF拖慢速度 |
| **EGO-Planner（选定）** | **官方有ROS2分支(ego-planner-swarm/ros2_version)**，工程成熟、静态点云场景够用 | 非2025年最快，但对课程作业完全够用 |
| EGO-Planner-v2(MINCO) | 速度/轨迹质量更好 | 属于Swarm Playground大平台，无官方ROS2版本，移植成本对一学期作业不现实 |
| 2024新作(Primitive-Planner等) | 论文新、更轻量 | 小组维护、文档少，大概率也无ROS2版本 |
| RL学习式 | 能处理动态障碍物 | 训练成本高，难以体现"自己确认核心公式" |

**结论**：EGO-Planner在"先进程度、工程成熟度、ROS2可落地性"三者平衡最好，尤其"官方ROS2分支"这一点在整条技术脉络里目前唯一。若时间充裕，可将"B样条优化升级为MINCO表示"作为加分项里的"自由发挥"，但不作为主线（风险收益比不划算）。

- 现实可行性：EGO-Planner（`ego-planner-swarm`仓库）**官方自带 `ros2_version` 分支**，这在同类学术规划器仓库里比较少见（比如其前身Fast-Planner就没有官方ROS2版本），移植成本明显低于其他"先进规划器"，是当前最方便落地到ROS2 Humble的选择。
- **红线（唯一不能碰的部分）**：EGO-Planner自带的 `so3_control`(姿态控制器)和 `fake_drone`/`so3_quadrotor_simulator`(动力学仿真)**必须完全弃用**，因为动力学与控制器是本次作业的核心考核点，不能被替换/外包。可以移植的只是它的**前端搜索+B样条优化+感知建图**这条规划管线。
- 单机使用方式：`ego-planner-swarm` 面向集群设计，单机场景下按官方说明将 `drone_id` 设为 0 即可，这是官方支持的标准用法，不算额外破解。
- 建议第一步先把 `bspline_opt`+`path_searching`+`plan_env` 三个包单独拉出来试编译（可能需要装NLopt/Armadillo），确认在Humble下能过编译再往下接，避免中途因依赖问题返工。

### 2.1 可用资源
- 原版仓库（ROS1/catkin）：`https://github.com/ZJU-FAST-Lab/ego-planner-swarm`
- **官方已有 ROS2 分支**：同一仓库的 `ros2_version` 分支，已经是面向 ROS2 的实现，可作为算法与接口的直接参考/移植源，比手工翻译 ROS1→ROS2 更可靠。
- 核心算法包（与 ROS 版本无关的纯算法/数学部分，是我们要复用的重点）：
  - `bspline_opt`：均匀 B 样条表示 + 基于梯度的轨迹优化（`bspline_optimizer.h/.cpp`，核心函数 `BsplineOptimizeTrajRebound`、`calcDistanceCostRebound`(碰撞代价，ESDF-free核心)、`calcSmoothnessCost`等）
  - `path_searching`：kinodynamic A* 前端路径搜索
  - `plan_env`：占据地图/点云接入部分（**不整体照搬**，改为对接我们自己的 `drone_map` 点云）
  - `plan_manage` / `ego_replan_fsm`：规划状态机（INIT/WAIT_TARGET/GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/EMERGENCY_STOP），这部分逻辑值得参考但要**重写成对接我们自己的话题契约**
- 依赖：原版用到 NLopt / LBFGS-Lite（单头文件，轻量，可直接带入我们工程）、Armadillo（主要给 uav_simulator 用，若不用其仿真器可尝试去掉这个依赖，减少环境负担）

### 2.2 我们要做的整合策略（不是"简单运行原仓库"）
1. **不直接 `roslaunch`/克隆整包运行**。而是：把 `bspline_opt`、`path_searching` 这两个相对独立、不依赖 ROS 太深的算法包，参考 `ros2_version` 分支的写法，**移植进我们自己的 `drone_planner` 包**（namespace/CMake/接口全部按我们工程规范重写）。
2. **地图与感知层采用"两层"策略（重要，结合pengyu_sim实际做法后修正）**：
   - **世界真值层（我们自己生成，保证可验收）**：不必从零手写点云采样器，可**移植MARSIM/参考pengyu_sim的`map_generator`包**（读取/发布全局`PointCloud2`点云的逻辑），但输入源改成**我们自己用固定随机种子程序生成的障碍物点云**（≥5个障碍物簇、位于起点终点之间），而不是加载现成的PCD实景地图——这样既复用了成熟的点云发布/管理代码，又保证了"老师改种子/改参数能验证是你自己生成的"这一硬性要求。
   - **感知/建图层（移植MARSIM/pengyu_sim的`local_sensing`即`pcl_render_node`）**：这一层负责把全局点云+当前位姿渲染成局部可见点云/深度图，供规划器使用，pengyu_sim的TODO里明确写了这部分"主要从MARSIM复制过来，动力学部分已经重写"——说明该实验室认可这种移植方式，我们照做即可，同时这一层直接实现了加分项"局部感知范围"。
   - **不要移植**：pengyu_sim里`px4_control_simulator`下的`fake_mavros_bridge_node`、`px4_control_sim_node`——这是伪装MAVROS给`Sunray_v2`用的适配层，跟本作业无关。
   - 两层的关系：`drone_map`(固定种子生成障碍物点云，我们自己的部分) → 移植的`local_sensing`感知渲染(可选，加分项) → `plan_env`占据栅格转换(移植自EGO-Planner) → `path_searching`+`bspline_opt`(移植自EGO-Planner)。若时间紧张，`local_sensing`这一层可以跳过，直接把`drone_map`的全局点云喂给EGO-Planner的`plan_env`，不影响核心验收，只是少一个加分点。
3. `plan_manage`/FSM 逻辑**参考其状态机设计思路，但用我们自己的话题契约重写**：输入 `/drone/goal` + `/drone/odom` + `/map/obstacles`，输出 `/planner/trajectory`(nav_msgs/Path，采样自优化后B样条) + `/planner/local_goal`(喂给我们自己的controller的滚动目标点) + `/planner/status`。
4. **完全不用**原版的 `so3_control`（他们自带的姿态控制器）和 `fake_drone`/`so3_quadrotor_simulator`（他们自带的动力学仿真）——这两块必须是我们自己从零写的 `drone_dynamics` / `drone_controller`，这是作业的核心考核点，不能替换。
5. 移植过程中产生的**每一处修改/替换/简化**都要在代码注释里写明"参考自 ego-planner (文件路径)，改动点：xxx"，方便后续写报告的"与参考仓库关系"章节和 `ai_usage.md`。
6. 如果移植 `bspline_opt`/`path_searching` 遇到编译困难（NLopt/Armadillo 环境问题、Eigen版本差异等），**允许降级为自己实现的简化版**（前端 kinodynamic A* + 后端自己写的梯度下降优化 B 样条控制点，代价函数按 EGO-Planner 论文公式自己实现），但要在代码注释/commit里说明"因为环境原因从移植改为参考论文自实现"。这样无论走哪条路都不算"简单包装"，都算"自己的ROS2工程，参考了先进方法"。

### 2.3 给 Claude Code 的具体执行建议
```
1. git clone -b ros2_version https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git /tmp/ego_planner_ref  (仅作参考，不放进最终仓库/或作为 git submodule + 明确声明来源)
2. 阅读 /tmp/ego_planner_ref 里 bspline_opt, path_searching 两个包的 CMakeLists.txt 和依赖，确认 Humble 下能否直接编译
3. 若能编译：以此为基础，改包名为我们自己的命名空间，接入我们的话题；
   若不能编译（缺依赖/API变化）：记录具体报错，决定是修依赖还是转为自实现简化版
4. 无论哪种方式，最终 drone_planner 包对外的话题契约必须严格符合本计划第3节的定义，不能依赖原仓库的话题名/消息类型
```

---

## 3. 系统架构与话题契约

```
/drone/goal (PoseStamped) ──► drone_planner ──► /planner/trajectory (nav_msgs/Path，B样条采样)
/drone/odom ─────────────────►     │      └───► /planner/local_goal (PoseStamped，滚动目标)
/map/obstacles (PointCloud2) ─►    │      └───► /planner/status (drone_msgs/PlannerStatus)
                              drone_map                          │
                                                          drone_controller ──► /drone/motor_rpm_cmd (drone_msgs/MotorCommand)
                                                                                        │
                                                                                        ▼
                                                                                drone_dynamics ──► /drone/odom, /drone/imu, /tf, /drone/path
```

| Topic | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/drone/motor_rpm_cmd` | `drone_msgs/MotorCommand`(float64[4]) | controller | dynamics |
| `/drone/odom` | `nav_msgs/Odometry` | dynamics | controller, planner, viz |
| `/drone/imu` | `sensor_msgs/Imu` | dynamics | (噪声/滤波加分项用) |
| `/tf` | map→base_link | dynamics | rviz |
| `/drone/path` | `nav_msgs/Path` | dynamics | rviz, 评测脚本 |
| `/drone/goal` | `geometry_msgs/PoseStamped` | 用户/RViz 2D-3D Goal / Python轨迹发生器 | planner |
| `/map/obstacles` | `sensor_msgs/PointCloud2` | drone_map | planner, rviz |
| `/map/obstacles_markers` | `visualization_msgs/MarkerArray` | drone_map | rviz |
| `/planner/trajectory` | `nav_msgs/Path` | planner | controller, rviz |
| `/planner/local_goal` | `geometry_msgs/PoseStamped` | planner | controller |
| `/planner/status` | `drone_msgs/PlannerStatus` | planner | rviz / 评测脚本 / 地面站 |

**坐标系**：`map`(世界系，ENU) → `base_link`(机体系，x前 y左 z上)。所有障碍物/目标点/里程计统一在 `map` 系下。

---

## 4. 包结构（要求 Claude Code 严格按此建目录）

```
drone_ws/
├── PLAN.md                  # 本文件
├── README.md                # 编译/运行说明，六场景一键启动指令
├── src/
│   ├── drone_msgs/          # MotorCommand.msg, PlannerStatus.msg
│   ├── drone_dynamics/      # C++ 刚体动力学节点（核心考核点，纯自研）
│   ├── drone_controller/    # C++ 级联PID + mixer（核心考核点，纯自研）
│   ├── drone_map/           # C++ 点云地图节点，固定种子，≥5障碍物+局部感知半径(加分)
│   ├── drone_planner/       # C++ EGO-Planner移植/复现（前端A* + B样条梯度优化 + FSM）
│   ├── drone_visualization/ # RViz配置 + 机体marker发布
│   └── drone_bringup/       # 顶层launch(六场景) + 全部yaml参数
├── scripts/                 # Python: 轨迹发生器(圆/8字/waypoint list)、评测脚本、gtest配套
└── report/                  # 占位，本轮不处理
```

---

## 5. 各模块技术要点（Claude Code 实现细则）

### 5.1 drone_dynamics（纯自研，禁止参考EGO-Planner的仿真器）
- 状态：位置p、速度v、姿态四元数q(w,x,y,z)、机体角速度ω、4个电机转速。
- 电机一阶响应：`ω̇_i=(ω_cmd,i-ω_i)/τ_motor`；推力 `F_i=k_F·ω_i²`；反扭矩 `M_i=k_M·ω_i²`。
- X型布局力矩分配矩阵：由电机臂长L、安装角、自旋方向构造 `[T,τx,τy,τz]ᵀ = A·[ω1²,ω2²,ω3²,ω4²]ᵀ`，四元数用 `q_dot=0.5*q⊗[0,ω]`积分并每步归一化。
- 平动：`m·v̇=R·[0,0,T]+m·g+F_ext`；转动：`I·ω̇=τ-ω×(Iω)`。
- 电机上下限、固定步长积分器（建议500Hz内部积分，100~200Hz发布odom）。
- 加分项预留接口：风扰(常值+正弦阵风力，开关可配)、IMU高斯噪声+bias随机游走(开关可配)。
- 所有参数（质量/惯量/kF/kM/τ_motor/限幅/风扰/噪声开关）走 `declare_parameter` + yaml，不写死。

### 5.2 drone_controller（纯自研）
级联：位置PID→期望加速度(含重力前馈) → 期望roll/pitch+总推力(小角度近似) → 姿态/角速度PD→三轴力矩 → mixer(用与dynamics相同的分配矩阵求逆) → 4个电机RPM。
- 各级限幅：推力范围、姿态角范围、RPM范围；目标过远时对期望速度/加速度做trapezoidal限速限幅。
- controller 直接订阅 `/planner/local_goal`（优先）或 `/drone/goal`（若planner未运行，做fallback，方便单独测试controller/dynamics而不依赖planner）。

### 5.3 drone_map

**分场景多模式生成（重要调整：障碍物数量从"≥5个"的最低门槛提升到两位数，老师明确要求）**：

`drone_map` 不用一套参数覆盖所有场景，按 `map_mode` 参数支持三种模式：

| 场景 | map_mode | 障碍物数量 | 场地尺寸 | 说明 |
|---|---|---|---|---|
| 悬停/单目标点/多目标点 | `sparse` | 0（或1~2个不挡路的装饰性障碍物） | 沿用小场地(约8×8×2.5m) | 纯测动力学+控制器，不掺避障变量 |
| 静态避障(场景4) | `dense_field` | **80个**(50~100区间取中) | 18m(飞行方向)×10m(宽)×2.5m(高) | 老师明确要求的两位数密度主战场 |
| 狭窄通道(场景5) | `narrow_corridor` | dense_field基础上+确定性构造的两道"墙"夹一条缝 | 同上 | 随机撒点很难可靠产生"必须精确穿越的窄缝"，这个场景不完全靠随机，用确定性结构保证效果 |

**参数细化**：
- 障碍物半径：0.12~0.3m（无人机外廓直径按`2×arm_length×√2≈0.5m`估算，障碍物与之同量级）
- 障碍物间最小间距：0.8~1.0m（`dense_field`密度提高后从早期方案的1.2m收紧）
- 起点/终点周边净空半径：1.2m内不放障碍物，保证起降不被卡死
- 狭窄通道缺口宽度：1.2~1.8m（约为无人机外廓的2.5~3.5倍，太窄对控制器精度要求过高，太宽体现不出"狭窄"）
- 安全距离(硬指标0.3~0.5m)：取0.35~0.4m，独立于障碍物本身大小
- 障碍物膨胀半径经验公式（参考EGO-Planner）：`1.5×无人机外廓尺寸` 且不超过 `4×点云分辨率`，两者取小值，约0.4m

**必须加的安全阀——连通性检测（防止密集随机障碍物把起点终点完全堵死）**：
```
1. 用固定种子(seed)生成候选障碍物位置
2. 对场地做栅格化(按安全距离膨胀障碍物)，用BFS/Flood-fill检查起点栅格→终点栅格是否连通
3. 若不连通：seed不变，用attempt计数器(seed, attempt+1)重新生成，直到连通或达到重试上限
4. 把最终生效的(seed, attempt)记录进日志/参数回显，保证结果可复现、可供老师复查
```
这一步不是可选项，`dense_field`/`narrow_corridor`模式必须实现，否则80个随机障碍物有实际概率把路完全堵死。

- 发布 `sensor_msgs/PointCloud2`（世界真值全局点云，可参考/移植MARSIM与pengyu_sim的`map_generator`包做法：锁存发布、支持降采样参数）+ `MarkerArray`(RViz直观展示)。
- 加分项：局部感知半径裁剪。可以自己简单实现（按距离过滤点云），也可以移植MARSIM/pengyu_sim的`local_sensing`(`pcl_render_node`)获得更完整的局部渲染(局部点云/深度图/传感器位姿)，二选一即可，非必须都做。

### 5.4 drone_planner（EGO-Planner移植/复现，见第2节）
- 前端 kinodynamic A*：在点云占据判断下搜索初始路径。
- 表示：均匀 B 样条控制点。
- 后端：梯度优化控制点（平滑代价+碰撞代价[ESDF-free，直接用点云KD-tree最近点构造排斥梯度]+可行性代价[速度/加速度超限惩罚]）。
- FSM 触发重规划（新目标/地图更新/到达局部窗口末端）。
- 失败条件处理：前端搜索超时/无可行路径→发布status说明+保持悬停，不能崩溃退出。

### 5.5 drone_visualization / drone_bringup
- RViz2 config：机体marker、`/drone/path`、`/planner/trajectory`、障碍物、目标点；支持2D/3D Goal直接发`/drone/goal`。
- `drone_bringup` 下六个 launch 文件，一一对应六个验收场景，全部参数走yaml，一条命令跑起来。

---

## 6. 六个验收场景 → 对应 launch + 验收标准

| # | 场景 | launch文件 | 验收标准 |
|---|---|---|---|
| 1 | 悬停 | `hover.launch.py` | 从地面起飞稳定在(0,0,1.5)，误差≤0.3m |
| 2 | 单目标点 | `single_goal.launch.py` | 飞到(2,1,1.5)并悬停，误差≤0.3m |
| 3 | 多目标点 | `multi_goal.launch.py` | 顺序飞3~4点(正方形航线) |
| 4 | 静态避障 | `avoidance.launch.py` | ≥5障碍物，绕开飞行，最小距离>安全距离 |
| 5 | 狭窄通道绕行 | `narrow_passage.launch.py` | 展示规划路径+实际轨迹重合度 |
| 6 | 稳定性展示 | `stability_demo.launch.py` | 位置误差曲线/RPM曲线/轨迹图/最小障碍物距离曲线，用`scripts/`里的评测脚本自动生成CSV+图 |

---

## 7. 加分项落地清单

| 加分项 | 落地方式 | 所在模块 |
|---|---|---|
| 参数文件配置 | 全部yaml化 | drone_bringup/config |
| 风扰+恢复展示 | dynamics内置风扰模型开关 | drone_dynamics |
| 传感器噪声 | IMU高斯噪声+bias随机游走开关 | drone_dynamics |
| 点云/体素地图+局部感知 | PointCloud2 + 局部感知半径裁剪 | drone_map |
| 多无人机 | namespace参数化，launch里spawn多份 | 全部节点 |
| 轨迹输入(圆/8字/waypoint) | Python脚本按序发布`/drone/goal` | scripts |
| 单元测试 | gtest测试推力/力矩计算、mixer求逆正确性 | drone_dynamics, drone_controller |
| 脚本化评测 | 订阅odom/status自动算误差指标、导出CSV/图 | scripts |
| 与参考仓库设计差异对比 | 本轮先在代码注释里记录，报告阶段整理 | 全局 |
| 自由发挥 / 地面站 | 视时间决定，优先级最低 | - |

---

## 8. 执行顺序（Claude Code 按此顺序推进，每步验证可编译再进下一步）

1. `drone_msgs`（先建消息，后面所有包都依赖它）
2. `drone_dynamics`（含单元测试）
3. `drone_controller`（含单元测试）—— 用简单目标点手动测试 dynamics+controller 闭环（不依赖planner）
4. `drone_map`（固定种子点云生成）
5. `drone_planner`（按第2节策略：先尝试移植ros2_version分支的bspline_opt/path_searching，行不通则自实现简化版）
6. `drone_visualization` + RViz config
7. `drone_bringup`（六场景launch + 全部yaml）
8. `scripts/`（轨迹发生器 + 评测脚本）
9. 顶层 `README.md`（编译/运行/场景说明/已知问题）
10. 逐场景真机(仿真)验证六个验收标准，记录哪些达标/哪些还需调参

**本轮不做**：PDF报告、`ai_usage.md`、演示视频、与pengyu_sim/MARSIM的正式对比文档（但过程记录要留着）。

---

## 9. 风险点提醒

- EGO-Planner 原版四元数/坐标系约定、Bspline节点间隔ts的选取，需要跟我们自己的 dynamics/controller 单位统一（rad vs deg，NED vs ENU等），移植时逐一核对，不要假设一致。
- NLopt/Armadillo 在 Humble 环境下可能需要额外 apt/vcpkg 安装，若环境受限（如离线/网络受限的开发机），优先考虑用轻量 LBFGS-Lite(单头文件)替代 NLopt，可省掉一个重依赖。
- 悬停误差0.3m和避障最小距离这两个硬指标要在场景5跑完后专门用评测脚本核对，不要只靠肉眼看RViz。
- 电机反扭矩正负号、自旋方向配对（对角同向）如果搞反，仿真会出现无法控制yaw或自发旋转，务必先用单元测试验证`allocationMatrix`和其逆矩阵的物理正确性（比如全部电机等转速时τx=τy=τz=0）。

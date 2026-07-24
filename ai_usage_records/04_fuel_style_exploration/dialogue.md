# Dialogue record — FUEL-style exploration

- Source transcript: `4388ed19-9096-4f14-8c7c-34ff76388ddd.jsonl`
- Extracted: 2026-07-21T21:03:26
- Anchors: 2 (SwitchMode / CreatePlan)

---

## [1108] user

<timestamp>Wednesday, Jul 15, 2026, 2:10 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1109] user

<timestamp>Wednesday, Jul 15, 2026, 2:10 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1110] user

<timestamp>Wednesday, Jul 15, 2026, 2:10 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1111] user

<timestamp>Wednesday, Jul 15, 2026, 2:10 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1112] user

<timestamp>Wednesday, Jul 15, 2026, 2:10 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1113] user

<timestamp>Wednesday, Jul 15, 2026, 2:10 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1115] assistant

后台任务结果汇总：

- **ego_planner / drone_bringup、gcopter** 均已重新编译成功。
- **修复后验证**：official_forest 点云正常中继（width≈31782），无崩溃。
- **首轮 30 组合烟雾**被中止（QoS/`pkill` 问题导致假 FAIL）；后续已重跑并通过，**30/30 PASS**（见 `report/map_smoke.json`）。

---

## [1116] user

<timestamp>Wednesday, Jul 15, 2026, 2:44 AM (UTC+8)</timestamp>
<user_query>
你看看还有没有什么前沿规划器可以接入
</user_query>

---

## [1117] assistant

先看你们现有规划器栈和参考笔记，再对照近几年可接入的前沿方案。

---

## [1119] assistant

你们已经覆盖了当前主流三角：**局部重规划（EGO）+ 全局走廊优化（GCOPTER/MINCO）+ 自研同构版**。还能接的“前沿”，按**接入性价比**排：

### 最值得接（同生态、契约好对齐）

| 候选 | 相对你们的增量 | 难度 | 备注 |
|------|----------------|------|------|
| **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/ego-planner-v2)** | Science Robotics 野外蜂群那套；MINCO 后端 + 更强集群 | 中高 | 官仓偏 ROS1 / 仿真沙盘，要裁成 Path D；比再包装一份 EGO-Swarm 更“新” |
| **[Fast-Racing](https://github.com/ZJU-FAST-Lab/Fast-Racing)** | SE(3) 激进轨迹、赛道形约束 | 中 | 后端就是 GCOPTER 扩展；适合炫技/窄通道高压测，不强求做日常 Path |
| **[Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation)** | 稠密环境保队形 | 中 | 你们已有 formation/ego_swarm，这块是官方升级路线 |

### 换赛道（不是避障点到点，而是任务形态）

| 候选 | 能证明什么 | 难度 |
|------|------------|------|
| **[FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL)** | 自主探索 + 前沿面覆盖（不是给定 goal） | 高：要局部建图/Raycast，不只全局点云 |
| **[Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)**（kinodynamic + ESDF） | EGO 的“上一代基底”，可做对照实验 | 中高：ROS1 多，ESDF 成本大；你们 EGO 已替代其定位 |

### 可看、暂不优先

- **PE-Planner**（USTC）：速度/抗扰更强，但对齐 **PX4+Gazebo**，和你们自研植物契约差得远  
- **Bubble Planner**：高速球廊 + MINCO，论文硬，**开源完整度弱**，基本得自实现  
- **Nav2 / RRT\***：偏地面/几何路径，不够“空中前沿”，性价比低  

### 建议怎么选

1. **作业/报告展示**：现有 A/B/C 已够“三类规划器”；不必强行加第四套同类局部规划。  
2. **真要多一条官方线**：优先 **EGO-Planner-v2（算法核）** 或 **Fast-Racing（GCOPTER 进阶）**——都能复用你们 `/drone/odom` + 点云 + `TrajectoryCommand` 桥，不碰 SO3 植物。  
3. **差异化加分**：接 **FUEL 式探索**（无 goal、主动扫图），和点到点避障是不同维度。

如果你定一条，我可以直接按现有 `planner_sim.launch.py` 契约起 **Path D** 骨架（vendor + bridge + dashboard 选项）。我个人推荐先做 **Fast-Racing 作 Path D**（和已有 GCOPTER 共享最多代码），或 **EGO-v2 精简单机核**。

---

## [1120] user

<timestamp>Wednesday, Jul 15, 2026, 2:46 AM (UTC+8)</timestamp>
<user_query>
```chat_selection
agent_id: 4388ed19-9096-4f14-8c7c-34ff76388ddd
composer_id: 4388ed19-9096-4f14-8c7c-34ff76388ddd
selected_text:
FUEL 式探索
```
 我想试试这个，记得别损坏源代码
</user_query>

---

## [1121] assistant

按你们现有 Path A/B/C 的 vendor 模式接入 FUEL 式探索，不动自研动力学/控制器，官方源码隔离vendoring。
[SwitchMode → plan] FUEL 是探索规划（非点到点），接入方式/裁剪范围差别大，先定不破坏现有源码的安全方案再动手。

---

## [1123] assistant

接入前有一个会决定工期和风险的分叉，需要你先定一下：
[AskQuestion] {"title": "FUEL 接入范围（保护现有源码）", "questions": [{"id": "fuel_scope", "prompt": "FUEL 官方是 ROS1 + Fast-Planner 全家桶，且要自建 OCC/ESDF、不能硬接你们现成的全局点云。在「不改坏动力学/控制/A·B·C」的前提下，你想先走哪条？", "options": [{"id": "inspired", "label": "先试：FUEL 式探索层（自写 frontier/视点 FSM）+ 驱动现有 homemade/ego 飞；官方 FUEL 只克隆到 reference_repos（COLCON_IGNORE，只读）——最快可跑、零伤植物"}, {"id": "vendor_port", "label": "正经 Path D：vendor 移植 fuel_planner（去掉 SO3），做 ROS2 桥接 + 局部感知；工期长，但尽量贴近官方算法"}, {"id": "inspired_then_port", "label": "分阶段：先做「先试」跑通演示，文档/入口预留 Path D，后续再慢慢 port 官方核"}]}]}

---

## [1124] assistant

[CreatePlan] name='FUEL-style exploration'
overview=在不改动植物端与现有 Path A/B/C 源码的前提下，把官方 FUEL 只读克隆到 reference_repos，并新增一条可开关的「FUEL 式探索」Path D：雾面感知 + frontier/视点 FSM + 复用 homemade 轨迹契约；为后续真移植官方核预留入口。

---

## [1125] user

<timestamp>Wednesday, Jul 15, 2026, 2:51 AM (UTC+8)</timestamp>
<user_query>
FUEL 式探索（安全接入）

Implement the plan as specified, it is attached for your reference. Do NOT edit the plan file itself.

To-do's from the plan have already been created. Do not create them again. Mark them as in_progress as you work, starting with the first one. Don't stop until you have completed all the to-dos.
</user_query>

---

## [1126] assistant

开始按计划安全接入 FUEL 式探索：先只读克隆官方仓，再新增独立包与 launch。

---

## [1131] assistant

FUEL 已只读克隆。正在更新文档并创建 `drone_exploration` 包。

---

## [1132] assistant

(tools: Write, Write, Write, Write, Write, Write)

---

## [1133] assistant

(tools: Write, Write, Write)

---

## [1136] assistant

(tools: Write)

---

## [1137] assistant

正在重写 launch 文件，去掉先前的临时写法。

---

## [1138] assistant

(tools: StrReplace, StrReplace, StrReplace, StrReplace)

---

## [1140] assistant

(tools: StrReplace, StrReplace, StrReplace)

---

## [1141] assistant

构建成功。正在跑 Path D 探索演示与 Path A 抽检。

---

## [1142] assistant

演示脚本的 heredoc 引号有问题，改成独立脚本再跑。

---

## [1156] assistant

Path D（FUEL 式探索）已按「不损坏现有源码」接入并验收。

### 做了什么
- **只读参考**：`reference_repos/FUEL/`（`COLCON_IGNORE`，未改官方树）
- **新包** [`drone_exploration`](src/drone_exploration/)：雾面 `local_sensing` + frontier `exploration_fsm`
- **Launch / UI**：`fuel_explore.launch.py`、`planner:=fuel_explore`、Dashboard Path D
- **植物端 / A·B·C**：未改动力学与控制；官方 EGO/GCOPTER 算法文件未动

### 怎么跑
```bash
source install/setup.bash
ros2 launch drone_bringup planner_sim.launch.py planner:=fuel_explore map:=dense_field
```
RViz **2D Goal Pose** 只作**启动探索**；之后目标走 `/exploration/nav_goal`，规划器吃 `/map/obstacles_local`。

### 验收
- Path D：`local_width=1004`，连续 6 个 nav goal，A* 有路径 → `FUEL_SMOKE=0`
- Path A 抽检：`homemade × dense_field` **PASS**

---

## [1157] user

<timestamp>Wednesday, Jul 15, 2026, 3:27 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

## [1158] user

<timestamp>Wednesday, Jul 15, 2026, 3:27 AM (UTC+8)</timestamp>

<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

# PLAN.md 验收对照表

> 自动生成时间：2026-08-25 21:57:01 CST
> 通过：**6/6** 场景

## 总览

| # | 场景 | launch | 结果 | 关键指标 |
|---|------|--------|------|----------|
| 1 | 悬停 | `hover.launch.py` | ✅ PASS | mean_err=0.0004 m; final=0.0001 m; min_obs=1.7550 m; planner_ok=False; hold=True |
| 2 | 单目标点 | `single_goal.launch.py` | ✅ PASS | mean_err=0.0070 m; final=0.0013 m; min_obs=2.9299 m; planner_ok=False; hold=True |
| 3 | 多目标点(正方形4点) | `multi_goal.launch.py` | ✅ PASS | mean_err=0.3120 m; final=0.0009 m; min_obs=0.7433 m; planner_ok=False; hold=True; wp=5/5 |
| 4 | 静态避障 | `avoidance.launch.py` | ✅ PASS | mean_err=8.7713 m; final=0.0662 m; min_obs=0.4910 m; planner_ok=True; hold=True; wp=8/8 |
| 5 | 狭窄通道绕行 | `narrow_passage.launch.py` | ✅ PASS | mean_err=1.4014 m; final=0.0005 m; min_obs=0.9595 m; planner_ok=True; hold=True |
| 6 | 稳定性展示 | `stability_demo.launch.py` | ✅ PASS | mean_err=0.0354 m; final=0.0367 m; min_obs=1.7177 m; planner_ok=False; hold=True |

## 分项检查

### 场景 1：悬停

- 说明：目标 (0,0,1.5)，3s 后自动发 goal
- 评测图：

  ![scenario 1 evaluation](acceptance_runs/scenario_01_hover/evaluation.png)

- 检查项：
  - ✅ 位置误差≤0.3m(末段均值)
  - ✅ 最终位置误差≤0.3m
- 补充检查（不计入原六项通过判定）：
  - ✅ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 901
  - `mean_pos_err`: 0.0004 m
  - `max_pos_err`: 0.0142 m
  - `final_pos_err`: 0.0001 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `min_obstacle_distance`: 1.7550 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 0.0392 m
  - `detour_ratio`: 2.7522
  - `mean_jerk`: 0.0038 m/s^3
- launch 日志末尾（截断）：
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [506905]
[INFO] [controller_node-2]: process started with pid [506907]
[INFO] [map_node-3]: process started with pid [506909]
[INFO] [viz_node-4]: process started with pid [506911]
[viz_node-4] [INFO] [1787665281.971337558] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1787665281.975577152] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[controller_node-2] [INFO] [1787665281.975979538] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[map_node-3] [INFO] [1787665281.978357344] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=2 points=600 downsample_voxel=0.000
[map_node-3] [INFO] [1787665281.979108274] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=600 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-5]: process started with pid [506970]
[send_goal-5] [INFO] [1787665285.521271201] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
```

### 场景 2：单目标点

- 说明：目标 (2,1,1.5)；RViz 开启时等待界面就绪后发 goal
- 评测图：

  ![scenario 2 evaluation](acceptance_runs/scenario_02_single_goal/evaluation.png)

- 检查项：
  - ✅ 到达目标误差≤0.3m
  - ✅ 最大误差≤3.0m(起飞过程)
- 补充检查（不计入原六项通过判定）：
  - ✅ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 1002
  - `mean_pos_err`: 0.0070 m
  - `max_pos_err`: 0.1096 m
  - `final_pos_err`: 0.0013 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `min_obstacle_distance`: 2.9299 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 0.4516 m
  - `detour_ratio`: 10.2603
  - `mean_jerk`: 1.3928 m/s^3
- launch 日志末尾（截断）：
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [507237]
[INFO] [controller_node-2]: process started with pid [507239]
[INFO] [map_node-3]: process started with pid [507241]
[INFO] [viz_node-4]: process started with pid [507243]
[viz_node-4] [INFO] [1787665346.291216894] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1787665346.291684002] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1787665346.292177348] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1787665346.296285354] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=2 points=711 downsample_voxel=0.000
[map_node-3] [INFO] [1787665346.297006609] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=711 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-5]: process started with pid [507302]
[send_goal-5] [INFO] [1787665349.839193107] [send_goal]: Published goal (2.00, 1.00, 1.50) yaw=0.00 remaining=0 subs=1
```

### 场景 3：多目标点(正方形4点)

- 说明：原点起步的 2m×2m 闭合正方形；到点且低速后切换下一航点
- 评测图：

  ![scenario 3 evaluation](acceptance_runs/scenario_03_multi_goal/evaluation.png)

- 检查项：
  - ✅ 4条边的角点均访问
  - ✅ 最终回到起点附近
- 原始指标：
  - `samples`: 1202
  - `mean_pos_err`: 0.3120 m
  - `max_pos_err`: 2.9136 m
  - `final_pos_err`: 0.0009 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `min_obstacle_distance`: 0.7433 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 9.7594 m
  - `detour_ratio`: 6.5063
  - `mean_jerk`: 1.8465 m/s^3
- launch 日志末尾（截断）：
```
[controller_node-2] [INFO] [1787665415.414438258] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[viz_node-4] [INFO] [1787665415.416263907] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1787665415.417798699] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1787665415.425299989] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=2 points=600 downsample_voxel=0.000
[map_node-3] [INFO] [1787665415.425948727] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=600 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [waypoint_publisher-5]: process started with pid [507646]
[waypoint_publisher-5] [INFO] [1787665420.624674777] [waypoint_publisher]: Waypoint publisher: 4 pts/cycle × 1 cycles, mode=arrival, topic=/drone/goal
[waypoint_publisher-5] [INFO] [1787665421.115274131] [waypoint_publisher]: Waypoint cycle 1/1 1/4: (2.00, 0.00, 1.50)
[waypoint_publisher-5] [INFO] [1787665423.415560275] [waypoint_publisher]: Waypoint cycle 1/1 2/4: (2.00, 2.00, 1.50)
[waypoint_publisher-5] [INFO] [1787665425.615206302] [waypoint_publisher]: Waypoint cycle 1/1 3/4: (0.00, 2.00, 1.50)
[waypoint_publisher-5] [INFO] [1787665427.815168040] [waypoint_publisher]: Waypoint cycle 1/1 4/4: (0.00, 0.00, 1.50)
[waypoint_publisher-5] [INFO] [1787665430.014675420] [waypoint_publisher]: Waypoint sequence complete
```

### 场景 4：静态避障

- 说明：Path B EGO + official_forest: lap1 矩形, lap2 漏斗(对角→宽→对角→宽)
- 评测图：

  ![scenario 4 evaluation](acceptance_runs/scenario_04_avoidance/evaluation.png)

- 检查项：
  - ✅ 循环航点均访问
  - ✅ 最小障碍距离>0.30m
  - ✅ 规划器曾报告success
- 原始指标：
  - `samples`: 8402
  - `mean_pos_err`: 8.7713 m
  - `max_pos_err`: 19.6685 m
  - `final_pos_err`: 0.0662 m
  - `hover_pass_0.3m`: False
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: TRAJ_CMD
  - `min_obstacle_distance`: 0.4910 m
  - `avoidance_safety_distance`: 0.3000 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 130.1320 m
  - `detour_ratio`: 14.2241
  - `mean_jerk`: 1.6430 m/s^3
  - `planned_tracking_error_mean`: 7.7560 m
- launch 日志末尾（截断）：
```
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[map_adapter-2] [INFO] [1787665924.300873287] [map_adapter]: adapted cloud #4340 width=27388 occ=136x88 topdown_cells=527
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[local_sense_cloud-6] [INFO] [1787665924.849630992] [drone_0_local_sense_cloud]: global cloud cached: 25800 points
[local_sense_cloud-6] [INFO] [1787665925.275979063] [drone_0_local_sense_cloud]: local cloud #4350: 6684 pts @ (-8.0,-6.0,1.1)
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[map_adapter-2] [INFO] [1787665926.304980504] [map_adapter]: adapted cloud #4360 width=27388 occ=136x88 topdown_cells=527
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
```

### 场景 5：狭窄通道绕行

- 说明：narrow_corridor S-bend: 3×1.6m doors + side clutter (PLAN §5.3)
- 评测图：

  ![scenario 5 evaluation](acceptance_runs/scenario_05_narrow_passage/evaluation.png)

- 检查项：
  - ✅ 到达目标误差≤0.5m
  - ✅ 最小障碍距离>0.35m
  - ✅ 无A*失败日志
- 补充检查（不计入原六项通过判定）：
  - ✅ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 3002
  - `mean_pos_err`: 1.4014 m
  - `max_pos_err`: 12.6390 m
  - `final_pos_err`: 0.0005 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 0.9595 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 25.4617 m
  - `detour_ratio`: 2.0145
  - `mean_jerk`: 6.8399 m/s^3
  - `planned_tracking_error_mean`: 0.0243 m
- launch 日志末尾（截断）：
```
[controller_node-2] [INFO] [1787665935.889161880] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[planner_node-4] [INFO] [1787665935.890019580] [drone_planner]: drone_planner ready (DynAStar=on, Bspline=off, local_mapping=off, map=/map/obstacles, peers=)
[map_node-3] [INFO] [1787665935.906641476] [drone_map]: Map generated: mode=narrow_corridor seed=42 attempt=0 connected=true obstacles=83 points=137567 downsample_voxel=0.000
[map_node-3] [INFO] [1787665935.910610781] [drone_map]: drone_map ready: mode=narrow_corridor seed=42 attempt=0 connected=yes points=137567 (native cylinders+walls for dense/narrow; ego_* modes optional)
[planner_node-4] [INFO] [1787665935.966399311] [drone_planner]: Map ingested: 137567 raw -> 61589 voxels | origin=(-4.7,-4.7,-0.2) size=(29.4,19.4,4.7) res=0.15 inflate=0.24 [auto_fit] [auto_inflate]
[INFO] [send_goal-6]: process started with pid [508556]
[planner_node-4] [INFO] [1787665940.436676801] [drone_planner]: New goal (17.00, 5.00, 1.50)
[planner_node-4] [INFO] [1787665940.440055356] [drone_planner]: Planning (1.0,5.0,1.4)->(17.0,5.0,1.5)
[planner_node-4] [INFO] [1787665940.440080072] [drone_planner]: Running A* with 0 peer keep-outs (true_3d=no)
[send_goal-6] [INFO] [1787665940.449392391] [send_goal]: Published goal (17.00, 5.00, 1.50) yaw=0.00 remaining=0 subs=2
[planner_node-4] [INFO] [1787665940.467211785] [drone_planner]: A* finished found=yes guide=153
[planner_node-4] [INFO] [1787665940.467513201] [drone_planner]: Planned path: 278 waypoints, length=28.33 m (horizontal avoidance)
```

### 场景 6：稳定性展示

- 说明：wind_enable+imu_noise_enable, 手动跑 evaluate
- 评测图：

  ![scenario 6 evaluation](acceptance_runs/scenario_06_stability_demo/evaluation.png)

- 检查项：
  - ✅ 风扰下悬停误差≤0.3m
  - ✅ 评测图已生成
- 原始指标：
  - `samples`: 1801
  - `mean_pos_err`: 0.0354 m
  - `max_pos_err`: 0.0820 m
  - `final_pos_err`: 0.0367 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `min_obstacle_distance`: 1.7177 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 3.9949 m
  - `detour_ratio`: 63.9746
  - `mean_jerk`: 1.5567 m/s^3
- launch 日志末尾（截断）：
```
[INFO] [controller_node-2]: process started with pid [508872]
[INFO] [map_node-3]: process started with pid [508874]
[INFO] [viz_node-4]: process started with pid [508876]
[viz_node-4] [INFO] [1787666108.520361410] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1787666108.522307345] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1787666108.522936604] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1787666108.527871549] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=2 points=600 downsample_voxel=0.000
[map_node-3] [INFO] [1787666108.529008537] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=600 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [interference_monitor-5]: process started with pid [508934]
[interference_monitor-5] [INFO] [1787666111.482879439] [interference_monitor]: Interference monitor ready (goal=(0.0, 0.0, 1.5), limit=0.3 m)
[INFO] [send_goal-6]: process started with pid [508962]
[send_goal-6] [INFO] [1787666112.398532197] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
```

## PLAN.md 硬指标对照

| 硬指标 | PLAN 要求 | 验收方式 | 当前状态 |
|--------|-----------|----------|----------|
| 悬停误差 | ≤ 0.3 m | scenario 1 evaluate.py | ✅ |
| 避障最小距离 | > 安全距离 0.30 m | scenario 4 min_obstacle_distance | ✅ |
| 狭窄通道 | 规划路径+实际轨迹可展示 | scenario 5 到达+无A*失败 | ✅ |
| 稳定性 | 误差/RPM曲线 | scenario 6 CSV+PNG | ✅ |

## 产物路径

- JSON：`report/acceptance_results.json`
- 各场景原始数据：`report/acceptance_runs/scenario_XX_*/metrics.csv`
- 各场景评测图：`report/acceptance_runs/scenario_XX_*/evaluation.png`

## 复现命令

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && source install/setup.bash
python3 scripts/run_acceptance.py
```


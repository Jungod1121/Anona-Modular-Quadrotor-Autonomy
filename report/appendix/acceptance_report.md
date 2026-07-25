# PLAN.md 验收对照表

> 自动生成时间：2026-07-25 11:24:31 CST
> 通过：**6/6** 场景

## 总览

| # | 场景 | launch | 结果 | 关键指标 |
|---|------|--------|------|----------|
| 1 | 悬停 | `hover.launch.py` | ✅ PASS | mean_err=0.0006 m; final=0.0001 m; planner_ok=False; hold=True |
| 2 | 单目标点 | `single_goal.launch.py` | ✅ PASS | mean_err=0.0069 m; final=0.0013 m; planner_ok=False; hold=True |
| 3 | 多目标点(正方形4点) | `multi_goal.launch.py` | ✅ PASS | mean_err=0.2933 m; final=0.0003 m; planner_ok=False; hold=True; wp=5/5 |
| 4 | 静态避障 | `avoidance.launch.py` | ✅ PASS | mean_err=9.3207 m; final=0.1027 m; min_obs=0.3132 m; planner_ok=True; hold=True; wp=8/8 |
| 5 | 狭窄通道绕行 | `narrow_passage.launch.py` | ✅ PASS | mean_err=0.7364 m; final=0.0003 m; min_obs=0.9870 m; planner_ok=True; hold=True |
| 6 | 稳定性展示 | `stability_demo.launch.py` | ✅ PASS | mean_err=0.0358 m; final=0.0433 m; planner_ok=False; hold=True |

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
  - `mean_pos_err`: 0.0006 m
  - `max_pos_err`: 0.0277 m
  - `final_pos_err`: 0.0001 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 0.0525 m
  - `detour_ratio`: 1.8967
  - `mean_jerk`: 0.0050 m/s^3
- launch 日志末尾（截断）：
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [17814]
[INFO] [controller_node-2]: process started with pid [17816]
[INFO] [map_node-3]: process started with pid [17818]
[INFO] [viz_node-4]: process started with pid [17820]
[viz_node-4] [INFO] [1784898725.807508590] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1784898725.814808239] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[map_node-3] [INFO] [1784898725.814915709] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784898725.815772376] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[dynamics_node-1] [INFO] [1784898725.827574289] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[INFO] [send_goal-5]: process started with pid [17927]
[send_goal-5] [INFO] [1784898729.702915405] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
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
  - `mean_pos_err`: 0.0069 m
  - `max_pos_err`: 0.1096 m
  - `final_pos_err`: 0.0013 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 0.4400 m
  - `detour_ratio`: 8.6979
  - `mean_jerk`: 1.3899 m/s^3
- launch 日志末尾（截断）：
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [18369]
[INFO] [controller_node-2]: process started with pid [18371]
[INFO] [map_node-3]: process started with pid [18373]
[INFO] [viz_node-4]: process started with pid [18375]
[viz_node-4] [INFO] [1784898798.167049579] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1784898798.168058659] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1784898798.169756950] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1784898798.175179326] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784898798.175846486] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-5]: process started with pid [18463]
[send_goal-5] [INFO] [1784898802.038343360] [send_goal]: Published goal (2.00, 1.00, 1.50) yaw=0.00 remaining=0 subs=1
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
  - `mean_pos_err`: 0.2933 m
  - `max_pos_err`: 2.9136 m
  - `final_pos_err`: 0.0003 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 9.7599 m
  - `detour_ratio`: 6.5066
  - `mean_jerk`: 1.8073 m/s^3
- launch 日志末尾（截断）：
```
[controller_node-2] [INFO] [1784898875.452582733] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[viz_node-4] [INFO] [1784898875.452870740] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1784898875.452954439] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1784898875.462585117] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784898875.463281148] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [waypoint_publisher-5]: process started with pid [19067]
[waypoint_publisher-5] [INFO] [1784898880.665801029] [waypoint_publisher]: Waypoint publisher: 4 pts/cycle × 1 cycles, mode=arrival, topic=/drone/goal
[waypoint_publisher-5] [INFO] [1784898881.156101715] [waypoint_publisher]: Waypoint cycle 1/1 1/4: (2.00, 0.00, 1.50)
[waypoint_publisher-5] [INFO] [1784898883.455936598] [waypoint_publisher]: Waypoint cycle 1/1 2/4: (2.00, 2.00, 1.50)
[waypoint_publisher-5] [INFO] [1784898885.655954630] [waypoint_publisher]: Waypoint cycle 1/1 3/4: (0.00, 2.00, 1.50)
[waypoint_publisher-5] [INFO] [1784898887.856029738] [waypoint_publisher]: Waypoint cycle 1/1 4/4: (0.00, 0.00, 1.50)
[waypoint_publisher-5] [INFO] [1784898890.055824791] [waypoint_publisher]: Waypoint sequence complete
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
  - `samples`: 5602
  - `mean_pos_err`: 9.3207 m
  - `max_pos_err`: 19.6716 m
  - `final_pos_err`: 0.1027 m
  - `hover_pass_0.3m`: False
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: TRAJ_CMD
  - `min_obstacle_distance`: 0.3132 m
  - `avoidance_safety_distance`: 0.3000 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: False
  - `flown_path_length`: 122.6285 m
  - `detour_ratio`: 13.4281
  - `mean_jerk`: 2.6285 m/s^3
  - `planned_tracking_error_mean`: 7.4251 m
- launch 日志末尾（截断）：
```
[local_sense_cloud-6] [INFO] [1784949862.968721386] [drone_0_local_sense_cloud]: global cloud cached: 31782 points
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[map_adapter-2] [INFO] [1784949863.882275363] [map_adapter]: adapted cloud #2820 width=33370 occ=136x88 topdown_cells=650
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[local_sense_cloud-6] [INFO] [1784949864.912827441] [drone_0_local_sense_cloud]: local cloud #2950: 6891 pts @ (-7.9,-6.0,1.0)
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[map_adapter-2] [INFO] [1784949865.953678458] [map_adapter]: adapted cloud #2840 width=33370 occ=136x88 topdown_cells=650
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
  - `mean_pos_err`: 0.7364 m
  - `max_pos_err`: 11.1025 m
  - `final_pos_err`: 0.0003 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 0.9870 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 19.9694 m
  - `detour_ratio`: 1.7986
  - `mean_jerk`: 6.5304 m/s^3
  - `planned_tracking_error_mean`: 0.0232 m
- launch 日志末尾（截断）：
```
[dynamics_node-1] [INFO] [1784899263.256658363] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[planner_node-4] [INFO] [1784899263.258311716] [drone_planner]: drone_planner ready (DynAStar=on, Bspline=off, local_mapping=off, map=/map/obstacles, peers=)
[map_node-3] [INFO] [1784899263.268908097] [drone_map]: Map generated: mode=narrow_corridor seed=42 attempt=0 connected=true obstacles=83 points=137567 downsample_voxel=0.000
[map_node-3] [INFO] [1784899263.272387328] [drone_map]: drone_map ready: mode=narrow_corridor seed=42 attempt=0 connected=yes points=137567 (native cylinders+walls for dense/narrow; ego_* modes optional)
[planner_node-4] [INFO] [1784899263.294515891] [drone_planner]: Map ingested: 137567 raw -> 15616 voxels | origin=(-4.7,-4.7,-0.2) size=(29.4,19.4,4.7) res=0.25 inflate=0.08 [auto_fit] [auto_inflate]
[INFO] [send_goal-6]: process started with pid [22187]
[planner_node-4] [INFO] [1784899267.791067478] [drone_planner]: New goal (17.00, 5.00, 1.50)
[send_goal-6] [INFO] [1784899267.801828888] [send_goal]: Published goal (17.00, 5.00, 1.50) yaw=0.00 remaining=0 subs=2
[planner_node-4] [INFO] [1784899267.808386714] [drone_planner]: Planning (1.0,5.0)->(17.0,5.0)
[planner_node-4] [INFO] [1784899267.808420258] [drone_planner]: Running A* with 0 peer keep-outs
[planner_node-4] [INFO] [1784899267.813781601] [drone_planner]: A* finished found=yes guide=87
[planner_node-4] [INFO] [1784899267.813872262] [drone_planner]: Planned path: 138 waypoints, length=26.78 m (horizontal avoidance)
```

### 场景 6：稳定性展示

- 说明：wind_enable+imu_noise_enable, 手动跑 evaluate
- 评测图：

  ![scenario 6 evaluation](acceptance_runs/scenario_06_stability_demo/evaluation.png)

- 检查项：
  - ✅ 风扰下悬停误差≤0.3m
  - ✅ 评测图已生成
- 原始指标：
  - `samples`: 1802
  - `mean_pos_err`: 0.0358 m
  - `max_pos_err`: 0.0917 m
  - `final_pos_err`: 0.0433 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 4.1290 m
  - `detour_ratio`: 45.0401
  - `mean_jerk`: 1.4263 m/s^3
- launch 日志末尾（截断）：
```
[INFO] [controller_node-2]: process started with pid [22661]
[INFO] [map_node-3]: process started with pid [22663]
[INFO] [viz_node-4]: process started with pid [22665]
[controller_node-2] [INFO] [1784899444.531567506] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1784899444.532102421] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1784899444.533255027] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[viz_node-4] [INFO] [1784899444.533780654] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[map_node-3] [INFO] [1784899444.533968487] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [interference_monitor-5]: process started with pid [22737]
[interference_monitor-5] [INFO] [1784899447.471422349] [interference_monitor]: Interference monitor ready (goal=(0.0, 0.0, 1.5), limit=0.3 m)
[INFO] [send_goal-6]: process started with pid [22765]
[send_goal-6] [INFO] [1784899448.455194834] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
```

## PLAN.md 硬指标对照

| 硬指标 | PLAN 要求 | 验收方式 | 当前状态 |
|--------|-----------|----------|----------|
| 悬停误差 | ≤ 0.3 m | scenario 1 evaluate.py | ✅ |
| 避障最小距离 | > 安全距离 0.30 m | scenario 4 min_obstacle_distance | ❌ |
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


# PLAN.md 验收对照表

> 自动生成时间：2026-07-14 01:49:26 CST
> 通过：**6/6** 场景

## 总览

| # | 场景 | launch | 结果 | 关键指标 |
|---|------|--------|------|----------|
| 1 | 悬停 | `hover.launch.py` | ✅ PASS | mean_err=0.0001 m; final=0.0000 m; planner_ok=False |
| 2 | 单目标点 | `single_goal.launch.py` | ✅ PASS | mean_err=0.0050 m; final=0.0000 m; planner_ok=False |
| 3 | 多目标点(正方形4点) | `multi_goal.launch.py` | ✅ PASS | mean_err=0.5674 m; final=0.0000 m; planner_ok=False; wp=4/4 |
| 4 | 静态避障 | `avoidance.launch.py` | ✅ PASS | mean_err=0.1294 m; final=0.0000 m; min_obs=0.5127 m; planner_ok=True |
| 5 | 狭窄通道绕行 | `narrow_passage.launch.py` | ✅ PASS | mean_err=0.0322 m; final=0.0000 m; min_obs=2.7056 m; planner_ok=True |
| 6 | 稳定性展示 | `stability_demo.launch.py` | ✅ PASS | mean_err=0.2044 m; final=0.1815 m; planner_ok=False |

## 分项检查

### 场景 1：悬停

- 说明：目标 (0,0,1.5)，3s 后自动发 goal
- 检查项：
  - ✅ 位置误差≤0.3m(末段均值)
  - ✅ 最终位置误差≤0.3m
- 原始指标：
  - `samples`: 902
  - `mean_pos_err`: 0.0001 m
  - `max_pos_err`: 0.0071 m
  - `final_pos_err`: 0.0000 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `planner_success_ever`: False
  - `final_planner_state`: 
- launch 日志末尾：
```
[INFO] [launch]: All log files can be found below /home/jungod/.ros/log/2026-07-14-01-37-43-619579-jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV-436940
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [436957]
[INFO] [controller_node-2]: process started with pid [436959]
[INFO] [map_node-3]: process started with pid [436961]
[INFO] [viz_node-4]: process started with pid [436963]
[viz_node-4] [INFO] [1783964263.679189621] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1783964263.680578792] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[controller_node-2] [INFO] [1783964263.680943870] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00 (cascade PID + mixer, self-developed)
[map_node-3] [INFO] [1783964263.689564586] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1783964263.689706239] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-5]: process started with pid [437029]
[send_goal-5] [INFO] [1783964267.182774712] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0
```

### 场景 2：单目标点

- 说明：目标 (2,1,1.5)
- 检查项：
  - ✅ 到达目标误差≤0.3m
  - ✅ 最大误差≤3.0m(起飞过程)
- 原始指标：
  - `samples`: 1002
  - `mean_pos_err`: 0.0050 m
  - `max_pos_err`: 0.1644 m
  - `final_pos_err`: 0.0000 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `planner_success_ever`: False
  - `final_planner_state`: 
- launch 日志末尾：
```
[INFO] [launch]: All log files can be found below /home/jungod/.ros/log/2026-07-14-01-38-46-177879-jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV-437250
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [437267]
[INFO] [controller_node-2]: process started with pid [437269]
[INFO] [map_node-3]: process started with pid [437271]
[INFO] [viz_node-4]: process started with pid [437273]
[controller_node-2] [INFO] [1783964326.247145713] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00 (cascade PID + mixer, self-developed)
[viz_node-4] [INFO] [1783964326.247318214] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1783964326.247665619] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1783964326.260546278] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1783964326.260773180] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-5]: process started with pid [437353]
[send_goal-5] [INFO] [1783964330.099779343] [send_goal]: Published goal (2.00, 1.00, 1.50) yaw=0.00 remaining=0
```

### 场景 3：多目标点(正方形4点)

- 说明：square side=2m, hold=8s/点, 5s 后开始发航点
- 检查项：
  - ✅ 4个角点均访问
  - ✅ 最终回到起点附近
- 原始指标：
  - `samples`: 2201
  - `mean_pos_err`: 0.5674 m
  - `max_pos_err`: 3.2364 m
  - `final_pos_err`: 0.0000 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `planner_success_ever`: False
  - `final_planner_state`: 
- launch 日志末尾：
```
[INFO] [map_node-3]: process started with pid [437608]
[INFO] [viz_node-4]: process started with pid [437610]
[controller_node-2] [INFO] [1783964392.589400431] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00 (cascade PID + mixer, self-developed)
[viz_node-4] [INFO] [1783964392.591168758] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1783964392.594572686] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1783964392.605505621] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1783964392.605656522] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [waypoint_publisher-5]: process started with pid [437688]
[waypoint_publisher-5] [INFO] [1783964397.824281853] [waypoint_publisher]: Waypoint publisher: 5 points, hold=8.0s, topic=/drone/goal
[waypoint_publisher-5] [INFO] [1783964405.811956921] [waypoint_publisher]: Waypoint 1/5: (1.00, 1.00, 1.50)
[waypoint_publisher-5] [INFO] [1783964413.812114051] [waypoint_publisher]: Waypoint 2/5: (-1.00, 1.00, 1.50)
[waypoint_publisher-5] [INFO] [1783964421.812111794] [waypoint_publisher]: Waypoint 3/5: (-1.00, -1.00, 1.50)
[waypoint_publisher-5] [INFO] [1783964429.812071149] [waypoint_publisher]: Waypoint 4/5: (1.00, -1.00, 1.50)
[waypoint_publisher-5] [INFO] [1783964437.811949512] [waypoint_publisher]: Waypoint 5/5: (1.00, 1.00, 1.50)
[waypoint_publisher-5] [INFO] [1783964445.812324172] [waypoint_publisher]: Waypoint sequence complete
```

### 场景 4：静态避障

- 说明：dense_field 80圆柱+围墙, seed=42
- 检查项：
  - ✅ 到达目标误差≤0.5m
  - ✅ 最小障碍距离>0.35m
  - ✅ 规划器曾报告success
- 原始指标：
  - `samples`: 3002
  - `mean_pos_err`: 0.1294 m
  - `max_pos_err`: 7.0328 m
  - `final_pos_err`: 0.0000 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 0.5127 m
  - `avoidance_pass_0.35m`: True
- launch 日志末尾：
```
[INFO] [controller_node-2]: process started with pid [438003]
[INFO] [map_node-3]: process started with pid [438005]
[INFO] [planner_node-4]: process started with pid [438007]
[INFO] [viz_node-5]: process started with pid [438009]
[planner_node-4] [INFO] [1783964520.972448891] [drone_planner]: drone_planner ready (DynAStar=off, Bspline=off, local_raycast=off)
[viz_node-5] [INFO] [1783964520.972943891] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1783964520.973458548] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00 (cascade PID + mixer, self-developed)
[dynamics_node-1] [INFO] [1783964520.973724142] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1783964521.091528298] [drone_map]: Map generated: mode=dense_field seed=42 attempt=0 connected=true obstacles=84 points=96008 downsample_voxel=0.000
[map_node-3] [INFO] [1783964521.094760260] [drone_map]: drone_map ready: mode=dense_field seed=42 attempt=0 connected=yes points=96008 (native cylinders+walls for dense/narrow; ego_* modes optional)
[planner_node-4] [INFO] [1783964521.113666456] [drone_planner]: Map ingested: 96008 raw -> 7794 voxels (+boundary sealed)
[INFO] [send_goal-6]: process started with pid [438117]
[planner_node-4] [INFO] [1783964525.465228595] [drone_planner]: New goal (17.00, 5.00, 1.50)
[planner_node-4] [INFO] [1783964525.472640701] [drone_planner]: Planned path: 98 waypoints, length=19.39 m (horizontal avoidance)
[send_goal-6] [INFO] [1783964525.477189943] [send_goal]: Published goal (17.00, 5.00, 1.50) yaw=0.00 remaining=0
```

### 场景 5：狭窄通道绕行

- 说明：narrow_corridor 80圆柱+窄门+围墙
- 检查项：
  - ✅ 到达目标误差≤0.5m
  - ✅ 最小障碍距离>0.35m
  - ✅ 无A*失败日志
- 原始指标：
  - `samples`: 3002
  - `mean_pos_err`: 0.0322 m
  - `max_pos_err`: 3.7069 m
  - `final_pos_err`: 0.0000 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 2.7056 m
  - `avoidance_pass_0.35m`: True
- launch 日志末尾：
```
[INFO] [controller_node-2]: process started with pid [438405]
[INFO] [map_node-3]: process started with pid [438407]
[INFO] [planner_node-4]: process started with pid [438409]
[INFO] [viz_node-5]: process started with pid [438411]
[viz_node-5] [INFO] [1783964692.125002988] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1783964692.125596182] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00 (cascade PID + mixer, self-developed)
[dynamics_node-1] [INFO] [1783964692.125896311] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[planner_node-4] [INFO] [1783964692.127547063] [drone_planner]: drone_planner ready (DynAStar=off, Bspline=off, local_raycast=off)
[map_node-3] [INFO] [1783964692.351878492] [drone_map]: Map generated: mode=narrow_corridor seed=42 attempt=0 connected=true obstacles=60 points=97073 downsample_voxel=0.000
[map_node-3] [INFO] [1783964692.354140010] [drone_map]: drone_map ready: mode=narrow_corridor seed=42 attempt=0 connected=yes points=97073 (native cylinders+walls for dense/narrow; ego_* modes optional)
[planner_node-4] [INFO] [1783964692.369978875] [drone_planner]: Map ingested: 97073 raw -> 7797 voxels (+boundary sealed)
[INFO] [send_goal-6]: process started with pid [438502]
[planner_node-4] [INFO] [1783964696.621913254] [drone_planner]: New goal (17.00, 5.00, 1.50)
[planner_node-4] [INFO] [1783964696.627598414] [drone_planner]: Planned path: 66 waypoints, length=16.07 m (horizontal avoidance)
[send_goal-6] [INFO] [1783964696.634063154] [send_goal]: Published goal (17.00, 5.00, 1.50) yaw=0.00 remaining=0
```

### 场景 6：稳定性展示

- 说明：wind_enable+imu_noise_enable, 手动跑 evaluate
- 检查项：
  - ✅ 风扰下悬停误差≤0.3m
  - ✅ 评测图已生成
- 原始指标：
  - `samples`: 1801
  - `mean_pos_err`: 0.2044 m
  - `max_pos_err`: 0.3035 m
  - `final_pos_err`: 0.1815 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `planner_success_ever`: False
  - `final_planner_state`: 
- launch 日志末尾：
```
[INFO] [launch]: All log files can be found below /home/jungod/.ros/log/2026-07-14-01-47-42-462856-jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV-438718
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dynamics_node-1]: process started with pid [438735]
[INFO] [controller_node-2]: process started with pid [438737]
[INFO] [map_node-3]: process started with pid [438739]
[INFO] [viz_node-4]: process started with pid [438741]
[viz_node-4] [INFO] [1783964862.860306075] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1783964862.860422081] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00 (cascade PID + mixer, self-developed)
[map_node-3] [INFO] [1783964862.860465702] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1783964862.860670554] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[dynamics_node-1] [INFO] [1783964862.861697165] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[INFO] [send_goal-5]: process started with pid [438834]
[send_goal-5] [INFO] [1783964866.019950218] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0
```

## PLAN.md 硬指标对照

| 硬指标 | PLAN 要求 | 验收方式 | 当前状态 |
|--------|-----------|----------|----------|
| 悬停误差 | ≤ 0.3 m | scenario 1 evaluate.py | ✅ |
| 避障最小距离 | > 安全距离 0.35 m | scenario 4 min_obstacle_distance | ✅ |
| 狭窄通道 | 规划路径+实际轨迹可展示 | scenario 5 到达+无A*失败 | ✅ |
| 稳定性 | 误差/RPM曲线 | scenario 6 CSV+PNG | ✅ |

## 产物路径

- JSON：`report/acceptance_results.json`
- 各场景原始数据：`report/acceptance_runs/scenario_XX_*/metrics.csv`

## 复现命令

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_ws && source install/setup.bash
python3 scripts/run_acceptance.py
```


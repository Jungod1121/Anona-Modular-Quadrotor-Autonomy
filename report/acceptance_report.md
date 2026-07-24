# PLAN.md 验收对照表

> 自动生成时间：2026-07-24 00:02:06 CST
> 通过：**6/6** 场景

## 总览

| # | 场景 | launch | 结果 | 关键指标 |
|---|------|--------|------|----------|
| 1 | 悬停 | `hover.launch.py` | ✅ PASS | mean_err=0.0004 m; final=0.0001 m; planner_ok=False; hold=True |
| 2 | 单目标点 | `single_goal.launch.py` | ✅ PASS | mean_err=0.0093 m; final=0.0051 m; planner_ok=False; hold=True |
| 3 | 多目标点(正方形4点) | `multi_goal.launch.py` | ✅ PASS | mean_err=0.4895 m; final=0.0067 m; planner_ok=False; hold=True; wp=5/5 |
| 4 | 静态避障 | `avoidance.launch.py` | ✅ PASS | mean_err=9.3207 m; final=0.1027 m; min_obs=0.3132 m; planner_ok=True; hold=True; wp=8/8 |
| 5 | 狭窄通道绕行 | `narrow_passage.launch.py` | ✅ PASS | mean_err=0.8291 m; final=0.0003 m; min_obs=0.9852 m; planner_ok=True; hold=True |
| 6 | 稳定性展示 | `stability_demo.launch.py` | ✅ PASS | mean_err=0.0363 m; final=0.0521 m; planner_ok=False; hold=True |

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
  - `max_pos_err`: 0.0077 m
  - `final_pos_err`: 0.0001 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 0.0322 m
  - `detour_ratio`: 7.3892
  - `mean_jerk`: 0.1088 m/s^3
- launch 日志末尾（截断）：
```
[controller_node-2] [INFO] [1784805427.594458411] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1784805427.604257113] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1784805427.621117739] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784805427.621803877] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-6]: process started with pid [3037859]
[send_goal-6] [INFO] [1784805431.119727190] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[rviz2-5] [INFO] [1784805432.897559992] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784805432.897743397] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784805432.970946021] [rviz2]: Stereo is NOT SUPPORTED
```

### 场景 2：单目标点

- 说明：目标 (2,1,1.5)
- 评测图：

  ![scenario 2 evaluation](acceptance_runs/scenario_02_single_goal/evaluation.png)

- 检查项：
  - ✅ 到达目标误差≤0.3m
  - ✅ 最大误差≤3.0m(起飞过程)
- 补充检查（不计入原六项通过判定）：
  - ✅ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 1002
  - `mean_pos_err`: 0.0093 m
  - `max_pos_err`: 0.1099 m
  - `final_pos_err`: 0.0051 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 0.8020 m
  - `detour_ratio`: 11.1603
  - `mean_jerk`: 4.0363 m/s^3
- launch 日志末尾（截断）：
```
[dynamics_node-1] [INFO] [1784805496.925856630] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[viz_node-4] [INFO] [1784805496.931148102] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[map_node-3] [INFO] [1784805496.976030487] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784805496.977824404] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-6]: process started with pid [3038650]
[send_goal-6] [INFO] [1784805500.423372363] [send_goal]: Published goal (2.00, 1.00, 1.50) yaw=0.00 remaining=0 subs=1
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[rviz2-5] [INFO] [1784805502.292539554] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784805502.292735712] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784805502.359494455] [rviz2]: Stereo is NOT SUPPORTED
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
  - `mean_pos_err`: 0.4895 m
  - `max_pos_err`: 2.8156 m
  - `final_pos_err`: 0.0067 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 8.8292 m
  - `detour_ratio`: 5.8861
  - `mean_jerk`: 4.6989 m/s^3
- launch 日志末尾（截断）：
```
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[waypoint_publisher-6] [INFO] [1784812301.857079482] [waypoint_publisher]: Waypoint publisher: 4 pts/cycle × 1 cycles, mode=arrival,v<0.10, topic=/drone/goal
[rviz2-5] [INFO] [1784812301.898624882] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784812301.898924474] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784812301.936567636] [rviz2]: Stereo is NOT SUPPORTED
[waypoint_publisher-6] [INFO] [1784812302.347780955] [waypoint_publisher]: Waypoint cycle 1/1 1/4: (2.00, 0.00, 1.50)
[waypoint_publisher-6] [INFO] [1784812307.446894707] [waypoint_publisher]: Waypoint cycle 1/1 2/4: (2.00, 2.00, 1.50)
[waypoint_publisher-6] [INFO] [1784812311.046801676] [waypoint_publisher]: Waypoint cycle 1/1 3/4: (0.00, 2.00, 1.50)
[waypoint_publisher-6] [INFO] [1784812314.647144675] [waypoint_publisher]: Waypoint cycle 1/1 4/4: (0.00, 0.00, 1.50)
[waypoint_publisher-6] [INFO] [1784812318.246670145] [waypoint_publisher]: Waypoint sequence complete
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
  - `max_pos_err`: 19.6749 m
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
  - `flown_path_length`: 122.4693 m
  - `detour_ratio`: 13.4077
  - `mean_jerk`: 3.8788 m/s^3
  - `planned_tracking_error_mean`: 7.9707 m
- launch 日志末尾（截断）：
```
[local_sense_cloud-6] [INFO] [1784822187.786440050] [drone_0_local_sense_cloud]: global cloud cached: 31782 points
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[map_adapter-2] [INFO] [1784822189.450467528] [map_adapter]: adapted cloud #2940 width=33370 occ=136x88 topdown_cells=650
[local_sense_cloud-6] [INFO] [1784822189.533896003] [drone_0_local_sense_cloud]: local cloud #2950: 6884 pts @ (-7.9,-6.0,1.1)
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[ego_planner_node-5] [FSM]: state: WAIT_TARGET
[ego_planner_node-5] wait for goal or trigger.
[map_adapter-2] [INFO] [1784822191.452829143] [map_adapter]: adapted cloud #2960 width=33370 occ=136x88 topdown_cells=650
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
  - `mean_pos_err`: 0.8291 m
  - `max_pos_err`: 11.3824 m
  - `final_pos_err`: 0.0003 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 0.9852 m
  - `avoidance_safety_distance`: 0.3500 m
  - `avoidance_pass`: True
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 21.0806 m
  - `detour_ratio`: 1.8520
  - `mean_jerk`: 3.9245 m/s^3
  - `planned_tracking_error_mean`: 0.0242 m
- launch 日志末尾（截断）：
```
[map_node-3] [INFO] [1784822221.734576246] [drone_map]: drone_map ready: mode=narrow_corridor seed=42 attempt=0 connected=yes points=137567 (native cylinders+walls for dense/narrow; ego_* modes optional)
[planner_node-4] [INFO] [1784822221.755791718] [drone_planner]: Map ingested: 137567 raw -> 15616 voxels | origin=(-4.7,-4.7,-0.2) size=(29.4,19.4,4.7) res=0.25 inflate=0.08 [auto_fit] [auto_inflate]
[rviz2-6] [INFO] [1784822222.167282117] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-6] [INFO] [1784822222.167414175] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-6] [INFO] [1784822222.186602138] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [send_goal-7]: process started with pid [121756]
[planner_node-4] [INFO] [1784822226.592312239] [drone_planner]: New goal (17.00, 5.00, 1.50)
[send_goal-7] [INFO] [1784822226.602805300] [send_goal]: Published goal (17.00, 5.00, 1.50) yaw=0.00 remaining=0 subs=3
[planner_node-4] [INFO] [1784822226.618582613] [drone_planner]: Planning (1.0,5.0)->(17.0,5.0)
[planner_node-4] [INFO] [1784822226.618647194] [drone_planner]: Running A* with 0 peer keep-outs
[planner_node-4] [INFO] [1784822226.623975578] [drone_planner]: A* finished found=yes guide=87
[planner_node-4] [INFO] [1784822226.624065376] [drone_planner]: Planned path: 138 waypoints, length=26.78 m (horizontal avoidance)
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
  - `mean_pos_err`: 0.0363 m
  - `max_pos_err`: 0.0804 m
  - `final_pos_err`: 0.0521 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 4.1398 m
  - `detour_ratio`: 65.0275
  - `mean_jerk`: 1.3920 m/s^3
- launch 日志末尾（截断）：
```
[viz_node-4] [INFO] [1784822413.849002803] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1784822413.851074927] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1784822413.851421557] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1784822413.856399560] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784822413.857110382] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[rviz2-5] [INFO] [1784822414.287995474] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784822414.288147459] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-5] [INFO] [1784822414.316783865] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [interference_monitor-6]: process started with pid [122622]
[interference_monitor-6] [INFO] [1784822416.813790697] [interference_monitor]: Interference monitor ready (goal=(0.0, 0.0, 1.5), limit=0.3 m)
[INFO] [send_goal-7]: process started with pid [122650]
[send_goal-7] [INFO] [1784822417.811491217] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=2
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


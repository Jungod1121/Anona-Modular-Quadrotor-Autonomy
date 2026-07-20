# PLAN.md 验收对照表

> 自动生成时间：2026-07-19 21:21:32 CST
> 通过：**4/6** 场景

## 总览

| # | 场景 | launch | 结果 | 关键指标 |
|---|------|--------|------|----------|
| 1 | 悬停 | `hover.launch.py` | ✅ PASS | mean_err=0.0009 m; final=0.0001 m; planner_ok=False; hold=True |
| 2 | 单目标点 | `single_goal.launch.py` | ❌ FAIL | mean_err=6.9489 m; final=9.5205 m; planner_ok=False; hold=False |
| 3 | 多目标点(正方形4点) | `multi_goal.launch.py` | ✅ PASS | mean_err=0.5841 m; final=0.0010 m; planner_ok=False; hold=True; wp=4/4 |
| 4 | 静态避障 | `avoidance.launch.py` | ❌ FAIL | mean_err=22.1748 m; final=24.0416 m; min_obs=0.1503 m; planner_ok=True; hold=False |
| 5 | 狭窄通道绕行 | `narrow_passage.launch.py` | ✅ PASS | mean_err=0.7831 m; final=0.0003 m; min_obs=0.9826 m; planner_ok=True; hold=True |
| 6 | 稳定性展示 | `stability_demo.launch.py` | ✅ PASS | mean_err=0.0368 m; final=0.0473 m; planner_ok=False; hold=True |

## 分项检查

### 场景 1：悬停

- 说明：目标 (0,0,1.5)，3s 后自动发 goal
- 检查项：
  - ✅ 位置误差≤0.3m(末段均值)
  - ✅ 最终位置误差≤0.3m
- 补充检查（不计入原六项通过判定）：
  - ✅ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 902
  - `mean_pos_err`: 0.0009 m
  - `max_pos_err`: 0.0400 m
  - `final_pos_err`: 0.0001 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 0.0663 m
  - `detour_ratio`: 1.6580
  - `mean_jerk`: 0.0381 m/s^3
- launch 日志末尾：
```
[INFO] [rviz2-5]: process started with pid [670773]
[viz_node-4] [INFO] [1784466570.067624497] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[dynamics_node-1] [INFO] [1784466570.069364857] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[controller_node-2] [INFO] [1784466570.069893230] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[map_node-3] [INFO] [1784466570.073977834] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784466570.074843982] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-6]: process started with pid [670836]
[send_goal-6] [INFO] [1784466573.649545439] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[rviz2-5] [INFO] [1784466575.325631275] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784466575.325772530] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784466575.377890650] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [rviz2-5]: process has finished cleanly [pid 670773]
```

### 场景 2：单目标点

- 说明：目标 (2,1,1.5)
- 检查项：
  - ❌ 到达目标误差≤0.3m
  - ❌ 最大误差≤3.0m(起飞过程)
- 补充检查（不计入原六项通过判定）：
  - ❌ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 1002
  - `mean_pos_err`: 6.9489 m
  - `max_pos_err`: 9.7919 m
  - `final_pos_err`: 9.5205 m
  - `hover_pass_0.3m`: False
  - `goal_pass_0.3m`: False
  - `hold_at_goal_pass_0.3m`: False
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 11.6803 m
  - `detour_ratio`: 90.4377
  - `mean_jerk`: 3.7949 m/s^3
- launch 日志末尾：
```
[dynamics_node-1] [INFO] [1784466635.348430608] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[controller_node-2] [INFO] [1784466635.348674907] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[viz_node-4] [INFO] [1784466635.349173433] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[map_node-3] [INFO] [1784466635.355443675] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784466635.356772371] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[rviz2-5] [INFO] [1784466635.652068264] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784466635.652193499] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784466635.687264093] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [send_goal-6]: process started with pid [671368]
[send_goal-6] [INFO] [1784466639.297332846] [send_goal]: Published goal (2.00, 1.00, 1.50) yaw=0.00 remaining=0 subs=2
[rviz2-5] [INFO] [1784466654.837977831] [rviz2]: Setting goal pose: Frame:map, Position(10.9906, 3.74765, 0), Orientation(0, 0, 0.187503, 0.982264) = Angle: 0.377239
[INFO] [rviz2-5]: process has finished cleanly [pid 671260]
```

### 场景 3：多目标点(正方形4点)

- 说明：square side=2m, hold=8s/点, 5s 后开始发航点
- 检查项：
  - ✅ 4个角点均访问
  - ✅ 最终回到起点附近
- 原始指标：
  - `samples`: 2202
  - `mean_pos_err`: 0.5841 m
  - `max_pos_err`: 3.2212 m
  - `final_pos_err`: 0.0010 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 18.3089 m
  - `detour_ratio`: 8.8811
  - `mean_jerk`: 3.5971 m/s^3
- launch 日志末尾：
```
[INFO] [waypoint_publisher-6]: process started with pid [671838]
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[waypoint_publisher-6] [INFO] [1784466711.036340119] [waypoint_publisher]: Waypoint publisher: 5 points, hold=8.0s, topic=/drone/goal
[rviz2-5] [INFO] [1784466711.058421265] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784466711.058892200] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784466711.111627019] [rviz2]: Stereo is NOT SUPPORTED
[waypoint_publisher-6] [INFO] [1784466719.018419602] [waypoint_publisher]: Waypoint 1/5: (1.00, 1.00, 1.50)
[waypoint_publisher-6] [INFO] [1784466727.019111566] [waypoint_publisher]: Waypoint 2/5: (-1.00, 1.00, 1.50)
[waypoint_publisher-6] [INFO] [1784466735.019554833] [waypoint_publisher]: Waypoint 3/5: (-1.00, -1.00, 1.50)
[waypoint_publisher-6] [INFO] [1784466743.019125322] [waypoint_publisher]: Waypoint 4/5: (1.00, -1.00, 1.50)
[waypoint_publisher-6] [INFO] [1784466751.019852988] [waypoint_publisher]: Waypoint 5/5: (1.00, 1.00, 1.50)
[waypoint_publisher-6] [INFO] [1784466759.018855936] [waypoint_publisher]: Waypoint sequence complete
[INFO] [rviz2-5]: process has finished cleanly [pid 671774]
```

### 场景 4：静态避障

- 说明：dense_field 80圆柱+围墙, seed=42
- 检查项：
  - ❌ 到达目标误差≤0.5m
  - ❌ 最小障碍距离>0.35m
  - ✅ 规划器曾报告success
- 补充检查（不计入原六项通过判定）：
  - ❌ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 2263
  - `mean_pos_err`: 22.1748 m
  - `max_pos_err`: 24.3024 m
  - `final_pos_err`: 24.0416 m
  - `hover_pass_0.3m`: False
  - `goal_pass_0.3m`: False
  - `hold_at_goal_pass_0.3m`: False
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 0.1503 m
  - `avoidance_pass_0.35m`: False
  - `flown_path_length`: 33.3581 m
  - `detour_ratio`: 3.0604
  - `mean_jerk`: 7.5291 m/s^3
  - `planned_tracking_error_mean`: 0.0207 m
- launch 日志末尾：
```
[rviz2-7] Qt: Session management error: Could not open network socket
[rviz2-7] [INFO] [1784466843.790278153] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] [INFO] [1784466843.790404210] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-7] [INFO] [1784466843.822592852] [rviz2]: Stereo is NOT SUPPORTED
[planner_node-5] [INFO] [1784466845.007155781] [drone_planner]: Map ingested: 236173 raw -> 42881 voxels | origin=(-10.4,-10.4,-0.2) size=(60.7,44.7,4.7) res=0.25 inflate=0.08 [auto_fit] [auto_inflate]
[planner_node-5] [INFO] [1784466845.057041239] [drone_planner]: Planning (2.7,12.0)->(40.0,12.0)
[planner_node-5] [INFO] [1784466845.057096042] [drone_planner]: Running A* with 0 peer keep-outs
[planner_node-5] [WARN] [1784466845.058678765] [drone_planner]: DynA* failed at cruise_z=1.50 (band=±0.35) — fallback GridA*
[planner_node-5] [INFO] [1784466845.058966325] [drone_planner]: A* finished found=yes guide=150
[planner_node-5] [INFO] [1784466845.059081131] [drone_planner]: Planned path: 175 waypoints, length=39.75 m (horizontal avoidance)
[INFO] [rviz2-7]: process has finished cleanly [pid 672553]
[map_adapter-2] [INFO] [1784466877.514672609] [map_adapter]: adapted cloud #20 width=236173 occ=224x160 topdown_cells=3254
[map_adapter-2] [INFO] [1784466917.422543837] [map_adapter]: adapted cloud #40 width=236173 occ=224x160 topdown_cells=3254
[map_adapter-2] [INFO] [1784466957.423138105] [map_adapter]: adapted cloud #60 width=236173 occ=224x160 topdown_cells=3254
[map_adapter-2] [INFO] [1784466997.414237521] [map_adapter]: adapted cloud #80 width=236173 occ=224x160 topdown_cells=3254
```

### 场景 5：狭窄通道绕行

- 说明：narrow_corridor S-bend: 3×1.6m doors + side clutter (PLAN §5.3)
- 检查项：
  - ✅ 到达目标误差≤0.5m
  - ✅ 最小障碍距离>0.35m
  - ✅ 无A*失败日志
- 补充检查（不计入原六项通过判定）：
  - ✅ 末段悬停≤0.3m
- 原始指标：
  - `samples`: 2989
  - `mean_pos_err`: 0.7831 m
  - `max_pos_err`: 11.2093 m
  - `final_pos_err`: 0.0003 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: True
  - `final_planner_state`: EXEC_TRAJ
  - `min_obstacle_distance`: 0.9826 m
  - `avoidance_pass_0.35m`: True
  - `flown_path_length`: 20.6643 m
  - `detour_ratio`: 1.8435
  - `mean_jerk`: 28.7290 m/s^3
  - `planned_tracking_error_mean`: 0.0232 m
- launch 日志末尾：
```
[rviz2-6] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-6] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-6] Qt: Session management error: Could not open network socket
[planner_node-4] [INFO] [1784467013.413792574] [drone_planner]: Map ingested: 137567 raw -> 15616 voxels | origin=(-4.7,-4.7,-0.2) size=(29.4,19.4,4.7) res=0.25 inflate=0.08 [auto_fit] [auto_inflate]
[rviz2-6] [INFO] [1784467013.553878109] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-6] [INFO] [1784467013.554349755] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-6] [INFO] [1784467013.606290307] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [send_goal-7]: process started with pid [673429]
[planner_node-4] [INFO] [1784467017.885290004] [drone_planner]: New goal (17.00, 5.00, 1.50)
[send_goal-7] [INFO] [1784467017.903028587] [send_goal]: Published goal (17.00, 5.00, 1.50) yaw=0.00 remaining=0 subs=3
[planner_node-4] [INFO] [1784467017.915111885] [drone_planner]: Planning (1.0,5.0)->(17.0,5.0)
[planner_node-4] [INFO] [1784467017.915175234] [drone_planner]: Running A* with 0 peer keep-outs
[planner_node-4] [INFO] [1784467017.932252625] [drone_planner]: A* finished found=yes guide=87
[planner_node-4] [INFO] [1784467017.932385254] [drone_planner]: Planned path: 138 waypoints, length=26.78 m (horizontal avoidance)
[INFO] [rviz2-6]: process has finished cleanly [pid 673310]
```

### 场景 6：稳定性展示

- 说明：wind_enable+imu_noise_enable, 手动跑 evaluate
- 检查项：
  - ✅ 风扰下悬停误差≤0.3m
  - ✅ 评测图已生成
- 原始指标：
  - `samples`: 1801
  - `mean_pos_err`: 0.0368 m
  - `max_pos_err`: 0.0827 m
  - `final_pos_err`: 0.0473 m
  - `hover_pass_0.3m`: True
  - `goal_pass_0.3m`: True
  - `hold_at_goal_pass_0.3m`: True
  - `hold_at_goal_samples`: 30
  - `planner_success_ever`: False
  - `final_planner_state`: 
  - `flown_path_length`: 4.2773 m
  - `detour_ratio`: 67.7021
  - `mean_jerk`: 4.0780 m/s^3
- launch 日志末尾：
```
[INFO] [viz_node-4]: process started with pid [674129]
[INFO] [rviz2-5]: process started with pid [674131]
[viz_node-4] [INFO] [1784467187.868453275] [drone_visualization]: drone_visualization ready (quadrotor marker from /drone/odom)
[controller_node-2] [INFO] [1784467187.870542790] [drone_controller]: drone_controller ready: 100 Hz, mass=1.00, goal_fallback=on
[dynamics_node-1] [INFO] [1784467187.870847142] [drone_dynamics]: drone_dynamics ready: integ=500Hz pub=100Hz mass=1.00 (self-developed, not MARSIM/EGO sim)
[map_node-3] [INFO] [1784467187.876276029] [drone_map]: Map generated: mode=sparse seed=42 attempt=0 connected=true obstacles=0 points=0 downsample_voxel=0.000
[map_node-3] [INFO] [1784467187.877024465] [drone_map]: drone_map ready: mode=sparse seed=42 attempt=0 connected=yes points=0 (native cylinders+walls for dense/narrow; ego_* modes optional)
[INFO] [send_goal-6]: process started with pid [674192]
[send_goal-6] [INFO] [1784467191.433142367] [send_goal]: Published goal (0.00, 0.00, 1.50) yaw=0.00 remaining=0 subs=1
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] _IceTransSocketUNIXConnect: Cannot connect to non-local host jungod-ASUS-TUF-Gaming-A15-FA507XV-FA507XV
[rviz2-5] Qt: Session management error: Could not open network socket
[rviz2-5] [INFO] [1784467193.093599130] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1784467193.093710088] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1784467193.127466337] [rviz2]: Stereo is NOT SUPPORTED
```

## PLAN.md 硬指标对照

| 硬指标 | PLAN 要求 | 验收方式 | 当前状态 |
|--------|-----------|----------|----------|
| 悬停误差 | ≤ 0.3 m | scenario 1 evaluate.py | ✅ |
| 避障最小距离 | > 安全距离 0.35 m | scenario 4 min_obstacle_distance | ❌ |
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


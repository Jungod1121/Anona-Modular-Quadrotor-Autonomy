# Plant Interface Contract（动力学/控制层边界契约）

本文档定义 plant（`drone_dynamics`）与 controller（`drone_controller`）之间
**隐式接口的显式契约**。目标：让"换一个动力学模型"或"换一个控制器"成为
有据可依的工程操作，而不是靠读源码猜。

## 1. 话题契约

| 方向 | 话题 | 类型 | 语义 |
|---|---|---|---|
| controller → plant | `/drone/motor_rpm_cmd` | `drone_msgs/MotorCommand` | 四电机转速指令，单位 **RPM**，frame `base_link` |
| plant → controller | `/drone/odom` | `nav_msgs/Odometry` | pose 在 `map`(ENU)；**twist 为 body 系**（REP-105，child_frame=base_link） |
| plant → controller | `/drone/imu` | `sensor_msgs/Imu` | 角速度 body 系 |
| plant → 双方 | `/drone/plant_signature` | `std_msgs/String` | 锁存(latched) 的机身签名，见 §3 |

## 2. 物理耦合（不可分割的约定）

1. **Allocation 一致性**：plant 的力/扭矩分配矩阵与 controller 的混控矩阵
   必须从**同一组** `arm_length / k_F / k_M / mass` 推导。
2. **签名互检（运行时门禁）**：两侧各自用本地参数计算
   `quad_x/L={arm_length:.3e}/kF={k_F:.3e}/kM={k_M:.3e}`；
   dynamics 发布到 `/drone/plant_signature`，controller 订阅并比对，
   **不一致直接 FATAL 拒绝起飞**（防止单侧改 yaml 导致的静默扭矩失配）。
3. **方向约定**：0:FL-CCW / 1:FR-CW / 2:RR-CCW / 3:RL-CW（X 布局），
   正 RPM → 正推力；姿态小角近似反解已由 `test_mixer` / `test_allocation`
   与闭环测试锁定。

## 3. 安全行为（controller 侧内建，替换控制器时需自行实现等价物）

- **odom 看门狗**：`odom_timeout_sec`（默认 0.2s）内无里程计 → 切电机
- **目标 z 围栏**：`goal_z_min/max`（默认 0.35/1.9m）钳位一切规划器指令
- **指令超时**：`local_goal_timeout` / `trajectory_cmd_timeout` 后回退锚定悬停

## 4. 如何新增第二个动力学模型（检查单）

- [ ] 实现同一组输入输出话题（§1），twist 必须 body 系
- [ ] 支持相同参数名（`mass/arm_length/k_F/k_M/...`）并发布签名
- [ ] 通过 `test_closed_loop` 同款闭环测试（悬停误差、阶跃跟踪）
- [ ] launch 以 `plant:=<name>` 切换，controller 无需改动即应工作
- [ ] 若物理差异显著（如加机体阻力），在 controller 侧提供对应增益 profile

> 当前状态：仅有一个 plant 实现。本文件即 B 阶段降级交付的"接口隔离文档 +
> 启动校验"；第二实现待后续按此清单落地。

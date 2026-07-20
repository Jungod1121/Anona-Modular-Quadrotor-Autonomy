# Baseline interface snapshot 2026-07-16T22:58:25+08:00

## Package versions / sources
### drone_dynamics
总计 28
drwxrwxr-x  5 root root 4096  7月 13 19:38 .
drwxrwxr-x 17 root root 4096  7月 16 13:50 ..
-rw-rw-r--  1 root root 1455  7月 14 23:31 CMakeLists.txt
drwxrwxr-x  3 root root 4096  7月 13 19:10 include
/home/jungod/drone_ws/src/drone_dynamics/package.xml
### drone_controller
总计 28
drwxrwxr-x  5 root root 4096  7月 13 19:38 .
drwxrwxr-x 17 root root 4096  7月 16 13:50 ..
-rw-rw-r--  1 root root 1377  7月 14 23:31 CMakeLists.txt
drwxrwxr-x  3 root root 4096  7月 13 19:16 include
/home/jungod/drone_ws/src/drone_controller/package.xml
### drone_bringup
总计 48
drwxrwxr-x  6 root root 4096  7月 15 00:50 .
drwxrwxr-x 17 root root 4096  7月 16 13:50 ..
drwxrwxr-x  2 root root 4096  7月 14 00:42 config
drwxrwxr-x  4 root root 4096  7月 15 20:44 drone_bringup
/home/jungod/drone_ws/src/drone_bringup/package.xml
### drone_msgs
总计 24
drwxrwxr-x  4 root root 4096  7月 13 19:38 .
drwxrwxr-x 17 root root 4096  7月 16 13:50 ..
-rw-rw-r--  1 root root  630  7月 14 23:31 CMakeLists.txt
drwxrwxr-x  2 root root 4096  7月 13 21:55 msg
/home/jungod/drone_ws/src/drone_msgs/msg/PlannerStatus.msg
/home/jungod/drone_ws/src/drone_msgs/msg/TrajectoryCommand.msg
/home/jungod/drone_ws/src/drone_msgs/msg/MotorCommand.msg
/home/jungod/drone_ws/src/drone_msgs/package.xml
### drone_map
总计 24
drwxrwxr-x  4 root root 4096  7月 13 19:38 .
drwxrwxr-x 17 root root 4096  7月 16 13:50 ..
-rw-rw-r--  1 root root 1592  7月 14 23:31 CMakeLists.txt
drwxrwxr-x  3 root root 4096  7月 13 19:18 include
/home/jungod/drone_ws/src/drone_map/package.xml

## Dynamics sources
/home/jungod/drone_ws/src/drone_dynamics/include/drone_dynamics/quadrotor_model.hpp
/home/jungod/drone_ws/src/drone_dynamics/src/dynamics_node.cpp
/home/jungod/drone_ws/src/drone_dynamics/src/quadrotor_model.cpp
/home/jungod/drone_ws/src/drone_dynamics/test/test_allocation.cpp

## Controller sources
/home/jungod/drone_ws/src/drone_controller/include/drone_controller/cascade_pid.hpp
/home/jungod/drone_ws/src/drone_controller/src/cascade_pid.cpp
/home/jungod/drone_ws/src/drone_controller/src/controller_node.cpp
/home/jungod/drone_ws/src/drone_controller/test/test_mixer.cpp

## Messages
/home/jungod/drone_ws/src/drone_msgs/CMakeLists.txt
/home/jungod/drone_ws/src/drone_msgs/msg/MotorCommand.msg
/home/jungod/drone_ws/src/drone_msgs/msg/PlannerStatus.msg
/home/jungod/drone_ws/src/drone_msgs/msg/TrajectoryCommand.msg
/home/jungod/drone_ws/src/drone_msgs/package.xml

## Launch files
avoidance.launch.py
ego_avoidance.launch.py
ego_swarm.launch.py
fast_planner_avoidance.launch.py
formation.launch.py
fuel_explore.launch.py
gcopter_avoidance.launch.py
hover.launch.py
mighty_avoidance.launch.py
multi_drone.launch.py
multi_goal.launch.py
narrow_passage.launch.py
planner_sim.launch.py
__pycache__
rl_avoidance.launch.py
sac_avoidance.launch.py
shared_field.launch.py
single_goal.launch.py
stability_demo.launch.py

## Dynamics YAML
/**:
  ros__parameters:
    mass: 1.0
    gravity: 9.81
    arm_length: 0.18
    Ixx: 0.01
    Iyy: 0.01
    Izz: 0.02
    k_F: 3.0e-5
    k_M: 5.0e-7
    tau_motor: 0.02
    omega_min: 0.0
    omega_max: 800.0
    integration_rate: 500.0
    publish_rate: 100.0
    cmd_timeout: 0.5
    init_x: 0.0
    init_y: 0.0
    init_z: 0.0
    init_yaw: 0.0
    wind_enable: false
    wind_const_x: 0.3
    wind_const_y: -0.2
    wind_const_z: 0.0
    wind_sin_amp: 0.15
    wind_sin_freq: 0.25
    imu_noise_enable: false
    imu_accel_noise_std: 0.02
    imu_gyro_noise_std: 0.01
    imu_accel_bias_rw: 1.0e-4
    imu_gyro_bias_rw: 1.0e-5

## Controller YAML
/**:
  ros__parameters:
    mass: 1.0
    gravity: 9.81
    arm_length: 0.18
    k_F: 3.0e-5
    k_M: 5.0e-7
    control_rate: 100.0
    local_goal_timeout: 2.0
    trajectory_cmd_timeout: 0.25
    # Integral + DOB: wind recovery becomes better (not worse) when wind is on.
    pos_kp: {x: 1.2, y: 1.2, z: 1.8}
    pos_kd: {x: 1.4, y: 1.4, z: 2.0}
    pos_ki: {x: 0.35, y: 0.35, z: 0.55}
    pos_i_limit: {x: 1.2, y: 1.2, z: 1.5}
    disturbance_reject_enable: true
    disturbance_gain: 1.2
    disturbance_leak: 0.08
    disturbance_limit: {x: 1.8, y: 1.8, z: 2.5}
    imu_aid_enable: true
    imu_gyro_lpf_hz: 12.0
    imu_rate_blend: 0.65
    att_kp: {x: 6.0, y: 6.0, z: 3.0}
    att_kd: {x: 0.4, y: 0.4, z: 0.2}
    max_vel: 1.0
    max_acc: 1.5
    max_tilt: 0.35
    max_yaw_rate: 1.0
    max_torque: {x: 0.08, y: 0.08, z: 0.04}
    min_thrust: 0.0
    max_thrust: 0.0
    rpm_min: 0.0
    rpm_max: 7000.0
    goal_slowdown_dist: 3.0

## Build note
User-local setuptools 83 breaks ament_python (`--uninstall`). Use:
```bash
export PYTHONNOUSERSITE=1
export PYTHONPATH=/usr/lib/python3/dist-packages
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Canonical plant topics (contract)
- `/drone/motor_rpm_cmd` (drone_msgs/MotorCommand) — controller → dynamics
- `/drone/odom` (nav_msgs/Odometry)
- `/drone/imu` (sensor_msgs/Imu)
- `/drone/path` (nav_msgs/Path)
- `/drone/goal` (geometry_msgs/PoseStamped)
- `/tf` (world → body)
- `/planner/local_goal`, `/planner/trajectory_cmd`, `/planner/trajectory`, `/planner/status`
- `/map/obstacles` (sensor_msgs/PointCloud2)

## Reference repos (external, conceptual only)
- /home/jungod/reference_repos/pengyu_sim
- /home/jungod/reference_repos/MARSIM

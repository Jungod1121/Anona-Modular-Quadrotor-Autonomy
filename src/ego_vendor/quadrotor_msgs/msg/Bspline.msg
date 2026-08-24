# Classic Fast-Planner / plan_manage Bspline trajectory (no drone_id).
# Restored from build/quadrotor_msgs/rosidl_adapter artifacts after the
# reference_repos checkout lost it (see fast_planner_vendor/VENDOR_NOTES.md).
int32 order
int64 traj_id
builtin_interfaces/Time start_time
float64[] knots
geometry_msgs/Point[] pos_pts
float64[] yaw_pts
float64 yaw_dt

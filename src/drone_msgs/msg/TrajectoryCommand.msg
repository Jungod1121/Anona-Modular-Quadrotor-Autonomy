# traj_server-style feedforward command (position + velocity + acceleration + yaw).
# position/velocity/acceleration: map frame (ENU); units m, m/s, m/s^2.
# yaw/yaw_dot: yaw about map +Z; units rad, rad/s.
# trajectory_ready: when false the controller drops this message without tracking it.
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw
float64 yaw_dot
bool trajectory_ready

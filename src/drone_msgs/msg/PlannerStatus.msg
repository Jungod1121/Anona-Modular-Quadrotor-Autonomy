# Planner FSM status for evaluation / RViz / scripts
std_msgs/Header header
string state            # INIT / WAIT_TARGET / GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ / EMERGENCY_STOP / FAIL
bool success
string message
float64 path_length
float64 min_obstacle_distance

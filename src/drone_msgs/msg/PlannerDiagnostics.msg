# Extended planner diagnostics for evaluation / batch matrix
std_msgs/Header header
string planner_id
string state
bool fallback_active
string fallback_reason
float64 solve_time_ms
float64 clearance_m
float64 tracking_error_m
float64 last_replan_ms
uint32 replan_count
uint32 fail_count

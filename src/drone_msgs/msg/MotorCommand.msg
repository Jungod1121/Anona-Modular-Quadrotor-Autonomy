# Four motor RPM commands (X-layout: 0 FL, 1 FR, 2 RR, 3 RL)
# Units: rpm = revolutions per minute, one entry per motor.
# Frame: base_link (motor order is fixed relative to the body).
std_msgs/Header header
float64[4] rpm

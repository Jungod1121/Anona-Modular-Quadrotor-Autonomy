# Plant comparison: drone_ws vs pengyu_sim vs MARSIM

Conceptual audit only. References are not executed or wrapped.

| Aspect | Ours | pengyu_sim | MARSIM |
|--------|------|------------|--------|
| State | p,v,q,ω,motor_ω (rad/s) | p,v,quat,ang_vel,motor RPM | p,v,q,w; motors not filtered |
| Thrust | F=k_F ω² (rad/s) | F=k_F ω² (RPM) | F=k_F RPM² |
| Motor lag | single τ | τ_up / τ_down | none (instant) |
| Integration | Euler + clamps | Euler | Euler, wall-clock dt |
| Controller | cascade pos→att PD→mixer | same spirit + anti-windup + RPM slew | not in this tree |
| Rates | 500 Hz integ / 100 Hz pub+ctrl | ~200 Hz | ~200 Hz wall clock |
| Wind/IMU | wind force + IMU noise hooks | rich IMU model | truth IMU |

## Independent improvement targets
1. Asymmetric motor τ_up/τ_down
2. Optional disable of hard a/v/ω clamps for fidelity tests
3. Controller anti-windup + RPM rate limit
4. Expand unit tests: allocation signs, motor step response, quat norm, plant↔mixer A match, closed-loop settle
5. Align test k_F/k_M with YAML defaults (3e-5 / 5e-7)

## Provenance constraint
Any rewrite remains original ROS2 code under src/drone_dynamics and src/drone_controller.

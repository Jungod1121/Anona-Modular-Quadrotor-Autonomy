#pragma once
/**
 * Self-developed cascaded PID + mixer for X-layout quadrotor.
 * Allocation matrix matches drone_dynamics exactly (ω² domain, rad/s internally).
 */
#include <Eigen/Dense>
#include <cmath>

namespace drone_controller
{

struct ControllerParams
{
  double mass{1.0};
  double gravity{9.81};
  double arm_length{0.18};
  double k_F{3.0e-5};
  double k_M{5.0e-7};

  Eigen::Vector3d pos_kp{1.2, 1.2, 2.0};
  Eigen::Vector3d pos_kd{1.0, 1.0, 1.5};
  Eigen::Vector3d pos_ki{0.35, 0.35, 0.55};
  Eigen::Vector3d pos_i_limit{1.2, 1.2, 1.5};

  Eigen::Vector3d att_kp{6.0, 6.0, 3.0};
  Eigen::Vector3d att_kd{0.4, 0.4, 0.2};

  double max_vel{2.0};
  double max_acc{3.0};
  double max_tilt{0.45};       // rad
  double max_yaw_rate{1.0};    // rad/s
  Eigen::Vector3d max_torque{0.08, 0.08, 0.04};

  double min_thrust{0.0};
  double max_thrust{0.0};      // 0 → auto: 4*k_F*(rpm_max in rad/s)²
  double rpm_min{0.0};
  double rpm_max{7000.0};
  double max_motor_rpm_rate{12000.0};  // RPM/s slew limit (0 = disabled)

  double goal_slowdown_dist{3.0};  // beyond this, cap desired speed/accel toward goal

  /** Constant-disturbance observer (wind recovery). Adds accel cancelation. */
  bool disturbance_reject_enable{true};
  double disturbance_gain{1.2};       // 1/s on force/mass estimate
  double disturbance_leak{0.08};      // 1/s leak so estimate does not wind up forever
  Eigen::Vector3d disturbance_limit{1.8, 1.8, 2.5};  // m/s^2
};

struct VehicleState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d omega{Eigen::Vector3d::Zero()};
  bool valid{false};
};

struct GoalState
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_dot{0.0};
  bool use_feedforward{false};
  bool valid{false};
};

/**
 * Motor order (X layout): 0 FL CCW, 1 FR CW, 2 RR CCW, 3 RL CW.
 * [T, τx, τy, τz]^T = A · [ω0², ω1², ω2², ω3²]^T  (ω in rad/s)
 */
class Mixer
{
public:
  Mixer() = default;
  Mixer(double arm_length, double k_F, double k_M);

  void configure(double arm_length, double k_F, double k_M);

  const Eigen::Matrix4d & A() const { return A_; }
  const Eigen::Matrix4d & AInv() const { return A_inv_; }

  Eigen::Vector4d wrenchToOmegaSq(const Eigen::Vector4d & wrench) const;
  Eigen::Vector4d omegaSqToWrench(const Eigen::Vector4d & omega_sq) const;

  static double rpmToRad(double rpm) { return rpm * 2.0 * M_PI / 60.0; }
  static double radToRpm(double rad) { return rad * 60.0 / (2.0 * M_PI); }

private:
  Eigen::Matrix4d A_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d A_inv_{Eigen::Matrix4d::Identity()};
};

class CascadePid
{
public:
  explicit CascadePid(const ControllerParams & params = ControllerParams{});

  void setParams(const ControllerParams & params);
  const ControllerParams & params() const { return params_; }
  const Mixer & mixer() const { return mixer_; }

  /** Compute four motor RPM commands from state and goal. */
  Eigen::Vector4d compute(const VehicleState & state, const GoalState & goal, double dt);

  void reset();

private:
  static double clamp(double v, double lo, double hi);
  static double wrapAngle(double a);
  static Eigen::Vector3d quatToRpy(const Eigen::Quaterniond & q);

  Eigen::Vector3d limitDesiredVelocity(
    const Eigen::Vector3d & pos_err,
    const Eigen::Vector3d & vel) const;

  ControllerParams params_;
  Mixer mixer_;
  Eigen::Vector3d pos_integral_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d disturbance_acc_{Eigen::Vector3d::Zero()};  // estimated a_dist (world)
  Eigen::Vector4d last_rpm_{Eigen::Vector4d::Zero()};
  bool rpm_initialized_{false};
  double filtered_yaw_{0.0};
  bool yaw_initialized_{false};
};

}  // namespace drone_controller

#include "drone_controller/cascade_pid.hpp"

#include <algorithm>

namespace drone_controller
{

Mixer::Mixer(double arm_length, double k_F, double k_M)
{
  configure(arm_length, k_F, k_M);
}

void Mixer::configure(double arm_length, double k_F, double k_M)
{
  const double l = arm_length / std::sqrt(2.0);
  A_ <<
    k_F,          k_F,          k_F,          k_F,
    k_F * l,     -k_F * l,     -k_F * l,      k_F * l,
   -k_F * l,     -k_F * l,      k_F * l,      k_F * l,
    k_M,         -k_M,          k_M,         -k_M;
  A_inv_ = A_.inverse();
}

Eigen::Vector4d Mixer::wrenchToOmegaSq(const Eigen::Vector4d & wrench) const
{
  return A_inv_ * wrench;
}

Eigen::Vector4d Mixer::omegaSqToWrench(const Eigen::Vector4d & omega_sq) const
{
  return A_ * omega_sq;
}

CascadePid::CascadePid(const ControllerParams & params)
: params_(params),
  mixer_(params.arm_length, params.k_F, params.k_M)
{
  setParams(params);
}

void CascadePid::setParams(const ControllerParams & params)
{
  params_ = params;
  mixer_.configure(params.arm_length, params.k_F, params.k_M);

  if (params_.max_thrust <= params_.min_thrust) {
    const double omega_max = Mixer::rpmToRad(params_.rpm_max);
    params_.max_thrust = 4.0 * params_.k_F * omega_max * omega_max;
    params_.min_thrust = 4.0 * params_.k_F * Mixer::rpmToRad(params_.rpm_min) *
      Mixer::rpmToRad(params_.rpm_min);
  }
}

void CascadePid::reset()
{
  pos_integral_.setZero();
  disturbance_acc_.setZero();
  yaw_initialized_ = false;
}

double CascadePid::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double CascadePid::wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

Eigen::Vector3d CascadePid::quatToRpy(const Eigen::Quaterniond & q)
{
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = (std::abs(sinp) >= 1.0) ?
    std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Vector3d CascadePid::limitDesiredVelocity(
  const Eigen::Vector3d & pos_err,
  const Eigen::Vector3d & vel) const
{
  const double dist = pos_err.norm();
  Eigen::Vector3d v_des = params_.pos_kp.cwiseProduct(pos_err);

  if (dist > params_.goal_slowdown_dist && dist > 1e-6) {
    const Eigen::Vector3d dir = pos_err / dist;
    const double along = v_des.dot(dir);
    v_des = dir * clamp(along, -params_.max_vel, params_.max_vel);
  } else {
    const double v_norm = v_des.norm();
    if (v_norm > params_.max_vel) {
      v_des *= params_.max_vel / v_norm;
    }
  }

  (void)vel;
  return v_des;
}

Eigen::Vector4d CascadePid::compute(
  const VehicleState & state,
  const GoalState & goal,
  double dt)
{
  Eigen::Vector4d rpm = Eigen::Vector4d::Zero();
  if (!state.valid || dt <= 0.0) {
    return rpm;
  }

  const Eigen::Vector3d rpy = quatToRpy(state.attitude);
  if (!yaw_initialized_) {
    filtered_yaw_ = goal.valid ? goal.yaw : rpy.z();
    yaw_initialized_ = true;
  }

  const Eigen::Vector3d target_pos = goal.valid ? goal.position : state.position;
  const double target_yaw = goal.valid ? goal.yaw : filtered_yaw_;

  const Eigen::Vector3d pos_err = target_pos - state.position;
  const double dist = pos_err.norm();
  const double speed = state.velocity.norm();
  // Wind / bias recovery ONLY when settling near a fixed hover goal.
  // Applying DOB/Ki while chasing planner local_goal pushes the drone
  // into obstacles (was min_obs regression on dense_field).
  const bool settling =
    goal.valid &&
    !goal.use_feedforward &&
    dist < 0.85 &&
    speed < 0.45;

  if (settling) {
    pos_integral_ += pos_err * dt;
  } else {
    pos_integral_ *= std::max(0.0, 1.0 - 3.0 * dt);
  }
  for (int i = 0; i < 3; ++i) {
    pos_integral_(i) = clamp(
      pos_integral_(i),
      -params_.pos_i_limit(i),
      params_.pos_i_limit(i));
  }

  if (params_.disturbance_reject_enable && settling) {
    disturbance_acc_ +=
      (params_.disturbance_gain * pos_err - params_.disturbance_leak * disturbance_acc_) * dt;
    for (int i = 0; i < 3; ++i) {
      disturbance_acc_(i) = clamp(
        disturbance_acc_(i),
        -params_.disturbance_limit(i),
        params_.disturbance_limit(i));
    }
  } else {
    disturbance_acc_ *= std::max(0.0, 1.0 - 4.0 * dt);
  }

  Eigen::Vector3d v_des;
  if (goal.valid && goal.use_feedforward) {
    v_des = goal.velocity;
  } else {
    v_des = limitDesiredVelocity(pos_err, state.velocity);
  }
  const Eigen::Vector3d vel_err = v_des - state.velocity;

  Eigen::Vector3d a_des =
    params_.pos_kp.cwiseProduct(pos_err) +
    params_.pos_kd.cwiseProduct(vel_err);
  if (settling) {
    a_des += params_.pos_ki.cwiseProduct(pos_integral_) + disturbance_acc_;
  }
  if (goal.valid && goal.use_feedforward) {
    a_des += 0.55 * goal.acceleration;
  }

  // Trapezoidal-style accel cap when goal is far
  if (dist > params_.goal_slowdown_dist) {
    const double a_norm = a_des.norm();
    if (a_norm > params_.max_acc) {
      a_des *= params_.max_acc / a_norm;
    }
  } else {
    a_des.x() = clamp(a_des.x(), -params_.max_acc, params_.max_acc);
    a_des.y() = clamp(a_des.y(), -params_.max_acc, params_.max_acc);
    a_des.z() = clamp(a_des.z(), -params_.max_acc, params_.max_acc);
  }

  // Gravity feedforward: at hover a_des ≈ 0 ⇒ T = m·g
  double thrust = params_.mass * (params_.gravity + a_des.z());
  thrust = clamp(thrust, params_.min_thrust, params_.max_thrust);

  // Small-angle roll/pitch from horizontal desired acceleration
  const double g_eff = std::max(0.5, params_.gravity + a_des.z());
  const double yaw_hold = filtered_yaw_;
  double roll_des = std::atan2(
    a_des.x() * std::sin(yaw_hold) - a_des.y() * std::cos(yaw_hold),
    g_eff);
  double pitch_des = std::atan2(
    a_des.x() * std::cos(yaw_hold) + a_des.y() * std::sin(yaw_hold),
    g_eff);
  roll_des = clamp(roll_des, -params_.max_tilt, params_.max_tilt);
  pitch_des = clamp(pitch_des, -params_.max_tilt, params_.max_tilt);

  const double yaw_err = wrapAngle(target_yaw - filtered_yaw_);
  const double yaw_step = goal.valid && goal.use_feedforward ?
    clamp(
      goal.yaw_dot * dt + yaw_err * 0.5,
      -params_.max_yaw_rate * dt,
      params_.max_yaw_rate * dt) :
    clamp(
      yaw_err,
      -params_.max_yaw_rate * dt,
      params_.max_yaw_rate * dt);
  filtered_yaw_ = wrapAngle(filtered_yaw_ + yaw_step);

  Eigen::Vector3d att_err(roll_des - rpy.x(), pitch_des - rpy.y(), yaw_step);
  att_err.x() = wrapAngle(att_err.x());
  att_err.y() = wrapAngle(att_err.y());

  const Eigen::Vector3d rate_des(0.0, 0.0, yaw_step / dt);
  const Eigen::Vector3d rate_err = rate_des - state.omega;

  Eigen::Vector3d tau =
    params_.att_kp.cwiseProduct(att_err) +
    params_.att_kd.cwiseProduct(rate_err);

  for (int i = 0; i < 3; ++i) {
    tau(i) = clamp(tau(i), -params_.max_torque(i), params_.max_torque(i));
  }

  Eigen::Vector4d wrench;
  wrench << thrust, tau.x(), tau.y(), tau.z();

  Eigen::Vector4d omega_sq = mixer_.wrenchToOmegaSq(wrench);
  for (int i = 0; i < 4; ++i) {
    omega_sq(i) = std::max(0.0, omega_sq(i));
    rpm(i) = Mixer::radToRpm(std::sqrt(omega_sq(i)));
    rpm(i) = clamp(rpm(i), params_.rpm_min, params_.rpm_max);
  }

  return rpm;
}

}  // namespace drone_controller

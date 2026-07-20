#include "drone_dynamics/quadrotor_model.hpp"

#include <algorithm>
#include <cmath>

namespace drone_dynamics
{

AllocationMatrix::AllocationMatrix(double arm_length, double k_F, double k_M)
{
  const double l = arm_length / std::sqrt(2.0);  // lever arm along body x/y
  // Columns: motor 0 FL, 1 FR, 2 RR, 3 RL  (ω_i²)
  // Row0: thrust
  // Row1: roll τx  (positive roll: left side up → FL+RL positive thrust moment about +x)
  // Row2: pitch τy (positive pitch: nose up → rear positive)
  // Row3: yaw τz   (CCW motors positive: 0,2)
  A_ <<
    k_F,          k_F,          k_F,          k_F,
    k_F * l,     -k_F * l,     -k_F * l,      k_F * l,
   -k_F * l,     -k_F * l,      k_F * l,      k_F * l,
    k_M,         -k_M,          k_M,         -k_M;
  A_inv_ = A_.inverse();
}

Eigen::Vector4d AllocationMatrix::wrenchToOmegaSq(const Eigen::Vector4d & wrench) const
{
  return A_inv_ * wrench;
}

Eigen::Vector4d AllocationMatrix::omegaSqToWrench(const Eigen::Vector4d & omega_sq) const
{
  return A_ * omega_sq;
}

QuadrotorModel::QuadrotorModel(const DynamicsParams & params)
: params_(params),
  allocation_(params.arm_length, params.k_F, params.k_M)
{
}

void QuadrotorModel::setParams(const DynamicsParams & params)
{
  params_ = params;
  allocation_ = AllocationMatrix(params.arm_length, params.k_F, params.k_M);
}

Eigen::Vector3d QuadrotorModel::windForce(double sim_time) const
{
  if (!params_.wind_enable) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d f(
    params_.wind_const_x,
    params_.wind_const_y,
    params_.wind_const_z);
  f.x() += params_.wind_sin_amp * std::sin(2.0 * M_PI * params_.wind_sin_freq * sim_time);
  return f;
}

double QuadrotorModel::nextGaussian()
{
  // Box-Muller with simple LCG
  rng_state_ = 1664525u * rng_state_ + 1013904223u;
  const double u1 = (rng_state_ & 0xffffff) / static_cast<double>(0x1000000);
  rng_state_ = 1664525u * rng_state_ + 1013904223u;
  const double u2 = (rng_state_ & 0xffffff) / static_cast<double>(0x1000000);
  const double uu = std::max(u1, 1e-12);
  return std::sqrt(-2.0 * std::log(uu)) * std::cos(2.0 * M_PI * u2);
}

void QuadrotorModel::step(const Eigen::Vector4d & motor_rpm_cmd, double dt, double sim_time)
{
  // Motor first-order with asymmetric τ_up / τ_down (pengyu_sim concept, own code).
  Eigen::Vector4d omega_cmd;
  for (int i = 0; i < 4; ++i) {
    omega_cmd(i) = std::clamp(rpmToRad(motor_rpm_cmd(i)), params_.omega_min, params_.omega_max);
  }
  for (int i = 0; i < 4; ++i) {
    const double err = omega_cmd(i) - state_.motor_omega(i);
    double tau = params_.tau_motor;
    if (params_.tau_motor_up > 0.0 && params_.tau_motor_down > 0.0) {
      tau = (err >= 0.0) ? params_.tau_motor_up : params_.tau_motor_down;
    }
    tau = std::max(tau, 1e-4);
    state_.motor_omega(i) += err * (dt / tau);
    state_.motor_omega(i) = std::clamp(state_.motor_omega(i), params_.omega_min, params_.omega_max);
  }

  Eigen::Vector4d omega_sq;
  for (int i = 0; i < 4; ++i) {
    omega_sq(i) = state_.motor_omega(i) * state_.motor_omega(i);
  }
  const Eigen::Vector4d wrench = allocation_.omegaSqToWrench(omega_sq);
  const double T = wrench(0);
  const Eigen::Vector3d tau(wrench(1), wrench(2), wrench(3));

  const Eigen::Matrix3d R = state_.q.toRotationMatrix();
  const Eigen::Vector3d thrust_world = R * Eigen::Vector3d(0.0, 0.0, T);
  const Eigen::Vector3d gravity(0.0, 0.0, -params_.mass * params_.gravity);
  const Eigen::Vector3d F_ext = windForce(sim_time);
  const Eigen::Vector3d a = (thrust_world + gravity + F_ext) / params_.mass;
  last_body_accel_ = R.transpose() * (a + Eigen::Vector3d(0, 0, params_.gravity));  // specific force sense

  Eigen::Vector3d a_use = a;
  if (params_.enable_state_clamps) {
    const double a_norm = a_use.norm();
    constexpr double kMaxAccel = 25.0;  // m/s^2
    if (a_norm > kMaxAccel) {
      a_use *= kMaxAccel / a_norm;
    }
  }
  state_.v += a_use * dt;
  if (params_.enable_state_clamps) {
    constexpr double kMaxVel = 8.0;  // m/s
    const double v_norm = state_.v.norm();
    if (v_norm > kMaxVel) {
      state_.v *= kMaxVel / v_norm;
    }
  }
  state_.p += state_.v * dt;
  // Ground contact: unilateral constraint + light horizontal friction (anti-slide).
  if (state_.p.z() < 0.0) {
    state_.p.z() = 0.0;
    if (state_.v.z() < 0.0) {
      state_.v.z() = 0.0;
    }
  }
  if (state_.p.z() <= 1e-3 && params_.ground_friction > 0.0) {
    const double damp = std::exp(-params_.ground_friction * dt);
    state_.v.x() *= damp;
    state_.v.y() *= damp;
  }

  // Rotation: I ω̇ = τ - ω × (I ω)
  Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
  I(0, 0) = params_.Ixx;
  I(1, 1) = params_.Iyy;
  I(2, 2) = params_.Izz;
  const Eigen::Vector3d Iomega = I * state_.omega;
  const Eigen::Vector3d omega_dot = I.inverse() * (tau - state_.omega.cross(Iomega));
  state_.omega += omega_dot * dt;
  if (params_.enable_state_clamps) {
    constexpr double kMaxOmega = 12.0;  // rad/s — prevents TF NaN after flips
    const double w_norm = state_.omega.norm();
    if (w_norm > kMaxOmega) {
      state_.omega *= kMaxOmega / w_norm;
    }
  }

  // Quaternion integrate: q̇ = 0.5 * q ⊗ [0, ω], then renormalize each step
  const Eigen::Quaterniond omega_q(0.0, state_.omega.x(), state_.omega.y(), state_.omega.z());
  const Eigen::Quaterniond dq = state_.q * omega_q;
  state_.q.w() += 0.5 * dq.w() * dt;
  state_.q.x() += 0.5 * dq.x() * dt;
  state_.q.y() += 0.5 * dq.y() * dt;
  state_.q.z() += 0.5 * dq.z() * dt;
  const double qn = state_.q.norm();
  if (!std::isfinite(qn) || qn < 1e-8) {
    // Recover from numerical blow-up instead of publishing NaN TF.
    state_.q = Eigen::Quaterniond::Identity();
    state_.omega.setZero();
    state_.v.setZero();
    if (!std::isfinite(state_.p.x()) || !std::isfinite(state_.p.y()) || !std::isfinite(state_.p.z())) {
      state_.p = Eigen::Vector3d(0.0, 0.0, 0.1);
    }
    state_.p.z() = std::max(0.1, state_.p.z());
  } else {
    state_.q.normalize();
  }
}

void QuadrotorModel::sampleImu(Eigen::Vector3d & accel_out, Eigen::Vector3d & gyro_out, double dt)
{
  accel_out = last_body_accel_;
  gyro_out = state_.omega;
  if (!params_.imu_noise_enable) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    accel_bias_(i) += params_.imu_accel_bias_rw * std::sqrt(dt) * nextGaussian();
    gyro_bias_(i) += params_.imu_gyro_bias_rw * std::sqrt(dt) * nextGaussian();
    accel_out(i) += accel_bias_(i) + params_.imu_accel_noise_std * nextGaussian();
    gyro_out(i) += gyro_bias_(i) + params_.imu_gyro_noise_std * nextGaussian();
  }
}

}  // namespace drone_dynamics

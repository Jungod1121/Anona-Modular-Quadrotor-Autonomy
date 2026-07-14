#pragma once
/**
 * Self-developed X-layout quadrotor dynamics model.
 * Do NOT reference MARSIM mars_drone_sim or EGO-Planner so3_quadrotor_simulator.
 * Allocation matrix design cross-checked conceptually against pengyu_sim
 * quadrotor_dynamics (RPM in → wrench), but equations implemented from first principles.
 */
#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace drone_dynamics
{

struct DynamicsParams
{
  double mass{1.0};
  double gravity{9.81};
  double arm_length{0.18};  // L, meters
  double Ixx{0.01};
  double Iyy{0.01};
  double Izz{0.02};
  double k_F{3.0e-5};       // thrust coeff: F = k_F * omega^2 (hover ~286 rad/s for 1kg)
  double k_M{5.0e-7};       // torque coeff: M = k_M * omega^2
  double tau_motor{0.02};   // motor first-order time constant [s]
  double omega_min{0.0};
  double omega_max{800.0};  // rad/s internal limit; MotorCommand topic uses RPM
  bool wind_enable{false};
  double wind_const_x{0.0};
  double wind_const_y{0.0};
  double wind_const_z{0.0};
  double wind_sin_amp{0.0};
  double wind_sin_freq{0.2};
  bool imu_noise_enable{false};
  double imu_accel_noise_std{0.02};
  double imu_gyro_noise_std{0.01};
  double imu_accel_bias_rw{1e-4};
  double imu_gyro_bias_rw{1e-5};
};

struct DroneState
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};           // position in map (ENU)
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};           // velocity in map
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()}; // body→map
  Eigen::Vector3d omega{Eigen::Vector3d::Zero()};       // body angular rate
  Eigen::Vector4d motor_omega{Eigen::Vector4d::Zero()}; // rad/s
};

/**
 * Motor order (X layout, top view, x forward / y left):
 *   0: Front-Left  (CCW, +yaw torque)
 *   1: Front-Right (CW,  -yaw torque)
 *   2: Rear-Right  (CCW, +yaw torque)
 *   3: Rear-Left   (CW,  -yaw torque)
 * Positions at (±L/√2, ±L/√2). Equal RPM ⇒ τx=τy=τz=0, T=4*kF*ω².
 */
class AllocationMatrix
{
public:
  explicit AllocationMatrix(double arm_length, double k_F, double k_M);

  /** [T, τx, τy, τz]^T = A * [ω0², ω1², ω2², ω3²]^T */
  Eigen::Matrix4d A() const { return A_; }

  /** Solve ω² from desired wrench (pseudo-inverse / exact inverse of A). */
  Eigen::Vector4d wrenchToOmegaSq(const Eigen::Vector4d & wrench) const;

  Eigen::Vector4d omegaSqToWrench(const Eigen::Vector4d & omega_sq) const;

private:
  Eigen::Matrix4d A_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d A_inv_{Eigen::Matrix4d::Identity()};
};

class QuadrotorModel
{
public:
  explicit QuadrotorModel(const DynamicsParams & params);

  void setParams(const DynamicsParams & params);
  const DynamicsParams & params() const { return params_; }
  DroneState & state() { return state_; }
  const DroneState & state() const { return state_; }
  const AllocationMatrix & allocation() const { return allocation_; }

  /** Integrate one fixed step. motor_rpm_cmd in RPM. */
  void step(const Eigen::Vector4d & motor_rpm_cmd, double dt, double sim_time);

  /** True body-frame specific force / gyro for IMU (before optional noise). */
  Eigen::Vector3d bodyAccel() const { return last_body_accel_; }
  Eigen::Vector3d bodyGyro() const { return state_.omega; }

  /** Apply Gaussian noise + bias random walk if enabled; updates bias state. */
  void sampleImu(Eigen::Vector3d & accel_out, Eigen::Vector3d & gyro_out, double dt);

  static double rpmToRad(double rpm) { return rpm * 2.0 * M_PI / 60.0; }
  static double radToRpm(double rad) { return rad * 60.0 / (2.0 * M_PI); }

private:
  DynamicsParams params_;
  DroneState state_;
  AllocationMatrix allocation_;
  Eigen::Vector3d last_body_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  unsigned rng_state_{1};

  double nextGaussian();
  Eigen::Vector3d windForce(double sim_time) const;
};

}  // namespace drone_dynamics

#include "drone_dynamics/quadrotor_model.hpp"
#include <gtest/gtest.h>
#include <cmath>

using drone_dynamics::AllocationMatrix;
using drone_dynamics::DynamicsParams;
using drone_dynamics::QuadrotorModel;

namespace {
constexpr double kF = 3.0e-5;
constexpr double kM = 5.0e-7;
constexpr double kL = 0.18;
}  // namespace

TEST(Allocation, EqualRpmGivesZeroTorque)
{
  AllocationMatrix A(kL, kF, kM);
  Eigen::Vector4d wsq = Eigen::Vector4d::Constant(1e6);
  Eigen::Vector4d wrench = A.omegaSqToWrench(wsq);
  EXPECT_NEAR(wrench(1), 0.0, 1e-9);
  EXPECT_NEAR(wrench(2), 0.0, 1e-9);
  EXPECT_NEAR(wrench(3), 0.0, 1e-9);
  EXPECT_NEAR(wrench(0), 4.0 * kF * 1e6, 1e-6);
}

TEST(Allocation, InverseRoundTrip)
{
  AllocationMatrix A(kL, kF, kM);
  Eigen::Vector4d wrench(4.0 * kF * 1e6, 0.01, -0.02, 0.005);
  Eigen::Vector4d wsq = A.wrenchToOmegaSq(wrench);
  Eigen::Vector4d back = A.omegaSqToWrench(wsq);
  EXPECT_NEAR((wrench - back).norm(), 0.0, 1e-8);
}

TEST(Allocation, HoverThrustPositiveOmega)
{
  DynamicsParams p;
  AllocationMatrix A(p.arm_length, p.k_F, p.k_M);
  const double T_hover = p.mass * p.gravity;
  Eigen::Vector4d wrench(T_hover, 0, 0, 0);
  Eigen::Vector4d wsq = A.wrenchToOmegaSq(wrench);
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(wsq(i), 0.0);
  }
  const double omega_h = std::sqrt(T_hover / (4.0 * p.k_F));
  EXPECT_NEAR(std::sqrt(wsq(0)), omega_h, 1e-6);
}

TEST(Allocation, Motor0SignProbe)
{
  // Motor 0 FL CCW: +τx, -τy, +τz
  AllocationMatrix A(kL, kF, kM);
  Eigen::Vector4d wsq = Eigen::Vector4d::Zero();
  wsq(0) = 1e6;
  const Eigen::Vector4d wrench = A.omegaSqToWrench(wsq);
  EXPECT_GT(wrench(1), 0.0);
  EXPECT_LT(wrench(2), 0.0);
  EXPECT_GT(wrench(3), 0.0);
}

TEST(Allocation, Motor1SignProbe)
{
  // Motor 1 FR CW: -τx, -τy, -τz
  AllocationMatrix A(kL, kF, kM);
  Eigen::Vector4d wsq = Eigen::Vector4d::Zero();
  wsq(1) = 1e6;
  const Eigen::Vector4d wrench = A.omegaSqToWrench(wsq);
  EXPECT_LT(wrench(1), 0.0);
  EXPECT_LT(wrench(2), 0.0);
  EXPECT_LT(wrench(3), 0.0);
}

TEST(Model, HoverRpmNearEquilibrium)
{
  DynamicsParams p;
  QuadrotorModel model(p);
  AllocationMatrix A(p.arm_length, p.k_F, p.k_M);
  const double T_hover = p.mass * p.gravity;
  Eigen::Vector4d wsq = A.wrenchToOmegaSq(Eigen::Vector4d(T_hover, 0, 0, 0));
  Eigen::Vector4d rpm;
  for (int i = 0; i < 4; ++i) {
    rpm(i) = QuadrotorModel::radToRpm(std::sqrt(wsq(i)));
    model.state().motor_omega(i) = QuadrotorModel::rpmToRad(rpm(i));
  }
  model.state().p.z() = 1.5;
  for (int k = 0; k < 500; ++k) {
    model.step(rpm, 0.002, k * 0.002);
  }
  EXPECT_NEAR(model.state().p.z(), 1.5, 0.15);
  EXPECT_LT(model.state().v.norm(), 0.3);
  EXPECT_NEAR(model.state().q.norm(), 1.0, 1e-9);
}

TEST(Model, MotorStepAsymmetricTau)
{
  DynamicsParams p;
  p.tau_motor_up = 0.02;
  p.tau_motor_down = 0.05;
  QuadrotorModel model(p);
  const double omega_cmd = 300.0;
  const double rpm = QuadrotorModel::radToRpm(omega_cmd);
  Eigen::Vector4d cmd = Eigen::Vector4d::Constant(rpm);
  // At t=τ_up, expect ~ (1-1/e) of step for rising edge.
  const int steps = static_cast<int>(std::lround(p.tau_motor_up / 0.001));
  for (int k = 0; k < steps; ++k) {
    model.step(cmd, 0.001, k * 0.001);
  }
  const double expected = (1.0 - std::exp(-1.0)) * omega_cmd;
  EXPECT_NEAR(model.state().motor_omega(0), expected, 8.0);

  // Fall: command zero, use τ_down.
  cmd.setZero();
  const double start = model.state().motor_omega(0);
  const int steps_down = static_cast<int>(std::lround(p.tau_motor_down / 0.001));
  for (int k = 0; k < steps_down; ++k) {
    model.step(cmd, 0.001, k * 0.001);
  }
  const double expected_down = start * std::exp(-1.0);
  EXPECT_NEAR(model.state().motor_omega(0), expected_down, 8.0);
}

TEST(Model, FreeFallNoThrust)
{
  DynamicsParams p;
  p.enable_state_clamps = false;
  p.ground_friction = 0.0;
  QuadrotorModel model(p);
  model.state().p.z() = 10.0;
  const double dt = 0.002;
  const double T = 0.4;
  const int n = static_cast<int>(T / dt);
  for (int k = 0; k < n; ++k) {
    model.step(Eigen::Vector4d::Zero(), dt, k * dt);
  }
  EXPECT_NEAR(model.state().v.z(), -p.gravity * T, 0.15);
  EXPECT_NEAR(model.state().p.z(), 10.0 - 0.5 * p.gravity * T * T, 0.25);
}

TEST(Model, QuaternionStaysNormalized)
{
  DynamicsParams p;
  QuadrotorModel model(p);
  model.state().p.z() = 1.0;
  Eigen::Vector4d rpm = Eigen::Vector4d::Constant(2800.0);
  for (int k = 0; k < 2000; ++k) {
    // Slight yaw torque via unequal RPM
    rpm(0) = 2900.0;
    rpm(1) = 2700.0;
    rpm(2) = 2900.0;
    rpm(3) = 2700.0;
    model.step(rpm, 0.002, k * 0.002);
    EXPECT_NEAR(model.state().q.norm(), 1.0, 1e-6);
  }
}

TEST(Model, ImuHoverSpecificForce)
{
  DynamicsParams p;
  p.imu_noise_enable = false;
  QuadrotorModel model(p);
  AllocationMatrix A(p.arm_length, p.k_F, p.k_M);
  const double T_hover = p.mass * p.gravity;
  Eigen::Vector4d wsq = A.wrenchToOmegaSq(Eigen::Vector4d(T_hover, 0, 0, 0));
  Eigen::Vector4d rpm;
  for (int i = 0; i < 4; ++i) {
    rpm(i) = QuadrotorModel::radToRpm(std::sqrt(wsq(i)));
    model.state().motor_omega(i) = QuadrotorModel::rpmToRad(rpm(i));
  }
  model.state().p.z() = 1.5;
  for (int k = 0; k < 200; ++k) {
    model.step(rpm, 0.002, k * 0.002);
  }
  Eigen::Vector3d accel, gyro;
  model.sampleImu(accel, gyro, 0.002);
  EXPECT_NEAR(accel.x(), 0.0, 0.2);
  EXPECT_NEAR(accel.y(), 0.0, 0.2);
  EXPECT_NEAR(accel.z(), p.gravity, 0.35);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

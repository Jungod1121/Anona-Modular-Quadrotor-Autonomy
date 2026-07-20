#include "drone_controller/cascade_pid.hpp"

#include <gtest/gtest.h>
#include <cmath>

using drone_controller::CascadePid;
using drone_controller::ControllerParams;
using drone_controller::GoalState;
using drone_controller::Mixer;
using drone_controller::VehicleState;

namespace {
constexpr double kF = 3.0e-5;
constexpr double kM = 5.0e-7;
constexpr double kL = 0.18;
}  // namespace

TEST(Mixer, InverseRoundTrip)
{
  Mixer mixer(kL, kF, kM);
  Eigen::Vector4d wrench(4.0 * kF * 1e6, 0.01, -0.02, 0.005);
  const Eigen::Vector4d omega_sq = mixer.wrenchToOmegaSq(wrench);
  const Eigen::Vector4d back = mixer.omegaSqToWrench(omega_sq);
  EXPECT_NEAR((wrench - back).norm(), 0.0, 1e-8);
}

TEST(Mixer, EqualRpmGivesZeroTorque)
{
  Mixer mixer(kL, kF, kM);
  const Eigen::Vector4d omega_sq = Eigen::Vector4d::Constant(1e6);
  const Eigen::Vector4d wrench = mixer.omegaSqToWrench(omega_sq);
  EXPECT_NEAR(wrench(1), 0.0, 1e-9);
  EXPECT_NEAR(wrench(2), 0.0, 1e-9);
  EXPECT_NEAR(wrench(3), 0.0, 1e-9);
  EXPECT_GT(wrench(0), 0.0);
}

TEST(Mixer, HoverThrustPositiveRpm)
{
  ControllerParams p;
  Mixer mixer(p.arm_length, p.k_F, p.k_M);
  const double T_hover = p.mass * p.gravity;
  const Eigen::Vector4d wrench(T_hover, 0.0, 0.0, 0.0);
  const Eigen::Vector4d omega_sq = mixer.wrenchToOmegaSq(wrench);
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(omega_sq(i), 0.0);
    const double rpm = Mixer::radToRpm(std::sqrt(omega_sq(i)));
    EXPECT_GT(rpm, 0.0);
    EXPECT_LT(rpm, p.rpm_max);
  }
}

TEST(Mixer, AllocationSignsMatchPlantConvention)
{
  Mixer mixer(kL, kF, kM);
  Eigen::Vector4d wsq = Eigen::Vector4d::Zero();
  wsq(0) = 1e6;
  const Eigen::Vector4d w0 = mixer.omegaSqToWrench(wsq);
  EXPECT_GT(w0(1), 0.0);
  EXPECT_LT(w0(2), 0.0);
  EXPECT_GT(w0(3), 0.0);
}

TEST(CascadePid, HoverProducesPositiveRpm)
{
  ControllerParams p;
  CascadePid controller(p);

  VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  state.valid = true;

  GoalState goal;
  goal.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  goal.yaw = 0.0;
  goal.valid = true;

  const Eigen::Vector4d rpm = controller.compute(state, goal, 0.01);
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(rpm(i), 0.0);
  }
  const double hover_rpm = Mixer::radToRpm(std::sqrt(p.mass * p.gravity / (4.0 * p.k_F)));
  EXPECT_NEAR(rpm.mean(), hover_rpm, 0.25 * hover_rpm);
}

TEST(CascadePid, TiltSaturation)
{
  ControllerParams p;
  p.max_tilt = 0.35;
  CascadePid controller(p);

  VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  state.valid = true;

  GoalState goal;
  goal.position = Eigen::Vector3d(50.0, 0.0, 1.5);
  goal.valid = true;

  Eigen::Vector4d rpm;
  for (int i = 0; i < 50; ++i) {
    rpm = controller.compute(state, goal, 0.01);
  }
  EXPECT_GT(rpm.minCoeff(), 0.0);
  EXPECT_TRUE(rpm.allFinite());
  EXPECT_GT((rpm.maxCoeff() - rpm.minCoeff()), 1.0);
}

TEST(CascadePid, RpmSlewLimitsStep)
{
  ControllerParams p;
  p.max_motor_rpm_rate = 5000.0;
  CascadePid controller(p);

  VehicleState state;
  state.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  state.valid = true;
  GoalState goal;
  goal.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  goal.valid = true;

  const Eigen::Vector4d r0 = controller.compute(state, goal, 0.01);
  goal.position.z() = 5.0;
  const Eigen::Vector4d r1 = controller.compute(state, goal, 0.01);
  const double max_delta = (r1 - r0).cwiseAbs().maxCoeff();
  EXPECT_LE(max_delta, p.max_motor_rpm_rate * 0.01 + 1e-3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

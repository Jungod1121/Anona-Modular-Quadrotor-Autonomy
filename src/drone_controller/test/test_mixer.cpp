#include "drone_controller/cascade_pid.hpp"

#include <gtest/gtest.h>
#include <cmath>

using drone_controller::CascadePid;
using drone_controller::ControllerParams;
using drone_controller::GoalState;
using drone_controller::Mixer;
using drone_controller::VehicleState;

TEST(Mixer, InverseRoundTrip)
{
  Mixer mixer(0.18, 1.5e-6, 2.5e-8);
  Eigen::Vector4d wrench(4.0 * 1.5e-6 * 1e6, 0.01, -0.02, 0.005);
  const Eigen::Vector4d omega_sq = mixer.wrenchToOmegaSq(wrench);
  const Eigen::Vector4d back = mixer.omegaSqToWrench(omega_sq);
  EXPECT_NEAR((wrench - back).norm(), 0.0, 1e-8);
}

TEST(Mixer, EqualRpmGivesZeroTorque)
{
  Mixer mixer(0.18, 1.5e-6, 2.5e-8);
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
  }
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
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

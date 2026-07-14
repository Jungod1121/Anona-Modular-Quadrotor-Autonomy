#include "drone_dynamics/quadrotor_model.hpp"
#include <gtest/gtest.h>
#include <cmath>

using drone_dynamics::AllocationMatrix;
using drone_dynamics::DynamicsParams;
using drone_dynamics::QuadrotorModel;

TEST(Allocation, EqualRpmGivesZeroTorque)
{
  AllocationMatrix A(0.18, 1.5e-6, 2.5e-8);
  Eigen::Vector4d wsq = Eigen::Vector4d::Constant(1e6);  // equal ω²
  Eigen::Vector4d wrench = A.omegaSqToWrench(wsq);
  EXPECT_NEAR(wrench(1), 0.0, 1e-9);  // τx
  EXPECT_NEAR(wrench(2), 0.0, 1e-9);  // τy
  EXPECT_NEAR(wrench(3), 0.0, 1e-9);  // τz
  EXPECT_GT(wrench(0), 0.0);          // thrust
}

TEST(Allocation, InverseRoundTrip)
{
  AllocationMatrix A(0.18, 1.5e-6, 2.5e-8);
  Eigen::Vector4d wrench(4.0 * 1.5e-6 * 1e6, 0.01, -0.02, 0.005);
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
  }
  // Warm-start motors at hover
  for (int i = 0; i < 4; ++i) {
    model.state().motor_omega(i) = QuadrotorModel::rpmToRad(rpm(i));
  }
  model.state().p.z() = 1.5;
  for (int k = 0; k < 500; ++k) {
    model.step(rpm, 0.002, k * 0.002);
  }
  EXPECT_NEAR(model.state().p.z(), 1.5, 0.15);
  EXPECT_LT(model.state().v.norm(), 0.3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

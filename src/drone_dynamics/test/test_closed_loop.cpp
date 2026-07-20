#include "drone_dynamics/quadrotor_model.hpp"
#include "drone_controller/cascade_pid.hpp"

#include <gtest/gtest.h>

/**
 * Offline closed-loop settle (no ROS). Links cascade_pid for plant↔controller
 * provenance checks inspired by pengyu_sim offline eval — independent code.
 */
TEST(ClosedLoop, HoverSettleOffline)
{
  drone_dynamics::DynamicsParams dp;
  drone_dynamics::QuadrotorModel plant(dp);
  plant.state().p.z() = 0.15;

  drone_controller::ControllerParams cp;
  drone_controller::CascadePid ctrl(cp);

  drone_controller::GoalState goal;
  goal.position = Eigen::Vector3d(0.0, 0.0, 1.5);
  goal.valid = true;

  const double dt = 0.01;
  Eigen::Vector4d rpm = Eigen::Vector4d::Zero();
  for (int k = 0; k < 800; ++k) {
    drone_controller::VehicleState st;
    st.position = plant.state().p;
    st.velocity = plant.state().v;
    st.attitude = plant.state().q;
    st.omega = plant.state().omega;
    st.valid = true;
    rpm = ctrl.compute(st, goal, dt);
    for (int s = 0; s < 5; ++s) {
      plant.step(rpm, 0.002, (k * 5 + s) * 0.002);
    }
  }
  EXPECT_NEAR(plant.state().p.z(), 1.5, 0.25);
  EXPECT_LT(plant.state().p.head<2>().norm(), 0.35);
  EXPECT_LT(plant.state().v.norm(), 0.4);
}

TEST(ClosedLoop, PlantControllerAllocationMatch)
{
  drone_dynamics::AllocationMatrix plant(0.18, 3.0e-5, 5.0e-7);
  drone_controller::Mixer mixer(0.18, 3.0e-5, 5.0e-7);
  EXPECT_NEAR((plant.A() - mixer.A()).norm(), 0.0, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

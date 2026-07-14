#pragma once
/**
 * Voxel ray traversal (Amanatides & Woo 1987).
 * Adapted from ego-planner plan_env/raycast.{h,cpp} — standalone, no ROS.
 */
#include <Eigen/Dense>
#include <vector>

namespace drone_planner
{

class RayCaster
{
public:
  bool setInput(const Eigen::Vector3d & start, const Eigen::Vector3d & end);
  bool step(Eigen::Vector3d & ray_pt);

private:
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  int x_{0}, y_{0}, z_{0};
  int end_x_{0}, end_y_{0}, end_z_{0};
  double max_dist_sq_{0.0};
  double dx_{0}, dy_{0}, dz_{0};
  int step_x_{0}, step_y_{0}, step_z_{0};
  double t_max_x_{0}, t_max_y_{0}, t_max_z_{0};
  double t_delta_x_{0}, t_delta_y_{0}, t_delta_z_{0};
};

void raycastVoxels(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end,
  std::vector<Eigen::Vector3d> & output);

}  // namespace drone_planner

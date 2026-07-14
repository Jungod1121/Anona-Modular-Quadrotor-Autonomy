#include "drone_planner/raycast.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_planner
{
namespace
{

int signumInt(int x)
{
  return x == 0 ? 0 : (x < 0 ? -1 : 1);
}

double mod1(double value)
{
  return std::fmod(std::fmod(value, 1.0) + 1.0, 1.0);
}

double intbound(double s, double ds)
{
  if (ds < 0.0) {
    return intbound(-s, -ds);
  }
  s = mod1(s);
  return (1.0 - s) / ds;
}

}  // namespace

void raycastVoxels(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end,
  std::vector<Eigen::Vector3d> & output)
{
  output.clear();
  RayCaster caster;
  if (!caster.setInput(start, end)) {
    return;
  }
  Eigen::Vector3d pt;
  while (caster.step(pt)) {
    output.push_back(pt);
    if (output.size() > 2000) {
      throw std::out_of_range("raycastVoxels: too many voxels");
    }
  }
}

bool RayCaster::setInput(const Eigen::Vector3d & start, const Eigen::Vector3d & end)
{
  start_ = start;
  end_ = end;

  x_ = static_cast<int>(std::floor(start_.x()));
  y_ = static_cast<int>(std::floor(start_.y()));
  z_ = static_cast<int>(std::floor(start_.z()));
  end_x_ = static_cast<int>(std::floor(end_.x()));
  end_y_ = static_cast<int>(std::floor(end_.y()));
  end_z_ = static_cast<int>(std::floor(end_.z()));

  const Eigen::Vector3d direction = end_ - start_;
  max_dist_sq_ = direction.squaredNorm();

  dx_ = static_cast<double>(end_x_ - x_);
  dy_ = static_cast<double>(end_y_ - y_);
  dz_ = static_cast<double>(end_z_ - z_);

  step_x_ = signumInt(static_cast<int>(dx_));
  step_y_ = signumInt(static_cast<int>(dy_));
  step_z_ = signumInt(static_cast<int>(dz_));

  t_max_x_ = intbound(start_.x(), dx_);
  t_max_y_ = intbound(start_.y(), dy_);
  t_max_z_ = intbound(start_.z(), dz_);

  t_delta_x_ = step_x_ == 0 ? 0.0 : static_cast<double>(step_x_) / dx_;
  t_delta_y_ = step_y_ == 0 ? 0.0 : static_cast<double>(step_y_) / dy_;
  t_delta_z_ = step_z_ == 0 ? 0.0 : static_cast<double>(step_z_) / dz_;

  if (step_x_ == 0 && step_y_ == 0 && step_z_ == 0) {
    return false;
  }
  return true;
}

bool RayCaster::step(Eigen::Vector3d & ray_pt)
{
  ray_pt = Eigen::Vector3d(static_cast<double>(x_), static_cast<double>(y_), static_cast<double>(z_));

  if (x_ == end_x_ && y_ == end_y_ && z_ == end_z_) {
    return false;
  }

  const double dist_sq =
    (ray_pt - start_).squaredNorm();
  if (dist_sq > max_dist_sq_) {
    return false;
  }

  if (t_max_x_ < t_max_y_) {
    if (t_max_x_ < t_max_z_) {
      x_ += step_x_;
      t_max_x_ += t_delta_x_;
    } else {
      z_ += step_z_;
      t_max_z_ += t_delta_z_;
    }
  } else {
    if (t_max_y_ < t_max_z_) {
      y_ += step_y_;
      t_max_y_ += t_delta_y_;
    } else {
      z_ += step_z_;
      t_max_z_ += t_delta_z_;
    }
  }
  return true;
}

}  // namespace drone_planner

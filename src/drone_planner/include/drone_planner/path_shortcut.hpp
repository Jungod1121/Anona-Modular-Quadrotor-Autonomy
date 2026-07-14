#pragma once
#include "drone_planner/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_planner
{

inline bool segmentCollisionFree(
  const OccupancyGrid & grid,
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b)
{
  const double len = (b - a).norm();
  if (len < 1e-6) {
    return !grid.isOccupied(a);
  }
  const int steps = std::max(2, static_cast<int>(std::ceil(len / grid.resolution())));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    if (grid.isOccupied(a + t * (b - a))) {
      return false;
    }
  }
  return true;
}

/** String-pull / shortcut on grid — removes A* zigzag before B-spline. */
inline std::vector<Eigen::Vector3d> shortcutPath(
  const OccupancyGrid & grid,
  const std::vector<Eigen::Vector3d> & path)
{
  if (path.size() < 2) {
    return path;
  }
  std::vector<Eigen::Vector3d> out;
  out.push_back(path.front());
  size_t i = 0;
  while (i + 1 < path.size()) {
    size_t best = i + 1;
    for (size_t j = path.size() - 1; j > i + 1; --j) {
      if (segmentCollisionFree(grid, path[i], path[j])) {
        best = j;
        break;
      }
    }
    out.push_back(path[best]);
    i = best;
  }
  return out;
}

/** Densify polyline for stable lookahead tracking (pre-P2 style). */
inline std::vector<Eigen::Vector3d> densifyPath(
  const std::vector<Eigen::Vector3d> & path, double spacing = 0.25)
{
  if (path.size() < 2) {
    return path;
  }
  spacing = std::max(0.1, spacing);
  std::vector<Eigen::Vector3d> out;
  out.push_back(path.front());
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d a = path[i];
    const Eigen::Vector3d b = path[i + 1];
    const double len = (b - a).norm();
    if (len < 1e-6) {
      continue;
    }
    const int n = std::max(1, static_cast<int>(std::ceil(len / spacing)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      out.push_back(a + t * (b - a));
    }
  }
  return out;
}

}  // namespace drone_planner

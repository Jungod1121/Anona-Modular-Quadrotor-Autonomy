#include "drone_planner/path_shortcut.hpp"

#include <gtest/gtest.h>

TEST(PathShortcut, ReducesZigzag)
{
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.5, Eigen::Vector3d(-2, -2, 0), Eigen::Vector3d(24, 14, 3));
  for (double x = 2.0; x <= 16.0; x += 0.2) {
    grid.addPoint(Eigen::Vector3d(x, 3.5, 1.5));
    grid.addPoint(Eigen::Vector3d(x, 6.5, 1.5));
  }
  grid.sealBoundary(2);
  grid.rebuildInflateLayer();

  std::vector<Eigen::Vector3d> zigzag;
  for (int i = 0; i <= 20; ++i) {
    zigzag.emplace_back(static_cast<double>(i) * 0.5, 5.0 + ((i % 2) ? 0.2 : -0.2), 1.5);
  }

  const auto short_path = drone_planner::shortcutPath(grid, zigzag);
  EXPECT_LT(short_path.size(), zigzag.size());
  for (size_t i = 0; i + 1 < short_path.size(); ++i) {
    EXPECT_TRUE(drone_planner::segmentCollisionFree(grid, short_path[i], short_path[i + 1]));
  }
}

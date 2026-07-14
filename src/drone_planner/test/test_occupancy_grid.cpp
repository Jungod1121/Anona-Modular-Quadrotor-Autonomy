#include "drone_planner/occupancy_grid.hpp"

#include <gtest/gtest.h>

TEST(OccupancyGrid, LayeredInflate)
{
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.4, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(4, 4, 2));
  grid.addPoint(Eigen::Vector3d(2.0, 2.0, 1.0));
  grid.rebuildInflateLayer();

  EXPECT_FALSE(grid.isOccupiedRaw(Eigen::Vector3d(2.7, 2.0, 1.0)));
  EXPECT_TRUE(grid.isOccupied(Eigen::Vector3d(2.7, 2.0, 1.0)));
}

TEST(OccupancyGrid, RaycastClearsFreeVoxels)
{
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.2, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(6, 6, 2));
  grid.addPoint(Eigen::Vector3d(4.0, 2.0, 1.0));
  grid.rebuildInflateLayer();
  EXPECT_TRUE(grid.isOccupied(Eigen::Vector3d(3.9, 2.0, 1.0)));

  grid.integrateRayHit(Eigen::Vector3d(0.5, 2.0, 1.0), Eigen::Vector3d(4.0, 2.0, 1.0));
  EXPECT_FALSE(grid.isOccupiedRaw(Eigen::Vector3d(1.5, 2.0, 1.0)));
  EXPECT_TRUE(grid.isOccupiedRaw(Eigen::Vector3d(4.0, 2.0, 1.0)));
}

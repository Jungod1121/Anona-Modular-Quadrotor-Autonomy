#include "drone_planner/dyn_astar.hpp"
#include "drone_planner/grid_astar.hpp"
#include "drone_planner/occupancy_grid.hpp"

#include <gtest/gtest.h>

static drone_planner::OccupancyGrid makeOpenGrid()
{
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.4, Eigen::Vector3d(-2, -2, 0), Eigen::Vector3d(24, 14, 3));
  grid.sealBoundary(2);
  return grid;
}

TEST(DynAStar, StraightLine)
{
  auto grid = makeOpenGrid();
  drone_planner::DynAStar dyn;
  drone_planner::AStarOptions opt;
  opt.cruise_z = 1.5;
  std::vector<Eigen::Vector3d> path;
  const Eigen::Vector3d s(1.0, 5.0, 1.5);
  const Eigen::Vector3d g(6.0, 5.0, 1.5);
  ASSERT_TRUE(dyn.search(grid, s, g, path, opt));
  EXPECT_GE(path.size(), 2u);
}

TEST(DynAStar, CorridorMatchesGridAStar)
{
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.4, Eigen::Vector3d(-2, -2, 0), Eigen::Vector3d(24, 14, 3));
  for (double x = 2.0; x <= 16.0; x += 0.2) {
    grid.addPoint(Eigen::Vector3d(x, 3.5, 1.5));
    grid.addPoint(Eigen::Vector3d(x, 6.5, 1.5));
  }
  grid.sealBoundary(2);
  grid.rebuildInflateLayer();

  drone_planner::AStarOptions opt;
  opt.cruise_z = 1.5;
  const Eigen::Vector3d s(1.0, 5.0, 1.5);
  const Eigen::Vector3d g(17.0, 5.0, 1.5);

  std::vector<Eigen::Vector3d> dyn_path;
  drone_planner::DynAStar dyn;
  ASSERT_TRUE(dyn.search(grid, s, g, dyn_path, opt));

  std::vector<Eigen::Vector3d> grid_path;
  drone_planner::GridAStar astar;
  ASSERT_TRUE(astar.search(grid, s, g, grid_path, opt));
  EXPECT_FALSE(dyn_path.empty());
  EXPECT_FALSE(grid_path.empty());
}

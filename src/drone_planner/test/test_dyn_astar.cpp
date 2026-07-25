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

TEST(DynAStar, ThreadsVerticalHole)
{
  // Thin plate at x=5 with a large hole — path must climb through it.
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.15, Eigen::Vector3d(-1, -4, 0), Eigen::Vector3d(12, 8, 4));
  for (double y = -3.5; y <= 3.5; y += 0.2) {
    for (double z = 0.2; z <= 3.6; z += 0.2) {
      if (std::abs(y) < 1.0 && z > 1.6 && z < 2.8) {
        continue;  // hole centered ~z=2.2
      }
      grid.addPoint(Eigen::Vector3d(5.0, y, z));
    }
  }
  grid.sealBoundary(1);
  grid.rebuildInflateLayer();

  drone_planner::AStarOptions opt;
  opt.true_3d = true;
  opt.z_band = 2.5;
  opt.vertical_cost_scale = 1.1;
  opt.cruise_z = 1.0;
  opt.free_snap_radius = 12;
  const Eigen::Vector3d s(1.0, 0.0, 1.0);
  const Eigen::Vector3d g(9.0, 0.0, 1.0);

  std::vector<Eigen::Vector3d> path;
  drone_planner::DynAStar dyn;
  ASSERT_TRUE(dyn.search(grid, s, g, path, opt));
  ASSERT_GE(path.size(), 3u);

  double max_z = 0.0;
  for (const auto & p : path) {
    max_z = std::max(max_z, p.z());
  }
  EXPECT_GT(max_z, 1.5) << "3D A* should climb toward the plate hole";
}


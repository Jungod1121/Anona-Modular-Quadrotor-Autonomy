#include "drone_planner/bspline_optimizer.hpp"
#include "drone_planner/occupancy_grid.hpp"
#include "drone_planner/grid_astar.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(BsplineOptimizer, ReboundLbfgsOpenSpace)
{
  drone_planner::BsplineOptimizer opt;
  opt.setWeights({});
  std::vector<Eigen::Vector3d> guide{{0, 0, 1.5}, {2, 0, 1.5}, {4, 0, 1.5}};
  std::vector<Eigen::Vector3d> traj;
  Eigen::MatrixXd ctrl;
  ASSERT_TRUE(opt.optimize(guide, Eigen::Vector3d::Zero(), traj, ctrl));
  EXPECT_GE(traj.size(), 2u);
}

TEST(BsplineOptimizer, ReboundLbfgsWithClearance)
{
  drone_planner::BsplineOptimizer opt;
  drone_planner::OptWeights w;
  w.dist_threshold = 0.4;
  w.lambda_collision = 15.0;
  opt.setWeights(w);

  std::vector<Eigen::Vector3d> obstacles;
  for (double x = 1.0; x <= 3.0; x += 0.15) {
    for (double y = -0.2; y <= 0.25; y += 0.15) {
      obstacles.emplace_back(x, y, 1.5);
    }
  }
  opt.setObstacles(obstacles);

  std::vector<Eigen::Vector3d> guide;
  for (int i = 0; i <= 8; ++i) {
    guide.emplace_back(static_cast<double>(i) * 0.5, 0.6, 1.5);
  }

  std::vector<Eigen::Vector3d> traj;
  Eigen::MatrixXd ctrl;
  ASSERT_TRUE(opt.optimize(guide, Eigen::Vector3d::Zero(), traj, ctrl));
  EXPECT_GT(opt.minDistanceToObstacles(traj), 0.30);
}

TEST(BsplineOptimizer, ReboundLbfgsFarObstacles)
{
  drone_planner::BsplineOptimizer opt;
  drone_planner::OptWeights w;
  w.dist_threshold = 0.45;
  opt.setWeights(w);
  opt.setObstacles({{5.0, 5.0, 1.5}});
  std::vector<Eigen::Vector3d> guide{{0, 0, 1.5}, {2, 0, 1.5}, {4, 0, 1.5}};
  std::vector<Eigen::Vector3d> traj;
  Eigen::MatrixXd ctrl;
  ASSERT_TRUE(opt.optimize(guide, Eigen::Vector3d::Zero(), traj, ctrl));
}

static drone_planner::OccupancyGrid buildCorridorGrid()
{
  drone_planner::OccupancyGrid grid;
  grid.setParams(0.25, 0.4, Eigen::Vector3d(-2, -2, 0), Eigen::Vector3d(24, 14, 3));
  for (double x = 2.0; x <= 16.0; x += 0.2) {
    for (double dy = -0.15; dy <= 0.15; dy += 0.08) {
      grid.addPoint(Eigen::Vector3d(x, 3.5 + dy, 1.5));
      grid.addPoint(Eigen::Vector3d(x, 6.5 + dy, 1.5));
    }
  }
  grid.sealBoundary(2);
  return grid;
}

TEST(BsplineOptimizer, DenseCorridorOnInflatedGrid)
{
  auto grid = buildCorridorGrid();
  drone_planner::AStarOptions astar_opt;
  astar_opt.cruise_z = 1.5;

  const Eigen::Vector3d start(1.0, 5.0, 1.5);
  const Eigen::Vector3d goal(17.0, 5.0, 1.5);
  std::vector<Eigen::Vector3d> guide;
  drone_planner::GridAStar astar;
  ASSERT_TRUE(astar.search(grid, start, goal, guide, astar_opt));

  drone_planner::BsplineOptimizer opt;
  drone_planner::OptWeights w;
  w.dist_threshold = std::max(0.28, 0.4 * 0.80);
  w.lambda_collision = 12.0;
  w.ts = 0.25;
  opt.setWeights(w);
  opt.setOccupancyGrid(&grid);

  std::vector<Eigen::Vector3d> traj;
  Eigen::MatrixXd ctrl;
  ASSERT_TRUE(opt.optimize(guide, Eigen::Vector3d::Zero(), traj, ctrl));
  EXPECT_GT(traj.size(), guide.size() / 4u);
  EXPECT_GT(opt.minDistanceToObstacles(traj), w.dist_threshold * 0.75);
}

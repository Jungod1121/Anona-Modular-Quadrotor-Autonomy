#include "drone_map/ego_map.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone_map::ego::MazeConfig;
using drone_map::ego::generateMaze2D;

namespace
{

MazeConfig makeConfig(double x_len, double y_len)
{
  MazeConfig cfg;
  cfg.seed = 7;
  cfg.x_length = x_len;
  cfg.y_length = y_len;
  cfg.z_length = 2.0;
  cfg.resolution = 0.1;
  cfg.road_width = 0.6;
  cfg.add_wall_x = true;
  cfg.add_wall_y = true;
  cfg.origin_x = x_len / 2.0;
  cfg.origin_y = y_len / 2.0;
  cfg.start_x = 0.5 * x_len;
  cfg.start_y = 0.5 * y_len;
  cfg.goal_x = x_len - 1.0;
  cfg.goal_y = y_len / 2.0;
  cfg.clearance_radius = 1.2;
  return cfg;
}

/// Regression: recursiveDivision indexed rows/cols inconsistently and only
/// stayed in-bounds because mazes happened to be square (20x20).
TEST(NonSquareMaze, WideMazeGeneratesInBounds)
{
  const auto cloud = generateMaze2D(makeConfig(24.0, 12.0));
  ASSERT_FALSE(cloud.points.empty());
}

TEST(NonSquareMaze, TallMazeGeneratesInBounds)
{
  const auto cloud = generateMaze2D(makeConfig(10.0, 22.0));
  ASSERT_FALSE(cloud.points.empty());
}

TEST(NonSquareMaze, CloudStaysWithinDeclaredEnvelope)
{
  const double x_len = 18.0;
  const double y_len = 9.0;
  const auto cloud = generateMaze2D(makeConfig(x_len, y_len));
  for (const auto & p : cloud.points) {
    EXPECT_GE(p.x, -0.5);
    EXPECT_LE(p.x, x_len + 0.5);
    EXPECT_GE(p.y, -0.5);
    EXPECT_LE(p.y, y_len + 0.5);
    EXPECT_GE(p.z, 0.0);
  }
}

}  // namespace

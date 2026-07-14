#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace drone_map
{
namespace ego
{

/// Parameters for EGO-Planner mockamap maze2D (type=3).
/// Reference: ego-planner-swarm/src/uav_simulator/mockamap/src/maps.cpp
struct MazeConfig
{
  int seed{510};
  double x_length{24.0};
  double y_length{14.0};
  double z_length{2.5};
  double resolution{0.1};
  double road_width{0.6};
  int add_wall_x{0};
  int add_wall_y{0};
  /// Map frame origin (center of field).
  double origin_x{10.0};
  double origin_y{5.0};
  double start_x{1.0};
  double start_y{5.0};
  double goal_x{17.0};
  double goal_y{5.0};
  double clearance_radius{1.2};
};

/// Parameters for EGO-Planner map_generator random_forest.
/// Reference: ego-planner-swarm/.../map_generator/src/random_forest_sensing.cpp
struct ForestConfig
{
  int seed{42};
  double x_length{24.0};
  double y_length{14.0};
  double z_length{2.5};
  double resolution{0.1};
  int obs_num{100};
  int circle_num{40};
  double min_distance{0.8};
  double lower_rad{0.3};
  double upper_rad{0.6};
  double lower_hei{2.0};
  double upper_hei{2.5};
  double origin_x{10.0};
  double origin_y{5.0};
  double start_x{1.0};
  double start_y{5.0};
  double goal_x{17.0};
  double goal_y{5.0};
  double clearance_radius{1.2};
};

pcl::PointCloud<pcl::PointXYZ> generateMaze2D(const MazeConfig & cfg);
pcl::PointCloud<pcl::PointXYZ> generateRandomForest(const ForestConfig & cfg);

}  // namespace ego
}  // namespace drone_map

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace drone_map
{

enum class MapMode
{
  SPARSE,
  DENSE_FIELD,
  NARROW_CORRIDOR,
  /// EGO-Planner mockamap maze2D (optional alternative to NARROW_CORRIDOR).
  EGO_MAZE2D,
  /// EGO-Planner random_forest (optional alternative to DENSE_FIELD).
  EGO_DENSE_FOREST
};

MapMode parseMapMode(const std::string & mode_str);

struct Obstacle
{
  enum class Shape
  {
    CYLINDER,
    SPHERE,
    WALL
  };

  Shape shape{Shape::CYLINDER};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.2};
  double height{2.0};
  /// Wall span (m); axis set by length_along_y.
  double length{1.0};
  /// Wall thickness (m); axis set by length_along_y.
  double thickness{0.3};
  /// If true: length along Y, thickness along X (gate walls); else length along X.
  bool length_along_y{false};
  /// Perimeter enclosure walls (not inner gate walls).
  bool is_boundary{false};
};

struct FieldBounds
{
  double x_min{0.0};
  double x_max{8.0};
  double y_min{0.0};
  double y_max{8.0};
  double z_min{0.0};
  double z_max{2.5};
};

struct MapConfig
{
  MapMode mode{MapMode::SPARSE};
  int seed{42};
  int max_attempts{200};

  double start_x{0.0};
  double start_y{0.0};
  double start_z{1.5};
  double goal_x{2.0};
  double goal_y{0.0};
  double goal_z{1.5};

  double safety_distance{0.4};
  double clearance_radius{1.2};
  double min_obstacle_spacing{0.9};
  double min_obstacle_radius{0.12};
  double max_obstacle_radius{0.3};

  /// Surface sampling resolution for obstacle point clouds (m).
  double point_resolution{0.08};
  /// VoxelGrid leaf size; <= 0 disables downsampling.
  double downsample_voxel{0.0};
  /// Occupancy grid resolution for connectivity BFS (m).
  double grid_resolution{0.2};

  /// EGO mockamap maze2D: corridor width (m).
  double ego_road_width{0.6};
  /// EGO map voxel size (m).
  double ego_resolution{0.1};
  int ego_obs_num{100};
  int ego_circle_num{40};
  double ego_min_distance{0.8};

  /// narrow_corridor gate opening width (m); PLAN §5.3 clamps to [1.2, 1.8].
  double corridor_gap_width{1.5};
  /// Number of N–S 通道墙 / staggered doors (3, 5, or 7).
  int corridor_gate_count{5};

  /// When true, enclose the field with perimeter WALL obstacles.
  bool add_boundary_walls{true};
  /// Optional AABB override (used when x_max > x_min).
  bool use_custom_bounds{false};
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
  double z_min{0.0};
  double z_max{0.0};
};

struct MapGenerationResult
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<Obstacle> obstacles;
  FieldBounds bounds;
  int seed{0};
  int attempt{0};
  bool connected{false};
};

class MapGenerator
{
public:
  explicit MapGenerator(MapConfig config);

  MapGenerationResult generate();

  static FieldBounds boundsForMode(MapMode mode);

private:
  MapConfig config_;

  std::vector<Obstacle> generateCandidateObstacles(
    const FieldBounds & bounds, int obstacle_count, int effective_seed) const;

  void addNarrowPassageGate(
    std::vector<Obstacle> & obstacles, const FieldBounds & bounds) const;

  /// S-bend waypoints used for gate centers + cylinder keep-out (绕行).
  std::vector<Eigen::Vector2d> narrowPassageWaypoints() const;

  void addBoundaryWalls(
    std::vector<Obstacle> & obstacles, const FieldBounds & bounds) const;

  bool checkConnectivity(
    const std::vector<Obstacle> & obstacles,
    const FieldBounds & bounds) const;

  bool isCellOccupied(
    double x, double y, double z,
    const std::vector<Obstacle> & obstacles) const;

  pcl::PointCloud<pcl::PointXYZ> sampleObstacleSurface(
    const Obstacle & obstacle) const;

  pcl::PointCloud<pcl::PointXYZ> buildCloud(
    const std::vector<Obstacle> & obstacles) const;

  pcl::PointCloud<pcl::PointXYZ> downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud) const;

  int obstacleCountForMode(MapMode mode) const;
  int effectiveSeed(int attempt) const;
};

}  // namespace drone_map

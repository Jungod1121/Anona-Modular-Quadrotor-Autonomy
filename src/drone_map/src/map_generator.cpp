#include "drone_map/ego_map.hpp"
#include "drone_map/map_generator.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <random>
#include <stdexcept>

namespace drone_map
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

int gridIndex(double value, double origin, double resolution)
{
  return static_cast<int>(std::floor((value - origin) / resolution));
}

double clampValue(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

}  // namespace

MapMode parseMapMode(const std::string & mode_str)
{
  if (mode_str == "sparse") {
    return MapMode::SPARSE;
  }
  if (mode_str == "dense_field") {
    return MapMode::DENSE_FIELD;
  }
  if (mode_str == "narrow_corridor") {
    return MapMode::NARROW_CORRIDOR;
  }
  if (mode_str == "ego_maze2d") {
    return MapMode::EGO_MAZE2D;
  }
  if (mode_str == "ego_dense_forest") {
    return MapMode::EGO_DENSE_FOREST;
  }
  throw std::invalid_argument("Unknown map_mode: " + mode_str);
}

MapGenerator::MapGenerator(MapConfig config)
: config_(std::move(config))
{
  config_.corridor_gap_width = clampValue(config_.corridor_gap_width, 1.2, 1.8);
  config_.min_obstacle_spacing = clampValue(config_.min_obstacle_spacing, 0.8, 1.0);
  config_.min_obstacle_radius = clampValue(config_.min_obstacle_radius, 0.12, 0.3);
  config_.max_obstacle_radius = clampValue(config_.max_obstacle_radius, 0.12, 0.3);
  if (config_.max_obstacle_radius < config_.min_obstacle_radius) {
    std::swap(config_.min_obstacle_radius, config_.max_obstacle_radius);
  }
}

FieldBounds MapGenerator::boundsForMode(MapMode mode)
{
  FieldBounds b;
  switch (mode) {
    case MapMode::SPARSE:
      b.x_min = -4.0;
      b.x_max = 4.0;
      b.y_min = -4.0;
      b.y_max = 4.0;
      b.z_min = 0.0;
      b.z_max = 2.5;
      break;
    case MapMode::DENSE_FIELD:
    case MapMode::NARROW_CORRIDOR:
    case MapMode::EGO_MAZE2D:
    case MapMode::EGO_DENSE_FOREST:
      // Outer flyable envelope (boundary walls sit on these edges).
      b.x_min = -2.0;
      b.x_max = 22.0;
      b.y_min = -2.0;
      b.y_max = 12.0;
      b.z_min = 0.0;
      b.z_max = 2.5;
      break;
  }
  return b;
}

int MapGenerator::obstacleCountForMode(MapMode mode) const
{
  switch (mode) {
    case MapMode::SPARSE:
      return 2;
    case MapMode::DENSE_FIELD:
      return 80;
    case MapMode::NARROW_CORRIDOR:
      return 80;
    case MapMode::EGO_DENSE_FOREST:
      return 0;
    case MapMode::EGO_MAZE2D:
      return 0;
    default:
      return 0;
  }
}

int MapGenerator::effectiveSeed(int attempt) const
{
  // PLAN §5.3: keep base seed, vary attempt counter for reproducible retries.
  return config_.seed + attempt * 10007;
}

MapGenerationResult MapGenerator::generate()
{
  MapGenerationResult result;
  result.bounds = boundsForMode(config_.mode);
  result.seed = config_.seed;

  const double origin_x = 0.5 * (result.bounds.x_min + result.bounds.x_max);
  const double origin_y = 0.5 * (result.bounds.y_min + result.bounds.y_max);
  const double field_x = result.bounds.x_max - result.bounds.x_min;
  const double field_y = result.bounds.y_max - result.bounds.y_min;
  const double field_z = result.bounds.z_max - result.bounds.z_min;

  // Optional EGO-Planner maps (mockamap maze2D / random_forest).
  if (config_.mode == MapMode::EGO_MAZE2D) {
    ego::MazeConfig mc;
    mc.seed = config_.seed;
    // Match EGO mockamap maze2d.launch (20×20 m); centered on field via origin_*.
    mc.x_length = 20.0;
    mc.y_length = 20.0;
    mc.z_length = field_z;
    mc.resolution = config_.ego_resolution;
    mc.road_width = config_.ego_road_width;
    mc.origin_x = origin_x;
    mc.origin_y = origin_y;
    mc.start_x = config_.start_x;
    mc.start_y = config_.start_y;
    mc.goal_x = config_.goal_x;
    mc.goal_y = config_.goal_y;
    mc.clearance_radius = config_.clearance_radius;
    result.attempt = 0;
    result.connected = true;
    result.cloud = downsampleCloud(ego::generateMaze2D(mc));
    return result;
  }

  // EGO-Planner random_forest (optional mode ego_forest).
  if (config_.mode == MapMode::EGO_DENSE_FOREST) {
    for (int attempt = 0; attempt < config_.max_attempts; ++attempt) {
      ego::ForestConfig fc;
      fc.seed = effectiveSeed(attempt);
      fc.x_length = field_x;
      fc.y_length = field_y;
      fc.z_length = field_z;
      fc.resolution = config_.ego_resolution;
      fc.obs_num = config_.ego_obs_num;
      fc.circle_num = config_.ego_circle_num;
      fc.min_distance = config_.ego_min_distance;
      fc.lower_rad = config_.min_obstacle_radius;
      fc.upper_rad = config_.max_obstacle_radius;
      fc.lower_hei = 1.5;
      fc.upper_hei = field_z;
      fc.origin_x = origin_x;
      fc.origin_y = origin_y;
      fc.start_x = config_.start_x;
      fc.start_y = config_.start_y;
      fc.goal_x = config_.goal_x;
      fc.goal_y = config_.goal_y;
      fc.clearance_radius = config_.clearance_radius;
      result.attempt = attempt;
      result.connected = true;
      result.cloud = downsampleCloud(ego::generateRandomForest(fc));
      return result;
    }
  }

  int target_count = obstacleCountForMode(config_.mode);
  if (config_.mode == MapMode::SPARSE) {
    target_count = effectiveSeed(0) % 3;
  }

  const bool needs_connectivity =
    config_.mode == MapMode::DENSE_FIELD ||
    config_.mode == MapMode::NARROW_CORRIDOR;

  for (int attempt = 0; attempt < config_.max_attempts; ++attempt) {
    auto obstacles = generateCandidateObstacles(
      result.bounds, target_count, effectiveSeed(attempt));

    if (config_.mode == MapMode::NARROW_CORRIDOR) {
      addNarrowPassageGate(obstacles, result.bounds);
    }

    const bool connected = !needs_connectivity ||
      checkConnectivity(obstacles, result.bounds);

    if (connected || attempt == config_.max_attempts - 1) {
      result.attempt = attempt;
      result.obstacles = std::move(obstacles);
      result.connected = connected;
      if (config_.mode == MapMode::DENSE_FIELD ||
        config_.mode == MapMode::NARROW_CORRIDOR)
      {
        addBoundaryWalls(result.obstacles, result.bounds);
      }
      result.cloud = downsampleCloud(buildCloud(result.obstacles));
      return result;
    }
  }

  result.attempt = config_.max_attempts - 1;
  result.connected = false;
  result.cloud = downsampleCloud(buildCloud(result.obstacles));
  return result;
}

std::vector<Obstacle> MapGenerator::generateCandidateObstacles(
  const FieldBounds & bounds,
  int obstacle_count,
  int effective_seed) const
{
  std::vector<Obstacle> obstacles;
  if (obstacle_count <= 0) {
    return obstacles;
  }

  // Keep random obstacles away from perimeter walls (inner playable corral).
  constexpr double kWallObstacleMargin = 2.0;
  FieldBounds inner = bounds;
  inner.x_min += kWallObstacleMargin;
  inner.x_max -= kWallObstacleMargin;
  inner.y_min += kWallObstacleMargin;
  inner.y_max -= kWallObstacleMargin;

  std::mt19937 rng(static_cast<unsigned>(effective_seed));
  std::uniform_real_distribution<double> x_dist(inner.x_min, inner.x_max);
  std::uniform_real_distribution<double> y_dist(inner.y_min, inner.y_max);
  std::uniform_real_distribution<double> radius_dist(
    config_.min_obstacle_radius, config_.max_obstacle_radius);
  std::uniform_real_distribution<double> height_dist(0.8, bounds.z_max);
  std::uniform_real_distribution<double> shape_coin(0.0, 1.0);

  const Eigen::Vector2d start(config_.start_x, config_.start_y);
  const Eigen::Vector2d goal(config_.goal_x, config_.goal_y);
  const bool narrow_mode = config_.mode == MapMode::NARROW_CORRIDOR;
  const double passage_center_y = 0.5 * (config_.start_y + config_.goal_y);
  const double gate_x = 0.5 * (config_.start_x + config_.goal_x);
  const double gate_half_gap = config_.corridor_gap_width * 0.5;
  // Side bays stay dense; keep a flyable lane toward the gate (EGO maze road + door).
  const double corridor_half_width = narrow_mode ? 2.5 : 0.0;
  const double gate_clear_x = 1.0;

  int placed = 0;
  int rejections = 0;
  const int max_rejections = obstacle_count * 500;

  while (placed < obstacle_count && rejections < max_rejections) {
    Obstacle obs;
    obs.shape = (shape_coin(rng) < 0.75) ? Obstacle::Shape::CYLINDER : Obstacle::Shape::SPHERE;
    obs.radius = radius_dist(rng);
    obs.height = height_dist(rng);
    obs.center.x() = x_dist(rng);
    obs.center.y() = y_dist(rng);
    obs.center.z() = bounds.z_min + obs.height * 0.5;

    if (obs.shape == Obstacle::Shape::SPHERE) {
      obs.center.z() = bounds.z_min + obs.radius +
        (bounds.z_max - bounds.z_min - 2.0 * obs.radius) * shape_coin(rng);
      obs.height = 2.0 * obs.radius;
    }

    const Eigen::Vector2d pos(obs.center.x(), obs.center.y());

    // Narrow corridor: dense clutter in side bays; central lane + gate mouth stay open.
    if (narrow_mode &&
      std::abs(obs.center.y() - passage_center_y) < corridor_half_width + obs.radius)
    {
      ++rejections;
      continue;
    }
    if (narrow_mode &&
      std::abs(obs.center.x() - gate_x) < gate_clear_x + obs.radius &&
      std::abs(obs.center.y() - passage_center_y) < gate_half_gap + obs.radius + 0.15)
    {
      ++rejections;
      continue;
    }

    if ((pos - start).norm() < config_.clearance_radius + obs.radius) {
      ++rejections;
      continue;
    }
    if ((pos - goal).norm() < config_.clearance_radius + obs.radius) {
      ++rejections;
      continue;
    }

    bool too_close = false;
    for (const auto & existing : obstacles) {
      const Eigen::Vector2d ep(existing.center.x(), existing.center.y());
      const double required = config_.min_obstacle_spacing + existing.radius + obs.radius;
      if ((pos - ep).norm() < required) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      ++rejections;
      continue;
    }

    obstacles.push_back(obs);
    ++placed;
  }

  // sparse mode may intentionally place fewer than requested decorative obstacles.
  if (config_.mode == MapMode::SPARSE && obstacles.size() > 2) {
    obstacles.resize(2);
  }

  return obstacles;
}

void MapGenerator::addNarrowPassageGate(
  std::vector<Obstacle> & obstacles,
  const FieldBounds & bounds) const
{
  // PLAN §5.3 + EGO mockamap maze pattern: dense clutter + perpendicular gate with door.
  const double passage_center_y = 0.5 * (config_.start_y + config_.goal_y);
  const double gate_x = 0.5 * (config_.start_x + config_.goal_x);
  const double gap = config_.corridor_gap_width;
  const double gap_low = passage_center_y - gap * 0.5;
  const double gap_high = passage_center_y + gap * 0.5;
  const double wall_height = bounds.z_max - bounds.z_min;
  const double wall_thickness = 0.15;
  constexpr double kInnerMargin = 1.0;
  const double y_inner_min = bounds.y_min + kInnerMargin;
  const double y_inner_max = bounds.y_max - kInnerMargin;

  auto add_y_wall = [&](double y0, double y1) {
    if (y1 - y0 < 0.5) {
      return;
    }
    Obstacle wall;
    wall.shape = Obstacle::Shape::WALL;
    wall.length_along_y = true;
    wall.length = y1 - y0;
    wall.thickness = wall_thickness;
    wall.height = wall_height;
    wall.radius = wall_thickness * 0.5;
    wall.center.x() = gate_x;
    wall.center.y() = 0.5 * (y0 + y1);
    wall.center.z() = bounds.z_min + wall_height * 0.5;
    obstacles.push_back(wall);
  };

  add_y_wall(y_inner_min, gap_low);
  add_y_wall(gap_high, y_inner_max);
}

void MapGenerator::addBoundaryWalls(
  std::vector<Obstacle> & obstacles,
  const FieldBounds & bounds) const
{
  const double t = 0.2;
  const double h = bounds.z_max - bounds.z_min;
  const double zc = bounds.z_min + 0.5 * h;
  const double lx = bounds.x_max - bounds.x_min;
  const double ly = bounds.y_max - bounds.y_min;

  auto make_wall = [&](double cx, double cy, double length, double thickness) {
    Obstacle w;
    w.shape = Obstacle::Shape::WALL;
    w.length = length;
    w.thickness = thickness;
    w.height = h;
    w.radius = thickness * 0.5;
    w.center = Eigen::Vector3d(cx, cy, zc);
    w.is_boundary = true;
    obstacles.push_back(w);
  };

  // WALL model: length along X, thickness along Y.
  make_wall(bounds.x_min - t * 0.5, 0.5 * (bounds.y_min + bounds.y_max), t, ly + 2.0 * t);  // west
  make_wall(bounds.x_max + t * 0.5, 0.5 * (bounds.y_min + bounds.y_max), t, ly + 2.0 * t);  // east
  make_wall(0.5 * (bounds.x_min + bounds.x_max), bounds.y_min - t * 0.5, lx + 2.0 * t, t);  // south
  make_wall(0.5 * (bounds.x_min + bounds.x_max), bounds.y_max + t * 0.5, lx + 2.0 * t, t);  // north
}

bool MapGenerator::isCellOccupied(
  double x, double y, double z,
  const std::vector<Obstacle> & obstacles) const
{
  for (const auto & obs : obstacles) {
    switch (obs.shape) {
      case Obstacle::Shape::CYLINDER: {
        const double dx = x - obs.center.x();
        const double dy = y - obs.center.y();
        const double inflate = obs.radius + config_.safety_distance;
        if (dx * dx + dy * dy <= inflate * inflate) {
          const double half_h = obs.height * 0.5;
          if (z >= obs.center.z() - half_h && z <= obs.center.z() + half_h) {
            return true;
          }
        }
        break;
      }
      case Obstacle::Shape::SPHERE: {
        const Eigen::Vector3d p(x, y, z);
        const double inflate = obs.radius + config_.safety_distance;
        if ((p - obs.center).squaredNorm() <= inflate * inflate) {
          return true;
        }
        break;
      }
      case Obstacle::Shape::WALL: {
        const double half_l = obs.length * 0.5;
        const double half_t = obs.thickness * 0.5 + config_.safety_distance;
        const double half_h = obs.height * 0.5;
        const double dx = x - obs.center.x();
        const double dy = y - obs.center.y();
        const bool inside_xy = obs.length_along_y ?
          (std::abs(dx) <= half_t && std::abs(dy) <= half_l) :
          (std::abs(dx) <= half_l && std::abs(dy) <= half_t);
        if (inside_xy &&
          z >= obs.center.z() - half_h && z <= obs.center.z() + half_h)
        {
          return true;
        }
        break;
      }
    }
  }
  return false;
}

bool MapGenerator::checkConnectivity(
  const std::vector<Obstacle> & obstacles,
  const FieldBounds & bounds) const
{
  const double res = config_.grid_resolution;
  const int nx = std::max(1, static_cast<int>(std::ceil((bounds.x_max - bounds.x_min) / res)));
  const int ny = std::max(1, static_cast<int>(std::ceil((bounds.y_max - bounds.y_min) / res)));

  const int sx = gridIndex(config_.start_x, bounds.x_min, res);
  const int sy = gridIndex(config_.start_y, bounds.y_min, res);
  const int gx = gridIndex(config_.goal_x, bounds.x_min, res);
  const int gy = gridIndex(config_.goal_y, bounds.y_min, res);

  if (sx < 0 || sx >= nx || sy < 0 || sy >= ny ||
    gx < 0 || gx >= nx || gy < 0 || gy >= ny)
  {
    return false;
  }

  const double check_z = 0.5 * (config_.start_z + config_.goal_z);

  auto idx = [nx](int ix, int iy) { return iy * nx + ix; };

  std::vector<uint8_t> blocked(static_cast<size_t>(nx * ny), 0);
  for (int iy = 0; iy < ny; ++iy) {
    for (int ix = 0; ix < nx; ++ix) {
      const double x = bounds.x_min + (ix + 0.5) * res;
      const double y = bounds.y_min + (iy + 0.5) * res;
      if (isCellOccupied(x, y, check_z, obstacles)) {
        blocked[static_cast<size_t>(idx(ix, iy))] = 1;
      }
    }
  }

  if (blocked[static_cast<size_t>(idx(sx, sy))] ||
    blocked[static_cast<size_t>(idx(gx, gy))])
  {
    return false;
  }

  std::vector<uint8_t> visited(static_cast<size_t>(nx * ny), 0);
  std::queue<std::pair<int, int>> q;
  q.emplace(sx, sy);
  visited[static_cast<size_t>(idx(sx, sy))] = 1;

  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  while (!q.empty()) {
    const auto [cx, cy] = q.front();
    q.pop();
    if (cx == gx && cy == gy) {
      return true;
    }
    for (int k = 0; k < 4; ++k) {
      const int nx_c = cx + dx[k];
      const int ny_c = cy + dy[k];
      if (nx_c < 0 || nx_c >= nx || ny_c < 0 || ny_c >= ny) {
        continue;
      }
      const size_t id = static_cast<size_t>(idx(nx_c, ny_c));
      if (visited[id] || blocked[id]) {
        continue;
      }
      visited[id] = 1;
      q.emplace(nx_c, ny_c);
    }
  }

  return false;
}

pcl::PointCloud<pcl::PointXYZ> MapGenerator::sampleObstacleSurface(
  const Obstacle & obstacle) const
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const double step = std::max(config_.point_resolution, 0.02);

  if (obstacle.shape == Obstacle::Shape::CYLINDER) {
    const double r = obstacle.radius;
    const double half_h = obstacle.height * 0.5;
    const double z_bottom = obstacle.center.z() - half_h;
    const double z_top = obstacle.center.z() + half_h;
    const double angle_step = step / std::max(r, 1e-3);
    const double z_step = step;

    for (double z = z_bottom; z <= z_top + 1e-6; z += z_step) {
      for (double a = 0.0; a < 2.0 * kPi; a += angle_step) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(obstacle.center.x() + r * std::cos(a));
        pt.y = static_cast<float>(obstacle.center.y() + r * std::sin(a));
        pt.z = static_cast<float>(z);
        cloud.points.push_back(pt);
      }
    }

    for (double rr = 0.0; rr <= r; rr += step) {
      for (double a = 0.0; a < 2.0 * kPi; a += angle_step) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(obstacle.center.x() + rr * std::cos(a));
        pt.y = static_cast<float>(obstacle.center.y() + rr * std::sin(a));
        pt.z = static_cast<float>(z_top);
        cloud.points.push_back(pt);
      }
    }
  } else if (obstacle.shape == Obstacle::Shape::SPHERE) {
    const double r = obstacle.radius;
    const int lat_steps = std::max(3, static_cast<int>(std::ceil((2.0 * r) / step)));
    for (int i = 0; i <= lat_steps; ++i) {
      const double v = -1.0 + 2.0 * static_cast<double>(i) / lat_steps;
      const double phi = std::acos(clampValue(v, -1.0, 1.0));
      const double ring_r = r * std::sin(phi);
      const double z = obstacle.center.z() + r * std::cos(phi);
      const double angle_step = step / std::max(ring_r, 1e-3);
      for (double a = 0.0; a < 2.0 * kPi; a += angle_step) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(obstacle.center.x() + ring_r * std::cos(a));
        pt.y = static_cast<float>(obstacle.center.y() + ring_r * std::sin(a));
        pt.z = static_cast<float>(z);
        cloud.points.push_back(pt);
      }
    }
  } else if (obstacle.shape == Obstacle::Shape::WALL) {
    const double half_l = obstacle.length * 0.5;
    const double half_t = obstacle.thickness * 0.5;
    const double half_h = obstacle.height * 0.5;
    double x0 = 0.0;
    double x1 = 0.0;
    double y0 = 0.0;
    double y1 = 0.0;
    if (obstacle.length_along_y) {
      x0 = obstacle.center.x() - half_t;
      x1 = obstacle.center.x() + half_t;
      y0 = obstacle.center.y() - half_l;
      y1 = obstacle.center.y() + half_l;
    } else {
      x0 = obstacle.center.x() - half_l;
      x1 = obstacle.center.x() + half_l;
      y0 = obstacle.center.y() - half_t;
      y1 = obstacle.center.y() + half_t;
    }
    const double z0 = obstacle.center.z() - half_h;
    const double z1 = obstacle.center.z() + half_h;

    auto add_face_grid =
      [&](double ua0, double ua1, double va0, double va1,
        auto map_fn) {
        for (double u = ua0; u <= ua1 + 1e-6; u += step) {
          for (double v = va0; v <= va1 + 1e-6; v += step) {
            const Eigen::Vector3d p = map_fn(u, v);
            cloud.points.emplace_back(
              static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
          }
        }
      };

    add_face_grid(x0, x1, z0, z1, [&](double x, double z) {
      return Eigen::Vector3d(x, y0, z);
    });
    add_face_grid(x0, x1, z0, z1, [&](double x, double z) {
      return Eigen::Vector3d(x, y1, z);
    });
    add_face_grid(y0, y1, z0, z1, [&](double y, double z) {
      return Eigen::Vector3d(x0, y, z);
    });
    add_face_grid(y0, y1, z0, z1, [&](double y, double z) {
      return Eigen::Vector3d(x1, y, z);
    });
    add_face_grid(x0, x1, y0, y1, [&](double x, double y) {
      return Eigen::Vector3d(x, y, z0);
    });
    add_face_grid(x0, x1, y0, y1, [&](double x, double y) {
      return Eigen::Vector3d(x, y, z1);
    });
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ> MapGenerator::buildCloud(
  const std::vector<Obstacle> & obstacles) const
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto & obs : obstacles) {
    const auto part = sampleObstacleSurface(obs);
    cloud.points.insert(cloud.points.end(), part.points.begin(), part.points.end());
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ> MapGenerator::downsampleCloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  if (config_.downsample_voxel <= 0.0 || cloud.empty()) {
    return cloud;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(
    static_cast<float>(config_.downsample_voxel),
    static_cast<float>(config_.downsample_voxel),
    static_cast<float>(config_.downsample_voxel));
  voxel.setInputCloud(cloud.makeShared());
  pcl::PointCloud<pcl::PointXYZ> filtered;
  voxel.filter(filtered);
  filtered.width = filtered.points.size();
  filtered.height = 1;
  filtered.is_dense = true;
  return filtered;
}

}  // namespace drone_map

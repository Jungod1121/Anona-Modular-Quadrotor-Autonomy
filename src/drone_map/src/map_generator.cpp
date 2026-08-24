#include "drone_map/ego_map.hpp"
#include "drone_map/map_generator.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

/// Dense / narrow: walls and cylinders share a fixed height (4 m).
struct VerticalProfile
{
  double wall_h{4.0};
  double pillar_h{4.0};
};

VerticalProfile verticalProfileFor(const FieldBounds & bounds)
{
  const double field_h = std::max(0.5, bounds.z_max - bounds.z_min);
  VerticalProfile vp;
  vp.wall_h = clampValue(4.0, 0.5, field_h);
  vp.pillar_h = vp.wall_h;  // same height as walls
  return vp;
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
  // PLAN §5.3: 1.2~1.8 m ≈ 2.5~3.5× quadrotor outline — wider is not "narrow".
  config_.corridor_gap_width = clampValue(config_.corridor_gap_width, 1.2, 1.8);
  // Prefer odd counts so the weave ends near mid-Y before the goal.
  if (config_.corridor_gate_count < 3) {
    config_.corridor_gate_count = 3;
  } else if (config_.corridor_gate_count > 7) {
    config_.corridor_gate_count = 7;
  } else if (config_.corridor_gate_count % 2 == 0) {
    config_.corridor_gate_count += 1;  // 4→5, 6→7
  }
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
      // Wide open field filling a typical RViz orbit (~50×40 m). No perimeter
      // walls by default (see add_boundary_walls); pillars fill to the edges.
      b.x_min = -8.0;
      b.x_max = 48.0;
      b.y_min = -8.0;
      b.y_max = 32.0;
      b.z_min = 0.0;
      b.z_max = 4.0;
      break;
    case MapMode::NARROW_CORRIDOR:
      // Outer flyable envelope; z matches fixed 4 m walls / pillars.
      b.x_min = -2.0;
      b.x_max = 22.0;
      b.y_min = -2.0;
      b.y_max = 12.0;
      b.z_min = 0.0;
      b.z_max = 4.0;
      break;
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
  if (config_.use_custom_bounds && config_.x_max > config_.x_min &&
    config_.y_max > config_.y_min)
  {
    result.bounds.x_min = config_.x_min;
    result.bounds.x_max = config_.x_max;
    result.bounds.y_min = config_.y_min;
    result.bounds.y_max = config_.y_max;
    if (config_.z_max > config_.z_min) {
      result.bounds.z_min = config_.z_min;
      result.bounds.z_max = config_.z_max;
    }
  }
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
    // Honor custom bounds; fall back to the EGO mockamap 20×20 m default.
    const bool have_bounds = config_.use_custom_bounds &&
      config_.x_max > config_.x_min && config_.y_max > config_.y_min;
    mc.x_length = have_bounds ? field_x : 20.0;
    mc.y_length = have_bounds ? field_y : 20.0;
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
      result.cloud = downsampleCloud(ego::generateRandomForest(fc));
      if (checkCloudConnectivity(result.cloud, result.bounds)) {
        result.connected = true;
        return result;
      }
      // Blocked: retry with a different effective seed; publish the last try
      // with connected=false so the node logs the degraded state honestly.
    }
    result.connected = false;
    return result;
  }

  int target_count = obstacleCountForMode(config_.mode);
  if (config_.mode == MapMode::SPARSE) {
    // "Sparse" still means a few pillars — seed % 3 could legally produce 0,
    // yielding an empty cloud that contradicted the catalog entry.
    target_count = std::max(2, effectiveSeed(0) % 5);
  }
  if (config_.mode == MapMode::DENSE_FIELD) {
    // Keep pillar density ≈ 80 / (24×14) while the field grows.
    constexpr double kRefArea = 24.0 * 14.0;
    constexpr double kRefCount = 80.0;
    const double area = field_x * field_y;
    target_count = std::max(
      80, static_cast<int>(std::lround(kRefCount * area / kRefArea)));
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
      const bool want_walls =
        config_.add_boundary_walls &&
        (config_.mode == MapMode::DENSE_FIELD ||
         config_.mode == MapMode::NARROW_CORRIDOR);
      if (want_walls) {
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

  // Keep pillars slightly inset from the AABB edge (no hard walls when
  // add_boundary_walls is false — still avoid clipping the cloud silhouette).
  const double kWallObstacleMargin = config_.add_boundary_walls ? 2.0 : 0.35;
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

  const bool uniform_pillars =
    config_.mode == MapMode::DENSE_FIELD ||
    config_.mode == MapMode::NARROW_CORRIDOR;
  const VerticalProfile vp = verticalProfileFor(bounds);

  const Eigen::Vector2d start(config_.start_x, config_.start_y);
  const Eigen::Vector2d goal(config_.goal_x, config_.goal_y);
  const bool narrow_mode = config_.mode == MapMode::NARROW_CORRIDOR;
  // Keep-out tube around the forced S-bend only (NOT a 5 m open highway).
  // Inspired by EGO maze roads / AirSim narrow-corridor benchmarks + PLAN §5.3.
  const auto narrow_path = narrow_mode ? narrowPassageWaypoints() : std::vector<Eigen::Vector2d>{};
  const double lane_half = narrow_mode ?
    (0.5 * config_.corridor_gap_width + 0.45) : 0.0;

  auto dist_to_narrow_path = [&](const Eigen::Vector2d & p) -> double {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < narrow_path.size(); ++i) {
      const Eigen::Vector2d & a = narrow_path[i - 1];
      const Eigen::Vector2d & b = narrow_path[i];
      const Eigen::Vector2d ab = b - a;
      const double ab2 = ab.squaredNorm();
      double t = (ab2 > 1e-12) ? (p - a).dot(ab) / ab2 : 0.0;
      t = clampValue(t, 0.0, 1.0);
      best = std::min(best, (p - (a + t * ab)).norm());
    }
    return best;
  };

  int placed = 0;
  int rejections = 0;
  const int max_rejections = obstacle_count * 500;

  while (placed < obstacle_count && rejections < max_rejections) {
    Obstacle obs;
    obs.shape = (shape_coin(rng) < 0.75) ? Obstacle::Shape::CYLINDER : Obstacle::Shape::SPHERE;
    obs.radius = radius_dist(rng);
    // Dense / narrow: every cylinder shares one height, slightly above walls.
    obs.height = (uniform_pillars && obs.shape == Obstacle::Shape::CYLINDER)
      ? vp.pillar_h
      : height_dist(rng);
    obs.center.x() = x_dist(rng);
    obs.center.y() = y_dist(rng);
    obs.center.z() = bounds.z_min + obs.height * 0.5;

    if (obs.shape == Obstacle::Shape::SPHERE) {
      obs.center.z() = bounds.z_min + obs.radius +
        (bounds.z_max - bounds.z_min - 2.0 * obs.radius) * shape_coin(rng);
      obs.height = 2.0 * obs.radius;
    }

    const Eigen::Vector2d pos(obs.center.x(), obs.center.y());

    // Narrow corridor: clutter the side bays; only a thin tube along the S-path stays open.
    if (narrow_mode && dist_to_narrow_path(pos) < lane_half + obs.radius) {
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

std::vector<Eigen::Vector2d> MapGenerator::narrowPassageWaypoints() const
{
  // Staggered gates force an S-bend (场景5「绕行」), not a straight shot through one door.
  const double x0 = config_.start_x;
  const double y0 = config_.start_y;
  const double x1 = config_.goal_x;
  const double y1 = config_.goal_y;
  const double y_mid = 0.5 * (y0 + y1);
  const double span = std::max(1.0, x1 - x0);
  const double gap = config_.corridor_gap_width;
  // Lateral offset must exceed the gap so successive doors do not align.
  const double offset = std::max(2.0, gap + 0.7);
  const int n_gates = config_.corridor_gate_count;

  std::vector<Eigen::Vector2d> wps;
  wps.reserve(static_cast<size_t>(n_gates) + 2);
  wps.emplace_back(x0, y0);
  for (int i = 0; i < n_gates; ++i) {
    const double t = (i + 1.0) / (n_gates + 1.0);
    const double x = x0 + t * span;
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    wps.emplace_back(x, y_mid + sign * offset);
  }
  wps.emplace_back(x1, y1);
  return wps;
}

void MapGenerator::addNarrowPassageGate(
  std::vector<Obstacle> & obstacles,
  const FieldBounds & bounds) const
{
  // PLAN §5.3: deterministic walls that leave only a 1.2~1.8 m slit.
  // N full-span N–S walls with offset doors → must weave (绕行).
  const auto wps = narrowPassageWaypoints();
  if (wps.size() < 3) {
    return;
  }
  const double gap = config_.corridor_gap_width;
  const VerticalProfile vp = verticalProfileFor(bounds);
  const double wall_height = vp.wall_h;
  // ≥ ~2× SDF res (0.15) so solid voxels never leave a straight-line hole at y=mid.
  const double wall_thickness = 0.32;
  constexpr double kInnerMargin = 0.6;
  const double y_inner_min = bounds.y_min + kInnerMargin;
  const double y_inner_max = bounds.y_max - kInnerMargin;

  auto add_y_wall = [&](double gate_x, double y0, double y1) {
    if (y1 - y0 < 0.4) {
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

  auto add_gate_at = [&](double gate_x, double gap_cy) {
    const double gap_low = gap_cy - gap * 0.5;
    const double gap_high = gap_cy + gap * 0.5;
    add_y_wall(gate_x, y_inner_min, gap_low);
    add_y_wall(gate_x, gap_high, y_inner_max);
  };

  // Intermediate waypoints are door centers (skip start/goal).
  for (size_t i = 1; i + 1 < wps.size(); ++i) {
    add_gate_at(wps[i].x(), wps[i].y());
  }
}

void MapGenerator::addBoundaryWalls(
  std::vector<Obstacle> & obstacles,
  const FieldBounds & bounds) const
{
  const double t = 0.2;
  const VerticalProfile vp = verticalProfileFor(bounds);
  const double h = vp.wall_h;
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
        // Inflate all three dimensions symmetrically — ends and tops of
        // walls are just as lethal as their faces.
        const double half_l = obs.length * 0.5 + config_.safety_distance;
        const double half_t = obs.thickness * 0.5 + config_.safety_distance;
        const double half_h = obs.height * 0.5 + config_.safety_distance;
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

  // Sample the vertical flight band at three planes — a single mid-plane
  // misses spheres / overhangs floating above or below it.
  const double z_lo = std::min(config_.start_z, config_.goal_z);
  const double z_hi = std::max(config_.start_z, config_.goal_z);
  const double check_zs[3] = {
    z_lo, 0.5 * (config_.start_z + config_.goal_z), z_hi};

  auto idx = [nx](int ix, int iy) { return iy * nx + ix; };

  std::vector<uint8_t> blocked(static_cast<size_t>(nx * ny), 0);
  for (int iy = 0; iy < ny; ++iy) {
    for (int ix = 0; ix < nx; ++ix) {
      const double x = bounds.x_min + (ix + 0.5) * res;
      const double y = bounds.y_min + (iy + 0.5) * res;
      for (const double check_z : check_zs) {
        if (isCellOccupied(x, y, check_z, obstacles)) {
          blocked[static_cast<size_t>(idx(ix, iy))] = 1;
          break;
        }
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

bool MapGenerator::checkCloudConnectivity(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
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

  // Trunk/canopy points within the flight band block their horizontal cell
  // (inflated by the configured safety distance). Points above the band
  // (suspended EGO circles) are ignored — they do not obstruct cruise flight.
  const double z_band_top =
    std::max(config_.start_z, config_.goal_z) + 0.5;
  const double inflate = config_.safety_distance;

  std::vector<uint8_t> blocked(static_cast<size_t>(nx * ny), 0);
  for (const auto & p : cloud.points) {
    if (p.z > z_band_top) {
      continue;
    }
    // Mark every cell whose center lies within `inflate` of the point (XY).
    const int ix_min = std::max(0,
      static_cast<int>(std::floor((p.x - inflate - bounds.x_min) / res)));
    const int ix_max = std::min(nx - 1,
      static_cast<int>(std::floor((p.x + inflate - bounds.x_min) / res)));
    const int iy_min = std::max(0,
      static_cast<int>(std::floor((p.y - inflate - bounds.y_min) / res)));
    const int iy_max = std::min(ny - 1,
      static_cast<int>(std::floor((p.y + inflate - bounds.y_min) / res)));
    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double cx = bounds.x_min + (ix + 0.5) * res;
        const double cy = bounds.y_min + (iy + 0.5) * res;
        if ((cx - p.x) * (cx - p.x) + (cy - p.y) * (cy - p.y) <= inflate * inflate) {
          blocked[static_cast<size_t>(iy * nx + ix)] = 1;
        }
      }
    }
  }

  if (blocked[static_cast<size_t>(sy * nx + sx)] ||
    blocked[static_cast<size_t>(gy * nx + gx)])
  {
    return false;
  }

  std::vector<uint8_t> visited(static_cast<size_t>(nx * ny), 0);
  std::queue<std::pair<int, int>> q;
  q.emplace(sx, sy);
  visited[static_cast<size_t>(sy * nx + sx)] = 1;

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
      const size_t id = static_cast<size_t>(ny_c * nx + nx_c);
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

    // Solid fill (not faces only). Thin gate walls otherwise leave holes at
    // SDF resolution ~0.15 m so Fast-Planner flies straight through the slab.
    // Step ≥0.12 keeps the cloud manageable while sealing voxels.
    const double fill = std::max(step, 0.12);
    for (double x = x0; x <= x1 + 1e-6; x += fill) {
      for (double y = y0; y <= y1 + 1e-6; y += fill) {
        for (double z = z0; z <= z1 + 1e-6; z += fill) {
          cloud.points.emplace_back(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        }
      }
    }
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

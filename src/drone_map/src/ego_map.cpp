// Ported from EGO-Planner mockamap (maps.cpp) + map_generator (random_forest_sensing.cpp).
// Reference: ego-planner-swarm/src/uav_simulator/mockamap, map_generator.
#include "drone_map/ego_map.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <random>
#include <vector>

namespace drone_map
{
namespace ego
{
namespace
{

void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi & maze)
{
  // 如果当前区域大小为 5x5 或更大，将在该区域中生成一个垂直和水平的墙，将区域分成 4 个子区域，然后递归处理每个子区域
  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;    // 生成墙的中心位置
    while (valid == false)
    {
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                // add a wall at the sides)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // generating random number between yl+1 and yh-1(pointless to
                // add a wall at the sides)
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      valid = true;

    } // xm and ym are now the valid coordinate of the center of the wall
    // 添加垂直和水平墙
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    // 随机生成门的位置
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }    // 递归调用以分割四个子区域
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);    return;
  } // when the remaining area is larger than or equal to 5*5

  // 特殊区域处理
  else if (xl < xh - 2 && yl < yh - 2)
  {
    // bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // generating random number between xl+1 and xh-1(pointless to
                          // add a wall at the sides)
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // generating random number between yl+1 and yh-1(pointless to
           // add a wall at the sides)
           // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    return;
  }

  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    int doorcount = 0;
    int ym        = 0;
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  else
  {    return;
  }
}

void carveWorldDisk(
  Eigen::MatrixXi & maze,
  int mx, int my, double width, double scale,
  double x_length, double y_length,
  double origin_x, double origin_y,
  double wx, double wy, double radius_m)
{
  const double half_x = x_length * 0.5;
  const double half_y = y_length * 0.5;
  const double lx = wx - origin_x;
  const double ly = wy - origin_y;
  const int ci = static_cast<int>(std::floor((lx + half_x) / width));
  const int cj = static_cast<int>(std::floor((ly + half_y) / width));
  const int r_cells = std::max(1, static_cast<int>(std::ceil(radius_m / width)));
  for (int di = -r_cells; di <= r_cells; ++di) {
    for (int dj = -r_cells; dj <= r_cells; ++dj) {
      const int i = ci + di;
      const int j = cj + dj;
      if (i >= 0 && i < mx && j >= 0 && j < my) {
        const double cx = (i + 0.5) * width - half_x;
        const double cy = (j + 0.5) * width - half_y;
        if (std::hypot(cx - lx, cy - ly) <= radius_m + width * 0.6) {
          maze(i, j) = 0;
        }
      }
    }
  }
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ> generateMaze2D(const MazeConfig & cfg)
{
  std::srand(static_cast<unsigned>(cfg.seed));
  const double scale = 1.0 / cfg.resolution;
  const int sizeX = static_cast<int>(cfg.x_length * scale);
  const int sizeY = static_cast<int>(cfg.y_length * scale);
  const int sizeZ = static_cast<int>(cfg.z_length * scale);
  const double width = cfg.road_width;

  const int mx = sizeX / static_cast<int>(width * scale);
  const int my = sizeY / static_cast<int>(width * scale);
  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);

  if (cfg.add_wall_x) {
    for (int i = 0; i < mx; ++i) {
      maze(i, 0) = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (cfg.add_wall_y) {
    for (int i = 0; i < my; ++i) {
      maze(0, i) = 1;
      maze(mx - 1, i) = 1;
    }
  }

  carveWorldDisk(maze, mx, my, width, scale, cfg.x_length, cfg.y_length,
    cfg.origin_x, cfg.origin_y, cfg.start_x, cfg.start_y, cfg.clearance_radius);
  carveWorldDisk(maze, mx, my, width, scale, cfg.x_length, cfg.y_length,
    cfg.origin_x, cfg.origin_y, cfg.goal_x, cfg.goal_y, cfg.clearance_radius);

  // Our start→goal is east-west; open one maze row along the flight line.
  {
    const double half_y = cfg.y_length * 0.5;
    const double ly = cfg.start_y - cfg.origin_y + half_y;
    const int jc = static_cast<int>(std::floor(ly / width));
    const int half_w = std::max(1, static_cast<int>(std::ceil(cfg.road_width / width)));
    for (int i = 0; i < mx; ++i) {
      for (int dj = -half_w; dj <= half_w; ++dj) {
        const int j = jc + dj;
        if (j >= 0 && j < my) {
          maze(i, j) = 0;
        }
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  const double half_x = cfg.x_length * 0.5;
  const double half_y = cfg.y_length * 0.5;
  for (int i = 0; i < mx; ++i) {
    for (int j = 0; j < my; ++j) {
      if (!maze(i, j)) {
        continue;
      }
      const int voxels = static_cast<int>(width * scale);
      for (int ii = 0; ii < voxels; ++ii) {
        for (int jj = 0; jj < voxels; ++jj) {
          for (int k = 0; k < sizeZ; ++k) {
            pcl::PointXYZ pt;
            pt.x = static_cast<float>(
              i * width + ii / scale - half_x + cfg.origin_x);
            pt.y = static_cast<float>(
              j * width + jj / scale - half_y + cfg.origin_y);
            pt.z = static_cast<float>(k / scale);
            cloud.points.push_back(pt);
          }
        }
      }
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}


pcl::PointCloud<pcl::PointXYZ> generateRandomForest(const ForestConfig & cfg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::mt19937 eng(static_cast<unsigned>(cfg.seed));
  const double half_x = cfg.x_length * 0.5;
  const double half_y = cfg.y_length * 0.5;

  std::uniform_real_distribution<double> rand_x(-half_x, half_x);
  std::uniform_real_distribution<double> rand_y(-half_y, half_y);
  std::uniform_real_distribution<double> rand_w(cfg.lower_rad, cfg.upper_rad);
  std::uniform_real_distribution<double> rand_h(cfg.lower_hei, cfg.upper_hei);
  std::uniform_real_distribution<double> rand_inf(0.5, 1.5);
  std::uniform_real_distribution<double> rand_radius(cfg.lower_rad, cfg.upper_rad);
  std::uniform_real_distribution<double> rand_radius2(cfg.lower_rad, 1.2);
  std::uniform_real_distribution<double> rand_theta(-cfg.upper_rad, cfg.upper_rad);
  std::uniform_real_distribution<double> rand_z(0.5, cfg.z_length - 0.5);

  std::vector<Eigen::Vector2d> obs_position;
  const Eigen::Vector2d start_local(cfg.start_x - cfg.origin_x, cfg.start_y - cfg.origin_y);
  const Eigen::Vector2d goal_local(cfg.goal_x - cfg.origin_x, cfg.goal_y - cfg.origin_y);

  auto too_close_special = [&](double x, double y) {
    const Eigen::Vector2d p(x, y);
    if ((p - start_local).norm() < cfg.clearance_radius) {
      return true;
    }
    if ((p - goal_local).norm() < cfg.clearance_radius) {
      return true;
    }
    for (const auto & op : obs_position) {
      if ((p - op).norm() < cfg.min_distance) {
        return true;
      }
    }
    return false;
  };

  for (int i = 0; i < cfg.obs_num; ++i) {
    double x = rand_x(eng);
    double y = rand_y(eng);
    double w = rand_w(eng);
    const double inf = rand_inf(eng);
    if (too_close_special(x, y)) {
      --i;
      continue;
    }
    obs_position.emplace_back(x, y);

    x = std::floor(x / cfg.resolution) * cfg.resolution + cfg.resolution * 0.5;
    y = std::floor(y / cfg.resolution) * cfg.resolution + cfg.resolution * 0.5;

    const int wid_num = static_cast<int>(std::ceil((w * inf) / cfg.resolution));
    const double radius = (w * inf) * 0.5;
    for (int r = -wid_num / 2; r < wid_num / 2; ++r) {
      for (int s = -wid_num / 2; s < wid_num / 2; ++s) {
        const double h = rand_h(eng);
        const int hei_num = static_cast<int>(std::ceil(h / cfg.resolution));
        for (int t = -20; t < hei_num; ++t) {
          const double tx = x + (r + 0.5) * cfg.resolution + 1e-2;
          const double ty = y + (s + 0.5) * cfg.resolution + 1e-2;
          const double tz = (t + 0.5) * cfg.resolution + 1e-2;
          if (Eigen::Vector2d(tx - x, ty - y).norm() <= radius) {
            cloud.points.emplace_back(
              static_cast<float>(tx + cfg.origin_x),
              static_cast<float>(ty + cfg.origin_y),
              static_cast<float>(tz));
          }
        }
      }
    }
  }

  for (int i = 0; i < cfg.circle_num; ++i) {
    double x = rand_x(eng);
    double y = rand_y(eng);
    double z = rand_z(eng);
    if (too_close_special(x, y)) {
      --i;
      continue;
    }
    x = std::floor(x / cfg.resolution) * cfg.resolution + cfg.resolution * 0.5;
    y = std::floor(y / cfg.resolution) * cfg.resolution + cfg.resolution * 0.5;
    z = std::floor(z / cfg.resolution) * cfg.resolution + cfg.resolution * 0.5;

    const double theta = rand_theta(eng);
    Eigen::Matrix3d rotate;
    rotate << std::cos(theta), -std::sin(theta), 0.0,
      std::sin(theta), std::cos(theta), 0.0, 0, 0, 1;
    const double radius1 = rand_radius(eng);
    const double radius2 = rand_radius2(eng);
    const Eigen::Vector3d translate(x, y, z);

    for (double angle = 0.0; angle < 6.282; angle += cfg.resolution * 0.5) {
      Eigen::Vector3d cpt(0.0, radius1 * std::cos(angle), radius2 * std::sin(angle));
      const Eigen::Vector3d p = rotate * cpt + translate;
      cloud.points.emplace_back(
        static_cast<float>(p.x() + cfg.origin_x),
        static_cast<float>(p.y() + cfg.origin_y),
        static_cast<float>(p.z()));
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}


}  // namespace ego
}  // namespace drone_map

#include "drone_planner/occupancy_grid.hpp"
#include "drone_planner/raycast.hpp"

#include <algorithm>
#include <cmath>

namespace drone_planner
{

void OccupancyGrid::setParams(
  double resolution, double inflate_radius,
  const Eigen::Vector3d & origin, const Eigen::Vector3d & size)
{
  resolution_ = resolution;
  inv_res_ = 1.0 / resolution;
  inflate_ = inflate_radius;
  origin_ = origin;
  size_ = size;
  nx_ = std::max(1, static_cast<int>(std::ceil(size.x() * inv_res_)));
  ny_ = std::max(1, static_cast<int>(std::ceil(size.y() * inv_res_)));
  nz_ = std::max(1, static_cast<int>(std::ceil(size.z() * inv_res_)));
  inflate_cells_ = std::max(0, static_cast<int>(std::ceil(inflate_ * inv_res_)));
  const size_t n = static_cast<size_t>(nx_ * ny_ * nz_);
  occ_raw_.assign(n, 0);
  occ_inflate_.assign(n, 0);
}

void OccupancyGrid::clear()
{
  std::fill(occ_raw_.begin(), occ_raw_.end(), 0);
  std::fill(occ_inflate_.begin(), occ_inflate_.end(), 0);
}

void OccupancyGrid::sealBoundary(int thickness)
{
  thickness = std::max(1, thickness);
  for (int t = 0; t < thickness; ++t) {
    for (int y = 0; y < ny_; ++y) {
      for (int z = 0; z < nz_; ++z) {
        setRawOccupied(t, y, z);
        setRawOccupied(nx_ - 1 - t, y, z);
      }
    }
    for (int x = 0; x < nx_; ++x) {
      for (int z = 0; z < nz_; ++z) {
        setRawOccupied(x, t, z);
        setRawOccupied(x, ny_ - 1 - t, z);
      }
    }
  }
  rebuildInflateLayer();
}

size_t OccupancyGrid::cellIndex(int x, int y, int z) const
{
  return static_cast<size_t>(x + nx_ * (y + ny_ * z));
}

bool OccupancyGrid::indexValid(int x, int y, int z) const
{
  return x >= 0 && y >= 0 && z >= 0 && x < nx_ && y < ny_ && z < nz_;
}

void OccupancyGrid::setRawOccupied(int x, int y, int z)
{
  if (!indexValid(x, y, z)) {
    return;
  }
  occ_raw_[cellIndex(x, y, z)] = 1;
}

void OccupancyGrid::setRawFree(int x, int y, int z)
{
  if (!indexValid(x, y, z)) {
    return;
  }
  occ_raw_[cellIndex(x, y, z)] = 0;
}

void OccupancyGrid::inflateAround(int x, int y, int z)
{
  const int r = inflate_cells_;
  for (int dz = -r; dz <= r; ++dz) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy + dz * dz > r * r) {
          continue;
        }
        const int ix = x + dx;
        const int iy = y + dy;
        const int iz = z + dz;
        if (indexValid(ix, iy, iz)) {
          occ_inflate_[cellIndex(ix, iy, iz)] = 1;
        }
      }
    }
  }
}

void OccupancyGrid::addPoint(const Eigen::Vector3d & p)
{
  Eigen::Vector3i idx;
  if (!worldToIndex(p, idx)) {
    return;
  }
  setRawOccupied(idx.x(), idx.y(), idx.z());
  inflateAround(idx.x(), idx.y(), idx.z());
}

void OccupancyGrid::rebuildInflateLayer()
{
  std::fill(occ_inflate_.begin(), occ_inflate_.end(), 0);
  const int r = inflate_cells_;
  for (int z = 0; z < nz_; ++z) {
    for (int y = 0; y < ny_; ++y) {
      for (int x = 0; x < nx_; ++x) {
        if (!occ_raw_[cellIndex(x, y, z)]) {
          continue;
        }
        for (int dz = -r; dz <= r; ++dz) {
          for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
              if (dx * dx + dy * dy + dz * dz > r * r) {
                continue;
              }
              const int ix = x + dx;
              const int iy = y + dy;
              const int iz = z + dz;
              if (indexValid(ix, iy, iz)) {
                occ_inflate_[cellIndex(ix, iy, iz)] = 1;
              }
            }
          }
        }
      }
    }
  }
}

bool OccupancyGrid::isOccupied(const Eigen::Vector3d & p) const
{
  Eigen::Vector3i idx;
  if (!worldToIndex(p, idx)) {
    return true;
  }
  return occ_inflate_[cellIndex(idx.x(), idx.y(), idx.z())] != 0;
}

bool OccupancyGrid::isOccupiedRaw(const Eigen::Vector3d & p) const
{
  Eigen::Vector3i idx;
  if (!worldToIndex(p, idx)) {
    return true;
  }
  return occ_raw_[cellIndex(idx.x(), idx.y(), idx.z())] != 0;
}

double OccupancyGrid::clearanceAt(const Eigen::Vector3d & p, double max_dist) const
{
  if (isOccupied(p)) {
    return 0.0;
  }
  const int max_steps = std::max(1, static_cast<int>(std::ceil(max_dist * inv_res_)));
  double best = max_dist;
  const Eigen::Vector3d axes[6] = {
    Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()};
  for (const auto & axis : axes) {
    for (int s = 1; s <= max_steps; ++s) {
      const Eigen::Vector3d q = p + axis * (static_cast<double>(s) * resolution_);
      if (isOccupied(q)) {
        best = std::min(best, static_cast<double>(s - 1) * resolution_);
        break;
      }
    }
  }
  return best;
}

bool OccupancyGrid::worldToIndex(const Eigen::Vector3d & p, Eigen::Vector3i & idx) const
{
  idx.x() = static_cast<int>(std::floor((p.x() - origin_.x()) * inv_res_));
  idx.y() = static_cast<int>(std::floor((p.y() - origin_.y()) * inv_res_));
  idx.z() = static_cast<int>(std::floor((p.z() - origin_.z()) * inv_res_));
  return idx.x() >= 0 && idx.y() >= 0 && idx.z() >= 0 &&
         idx.x() < nx_ && idx.y() < ny_ && idx.z() < nz_;
}

Eigen::Vector3d OccupancyGrid::indexToWorld(const Eigen::Vector3i & idx) const
{
  return origin_ + Eigen::Vector3d(
    (idx.x() + 0.5) * resolution_,
    (idx.y() + 0.5) * resolution_,
    (idx.z() + 0.5) * resolution_);
}

bool OccupancyGrid::findFreeNearby(Eigen::Vector3d & p, int max_r) const
{
  Eigen::Vector3i idx;
  const bool in_map = worldToIndex(p, idx);
  if (in_map && !occ_inflate_[cellIndex(idx.x(), idx.y(), idx.z())]) {
    return true;
  }
  if (!in_map) {
    idx.x() = std::clamp(idx.x(), 0, nx_ - 1);
    idx.y() = std::clamp(idx.y(), 0, ny_ - 1);
    idx.z() = std::clamp(idx.z(), 0, nz_ - 1);
  }
  for (int r = 1; r <= max_r; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          const Eigen::Vector3i n = idx + Eigen::Vector3i(dx, dy, dz);
          if (!indexValid(n.x(), n.y(), n.z())) {
            continue;
          }
          if (!occ_inflate_[cellIndex(n.x(), n.y(), n.z())]) {
            p = indexToWorld(n);
            return true;
          }
        }
      }
    }
  }
  return false;
}

void OccupancyGrid::integrateRayHit(const Eigen::Vector3d & sensor, const Eigen::Vector3d & hit)
{
  Eigen::Vector3i s_idx, h_idx;
  if (!worldToIndex(sensor, s_idx) || !worldToIndex(hit, h_idx)) {
    return;
  }

  RayCaster caster;
  if (!caster.setInput(s_idx.cast<double>(), h_idx.cast<double>())) {
    setRawOccupied(h_idx.x(), h_idx.y(), h_idx.z());
    inflateAround(h_idx.x(), h_idx.y(), h_idx.z());
    return;
  }

  Eigen::Vector3d vox;
  while (caster.step(vox)) {
    const int ix = static_cast<int>(vox.x());
    const int iy = static_cast<int>(vox.y());
    const int iz = static_cast<int>(vox.z());
    if (!indexValid(ix, iy, iz)) {
      continue;
    }
    const size_t id = cellIndex(ix, iy, iz);
    occ_raw_[id] = 0;
    occ_inflate_[id] = 0;
  }

  setRawOccupied(h_idx.x(), h_idx.y(), h_idx.z());
  inflateAround(h_idx.x(), h_idx.y(), h_idx.z());
}

void OccupancyGrid::integrateLocalCloud(
  const Eigen::Vector3d & sensor,
  const std::vector<Eigen::Vector3d> & hits,
  bool clear_free_along_ray)
{
  for (const auto & h : hits) {
    if (clear_free_along_ray) {
      integrateRayHit(sensor, h);
    } else {
      Eigen::Vector3i idx;
      if (worldToIndex(h, idx)) {
        setRawOccupied(idx.x(), idx.y(), idx.z());
        inflateAround(idx.x(), idx.y(), idx.z());
      }
    }
  }
}

}  // namespace drone_planner

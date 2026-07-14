#pragma once
/**
 * Occupancy grid with layered inflation + raycast local mapping.
 * Reference: ego-planner plan_env/grid_map.h (inflate buffer + raycastProcess).
 * Changes: ingests PointCloud2 via planner; raw/inflate layers separated.
 */
#include <Eigen/Dense>
#include <cstdint>
#include <vector>

namespace drone_planner
{

class OccupancyGrid
{
public:
  void setParams(double resolution, double inflate_radius,
                 const Eigen::Vector3d & origin, const Eigen::Vector3d & size);

  void clear();

  void sealBoundary(int thickness = 1);

  /** Mark raw occupancy at point; updates inflate layer locally. */
  void addPoint(const Eigen::Vector3d & p);

  /** Rebuild full inflate layer from raw (after bulk ingest). */
  void rebuildInflateLayer();

  /** Inflated occupancy — used by A* / collision checks. */
  bool isOccupied(const Eigen::Vector3d & p) const;

  /** Raw obstacle voxel without inflation margin. */
  bool isOccupiedRaw(const Eigen::Vector3d & p) const;

  int getInflateOccupancy(const Eigen::Vector3d & p) const
  {
    return isOccupied(p) ? 1 : 0;
  }

  double clearanceAt(const Eigen::Vector3d & p, double max_dist = 2.0) const;

  bool worldToIndex(const Eigen::Vector3d & p, Eigen::Vector3i & idx) const;

  Eigen::Vector3d indexToWorld(const Eigen::Vector3i & idx) const;

  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  double resolution() const { return resolution_; }
  double inflateRadius() const { return inflate_; }
  const Eigen::Vector3d & origin() const { return origin_; }

  bool findFreeNearby(Eigen::Vector3d & p, int max_r = 8) const;

  /**
   * Raycast local mapping (EGO raycastProcess simplified):
   * voxels along sensor→hit marked free in raw layer; hit marked occupied.
   */
  void integrateRayHit(const Eigen::Vector3d & sensor, const Eigen::Vector3d & hit);

  void integrateLocalCloud(
    const Eigen::Vector3d & sensor,
    const std::vector<Eigen::Vector3d> & hits,
    bool clear_free_along_ray = false);

  /** Deep copy for multi-drone peer overlays without full remesh. */
  OccupancyGrid clone() const
  {
    return *this;
  }

private:
  size_t cellIndex(int x, int y, int z) const;
  bool indexValid(int x, int y, int z) const;
  void setRawOccupied(int x, int y, int z);
  void setRawFree(int x, int y, int z);
  void inflateAround(int x, int y, int z);

  double resolution_{0.2};
  double inv_res_{5.0};
  double inflate_{0.4};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size_{Eigen::Vector3d(20, 12, 3)};
  int nx_{1}, ny_{1}, nz_{1};
  int inflate_cells_{1};
  std::vector<uint8_t> occ_raw_;
  std::vector<uint8_t> occ_inflate_;
};

}  // namespace drone_planner

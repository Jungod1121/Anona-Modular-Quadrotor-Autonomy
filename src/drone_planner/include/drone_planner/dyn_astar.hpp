#pragma once
/**
 * EGO-style local-pool A* frontend (ego-planner path_searching/dyn_a_star).
 * Reference: dyn_a_star.{h,cpp} — geometric 3D A* on inflated occupancy grid.
 * Changes: OccupancyGrid instead of GridMap; true 3D search (plate mazes).
 */
#include "drone_planner/grid_astar.hpp"
#include "drone_planner/occupancy_grid.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <vector>

namespace drone_planner
{

class DynAStar
{
public:
  void initPool(const Eigen::Vector3i & pool_size);

  bool search(
    const OccupancyGrid & grid,
    Eigen::Vector3d start, Eigen::Vector3d goal,
    std::vector<Eigen::Vector3d> & path,
    const AStarOptions & opt = AStarOptions{});

  std::vector<Eigen::Vector3d> getPath() const { return path_; }

private:
  struct GridNode
  {
    enum State
    {
      OPEN = 1,
      CLOSED = 2,
      UNDEF = 3
    };

    int round{0};
    State state{UNDEF};
    Eigen::Vector3i index{Eigen::Vector3i::Zero()};
    double g{std::numeric_limits<double>::infinity()};
    double f{std::numeric_limits<double>::infinity()};
    int parent{-1};
  };

  struct NodeCmp
  {
    const std::vector<GridNode> * nodes;
    bool operator()(int a, int b) const
    {
      return (*nodes)[static_cast<size_t>(a)].f > (*nodes)[static_cast<size_t>(b)].f;
    }
  };

  bool coordToIndex(const Eigen::Vector3d & pt, Eigen::Vector3i & idx) const;
  Eigen::Vector3d indexToCoord(const Eigen::Vector3i & idx) const;
  int toAddress(const Eigen::Vector3i & idx) const;
  bool indexInPool(const Eigen::Vector3i & idx) const;
  bool checkOccupancy(const Eigen::Vector3d & p) const;
  double diagHeu(const Eigen::Vector3i & a, const Eigen::Vector3i & b) const;
  bool adjustStartEnd(Eigen::Vector3d & start, Eigen::Vector3d & goal,
                      Eigen::Vector3i & s_idx, Eigen::Vector3i & g_idx) const;
  bool retrievePath(int end_addr);

  const OccupancyGrid * grid_{nullptr};
  AStarOptions opt_{};
  Eigen::Vector3i pool_size_{Eigen::Vector3i::Zero()};
  Eigen::Vector3i center_idx_{Eigen::Vector3i::Zero()};
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()};
  double step_size_{0.25};
  double inv_step_{4.0};
  int round_{0};
  std::vector<GridNode> nodes_;
  std::vector<Eigen::Vector3d> path_;
  static constexpr double kTieBreaker = 1.0 + 1.0 / 10000.0;
};

}  // namespace drone_planner

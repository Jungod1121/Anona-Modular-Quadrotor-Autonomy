#pragma once
/**
 * Grid A* front-end.
 * Reference: ego-planner-swarm path_searching/dyn_a_star.{h,cpp}
 * Changes: OccupancyGrid instead of plan_env; geometric A* with strong preference
 * for horizontal avoidance (altitude band + vertical move penalty) so the drone
 * goes AROUND obstacles instead of climbing OVER them.
 */
#include "drone_planner/occupancy_grid.hpp"

#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>

namespace drone_planner
{

struct AStarOptions
{
  double cruise_z{1.5};
  double z_band{0.35};          // only search within |z - cruise_z| <= z_band
  double vertical_cost_scale{8.0};  // dz steps cost this much more
  /// Live peer drones (shared-field): treat as cylindrical keep-out.
  std::vector<Eigen::Vector3d> peer_centers;
  double peer_radius{0.75};
};

class GridAStar
{
public:
  bool search(const OccupancyGrid & grid,
              Eigen::Vector3d start, Eigen::Vector3d goal,
              std::vector<Eigen::Vector3d> & path,
              const AStarOptions & opt = AStarOptions{})
  {
    path.clear();

    auto blockedByPeer = [&](const Eigen::Vector3d & wp) {
      const double r2 = opt.peer_radius * opt.peer_radius;
      for (const auto & c : opt.peer_centers) {
        const double dx = wp.x() - c.x();
        const double dy = wp.y() - c.y();
        if (dx * dx + dy * dy <= r2) {
          return true;
        }
      }
      return false;
    };

    auto isBlocked = [&](const Eigen::Vector3d & wp) {
      return grid.isOccupied(wp) || blockedByPeer(wp);
    };

    // Snap start/goal onto cruise altitude for horizontal avoidance scenes.
    start.z() = opt.cruise_z;
    goal.z() = opt.cruise_z;

    if (!grid.findFreeNearby(start) || !grid.findFreeNearby(goal)) {
      return false;
    }
    // Keep cruise after free-space snap (findFreeNearby may drift in z).
    start.z() = opt.cruise_z;
    goal.z() = opt.cruise_z;
    // Push off peer bodies if we spawned inside keep-out.
    for (int k = 0; k < 12 && blockedByPeer(start); ++k) {
      start.y() += (k % 2 == 0 ? 1 : -1) * (0.3 + 0.1 * k);
    }
    for (int k = 0; k < 12 && blockedByPeer(goal); ++k) {
      goal.y() += (k % 2 == 0 ? 1 : -1) * (0.3 + 0.1 * k);
    }
    if (!grid.findFreeNearby(start) || !grid.findFreeNearby(goal)) {
      return false;
    }
    start.z() = opt.cruise_z;
    goal.z() = opt.cruise_z;

    Eigen::Vector3i sidx, gidx;
    if (!grid.worldToIndex(start, sidx) || !grid.worldToIndex(goal, gidx)) {
      return false;
    }

    auto key = [&](const Eigen::Vector3i & i) {
      return (static_cast<int64_t>(i.x()) << 42) ^
             (static_cast<int64_t>(i.y()) << 21) ^
             static_cast<int64_t>(i.z());
    };

    struct Node
    {
      Eigen::Vector3i idx;
      double g{0}, f{0};
      int64_t parent{-1};
    };

    auto heu = [&](const Eigen::Vector3i & a, const Eigen::Vector3i & b) {
      const double dx = std::abs(static_cast<double>(a.x() - b.x()));
      const double dy = std::abs(static_cast<double>(a.y() - b.y()));
      const double dz = std::abs(static_cast<double>(a.z() - b.z()));
      const double d1 = std::min({dx, dy, dz});
      const double d2 = std::max({std::min(dx, dy), std::min(dy, dz), std::min(dx, dz)});
      const double d3 = std::max({dx, dy, dz});
      const double h = (std::sqrt(3.0) - std::sqrt(2.0)) * d1 +
                       (std::sqrt(2.0) - 1.0) * d2 + d3;
      // Prefer planar progress toward goal
      return (h + (opt.vertical_cost_scale - 1.0) * dz) * (1.0 + 1e-4);
    };

    const double z_min = opt.cruise_z - opt.z_band;
    const double z_max = opt.cruise_z + opt.z_band;

    auto inAltitudeBand = [&](const Eigen::Vector3i & nidx) {
      const Eigen::Vector3d wp = grid.indexToWorld(nidx);
      return wp.z() >= z_min && wp.z() <= z_max;
    };

    std::unordered_map<int64_t, Node> nodes;
    using PQItem = std::pair<double, int64_t>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;

    Node sn;
    sn.idx = sidx;
    sn.g = 0;
    sn.f = heu(sidx, gidx);
    sn.parent = -1;
    const int64_t sk = key(sidx);
    nodes[sk] = sn;
    open.push({sn.f, sk});

    const int max_iters = 300000;
    int iters = 0;
    int64_t best = -1;

    while (!open.empty() && iters++ < max_iters) {
      const auto [fcur, ck] = open.top();
      open.pop();
      auto it = nodes.find(ck);
      if (it == nodes.end()) {
        continue;
      }
      if (fcur > it->second.f + 1e-9) {
        continue;
      }
      if (it->second.idx == gidx) {
        best = ck;
        break;
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            // Prefer planar neighbors: allow dz only with horizontal move, not pure vertical.
            if (dz != 0 && dx == 0 && dy == 0) {
              continue;
            }
            Eigen::Vector3i nidx = it->second.idx + Eigen::Vector3i(dx, dy, dz);
            if (nidx.x() < 0 || nidx.y() < 0 || nidx.z() < 0 ||
                nidx.x() >= grid.nx() || nidx.y() >= grid.ny() || nidx.z() >= grid.nz()) {
              continue;
            }
            if (!inAltitudeBand(nidx)) {
              continue;
            }
            const Eigen::Vector3d wp = grid.indexToWorld(nidx);
            if (isBlocked(wp)) {
              continue;
            }
            double step = std::sqrt(dx * dx + dy * dy + dz * dz) * grid.resolution();
            if (dz != 0) {
              step *= opt.vertical_cost_scale;
            }
            const double ng = it->second.g + step;
            const int64_t nk = key(nidx);
            auto nit = nodes.find(nk);
            if (nit == nodes.end() || ng < nit->second.g) {
              Node nn;
              nn.idx = nidx;
              nn.g = ng;
              nn.f = ng + heu(nidx, gidx);
              nn.parent = ck;
              nodes[nk] = nn;
              open.push({nn.f, nk});
            }
          }
        }
      }
    }

    if (best < 0) {
      return false;
    }
    std::vector<Eigen::Vector3i> rev;
    for (int64_t cur = best; cur >= 0; ) {
      rev.push_back(nodes[cur].idx);
      cur = nodes[cur].parent;
    }
    std::reverse(rev.begin(), rev.end());
    path.reserve(rev.size());
    for (const auto & i : rev) {
      Eigen::Vector3d p = grid.indexToWorld(i);
      p.z() = opt.cruise_z;  // flatten to cruise altitude
      path.push_back(p);
    }
    if (!path.empty()) {
      path.front() = start;
      path.back() = goal;
    }
    return path.size() >= 2;
  }
};

}  // namespace drone_planner

#include "drone_planner/dyn_astar.hpp"

#include <algorithm>
#include <chrono>

namespace drone_planner
{

void DynAStar::initPool(const Eigen::Vector3i & pool_size)
{
  pool_size_ = pool_size.cwiseMax(Eigen::Vector3i(8, 8, 8));
  center_idx_ = pool_size_ / 2;
  nodes_.assign(static_cast<size_t>(pool_size_.x() * pool_size_.y() * pool_size_.z()), GridNode{});
}

int DynAStar::toAddress(const Eigen::Vector3i & idx) const
{
  return idx.x() + pool_size_.x() * (idx.y() + pool_size_.y() * idx.z());
}

bool DynAStar::indexInPool(const Eigen::Vector3i & idx) const
{
  return idx.x() >= 0 && idx.y() >= 0 && idx.z() >= 0 &&
         idx.x() < pool_size_.x() && idx.y() < pool_size_.y() && idx.z() < pool_size_.z();
}

bool DynAStar::coordToIndex(const Eigen::Vector3d & pt, Eigen::Vector3i & idx) const
{
  idx = ((pt - center_) * inv_step_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + center_idx_;
  return indexInPool(idx);
}

Eigen::Vector3d DynAStar::indexToCoord(const Eigen::Vector3i & idx) const
{
  return ((idx - center_idx_).cast<double>() * step_size_) + center_;
}

bool DynAStar::checkOccupancy(const Eigen::Vector3d & p) const
{
  return grid_ != nullptr && grid_->getInflateOccupancy(p) != 0;
}

double DynAStar::diagHeu(const Eigen::Vector3i & a, const Eigen::Vector3i & b) const
{
  const double dx = std::abs(static_cast<double>(a.x() - b.x()));
  const double dy = std::abs(static_cast<double>(a.y() - b.y()));
  const double dz = std::abs(static_cast<double>(a.z() - b.z()));
  const double d1 = std::min({dx, dy, dz});
  const double d2 = std::max({std::min(dx, dy), std::min(dy, dz), std::min(dx, dz)});
  const double d3 = std::max({dx, dy, dz});
  return (std::sqrt(3.0) - std::sqrt(2.0)) * d1 +
         (std::sqrt(2.0) - 1.0) * d2 + d3;
}

bool DynAStar::adjustStartEnd(
  Eigen::Vector3d & start, Eigen::Vector3d & goal,
  Eigen::Vector3i & s_idx, Eigen::Vector3i & g_idx) const
{
  if (!coordToIndex(start, s_idx) || !coordToIndex(goal, g_idx)) {
    return false;
  }

  if (checkOccupancy(indexToCoord(s_idx))) {
    for (int k = 0; k < 80; ++k) {
      start = (start - goal).normalized() * step_size_ + start;
      if (!coordToIndex(start, s_idx)) {
        return false;
      }
      if (!checkOccupancy(indexToCoord(s_idx))) {
        break;
      }
    }
    if (checkOccupancy(indexToCoord(s_idx))) {
      return false;
    }
  }

  if (checkOccupancy(indexToCoord(g_idx))) {
    for (int k = 0; k < 80; ++k) {
      goal = (goal - start).normalized() * step_size_ + goal;
      if (!coordToIndex(goal, g_idx)) {
        return false;
      }
      if (!checkOccupancy(indexToCoord(g_idx))) {
        break;
      }
    }
    if (checkOccupancy(indexToCoord(g_idx))) {
      return false;
    }
  }
  return true;
}

bool DynAStar::retrievePath(int end_addr)
{
  path_.clear();
  int cur = end_addr;
  const int max_len = pool_size_.x() * pool_size_.y() * pool_size_.z();
  for (int guard = 0; guard < max_len; ++guard) {
    path_.push_back(indexToCoord(nodes_[static_cast<size_t>(cur)].index));
    if (nodes_[static_cast<size_t>(cur)].parent < 0) {
      break;
    }
    cur = nodes_[static_cast<size_t>(cur)].parent;
  }
  std::reverse(path_.begin(), path_.end());
  return path_.size() >= 2;
}

bool DynAStar::search(
  const OccupancyGrid & grid,
  Eigen::Vector3d start, Eigen::Vector3d goal,
  std::vector<Eigen::Vector3d> & path,
  const AStarOptions & opt)
{
  path.clear();
  path_.clear();
  grid_ = &grid;
  opt_ = opt;

  start.z() = opt.cruise_z;
  goal.z() = opt.cruise_z;

  if (!grid.findFreeNearby(start, opt.free_snap_radius) ||
      !grid.findFreeNearby(goal, opt.free_snap_radius)) {
    return false;
  }
  start.z() = opt.cruise_z;
  goal.z() = opt.cruise_z;

  step_size_ = grid.resolution();
  inv_step_ = 1.0 / step_size_;
  center_ = 0.5 * (start + goal);

  const double span_xy = (start - goal).norm() + 6.0;
  const int half_xy = std::clamp(static_cast<int>(span_xy / step_size_) + 8, 24, 60);
  const int half_z = std::max(8, static_cast<int>(std::ceil(opt.z_band / step_size_)) + 4);
  initPool(Eigen::Vector3i(2 * half_xy + 1, 2 * half_xy + 1, 2 * half_z + 1));

  Eigen::Vector3i s_idx, g_idx;
  if (!adjustStartEnd(start, goal, s_idx, g_idx)) {
    return false;
  }

  ++round_;
  const auto t0 = std::chrono::steady_clock::now();

  const int start_addr = toAddress(s_idx);
  const int goal_addr = toAddress(g_idx);

  auto & start_node = nodes_[static_cast<size_t>(start_addr)];
  start_node.round = round_;
  start_node.index = s_idx;
  start_node.g = 0.0;
  start_node.f = kTieBreaker * diagHeu(s_idx, g_idx);
  start_node.parent = -1;
  start_node.state = GridNode::OPEN;

  const auto & goal_node = nodes_[static_cast<size_t>(goal_addr)];
  (void)goal_node;

  std::priority_queue<int, std::vector<int>, NodeCmp> open{NodeCmp{&nodes_}};
  open.push(start_addr);

  const double z_min = opt.cruise_z - opt.z_band;
  const double z_max = opt.cruise_z + opt.z_band;

  while (!open.empty()) {
    const int cur_addr = open.top();
    open.pop();
    auto & cur = nodes_[static_cast<size_t>(cur_addr)];
    if (cur.round != round_) {
      continue;
    }
    if (cur.state == GridNode::CLOSED) {
      continue;
    }
    cur.state = GridNode::CLOSED;

    if (cur_addr == goal_addr) {
      const bool ok = retrievePath(goal_addr);
      if (ok) {
        path = path_;
      }
      return ok;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          if (dz != 0 && dx == 0 && dy == 0) {
            continue;
          }

          const Eigen::Vector3i n_idx = cur.index + Eigen::Vector3i(dx, dy, dz);
          if (!indexInPool(n_idx)) {
            continue;
          }
          if (n_idx.x() <= 0 || n_idx.y() <= 0 || n_idx.z() <= 0 ||
              n_idx.x() >= pool_size_.x() - 1 || n_idx.y() >= pool_size_.y() - 1 ||
              n_idx.z() >= pool_size_.z() - 1)
          {
            continue;
          }

          const Eigen::Vector3d n_pos = indexToCoord(n_idx);
          if (n_pos.z() < z_min || n_pos.z() > z_max) {
            continue;
          }
          if (checkOccupancy(n_pos)) {
            continue;
          }

          double step_cost = std::sqrt(
            static_cast<double>(dx * dx + dy * dy + dz * dz)) * step_size_;
          if (dz != 0) {
            step_cost *= opt.vertical_cost_scale;
          }

          const int n_addr = toAddress(n_idx);
          auto & neighbor = nodes_[static_cast<size_t>(n_addr)];
          const bool discovered = neighbor.round != round_;
          if (!discovered && neighbor.state == GridNode::CLOSED) {
            continue;
          }

          const double tentative_g = cur.g + step_cost;
          if (discovered || tentative_g < neighbor.g) {
            neighbor.round = round_;
            neighbor.index = n_idx;
            neighbor.parent = cur_addr;
            neighbor.g = tentative_g;
            neighbor.f = tentative_g + kTieBreaker * diagHeu(n_idx, g_idx);
            neighbor.state = GridNode::OPEN;
            open.push(n_addr);
          }
        }
      }
    }

    const auto t1 = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(t1 - t0).count() > 0.25) {
      return false;
    }
  }
  return false;
}

}  // namespace drone_planner

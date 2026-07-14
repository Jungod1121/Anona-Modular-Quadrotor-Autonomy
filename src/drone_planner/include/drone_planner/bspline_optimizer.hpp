#pragma once
/**
 * ESDF-free B-spline control-point optimization with Rebound + L-BFGS.
 * Reference: ego-planner bspline_opt/bspline_optimizer.cpp
 *   - ControlPoints (base_point + direction), calcDistanceCostRebound,
 *   - check_collision_and_rebound, rebound_optimize (lbfgs.hpp)
 * Changes vs EGO: standalone (no plan_env / swarm); obstacle list instead of GridMap;
 *   rebound dirs from guide + 6-axis escape search (no nested A* in optimizer).
 */
#include "drone_planner/uniform_bspline.hpp"
#include "drone_planner/occupancy_grid.hpp"
#include "drone_planner/path_shortcut.hpp"
#include "lbfgs.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace drone_planner
{

struct OptWeights
{
  double lambda_smooth{1.0};
  double lambda_collision{10.0};
  double lambda_feasibility{0.1};
  double dist_threshold{0.5};
  double max_vel{2.0};
  double max_acc{2.0};
  double ts{0.2};
};

/// EGO-style rebound control-point metadata (see bspline_optimizer.h ControlPoints).
struct ControlPoints
{
  double clearance{0.5};
  int size{0};
  Eigen::MatrixXd points;  // 3 x N, each column is one control point
  std::vector<std::vector<Eigen::Vector3d>> base_point;
  std::vector<std::vector<Eigen::Vector3d>> direction;
  std::vector<bool> flag_temp;

  void resize(int n)
  {
    size = n;
    points.resize(3, n);
    base_point.assign(static_cast<size_t>(n), {});
    direction.assign(static_cast<size_t>(n), {});
    flag_temp.assign(static_cast<size_t>(n), false);
  }

  void clearDirections()
  {
    for (int i = 0; i < size; ++i) {
      base_point[static_cast<size_t>(i)].clear();
      direction[static_cast<size_t>(i)].clear();
      flag_temp[static_cast<size_t>(i)] = false;
    }
  }
};

class BsplineOptimizer
{
public:
  static constexpr int kOrder = 3;

  void setObstacles(const std::vector<Eigen::Vector3d> & pts) { obstacles_ = pts; }
  void setOccupancyGrid(const OccupancyGrid * grid) { grid_ = grid; }
  void setWeights(const OptWeights & w) { w_ = w; }

  bool optimize(
    const std::vector<Eigen::Vector3d> & guide_path,
    const Eigen::Vector3d & start_vel,
    std::vector<Eigen::Vector3d> & traj_out,
    Eigen::MatrixXd & ctrl_out)
  {
    traj_out.clear();
    if (guide_path.size() < 2) {
      return false;
    }

    guide_path_ = downsampleGuide(guide_path);

    cps_.clearance = w_.dist_threshold;

    std::vector<Eigen::Vector3d> start_end = {
      start_vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::MatrixXd ctrl;
    UniformBspline::parameterizeToBspline(w_.ts, guide_path_, start_end, ctrl);

    if (!ctrl.allFinite()) {
      return false;
    }

    const int N = static_cast<int>(ctrl.cols());
    if (N < 2 * kOrder + 1) {
      sampleTrajectory(ctrl, traj_out);
      ctrl_out = ctrl;
      return trajIsSane(traj_out) && trajectoryMeetsClearance(traj_out);
    }

    cps_.resize(N);
    cps_.points = ctrl;
    restart_count_ = 0;
    initReboundDirections();
    enrichReboundFromTrajectory();
    augmentReboundFromTrajectory();

    // Prefer rebound; if it fails, keep initial B-spline if still collision-free.
    // Dense maps often fail rebound — rejecting everything was worse than pre-P2 A*.
    if (reboundOptimize()) {
      sampleTrajectory(cps_.points, traj_out);
      ctrl_out = cps_.points;
      if (trajIsSane(traj_out) && trajectoryMeetsClearance(traj_out)) {
        return true;
      }
    }

    sampleTrajectory(ctrl, traj_out);
    ctrl_out = ctrl;
    if (trajIsSane(traj_out) && trajectoryMeetsClearance(traj_out)) {
      return true;
    }
    return false;
  }

  double minDistanceToObstacles(const std::vector<Eigen::Vector3d> & traj) const
  {
    double mind = std::numeric_limits<double>::infinity();
    if (traj.empty()) {
      return mind;
    }
    for (const auto & p : traj) {
      if (grid_ != nullptr) {
        if (grid_->isOccupied(p)) {
          return 0.0;
        }
        mind = std::min(mind, grid_->clearanceAt(p));
        continue;
      }
      if (obstacles_.empty()) {
        continue;
      }
      const int stride = std::max(1, static_cast<int>(obstacles_.size() / 5000));
      for (size_t k = 0; k < obstacles_.size(); k += static_cast<size_t>(stride)) {
        mind = std::min(mind, (p - obstacles_[k]).norm());
      }
    }
    return mind;
  }

  bool trajectoryMeetsClearance(const std::vector<Eigen::Vector3d> & traj) const
  {
    if (traj.empty()) {
      return false;
    }
    // A* plans in the *inflated* grid (planning margin). B-spline curves always
    // chord slightly — evaluating acceptance against inflate rejects almost every
    // dense-map spline. Accept if free vs RAW obstacles + soft body radius.
    const double body_r = std::max(0.25, cps_.clearance * 0.55);
    if (grid_ != nullptr) {
      for (size_t i = 0; i < traj.size(); ++i) {
        if (grid_->isOccupiedRaw(traj[i])) {
          return false;
        }
        if (i + 1 < traj.size() &&
          !segmentCollisionFreeRaw(*grid_, traj[i], traj[i + 1]))
        {
          return false;
        }
      }
    }
    if (!obstacles_.empty()) {
      return minDistanceToObstacleCloud(traj) >= body_r;
    }
    return true;
  }

  /** Min distance from traj samples to obstacle point cloud (not inflate). */
  double minDistanceToObstacleCloud(const std::vector<Eigen::Vector3d> & traj) const
  {
    double mind = std::numeric_limits<double>::infinity();
    if (traj.empty() || obstacles_.empty()) {
      return mind;
    }
    const int stride = std::max(1, static_cast<int>(obstacles_.size() / 8000));
    for (const auto & p : traj) {
      for (size_t k = 0; k < obstacles_.size(); k += static_cast<size_t>(stride)) {
        mind = std::min(mind, (p - obstacles_[k]).norm());
        if (mind < 1e-3) {
          return 0.0;
        }
      }
    }
    return mind;
  }

  static bool segmentCollisionFreeRaw(
    const OccupancyGrid & grid,
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & b)
  {
    const double len = (b - a).norm();
    if (len < 1e-6) {
      return !grid.isOccupiedRaw(a);
    }
    const int steps = std::max(2, static_cast<int>(std::ceil(len / grid.resolution())));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      if (grid.isOccupiedRaw(a + t * (b - a))) {
        return false;
      }
    }
    return true;
  }

private:
  static std::vector<Eigen::Vector3d> downsampleGuide(
    const std::vector<Eigen::Vector3d> & guide_path)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.push_back(guide_path.front());
    for (size_t i = 1; i < guide_path.size(); ++i) {
      if ((guide_path[i] - pts.back()).norm() > 0.35) {
        pts.push_back(guide_path[i]);
      }
    }
    if ((pts.back() - guide_path.back()).norm() > 1e-3) {
      pts.push_back(guide_path.back());
    }
    while (pts.size() < 5) {
      pts.push_back(pts.back());
    }
    if (pts.size() > 40) {
      std::vector<Eigen::Vector3d> thin;
      thin.push_back(pts.front());
      const double step = static_cast<double>(pts.size() - 1) / 38.0;
      for (int i = 1; i < 39; ++i) {
        thin.push_back(pts[static_cast<size_t>(i * step)]);
      }
      thin.push_back(pts.back());
      pts.swap(thin);
    }
    return pts;
  }

  static bool trajIsSane(const std::vector<Eigen::Vector3d> & traj)
  {
    if (traj.size() < 2) {
      return false;
    }
    double len = 0.0;
    for (size_t i = 0; i < traj.size(); ++i) {
      if (!traj[i].allFinite()) {
        return false;
      }
      if (i > 0) {
        len += (traj[i] - traj[i - 1]).norm();
        if (len > 200.0) {
          return false;
        }
      }
    }
    return true;
  }

  double nearestObstacleDist(
    const Eigen::Vector3d & p,
    Eigen::Vector3d * nearest_out = nullptr) const
  {
    if (grid_ != nullptr) {
      const double d = grid_->clearanceAt(p);
      if (nearest_out) {
        *nearest_out = p;
      }
      return d;
    }

    double best = std::numeric_limits<double>::infinity();
    Eigen::Vector3d nearest = p;
    if (obstacles_.empty()) {
      if (nearest_out) {
        *nearest_out = nearest;
      }
      return best;
    }
    const int stride = std::max(1, static_cast<int>(obstacles_.size() / 5000));
    for (size_t k = 0; k < obstacles_.size(); k += static_cast<size_t>(stride)) {
      const double d = (p - obstacles_[k]).norm();
      if (d < best) {
        best = d;
        nearest = obstacles_[k];
      }
    }
    if (nearest_out) {
      *nearest_out = nearest;
    }
    return best;
  }

  static Eigen::Vector3d closestPointOnPolyline(
    const std::vector<Eigen::Vector3d> & poly,
    const Eigen::Vector3d & p)
  {
    if (poly.empty()) {
      return p;
    }
    if (poly.size() == 1) {
      return poly.front();
    }
    Eigen::Vector3d best = poly.front();
    double best_d2 = (p - best).squaredNorm();
    for (size_t i = 0; i + 1 < poly.size(); ++i) {
      const Eigen::Vector3d a = poly[i];
      const Eigen::Vector3d b = poly[i + 1];
      const Eigen::Vector3d ab = b - a;
      const double ab2 = ab.squaredNorm();
      double t = 0.0;
      if (ab2 > 1e-12) {
        t = std::clamp((p - a).dot(ab) / ab2, 0.0, 1.0);
      }
      const Eigen::Vector3d q = a + t * ab;
      const double d2 = (p - q).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best = q;
      }
    }
    return best;
  }

  Eigen::Vector3d escapeDirection(const Eigen::Vector3d & p) const
  {
    const std::vector<Eigen::Vector3d> axes{
      Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ()};

    Eigen::Vector3d best = Eigen::Vector3d::UnitY();
    double best_clear = -1.0;
    for (const auto & axis : axes) {
      const double clear = nearestObstacleDist(p + axis * cps_.clearance);
      if (clear > best_clear) {
        best_clear = clear;
        best = axis;
      }
    }

    const Eigen::Vector3d guide_pt = closestPointOnPolyline(guide_path_, p);
    const Eigen::Vector3d along_guide = guide_pt - p;
    if (along_guide.norm() > 1e-4 &&
      nearestObstacleDist(p + along_guide.normalized() * cps_.clearance) > best_clear)
    {
      best = along_guide.normalized();
    }
    return best.normalized();
  }

  void addReboundDirection(int idx, const Eigen::Vector3d & base, const Eigen::Vector3d & dir_in)
  {
    if (idx < 0 || idx >= cps_.size) {
      return;
    }
    Eigen::Vector3d dir = dir_in;
    if (dir.norm() < 1e-6) {
      dir = Eigen::Vector3d::UnitX();
    } else {
      dir.normalize();
    }
    cps_.base_point[static_cast<size_t>(idx)].push_back(base);
    cps_.direction[static_cast<size_t>(idx)].push_back(dir);
    cps_.flag_temp[static_cast<size_t>(idx)] = true;
  }

  /// Initialize rebound base_point / direction from guide path (EGO initControlPoints simplified).
  void initReboundDirections()
  {
    cps_.clearDirections();
    if (obstacles_.empty()) {
      return;
    }

    const int end_idx = cps_.size - kOrder;
    const double res = 0.1;

    for (int i = kOrder; i < end_idx; ++i) {
      Eigen::Vector3d nearest;
      const Eigen::Vector3d P = cps_.points.col(i);
      const double dist = nearestObstacleDist(P, &nearest);
      if (dist >= cps_.clearance) {
        continue;
      }

      Eigen::Vector3d dir = escapeDirection(P);
      if (dir.norm() < 1e-6) {
        continue;
      }

      // March along escape ray to find obstacle-side base point.
      const double length = cps_.clearance * 2.0;
      Eigen::Vector3d base = nearest;
      for (double a = 0.0; a <= length; a += res) {
        const Eigen::Vector3d sample = P + dir * a;
        if (nearestObstacleDist(sample) >= cps_.clearance) {
          base = P + dir * std::max(0.0, a - res);
          break;
        }
      }

      addReboundDirection(i, base, dir);

      // Propagate direction to neighboring colliding control points (EGO step 3).
      for (int j = i + 1; j < end_idx; ++j) {
        if (cps_.flag_temp[static_cast<size_t>(j)]) {
          break;
        }
        if (nearestObstacleDist(cps_.points.col(j)) < cps_.clearance) {
          cps_.base_point[static_cast<size_t>(j)].push_back(
            cps_.base_point[static_cast<size_t>(i)].back());
          cps_.direction[static_cast<size_t>(j)].push_back(
            cps_.direction[static_cast<size_t>(i)].back());
        } else {
          break;
        }
      }
      for (int j = i - 1; j >= kOrder; --j) {
        if (cps_.flag_temp[static_cast<size_t>(j)]) {
          break;
        }
        if (nearestObstacleDist(cps_.points.col(j)) < cps_.clearance) {
          cps_.base_point[static_cast<size_t>(j)].push_back(
            cps_.base_point[static_cast<size_t>(i)].back());
          cps_.direction[static_cast<size_t>(j)].push_back(
            cps_.direction[static_cast<size_t>(i)].back());
        } else {
          break;
        }
      }
    }
  }

  /// Add rebound metadata for control points near trajectory samples in collision.
  void augmentReboundFromTrajectory()
  {
    if (obstacles_.empty()) {
      return;
    }
    std::vector<Eigen::Vector3d> traj;
    sampleTrajectory(cps_.points, traj);
    const int end_idx = cps_.size - kOrder;
    for (const auto & p : traj) {
      Eigen::Vector3d nearest;
      if (nearestObstacleDist(p, &nearest) >= cps_.clearance) {
        continue;
      }
      int best_i = kOrder;
      double best_d = std::numeric_limits<double>::infinity();
      for (int i = kOrder; i < end_idx; ++i) {
        const double d = (cps_.points.col(i) - p).squaredNorm();
        if (d < best_d) {
          best_d = d;
          best_i = i;
        }
      }
      Eigen::Vector3d dir = p - nearest;
      if (dir.norm() < 1e-4) {
        dir = cps_.points.col(best_i) - nearest;
      }
      if (dir.norm() < 1e-6) {
        continue;
      }
      addReboundDirection(best_i, nearest, dir.normalized());
    }
  }

  /// Add rebound dirs where the initial spline arc (not just CPs) penetrates obstacles.
  void enrichReboundFromTrajectory()
  {
    if (obstacles_.empty()) {
      return;
    }
    std::vector<Eigen::Vector3d> traj;
    sampleTrajectory(cps_.points, traj);
    const int end_idx = cps_.size - kOrder;
    for (const auto & p : traj) {
      Eigen::Vector3d nearest;
      if (nearestObstacleDist(p, &nearest) >= cps_.clearance) {
        continue;
      }
      int best_i = kOrder;
      double best_d2 = std::numeric_limits<double>::infinity();
      for (int i = kOrder; i < end_idx; ++i) {
        const double d2 = (cps_.points.col(i) - p).squaredNorm();
        if (d2 < best_d2) {
          best_d2 = d2;
          best_i = i;
        }
      }
      Eigen::Vector3d dir = escapeDirection(cps_.points.col(best_i));
      if (dir.norm() < 1e-6) {
        continue;
      }
      addReboundDirection(best_i, nearest, dir);
    }
  }

  /// Mid-optimization rebound refresh (EGO check_collision_and_rebound simplified).
  bool checkCollisionAndRebound()
  {
    if (obstacles_.empty()) {
      return false;
    }

    bool new_obs = false;
    const int end_idx = cps_.size - kOrder;
    const int i_end = end_idx - (end_idx - kOrder) / 3;

    for (int i = kOrder; i <= i_end; ++i) {
      Eigen::Vector3d nearest;
      const Eigen::Vector3d P = cps_.points.col(i);
      const double dist = nearestObstacleDist(P, &nearest);
      if (dist >= cps_.clearance) {
        continue;
      }

      bool already_pushed = false;
      for (size_t k = 0; k < cps_.direction[static_cast<size_t>(i)].size(); ++k) {
        const double d_along = (P - cps_.base_point[static_cast<size_t>(i)][k])
          .dot(cps_.direction[static_cast<size_t>(i)][k]);
        if (d_along < cps_.clearance * 0.8) {
          already_pushed = false;
          break;
        }
        already_pushed = true;
      }
      if (already_pushed) {
        continue;
      }

      new_obs = true;
      const Eigen::Vector3d dir = escapeDirection(P);
      if (dir.norm() < 1e-6) {
        continue;
      }
      addReboundDirection(i, nearest, dir);
    }
    return new_obs;
  }

  void sampleTrajectory(const Eigen::MatrixXd & ctrl, std::vector<Eigen::Vector3d> & traj) const
  {
    traj.clear();
    if (!ctrl.allFinite()) {
      return;
    }
    UniformBspline spline(ctrl, kOrder, w_.ts);
    double um, ump;
    if (!spline.getTimeSpan(um, ump)) {
      return;
    }
    for (double t = um; t <= ump; t += 0.05) {
      Eigen::VectorXd v = spline.evaluateDeBoor(t);
      if (!v.allFinite()) {
        traj.clear();
        return;
      }
      traj.emplace_back(v(0), v(1), v(2));
    }
  }

  bool trajectoryHasCollision(const Eigen::MatrixXd & ctrl) const
  {
    std::vector<Eigen::Vector3d> traj;
    sampleTrajectory(ctrl, traj);
    if (traj.empty()) {
      return true;
    }
    for (size_t i = 0; i < traj.size(); ++i) {
      if (grid_ != nullptr && grid_->isOccupied(traj[i])) {
        return true;
      }
      if (grid_ != nullptr && i + 1 < traj.size() &&
        !segmentCollisionFree(*grid_, traj[i], traj[i + 1]))
      {
        return true;
      }
    }
    if (grid_ == nullptr) {
      return minDistanceToObstacles(traj) < cps_.clearance * collision_accept_ratio_;
    }
    return false;
  }

  void calcSmoothness(
    const Eigen::MatrixXd & q, double & cost, Eigen::MatrixXd & grad) const
  {
    for (int i = 0; i < q.cols() - 3; ++i) {
      const Eigen::Vector3d jerk =
        q.col(i + 3) - 3.0 * q.col(i + 2) + 3.0 * q.col(i + 1) - q.col(i);
      cost += w_.lambda_smooth * jerk.squaredNorm();
      const Eigen::Vector3d g = 2.0 * w_.lambda_smooth * jerk;
      grad.col(i) -= g;
      grad.col(i + 1) += 3.0 * g;
      grad.col(i + 2) -= 3.0 * g;
      grad.col(i + 3) += g;
    }
  }

  /// EGO calcDistanceCostRebound — uses base_point + direction elastic band.
  void calcDistanceCostRebound(
    const Eigen::MatrixXd & q, double & cost, Eigen::MatrixXd & grad) const
  {
    if (obstacles_.empty()) {
      return;
    }

    const int end_idx = q.cols() - kOrder;
    const double demarcation = cps_.clearance;
    const double a = 3.0 * demarcation;
    const double b = -3.0 * demarcation * demarcation;
    const double c = demarcation * demarcation * demarcation;

    for (int i = kOrder; i < end_idx; ++i) {
      for (size_t j = 0; j < cps_.direction[static_cast<size_t>(i)].size(); ++j) {
        const double dist = (q.col(i) - cps_.base_point[static_cast<size_t>(i)][j])
          .dot(cps_.direction[static_cast<size_t>(i)][j]);
        const double dist_err = cps_.clearance - dist;
        const Eigen::Vector3d dist_grad = cps_.direction[static_cast<size_t>(i)][j];

        if (dist_err < 0) {
          continue;
        }
        if (dist_err < demarcation) {
          cost += dist_err * dist_err * dist_err;
          grad.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        } else {
          cost += a * dist_err * dist_err + b * dist_err + c;
          grad.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void calcFeasibility(const Eigen::MatrixXd & q, double & cost, Eigen::MatrixXd & grad) const
  {
    const double ts = w_.ts;
    const double ts2 = ts * ts;
    for (int i = 0; i < q.cols() - 1; ++i) {
      const Eigen::Vector3d v = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 3; ++j) {
        if (v(j) > w_.max_vel) {
          const double e = v(j) - w_.max_vel;
          cost += w_.lambda_feasibility * e * e;
          grad(j, i + 1) += 2.0 * w_.lambda_feasibility * e / ts;
          grad(j, i) -= 2.0 * w_.lambda_feasibility * e / ts;
        } else if (v(j) < -w_.max_vel) {
          const double e = -w_.max_vel - v(j);
          cost += w_.lambda_feasibility * e * e;
          grad(j, i + 1) -= 2.0 * w_.lambda_feasibility * e / ts;
          grad(j, i) += 2.0 * w_.lambda_feasibility * e / ts;
        }
      }
    }
    for (int i = 0; i < q.cols() - 2; ++i) {
      const Eigen::Vector3d a = (q.col(i + 2) - 2.0 * q.col(i + 1) + q.col(i)) / ts2;
      for (int j = 0; j < 3; ++j) {
        if (std::abs(a(j)) > w_.max_acc) {
          const double e = std::abs(a(j)) - w_.max_acc;
          const double sgn = a(j) > 0 ? 1.0 : -1.0;
          cost += w_.lambda_feasibility * e * e;
          const double g = 2.0 * w_.lambda_feasibility * e * sgn / ts2;
          grad(j, i + 2) += g;
          grad(j, i + 1) -= 2.0 * g;
          grad(j, i) += g;
        }
      }
    }
  }

  void combineCostRebound(const double * x, double * grad, double & f_combine, int n)
  {
    const int offset = 3 * kOrder;
    std::memcpy(cps_.points.data() + offset, x, static_cast<size_t>(n) * sizeof(x[0]));

    double f_smooth = 0.0;
    double f_dist = 0.0;
    double f_feas = 0.0;

    Eigen::MatrixXd g_smooth = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_dist = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feas = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothness(cps_.points, f_smooth, g_smooth);
    if (iter_num_ > 3 &&
      f_smooth / std::max(1, cps_.size - 2 * kOrder) < 0.1)
    {
      checkCollisionAndRebound();
    }
    calcDistanceCostRebound(cps_.points, f_dist, g_dist);
    calcFeasibility(cps_.points, f_feas, g_feas);

    f_combine = f_smooth + lambda_collision_ * f_dist + f_feas;
    const Eigen::MatrixXd g_total =
      g_smooth + lambda_collision_ * g_dist + g_feas;

    std::memcpy(grad, g_total.data() + offset, static_cast<size_t>(n) * sizeof(grad[0]));
    ++iter_num_;
  }

  static double costFunctionRebound(void * func_data, const double * x, double * grad, const int n)
  {
    auto * opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    double cost = 0.0;
    opt->combineCostRebound(x, grad, cost, n);
    return cost;
  }

  bool reboundOptimize()
  {
    const int N = cps_.size;
    const int start_id = kOrder;
    const int end_id = N - kOrder;  // exclusive — last 3 CPs fixed
    if (end_id <= start_id) {
      return true;
    }

    variable_num_ = 3 * (end_id - start_id);
  restart:
    iter_num_ = 0;
    lambda_collision_ = w_.lambda_collision;

    std::vector<double> q(static_cast<size_t>(variable_num_));
    std::memcpy(q.data(), cps_.points.data() + 3 * start_id, static_cast<size_t>(variable_num_) * sizeof(q[0]));

    lbfgs::lbfgs_parameter_t params;
    lbfgs::lbfgs_load_default_parameters(&params);
    params.mem_size = 16;
    params.max_iterations = 300;
    params.g_epsilon = 0.005;

    double final_cost = 0.0;
    const int result = lbfgs::lbfgs_optimize(
      variable_num_, q.data(), &final_cost, costFunctionRebound, nullptr, nullptr, this, &params);

    std::memcpy(
      cps_.points.data() + 3 * start_id, q.data(),
      static_cast<size_t>(variable_num_) * sizeof(q[0]));

    if (!cps_.points.allFinite()) {
      return false;
    }

    const bool lbfgs_ok = result == lbfgs::LBFGS_CONVERGENCE ||
      result == lbfgs::LBFGS_STOP ||
      result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
      result == lbfgs::LBFGSERR_MAXIMUMITERATION;

    if (!lbfgs_ok) {
      return false;
    }

    if (trajectoryHasCollision(cps_.points)) {
      if (restart_count_ < kMaxRestarts) {
        ++restart_count_;
        lambda_collision_ = w_.lambda_collision * std::pow(3.0, restart_count_);
        initReboundDirections();
        enrichReboundFromTrajectory();
        goto restart;
      }
      std::vector<Eigen::Vector3d> traj;
      sampleTrajectory(cps_.points, traj);
      if (trajectoryMeetsClearance(traj)) {
        restart_count_ = 0;
        return true;
      }
      return false;
    }

    restart_count_ = 0;
    return true;
  }

  std::vector<Eigen::Vector3d> obstacles_;
  const OccupancyGrid * grid_{nullptr};
  std::vector<Eigen::Vector3d> guide_path_;
  OptWeights w_;
  static constexpr double collision_accept_ratio_{0.80};
  ControlPoints cps_;

  int iter_num_{0};
  int variable_num_{0};
  int restart_count_{0};
  double lambda_collision_{10.0};

  static constexpr int kMaxRestarts = 5;
};

}  // namespace drone_planner

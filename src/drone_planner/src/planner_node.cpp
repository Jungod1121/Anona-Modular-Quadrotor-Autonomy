/**
 * Planner FSM node.
 * FSM states inspired by ego-planner plan_manage/ego_replan_fsm
 * (INIT/WAIT_TARGET/GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/EMERGENCY_STOP),
 * rewritten for drone_ws topic contract — NOT a wrapper of ego launch.
 *
 * Topics in:  /drone/goal, /drone/odom, /map/obstacles, /map/local_obstacles
 * Topics out: /planner/trajectory, /planner/local_goal, /planner/trajectory_cmd,
 *             /planner/status
 */
#include "drone_planner/bspline_optimizer.hpp"
#include "drone_planner/dyn_astar.hpp"
#include "drone_planner/grid_astar.hpp"
#include "drone_planner/path_shortcut.hpp"
#include "drone_planner/occupancy_grid.hpp"
#include "drone_planner/uniform_bspline.hpp"

#include <drone_msgs/msg/planner_status.hpp>
#include <drone_msgs/msg/trajectory_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace drone_planner
{

enum class State
{
  INIT,
  WAIT_TARGET,
  GEN_NEW_TRAJ,
  EXEC_TRAJ,
  REPLAN_TRAJ,
  EMERGENCY_STOP,
  FAIL
};

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("drone_planner")
  {
    declare_parameter("namespace", "");
    declare_parameter("resolution", 0.25);
    declare_parameter("inflate_radius", 0.4);
    declare_parameter("map_origin_x", 0.0);
    declare_parameter("map_origin_y", 0.0);
    declare_parameter("map_origin_z", 0.0);
    declare_parameter("map_size_x", 20.0);
    declare_parameter("map_size_y", 12.0);
    declare_parameter("map_size_z", 3.0);
    // General-purpose mode: fit AABB + inflate from the cloud (any catalog map).
    declare_parameter("auto_map_fit", true);
    declare_parameter("auto_map_margin", 2.5);
    declare_parameter("auto_inflate", true);
    declare_parameter("auto_inflate_min", 0.15);
    declare_parameter("auto_inflate_max", 0.40);
    declare_parameter("auto_map_max_cells", 2500000.0);
    declare_parameter("replan_dist", 1.5);
    declare_parameter("goal_tolerance", 0.35);
    declare_parameter("local_goal_lookahead", 0.6);
    declare_parameter("control_rate", 20.0);
    declare_parameter("max_vel", 1.5);
    declare_parameter("max_acc", 2.0);
    declare_parameter("dist_threshold", 0.5);
    declare_parameter("bspline_clearance_auto", true);
    declare_parameter("bspline_ts", 0.25);
    declare_parameter("cruise_z", 1.5);
    declare_parameter("z_band", 2.0);
    declare_parameter("vertical_cost_scale", 1.25);
    declare_parameter("true_3d_astar", true);
    declare_parameter("free_snap_radius", 8);
    declare_parameter("seal_boundary_layers", 2);
    declare_parameter("collision_horizon", 1.5);
    declare_parameter("emergency_clearance", 0.12);
    declare_parameter("replan_clearance", 0.25);
    declare_parameter("emergency_recovery_sec", 1.0);
    declare_parameter("safety_grace_sec", 1.0);
    declare_parameter("replan_cooldown_sec", 2.0);
    declare_parameter("execution_safety_enable", false);
    declare_parameter("use_trajectory_cmd", false);
    declare_parameter("yaw_lookahead", 0.6);
    declare_parameter("use_dyn_astar", false);
    declare_parameter("enable_bspline_opt", true);
    declare_parameter("local_mapping_enable", false);
    declare_parameter("local_mapping_raycast_clear", false);
    declare_parameter("min_replan_interval_sec", 0.45);
    // Shared-field multi-drone: absolute map topic + peer namespaces (comma-separated).
    declare_parameter("map_topic", "");
    declare_parameter("peer_namespaces", "");
    declare_parameter("peer_radius", 0.75);
    declare_parameter("peer_replan_dist", 1.8);

    const std::string ns = get_parameter("namespace").as_string();
    prefix_ = ns.empty() ? "" : ("/" + ns);

    OptWeights w;
    w.max_vel = get_parameter("max_vel").as_double();
    w.max_acc = get_parameter("max_acc").as_double();
    const double inflate = get_parameter("inflate_radius").as_double();
    w.dist_threshold = get_parameter("dist_threshold").as_double();
    if (get_parameter("bspline_clearance_auto").as_bool()) {
      w.dist_threshold = std::max(0.28, inflate * 0.80);
    }
    w.ts = get_parameter("bspline_ts").as_double();
    w.lambda_collision = 14.0;
    optimizer_.setWeights(w);

    active_resolution_ = get_parameter("resolution").as_double();
    active_inflate_ = get_parameter("inflate_radius").as_double();
    active_origin_ = Eigen::Vector3d(
      get_parameter("map_origin_x").as_double(),
      get_parameter("map_origin_y").as_double(),
      get_parameter("map_origin_z").as_double());
    active_size_ = Eigen::Vector3d(
      get_parameter("map_size_x").as_double(),
      get_parameter("map_size_y").as_double(),
      get_parameter("map_size_z").as_double());
    grid_.setParams(active_resolution_, active_inflate_, active_origin_, active_size_);

    traj_pub_ = create_publisher<nav_msgs::msg::Path>(
      prefix_ + "/planner/trajectory",
      rclcpp::QoS(1).transient_local().reliable());
    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      prefix_ + "/planner/local_goal", 10);
    traj_cmd_pub_ = create_publisher<drone_msgs::msg::TrajectoryCommand>(
      prefix_ + "/planner/trajectory_cmd", 10);
    status_pub_ = create_publisher<drone_msgs::msg::PlannerStatus>(
      prefix_ + "/planner/status", 10);

    // Separate callback groups so map ingest cannot starve goal/odom/timer.
    map_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions control_opts;
    control_opts.callback_group = control_cb_group_;
    rclcpp::SubscriptionOptions map_opts;
    map_opts.callback_group = map_cb_group_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      prefix_ + "/drone/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        odom_ = *msg;
        have_odom_ = true;
      },
      control_opts);

    auto goal_qos = rclcpp::QoS(10);  // volatile — compatible with RViz 2D Goal Pose
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix_ + "/drone/goal", goal_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        geometry_msgs::msg::PoseStamped g = *msg;
        // RViz 2D Goal publishes z=0 on the ground plane — use cruise altitude.
        if (g.pose.position.z < 0.5) {
          g.pose.position.z = get_parameter("cruise_z").as_double();
        }
        const Eigen::Vector3d np(
          g.pose.position.x, g.pose.position.y, g.pose.position.z);
        if (have_goal_) {
          const Eigen::Vector3d op(
            goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);
          // Formation coordinator streams goals — only replan on meaningful moves.
          if ((np - op).norm() < 0.2) {
            goal_ = g;
            return;
          }
        }
        goal_ = g;
        have_goal_ = true;
        need_replan_ = true;
        RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f, %.2f)",
          goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);
      },
      control_opts);

    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      mapTopic(), rclcpp::QoS(1).transient_local().reliable(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ingestMap(*msg);
      },
      map_opts);

    if (get_parameter("local_mapping_enable").as_bool()) {
      local_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        prefix_ + "/map/local_obstacles", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          ingestLocalMap(*msg);
        },
        map_opts);
    }

    setupPeerSubscriptions(control_opts);

    const double hz = get_parameter("control_rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&PlannerNode::onTimer, this),
      control_cb_group_);

    state_ = State::INIT;
    publishStatus("INIT", true, "booting");
    RCLCPP_INFO(get_logger(),
      "drone_planner ready (DynAStar=%s, Bspline=%s, local_mapping=%s, map=%s, peers=%s)",
      get_parameter("use_dyn_astar").as_bool() ? "on" : "off",
      get_parameter("enable_bspline_opt").as_bool() ? "on" : "off",
      get_parameter("local_mapping_enable").as_bool() ? "on" : "off",
      mapTopic().c_str(),
      get_parameter("peer_namespaces").as_string().c_str());
  }

private:
  std::string mapTopic() const
  {
    const std::string custom = get_parameter("map_topic").as_string();
    if (!custom.empty()) {
      return custom;
    }
    return prefix_ + "/map/obstacles";
  }

  void setupPeerSubscriptions(const rclcpp::SubscriptionOptions & opts)
  {
    const std::string raw = get_parameter("peer_namespaces").as_string();
    if (raw.empty()) {
      return;
    }
    std::stringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ',')) {
      // trim
      const auto a = token.find_first_not_of(" \t");
      const auto b = token.find_last_not_of(" \t");
      if (a == std::string::npos) {
        continue;
      }
      token = token.substr(a, b - a + 1);
      if (token.empty() || token == get_parameter("namespace").as_string()) {
        continue;
      }
      const std::string topic = "/" + token + "/drone/odom";
      const std::string peer_ns = token;
      peer_subs_.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          topic, 10,
          [this, peer_ns](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(mtx_);
            peer_pos_[peer_ns] = Eigen::Vector3d(
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z);
            peer_stamp_[peer_ns] = now();
          },
          opts));
      RCLCPP_INFO(get_logger(), "Peer avoidance listening on %s", topic.c_str());
    }
  }

  void stampPeerBodies(OccupancyGrid & grid) const
  {
    const double r = get_parameter("peer_radius").as_double();
    const double res = std::max(0.2, get_parameter("resolution").as_double());
    std::unordered_map<std::string, Eigen::Vector3d> peers;
    std::unordered_map<std::string, rclcpp::Time> stamps;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      peers = peer_pos_;
      stamps = peer_stamp_;
    }
    const auto now_t = now();
    const int n = std::max(1, static_cast<int>(std::ceil(r / res)));
    for (const auto & kv : peers) {
      const auto it = stamps.find(kv.first);
      if (it == stamps.end() || (now_t - it->second).seconds() > 1.0) {
        continue;
      }
      const Eigen::Vector3d & c = kv.second;
      for (int ix = -n; ix <= n; ++ix) {
        for (int iy = -n; iy <= n; ++iy) {
          if (ix * ix + iy * iy > n * n) {
            continue;
          }
          grid.addPoint(Eigen::Vector3d(
            c.x() + ix * res, c.y() + iy * res, c.z()));
        }
      }
    }
  }

  /** Overlay live peers on a copy of the static base map (cheap). */
  void rebuildPlanningGridLocked()
  {
    RCLCPP_INFO(get_logger(), "rebuild: cloning base map");
    OccupancyGrid g;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_map_) {
        return;
      }
      g = base_grid_.clone();
    }
    RCLCPP_INFO(get_logger(), "rebuild: stamping peers");
    stampPeerBodies(g);
    RCLCPP_INFO(get_logger(), "rebuild: swapping grid");
    {
      std::lock_guard<std::mutex> lk(mtx_);
      grid_ = std::move(g);
      optimizer_.setOccupancyGrid(&grid_);
    }
  }

  /** Requires mtx_ already held (called from onTimer). */
  bool peersThreatenPathLocked(const Eigen::Vector3d & self) const
  {
    if (traj_.empty() || peer_pos_.empty()) {
      return false;
    }
    const double thr = get_parameter("peer_replan_dist").as_double();
    const auto now_t = now();
    for (const auto & kv : peer_pos_) {
      const auto it = peer_stamp_.find(kv.first);
      if (it == peer_stamp_.end() || (now_t - it->second).seconds() > 1.0) {
        continue;
      }
      if ((kv.second - self).norm() > thr * 2.5) {
        continue;
      }
      for (size_t i = traj_idx_; i < traj_.size(); i += 2) {
        if ((traj_[i] - kv.second).head<2>().norm() < thr) {
          return true;
        }
      }
    }
    return false;
  }
  /** Estimate inflate from XY obstacle spacing (thin for mazes, thicker for sparse). */
  static double estimateInflate(
    const std::vector<Eigen::Vector3d> & pts,
    double inflate_min, double inflate_max, double resolution)
  {
    const double body_floor = std::max(inflate_min, std::max(0.18, 0.9 * resolution));
    if (pts.size() < 16) {
      return std::clamp(0.25, body_floor, inflate_max);
    }
    const size_t stride = std::max<size_t>(1, pts.size() / 400);
    std::vector<double> nn;
    nn.reserve(400);
    for (size_t i = 0; i < pts.size(); i += stride) {
      double best = 1e9;
      for (size_t j = 0; j < pts.size(); j += stride) {
        if (i == j) {
          continue;
        }
        const double dx = pts[i].x() - pts[j].x();
        const double dy = pts[i].y() - pts[j].y();
        const double d = std::hypot(dx, dy);
        if (d > 1e-3 && d < best) {
          best = d;
        }
      }
      if (best < 1e8) {
        nn.push_back(best);
      }
    }
    if (nn.empty()) {
      return std::clamp(0.25, body_floor, inflate_max);
    }
    std::nth_element(nn.begin(), nn.begin() + nn.size() / 4, nn.end());
    const double gap = nn[nn.size() / 4];
    // Dense wall surfaces have tiny NN gaps — that is NOT a navigable slit.
    // Floor inflate so thin gate walls still occupy planning cell centers.
    if (gap < 0.20) {
      return std::clamp(body_floor, inflate_min, inflate_max);
    }
    return std::clamp(0.30 * gap, body_floor, inflate_max);
  }

  void ingestMap(const sensor_msgs::msg::PointCloud2 & msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(msg, cloud);

    // Ignore empty clouds (e.g. leftover transient_local latch from sparse hover map,
    // or late empty rediscovery) so we never wipe a good dense map.
    if (cloud.empty()) {
      if (have_map_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Ignoring empty /map/obstacles update (keeping %zu points)", obstacles_.size());
      }
      return;
    }

    double resolution = get_parameter("resolution").as_double();
    double inflate = get_parameter("inflate_radius").as_double();
    Eigen::Vector3d origin(
      get_parameter("map_origin_x").as_double(),
      get_parameter("map_origin_y").as_double(),
      get_parameter("map_origin_z").as_double());
    Eigen::Vector3d size(
      get_parameter("map_size_x").as_double(),
      get_parameter("map_size_y").as_double(),
      get_parameter("map_size_z").as_double());

    // First pass: collect finite points for AABB / spacing stats.
    std::vector<Eigen::Vector3d> raw_pts;
    raw_pts.reserve(cloud.size());
    for (const auto & pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      raw_pts.emplace_back(pt.x, pt.y, pt.z);
    }
    if (raw_pts.empty()) {
      return;
    }

    const bool auto_fit = get_parameter("auto_map_fit").as_bool();
    if (auto_fit) {
      Eigen::Vector3d pmin = raw_pts.front();
      Eigen::Vector3d pmax = raw_pts.front();
      for (const auto & p : raw_pts) {
        pmin = pmin.cwiseMin(p);
        pmax = pmax.cwiseMax(p);
      }
      // Include current odom / goal so forest starts outside the cloud still fit.
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (have_odom_) {
          const auto & p = odom_.pose.pose.position;
          pmin = pmin.cwiseMin(Eigen::Vector3d(p.x, p.y, p.z));
          pmax = pmax.cwiseMax(Eigen::Vector3d(p.x, p.y, p.z));
        }
        if (have_goal_) {
          const auto & p = goal_.pose.position;
          pmin = pmin.cwiseMin(Eigen::Vector3d(p.x, p.y, std::max(0.0, p.z)));
          pmax = pmax.cwiseMax(Eigen::Vector3d(p.x, p.y, std::max(0.5, p.z)));
        }
      }
      const double margin = get_parameter("auto_map_margin").as_double();
      const double cruise = get_parameter("cruise_z").as_double();
      origin = Eigen::Vector3d(
        pmin.x() - margin,
        pmin.y() - margin,
        std::min(0.0, pmin.z() - 0.2));
      size = Eigen::Vector3d(
        (pmax.x() - pmin.x()) + 2.0 * margin,
        (pmax.y() - pmin.y()) + 2.0 * margin,
        std::max(pmax.z() - origin.z() + 0.5, cruise + 1.5));
      size.x() = std::max(size.x(), 8.0);
      size.y() = std::max(size.y(), 8.0);
      size.z() = std::max(size.z(), 2.5);

      // Cap memory: coarsen resolution if the AABB would explode.
      const double max_cells = get_parameter("auto_map_max_cells").as_double();
      for (int k = 0; k < 8; ++k) {
        const double cells =
          std::ceil(size.x() / resolution) *
          std::ceil(size.y() / resolution) *
          std::ceil(size.z() / resolution);
        if (cells <= max_cells) {
          break;
        }
        resolution = std::min(0.6, resolution * 1.25);
      }
    }

    if (get_parameter("auto_inflate").as_bool()) {
      inflate = estimateInflate(
        raw_pts,
        get_parameter("auto_inflate_min").as_double(),
        get_parameter("auto_inflate_max").as_double(),
        resolution);
    }

    // Build off the hot path: downsample to ~grid resolution before inflate.
    const double voxel = std::max(0.15, resolution);
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(raw_pts.size() / 4 + 8);
    std::unordered_map<int64_t, bool> seen;
    seen.reserve(raw_pts.size() / 4 + 8);
    auto vox_key = [voxel](double x, double y, double z) -> int64_t {
      const int ix = static_cast<int>(std::floor(x / voxel));
      const int iy = static_cast<int>(std::floor(y / voxel));
      const int iz = static_cast<int>(std::floor(z / voxel));
      return (static_cast<int64_t>(ix) << 42) ^
             (static_cast<int64_t>(iy) << 21) ^
             static_cast<int64_t>(iz);
    };

    OccupancyGrid new_grid;
    new_grid.setParams(resolution, inflate, origin, size);

    for (const auto & p : raw_pts) {
      const int64_t k = vox_key(p.x(), p.y(), p.z());
      if (seen.count(k)) {
        continue;
      }
      seen[k] = true;
      pts.push_back(p);
      new_grid.addPoint(p);
    }
    new_grid.sealBoundary(std::max(0, static_cast<int>(get_parameter("seal_boundary_layers").as_int())));
    new_grid.rebuildInflateLayer();
    OccupancyGrid base_copy = new_grid.clone();

    {
      std::lock_guard<std::mutex> lk(mtx_);
      // Skip identical maps — map_node republishes every few seconds for latching.
      if (have_map_ && pts.size() == obstacles_.size() && pts.size() == last_map_voxels_ &&
          (active_origin_ - origin).norm() < 1e-3 &&
          (active_size_ - size).norm() < 1e-3 &&
          std::abs(active_inflate_ - inflate) < 1e-3)
      {
        return;
      }
      active_origin_ = origin;
      active_size_ = size;
      active_inflate_ = inflate;
      active_resolution_ = resolution;
      grid_ = std::move(new_grid);
      obstacles_ = std::move(pts);
      last_map_voxels_ = obstacles_.size();
      base_grid_ = std::move(base_copy);
      optimizer_.setObstacles(obstacles_);
      optimizer_.setOccupancyGrid(&grid_);
      have_map_ = true;
      need_replan_ = true;
    }
    RCLCPP_INFO(
      get_logger(),
      "Map ingested: %zu raw -> %zu voxels | origin=(%.1f,%.1f,%.1f) size=(%.1f,%.1f,%.1f) "
      "res=%.2f inflate=%.2f%s%s",
      cloud.size(), last_map_voxels_,
      origin.x(), origin.y(), origin.z(),
      size.x(), size.y(), size.z(),
      resolution, inflate,
      auto_fit ? " [auto_fit]" : "",
      get_parameter("auto_inflate").as_bool() ? " [auto_inflate]" : "");
  }

  void ingestLocalMap(const sensor_msgs::msg::PointCloud2 & msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(msg, cloud);
    if (cloud.empty()) {
      return;
    }

    Eigen::Vector3d sensor;
  {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_odom_ || !have_map_) {
        return;
      }
      sensor = Eigen::Vector3d(
        odom_.pose.pose.position.x,
        odom_.pose.pose.position.y,
        odom_.pose.pose.position.z);
    }

    std::vector<Eigen::Vector3d> hits;
    hits.reserve(cloud.size());
    for (const auto & pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      hits.emplace_back(pt.x, pt.y, pt.z);
    }
    if (hits.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    const bool clear_free = get_parameter("local_mapping_raycast_clear").as_bool();
    grid_.integrateLocalCloud(sensor, hits, clear_free);
    const double min_interval = get_parameter("min_replan_interval_sec").as_double();
    if ((now() - last_plan_time_).seconds() >= min_interval) {
      need_replan_ = true;
    }
  }

  void onTimer()
  {
    std::unique_lock<std::mutex> lk(mtx_);
    if (!have_odom_) {
      state_ = State::INIT;
      publishTrajectoryCmd(false);
      publishStatus("INIT", true, "waiting odom");
      return;
    }

    const Eigen::Vector3d pos(
      odom_.pose.pose.position.x,
      odom_.pose.pose.position.y,
      odom_.pose.pose.position.z);

    if (!have_goal_) {
      state_ = State::WAIT_TARGET;
      publishLocalGoal(pos);
      publishTrajectoryCmd(false);
      publishStatus("WAIT_TARGET", true, "waiting goal");
      return;
    }

    if (state_ == State::EMERGENCY_STOP) {
      publishLocalGoal(pos);
      publishTrajectoryCmd(false);
      const double recovery = get_parameter("emergency_recovery_sec").as_double();
      if ((now() - emergency_since_).seconds() >= recovery) {
        need_replan_ = true;
        state_ = State::REPLAN_TRAJ;
        RCLCPP_WARN(get_logger(), "Emergency recovery — attempting replan");
      } else {
        publishStatus("EMERGENCY_STOP", false, "holding hover after collision");
        return;
      }
    }

    const Eigen::Vector3d goal_p(
      goal_.pose.position.x,
      goal_.pose.position.y,
      goal_.pose.position.z);
    const double goal_tol = get_parameter("goal_tolerance").as_double();

    if ((pos - goal_p).norm() < goal_tol) {
      publishLocalGoal(goal_p);
      publishTrajectoryCmd(false);
      publishStatus("EXEC_TRAJ", true, "reached goal");
      return;
    }

    // Execution safety (EGO-style): do NOT use inflated occupancy for body checks —
    // inflate is a planning margin and falsely triggers EMERGENCY at start/near walls.
    // Only hard-stop on true near-contact; soft blockage ahead triggers throttled replan.
    if (get_parameter("execution_safety_enable").as_bool() &&
      state_ == State::EXEC_TRAJ && have_map_ && !traj_.empty() &&
      (now() - traj_start_).seconds() >= get_parameter("safety_grace_sec").as_double())
    {
      const double emergency_clear = get_parameter("emergency_clearance").as_double();
      const double body_dist = optimizer_.minDistanceToObstacles({pos});
      if (std::isfinite(body_dist) && body_dist < emergency_clear) {
        state_ = State::EMERGENCY_STOP;
        emergency_since_ = now();
        traj_.clear();
        has_bspline_traj_ = false;
        traj_idx_ = 0;
        publishLocalGoal(pos);
        publishTrajectoryCmd(false);
        RCLCPP_ERROR(get_logger(),
          "EMERGENCY_STOP — body dist to obstacle=%.3f m", body_dist);
        publishStatus("EMERGENCY_STOP", false, "body collision risk");
        return;
      }

      const double replan_clear = get_parameter("replan_clearance").as_double();
      const double replan_cooldown = get_parameter("replan_cooldown_sec").as_double();
      if ((now() - last_safety_replan_).seconds() >= replan_cooldown &&
        checkTrajectoryCollisionAhead(replan_clear))
      {
        need_replan_ = true;
        last_safety_replan_ = now();
        state_ = State::REPLAN_TRAJ;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "REPLAN_TRAJ — trajectory blocked ahead");
      }
    }

    const double replan_dist = get_parameter("replan_dist").as_double();
    bool near_end = false;
    if (!traj_.empty()) {
      near_end = (pos - traj_.back()).norm() < replan_dist &&
                 (traj_.back() - goal_p).norm() > goal_tol;
      while (traj_idx_ + 1 < traj_.size() &&
             (traj_[traj_idx_] - pos).norm() > (traj_[traj_idx_ + 1] - pos).norm()) {
        ++traj_idx_;
      }
    }

    if (need_replan_ || traj_.empty() || near_end ||
      peersThreatenPathLocked(pos) ||
      state_ == State::REPLAN_TRAJ || state_ == State::GEN_NEW_TRAJ)
    {
      const double min_interval = get_parameter("min_replan_interval_sec").as_double();
      const bool forced = traj_.empty() || state_ == State::REPLAN_TRAJ ||
        state_ == State::GEN_NEW_TRAJ;
      if (!forced && (now() - last_plan_time_).seconds() < min_interval) {
        // Throttle replans — keep tracking current trajectory.
      } else {
        state_ = State::GEN_NEW_TRAJ;
        // Keep controller on the path while A* runs (do not let it chase /drone/goal).
        if (!traj_.empty()) {
          publishLocalGoal(traj_[std::min(traj_idx_, traj_.size() - 1)]);
        } else {
          publishLocalGoal(pos);
        }
        publishTrajectoryCmd(false);
        lk.unlock();
        bool ok = false;
        {
          std::lock_guard<std::mutex> plan_lk(mtx_);
          ok = plan(pos, goal_p);
        }
        lk.lock();
        if (!ok) {
          state_ = State::FAIL;
          publishLocalGoal(pos);
          publishTrajectoryCmd(false);
          publishStatus("FAIL", false, "A*/optimize failed — holding hover");
          need_replan_ = false;
          return;
        }
        last_plan_time_ = now();
        need_replan_ = false;
        state_ = State::EXEC_TRAJ;
      }
    }

    if (traj_.empty()) {
      publishLocalGoal(pos);
      publishTrajectoryCmd(false);
      publishStatus("WAIT_TARGET", false, "no trajectory");
      return;
    }

    Eigen::Vector3d cmd_p = goal_p;
    Eigen::Vector3d cmd_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d cmd_a = Eigen::Vector3d::Zero();
    double cmd_yaw = have_cmd_yaw_ ? cmd_yaw_ : 0.0;
    double cmd_yaw_dot = 0.0;
    bool publish_traj_cmd = false;

    // Default: position lookahead along path (pre-P2 behaviour, stable on A* polylines).
    const double look = get_parameter("local_goal_lookahead").as_double();
    double acc_len = 0.0;
    cmd_p = traj_[traj_idx_];
    for (size_t i = traj_idx_; i + 1 < traj_.size(); ++i) {
      const double seg_len = (traj_[i + 1] - traj_[i]).norm();
      acc_len += seg_len;
      cmd_p = traj_[i + 1];
      // On long shortcut segments, never aim more than one segment ahead —
      // otherwise the controller chords through obstacles (blue ≠ yellow).
      if (acc_len >= look || seg_len > look * 1.5) {
        break;
      }
    }
    // Keep planned altitude in true-3D mode (do not flatten to cruise_z).
    if (!get_parameter("true_3d_astar").as_bool()) {
      cmd_p.z() = get_parameter("cruise_z").as_double();
    }
    // Soft lateral push so tracking also stays clear of peers between replans.
    // mtx_ already held by onTimer — do not try_lock again.
    {
      const double r = get_parameter("peer_radius").as_double();
      const auto now_t = now();
      for (const auto & kv : peer_pos_) {
        const auto it = peer_stamp_.find(kv.first);
        if (it == peer_stamp_.end() || (now_t - it->second).seconds() > 1.0) {
          continue;
        }
        Eigen::Vector2d d(cmd_p.x() - kv.second.x(), cmd_p.y() - kv.second.y());
        const double n = d.norm();
        if (n < 1e-3) {
          d = Eigen::Vector2d(0.0, 1.0);
        } else {
          d /= n;
        }
        if (n < r * 1.4) {
          const double push = (r * 1.4 - n);
          cmd_p.x() += d.x() * push;
          cmd_p.y() += d.y() * push;
        }
      }
    }

    // Cap commanded leap so cascade PID cannot tumble the craft (TF NaN root cause).
    {
      Eigen::Vector3d leap = cmd_p - pos;
      leap.z() = 0.0;
      const double leap_n = leap.norm();
      constexpr double kMaxLeap = 0.55;
      if (leap_n > kMaxLeap) {
        cmd_p.x() = pos.x() + leap.x() * (kMaxLeap / leap_n);
        cmd_p.y() = pos.y() + leap.y() * (kMaxLeap / leap_n);
      }
    }

    if (traj_idx_ + 1 < traj_.size()) {
      const Eigen::Vector3d tangent = traj_[traj_idx_ + 1] - traj_[traj_idx_];
      if (tangent.head<2>().norm() > 0.2) {
        const double yaw_tgt = std::atan2(tangent.y(), tangent.x());
        const double hz = get_parameter("control_rate").as_double();
        double dy = yaw_tgt - cmd_yaw;
        while (dy > M_PI) {
          dy -= 2.0 * M_PI;
        }
        while (dy < -M_PI) {
          dy += 2.0 * M_PI;
        }
        cmd_yaw_dot = std::clamp(dy * hz, -0.8, 0.8);
        cmd_yaw = cmd_yaw + cmd_yaw_dot / hz;
        cmd_yaw_ = cmd_yaw;
        have_cmd_yaw_ = true;
      }
    }

    // B-spline shapes the yellow path; local_goal lookahead above tracks it.
    // Do not open-loop traj_cmd FF — lag → FF races ahead → blue cuts through obstacles.
    publish_traj_cmd = false;

    const double ox = active_origin_.x();
    const double oy = active_origin_.y();
    const double sx = active_size_.x();
    const double sy = active_size_.y();
    const double margin = 0.5;
    cmd_p.x() = std::clamp(cmd_p.x(), ox + margin, ox + sx - margin);
    cmd_p.y() = std::clamp(cmd_p.y(), oy + margin, oy + sy - margin);
    if (!get_parameter("true_3d_astar").as_bool()) {
      cmd_p.z() = get_parameter("cruise_z").as_double();
    }

    publishLocalGoal(cmd_p);
    publishTrajectoryCmd(publish_traj_cmd, cmd_p, cmd_v, cmd_a, cmd_yaw, cmd_yaw_dot);
    publishTrajectory();
    const double mind = optimizer_.minDistanceToObstacles(traj_);
    publishStatus(state_ == State::REPLAN_TRAJ ? "REPLAN_TRAJ" : "EXEC_TRAJ",
      true, "tracking", pathLength(traj_), mind);
  }

  bool plan(const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
  {
    if (!have_map_) {
      // Never invent a ballistic start→goal polyline — that flies through every
      // obstacle once the map is late to arrive (reported on Path A/C dense maps).
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No map yet — refusing straight-line plan");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Planning (%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f)",
      start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());

    AStarOptions opt;
    opt.cruise_z = get_parameter("cruise_z").as_double();
    // Prefer goal altitude if set; otherwise parameter default.
    if (std::isfinite(goal.z()) && goal.z() > 0.2) {
      opt.cruise_z = goal.z();
    }
    opt.z_band = get_parameter("z_band").as_double();
    opt.vertical_cost_scale = get_parameter("vertical_cost_scale").as_double();
    opt.true_3d = get_parameter("true_3d_astar").as_bool();
    opt.peer_radius = get_parameter("peer_radius").as_double();
    int snap = std::max(4, static_cast<int>(get_parameter("free_snap_radius").as_int()));
    if (get_parameter("auto_map_fit").as_bool()) {
      // Larger maps / mazes: allow snapping out of walls farther.
      snap = std::max(snap, static_cast<int>(std::ceil(2.5 / std::max(0.1, active_resolution_))));
    }
    opt.free_snap_radius = snap;
    // NOTE: plan() is only called from onTimer while mtx_ is already held.
    // Do not lock again (std::mutex is non-recursive → permanent deadlock, then
    // controller falls back to /drone/goal and flies straight through obstacles).
    {
      const auto now_t = now();
      for (const auto & kv : peer_pos_) {
        const auto it = peer_stamp_.find(kv.first);
        if (it != peer_stamp_.end() && (now_t - it->second).seconds() <= 1.0) {
          opt.peer_centers.push_back(kv.second);
        }
      }
    }
    RCLCPP_INFO(get_logger(), "Running A* with %zu peer keep-outs (true_3d=%s)",
      opt.peer_centers.size(), opt.true_3d ? "yes" : "no");

    Eigen::Vector3d s = start;
    Eigen::Vector3d g = goal;
    if (!opt.true_3d) {
      s.z() = opt.cruise_z;
      g.z() = opt.cruise_z;
    }

    std::vector<Eigen::Vector3d> guide;
    const bool use_dyn = get_parameter("use_dyn_astar").as_bool();
    bool found = false;
    if (use_dyn) {
      DynAStar dyn;
      found = dyn.search(grid_, s, g, guide, opt);
      if (!found) {
        RCLCPP_WARN(get_logger(),
          "DynA* failed (true_3d=%s, band=±%.2f) — fallback GridA*",
          opt.true_3d ? "yes" : "no", opt.z_band);
        GridAStar astar;
        found = astar.search(grid_, s, g, guide, opt);
      }
    } else {
      GridAStar astar;
      found = astar.search(grid_, s, g, guide, opt);
    }
    RCLCPP_INFO(get_logger(), "A* finished found=%s guide=%zu",
      found ? "yes" : "no", guide.size());
    if (!found) {
      RCLCPP_WARN(get_logger(),
        "A* failed (true_3d=%s, band=±%.2f) — no free 3D route",
        opt.true_3d ? "yes" : "no", opt.z_band);
      return false;
    }

    // Legacy horizontal mode flattens; true 3D keeps vertical waypoints.
    if (!opt.true_3d) {
      for (auto & p : guide) {
        p.z() = opt.cruise_z;
      }
    }
    // Dense A* polyline for tracking. Optional B-spline disabled by default
    // (see ego_avoidance.launch.py for official EGO-Planner packaging).
    const auto dense_guide = densifyPath(guide, active_resolution_);
    const auto free_guide = densifyPath(
      shortcutPath(grid_, guide),
      std::max(0.35, active_resolution_));

    Eigen::Vector3d vel(
      odom_.twist.twist.linear.x,
      odom_.twist.twist.linear.y,
      0.0);
    Eigen::MatrixXd ctrl;
    std::vector<Eigen::Vector3d> opt_traj;
    const bool try_bspline = get_parameter("enable_bspline_opt").as_bool();
    if (try_bspline && optimizer_.optimize(free_guide, vel, opt_traj, ctrl)) {
      if (!opt.true_3d) {
        for (auto & p : opt_traj) {
          p.z() = opt.cruise_z;
        }
      }
      traj_ = densifyPath(opt_traj, std::min(0.25, get_parameter("resolution").as_double()));
      bspline_ctrl_ = ctrl;
      bspline_ts_ = get_parameter("bspline_ts").as_double();
      has_bspline_traj_ = true;
      traj_duration_ = pathLength(traj_) / std::max(0.2, get_parameter("max_vel").as_double());
      const double min_clear = optimizer_.minDistanceToObstacles(traj_);
      RCLCPP_INFO(get_logger(),
        "B-spline smoothed: %zu samples, min_clear=%.3f m",
        traj_.size(), min_clear);
    } else {
      if (try_bspline) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "B-spline rejected — dense Grid A* polyline (still safe)");
      }
      traj_ = dense_guide;
      has_bspline_traj_ = false;
      traj_duration_ = pathLength(traj_) / std::max(0.1, get_parameter("max_vel").as_double());
    }

    traj_start_ = now();
    traj_idx_ = 0;
    const double plen = pathLength(traj_);
    if (!std::isfinite(plen) || plen > 100.0 || traj_.size() < 2) {
      RCLCPP_ERROR(get_logger(),
        "Invalid path (len=%.3f, n=%zu) — holding hover", plen, traj_.size());
      traj_.clear();
      return false;
    }

    if (!has_bspline_traj_) {
      traj_duration_ = plen / std::max(0.2, get_parameter("max_vel").as_double());
    }
    RCLCPP_INFO(get_logger(), "Planned path: %zu waypoints, length=%.2f m (horizontal avoidance)",
      traj_.size(), plen);
    return true;
  }

  bool sampleTrajectoryAtTime(
    double t, Eigen::Vector3d & p, Eigen::Vector3d & v, Eigen::Vector3d & a) const
  {
    if (traj_.empty()) {
      return false;
    }
    const double t_clamp = std::clamp(t, 0.0, std::max(1e-3, traj_duration_));

    if (has_bspline_traj_ && bspline_ctrl_.cols() >= 6) {
      const double ts = get_parameter("bspline_ts").as_double();
      UniformBspline pos_spline(bspline_ctrl_, 3, ts);
      UniformBspline vel_spline = pos_spline.getDerivative();
      UniformBspline acc_spline = vel_spline.getDerivative();
      const Eigen::VectorXd pv = pos_spline.evaluateDeBoorT(t_clamp);
      const Eigen::VectorXd vv = vel_spline.evaluateDeBoorT(t_clamp);
      const Eigen::VectorXd av = acc_spline.evaluateDeBoorT(t_clamp);
      if (!pv.allFinite() || !vv.allFinite() || !av.allFinite()) {
        return false;
      }
      p = pv.head<3>();
      v = vv.head<3>();
      a = av.head<3>();
      return true;
    }

    const double total = pathLength(traj_);
    if (total < 1e-6) {
      p = traj_.front();
      v.setZero();
      a.setZero();
      return true;
    }
    const double s_target = (t_clamp / std::max(1e-3, traj_duration_)) * total;
    double s_acc = 0.0;
    p = traj_.front();
    Eigen::Vector3d tangent = Eigen::Vector3d::UnitX();
    for (size_t i = 1; i < traj_.size(); ++i) {
      const Eigen::Vector3d seg = traj_[i] - traj_[i - 1];
      const double seg_len = seg.norm();
      if (s_acc + seg_len >= s_target || i + 1 == traj_.size()) {
        const double alpha = seg_len > 1e-6 ?
          std::clamp((s_target - s_acc) / seg_len, 0.0, 1.0) : 0.0;
        p = traj_[i - 1] + alpha * seg;
        tangent = seg_len > 1e-6 ? seg / seg_len : tangent;
        break;
      }
      s_acc += seg_len;
    }
    const double speed = total / std::max(1e-3, traj_duration_);
    v = tangent * speed;
    a.setZero();
    return true;
  }

  void computeYaw(
    const Eigen::Vector3d & pos, const Eigen::Vector3d & vel, double t,
    double & yaw, double & yaw_dot) const
  {
    const double lookahead = get_parameter("yaw_lookahead").as_double();
    Eigen::Vector3d p2 = pos;
    Eigen::Vector3d v2 = vel;
    sampleTrajectoryAtTime(t + lookahead, p2, v2, v2);
    Eigen::Vector3d dir = p2 - pos;
    if (dir.norm() < 0.15) {
      dir = vel;
    }
    if (dir.norm() < 0.1) {
      yaw = have_cmd_yaw_ ? cmd_yaw_ : 0.0;
      yaw_dot = 0.0;
      return;
    }
    const double yaw_new = std::atan2(dir.y(), dir.x());
    yaw_dot = 0.0;
    if (have_cmd_yaw_) {
      double dy = yaw_new - cmd_yaw_;
      while (dy > M_PI) {
        dy -= 2.0 * M_PI;
      }
      while (dy < -M_PI) {
        dy += 2.0 * M_PI;
      }
      const double max_rate = 1.5;
      const double hz = get_parameter("control_rate").as_double();
      dy = std::clamp(dy, -max_rate / hz, max_rate / hz);
      yaw = cmd_yaw_ + dy;
      yaw_dot = dy * hz;
    } else {
      yaw = yaw_new;
    }
  }

  bool checkTrajectoryCollisionAhead(double clearance) const
  {
    const double horizon = get_parameter("collision_horizon").as_double();
    const double t_now = (now() - traj_start_).seconds();
    // Skip the near-body segment — inflated planning margin makes the first
    // half-meter look "colliding" even on a valid free path.
    const double t_start = t_now + 0.4;
    const double dt = 0.15;
    for (double t = t_start; t <= t_now + horizon; t += dt) {
      Eigen::Vector3d p, v, a;
      if (!sampleTrajectoryAtTime(t, p, v, a)) {
        continue;
      }
      const double d = optimizer_.minDistanceToObstacles({p});
      if (std::isfinite(d) && d < clearance) {
        return true;
      }
    }
    return false;
  }

  static double pathLength(const std::vector<Eigen::Vector3d> & p)
  {
    double L = 0;
    for (size_t i = 1; i < p.size(); ++i) {
      L += (p[i] - p[i - 1]).norm();
    }
    return L;
  }

  void publishTrajectoryCmd(
    bool ready,
    const Eigen::Vector3d & p = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & v = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & a = Eigen::Vector3d::Zero(),
    double yaw = 0.0,
    double yaw_dot = 0.0)
  {
    drone_msgs::msg::TrajectoryCommand msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.trajectory_ready = ready;
    msg.position.x = p.x();
    msg.position.y = p.y();
    msg.position.z = p.z();
    msg.velocity.x = v.x();
    msg.velocity.y = v.y();
    msg.velocity.z = v.z();
    msg.acceleration.x = a.x();
    msg.acceleration.y = a.y();
    msg.acceleration.z = a.z();
    msg.yaw = yaw;
    msg.yaw_dot = yaw_dot;
    traj_cmd_pub_->publish(msg);
  }

  void publishLocalGoal(const Eigen::Vector3d & p)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = p.x();
    msg.pose.position.y = p.y();
    msg.pose.position.z = p.z();
    msg.pose.orientation.w = 1.0;
    local_goal_pub_->publish(msg);
  }

  void publishTrajectory()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const auto & p : traj_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    traj_pub_->publish(path);
  }

  void publishStatus(const std::string & state, bool ok, const std::string & msg,
                     double plen = 0.0, double mind = 0.0)
  {
    drone_msgs::msg::PlannerStatus st;
    st.header.stamp = now();
    st.header.frame_id = "map";
    st.state = state;
    st.success = ok;
    st.message = msg;
    st.path_length = plen;
    st.min_obstacle_distance = mind;
    status_pub_->publish(st);
  }

  std::string prefix_;
  OccupancyGrid grid_;
  OccupancyGrid base_grid_;
  BsplineOptimizer optimizer_;
  State state_{State::INIT};
  Eigen::Vector3d active_origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d active_size_{Eigen::Vector3d(20, 12, 3)};
  double active_inflate_{0.4};
  double active_resolution_{0.25};

  mutable std::mutex mtx_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::PoseStamped goal_;
  bool have_odom_{false}, have_goal_{false}, have_map_{false}, need_replan_{false};
  std::vector<Eigen::Vector3d> obstacles_;
  std::vector<Eigen::Vector3d> traj_;
  size_t traj_idx_{0};
  size_t last_map_voxels_{0};
  std::unordered_map<std::string, Eigen::Vector3d> peer_pos_;
  std::unordered_map<std::string, rclcpp::Time> peer_stamp_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_subs_;

  bool has_bspline_traj_{false};
  Eigen::MatrixXd bspline_ctrl_;
  double bspline_ts_{0.25};
  double traj_duration_{0.0};
  rclcpp::Time traj_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time emergency_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_safety_replan_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  double cmd_yaw_{0.0};
  bool have_cmd_yaw_{false};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<drone_msgs::msg::TrajectoryCommand>::SharedPtr traj_cmd_pub_;
  rclcpp::Publisher<drone_msgs::msg::PlannerStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr map_cb_group_;
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
};

}  // namespace drone_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded so long map ingest does not drop /drone/goal callbacks.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<drone_planner::PlannerNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

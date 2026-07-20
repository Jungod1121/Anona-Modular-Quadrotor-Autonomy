/**
 * Path E — MIGHTY-inspired Hermite spline planner for drone_ws plant contract.
 * Front-end: inflated voxel A*; back-end: cubic Hermite samples → TrajectoryCommand.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <drone_msgs/msg/trajectory_command.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

struct Key {
  int x{0}, y{0};
  bool operator==(const Key &o) const { return x == o.x && y == o.y; }
};

struct KeyHash {
  size_t operator()(const Key &k) const {
    return (static_cast<size_t>(k.x) * 73856093u) ^ (static_cast<size_t>(k.y) * 19349663u);
  }
};

Eigen::Vector3d hermite(const Eigen::Vector3d &p0, const Eigen::Vector3d &m0,
                        const Eigen::Vector3d &p1, const Eigen::Vector3d &m1, double t) {
  const double t2 = t * t, t3 = t2 * t;
  return (2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * m0 +
         (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * m1;
}

}  // namespace

class MightyPlanningNode : public rclcpp::Node {
public:
  MightyPlanningNode() : Node("mighty_planning_node") {
    declare_parameter("MapTopic", std::string("/map_generator/global_cloud"));
    declare_parameter("TargetTopic", std::string("/drone/goal"));
    declare_parameter("OdomTopic", std::string("/drone/odom"));
    declare_parameter("CruiseHeight", 1.0);
    declare_parameter("MaxVel", 1.5);
    declare_parameter("DilateRadius", 0.35);
    declare_parameter("VoxelWidth", 0.25);
    declare_parameter("MapBound", std::vector<double>{-18, 18, -12, 12, 0, 4});

    map_topic_ = get_parameter("MapTopic").as_string();
    max_vel_ = get_parameter("MaxVel").as_double();
    dilate_ = get_parameter("DilateRadius").as_double();
    voxel_ = get_parameter("VoxelWidth").as_double();
    cruise_z_ = get_parameter("CruiseHeight").as_double();
    map_bound_ = get_parameter("MapBound").as_double_array();

    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/planner/local_goal", 10);
    traj_cmd_pub_ = create_publisher<drone_msgs::msg::TrajectoryCommand>("/planner/trajectory_cmd", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/planner/trajectory", rclcpp::QoS(1).transient_local().reliable());

    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MightyPlanningNode::onMap, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("OdomTopic").as_string(), 50,
      std::bind(&MightyPlanningNode::onOdom, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("TargetTopic").as_string(), 10,
      std::bind(&MightyPlanningNode::onGoal, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&MightyPlanningNode::onTick, this));

    RCLCPP_INFO(get_logger(),
      "Path E MIGHTY-adapter ready: map=%s → /planner/*", map_topic_.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
    have_odom_ = true;
  }

  void onMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    occ_.clear();
    if (msg->data.empty()) return;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
      const int dilate_cells = std::max(1, static_cast<int>(std::ceil(dilate_ / voxel_)));
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (*iz < map_bound_[4] - 0.5 || *iz > map_bound_[5] + 0.5) continue;
        const Key c{static_cast<int>(std::floor(*ix / voxel_)),
                    static_cast<int>(std::floor(*iy / voxel_))};
        for (int dx = -dilate_cells; dx <= dilate_cells; ++dx) {
          for (int dy = -dilate_cells; dy <= dilate_cells; ++dy) {
            occ_.insert(Key{c.x + dx, c.y + dy});
          }
        }
      }
      map_ok_ = true;
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "map parse: %s", e.what());
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!have_odom_ || !map_ok_) {
      RCLCPP_WARN(get_logger(), "Goal ignored — waiting for odom/map");
      return;
    }
    goal_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (goal_.z() < 0.5) goal_.z() = cruise_z_;
    plan();
  }

  Key toKey(const Eigen::Vector3d &p) const {
    return Key{static_cast<int>(std::floor(p.x() / voxel_)),
               static_cast<int>(std::floor(p.y() / voxel_))};
  }

  Eigen::Vector3d fromKey(const Key &k, double z) const {
    return Eigen::Vector3d((k.x + 0.5) * voxel_, (k.y + 0.5) * voxel_, z);
  }

  bool isFree(const Key &k) const { return occ_.find(k) == occ_.end(); }

  std::vector<Eigen::Vector3d> astar(const Eigen::Vector3d &s, const Eigen::Vector3d &g) {
    const Key sk = toKey(s), gk = toKey(g);
    struct Node {
      double g{0}, f{0};
      Key parent{};
      bool has_parent{false};
    };
    auto heur = [&](const Key &a) {
      return voxel_ * (std::abs(a.x - gk.x) + std::abs(a.y - gk.y));
    };
    struct QN {
      Key k;
      double f;
      bool operator>(const QN &o) const { return f > o.f; }
    };
    std::priority_queue<QN, std::vector<QN>, std::greater<QN>> open;
    std::unordered_map<Key, Node, KeyHash> nodes;
    nodes[sk] = Node{0, heur(sk), {}, false};
    open.push(QN{sk, nodes[sk].f});
    const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    bool found = false;
    Key cur{};
    for (int iter = 0; iter < 200000 && !open.empty(); ++iter) {
      cur = open.top().k;
      open.pop();
      if (cur == gk) {
        found = true;
        break;
      }
      const Node &cn = nodes[cur];
      for (const auto &d : dirs) {
        Key nk{cur.x + d[0], cur.y + d[1]};
        if (!isFree(nk) && !(nk == gk)) continue;
        const double step = (d[0] && d[1]) ? voxel_ * 1.414 : voxel_;
        const double ng = cn.g + step;
        auto it = nodes.find(nk);
        if (it == nodes.end() || ng < it->second.g) {
          nodes[nk] = Node{ng, ng + heur(nk), cur, true};
          open.push(QN{nk, nodes[nk].f});
        }
      }
    }
    std::vector<Eigen::Vector3d> path;
    if (!found) return path;
    Key k = gk;
    while (true) {
      path.push_back(fromKey(k, 0.5 * (s.z() + g.z())));
      auto it = nodes.find(k);
      if (it == nodes.end() || !it->second.has_parent) break;
      k = it->second.parent;
    }
    std::reverse(path.begin(), path.end());
    if (!path.empty()) {
      path.front() = s;
      path.back() = g;
    }
    return path;
  }

  void plan() {
    auto waypoints = astar(odom_, goal_);
    if (waypoints.size() < 2) {
      RCLCPP_WARN(get_logger(), "Path E A* failed");
      return;
    }
    std::vector<Eigen::Vector3d> samples;
    samples.push_back(waypoints.front());
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      const Eigen::Vector3d &p0 = waypoints[i];
      const Eigen::Vector3d &p1 = waypoints[i + 1];
      Eigen::Vector3d m0 = (p1 - p0);
      if (i > 0) m0 = 0.5 * (p1 - waypoints[i - 1]);
      Eigen::Vector3d m1 = (p1 - p0);
      if (i + 2 < waypoints.size()) m1 = 0.5 * (waypoints[i + 2] - p0);
      const int n = std::max(2, static_cast<int>(std::ceil((p1 - p0).norm() / 0.25)));
      for (int k = 1; k <= n; ++k) {
        samples.push_back(hermite(p0, m0, p1, m1, static_cast<double>(k) / n));
      }
    }
    traj_ = std::move(samples);
    traj_i_ = 0;
    last_advance_ = this->now().seconds();

    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Time(this->get_clock()->now());
    path.header.frame_id = "map";
    for (const auto &p : traj_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Path E planned %zu Hermite samples", traj_.size());
  }

  void onTick() {
    if (traj_.empty() || traj_i_ >= traj_.size()) return;
    const Eigen::Vector3d &p = traj_[traj_i_];
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (traj_i_ + 1 < traj_.size()) {
      const Eigen::Vector3d d = traj_[traj_i_ + 1] - p;
      if (d.norm() > 1e-3) v = d.normalized() * max_vel_;
    }

    geometry_msgs::msg::PoseStamped local;
    local.header.stamp = rclcpp::Time(this->get_clock()->now());
    local.header.frame_id = "map";
    local.pose.position.x = p.x();
    local.pose.position.y = p.y();
    local.pose.position.z = p.z();
    local.pose.orientation.w = 1.0;
    local_goal_pub_->publish(local);

    drone_msgs::msg::TrajectoryCommand tc;
    tc.header = local.header;
    tc.position.x = p.x();
    tc.position.y = p.y();
    tc.position.z = p.z();
    tc.velocity.x = v.x();
    tc.velocity.y = v.y();
    tc.velocity.z = v.z();
    tc.yaw = (v.head<2>().norm() > 1e-3) ? std::atan2(v.y(), v.x()) : 0.0;
    traj_cmd_pub_->publish(tc);

    if ((odom_ - p).head<2>().norm() < 0.4 || (this->now().seconds() - last_advance_) > 0.35) {
      ++traj_i_;
      last_advance_ = this->now().seconds();
    }
  }

  std::string map_topic_;
  double max_vel_{1.5}, dilate_{0.35}, voxel_{0.25}, cruise_z_{1.0};
  std::vector<double> map_bound_;
  std::unordered_set<Key, KeyHash> occ_;
  bool map_ok_{false}, have_odom_{false};
  Eigen::Vector3d odom_{Eigen::Vector3d::Zero()}, goal_{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> traj_;
  size_t traj_i_{0};
  double last_advance_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<drone_msgs::msg::TrajectoryCommand>::SharedPtr traj_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MightyPlanningNode>());
  rclcpp::shutdown();
  return 0;
}

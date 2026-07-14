#include "drone_map/map_generator.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace drone_map
{

/// ROS2 map node.
/// Publishing pattern (latched global PointCloud2 + optional voxel downsample) references
/// MARSIM/pengyu_sim map_generator; obstacle positions are self-developed fixed-seed
/// procedural generation — not loaded from external PCD files.
class MapNode : public rclcpp::Node
{
public:
  MapNode()
  : Node("drone_map")
  {
    declareAndLoadParams();

    const std::string prefix = topicPrefix();

    // Latched global map — same idea as map_generator global_cloud with latch=true.
    auto latch_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      prefix + "/map/obstacles", latch_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      prefix + "/map/obstacles_markers", latch_qos);
    local_obstacles_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      prefix + "/map/local_obstacles", 10);

    generateAndPublishMap();

    // Reinforce latched global map infrequently (planner ignores identical updates).
    republish_timer_ = create_wall_timer(
      10s, [this]() {
        if (map_result_.cloud.empty()) {
          return;
        }
        global_cloud_msg_.header.stamp = now();
        obstacles_pub_->publish(global_cloud_msg_);
      });

    if (local_sense_radius_ > 0.0) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        prefix + "/drone/odom", 20,
        std::bind(&MapNode::onOdom, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(),
      "drone_map ready: mode=%s seed=%d attempt=%d connected=%s points=%zu "
      "(native cylinders+walls for dense/narrow; ego_* modes optional)",
      map_mode_str_.c_str(), map_result_.seed, map_result_.attempt,
      map_result_.connected ? "yes" : "no", map_result_.cloud.points.size());
  }

private:
  void declareAndLoadParams()
  {
    declare_parameter("namespace", "");
    declare_parameter("map_mode", "sparse");
    declare_parameter("seed", 42);
    declare_parameter("max_attempts", 200);

    declare_parameter("start_x", 0.0);
    declare_parameter("start_y", 0.0);
    declare_parameter("start_z", 1.5);
    declare_parameter("goal_x", 2.0);
    declare_parameter("goal_y", 0.0);
    declare_parameter("goal_z", 1.5);

    declare_parameter("safety_distance", 0.4);
    declare_parameter("clearance_radius", 1.2);
    declare_parameter("min_obstacle_spacing", 0.9);
    declare_parameter("min_obstacle_radius", 0.12);
    declare_parameter("max_obstacle_radius", 0.3);

    declare_parameter("point_resolution", 0.08);
    declare_parameter("downsample_voxel", 0.0);
    declare_parameter("grid_resolution", 0.2);
    declare_parameter("corridor_gap_width", 1.5);
    declare_parameter("ego_road_width", 0.6);
    declare_parameter("ego_resolution", 0.1);
    declare_parameter("ego_obs_num", 100);
    declare_parameter("ego_circle_num", 40);
    declare_parameter("ego_min_distance", 0.8);

    declare_parameter("local_sense_radius", 5.0);
    declare_parameter("local_sense_rate", 10.0);

    declare_parameter("obstacle_marker_r", 0.9);
    declare_parameter("obstacle_marker_g", 0.3);
    declare_parameter("obstacle_marker_b", 0.2);
    declare_parameter("boundary_marker_r", 0.2);
    declare_parameter("boundary_marker_g", 0.4);
    declare_parameter("boundary_marker_b", 1.0);

    map_mode_str_ = get_parameter("map_mode").as_string();
    local_sense_radius_ = get_parameter("local_sense_radius").as_double();
    const double local_rate = get_parameter("local_sense_rate").as_double();

    MapConfig cfg;
    cfg.mode = parseMapMode(map_mode_str_);
    cfg.seed = get_parameter("seed").as_int();
    cfg.max_attempts = get_parameter("max_attempts").as_int();
    cfg.start_x = get_parameter("start_x").as_double();
    cfg.start_y = get_parameter("start_y").as_double();
    cfg.start_z = get_parameter("start_z").as_double();
    cfg.goal_x = get_parameter("goal_x").as_double();
    cfg.goal_y = get_parameter("goal_y").as_double();
    cfg.goal_z = get_parameter("goal_z").as_double();
    cfg.safety_distance = get_parameter("safety_distance").as_double();
    cfg.clearance_radius = get_parameter("clearance_radius").as_double();
    cfg.min_obstacle_spacing = get_parameter("min_obstacle_spacing").as_double();
    cfg.min_obstacle_radius = get_parameter("min_obstacle_radius").as_double();
    cfg.max_obstacle_radius = get_parameter("max_obstacle_radius").as_double();
    cfg.point_resolution = get_parameter("point_resolution").as_double();
    cfg.downsample_voxel = get_parameter("downsample_voxel").as_double();
    cfg.grid_resolution = get_parameter("grid_resolution").as_double();
    cfg.corridor_gap_width = get_parameter("corridor_gap_width").as_double();
    cfg.ego_road_width = get_parameter("ego_road_width").as_double();
    cfg.ego_resolution = get_parameter("ego_resolution").as_double();
    cfg.ego_obs_num = get_parameter("ego_obs_num").as_int();
    cfg.ego_circle_num = get_parameter("ego_circle_num").as_int();
    cfg.ego_min_distance = get_parameter("ego_min_distance").as_double();

    generator_ = std::make_unique<MapGenerator>(cfg);

    if (local_sense_radius_ > 0.0 && local_rate > 0.0) {
      local_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / local_rate),
        std::bind(&MapNode::publishLocalObstacles, this));
    }
  }

  std::string topicPrefix() const
  {
    const std::string ns = get_parameter("namespace").as_string();
    return ns.empty() ? "" : ("/" + ns);
  }

  void generateAndPublishMap()
  {
    map_result_ = generator_->generate();

    RCLCPP_INFO(get_logger(),
      "Map generated: mode=%s seed=%d attempt=%d connected=%s obstacles=%zu points=%zu "
      "downsample_voxel=%.3f",
      map_mode_str_.c_str(),
      map_result_.seed,
      map_result_.attempt,
      map_result_.connected ? "true" : "false",
      map_result_.obstacles.size(),
      map_result_.cloud.points.size(),
      get_parameter("downsample_voxel").as_double());

    if (!map_result_.connected &&
      (map_mode_str_ == "dense_field" || map_mode_str_ == "narrow_corridor"))
    {
      RCLCPP_WARN(get_logger(),
        "Connectivity check failed after %d attempts (seed=%d). Publishing best effort map.",
        map_result_.attempt + 1, map_result_.seed);
    }

    pcl::toROSMsg(map_result_.cloud, global_cloud_msg_);
    global_cloud_msg_.header.frame_id = "map";
    global_cloud_msg_.header.stamp = now();
    obstacles_pub_->publish(global_cloud_msg_);
    markers_pub_->publish(buildMarkers());
  }

  visualization_msgs::msg::MarkerArray buildMarkers() const
  {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;

    for (const auto & obs : map_result_.obstacles) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = now();
      marker.id = id++;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obs.center.x();
      marker.pose.position.y = obs.center.y();
      marker.pose.position.z = obs.center.z();
      marker.pose.orientation.w = 1.0;
      marker.ns = obs.is_boundary ? "boundary" : "obstacles";
      if (obs.is_boundary) {
        marker.color.r = static_cast<float>(get_parameter("boundary_marker_r").as_double());
        marker.color.g = static_cast<float>(get_parameter("boundary_marker_g").as_double());
        marker.color.b = static_cast<float>(get_parameter("boundary_marker_b").as_double());
      } else {
        marker.color.r = static_cast<float>(get_parameter("obstacle_marker_r").as_double());
        marker.color.g = static_cast<float>(get_parameter("obstacle_marker_g").as_double());
        marker.color.b = static_cast<float>(get_parameter("obstacle_marker_b").as_double());
      }
      marker.color.a = 0.85f;

      switch (obs.shape) {
        case Obstacle::Shape::CYLINDER:
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          marker.scale.x = 2.0 * obs.radius;
          marker.scale.y = 2.0 * obs.radius;
          marker.scale.z = obs.height;
          break;
        case Obstacle::Shape::SPHERE:
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.scale.x = 2.0 * obs.radius;
          marker.scale.y = 2.0 * obs.radius;
          marker.scale.z = 2.0 * obs.radius;
          break;
        case Obstacle::Shape::WALL:
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.scale.x = obs.length;
          marker.scale.y = obs.thickness;
          marker.scale.z = obs.height;
          break;
      }
      array.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = now();
    start_marker.ns = "start_goal";
    start_marker.id = id++;
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.pose.position.x = get_parameter("start_x").as_double();
    start_marker.pose.position.y = get_parameter("start_y").as_double();
    start_marker.pose.position.z = get_parameter("start_z").as_double();
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = 0.3;
    start_marker.scale.y = 0.3;
    start_marker.scale.z = 0.3;
    start_marker.color.r = 0.2f;
    start_marker.color.g = 0.9f;
    start_marker.color.b = 0.2f;
    start_marker.color.a = 1.0f;
    array.markers.push_back(start_marker);

    visualization_msgs::msg::Marker goal_marker = start_marker;
    goal_marker.id = id++;
    goal_marker.pose.position.x = get_parameter("goal_x").as_double();
    goal_marker.pose.position.y = get_parameter("goal_y").as_double();
    goal_marker.pose.position.z = get_parameter("goal_z").as_double();
    goal_marker.color.r = 0.2f;
    goal_marker.color.g = 0.4f;
    goal_marker.color.b = 1.0f;
    array.markers.push_back(goal_marker);

    return array;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void publishLocalObstacles()
  {
    if (!latest_odom_ || map_result_.cloud.empty()) {
      return;
    }

    const double px = latest_odom_->pose.pose.position.x;
    const double py = latest_odom_->pose.pose.position.y;
    const double pz = latest_odom_->pose.pose.position.z;
    const double r2 = local_sense_radius_ * local_sense_radius_;

    pcl::PointCloud<pcl::PointXYZ> local;
    local.points.reserve(map_result_.cloud.points.size());
    for (const auto & pt : map_result_.cloud.points) {
      const double dx = pt.x - px;
      const double dy = pt.y - py;
      const double dz = pt.z - pz;
      if (dx * dx + dy * dy + dz * dz <= r2) {
        local.points.push_back(pt);
      }
    }
    local.width = local.points.size();
    local.height = 1;
    local.is_dense = true;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    pcl::toROSMsg(local, msg);
    local_obstacles_pub_->publish(msg);
  }

  std::string map_mode_str_;
  double local_sense_radius_{5.0};
  MapGenerationResult map_result_;
  sensor_msgs::msg::PointCloud2 global_cloud_msg_;

  std::unique_ptr<MapGenerator> generator_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_obstacles_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr local_timer_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
};

}  // namespace drone_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_map::MapNode>());
  rclcpp::shutdown();
  return 0;
}

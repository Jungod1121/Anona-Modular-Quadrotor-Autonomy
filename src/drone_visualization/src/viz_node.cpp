#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace drone_visualization
{

/// Simplified quadrotor body marker from /drone/odom.
/// Marker layout inspired by pengyu_sim odom_visualization (arm + rotor cues),
/// rewritten for ROS2 and our topic contract.
class VizNode : public rclcpp::Node
{
public:
  VizNode()
  : Node("drone_visualization")
  {
    declare_parameter("namespace", "");
    declare_parameter("arm_length", 0.18);
    declare_parameter("body_scale", 0.12);
    declare_parameter("rotor_radius", 0.08);
    declare_parameter("show_mission_endpoints", false);
    declare_parameter("mission_start_x", 0.0);
    declare_parameter("mission_start_y", 0.0);
    declare_parameter("mission_start_z", 0.0);
    declare_parameter("mission_goal_x", 0.0);
    declare_parameter("mission_goal_y", 0.0);
    declare_parameter("mission_goal_z", 0.0);

    arm_length_ = get_parameter("arm_length").as_double();
    body_scale_ = get_parameter("body_scale").as_double();
    rotor_radius_ = get_parameter("rotor_radius").as_double();
    show_mission_endpoints_ = get_parameter("show_mission_endpoints").as_bool();

    // Namespaced multi-UAV markers are easy to miss at 30 m orbit — enlarge.
    const std::string ns0 = get_parameter("namespace").as_string();
    if (!ns0.empty()) {
      if (body_scale_ < 0.28) {
        body_scale_ = 0.28;
      }
      if (arm_length_ < 0.32) {
        arm_length_ = 0.32;
      }
      if (rotor_radius_ < 0.12) {
        rotor_radius_ = 0.12;
      }
    }

    const std::string prefix = topicPrefix();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      prefix + "/drone/body_markers", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      prefix + "/drone/odom", 10,
      std::bind(&VizNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "drone_visualization ready (quadrotor marker from /drone/odom)");
  }

private:
  std::string topicPrefix() const
  {
    const std::string ns = get_parameter("namespace").as_string();
    return ns.empty() ? "" : ("/" + ns);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray array;
    const auto & pose = msg->pose.pose;

    // Central body
    visualization_msgs::msg::Marker body;
    body.header = msg->header;
    body.ns = "drone_body";
    body.id = 0;
    body.type = visualization_msgs::msg::Marker::CUBE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose = pose;
    body.scale.x = body_scale_;
    body.scale.y = body_scale_;
    body.scale.z = body_scale_ * 0.5;
    body.color.r = 0.15f;
    body.color.g = 0.62f;
    body.color.b = 0.95f;
    body.color.a = 0.95f;
    // Distinct colors per UAV so multi-drone RViz is readable.
    {
      const std::string ns = get_parameter("namespace").as_string();
      if (ns == "uav1") {
        body.color.r = 0.20f;
        body.color.g = 0.85f;
        body.color.b = 0.35f;
      } else if (ns == "uav2") {
        body.color.r = 0.95f;
        body.color.g = 0.45f;
        body.color.b = 0.20f;
      }
    }
    array.markers.push_back(body);

    // Four arms + rotors (X configuration)
    const std::vector<std::pair<double, double>> arm_dirs = {
      {1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}};
    int id = 1;
    for (const auto & dir : arm_dirs) {
      const double norm = std::sqrt(dir.first * dir.first + dir.second * dir.second);
      const double ux = dir.first / norm;
      const double uy = dir.second / norm;

      visualization_msgs::msg::Marker arm;
      arm.header = msg->header;
      arm.ns = "drone_arms";
      arm.id = id++;
      arm.type = visualization_msgs::msg::Marker::LINE_STRIP;
      arm.action = visualization_msgs::msg::Marker::ADD;
      arm.pose.orientation.w = 1.0;
      arm.scale.x = 0.02;
      arm.color.r = 0.2f;
      arm.color.g = 0.2f;
      arm.color.b = 0.2f;
      arm.color.a = 1.0f;

      geometry_msgs::msg::Point p0, p1;
      p0.x = pose.position.x;
      p0.y = pose.position.y;
      p0.z = pose.position.z;
      p1.x = pose.position.x + ux * arm_length_;
      p1.y = pose.position.y + uy * arm_length_;
      p1.z = pose.position.z;
      arm.points = {p0, p1};
      array.markers.push_back(arm);

      visualization_msgs::msg::Marker rotor;
      rotor.header = msg->header;
      rotor.ns = "drone_rotors";
      rotor.id = id++;
      rotor.type = visualization_msgs::msg::Marker::CYLINDER;
      rotor.action = visualization_msgs::msg::Marker::ADD;
      rotor.pose.position.x = p1.x;
      rotor.pose.position.y = p1.y;
      rotor.pose.position.z = p1.z + 0.02;
      rotor.pose.orientation = pose.orientation;
      rotor.scale.x = 2.0 * rotor_radius_;
      rotor.scale.y = 2.0 * rotor_radius_;
      rotor.scale.z = 0.01;
      rotor.color.r = 0.9f;
      rotor.color.g = 0.9f;
      rotor.color.b = 0.9f;
      rotor.color.a = 0.8f;
      array.markers.push_back(rotor);
    }

    // Heading arrow
    visualization_msgs::msg::Marker arrow;
    arrow.header = msg->header;
    arrow.ns = "drone_heading";
    arrow.id = id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = pose;
    arrow.scale.x = arm_length_ * 1.2;
    arrow.scale.y = 0.04;
    arrow.scale.z = 0.04;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.85f;
    arrow.color.b = 0.1f;
    arrow.color.a = 0.9f;
    array.markers.push_back(arrow);

    // Floating name so uav0 / uav1 are obvious in multi RViz.
    {
      const std::string ns = get_parameter("namespace").as_string();
      if (!ns.empty()) {
        visualization_msgs::msg::Marker label;
        label.header = msg->header;
        label.ns = "drone_label";
        label.id = id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose = pose;
        label.pose.position.z += 0.45;
        label.scale.z = 0.45;
        label.color.r = body.color.r;
        label.color.g = body.color.g;
        label.color.b = body.color.b;
        label.color.a = 1.0f;
        label.text = ns;
        array.markers.push_back(label);
      }
    }

    if (show_mission_endpoints_) {
      const auto append_endpoint =
        [&](int marker_id, const std::string & text, double x, double y, double z,
          float red, float green, float blue)
        {
          visualization_msgs::msg::Marker point;
          point.header = msg->header;
          point.ns = "mission_endpoints";
          point.id = marker_id;
          point.type = visualization_msgs::msg::Marker::SPHERE;
          point.action = visualization_msgs::msg::Marker::ADD;
          point.pose.position.x = x;
          point.pose.position.y = y;
          point.pose.position.z = z;
          point.pose.orientation.w = 1.0;
          point.scale.x = 0.32;
          point.scale.y = 0.32;
          point.scale.z = 0.32;
          point.color.r = red;
          point.color.g = green;
          point.color.b = blue;
          point.color.a = 1.0f;
          array.markers.push_back(point);

          visualization_msgs::msg::Marker label = point;
          label.id = marker_id + 1;
          label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label.pose.position.z = z + 0.38;
          label.scale.x = 0.0;
          label.scale.y = 0.0;
          label.scale.z = 0.28;
          label.text = text;
          array.markers.push_back(label);
        };

      append_endpoint(
        100, "START",
        get_parameter("mission_start_x").as_double(),
        get_parameter("mission_start_y").as_double(),
        get_parameter("mission_start_z").as_double(),
        0.15f, 0.95f, 0.25f);
      append_endpoint(
        102, "GOAL",
        get_parameter("mission_goal_x").as_double(),
        get_parameter("mission_goal_y").as_double(),
        get_parameter("mission_goal_z").as_double(),
        0.95f, 0.20f, 0.20f);
    }

    marker_pub_->publish(array);
  }

  double arm_length_{0.18};
  double body_scale_{0.12};
  double rotor_radius_{0.08};
  bool show_mission_endpoints_{false};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace drone_visualization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_visualization::VizNode>());
  rclcpp::shutdown();
  return 0;
}

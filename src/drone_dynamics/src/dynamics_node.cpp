#include "drone_dynamics/quadrotor_model.hpp"

#include <drone_msgs/msg/motor_command.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace drone_dynamics
{

class DynamicsNode : public rclcpp::Node
{
public:
  DynamicsNode()
  : Node("drone_dynamics")
  {
    declareAndLoadParams();
    model_ = std::make_unique<QuadrotorModel>(params_);
    model_->state().p = Eigen::Vector3d(init_x_, init_y_, init_z_);
    model_->state().q = Eigen::AngleAxisd(init_yaw_, Eigen::Vector3d::UnitZ());

    motor_cmd_ = Eigen::Vector4d::Zero();

    const std::string ns = get_parameter("namespace").as_string();
    const std::string prefix = ns.empty() ? "" : ("/" + ns);
    body_frame_ = ns.empty() ? "base_link" : (ns + "/base_link");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(prefix + "/drone/odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(prefix + "/drone/imu", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(prefix + "/drone/path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    motor_sub_ = create_subscription<drone_msgs::msg::MotorCommand>(
      prefix + "/drone/motor_rpm_cmd", 10,
      [this](const drone_msgs::msg::MotorCommand::SharedPtr msg) {
        for (int i = 0; i < 4; ++i) {
          motor_cmd_(i) = msg->rpm[i];
        }
        last_cmd_time_ = now();
      });

    path_.header.frame_id = "map";
    const double pub_hz = get_parameter("publish_rate").as_double();
    const double integ_hz = get_parameter("integration_rate").as_double();
    steps_per_pub_ = std::max(1, static_cast<int>(std::lround(integ_hz / pub_hz)));
    dt_ = 1.0 / integ_hz;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / pub_hz),
      std::bind(&DynamicsNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "drone_dynamics ready: integ=%.0fHz pub=%.0fHz mass=%.2f (self-developed, not MARSIM/EGO sim)",
      integ_hz, pub_hz, params_.mass);
  }

private:
  void declareAndLoadParams()
  {
    declare_parameter("namespace", "");
    declare_parameter("integration_rate", 500.0);
    declare_parameter("publish_rate", 100.0);
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_z", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("cmd_timeout", 0.5);

    declare_parameter("mass", 1.0);
    declare_parameter("gravity", 9.81);
    declare_parameter("arm_length", 0.18);
    declare_parameter("Ixx", 0.01);
    declare_parameter("Iyy", 0.01);
    declare_parameter("Izz", 0.02);
    declare_parameter("k_F", 3.0e-5);
    declare_parameter("k_M", 5.0e-7);
    declare_parameter("tau_motor", 0.02);
    declare_parameter("omega_min", 0.0);
    declare_parameter("omega_max", 800.0);

    declare_parameter("wind_enable", false);
    declare_parameter("wind_const_x", 0.0);
    declare_parameter("wind_const_y", 0.0);
    declare_parameter("wind_const_z", 0.0);
    declare_parameter("wind_sin_amp", 0.0);
    declare_parameter("wind_sin_freq", 0.2);

    declare_parameter("imu_noise_enable", false);
    declare_parameter("imu_accel_noise_std", 0.02);
    declare_parameter("imu_gyro_noise_std", 0.01);
    declare_parameter("imu_accel_bias_rw", 1e-4);
    declare_parameter("imu_gyro_bias_rw", 1e-5);

    params_.mass = get_parameter("mass").as_double();
    params_.gravity = get_parameter("gravity").as_double();
    params_.arm_length = get_parameter("arm_length").as_double();
    params_.Ixx = get_parameter("Ixx").as_double();
    params_.Iyy = get_parameter("Iyy").as_double();
    params_.Izz = get_parameter("Izz").as_double();
    params_.k_F = get_parameter("k_F").as_double();
    params_.k_M = get_parameter("k_M").as_double();
    params_.tau_motor = get_parameter("tau_motor").as_double();
    params_.omega_min = get_parameter("omega_min").as_double();
    params_.omega_max = get_parameter("omega_max").as_double();
    params_.wind_enable = get_parameter("wind_enable").as_bool();
    params_.wind_const_x = get_parameter("wind_const_x").as_double();
    params_.wind_const_y = get_parameter("wind_const_y").as_double();
    params_.wind_const_z = get_parameter("wind_const_z").as_double();
    params_.wind_sin_amp = get_parameter("wind_sin_amp").as_double();
    params_.wind_sin_freq = get_parameter("wind_sin_freq").as_double();
    params_.imu_noise_enable = get_parameter("imu_noise_enable").as_bool();
    params_.imu_accel_noise_std = get_parameter("imu_accel_noise_std").as_double();
    params_.imu_gyro_noise_std = get_parameter("imu_gyro_noise_std").as_double();
    params_.imu_accel_bias_rw = get_parameter("imu_accel_bias_rw").as_double();
    params_.imu_gyro_bias_rw = get_parameter("imu_gyro_bias_rw").as_double();

    init_x_ = get_parameter("init_x").as_double();
    init_y_ = get_parameter("init_y").as_double();
    init_z_ = get_parameter("init_z").as_double();
    init_yaw_ = get_parameter("init_yaw").as_double();
    cmd_timeout_ = get_parameter("cmd_timeout").as_double();
  }

  void onTimer()
  {
    Eigen::Vector4d cmd = motor_cmd_;
    if ((now() - last_cmd_time_).seconds() > cmd_timeout_) {
      cmd.setZero();  // failsafe: motors off
    }
    for (int i = 0; i < steps_per_pub_; ++i) {
      model_->step(cmd, dt_, sim_time_);
      sim_time_ += dt_;
    }
    publish();
  }

  void publish()
  {
    const auto & s = model_->state();
    const auto stamp = now();
    if (!std::isfinite(s.p.x()) || !std::isfinite(s.p.y()) || !std::isfinite(s.p.z()) ||
        !std::isfinite(s.q.w()) || !std::isfinite(s.q.x()) ||
        !std::isfinite(s.q.y()) || !std::isfinite(s.q.z()))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Skipping odom/TF publish — non-finite state");
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = body_frame_;
    odom.pose.pose.position.x = s.p.x();
    odom.pose.pose.position.y = s.p.y();
    odom.pose.pose.position.z = s.p.z();
    odom.pose.pose.orientation.w = s.q.w();
    odom.pose.pose.orientation.x = s.q.x();
    odom.pose.pose.orientation.y = s.q.y();
    odom.pose.pose.orientation.z = s.q.z();
    odom.twist.twist.linear.x = s.v.x();
    odom.twist.twist.linear.y = s.v.y();
    odom.twist.twist.linear.z = s.v.z();
    odom.twist.twist.angular.x = s.omega.x();
    odom.twist.twist.angular.y = s.omega.y();
    odom.twist.twist.angular.z = s.omega.z();
    odom_pub_->publish(odom);

    Eigen::Vector3d accel, gyro;
    model_->sampleImu(accel, gyro, dt_ * steps_per_pub_);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = body_frame_;
    imu.orientation = odom.pose.pose.orientation;
    imu.angular_velocity.x = gyro.x();
    imu.angular_velocity.y = gyro.y();
    imu.angular_velocity.z = gyro.z();
    imu.linear_acceleration.x = accel.x();
    imu.linear_acceleration.y = accel.y();
    imu.linear_acceleration.z = accel.z();
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = body_frame_;
    tf.transform.translation.x = s.p.x();
    tf.transform.translation.y = s.p.y();
    tf.transform.translation.z = s.p.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    path_.header.stamp = stamp;
    path_.poses.push_back(ps);
    if (path_.poses.size() > 5000) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1000);
    }
    path_pub_->publish(path_);
  }

  DynamicsParams params_;
  std::unique_ptr<QuadrotorModel> model_;
  Eigen::Vector4d motor_cmd_;
  double dt_{0.002};
  int steps_per_pub_{5};
  double sim_time_{0.0};
  double init_x_{0}, init_y_{0}, init_z_{0}, init_yaw_{0};
  double cmd_timeout_{0.5};
  std::string body_frame_{"base_link"};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<drone_msgs::msg::MotorCommand>::SharedPtr motor_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_dynamics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_dynamics::DynamicsNode>());
  rclcpp::shutdown();
  return 0;
}

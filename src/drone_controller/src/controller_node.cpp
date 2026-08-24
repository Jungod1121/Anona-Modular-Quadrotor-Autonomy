#include "drone_controller/cascade_pid.hpp"

#include <drone_msgs/msg/motor_command.hpp>
#include <drone_msgs/msg/trajectory_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace drone_controller
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("drone_controller")
  {
    declareAndLoadParams();

    const std::string ns = get_parameter("namespace").as_string();
    const std::string prefix = ns.empty() ? "" : ("/" + ns);

    motor_pub_ = create_publisher<drone_msgs::msg::MotorCommand>(
      prefix + "/drone/motor_rpm_cmd", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      prefix + "/drone/odom", 10,
      std::bind(&ControllerNode::onOdom, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      prefix + "/drone/imu", 50,
      std::bind(&ControllerNode::onImu, this, std::placeholders::_1));

    local_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix + "/planner/local_goal", 10,
      std::bind(&ControllerNode::onLocalGoal, this, std::placeholders::_1));

    traj_cmd_sub_ = create_subscription<drone_msgs::msg::TrajectoryCommand>(
      prefix + "/planner/trajectory_cmd", 10,
      std::bind(&ControllerNode::onTrajectoryCmd, this, std::placeholders::_1));

    if (get_parameter("use_drone_goal_fallback").as_bool()) {
      goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        prefix + "/drone/goal", 10,
        std::bind(&ControllerNode::onGoal, this, std::placeholders::_1));
    }

    const double rate = get_parameter("control_rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&ControllerNode::onControl, this));

    RCLCPP_INFO(
      get_logger(),
      "drone_controller ready: %.0f Hz, mass=%.2f, goal_fallback=%s",
      rate, params_.mass,
      get_parameter("use_drone_goal_fallback").as_bool() ? "on" : "off");
  }

private:
  void declareAndLoadParams()
  {
    declare_parameter("namespace", "");
    declare_parameter("control_rate", 100.0);
    declare_parameter("local_goal_timeout", 0.5);
    declare_parameter("trajectory_cmd_timeout", 0.25);
    // Fail-safe: if /drone/odom stalls longer than this, stop trusting the
    // stale state and cut motors instead of integrating against phantom error.
    declare_parameter("odom_timeout_sec", 0.2);
    // When false, ignore /drone/goal ballistic fallback (multi/EGO-Swarm: goals
    // must not bypass the planner).
    declare_parameter("use_drone_goal_fallback", true);

    declare_parameter("mass", 1.0);
    declare_parameter("gravity", 9.81);
    declare_parameter("arm_length", 0.18);
    declare_parameter("k_F", 3.0e-5);
    declare_parameter("k_M", 5.0e-7);

    declare_parameter("pos_kp.x", 1.2);
    declare_parameter("pos_kp.y", 1.2);
    declare_parameter("pos_kp.z", 2.0);
    declare_parameter("pos_kd.x", 1.0);
    declare_parameter("pos_kd.y", 1.0);
    declare_parameter("pos_kd.z", 1.5);
    declare_parameter("pos_ki.x", 0.35);
    declare_parameter("pos_ki.y", 0.35);
    declare_parameter("pos_ki.z", 0.55);
    declare_parameter("pos_i_limit.x", 1.2);
    declare_parameter("pos_i_limit.y", 1.2);
    declare_parameter("pos_i_limit.z", 1.5);

    declare_parameter("att_kp.x", 6.0);
    declare_parameter("att_kp.y", 6.0);
    declare_parameter("att_kp.z", 3.0);
    declare_parameter("att_kd.x", 0.4);
    declare_parameter("att_kd.y", 0.4);
    declare_parameter("att_kd.z", 0.2);

    declare_parameter("max_vel", 2.0);
    declare_parameter("max_acc", 3.0);
    declare_parameter("max_tilt", 0.45);
    declare_parameter("max_yaw_rate", 1.0);
    declare_parameter("max_torque.x", 0.08);
    declare_parameter("max_torque.y", 0.08);
    declare_parameter("max_torque.z", 0.04);

    declare_parameter("min_thrust", 0.0);
    declare_parameter("max_thrust", 0.0);
    declare_parameter("rpm_min", 0.0);
    declare_parameter("rpm_max", 7000.0);
    declare_parameter("max_motor_rpm_rate", 12000.0);
    declare_parameter("goal_slowdown_dist", 3.0);

    declare_parameter("disturbance_reject_enable", true);
    declare_parameter("disturbance_gain", 1.2);
    declare_parameter("disturbance_leak", 0.08);
    declare_parameter("disturbance_limit.x", 1.8);
    declare_parameter("disturbance_limit.y", 1.8);
    declare_parameter("disturbance_limit.z", 2.5);

    // IMU gyro aid: LPF body rates from /drone/imu (improves damping when IMU noise is on).
    declare_parameter("imu_aid_enable", true);
    declare_parameter("imu_gyro_lpf_hz", 12.0);
    declare_parameter("imu_rate_blend", 0.65);

    loadParamsFromRos();
    controller_ = CascadePid(params_);
  }

  void loadParamsFromRos()
  {
    params_.mass = get_parameter("mass").as_double();
    params_.gravity = get_parameter("gravity").as_double();
    params_.arm_length = get_parameter("arm_length").as_double();
    params_.k_F = get_parameter("k_F").as_double();
    params_.k_M = get_parameter("k_M").as_double();

    params_.pos_kp = vec3("pos_kp");
    params_.pos_kd = vec3("pos_kd");
    params_.pos_ki = vec3("pos_ki");
    params_.pos_i_limit = vec3("pos_i_limit");
    params_.att_kp = vec3("att_kp");
    params_.att_kd = vec3("att_kd");
    params_.max_torque = vec3("max_torque");

    params_.max_vel = get_parameter("max_vel").as_double();
    params_.max_acc = get_parameter("max_acc").as_double();
    params_.max_tilt = get_parameter("max_tilt").as_double();
    params_.max_yaw_rate = get_parameter("max_yaw_rate").as_double();
    params_.min_thrust = get_parameter("min_thrust").as_double();
    params_.max_thrust = get_parameter("max_thrust").as_double();
    params_.rpm_min = get_parameter("rpm_min").as_double();
    params_.rpm_max = get_parameter("rpm_max").as_double();
    params_.max_motor_rpm_rate = get_parameter("max_motor_rpm_rate").as_double();
    params_.goal_slowdown_dist = get_parameter("goal_slowdown_dist").as_double();

    params_.disturbance_reject_enable =
      get_parameter("disturbance_reject_enable").as_bool();
    params_.disturbance_gain = get_parameter("disturbance_gain").as_double();
    params_.disturbance_leak = get_parameter("disturbance_leak").as_double();
    params_.disturbance_limit = vec3("disturbance_limit");
  }

  Eigen::Vector3d vec3(const std::string & prefix)
  {
    return Eigen::Vector3d(
      get_parameter(prefix + ".x").as_double(),
      get_parameter(prefix + ".y").as_double(),
      get_parameter(prefix + ".z").as_double());
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Eigen::Vector3d p(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    const Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    if (!p.allFinite() || !q.coeffs().allFinite() || q.norm() < 0.5) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Ignoring non-finite / denormalized odom");
      state_.valid = false;
      return;
    }
    state_.position = p;
    state_.velocity = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    state_.attitude = q.normalized();
    const Eigen::Vector3d omega_odom(
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z);

    // Blend LPF'd IMU gyro into rate feedback — cancels high-freq IMU noise and
    // improves damping vs odom-only when imu_noise_enable is on in dynamics.
    if (get_parameter("imu_aid_enable").as_bool() && have_imu_) {
      const double blend = std::clamp(get_parameter("imu_rate_blend").as_double(), 0.0, 1.0);
      state_.omega = (1.0 - blend) * omega_odom + blend * imu_omega_filt_;
    } else {
      state_.omega = omega_odom;
    }
    state_.valid = true;
    last_odom_time_ = now();
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const Eigen::Vector3d gyro(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    const double hz = std::max(1.0, get_parameter("imu_gyro_lpf_hz").as_double());
    const double alpha = std::clamp(1.0 - std::exp(-2.0 * M_PI * hz / 200.0), 0.05, 0.95);
    if (!have_imu_) {
      imu_omega_filt_ = gyro;
      have_imu_ = true;
    } else {
      imu_omega_filt_ = alpha * gyro + (1.0 - alpha) * imu_omega_filt_;
    }
  }

  void onLocalGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    local_goal_.position = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
    local_goal_.yaw = yawFromQuaternion(msg->pose.orientation);
    local_goal_.velocity.setZero();
    local_goal_.acceleration.setZero();
    local_goal_.yaw_dot = 0.0;
    local_goal_.use_feedforward = false;
    local_goal_.valid = true;
    last_local_goal_time_ = now();
    // Once the planner takes over, never ballistic-fly to the final /drone/goal
    // (that is what made blue /drone/path diverge from yellow /planner/trajectory).
    planner_guided_ = true;
    fallback_goal_.valid = false;
  }

  void onTrajectoryCmd(const drone_msgs::msg::TrajectoryCommand::SharedPtr msg)
  {
    if (!msg->trajectory_ready) {
      traj_cmd_.valid = false;
      return;
    }
    traj_cmd_.position = Eigen::Vector3d(
      msg->position.x, msg->position.y, msg->position.z);
    traj_cmd_.velocity = Eigen::Vector3d(
      msg->velocity.x, msg->velocity.y, msg->velocity.z);
    traj_cmd_.acceleration = Eigen::Vector3d(
      msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    traj_cmd_.yaw = msg->yaw;
    traj_cmd_.yaw_dot = msg->yaw_dot;
    traj_cmd_.use_feedforward = true;
    traj_cmd_.valid = true;
    last_traj_cmd_time_ = now();
    planner_guided_ = true;
    fallback_goal_.valid = false;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    fallback_goal_.position = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
    fallback_goal_.yaw = yawFromQuaternion(msg->pose.orientation);
    // New hard goal only used until planner publishes its first local_goal.
    fallback_goal_.valid = !planner_guided_;
  }

  GoalState activeGoal() const
  {
    const double traj_timeout = get_parameter("trajectory_cmd_timeout").as_double();
    const bool traj_fresh =
      traj_cmd_.valid &&
      (now() - last_traj_cmd_time_).seconds() <= traj_timeout;
    if (traj_fresh) {
      return traj_cmd_;
    }

    const double timeout = get_parameter("local_goal_timeout").as_double();
    const bool local_fresh =
      local_goal_.valid &&
      (now() - last_local_goal_time_).seconds() <= timeout;

    if (local_fresh) {
      return local_goal_;
    }
    // Prefer holding the last path waypoint over leaping to the final goal.
    if (planner_guided_ && local_goal_.valid) {
      return local_goal_;
    }
    if (fallback_goal_.valid) {
      return fallback_goal_;
    }
    return GoalState{};
  }

  void onControl()
  {
    if (!state_.valid) {
      return;
    }

    // Odometry staleness watchdog: dynamics crash / QoS drop must not leave
    // the integrator and disturbance observer running against a frozen pose.
    const double odom_age = (now() - last_odom_time_).seconds();
    if (odom_age > std::max(0.0, get_parameter("odom_timeout_sec").as_double())) {
      state_.valid = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Odometry stale (%.2f s > %.2f s) — cutting motors", odom_age,
        get_parameter("odom_timeout_sec").as_double());
      drone_msgs::msg::MotorCommand cmd;
      cmd.header.stamp = now();
      cmd.header.frame_id = "base_link";
      motor_pub_->publish(cmd);
      return;
    }

    const double dt = 1.0 / get_parameter("control_rate").as_double();
    const GoalState goal = activeGoal();
    const Eigen::Vector4d rpm = controller_.compute(state_, goal, dt);

    drone_msgs::msg::MotorCommand cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    for (int i = 0; i < 4; ++i) {
      cmd.rpm[i] = rpm(i);
    }
    motor_pub_->publish(cmd);
  }

  ControllerParams params_;
  CascadePid controller_;
  VehicleState state_;
  GoalState local_goal_;
  GoalState traj_cmd_;
  GoalState fallback_goal_;
  bool planner_guided_{false};
  Eigen::Vector3d imu_omega_filt_{Eigen::Vector3d::Zero()};
  bool have_imu_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_traj_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<drone_msgs::msg::MotorCommand>::SharedPtr motor_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_sub_;
  rclcpp::Subscription<drone_msgs::msg::TrajectoryCommand>::SharedPtr traj_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

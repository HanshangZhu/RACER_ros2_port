// PositionCommand -> geometry_msgs/Twist bridge for real quadrupeds
// (Unitree Go1/Go2 via unitree_ros2/go2_ros2_sdk, or any Gazebo/Isaac
// quadruped model that accepts cmd_vel).
//
// Listens to the latest /odom (SharedPtr-cached) to know the current body
// yaw, re-projects the planner's world-frame velocity into the body frame,
// clips per-axis, and publishes geometry_msgs/Twist at rate_hz.
//
// Params:
//   v_fwd_max, v_lat_max, omega_max  - body-frame caps (same as cmd2base)
//   rate_hz                          - Twist publish rate                [50]
//   odom_topic, cmd_topic, twist_topic - IO topic remaps

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CmdToTwist : public rclcpp::Node {
public:
  CmdToTwist() : rclcpp::Node("cmd_to_twist") {
    v_fwd_max_ = this->declare_parameter<double>("v_fwd_max", 1.0);
    v_lat_max_ = this->declare_parameter<double>("v_lat_max", 0.5);
    omega_max_ = this->declare_parameter<double>("omega_max", 1.0);
    const double rate_hz = this->declare_parameter<double>("rate_hz", 50.0);

    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "command", 1,
        std::bind(&CmdToTwist::cmdCb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 1,
        std::bind(&CmdToTwist::odomCb, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CmdToTwist::tick, this));
  }

private:
  void cmdCb(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    cmd_ = *msg;
    have_cmd_ = true;
  }
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void tick() {
    quadrotor_msgs::msg::PositionCommand cmd;
    double yaw = 0.0;
    bool ready = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      ready = have_cmd_ && have_odom_;
      if (ready) { cmd = cmd_; yaw = yaw_; }
    }
    geometry_msgs::msg::Twist out;
    if (ready) {
      const double cy = std::cos(yaw), sy = std::sin(yaw);
      const double v_body_x =  cy * cmd.velocity.x + sy * cmd.velocity.y;
      const double v_body_y = -sy * cmd.velocity.x + cy * cmd.velocity.y;
      out.linear.x = std::clamp(v_body_x, -v_fwd_max_, v_fwd_max_);
      out.linear.y = std::clamp(v_body_y, -v_lat_max_, v_lat_max_);
      out.linear.z = 0.0;
      out.angular.x = 0.0;
      out.angular.y = 0.0;
      out.angular.z = std::clamp(cmd.yaw_dot, -omega_max_, omega_max_);
    }
    twist_pub_->publish(out);
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mu_;
  quadrotor_msgs::msg::PositionCommand cmd_;
  double yaw_{0.0};
  bool have_cmd_{false}, have_odom_{false};

  double v_fwd_max_{1.0}, v_lat_max_{0.5}, omega_max_{1.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdToTwist>());
  rclcpp::shutdown();
  return 0;
}

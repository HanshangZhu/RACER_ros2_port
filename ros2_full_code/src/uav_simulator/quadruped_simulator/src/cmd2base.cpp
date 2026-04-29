// Quadruped light-sim: integrates a quadrotor_msgs/PositionCommand into
// nav_msgs/Odometry on the ground plane with a non-holonomic body-frame
// motion model (forward/lateral vel + yaw rate, each clipped).
//
// Design notes:
//   * Same IO contract as poscmd_2_odom (drop-in replacement when the
//     exploration stack is run with robot_type:=quadruped).
//   * Z is pinned to `stand_height` — the stock planner may emit z != this,
//     but the quadruped launch profile restricts the SDF box to a thin slab
//     around stand_height so the vertical drift stays tiny.
//   * Orientation uses yaw only (roll = pitch = 0). A real quadruped's
//     locomotion controller absorbs pitch/roll for gait — from the planner's
//     perspective the base frame is flat.
//   * Velocity is re-projected into the body frame and per-axis clipped.
//     This makes the simulated motion look quadruped-shaped even though the
//     planner is holonomic (Phase 3 fixes that).
//
// Params (dotted ROS 2 names):
//   stand_height      : base-link height above ground (m)              [0.30]
//   v_fwd_max         : max forward body velocity (m/s)                [1.00]
//   v_lat_max         : max lateral body velocity (m/s)                [0.50]
//   omega_max         : max yaw rate (rad/s)                           [1.00]
//   rate_hz           : integrator/publish rate (Hz)                   [50.0]
//   init_x, init_y    : initial pose                                   [0, 0]
//   drone_id          : sets Odometry.child_frame_id                   [1]

#include <chrono>
#include <cmath>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class Cmd2Base : public rclcpp::Node {
public:
  Cmd2Base() : rclcpp::Node("cmd2base") {
    stand_height_ = this->declare_parameter<double>("stand_height", 0.30);
    v_fwd_max_ = this->declare_parameter<double>("v_fwd_max", 1.0);
    v_lat_max_ = this->declare_parameter<double>("v_lat_max", 0.5);
    omega_max_ = this->declare_parameter<double>("omega_max", 1.0);
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    init_x_ = this->declare_parameter<double>("init_x", 0.0);
    init_y_ = this->declare_parameter<double>("init_y", 0.0);
    drone_id_ = this->declare_parameter<int>("drone_id", 1);

    x_ = init_x_;
    y_ = init_y_;
    yaw_ = 0.0;

    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "command", 1,
        std::bind(&Cmd2Base::rcvPosCmdCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&Cmd2Base::pubOdom, this));
    last_tick_ = this->now();
  }

private:
  void rcvPosCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
    rcv_cmd_ = true;
    cmd_ = *cmd;
  }

  void pubOdom() {
    const rclcpp::Time now_t = this->now();
    double dt = (now_t - last_tick_).seconds();
    last_tick_ = now_t;
    if (dt <= 0.0 || dt > 0.5) dt = 1.0 / rate_hz_;

    double v_fwd = 0.0, v_lat = 0.0, omega = 0.0;

    // Stay silent until the first PositionCommand arrives. The planner's
    // frontierCallback races against sensor-map population; emitting odom
    // before the first cloud/PositionCommand arrives transitions the FSM
    // to WAIT_TRIGGER and lets the race fire. Matches poscmd_2_odom.
    if (!rcv_cmd_) return;

    if (rcv_cmd_) {
      // Re-project the planner's world-frame velocity into the robot body frame.
      const double cy = std::cos(yaw_), sy = std::sin(yaw_);
      const double v_body_x =  cy * cmd_.velocity.x + sy * cmd_.velocity.y;
      const double v_body_y = -sy * cmd_.velocity.x + cy * cmd_.velocity.y;
      v_fwd = std::clamp(v_body_x, -v_fwd_max_, v_fwd_max_);
      v_lat = std::clamp(v_body_y, -v_lat_max_, v_lat_max_);
      omega = std::clamp(cmd_.yaw_dot, -omega_max_, omega_max_);

      // Integrate SE(2) pose.
      const double cy2 = std::cos(yaw_), sy2 = std::sin(yaw_);
      x_ += (cy2 * v_fwd - sy2 * v_lat) * dt;
      y_ += (sy2 * v_fwd + cy2 * v_lat) * dt;
      yaw_ += omega * dt;
      // Wrap yaw to [-pi, pi].
      while (yaw_ >  M_PI) yaw_ -= 2.0 * M_PI;
      while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now_t;
    odom.header.frame_id = "world";
    odom.child_frame_id = std::to_string(drone_id_);

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = stand_height_;

    // Yaw-only quaternion (roll = pitch = 0).
    const double half = 0.5 * yaw_;
    odom.pose.pose.orientation.w = std::cos(half);
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half);

    // Expose body-frame velocity in twist (child_frame_id is the body).
    odom.twist.twist.linear.x = v_fwd;
    odom.twist.twist.linear.y = v_lat;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = omega;

    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  quadrotor_msgs::msg::PositionCommand cmd_;
  double stand_height_{0.30};
  double v_fwd_max_{1.0}, v_lat_max_{0.5}, omega_max_{1.0};
  double rate_hz_{50.0};
  double init_x_{0.0}, init_y_{0.0};
  int drone_id_{1};

  double x_{0.0}, y_{0.0}, yaw_{0.0};
  rclcpp::Time last_tick_;
  bool rcv_cmd_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Cmd2Base>());
  rclcpp::shutdown();
  return 0;
}

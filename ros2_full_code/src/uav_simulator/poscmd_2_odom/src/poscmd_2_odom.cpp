#include <cmath>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class PoscmdToOdom : public rclcpp::Node {
public:
  PoscmdToOdom() : rclcpp::Node("odom_generator") {
    init_x_ = this->declare_parameter<double>("init_x", 0.0);
    init_y_ = this->declare_parameter<double>("init_y", 0.0);
    init_z_ = this->declare_parameter<double>("init_z", 0.0);
    drone_id_ = this->declare_parameter<int>("drone_id", 1);

    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "command", 1,
        std::bind(&PoscmdToOdom::rcvPosCmdCallBack, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&PoscmdToOdom::pubOdom, this));
  }

private:
  void rcvPosCmdCallBack(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
    rcv_cmd_ = true;
    cmd_ = *cmd;
  }

  void pubOdom() {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "world";
    odom.child_frame_id = std::to_string(drone_id_);

    if (rcv_cmd_) {
      odom.pose.pose.position.x = cmd_.position.x;
      odom.pose.pose.position.y = cmd_.position.y;
      odom.pose.pose.position.z = cmd_.position.z;

      Eigen::Vector3d alpha =
          Eigen::Vector3d(cmd_.acceleration.x, cmd_.acceleration.y, cmd_.acceleration.z) +
          9.8 * Eigen::Vector3d(0, 0, 1);
      Eigen::Vector3d xC(std::cos(cmd_.yaw), std::sin(cmd_.yaw), 0);
      Eigen::Vector3d yC(-std::sin(cmd_.yaw), std::cos(cmd_.yaw), 0);
      Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
      Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
      Eigen::Vector3d zB = xB.cross(yB);
      Eigen::Matrix3d R;
      R.col(0) = xB;
      R.col(1) = yB;
      R.col(2) = zB;
      Eigen::Quaterniond q(R);
      odom.pose.pose.orientation.w = q.w();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();

      odom.twist.twist.linear.x = cmd_.velocity.x;
      odom.twist.twist.linear.y = cmd_.velocity.y;
      odom.twist.twist.linear.z = cmd_.velocity.z;

      odom.twist.twist.angular.x = cmd_.acceleration.x;
      odom.twist.twist.angular.y = cmd_.acceleration.y;
      odom.twist.twist.angular.z = cmd_.acceleration.z;
    } else {
      odom.pose.pose.position.x = init_x_;
      odom.pose.pose.position.y = init_y_;
      odom.pose.pose.position.z = init_z_;
      odom.pose.pose.orientation.w = 1;
      odom.pose.pose.orientation.x = 0;
      odom.pose.pose.orientation.y = 0;
      odom.pose.pose.orientation.z = 0;
    }
    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  quadrotor_msgs::msg::PositionCommand cmd_;
  double init_x_{0.0}, init_y_{0.0}, init_z_{0.0};
  int drone_id_{1};
  bool rcv_cmd_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoscmdToOdom>());
  rclcpp::shutdown();
  return 0;
}

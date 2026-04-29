#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <so3_control/SO3Control.h>

namespace so3_control {

class SO3ControlComponent : public rclcpp::Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SO3ControlComponent(const rclcpp::NodeOptions& options)
      : rclcpp::Node("so3_control", options) {
    auto quadrotor_name = this->declare_parameter<std::string>("quadrotor_name", "quadrotor");
    frame_id_ = "/" + quadrotor_name;

    double mass = this->declare_parameter<double>("mass", 0.74);
    controller_.setMass(mass);

    use_external_yaw_ = this->declare_parameter<bool>("use_external_yaw", true);

    k_r_[0] = this->declare_parameter<double>("gains.rot.x", 1.5);
    k_r_[1] = this->declare_parameter<double>("gains.rot.y", 1.5);
    k_r_[2] = this->declare_parameter<double>("gains.rot.z", 1.0);
    k_om_[0] = this->declare_parameter<double>("gains.ang.x", 0.13);
    k_om_[1] = this->declare_parameter<double>("gains.ang.y", 0.13);
    k_om_[2] = this->declare_parameter<double>("gains.ang.z", 0.1);

    corrections_[0] = this->declare_parameter<double>("corrections.z", 0.0);
    corrections_[1] = this->declare_parameter<double>("corrections.r", 0.0);
    corrections_[2] = this->declare_parameter<double>("corrections.p", 0.0);

    so3_command_pub_ =
        this->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
        std::bind(&SO3ControlComponent::odomCb, this, std::placeholders::_1));
    position_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "position_cmd", 10,
        std::bind(&SO3ControlComponent::positionCmdCb, this, std::placeholders::_1));
    enable_motors_sub_ = this->create_subscription<std_msgs::msg::Bool>("motors", 2,
        std::bind(&SO3ControlComponent::enableMotorsCb, this, std::placeholders::_1));
    corrections_sub_ = this->create_subscription<quadrotor_msgs::msg::Corrections>(
        "corrections", 10,
        std::bind(&SO3ControlComponent::correctionsCb, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10,
        std::bind(&SO3ControlComponent::imuCb, this, std::placeholders::_1));
  }

private:
  void publishSO3Command() {
    controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_, kx_, kv_);
    const Eigen::Vector3d& force = controller_.getComputedForce();
    const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

    quadrotor_msgs::msg::SO3Command cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = frame_id_;
    cmd.force.x = force(0);
    cmd.force.y = force(1);
    cmd.force.z = force(2);
    cmd.orientation.x = orientation.x();
    cmd.orientation.y = orientation.y();
    cmd.orientation.z = orientation.z();
    cmd.orientation.w = orientation.w();
    for (int i = 0; i < 3; i++) {
      cmd.k_r[i] = k_r_[i];
      cmd.k_om[i] = k_om_[i];
    }
    cmd.aux.current_yaw = current_yaw_;
    cmd.aux.kf_correction = corrections_[0];
    cmd.aux.angle_corrections[0] = corrections_[1];
    cmd.aux.angle_corrections[1] = corrections_[2];
    cmd.aux.enable_motors = enable_motors_;
    cmd.aux.use_external_yaw = use_external_yaw_;
    so3_command_pub_->publish(cmd);
  }

  void positionCmdCb(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
    des_pos_ << cmd->position.x, cmd->position.y, cmd->position.z;
    des_vel_ << cmd->velocity.x, cmd->velocity.y, cmd->velocity.z;
    des_acc_ << cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z;
    kx_ << cmd->kx[0], cmd->kx[1], cmd->kx[2];
    kv_ << cmd->kv[0], cmd->kv[1], cmd->kv[2];
    des_yaw_ = cmd->yaw;
    des_yaw_dot_ = cmd->yaw_dot;
    position_cmd_updated_ = true;
    position_cmd_init_ = true;
    publishSO3Command();
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr odom) {
    Eigen::Vector3d position(odom->pose.pose.position.x, odom->pose.pose.position.y,
        odom->pose.pose.position.z);
    Eigen::Vector3d velocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y,
        odom->twist.twist.linear.z);
    current_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
    controller_.setPosition(position);
    controller_.setVelocity(velocity);

    if (position_cmd_init_) {
      if (!position_cmd_updated_) publishSO3Command();
      position_cmd_updated_ = false;
    }
  }

  void enableMotorsCb(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), msg->data ? "Enabling motors" : "Disabling motors");
    enable_motors_ = msg->data;
  }

  void correctionsCb(const quadrotor_msgs::msg::Corrections::SharedPtr msg) {
    corrections_[0] = msg->kf_correction;
    corrections_[1] = msg->angle_corrections[0];
    corrections_[2] = msg->angle_corrections[1];
  }

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr imu) {
    Eigen::Vector3d acc(
        imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    controller_.setAcc(acc);
  }

  SO3Control controller_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  bool position_cmd_updated_{false}, position_cmd_init_{false};
  std::string frame_id_;
  Eigen::Vector3d des_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d des_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d des_acc_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d kx_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d kv_{Eigen::Vector3d::Zero()};
  double des_yaw_{0}, des_yaw_dot_{0};
  double current_yaw_{0};
  bool enable_motors_{true};
  bool use_external_yaw_{false};
  double k_r_[3]{0, 0, 0};
  double k_om_[3]{0, 0, 0};
  double corrections_[3]{0, 0, 0};
};

}  // namespace so3_control

RCLCPP_COMPONENTS_REGISTER_NODE(so3_control::SO3ControlComponent)

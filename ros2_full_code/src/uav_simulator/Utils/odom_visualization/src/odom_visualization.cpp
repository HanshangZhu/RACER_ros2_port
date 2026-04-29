// ROS 2 port of odom_visualization.
// Publishes a colored mesh marker tracking either odometry or commanded position.
// Additional pose/path/velocity/covariance outputs exist in the upstream ROS 1
// source but were already commented out there — they are omitted here for clarity.

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <armadillo>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pose_utils/pose_utils.h>
#include <quadrotor_msgs/msg/position_command.hpp>

using namespace arma;

static Eigen::Vector4d getColor(double h, double alpha) {
  if (h < 0.0 || h > 1.0) h = 0.0;
  double lambda;
  Eigen::Vector4d c1, c2;
  if (h < 1.0 / 6) {
    lambda = h * 6;
    c1 = {1, 0, 0, 1};
    c2 = {1, 0, 1, 1};
  } else if (h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    c1 = {1, 0, 1, 1};
    c2 = {0, 0, 1, 1};
  } else if (h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    c1 = {0, 0, 1, 1};
    c2 = {0, 1, 1, 1};
  } else if (h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    c1 = {0, 1, 1, 1};
    c2 = {0, 1, 0, 1};
  } else if (h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    c1 = {0, 1, 0, 1};
    c2 = {1, 1, 0, 1};
  } else {
    lambda = (h - 5.0 / 6) * 6;
    c1 = {1, 1, 0, 1};
    c2 = {1, 0, 0, 1};
  }
  Eigen::Vector4d fcolor = (1 - lambda) * c1 + lambda * c2;
  fcolor(3) = alpha;
  return fcolor;
}

class OdomVisualization : public rclcpp::Node {
public:
  OdomVisualization() : rclcpp::Node("odom_visualization") {
    mesh_resource_ = this->declare_parameter<std::string>(
        "mesh_resource", "package://odom_visualization/meshes/hummingbird.mesh");
    color_r_ = this->declare_parameter<double>("color.r", 1.0);
    color_g_ = this->declare_parameter<double>("color.g", 0.0);
    color_b_ = this->declare_parameter<double>("color.b", 0.0);
    color_a_ = this->declare_parameter<double>("color.a", 1.0);
    origin_ = this->declare_parameter<bool>("origin", false);
    scale_ = this->declare_parameter<double>("robot_scale", 2.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "world");
    cross_config_ = this->declare_parameter<bool>("cross_config", false);
    tf45_ = this->declare_parameter<bool>("tf45", false);
    cov_scale_ = this->declare_parameter<double>("covariance_scale", 100.0);
    cov_pos_ = this->declare_parameter<bool>("covariance_position", false);
    cov_vel_ = this->declare_parameter<bool>("covariance_velocity", false);
    cov_color_ = this->declare_parameter<bool>("covariance_color", false);
    drone_id_ = this->declare_parameter<int>("drone_id", 1);
    drone_num_ = this->declare_parameter<int>("drone_num", 1);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 100,
        std::bind(&OdomVisualization::odomCb, this, std::placeholders::_1));
    sub_cmd_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>("cmd", 100,
        std::bind(&OdomVisualization::cmdCb, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(100).transient_local();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", qos);
    vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("velocity", qos);
    cov_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("covariance", qos);
    cov_vel_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("covariance_velocity", qos);
    traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory", qos);
    sensor_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sensor", qos);
    mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot", qos);
    height_pub_ = this->create_publisher<sensor_msgs::msg::Range>("height", qos);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void applyYawOffset(colvec& q) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * M_PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg->header.frame_id == "null") return;

    colvec q(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    if (cross_config_) applyYawOffset(q);

    visualization_msgs::msg::Marker mesh;
    mesh.header.frame_id = frame_id_;
    mesh.header.stamp = msg->header.stamp;
    mesh.ns = "mesh";
    mesh.id = 0;
    mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh.action = visualization_msgs::msg::Marker::ADD;
    mesh.pose.position = msg->pose.pose.position;
    mesh.pose.orientation.w = q(0);
    mesh.pose.orientation.x = q(1);
    mesh.pose.orientation.y = q(2);
    mesh.pose.orientation.z = q(3);
    mesh.scale.x = scale_;
    mesh.scale.y = scale_;
    mesh.scale.z = scale_;

    double hue = drone_num_ > 1 ? (drone_id_ - 1) / double(drone_num_ - 1) : 0.0;
    auto color = getColor(hue, 1);
    mesh.color.r = color[0];
    mesh.color.g = color[1];
    mesh.color.b = color[2];
    mesh.color.a = color[3];
    mesh.mesh_resource = mesh_resource_;
    mesh_pub_->publish(mesh);
  }

  void cmdCb(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
    if (cmd->header.frame_id == "null") return;

    colvec q(4);
    q(0) = 1.0;
    q(1) = q(2) = q(3) = 0.0;
    if (cross_config_) applyYawOffset(q);

    visualization_msgs::msg::Marker mesh;
    mesh.header.frame_id = frame_id_;
    mesh.header.stamp = cmd->header.stamp;
    mesh.ns = "mesh";
    mesh.id = 0;
    mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh.action = visualization_msgs::msg::Marker::ADD;
    mesh.pose.position.x = cmd->position.x;
    mesh.pose.position.y = cmd->position.y;
    mesh.pose.position.z = cmd->position.z;
    mesh.pose.orientation.w = q(0);
    mesh.pose.orientation.x = q(1);
    mesh.pose.orientation.y = q(2);
    mesh.pose.orientation.z = q(3);
    mesh.scale.x = 2.0;
    mesh.scale.y = 2.0;
    mesh.scale.z = 2.0;
    mesh.color.a = color_a_;
    mesh.color.r = color_r_;
    mesh.color.g = color_g_;
    mesh.color.b = color_b_;
    mesh.mesh_resource = mesh_resource_;
    mesh_pub_->publish(mesh);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cov_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cov_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr height_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::string mesh_resource_;
  std::string frame_id_;
  double color_r_, color_g_, color_b_, color_a_, cov_scale_, scale_;
  int drone_id_, drone_num_;
  bool origin_, cross_config_, tf45_, cov_pos_, cov_vel_, cov_color_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomVisualization>());
  rclcpp::shutdown();
  return 0;
}

// ROS 2 port of plan_manage/test/rotation.cpp
//
// Standalone test that publishes three rotated quaternions as Odometry
// messages plus a SPHERE_LIST marker showing points transformed by the
// same rotations. Purely a visualization sanity-check for Euler-to-quat
// math — no planner dependencies.

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Eigen>

#include <chrono>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rotation");

  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/rotation/odom", 10);
  auto mark_pub = node->create_publisher<visualization_msgs::msg::Marker>("/rotation/point", 10);
  rclcpp::sleep_for(std::chrono::seconds(1));

  // write the euler angle represented rotation matrix
  Matrix3d Rx, Ry, Rz;
  const double a = 0.785;

  Rx << 1.0, 0.0, 0.0, 0.0, cos(a), -sin(a), 0.0, sin(a), cos(a);
  Ry << cos(a), 0.0, sin(a), 0.0, 1.0, 0.0, -sin(a), 0.0, cos(a);
  Rz << cos(a), -sin(a), 0.0, sin(a), cos(a), 0.0, 0.0, 0.0, 1.0;

  Matrix3d R1 = Rz;
  Matrix3d R2 = Rz * Ry;
  Matrix3d R3 = Rz * Ry * Rx;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = node->get_clock()->now();

  // pub R
  Quaterniond q1(R1);
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 1.0;
  odom.pose.pose.orientation.w = q1.w();
  odom.pose.pose.orientation.x = q1.x();
  odom.pose.pose.orientation.y = q1.y();
  odom.pose.pose.orientation.z = q1.z();
  odom_pub->publish(odom);

  Quaterniond q2(R2);
  odom.pose.pose.position.x = 2.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.orientation.w = q2.w();
  odom.pose.pose.orientation.x = q2.x();
  odom.pose.pose.orientation.y = q2.y();
  odom.pose.pose.orientation.z = q2.z();
  odom_pub->publish(odom);

  Quaterniond q3(R3);
  odom.pose.pose.position.x = 3.0;
  odom.pose.pose.position.y = 3.0;
  odom.pose.pose.orientation.w = q3.w();
  odom.pose.pose.orientation.x = q3.x();
  odom.pose.pose.orientation.y = q3.y();
  odom.pose.pose.orientation.z = q3.z();
  odom_pub->publish(odom);

  // test the transformed points
  Vector3d p1, p2, p3, p4;
  Vector3d p0(1, 0, 0);
  p1 = R1 * p0;
  p2 = R2 * p0;
  p3 = Ry * p0;
  p4 = Rz * Ry * p0;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = node->get_clock()->now();
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.id = 0;
  m.color.r = 1;
  m.color.a = 1;

  geometry_msgs::msg::Point p;
  p.x = p1(0);
  p.y = p1(1);
  p.z = p1(2);
  m.points.push_back(p);
  p.x = p2(0);
  p.y = p2(1);
  p.z = p2(2);
  m.points.push_back(p);

  p.x = p3(0);
  p.y = p3(1);
  p.z = p3(2);
  m.points.push_back(p);
  p.x = p4(0);
  p.y = p4(1);
  p.z = p4(2);
  m.points.push_back(p);

  mark_pub->publish(m);

  rclcpp::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
  return 0;
}

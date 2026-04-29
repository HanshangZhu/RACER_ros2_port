// ROS 2 port of plan_manage/test/pos_vel_acc.cpp
//
// Standalone viz: builds a 3-deg NonUniformBspline from a hand-coded set
// of control points and publishes position / velocity / acceleration
// SPHERE_LIST markers. Links against the ported `bspline` library.
//
// NOTE(ros2-port): upstream #includes <dyn_planner/non_uniform_bspline.h>
// which is a legacy path that never existed in the RACER tree; the actual
// header lives under <bspline/non_uniform_bspline.h>. Fixed here.

#include <rclcpp/rclcpp.hpp>

#include <bspline/non_uniform_bspline.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Eigen>

#include <chrono>
#include <vector>

using namespace fast_planner;
using std::vector;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pos_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr acc_pub_;

int pub_id;
rclcpp::Node::SharedPtr g_node;

void displaySphereList(
    vector<Eigen::Vector3d> list, double resolution, Eigen::Vector4d color, int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = g_node->get_clock()->now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  if (pub_id == 0) {
    pos_pub_->publish(mk);
  } else if (pub_id == 1) {
    vel_pub_->publish(mk);
  } else if (pub_id == 2) {
    acc_pub_->publish(mk);
  }
}

void drawBspline(NonUniformBspline bspline, double size, Eigen::Vector4d color, bool show_ctrl_pts,
    double size2, Eigen::Vector4d color2, int id1, int id2) {
  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, id1);

  if (!show_ctrl_pts) return;

  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();

  vector<Eigen::Vector3d> ctp;
  for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }
  displaySphereList(ctp, size2, color2, id2);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("fast_planner_node");

  pos_pub_ = g_node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/pos", 10);
  vel_pub_ = g_node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/vel", 10);
  acc_pub_ = g_node->create_publisher<visualization_msgs::msg::Marker>("/planning_vis/acc", 10);

  rclcpp::sleep_for(std::chrono::seconds(1));

  /* ---------- set ctrl points ---------- */
  Eigen::MatrixXd ctrl_pts(8, 3);
  ctrl_pts.row(0) = Eigen::Vector3d(0, 0, 0);
  ctrl_pts.row(1) = Eigen::Vector3d(0.7, 0.9, 0);
  ctrl_pts.row(2) = Eigen::Vector3d(0.9, 2.1, 0);
  ctrl_pts.row(3) = Eigen::Vector3d(2.3, 2.8, 0);
  ctrl_pts.row(4) = Eigen::Vector3d(3.2, 2.8, 0);

  ctrl_pts.row(5) = Eigen::Vector3d(3.7, 1.6, 0);
  ctrl_pts.row(6) = Eigen::Vector3d(5.2, 1.3, 0);
  ctrl_pts.row(7) = Eigen::Vector3d(4.3, 0.6, 0);

  /* ---------- draw pos vel acc ---------- */
  const double ts = 1.5;
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);
  NonUniformBspline vel = pos.getDerivative();
  NonUniformBspline acc = vel.getDerivative();

  pub_id = 0;
  drawBspline(
      pos, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true, 0.12, Eigen::Vector4d(0, 1, 0, 1), 0, 1);

  pub_id = 1;
  drawBspline(
      vel, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1), true, 0.12, Eigen::Vector4d(0, 0, 1, 1), 0, 1);

  pub_id = 2;
  drawBspline(
      acc, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true, 0.12, Eigen::Vector4d(0, 1, 0, 1), 0, 1);

  rclcpp::spin(g_node);
  rclcpp::shutdown();

  return 0;
}

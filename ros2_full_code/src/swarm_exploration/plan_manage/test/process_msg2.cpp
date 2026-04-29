// ROS 2 port of plan_manage/test/process_msg2.cpp
//
// Self-contained viz preprocessor (same spirit as traj_utils/process_msg but
// with FOV-aware post-processing): subscribes to planner viz markers and
// republishes styled / bridged / maze variants for benchmark overlays.

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <string>
#include <vector>

using std::string;
using std::vector;

class ProcessMsg2 : public rclcpp::Node {
public:
  ProcessMsg2() : rclcpp::Node("process_msg2") {
    alpha_ = this->declare_parameter<double>("process_msg.alpha", 0.9);
    last_yaw_ = 0.0;

    marker1_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/process_msg/marker1", 10);
    cmd_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/planning/position_cmd_vis", 10,
        std::bind(&ProcessMsg2::fovCallback, this, std::placeholders::_1));
    cmd_traj_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/planning/travel_traj", 10,
        std::bind(&ProcessMsg2::cmdTrajCallback, this, std::placeholders::_1));
    plan_traj_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/planning_vis/trajectory", 10,
        std::bind(&ProcessMsg2::planTrajCallback, this, std::placeholders::_1));
    view_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/planning_vis/viewpoints", 10,
        std::bind(&ProcessMsg2::viewCallback, this, std::placeholders::_1));
    nbvp_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/firefly/visualization_marker", 10,
        std::bind(&ProcessMsg2::nbvpCallback, this, std::placeholders::_1));

    buildCameraFov();
  }

private:
  void buildCameraFov() {
    const double vert_ang = 0.56125;
    const double hor_ang = 0.68901;
    const double cam_scale = 0.8;
    const double hor = cam_scale * std::tan(hor_ang);
    const double vert = cam_scale * std::tan(vert_ang);
    const Eigen::Vector3d origin(0, 0, 0);
    const Eigen::Vector3d left_up(cam_scale, hor, vert);
    const Eigen::Vector3d left_down(cam_scale, hor, -vert);
    const Eigen::Vector3d right_up(cam_scale, -hor, vert);
    const Eigen::Vector3d right_down(cam_scale, -hor, -vert);
    cam1_ = {origin, origin, origin, origin, left_up, right_up, right_down, left_down};
    cam2_ = {left_up, left_down, right_up, right_down, right_up, right_down, left_down, left_up};
  }

  void drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
      double line_width, const Eigen::Vector4d& color, const string& ns, int id) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->get_clock()->now();
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    mk.action = visualization_msgs::msg::Marker::DELETE;
    mk.id = id;
    mk.ns = ns;
    marker1_pub_->publish(mk);

    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = line_width;

    geometry_msgs::msg::Point pt;
    for (size_t i = 0; i < list1.size(); ++i) {
      pt.x = list1[i](0); pt.y = list1[i](1); pt.z = list1[i](2);
      mk.points.push_back(pt);
      pt.x = list2[i](0); pt.y = list2[i](1); pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
    marker1_pub_->publish(mk);
  }

  void calcNextYaw(double last_yaw, double& yaw) {
    double round_last = last_yaw;
    while (round_last < -M_PI) round_last += 2 * M_PI;
    while (round_last > M_PI) round_last -= 2 * M_PI;
    double diff = yaw - round_last;
    if (std::fabs(diff) <= M_PI) yaw = last_yaw + diff;
    else if (diff > M_PI) yaw = last_yaw + diff - 2 * M_PI;
    else yaw = last_yaw + diff + 2 * M_PI;
  }

  void fovCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (msg->points.empty()) return;
    Eigen::Vector3d p0(msg->points[0].x, msg->points[0].y, msg->points[0].z);
    last_cmd_pos_ = p0;
    if (msg->points.size() < 6) return;
    Eigen::Vector3d p1(msg->points[1].x, msg->points[1].y, msg->points[1].z);
    Eigen::Vector3d p5(msg->points[5].x, msg->points[5].y, msg->points[5].z);
    Eigen::Vector3d dir = p1 - p0 + p5 - p0;
    double tmp_yaw = std::atan2(dir[1], dir[0]);
    calcNextYaw(last_yaw_, tmp_yaw);
    double yaw = (1 - alpha_) * tmp_yaw + alpha_ * last_yaw_;
    last_yaw_ = yaw;

    Eigen::Matrix3d Rwb;
    Rwb << std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1;
    vector<Eigen::Vector3d> l1, l2;
    for (size_t i = 0; i < cam1_.size(); ++i) {
      l1.push_back(Rwb * cam1_[i] + p0);
      l2.push_back(Rwb * cam2_[i] + p0);
    }
    drawLines(l1, l2, 0.04, Eigen::Vector4d(0, 0, 0, 1), "fov", 0);
  }

  void cmdTrajCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (msg->points.empty()) return;
    visualization_msgs::msg::Marker mk = *msg;
    mk.scale.x = 0.1; mk.scale.y = 0.1; mk.scale.z = 0.1;
    if (mk.id == 2) {
      mk.color.r = 1; mk.color.g = 0; mk.color.b = 0;
      mk.ns = "classic";
      for (auto& p : mk.points) p.y = -p.y;
    }
    if (mk.id == 3) {
      mk.color.r = 0; mk.color.g = 1; mk.color.b = 0;
      mk.ns = "rapid";
    }
    if (mk.id == 4) {
      mk.color.r = 0; mk.color.g = 0; mk.color.b = 1;
      mk.ns = "propose";
    }
    marker1_pub_->publish(mk);
  }

  void planTrajCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (msg->points.empty()) return;
    visualization_msgs::msg::Marker mk = *msg;
    Eigen::Vector3d p0(mk.points[0].x, mk.points[0].y, mk.points[0].z);
    Eigen::Vector3d diff = last_cmd_pos_ - p0;
    for (auto& p : mk.points) {
      p.x += diff[0]; p.y += diff[1]; p.z += diff[2];
    }
    mk.color.a = 0.5; mk.ns = "plan_traj"; mk.id = 0;
    marker1_pub_->publish(mk);

    if (view_mk_.points.size() < 6) return;
    Eigen::Vector3d p0_next(view_mk_.points[0].x, view_mk_.points[0].y, view_mk_.points[0].z);
    Eigen::Vector3d p1(view_mk_.points[1].x, view_mk_.points[1].y, view_mk_.points[1].z);
    Eigen::Vector3d p5(view_mk_.points[5].x, view_mk_.points[5].y, view_mk_.points[5].z);
    Eigen::Vector3d dir = p1 + p5 - 2 * p0_next;
    double next_yaw = std::atan2(dir[1], dir[0]);
    Eigen::Matrix3d Rwb;
    Rwb << std::cos(next_yaw), -std::sin(next_yaw), 0, std::sin(next_yaw), std::cos(next_yaw), 0, 0, 0, 1;
    auto p_msg_end = mk.points.back();
    Eigen::Vector3d p_end(p_msg_end.x, p_msg_end.y, p_msg_end.z);
    vector<Eigen::Vector3d> l1, l2;
    for (size_t i = 0; i < cam1_.size(); ++i) {
      l1.push_back(Rwb * cam1_[i] + p_end);
      l2.push_back(Rwb * cam2_[i] + p_end);
    }
    drawLines(l1, l2, 0.04, Eigen::Vector4d(1, 0, 0, 1), "plan_traj", 1);
  }

  void viewCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (msg->ns == "global_tour" && msg->points.empty()) {
      visualization_msgs::msg::Marker mk = *msg;
      mk.ns = "plan_traj";
      marker1_pub_->publish(mk);
      mk.ns = "next_fov";
      marker1_pub_->publish(mk);
      return;
    }
    if (msg->ns != "refined_view" || msg->points.empty()) return;
    view_mk_ = *msg;
    if (view_mk_.points.size() > 16)
      view_mk_.points.erase(view_mk_.points.begin() + 16, view_mk_.points.end());
  }

  void nbvpCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    visualization_msgs::msg::Marker mk = *msg;
    mk.scale.x = 0.1; mk.scale.y = 0.1; mk.scale.z = 0.1;
    mk.color.r = 1; mk.color.g = 0; mk.color.b = 1;
    mk.ns = "nbvp";
    for (auto& p : mk.points) {
      double tx = p.x, ty = p.y;
      p.x = -ty; p.y = -tx;
    }
    marker1_pub_->publish(mk);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker1_pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr cmd_sub_, cmd_traj_sub_,
      plan_traj_sub_, view_sub_, nbvp_sub_;

  double alpha_{0.9};
  double last_yaw_{0};
  Eigen::Vector3d last_cmd_pos_{Eigen::Vector3d::Zero()};
  visualization_msgs::msg::Marker view_mk_;
  vector<Eigen::Vector3d> cam1_, cam2_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessMsg2>());
  rclcpp::shutdown();
  return 0;
}

// ROS 2 port of plan_manage/test/process_msg.cpp
//
// Benchmark helper: subscribes to /position_cmd, /sdf_map/occupancy_local,
// and /planning_vis/trajectory; republishes prettied-up markers and a
// downsampled global point cloud on /process_msg/*. Kept for parity with
// the ROS 1 build even though it isn't part of the runtime demo path.

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

using namespace std;
using std::placeholders::_1;

class ProcessMsg : public rclcpp::Node {
public:
  ProcessMsg() : rclcpp::Node("faster") {
    last_pos_.setZero();
    pts_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    // Camera FOV vertices in body frame
    const double vert_ang = 0.56125;
    const double hor_ang = 0.68901;
    const double cam_scale = 0.5;
    double hor = cam_scale * tan(hor_ang);
    double vert = cam_scale * tan(vert_ang);
    Eigen::Vector3d origin(0, 0, 0);
    Eigen::Vector3d left_up(cam_scale, hor, vert);
    Eigen::Vector3d left_down(cam_scale, hor, -vert);
    Eigen::Vector3d right_up(cam_scale, -hor, vert);
    Eigen::Vector3d right_down(cam_scale, -hor, -vert);

    cam1_.push_back(origin);
    cam2_.push_back(left_up);
    cam1_.push_back(origin);
    cam2_.push_back(left_down);
    cam1_.push_back(origin);
    cam2_.push_back(right_up);
    cam1_.push_back(origin);
    cam2_.push_back(right_down);

    cam1_.push_back(left_up);
    cam2_.push_back(right_up);
    cam1_.push_back(right_up);
    cam2_.push_back(right_down);
    cam1_.push_back(right_down);
    cam2_.push_back(left_down);
    cam1_.push_back(left_down);
    cam2_.push_back(left_up);

    distance1_ = 0.0;

    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "/position_cmd", 10, std::bind(&ProcessMsg::cmdCallback, this, _1));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/occupancy_local", 10, std::bind(&ProcessMsg::cloudCallback, this, _1));
    traj_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/planning_vis/trajectory", 10, std::bind(&ProcessMsg::trajCallback, this, _1));
    ewok_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/firefly/optimal_trajectory", 10, std::bind(&ProcessMsg::ewokCallback, this, _1));

    traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/process_msg/execute_traj", 10);
    yaw_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/process_msg/execute_yaw", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/process_msg/global_cloud", 10);
    ewok_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/process_msg/ewok", 10);
    traj_pub2_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/planning/travel_traj", 10);
  }

private:
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
      double line_width, const Eigen::Vector4d& color, int id) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->now();
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    mk.action = visualization_msgs::msg::Marker::DELETE;
    mk.id = id;
    yaw_pub_->publish(mk);

    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = line_width;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
    yaw_pub_->publish(mk);
  }

  void displayTrajWithColor(
      vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->now();
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::DELETE;
    mk.id = id;

    traj_pub_->publish(mk);

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
    for (int i = 0; i < int(path.size()); i++) {
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);
      mk.points.push_back(pt);
    }
    traj_pub_->publish(mk);
  }

  void cmdCallback(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "start");
    Eigen::Vector3d pt, v;
    pt(0) = msg->position.x;
    pt(1) = msg->position.y;
    pt(2) = msg->position.z;
    v(0) = msg->velocity.x;
    v(1) = msg->velocity.y;
    v(2) = msg->velocity.z;

    if (traj_.size() > 0) {
      distance1_ += (pt - traj_.back()).norm();
    }

    traj_.push_back(pt);
    vel_.push_back(v);
    displayTrajWithColor(traj_, 0.1, Eigen::Vector4d(1, 0, 0, 1), 0);

    double phi = msg->yaw;
    yaw_.push_back(phi);
    yaw1_.clear();
    yaw2_.clear();
    for (int k = 0; k < 4; ++k) {
      int idx = (int)yaw_.size() - 1 - 30 * k;
      if (idx < 0) continue;
      double phi_k = yaw_[idx];
      Eigen::Vector3d pt_k = traj_[idx];
      Eigen::Matrix3d Rwb;
      Rwb << cos(phi_k), -sin(phi_k), 0, sin(phi_k), cos(phi_k), 0, 0, 0, 1;
      for (size_t i = 0; i < cam1_.size(); ++i) {
        auto p1 = Rwb * cam1_[i] + pt_k;
        auto p2 = Rwb * cam2_[i] + pt_k;
        yaw1_.push_back(p1);
        yaw2_.push_back(p2);
      }
    }

    if (v.norm() < 1e-3) {
      RCLCPP_INFO(this->get_logger(), "end, distance: %lf", distance1_);
    }
  }

  void trajCallback(const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
    if (msg->id != 399) return;

    visualization_msgs::msg::Marker mk = *msg;
    mk.color.a = 0.3;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.id = 1;

    traj_pub2_->publish(mk);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    static int msg_num = 0;
    if (++msg_num % 10 != 0) return;

    pcl::PointCloud<pcl::PointXYZ> pts2, pts3;
    pcl::fromROSMsg(*msg, pts2);

    for (size_t i = 0; i < pts2.points.size(); ++i) {
      pts_->push_back(pts2[i]);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pts_);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(pts3);

    pts_->points = pts3.points;
    pts_->width = pts_->points.size();
    pts_->height = 1;
    pts_->is_dense = true;
    pts_->header.frame_id = "world";

    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(*pts_, cloud);
    cloud_pub_->publish(cloud);
  }

  void ewokCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    if (msg->markers.empty()) return;
    auto marker = msg->markers[0];
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    ewok_pub_->publish(marker);
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr traj_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr ewok_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub2_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ewok_pub_;

  Eigen::Vector3d last_pos_;
  vector<Eigen::Vector3d> traj_, vel_, yaw1_, yaw2_;
  vector<double> yaw_;
  vector<Eigen::Vector3d> cam1_, cam2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pts_;
  double distance1_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessMsg>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// ROS 2 port of ground_node.cpp — centralized aggregator that visualizes
// swarm frontiers / grid tours and republishes the latest HGrid.
//
// Uses MultiThreadedExecutor because FastExplorationManager internally
// calls out to LKH services; the ground node itself doesn't issue those
// calls directly, but shares the same manager API.

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <active_perception/frontier_finder.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <exploration_manager/msg/grid_tour.hpp>
#include <exploration_manager/msg/h_grid.hpp>
// See fast_exploration_fsm.cpp for the ViewConstraint redefinition rationale.
#define _TRAJ_VISIBILITY_H_
namespace fast_planner {
struct VisiblePair;
struct ViewConstraint;
}  // namespace fast_planner
#include <traj_utils/planning_visualization.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace fast_planner {

class ExplGroundNode : public rclcpp::Node {
public:
  ExplGroundNode() : rclcpp::Node("ground_node") {
  }

  void init() {
    auto self = shared_from_this();
    expl_ = std::make_shared<FastExplorationManager>();
    expl_->initialize(self);

    frontier_timer_ = this->create_wall_timer(
        500ms, std::bind(&ExplGroundNode::frontierCallback, this));

    frontier_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/expl_ground_node/frontier", 10);
    grid_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/expl_ground_node/grid", 10);

    grid_tour_sub_ = this->create_subscription<exploration_manager::msg::GridTour>(
        "/swarm_expl/grid_tour_recv", 10,
        std::bind(&ExplGroundNode::gridTourCallback, this, _1));
    hgrid_sub_ = this->create_subscription<exploration_manager::msg::HGrid>(
        "/swarm_expl/hgrid_recv", 10, std::bind(&ExplGroundNode::HGridCallback, this, _1));

    if (!this->has_parameter("exploration.drone_num")) {
      this->declare_parameter<int>("exploration.drone_num", 4);
    }
    this->get_parameter("exploration.drone_num", drone_num_);

    grid_tour_stamp_.assign(drone_num_, 0.0);
    hgrid_stamp_ = 0.0;
  }

private:
  void frontierCallback() {
    static int delay = 0;
    if (++delay < 5) return;

    auto ft = expl_->frontier_finder_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();

    std::vector<std::vector<Eigen::Vector3d>> ftrs;
    ft->getFrontiers(ftrs);

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (auto& cluster : ftrs) {
      for (auto& cell : cluster) {
        pt.x = cell[0];
        pt.y = cell[1];
        pt.z = cell[2];
        cloud.push_back(pt);
      }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    frontier_pub_->publish(cloud_msg);
  }

  void HGridCallback(const exploration_manager::msg::HGrid::ConstSharedPtr msg) {
    if (msg->stamp <= hgrid_stamp_ + 1e-4) return;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->now();
    mk.id = 0;
    mk.ns = "hgrid";
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 0.5;

    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;

    mk.action = visualization_msgs::msg::Marker::DELETE;
    grid_pub_->publish(mk);

    for (size_t i = 0; i < msg->points1.size(); ++i) {
      mk.points.push_back(msg->points1[i]);
      mk.points.push_back(msg->points2[i]);
    }

    mk.action = visualization_msgs::msg::Marker::ADD;
    grid_pub_->publish(mk);
    rclcpp::sleep_for(500us);

    hgrid_stamp_ = msg->stamp;
  }

  void gridTourCallback(const exploration_manager::msg::GridTour::ConstSharedPtr msg) {
    if (msg->stamp <= grid_tour_stamp_[msg->drone_id - 1] + 1e-4) return;
    if (msg->points.empty()) return;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->now();
    mk.id = msg->drone_id;
    mk.ns = "grid tour";
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    auto color = PlanningVisualization::getColor((msg->drone_id - 1) / double(drone_num_));

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;

    mk.action = visualization_msgs::msg::Marker::DELETE;
    grid_pub_->publish(mk);

    for (size_t i = 0; i + 1 < msg->points.size(); ++i) {
      mk.points.push_back(msg->points[i]);
      mk.points.push_back(msg->points[i + 1]);
    }

    mk.action = visualization_msgs::msg::Marker::ADD;
    grid_pub_->publish(mk);
    rclcpp::sleep_for(500us);

    grid_tour_stamp_[msg->drone_id - 1] = msg->stamp;
  }

  // Data ------------------------
  std::shared_ptr<FastExplorationManager> expl_;

  rclcpp::TimerBase::SharedPtr frontier_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_pub_;
  rclcpp::Subscription<exploration_manager::msg::GridTour>::SharedPtr grid_tour_sub_;
  rclcpp::Subscription<exploration_manager::msg::HGrid>::SharedPtr hgrid_sub_;

  std::vector<double> grid_tour_stamp_;
  double hgrid_stamp_ = 0.0;
  int drone_num_ = 4;
};

}  // namespace fast_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fast_planner::ExplGroundNode>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

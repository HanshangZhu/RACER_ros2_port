// ROS 2 port of multi_map_visualization.
// Subscribes to aggregated multi-drone 2D / 3D occupancy deltas and re-publishes
// them as (a) the merged MultiOccupancyGrid and (b) a world-frame PointCloud
// for RViz visualization.

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <pose_utils/pose_utils.h>

#include <multi_map_server/msg/multi_occupancy_grid.hpp>
#include <multi_map_server/msg/multi_sparse_map3_d.hpp>
#include <multi_map_server/Map2D.h>
#include <multi_map_server/Map3D.h>

using std::string;
using std::vector;

class MultiMapVisualization : public rclcpp::Node {
public:
  MultiMapVisualization() : rclcpp::Node("multi_map_visualization") {
    pub1_ = this->create_publisher<multi_map_server::msg::MultiOccupancyGrid>(
        "maps2d", rclcpp::QoS(1).transient_local());
    pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "map3d", rclcpp::QoS(1).transient_local());

    sub1_ = this->create_subscription<multi_map_server::msg::MultiOccupancyGrid>(
        "dmaps2d", 1,
        std::bind(&MultiMapVisualization::maps2dCallback, this, std::placeholders::_1));
    sub2_ = this->create_subscription<multi_map_server::msg::MultiSparseMap3D>(
        "dmaps3d", 1,
        std::bind(&MultiMapVisualization::maps3dCallback, this, std::placeholders::_1));
  }

private:
  void maps2dCallback(const multi_map_server::msg::MultiOccupancyGrid::SharedPtr msg) {
    // Merge map
    maps2d_.resize(msg->maps.size(), Map2D(4));
    for (unsigned int k = 0; k < msg->maps.size(); k++) {
      maps2d_[k].SetClock(this->get_clock());
      maps2d_[k].Replace(msg->maps[k]);
    }
    origins2d_ = msg->origins;
    // Assemble and publish map
    multi_map_server::msg::MultiOccupancyGrid m;
    m.maps.resize(maps2d_.size());
    m.origins.resize(maps2d_.size());
    for (unsigned int k = 0; k < maps2d_.size(); k++) {
      m.maps[k] = maps2d_[k].GetMap();
      m.origins[k] = origins2d_[k];
    }
    pub1_->publish(m);
  }

  void maps3dCallback(const multi_map_server::msg::MultiSparseMap3D::SharedPtr msg) {
    // Update incremental map
    maps3d_.resize(msg->maps.size());
    for (unsigned int k = 0; k < msg->maps.size(); k++) {
      maps3d_[k].SetClock(this->get_clock());
      maps3d_[k].UnpackMsg(msg->maps[k]);
    }
    origins3d_ = msg->origins;
    // Publish
    sensor_msgs::msg::PointCloud m;
    for (unsigned int k = 0; k < msg->maps.size(); k++) {
      arma::colvec po(6);
      po(0) = origins3d_[k].position.x;
      po(1) = origins3d_[k].position.y;
      po(2) = origins3d_[k].position.z;
      arma::colvec poq(4);
      poq(0) = origins3d_[k].orientation.w;
      poq(1) = origins3d_[k].orientation.x;
      poq(2) = origins3d_[k].orientation.y;
      poq(3) = origins3d_[k].orientation.z;
      po.rows(3, 5) = R_to_ypr(quaternion_to_R(poq));
      arma::colvec tpo = po.rows(0, 2);
      arma::mat Rpo = ypr_to_R(po.rows(3, 5));
      vector<arma::colvec> pts = maps3d_[k].GetOccupancyWorldFrame(OCCUPIED);
      for (unsigned int i = 0; i < pts.size(); i++) {
        arma::colvec pt = Rpo * pts[i] + tpo;
        geometry_msgs::msg::Point32 _pt;
        _pt.x = pt(0);
        _pt.y = pt(1);
        _pt.z = pt(2);
        m.points.push_back(_pt);
      }
    }
    // Publish
    m.header.stamp = this->get_clock()->now();
    m.header.frame_id = string("map");
    pub2_->publish(m);
  }

  rclcpp::Publisher<multi_map_server::msg::MultiOccupancyGrid>::SharedPtr pub1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub2_;
  rclcpp::Subscription<multi_map_server::msg::MultiOccupancyGrid>::SharedPtr sub1_;
  rclcpp::Subscription<multi_map_server::msg::MultiSparseMap3D>::SharedPtr sub2_;

  vector<Map2D> maps2d_;
  vector<geometry_msgs::msg::Pose> origins2d_;
  vector<Map3D> maps3d_;
  vector<geometry_msgs::msg::Pose> origins3d_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMapVisualization>());
  rclcpp::shutdown();
  return 0;
}

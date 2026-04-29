#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

using namespace std;
using std::placeholders::_1;

class ProcessMsg : public rclcpp::Node {
public:
  ProcessMsg() : Node("process_msg") {
    map_origin_ << -20, -10, -1;
    last_cloud_time_ = this->now();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", 10,
        std::bind(&ProcessMsg::cloudCallback, this, _1));
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/process_msg/global_cloud", 10);

    RCLCPP_WARN(this->get_logger(), "[process_msg]: ready.");
  }

private:
  void inflatePoint(const Eigen::Vector3d& pt, int step, vector<Eigen::Vector3d>& pts) {
    const double res = 0.1;
    int num = 0;
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          pts[num++] = Eigen::Vector3d(pt(0) + x * res, pt(1) + y * res, pt(2) + z * res);
        }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    auto tn = this->now();
    if ((tn - last_cloud_time_).seconds() < 5) {
      return;
    }
    last_cloud_time_ = tn;

    pcl::PointCloud<pcl::PointXYZ> pts;
    pcl::PointCloud<pcl::PointXYZ> pts2;
    pcl::fromROSMsg(*msg, pts);
    vector<Eigen::Vector3d> inf_pts(27);

    for (size_t i = 0; i < pts.points.size(); ++i) {
      Eigen::Vector3d pt;
      pt(0) = pts[i].x;
      pt(1) = pts[i].y;
      pt(2) = pts[i].z;
      for (int j = 0; j < 3; ++j)
        pt(j) = floor((pt(j) - map_origin_(j)) * 10);
      for (int j = 0; j < 3; ++j)
        pt(j) = (pt(j) + 0.5) * 0.1 + map_origin_(j);
      inflatePoint(pt, 1, inf_pts);
      for (auto pi : inf_pts) {
        pcl::PointXYZ pj;
        pj.x = pi(0);
        pj.y = pi(1);
        pj.z = pi(2);
        pts2.points.push_back(pj);
      }
    }
    pts2.width = pts2.points.size();
    pts2.height = 1;
    pts2.is_dense = true;
    pts2.header.frame_id = "world";

    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(pts2, cloud);
    cloud_pub_->publish(cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  Eigen::Vector3d map_origin_;
  rclcpp::Time last_cloud_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessMsg>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

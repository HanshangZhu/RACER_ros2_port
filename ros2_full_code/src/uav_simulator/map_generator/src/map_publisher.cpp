#include <memory>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapPublisher : public rclcpp::Node {
public:
  MapPublisher(const std::string& file_name) : rclcpp::Node("map_pub") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", qos);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "can't read file %s", file_name.c_str());
      rclcpp::shutdown();
      return;
    }

    for (double x = -7; x <= 7; x += 0.1) {
      for (double y = -15; y <= 15; y += 0.1) {
        cloud.push_back(pcl::PointXYZ(x, y, 0));
      }
    }

    pcl::toROSMsg(cloud, msg_);
    msg_.header.frame_id = "world";

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
        [this]() { cloud_pub_->publish(msg_); });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 msg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (argc < 2) {
    std::cerr << "usage: map_pub <pcd_file>" << std::endl;
    return 1;
  }
  rclcpp::spin(std::make_shared<MapPublisher>(argv[1]));
  rclcpp::shutdown();
  return 0;
}

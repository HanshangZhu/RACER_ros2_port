#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapRecorder : public rclcpp::Node {
public:
  MapRecorder() : rclcpp::Node("map_recorder") {
    out_path_ = this->declare_parameter<std::string>("out_path", "/tmp/tmp.pcd");
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", 10,
        std::bind(&MapRecorder::cloudCb, this, std::placeholders::_1));
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::io::savePCDFileASCII(out_path_, cloud);
    RCLCPP_WARN(this->get_logger(), "[Map Recorder]: saved %s (%zu points)", out_path_.c_str(),
        cloud.size());
    rclcpp::shutdown();
  }

  std::string out_path_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapRecorder>());
  rclcpp::shutdown();
  return 0;
}

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class RandomPilarMap : public rclcpp::Node {
public:
  RandomPilarMap() : rclcpp::Node("pilar_map") {
    x_size_ = this->declare_parameter<double>("map.x_size", 50.0);
    y_size_ = this->declare_parameter<double>("map.y_size", 50.0);
    obs_num_ = this->declare_parameter<int>("map.obs_num", 30);
    retry_num_ = this->declare_parameter<int>("map.retry_num", 30);
    resolution_ = this->declare_parameter<double>("map.resolution", 0.1);
    min_r_ = this->declare_parameter<double>("map.min_r", 0.3);
    max_r_ = this->declare_parameter<double>("map.max_r", 0.8);
    fb_min_x_ = this->declare_parameter<double>("map.fb_min_x", 0.8);
    fb_max_x_ = this->declare_parameter<double>("map.fb_max_x", 0.8);
    fb_min_y_ = this->declare_parameter<double>("map.fb_min_y", 0.8);
    fb_max_y_ = this->declare_parameter<double>("map.fb_max_y", 0.8);
    int seed = this->declare_parameter<int>("map.seed", -1);

    min_x_ = -x_size_ / 2.0;
    max_x_ = +x_size_ / 2.0;
    min_y_ = -y_size_ / 2.0;
    max_y_ = +y_size_ / 2.0;
    obs_num_ = std::min(obs_num_, static_cast<int>(x_size_) * 10);

    std::random_device rd;
    if (seed < 0) seed = rd() % INT32_MAX;
    eng_ = std::default_random_engine(seed);
    RCLCPP_INFO(this->get_logger(), "map seed: %d", seed);

    all_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", 1);

    generatePilarMap();
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
        std::bind(&RandomPilarMap::republish, this));
  }

private:
  void generatePilarMap() {
    std::uniform_real_distribution<double> rand_x(min_x_, max_x_);
    std::uniform_real_distribution<double> rand_y(min_y_, max_y_);
    std::uniform_real_distribution<double> rand_w(min_r_, max_r_);
    pcl::PointXYZ pt_random;

    for (double x = -16; x <= 16; x += resolution_) {
      for (double y = -16; y <= 16; y += resolution_) {
        pt_random.x = x;
        pt_random.y = y;
        pt_random.z = 0.0;
        map_cloud_.push_back(pt_random);
      }
    }

    int gen_num = 0;
    int fail_num = 0;
    while (gen_num < obs_num_) {
      double x = rand_x(eng_);
      double y = rand_y(eng_);
      double w = rand_w(eng_);
      if (x < fb_max_x_ && x > fb_min_x_ && y < fb_max_y_ && y > fb_min_y_) continue;

      x = std::floor(x / resolution_) * resolution_ + resolution_ / 2.0;
      y = std::floor(y / resolution_) * resolution_ + resolution_ / 2.0;
      Eigen::Vector3d pt(x, y, w);
      bool occupied = false;
      for (const auto& pre : points_) {
        if ((pre - pt).head<2>().norm() < 1.414 * (pre[2] + pt[2]) + 2.0) {
          occupied = true;
          break;
        }
      }
      if (occupied) {
        if (++fail_num > retry_num_) break;
        continue;
      }
      points_.push_back(pt);

      int wid_num = static_cast<int>(std::ceil(w / resolution_));
      for (int r = -wid_num; r < wid_num; r++) {
        for (int s = -wid_num; s < wid_num; s++) {
          int hei_num = static_cast<int>(std::ceil(3.0 / resolution_));
          for (int t = -10; t < hei_num; t++) {
            pt_random.x = x + r * resolution_ + 1e-2;
            pt_random.y = y + s * resolution_ + 1e-2;
            pt_random.z = (t + 0.5) * resolution_ + 1e-2;
            map_cloud_.push_back(pt_random);
          }
        }
      }
      gen_num++;
      fail_num = 0;
    }

    map_cloud_.width = map_cloud_.points.size();
    map_cloud_.height = 1;
    map_cloud_.is_dense = true;
    RCLCPP_WARN(this->get_logger(), "Finished generate random map");
  }

  void republish() {
    pcl::toROSMsg(map_cloud_, map_msg_);
    map_msg_.header.frame_id = "world";
    all_map_pub_->publish(map_msg_);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 map_msg_;
  pcl::PointCloud<pcl::PointXYZ> map_cloud_;
  std::vector<Eigen::Vector3d> points_;
  std::default_random_engine eng_;

  int obs_num_{30}, retry_num_{30};
  double x_size_, y_size_, min_r_, max_r_, resolution_;
  double fb_min_x_, fb_max_x_, fb_min_y_, fb_max_y_;
  double min_x_, max_x_, min_y_, max_y_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomPilarMap>());
  rclcpp::shutdown();
  return 0;
}

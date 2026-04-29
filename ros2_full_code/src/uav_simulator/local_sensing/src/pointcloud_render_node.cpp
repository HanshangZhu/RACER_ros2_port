// ROS 2 Humble port of local_sensing/pointcloud_render_node.
// Non-CUDA path only. dynamic_reconfigure has been dropped in favor of
// declare_parameter + add_on_set_parameters_callback (see
// so3_disturbance_generator for the gold-standard pattern).

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class PclRenderNode : public rclcpp::Node {
public:
  PclRenderNode() : rclcpp::Node("pcl_render") {
    // Parameters (formerly dynamic_reconfigure + nh.getParam).
    width_ = this->declare_parameter<int>("cam_width", 640);
    height_ = this->declare_parameter<int>("cam_height", 480);
    fx_ = this->declare_parameter<double>("cam_fx", 387.229);
    fy_ = this->declare_parameter<double>("cam_fy", 387.229);
    cx_ = this->declare_parameter<double>("cam_cx", 321.046);
    cy_ = this->declare_parameter<double>("cam_cy", 243.449);
    sensing_horizon_ = this->declare_parameter<double>("sensing_horizon", 5.0);
    sensing_rate_ = this->declare_parameter<double>("sensing_rate", 30.0);
    estimation_rate_ = this->declare_parameter<double>("estimation_rate", 30.0);

    // Parameter callback replaces dynamic_reconfigure.
    param_cb_ = this->add_on_set_parameters_callback(
        std::bind(&PclRenderNode::onParamSet, this, _1));

    // cam-in-body fixed transform (z_cam = x_body, x_cam = -y_body, y_cam = -z_body).
    cam02body_ << 0.0, 0.0, 1.0, 0.0,
                  -1.0, 0.0, 0.0, 0.0,
                  0.0, -1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;
    cam2world_ = Eigen::Matrix4d::Identity();

    // Subscribers.
    global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "global_map", rclcpp::QoS(1).transient_local(),
        std::bind(&PclRenderNode::pointCloudCallBack, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(50),
        std::bind(&PclRenderNode::odometryCallback, this, _1));

    // Publishers.
    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/pcl_render_node/depth", rclcpp::QoS(1000));
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pcl_render_node/sensor_pose", rclcpp::QoS(1000));

    // Timers (formerly ros::Timer).
    const double sensing_dt = 1.0 / std::max(1e-3, sensing_rate_);
    const double est_dt = 1.0 / std::max(1e-3, estimation_rate_);
    auto to_chrono = [](double s) {
      return std::chrono::nanoseconds(static_cast<int64_t>(s * 1e9));
    };
    local_sensing_timer_ = this->create_wall_timer(
        to_chrono(sensing_dt),
        std::bind(&PclRenderNode::renderSensedPoints, this));
    estimation_timer_ = this->create_wall_timer(
        to_chrono(est_dt),
        std::bind(&PclRenderNode::pubCameraPose, this));

    last_odom_stamp_ = this->get_clock()->now();
  }

private:
  // ---- Parameter callback (replaces dynamic_reconfigure) ----
  rcl_interfaces::msg::SetParametersResult onParamSet(
      const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      const auto& n = p.get_name();
      if (n == "cam_width") width_ = p.as_int();
      else if (n == "cam_height") height_ = p.as_int();
      else if (n == "cam_fx") fx_ = p.as_double();
      else if (n == "cam_fy") fy_ = p.as_double();
      else if (n == "cam_cx") cx_ = p.as_double();
      else if (n == "cam_cy") cy_ = p.as_double();
      else if (n == "sensing_horizon") sensing_horizon_ = p.as_double();
      else if (n == "sensing_rate") sensing_rate_ = p.as_double();
      else if (n == "estimation_rate") estimation_rate_ = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // ---- Subscribers ----
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    has_odom_ = true;
    odom_ = *msg;

    Eigen::Matrix4d Pose_receive = Eigen::Matrix4d::Identity();
    Eigen::Vector3d req_pos;
    Eigen::Quaterniond req_q;
    req_pos.x() = msg->pose.pose.position.x;
    req_pos.y() = msg->pose.pose.position.y;
    req_pos.z() = msg->pose.pose.position.z;
    req_q.x() = msg->pose.pose.orientation.x;
    req_q.y() = msg->pose.pose.orientation.y;
    req_q.z() = msg->pose.pose.orientation.z;
    req_q.w() = msg->pose.pose.orientation.w;
    Pose_receive.block<3, 3>(0, 0) = req_q.toRotationMatrix();
    Pose_receive(0, 3) = req_pos.x();
    Pose_receive(1, 3) = req_pos.y();
    Pose_receive(2, 3) = req_pos.z();

    cam2world_ = Pose_receive * cam02body_;
    cam2world_quat_ = Eigen::Quaterniond(cam2world_.block<3, 3>(0, 0));
    last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
    last_pose_world_ = req_pos;
  }

  void pointCloudCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (has_global_map_) return;
    RCLCPP_WARN(this->get_logger(), "Global Pointcloud received..");
    pcl::fromROSMsg(*msg, cloud_in_);
    RCLCPP_INFO(this->get_logger(),
                "global map has points: %zu.", cloud_in_.points.size());
    has_global_map_ = true;
  }

  // ---- Timer callbacks ----
  void pubCameraPose() {
    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = odom_.header;
    camera_pose.header.frame_id = "map";
    camera_pose.pose.position.x = cam2world_(0, 3);
    camera_pose.pose.position.y = cam2world_(1, 3);
    camera_pose.pose.position.z = cam2world_(2, 3);
    camera_pose.pose.orientation.w = cam2world_quat_.w();
    camera_pose.pose.orientation.x = cam2world_quat_.x();
    camera_pose.pose.orientation.y = cam2world_quat_.y();
    camera_pose.pose.orientation.z = cam2world_quat_.z();
    pub_pose_->publish(camera_pose);
  }

  void renderSensedPoints() {
    if (!has_global_map_ || !has_odom_) return;
    renderDepth();
  }

  void renderDepth() {
    depth_mat_ = cv::Mat::zeros(height_, width_, CV_32FC1);

    const Eigen::Matrix4d Tcw = cam2world_.inverse();
    const Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
    const Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);
    const Eigen::Vector3d pos = cam2world_.block<3, 1>(0, 3);

    for (const auto& pt : cloud_in_.points) {
      Eigen::Vector3d pw(pt.x, pt.y, pt.z);
      if ((pos - pw).norm() > sensing_horizon_) continue;

      Eigen::Vector3d pc = Rcw * pw + tcw;
      if (pc[2] <= 0.0) continue;

      float projected_x = static_cast<float>(pc[0] / pc[2] * fx_ + cx_);
      float projected_y = static_cast<float>(pc[1] / pc[2] * fy_ + cy_);
      if (projected_x < 0 || projected_x >= width_ ||
          projected_y < 0 || projected_y >= height_)
        continue;

      float dist = static_cast<float>(pc[2]);
      int r = static_cast<int>(0.0573 * fx_ / dist + 0.5);
      int min_x = std::max(int(projected_x - r), 0);
      int max_x = std::min(int(projected_x + r), width_ - 1);
      int min_y = std::max(int(projected_y - r), 0);
      int max_y = std::min(int(projected_y + r), height_ - 1);

      for (int to_x = min_x; to_x <= max_x; ++to_x) {
        for (int to_y = min_y; to_y <= max_y; ++to_y) {
          float value = depth_mat_.at<float>(to_y, to_x);
          if (value < 1e-3f) {
            depth_mat_.at<float>(to_y, to_x) = dist;
          } else {
            depth_mat_.at<float>(to_y, to_x) = std::min(value, dist);
          }
        }
      }
    }

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = last_odom_stamp_;
    out_msg.header.frame_id = "world";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image = depth_mat_.clone();
    pub_depth_->publish(*out_msg.toImageMsg());
  }

  // ---- State ----
  int width_{0}, height_{0};
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  double sensing_horizon_{5.0}, sensing_rate_{30.0}, estimation_rate_{30.0};

  bool has_global_map_{false};
  bool has_odom_{false};

  Eigen::Matrix4d cam02body_;
  Eigen::Matrix4d cam2world_;
  Eigen::Quaterniond cam2world_quat_{1, 0, 0, 0};
  Eigen::Vector3d last_pose_world_{0, 0, 0};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Odometry odom_;
  pcl::PointCloud<pcl::PointXYZ> cloud_in_;
  cv::Mat depth_mat_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::TimerBase::SharedPtr local_sensing_timer_;
  rclcpp::TimerBase::SharedPtr estimation_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclRenderNode>());
  rclcpp::shutdown();
  return 0;
}

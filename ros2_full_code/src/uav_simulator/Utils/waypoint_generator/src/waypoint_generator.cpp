#include <deque>
#include <memory>
#include <sstream>
#include <string>

#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sample_waypoints.h"

using bfmt = boost::format;

class WaypointGenerator : public rclcpp::Node {
public:
  WaypointGenerator() : rclcpp::Node("waypoint_generator") {
    waypoint_type_ = this->declare_parameter<std::string>("waypoint_type", "manual");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&WaypointGenerator::odom_cb, this, std::placeholders::_1));
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", qos, std::bind(&WaypointGenerator::goal_cb, this, std::placeholders::_1));
    sub_trig_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "traj_start_trigger", qos,
        std::bind(&WaypointGenerator::trigger_cb, this, std::placeholders::_1));

    pub_wp_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
    pub_wp_vis_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

    trigged_time_ = rclcpp::Time(0);
  }

private:
  void publish_waypoints() {
    waypoints_.header.frame_id = "world";
    waypoints_.header.stamp = this->now();
    pub_wp_->publish(waypoints_);
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom_.header;
    init_pose.pose = odom_.pose.pose;
    waypoints_.poses.insert(waypoints_.poses.begin(), init_pose);
    waypoints_.poses.clear();
  }

  void publish_waypoints_vis() {
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = "world";
    poseArray.header.stamp = this->now();

    poseArray.poses.push_back(odom_.pose.pose);
    for (auto& p : waypoints_.poses) {
      poseArray.poses.push_back(p.pose);
    }
    pub_wp_vis_->publish(poseArray);
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    is_odom_ready_ = true;
    odom_ = *msg;

    if (!waypoint_segments_.empty()) {
      auto expected_time = rclcpp::Time(waypoint_segments_.front().header.stamp);
      if (rclcpp::Time(odom_.header.stamp) >= expected_time) {
        waypoints_ = waypoint_segments_.front();

        std::stringstream ss;
        ss << bfmt("Series send %.3f from start:\n") % trigged_time_.seconds();
        for (auto& ps : waypoints_.poses) {
          ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") % ps.pose.position.x %
                  ps.pose.position.y % ps.pose.position.z % ps.pose.orientation.w %
                  ps.pose.orientation.x % ps.pose.orientation.y % ps.pose.orientation.z
             << std::endl;
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        publish_waypoints_vis();
        publish_waypoints();
        waypoint_segments_.pop_front();
      }
    }
  }

  void goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    trigged_time_ = this->now();
    this->get_parameter("waypoint_type", waypoint_type_);

    if (waypoint_type_ == "circle") {
      waypoints_ = circle();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "eight") {
      waypoints_ = eight();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "point") {
      waypoints_ = point();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "manual-lonely-waypoint") {
      if (msg->pose.position.z > -0.1) {
        waypoints_.poses.clear();
        waypoints_.poses.push_back(*msg);
        publish_waypoints_vis();
        publish_waypoints();
      } else {
        RCLCPP_WARN(this->get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
      }
    } else {
      if (msg->pose.position.z > 0) {
        geometry_msgs::msg::PoseStamped pt = *msg;
        if (waypoint_type_ == "noyaw") {
          double yaw = tf2::getYaw(odom_.pose.pose.orientation);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pt.pose.orientation = tf2::toMsg(q);
        }
        waypoints_.poses.push_back(pt);
        publish_waypoints_vis();
      } else if (msg->pose.position.z > -1.0) {
        if (!waypoints_.poses.empty()) {
          waypoints_.poses.pop_back();
        }
        publish_waypoints_vis();
      } else {
        if (!waypoints_.poses.empty()) {
          publish_waypoints_vis();
          publish_waypoints();
        }
      }
    }
  }

  void trigger_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    (void)msg;
    if (!is_odom_ready_) {
      RCLCPP_ERROR(this->get_logger(), "[waypoint_generator] No odom!");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "[waypoint_generator] Trigger!");
    trigged_time_ = odom_.header.stamp;
    this->get_parameter("waypoint_type", waypoint_type_);
    RCLCPP_INFO(this->get_logger(), "Pattern %s generated!", waypoint_type_.c_str());
    if (waypoint_type_ == "free" || waypoint_type_ == "point") {
      waypoints_ = point();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "circle") {
      waypoints_ = circle();
      publish_waypoints_vis();
      publish_waypoints();
    } else if (waypoint_type_ == "eight") {
      waypoints_ = eight();
      publish_waypoints_vis();
      publish_waypoints();
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_trig_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_wp_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_wp_vis_;

  std::string waypoint_type_{"manual"};
  bool is_odom_ready_{false};
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Path waypoints_;
  std::deque<nav_msgs::msg::Path> waypoint_segments_;
  rclcpp::Time trigged_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointGenerator>());
  rclcpp::shutdown();
  return 0;
}

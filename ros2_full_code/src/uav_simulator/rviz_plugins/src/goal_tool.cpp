// ROS 2 port of rviz_plugins/goal_tool.cpp — Goal3DTool.

#include "goal_tool.h"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace rviz_plugins {

Goal3DTool::Goal3DTool() : Pose3DTool() {
  shortcut_key_ = 'g';
  topic_property_ = new rviz_common::properties::StringProperty(
      "Topic", "goal",
      "The topic on which to publish navigation goals.",
      getPropertyContainer(), SLOT(updateTopic()), this);
}

void Goal3DTool::onInitialize() {
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  auto ros_abstraction = context_->getRosNodeAbstraction().lock();
  if (ros_abstraction) {
    node_ = ros_abstraction->get_raw_node();
  }
  updateTopic();
}

void Goal3DTool::updateTopic() {
  if (!node_) return;
  pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_property_->getStdString(), 1);
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta) {
  if (!node_ || !pub_) return;
  const std::string fixed_frame = context_->getFixedFrame().toStdString();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->get_clock()->now();
  goal.header.frame_id = fixed_frame;
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  RCLCPP_INFO(node_->get_logger(),
      "Setting 3D goal: frame=%s pos=(%.3f, %.3f, %.3f) yaw=%.3f rad",
      fixed_frame.c_str(), x, y, z, theta);
  pub_->publish(goal);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Goal3DTool, rviz_common::Tool)

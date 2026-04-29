// ROS 2 port of rviz_plugins/goal_tool.h — Goal3DTool.

#pragma once

#ifndef Q_MOC_RUN
#include <QObject>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "pose_tool.h"
#endif

namespace rviz_plugins {

class Goal3DTool : public Pose3DTool {
  Q_OBJECT
public:
  Goal3DTool();
  ~Goal3DTool() override = default;
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double z, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rviz_common::properties::StringProperty* topic_property_{nullptr};
};

}  // namespace rviz_plugins

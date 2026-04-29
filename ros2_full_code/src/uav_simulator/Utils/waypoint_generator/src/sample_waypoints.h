#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

inline nav_msgs::msg::Path point() {
  nav_msgs::msg::Path waypoints;
  geometry_msgs::msg::PoseStamped pt;
  pt.pose.orientation = yaw_to_quat(0.0);

  double h = 1.0;
  double scale = 7.0;

  pt.pose.position.y = scale * 0.0;
  pt.pose.position.x = scale * 2.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.0;
  pt.pose.position.x = scale * 4.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.25;
  pt.pose.position.x = scale * 5.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.5;
  pt.pose.position.x = scale * 5.3;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.75;
  pt.pose.position.x = scale * 5.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 1.0;
  pt.pose.position.x = scale * 4.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 1.0;
  pt.pose.position.x = scale * 2.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 1.0;
  pt.pose.position.x = scale * 0.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  return waypoints;
}

inline nav_msgs::msg::Path circle() {
  double h = 1.0;
  double scale = 5.0;
  nav_msgs::msg::Path waypoints;
  geometry_msgs::msg::PoseStamped pt;
  pt.pose.orientation = yaw_to_quat(0.0);

  auto push = [&](double x, double y) {
    pt.pose.position.x = x;
    pt.pose.position.y = y;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
  };

  push(2.5 * scale, -1.2 * scale);
  push(5.0 * scale, -2.4 * scale);
  push(5.0 * scale, 0.0);
  push(2.5 * scale, -1.2 * scale);
  push(0.0, -2.4 * scale);
  push(0.0, 0.0);
  push(2.5 * scale, -1.2 * scale);
  push(5.0 * scale, -2.4 * scale);
  push(5.0 * scale, 0.0);
  push(2.5 * scale, -1.2 * scale);
  push(0.0, -2.4 * scale);
  push(0.0, 0.0);

  return waypoints;
}

inline nav_msgs::msg::Path eight() {
  double offset_x = 0.0;
  double offset_y = 0.0;
  double r = 10.0;
  double h = 2.0;
  nav_msgs::msg::Path waypoints;
  geometry_msgs::msg::PoseStamped pt;
  pt.pose.orientation = yaw_to_quat(0.0);

  auto push = [&](double x, double y, double z) {
    pt.pose.position.x = x;
    pt.pose.position.y = y;
    pt.pose.position.z = z;
    waypoints.poses.push_back(pt);
  };

  for (int i = 0; i < 1; ++i) {
    push(r + offset_x, -r + offset_y, h / 2);
    push(r * 2 + offset_x * 2, 0, h);
    push(r * 3 + offset_x * 3, r, h / 2);
    push(r * 4 + offset_x * 4, 0, h);
    push(r * 3 + offset_x * 3, -r, h / 2);
    push(r * 2 + offset_x * 2, 0, h);
    push(r + offset_x * 2, r, h / 2);
    push(0 + offset_x, 0, h);
    push(r + offset_x, -r, h / 2 * 3);
    push(r * 2 + offset_x * 2, 0, h);
    push(r * 3 + offset_x * 3, r, h / 2 * 3);
    push(r * 4 + offset_x * 4, 0, h);
    push(r * 3 + offset_x * 3, -r, h / 2 * 3);
    push(r * 2 + offset_x * 2, 0, h);
    push(r + offset_x, r + offset_y, h / 2 * 3);
    push(0, 0, h);
  }
  return waypoints;
}

#endif

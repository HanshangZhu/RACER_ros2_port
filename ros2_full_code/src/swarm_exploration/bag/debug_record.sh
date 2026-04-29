#!/usr/bin/env bash
# ROS 2 equivalent of swarm_exploration/bag/debug_record.sh
# Records hardware-side VINS/depth topics for offline debugging.
set -e
exec ros2 bag record \
  /vins_estimator/imu_propagate \
  /camera/depth/image_rect_raw \
  /vins_estimator/camera_pose

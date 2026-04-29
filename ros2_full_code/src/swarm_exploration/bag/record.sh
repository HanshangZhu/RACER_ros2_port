#!/usr/bin/env bash
# ROS 2 equivalent of swarm_exploration/bag/record.sh
# Records camera pose, map, range, topo path, trajectory, and planner state.
set -e
exec ros2 bag record \
  /pcl_render_node/camera_pose \
  /sdf_map/occupancy_inflate \
  /sdf_map/update_range \
  /planning_vis/topo_path \
  /planning_vis/trajectory \
  /planning/state

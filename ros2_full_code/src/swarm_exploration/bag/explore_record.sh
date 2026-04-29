#!/usr/bin/env bash
# ROS 2 equivalent of swarm_exploration/bag/explore_record.sh
# Records single-drone exploration topics: camera, map, range, path, traj, cmd.
set -e
exec ros2 bag record \
  /sdf_map/occupancy_all_1 \
  /sdf_map/occupancy_local_1 \
  /planning_vis/trajectory_1 \
  /planning/travel_traj_1 \
  /planning_vis/frontier_1 \
  /planning/position_cmd_vis_1 \
  /planning_vis/viewpoints_1 \
  /planning/pos_cmd_1

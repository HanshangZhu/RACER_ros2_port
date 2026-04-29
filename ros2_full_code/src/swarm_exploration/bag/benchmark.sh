#!/usr/bin/env bash
# ROS 2 equivalent of swarm_exploration/bag/benchmark.sh
# Records the 4-drone swarm-exploration benchmark topics.
set -e
exec ros2 bag record \
  /sdf_map/occupancy_all_1 /planning_vis/trajectory_1 /planning/travel_traj_1 \
  /planning/position_cmd_vis_1 /planning_vis/viewpoints_1 /odom_visualization_1/robot \
  /planning_vis/trajectory_2 /planning/travel_traj_2 /planning/position_cmd_vis_2 \
  /planning_vis/viewpoints_2 /odom_visualization_2/robot \
  /planning_vis/trajectory_3 /planning/travel_traj_3 /planning/position_cmd_vis_3 \
  /planning_vis/viewpoints_3 /odom_visualization_3/robot \
  /planning_vis/trajectory_4 /planning/travel_traj_4 /planning/position_cmd_vis_4 \
  /planning_vis/viewpoints_4 /odom_visualization_4/robot

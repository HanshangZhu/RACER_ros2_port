#!/usr/bin/env bash
# Trigger the RACER swarm exploration FSM out of its INIT / "wait for trigger"
# state by publishing a pose on /move_base_simple/goal.
#
# Usage:
#     ./scripts/trigger_swarm.sh              # default: (0, 0, 1) — quadrotor
#     ./scripts/trigger_swarm.sh 1.0 2.0 1.0  # custom x y z
#     ./scripts/trigger_swarm.sh --quadruped  # (0, 0, 0.30) — Go1/Go2 stand height
#     ./scripts/trigger_swarm.sh --quadruped 3.0 0.0
#
# ROS 1 equivalent: clicking "2D Nav Goal" in RViz. In the RACER FSM, the pose
# itself is ignored — only receipt of one PoseStamped on /move_base_simple/goal
# flips `fd_->trigger_ = true` and transitions every drone from INIT to
# PLAN_TRAJ (see fast_exploration_fsm.cpp triggerCallback).
set -euo pipefail

DEFAULT_Z=1.0
if [[ "${1:-}" == "--quadruped" ]]; then
    DEFAULT_Z=0.30
    shift
fi

X=${1:-0.0}
Y=${2:-0.0}
Z=${3:-${DEFAULT_Z}}

exec ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'world'},
  pose:   {position: {x: ${X}, y: ${Y}, z: ${Z}},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
}"

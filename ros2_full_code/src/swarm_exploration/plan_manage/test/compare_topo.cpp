// TODO(ros2-port): legacy topology-planning benchmark test. Not ported.
//
// Depends on FastPlannerManager APIs that were deliberately trimmed during
// Wave 6 of the ROS 2 port (these were exploration-manager-specific and the
// swarm demo does not exercise them):
//   - FastPlannerManager::benckmarkReplan(...)
//   - FastPlannerManager::updateTrajectoryInfo()
//   - FastPlannerManager::traj_manager_.global_traj_
// and the corresponding TopologyPRM paths (topo_select_paths_, topo_traj_pos2_)
// through PlanningVisualization.
//
// Original ROS 1 source: swarm_exploration/plan_manage/test/compare_topo.cpp.
// To restore this benchmark, re-port the trimmed planner_manager methods and
// wire the three drawTopoPaths / drawBsplinesPhase2 helpers (already exported
// in traj_utils) together, then replace this stub with a ROS 2 node equivalent.

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("compare_topo_stub");
  RCLCPP_ERROR(node->get_logger(),
      "compare_topo is a stubbed test binary; see src for port blocker details.");
  rclcpp::shutdown();
  return 0;
}

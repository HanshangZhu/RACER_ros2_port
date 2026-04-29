// TODO(ros2-port): not ported.
//
// This test exercises BsplineOptimizer + SDFMap + PlanningVisualization
// against a hand-built occupancy grid. The ROS 2 port renamed several of the
// APIs it uses (SDFMap::init → SDFMap::initMap, PlanningVisualization ctor
// signature, BsplineOptimizer::HARD_CONSTRAINT enum value) and the draw helper
// overloads shifted, so a faithful port requires a careful adapter pass.
//
// Blocker is mechanical, not architectural — restore by matching the current
// ported signatures in:
//   plan_env/include/plan_env/sdf_map.h
//   bspline_opt/include/bspline_opt/bspline_optimizer.h
//   traj_utils/include/traj_utils/planning_visualization.h
// then replace this stub with a ROS 2 rewrite.

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_collision_cost_stub");
  RCLCPP_ERROR(node->get_logger(),
      "test_collision_cost is stubbed; see src for port blocker details.");
  rclcpp::shutdown();
  return 0;
}

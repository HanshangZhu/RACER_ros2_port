// TODO(ros2-port): not ported.
//
// Depends on the grad_traj_optimization package (display.h, grad_traj_optimizer.h,
// polynomial_traj.hpp) which is NOT in the RACER repo — it was an external
// dependency from a sibling workspace used only for this standalone benchmark.
// Not available to port against.
//
// Original ROS 1 source: swarm_exploration/plan_manage/test/opti_node.cpp.
// To restore, first vendor or re-port the grad_traj_optimization package.

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("opti_node_stub");
  RCLCPP_ERROR(node->get_logger(),
      "opti_node is stubbed; depends on the external grad_traj_optimization package.");
  rclcpp::shutdown();
  return 0;
}

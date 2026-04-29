// TODO(ros2-port): not ported.
//
// This test binary depends on plan_manage/local_explore_fsm.h, whose
// implementation file was intentionally excluded during Wave 6 of the ROS 2
// port (header-only placeholder with no .cpp in the ROS 1 tree either).
// The swarm demo does not use local_explore_fsm — exploration_manager's
// fast_exploration_fsm is the canonical FSM.
//
// To restore, port plan_manage/local_explore_fsm.{h,cpp} (if it ever shipped
// an implementation) then rewrite this node-style test against the ported
// FSM class.

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("local_explore_fsm_stub");
  RCLCPP_ERROR(node->get_logger(),
      "local_explore_fsm test is stubbed; see src for port blocker details.");
  rclcpp::shutdown();
  return 0;
}

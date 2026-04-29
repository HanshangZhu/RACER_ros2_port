// ROS 2 port of exploration_node.cpp — entry point for the per-drone FSM.
//
// A MultiThreadedExecutor is REQUIRED: FastExplorationManager's planner
// entry points call out to the LKH TSP/ACVRP services synchronously
// (future.wait_for(..).get()). With a single-threaded executor the service
// response would be parked behind the planner call in the same thread and
// the wait would deadlock. Reentrant callback groups on the service clients
// plus the multi-threaded executor let the response be delivered on a
// second thread while the FSM tick thread is blocked.

#include <rclcpp/rclcpp.hpp>
#include <exploration_manager/fast_exploration_fsm.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<fast_planner::FastExplorationFSM>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

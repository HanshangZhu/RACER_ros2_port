// ROS 2 port of exploration_manager/fast_exploration_fsm.h
//
// Wave 7:
//   - FastExplorationFSM is now a rclcpp::Node subclass.
//   - All timers are wall timers with no TimerEvent argument.
//   - Requires MultiThreadedExecutor because the underlying
//     FastExplorationManager blocks on LKH service responses
//     inside planner entry points.

#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline/msg/bspline.hpp>
#include <exploration_manager/msg/drone_state.hpp>
#include <exploration_manager/msg/grid_tour.hpp>
#include <exploration_manager/msg/h_grid.hpp>
#include <exploration_manager/msg/pair_opt.hpp>
#include <exploration_manager/msg/pair_opt_response.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class FastExplorationFSM : public rclcpp::Node {
public:
  FastExplorationFSM();
  ~FastExplorationFSM() override = default;

  // Two-phase init: rclcpp::Node's shared_from_this() is not valid inside the
  // constructor, so heavy setup (which hands `this` to submodules that cache
  // a SharedPtr) happens here after make_shared<>().
  void init();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  /* ROS callbacks (no TimerEvent in ROS 2) */
  void FSMCallback();
  void safetyCallback();
  void frontierCallback();
  void triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Swarm
  void droneStateTimerCallback();
  void droneStateMsgCallback(const exploration_manager::msg::DroneState::ConstSharedPtr msg);
  void optTimerCallback();
  void optMsgCallback(const exploration_manager::msg::PairOpt::ConstSharedPtr msg);
  void optResMsgCallback(const exploration_manager::msg::PairOptResponse::ConstSharedPtr msg);
  void swarmTrajCallback(const bspline::msg::Bspline::ConstSharedPtr msg);
  void swarmTrajTimerCallback();

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  // Callback groups split the timer/sub fleet: FSM ticks and subs run in a
  // single MutuallyExclusive group so they are serialised (matching ROS 1
  // single-threaded semantics for the FSM). Service clients stay out of these
  // groups so the LKH wait does not block the FSM when combined with a
  // MultiThreadedExecutor.
  rclcpp::CallbackGroup::SharedPtr fsm_cbgrp_;

  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr bspline_pub_;

  // Swarm state
  rclcpp::Publisher<exploration_manager::msg::DroneState>::SharedPtr drone_state_pub_;
  rclcpp::Publisher<exploration_manager::msg::PairOpt>::SharedPtr opt_pub_;
  rclcpp::Publisher<exploration_manager::msg::PairOptResponse>::SharedPtr opt_res_pub_;
  rclcpp::Publisher<bspline::msg::Bspline>::SharedPtr swarm_traj_pub_;
  rclcpp::Publisher<exploration_manager::msg::GridTour>::SharedPtr grid_tour_pub_;
  rclcpp::Publisher<exploration_manager::msg::HGrid>::SharedPtr hgrid_pub_;

  rclcpp::Subscription<exploration_manager::msg::DroneState>::SharedPtr drone_state_sub_;
  rclcpp::Subscription<exploration_manager::msg::PairOpt>::SharedPtr opt_sub_;
  rclcpp::Subscription<exploration_manager::msg::PairOptResponse>::SharedPtr opt_res_sub_;
  rclcpp::Subscription<bspline::msg::Bspline>::SharedPtr swarm_traj_sub_;

  rclcpp::TimerBase::SharedPtr drone_state_timer_, opt_timer_, swarm_traj_timer_;
};

}  // namespace fast_planner

#endif

// ROS 2 port of exploration_manager/fast_exploration_fsm.cpp
//
// Wave 7: FastExplorationFSM is an rclcpp::Node. All timers are wall
// timers, Subscriber callbacks take ConstSharedPtr, ros::Time is replaced
// with rclcpp::Time.
//
// Callback-group strategy:
//   - fsm_cbgrp_ (MutuallyExclusive): holds the FSM tick timer, the safety
//     timer, the frontier timer, odom/trigger subs, the optimization/swarm
//     timers + their subs. This serialises FSM state mutation, matching the
//     ROS 1 single-threaded semantics the upstream code assumed.
//   - The LKH service clients (inside FastExplorationManager) live in a
//     separate Reentrant group and are only driven via async_send_request
//     + future.wait_for, so the MultiThreadedExecutor can service the
//     response while the FSM thread is blocked.

#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/fast_exploration_manager.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/hgrid.h>
#include <active_perception/perception_utils.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
// NOTE(ros2-port): active_perception/traj_visibility.h and
// bspline_opt/bspline_optimizer.h both fully define `struct ViewConstraint`
// in the fast_planner namespace. planner_manager.h drags in bspline_opt's
// copy, planning_visualization.h drags in active_perception's copy through
// its #include of traj_visibility.h. The two definitions are byte-identical
// but C++ still rejects the redefinition. Same workaround as plan_manage's
// plan_container.hpp: short-circuit traj_visibility.h by pre-defining its
// include guard before including planning_visualization.h. The two
// visualization functions that actually use ViewConstraint/VisiblePair are
// never called from exploration_manager.
#define _TRAJ_VISIBILITY_H_
namespace fast_planner {
struct VisiblePair;
struct ViewConstraint;
}  // namespace fast_planner
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/msg/grid_tour.hpp>
#include <exploration_manager/msg/h_grid.hpp>

#include <chrono>
#include <fstream>
#include <functional>

using Eigen::Vector4d;
using namespace std::chrono_literals;
using std::placeholders::_1;

namespace fast_planner {

namespace {
template <typename T>
void declare_or_get(rclcpp::Node* node, const std::string& name, T& value,
    const T& default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  node->get_parameter(name, value);
}
}  // namespace

FastExplorationFSM::FastExplorationFSM() : rclcpp::Node("exploration_node") {
}

void FastExplorationFSM::init() {
  fp_ = std::make_shared<FSMParam>();
  fd_ = std::make_shared<FSMData>();

  /* FSM param */
  declare_or_get<double>(this, "fsm.thresh_replan1", fp_->replan_thresh1_, -1.0);
  declare_or_get<double>(this, "fsm.thresh_replan2", fp_->replan_thresh2_, -1.0);
  declare_or_get<double>(this, "fsm.thresh_replan3", fp_->replan_thresh3_, -1.0);
  declare_or_get<double>(this, "fsm.replan_time", fp_->replan_time_, -1.0);
  declare_or_get<double>(this, "fsm.attempt_interval", fp_->attempt_interval_, 0.2);
  declare_or_get<double>(this, "fsm.pair_opt_interval", fp_->pair_opt_interval_, 1.0);
  declare_or_get<int>(this, "fsm.repeat_send_num", fp_->repeat_send_num_, 10);

  /* Initialize main modules */
  auto self = shared_from_this();
  expl_manager_ = std::make_shared<FastExplorationManager>();
  expl_manager_->initialize(self);
  visualization_ = std::make_shared<PlanningVisualization>(self);

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH",
    "IDLE" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->avoid_collision_ = false;
  fd_->go_back_ = false;
  fd_->fsm_init_time_ = this->now();
  fd_->last_check_frontier_time_ = this->now();

  fsm_cbgrp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = fsm_cbgrp_;

  /* Timers */
  exec_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&FastExplorationFSM::FSMCallback, this),
      fsm_cbgrp_);
  safety_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FastExplorationFSM::safetyCallback, this),
      fsm_cbgrp_);
  frontier_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&FastExplorationFSM::frontierCallback, this),
      fsm_cbgrp_);

  /* Triggers and odom */
  trigger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 1, std::bind(&FastExplorationFSM::triggerCallback, this, _1),
      sub_opts);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 1, std::bind(&FastExplorationFSM::odometryCallback, this, _1), sub_opts);

  replan_pub_ = this->create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  new_pub_ = this->create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  bspline_pub_ = this->create_publisher<bspline::msg::Bspline>("/planning/bspline", 10);

  /* Swarm */
  drone_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(40), std::bind(&FastExplorationFSM::droneStateTimerCallback, this),
      fsm_cbgrp_);
  drone_state_pub_ = this->create_publisher<exploration_manager::msg::DroneState>(
      "/swarm_expl/drone_state_send", 10);
  drone_state_sub_ = this->create_subscription<exploration_manager::msg::DroneState>(
      "/swarm_expl/drone_state_recv", 10,
      std::bind(&FastExplorationFSM::droneStateMsgCallback, this, _1), sub_opts);

  opt_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&FastExplorationFSM::optTimerCallback, this), fsm_cbgrp_);
  opt_pub_ =
      this->create_publisher<exploration_manager::msg::PairOpt>("/swarm_expl/pair_opt_send", 10);
  opt_sub_ = this->create_subscription<exploration_manager::msg::PairOpt>(
      "/swarm_expl/pair_opt_recv", 100,
      std::bind(&FastExplorationFSM::optMsgCallback, this, _1), sub_opts);

  opt_res_pub_ = this->create_publisher<exploration_manager::msg::PairOptResponse>(
      "/swarm_expl/pair_opt_res_send", 10);
  opt_res_sub_ = this->create_subscription<exploration_manager::msg::PairOptResponse>(
      "/swarm_expl/pair_opt_res_recv", 100,
      std::bind(&FastExplorationFSM::optResMsgCallback, this, _1), sub_opts);

  swarm_traj_pub_ =
      this->create_publisher<bspline::msg::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ = this->create_subscription<bspline::msg::Bspline>("/planning/swarm_traj_recv",
      100, std::bind(&FastExplorationFSM::swarmTrajCallback, this, _1), sub_opts);
  swarm_traj_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&FastExplorationFSM::swarmTrajTimerCallback, this), fsm_cbgrp_);

  hgrid_pub_ =
      this->create_publisher<exploration_manager::msg::HGrid>("/swarm_expl/hgrid_send", 10);
  grid_tour_pub_ = this->create_publisher<exploration_manager::msg::GridTour>(
      "/swarm_expl/grid_tour_send", 10);
}

int FastExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

void FastExplorationFSM::FSMCallback() {
  auto clk = *this->get_clock();
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clk, 1000,
      "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      if (!fd_->have_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "no odom");
        return;
      }
      if ((this->now() - fd_->fsm_init_time_).seconds() < 2.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "wait for init");
        return;
      }
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "wait for trigger.");
      break;
    }

    case FINISH: {
      RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "finish exploration.");
      break;
    }

    case IDLE: {
      double check_interval = (this->now() - fd_->last_check_frontier_time_).seconds();
      if (check_interval > 100.0) {
        RCLCPP_WARN(this->get_logger(), "Go back to (0,0,1)");
        expl_manager_->ed_->next_pos_ = fd_->start_pos_;
        expl_manager_->ed_->next_yaw_ = 0.0;

        fd_->go_back_ = true;
        transitState(PLAN_TRAJ, "FSM");
      }
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();
        fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
      } else {
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (this->now() - info->start_time_).seconds() + fp_->replan_time_;
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }
      replan_pub_->publish(std_msgs::msg::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == FAIL) {
        fd_->static_state_ = true;
        RCLCPP_WARN(this->get_logger(), "Plan fail");
      } else if (res == NO_GRID) {
        fd_->static_state_ = true;
        fd_->last_check_frontier_time_ = this->now();
        RCLCPP_WARN(this->get_logger(), "No grid");
        transitState(IDLE, "FSM");
        visualize(1);
      }
      break;
    }

    case PUB_TRAJ: {
      // newest_traj_.start_time is builtin_interfaces/Time in ROS 2.
      rclcpp::Time traj_start(fd_->newest_traj_.start_time, this->get_clock()->get_clock_type());
      double dt = (this->now() - traj_start).seconds();
      if (dt > 0) {
        bspline_pub_->publish(fd_->newest_traj_);
        fd_->static_state_ = false;

        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_->publish(fd_->newest_traj_);

        std::thread vis_thread(&FastExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = this->now();
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).seconds();

      if (!fd_->go_back_) {
        bool need_replan = false;
        if (t_cur > fp_->replan_thresh2_ &&
            expl_manager_->frontier_finder_->isFrontierCovered()) {
          RCLCPP_WARN(
              this->get_logger(), "Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
          RCLCPP_WARN(this->get_logger(),
              "Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fp_->replan_thresh3_) {
          RCLCPP_WARN(this->get_logger(),
              "Replan: periodic call=======================================");
          need_replan = true;
        }

        if (need_replan) {
          if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
            std::thread vis_thread(&FastExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            fd_->last_check_frontier_time_ = this->now();
            transitState(IDLE, "FSM");
            RCLCPP_WARN(this->get_logger(), "Idle since no frontier is detected");
            fd_->static_state_ = true;
            replan_pub_->publish(std_msgs::msg::Empty());
            visualize(1);
          }
        }
      } else {
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_->publish(std_msgs::msg::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fp_->replan_thresh3_ || info->duration_ - t_cur < fp_->replan_thresh1_) {
          replan_pub_->publish(std_msgs::msg::Empty());
          transitState(PLAN_TRAJ, "FSM");
          std::thread vis_thread(&FastExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  rclcpp::Time time_r = this->now() + rclcpp::Duration::from_seconds(fp_->replan_time_);

  int res;
  if (fd_->avoid_collision_ || fd_->go_back_) {
    res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
        fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  } else {
    res = expl_manager_->planExploreMotion(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (this->now() - time_r).seconds() > 0 ? this->now() : time_r;

    bspline::msg::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;  // rclcpp::Time -> builtin_interfaces/Time
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize(int content) {
  auto info = &planner_manager_->local_data_;
  auto ed_ptr = expl_manager_->ed_;

  if (content == 1) {
    static int last_ftr_num = 0;
    for (size_t i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4), "frontier", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();
  } else if (content == 2) {
    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (size_t i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();
    }

    auto grid_tour = expl_manager_->ed_->grid_tour_;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 6);

    exploration_manager::msg::GridTour tour;
    for (size_t i = 0; i < grid_tour.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = grid_tour[i][0];
      point.y = grid_tour[i][1];
      point.z = grid_tour[i][2];
      tour.points.push_back(point);
    }
    tour.drone_id = expl_manager_->ep_->drone_id_;
    tour.stamp = this->now().seconds();
    grid_tour_pub_->publish(tour);

    visualization_->drawBspline(info->position_traj_, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        false, 0.15, Vector4d(1, 1, 0, 1));
  }
}

void FastExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
  }
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);
}

void FastExplorationFSM::frontierCallback() {
  if (state_ == WAIT_TRIGGER) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    expl_manager_->updateFrontierStruct(fd_->odom_pos_);

    std::cout << "odom: " << fd_->odom_pos_.transpose() << std::endl;
    vector<int> tmp_id1;
    vector<vector<int>> tmp_id2;
    bool status = expl_manager_->findGlobalTourOfGrid(
        { fd_->odom_pos_ }, { fd_->odom_vel_ }, tmp_id1, tmp_id2, true);

    for (size_t i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
          visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4), "frontier", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }
    if (status)
      visualize(2);
    else
      visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  }
}

void FastExplorationFSM::triggerCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr /*msg*/) {
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  std::cout << "Triggered!" << std::endl;
  fd_->start_pos_ = fd_->odom_pos_;
  RCLCPP_WARN_STREAM(this->get_logger(), "Start expl pos: " << fd_->start_pos_.transpose());

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    transitState(PLAN_TRAJ, "triggerCallback");
  } else {
    transitState(FINISH, "triggerCallback");
  }
}

void FastExplorationFSM::safetyCallback() {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      RCLCPP_WARN(
          this->get_logger(), "Replan: collision detected==================================");
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = this->now();
  }
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  RCLCPP_INFO_STREAM(this->get_logger(),
      "[" + pos_call + "]: Drone "
          << getId()
          << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

void FastExplorationFSM::droneStateTimerCallback() {
  exploration_manager::msg::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (this->now() - info->start_time_).seconds();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = this->now().seconds();
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto id : state.grid_ids_) msg.grid_ids.push_back(id);
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;

  drone_state_pub_->publish(msg);
}

void FastExplorationFSM::droneStateMsgCallback(
    const exploration_manager::msg::DroneState::ConstSharedPtr msg) {
  if (msg->drone_id == getId()) return;

  Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  (void)msg_pos;

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  drone_state.grid_ids_.clear();
  for (auto id : msg->grid_ids) drone_state.grid_ids_.push_back(id);
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;
}

void FastExplorationFSM::optTimerCallback() {
  if (state_ == INIT) return;

  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];
  bool urgent = state1.grid_ids_.empty();
  (void)urgent;
  auto tn = this->now().seconds();

  if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_) return;

  int select_id = -1;
  double max_interval = -1.0;
  for (size_t i = 0; i < states.size(); ++i) {
    if (int(i) + 1 <= getId()) continue;
    if (tn - states[i].stamp_ > 0.2) continue;
    if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_) continue;
    if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_) continue;
    if (states[i].grid_ids_.size() + state1.grid_ids_.size() == 0) continue;

    double interval = tn - states[i].recent_interact_time_;
    if (interval <= max_interval) continue;
    select_id = int(i) + 1;
    max_interval = interval;
  }
  if (select_id == -1) return;

  std::cout << "\nSelect: " << select_id << std::endl;
  RCLCPP_WARN(this->get_logger(), "Pair opt %d & %d", getId(), select_id);

  std::unordered_map<int, char> opt_ids_map;
  auto& state2 = states[select_id - 1];
  for (auto id : state1.grid_ids_) opt_ids_map[id] = 1;
  for (auto id : state2.grid_ids_) opt_ids_map[id] = 1;
  vector<int> opt_ids;
  for (auto pair : opt_ids_map) opt_ids.push_back(pair.first);

  std::cout << "Pair Opt id: ";
  for (auto id : opt_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);
  std::cout << "Missed: ";
  for (auto id : missed) std::cout << id << ", ";
  std::cout << "" << std::endl;
  opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());

  vector<Eigen::Vector3d> positions = { state1.pos_, state2.pos_ };
  vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0) };
  vector<int> first_ids1, second_ids1, first_ids2, second_ids2;
  if (state_ != WAIT_TRIGGER) {
    expl_manager_->hgrid_->getConsistentGrid(
        state1.grid_ids_, state1.grid_ids_, first_ids1, second_ids1);
    expl_manager_->hgrid_->getConsistentGrid(
        state2.grid_ids_, state2.grid_ids_, first_ids2, second_ids2);
  }

  auto t1 = this->now();

  vector<int> ego_ids, other_ids;
  expl_manager_->allocateGrids(positions, velocities, { first_ids1, first_ids2 },
      { second_ids1, second_ids2 }, opt_ids, ego_ids, other_ids);

  double alloc_time = (this->now() - t1).seconds();

  double prev_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.grid_ids_, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double prev_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.grid_ids_, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double cur_app1 = expl_manager_->computeGridPathCost(state1.pos_, ego_ids, first_ids1,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  double cur_app2 = expl_manager_->computeGridPathCost(state2.pos_, other_ids, first_ids2,
      { first_ids1, first_ids2 }, { second_ids1, second_ids2 }, true);
  if (cur_app1 + cur_app2 > prev_app1 + prev_app2 + 0.1) {
    RCLCPP_ERROR(this->get_logger(), "Larger cost after reallocation");
    if (state_ != WAIT_TRIGGER) {
      return;
    }
  }

  if (!state1.grid_ids_.empty() && !ego_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state1.grid_ids_[0], ego_ids[0])) {
    RCLCPP_ERROR(this->get_logger(), "Path 1 inconsistent");
  }
  if (!state2.grid_ids_.empty() && !other_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state2.grid_ids_[0], other_ids[0])) {
    RCLCPP_ERROR(this->get_logger(), "Path 2 inconsistent");
  }

  exploration_manager::msg::PairOpt opt;
  opt.from_drone_id = getId();
  opt.to_drone_id = select_id;
  opt.stamp = tn;
  for (auto id : ego_ids) opt.ego_ids.push_back(id);
  for (auto id : other_ids) opt.other_ids.push_back(id);

  for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_pub_->publish(opt);

  RCLCPP_WARN(this->get_logger(), "Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf",
      getId(), select_id, this->now().seconds() - tn, alloc_time);

  auto ed = expl_manager_->ed_;
  ed->ego_ids_ = ego_ids;
  ed->other_ids_ = other_ids;
  ed->pair_opt_stamp_ = opt.stamp;
  ed->wait_response_ = true;
  state1.recent_attempt_time_ = tn;
}

void FastExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
  std::unordered_map<int, char> active_map;
  for (auto ativ : actives) {
    active_map[ativ] = 1;
  }

  for (auto state : expl_manager_->ed_->swarm_state_) {
    for (auto id : state.grid_ids_) {
      if (active_map.find(id) != active_map.end()) {
        active_map.erase(id);
      }
    }
  }

  missed.clear();
  for (auto p : active_map) {
    missed.push_back(p.first);
  }
}

void FastExplorationFSM::optMsgCallback(
    const exploration_manager::msg::PairOpt::ConstSharedPtr msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto& state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
  auto& state2 = expl_manager_->ed_->swarm_state_[getId() - 1];

  exploration_manager::msg::PairOptResponse response;
  response.from_drone_id = msg->to_drone_id;
  response.to_drone_id = msg->from_drone_id;
  response.stamp = msg->stamp;

  if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
    RCLCPP_WARN(this->get_logger(), "Reject frequent attempt");
    response.status = 2;
  } else {
    response.status = 1;

    state1.grid_ids_.clear();
    state2.grid_ids_.clear();
    for (auto id : msg->ego_ids) state1.grid_ids_.push_back(id);
    for (auto id : msg->other_ids) state2.grid_ids_.push_back(id);

    state1.recent_interact_time_ = msg->stamp;
    state2.recent_attempt_time_ = this->now().seconds();
    expl_manager_->ed_->reallocated_ = true;

    if (state_ == IDLE && !state2.grid_ids_.empty()) {
      transitState(PLAN_TRAJ, "optMsgCallback");
      RCLCPP_WARN(this->get_logger(), "Restart after opt!");
    }
  }
  for (int i = 0; i < fp_->repeat_send_num_; ++i) opt_res_pub_->publish(response);
}

void FastExplorationFSM::optResMsgCallback(
    const exploration_manager::msg::PairOptResponse::ConstSharedPtr msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4)
    return;
  expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto ed = expl_manager_->ed_;
  if (!ed->wait_response_ || std::fabs(ed->pair_opt_stamp_ - msg->stamp) > 1e-5) return;

  ed->wait_response_ = false;
  RCLCPP_WARN(this->get_logger(), "get response %d", int(msg->status));

  if (msg->status != 1) return;

  auto& state1 = ed->swarm_state_[getId() - 1];
  auto& state2 = ed->swarm_state_[msg->from_drone_id - 1];
  state1.grid_ids_ = ed->ego_ids_;
  state2.grid_ids_ = ed->other_ids_;
  state2.recent_interact_time_ = this->now().seconds();
  ed->reallocated_ = true;

  if (state_ == IDLE && !state1.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optResMsgCallback");
    RCLCPP_WARN(this->get_logger(), "Restart after opt!");
  }
}

void FastExplorationFSM::swarmTrajCallback(const bspline::msg::Bspline::ConstSharedPtr msg) {
  auto& sdat = planner_manager_->swarm_traj_data_;

  if (msg->drone_id == sdat.drone_id_) return;

  // msg->start_time is builtin_interfaces/Time; swarm_trajs_ stores a double.
  rclcpp::Time msg_start(msg->start_time, this->get_clock()->get_clock_type());
  double msg_start_sec = msg_start.seconds();
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      msg_start_sec <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg_start_sec;
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      RCLCPP_ERROR(this->get_logger(), "Drone %d collide with drone %d.", sdat.drone_id_,
          msg->drone_id);
      fd_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

void FastExplorationFSM::swarmTrajTimerCallback() {
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_->publish(fd_->newest_traj_);
  } else if (state_ == WAIT_TRIGGER) {
    bspline::msg::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = this->now();
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_->publish(bspline);
  }
}

}  // namespace fast_planner

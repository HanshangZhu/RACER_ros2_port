// ROS 2 port of plan_manage/traj_server.cpp
//
// Wraps the B-spline trajectory consumer in a single rclcpp::Node. Uses wall
// timers for the control (100 Hz) and visualization (4 Hz) loops and plain
// publisher/subscriber pairs for topics. A MultiThreadedExecutor is NOT
// required — there are no service calls out of callbacks.

#include "bspline/non_uniform_bspline.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <bspline/msg/bspline.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>
#include <traj_utils/planning_visualization.h>

#include <Eigen/Eigen>
#include <chrono>
#include <memory>
#include <vector>

using fast_planner::NonUniformBspline;
using fast_planner::PerceptionUtils;
using fast_planner::PlanningVisualization;
using fast_planner::PolynomialTraj;

class TrajServerNode : public rclcpp::Node {
public:
  TrajServerNode() : rclcpp::Node("traj_server") {
    // Parameters (dotted names for ROS 2). Upstream used: traj_server/pub_traj_id,
    // fsm/replan_time, loop_correction/isLoopCorrection, traj_server/drone_id,
    // traj_server/drone_num. Mapped 1:1 with '.' instead of '/'.
    pub_traj_id_ = this->declare_parameter<int>("traj_server.pub_traj_id", -1);
    replan_time_ = this->declare_parameter<double>("fsm.replan_time", 0.1);
    isLoopCorrection = this->declare_parameter<bool>("loop_correction.isLoopCorrection", false);
    drone_id_ = this->declare_parameter<int>("traj_server.drone_id", 1);
    drone_num_ = this->declare_parameter<int>("traj_server.drone_num", 1);

    start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    start_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    end_time = start_time;
    last_time = start_time;

    traj_id_ = 0;
    pub_traj_id_state_ = 0;
    receive_traj_ = false;
    energy = 0.0;
    traj_duration_ = 0.0;

    R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
    T_loop = Eigen::Vector3d(0, 0, 0);

    // Control parameters
    cmd_.kx = { 5.7f, 5.7f, 6.2f };
    cmd_.kv = { 3.4f, 3.4f, 4.0f };
    cmd_.header.frame_id = "world";
    cmd_.trajectory_flag =
        quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd_.trajectory_id = traj_id_;
    cmd_.position.x = -4.5;
    cmd_.position.y = -3.0;
    cmd_.position.z = 0.0;
  }

  void init() {
    auto self = shared_from_this();

    bspline_sub_ = this->create_subscription<bspline::msg::Bspline>("planning/bspline", 10,
        std::bind(&TrajServerNode::bsplineCallback, this, std::placeholders::_1));
    replan_sub_ = this->create_subscription<std_msgs::msg::Empty>("planning/replan", 10,
        std::bind(&TrajServerNode::replanCallback, this, std::placeholders::_1));
    new_sub_ = this->create_subscription<std_msgs::msg::Empty>("planning/new", 10,
        std::bind(&TrajServerNode::newCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom_world", 50,
        std::bind(&TrajServerNode::odomCallback, this, std::placeholders::_1));
    pg_T_vio_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/loop_fusion/pg_T_vio", 10,
        std::bind(&TrajServerNode::pgTVioCallback, this, std::placeholders::_1));

    cmd_vis_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("planning/position_cmd_vis", 10);
    pos_cmd_pub_ =
        this->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);
    // Optional geometry_msgs/Twist output for ground robots (quadrupeds etc.)
    // that consume body-frame velocity commands directly. Controlled by the
    // traj_server.publish_twist launch param; default off (drone workflow).
    publish_twist_ = self->declare_parameter<bool>("traj_server.publish_twist", false);
    v_fwd_max_twist_ = self->declare_parameter<double>("traj_server.v_fwd_max", 1.0);
    v_lat_max_twist_ = self->declare_parameter<double>("traj_server.v_lat_max", 0.5);
    omega_max_twist_ = self->declare_parameter<double>("traj_server.omega_max", 1.0);
    if (publish_twist_) {
      twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
    }
    traj_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);

    cmd_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&TrajServerNode::cmdCallback, this));
    vis_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(250), std::bind(&TrajServerNode::visCallback, this));

    percep_utils_ = std::make_shared<PerceptionUtils>(self);

    RCLCPP_WARN(this->get_logger(), "[Traj server]: init...");
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_WARN(this->get_logger(), "[Traj server]: ready.");
  }

private:
  // Publishers/subscribers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;  // optional (quadrupeds)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  bool publish_twist_{false};
  double v_fwd_max_twist_{1.0}, v_lat_max_twist_{0.5}, omega_max_twist_{1.0};

  rclcpp::Subscription<bspline::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr new_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pg_T_vio_sub_;

  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  // State
  nav_msgs::msg::Odometry odom_;
  quadrotor_msgs::msg::PositionCommand cmd_;
  std::vector<NonUniformBspline> traj_;
  double traj_duration_;
  rclcpp::Time start_time_;
  int traj_id_;
  int pub_traj_id_;
  int pub_traj_id_state_;

  std::shared_ptr<PerceptionUtils> percep_utils_;

  bool receive_traj_ = false;
  double replan_time_;

  std::vector<Eigen::Vector3d> traj_cmd_, traj_real_;

  rclcpp::Time start_time, end_time, last_time;
  double energy;

  Eigen::Matrix3d R_loop;
  Eigen::Vector3d T_loop;
  bool isLoopCorrection = false;

  int drone_id_;
  int drone_num_;

  double calcPathLength(const std::vector<Eigen::Vector3d>& path) {
    if (path.empty()) return 0;
    double len = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      len += (path[i + 1] - path[i]).norm();
    }
    return len;
  }

  void displayTrajWithColor(
      std::vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->now();
    mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::msg::Marker::DELETE;
    mk.id = id;
    traj_pub_->publish(mk);

    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
      pt.x = path[i](0);
      pt.y = path[i](1);
      pt.z = path[i](2);
      mk.points.push_back(pt);
    }
    traj_pub_->publish(mk);
    rclcpp::sleep_for(std::chrono::microseconds(1000));
  }

  void drawFOV(
      const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->now();
    mk.id = 0;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    mk.pose.orientation.w = 1.0;

    Eigen::Vector4d color(0, 0, 0, 1);
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);
    mk.scale.x = 0.04;
    mk.scale.y = 0.04;
    mk.scale.z = 0.04;

    mk.action = visualization_msgs::msg::Marker::DELETE;
    cmd_vis_pub_->publish(mk);

    if (list1.size() == 0) return;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
    mk.action = visualization_msgs::msg::Marker::ADD;
    cmd_vis_pub_->publish(mk);
  }

  void replanCallback(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/) {
    const double time_out = 0.2;
    rclcpp::Time time_now = this->now();
    double t_stop = (time_now - start_time_).seconds() + time_out + replan_time_;
    traj_duration_ = std::min(t_stop, traj_duration_);
  }

  void newCallback(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/) {
    traj_cmd_.clear();
    traj_real_.clear();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;
    odom_ = *msg;
    traj_real_.push_back(Eigen::Vector3d(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z));

    if (traj_real_.size() > 10000)
      traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
  }

  void pgTVioCallback(const geometry_msgs::msg::Pose::ConstSharedPtr msg) {
    Eigen::Quaterniond q = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    R_loop = q.toRotationMatrix();
    T_loop << msg->position.x, msg->position.y, msg->position.z;
  }

  void visCallback() {
    displayTrajWithColor(traj_cmd_, 0.05,
        PlanningVisualization::getColor((drone_id_ - 1) / double(drone_num_)), pub_traj_id_);
  }

  void bsplineCallback(const bspline::msg::Bspline::ConstSharedPtr msg) {
    if (msg->traj_id <= traj_id_) {
      RCLCPP_ERROR(this->get_logger(), "out of order bspline.");
      return;
    }

    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) {
      knots(i) = msg->knots[i];
    }
    for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
      pos_pts(i, 0) = msg->pos_pts[i].x;
      pos_pts(i, 1) = msg->pos_pts[i].y;
      pos_pts(i, 2) = msg->pos_pts[i].z;
    }
    NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    for (size_t i = 0; i < msg->yaw_pts.size(); ++i) yaw_pts(i, 0) = msg->yaw_pts[i];
    NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);

    // start_time is builtin_interfaces/Time in ROS 2
    start_time_ = rclcpp::Time(msg->start_time, this->get_clock()->get_clock_type());
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_.push_back(yaw_traj);
    traj_.push_back(yaw_traj.getDerivative());
    traj_.push_back(traj_[2].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();

    receive_traj_ = true;

    if (start_time.nanoseconds() == 0) {
      RCLCPP_WARN(this->get_logger(), "start flight");
      start_time = this->now();
    }
  }

  void cmdCallback() {
    if (!receive_traj_) return;

    rclcpp::Time time_now = this->now();
    double t_cur = (time_now - start_time_).seconds();
    Eigen::Vector3d pos, vel, acc, jer;
    double yaw = 0.0, yawdot = 0.0;

    if (t_cur < traj_duration_ && t_cur >= 0.0) {
      pos = traj_[0].evaluateDeBoorT(t_cur);
      vel = traj_[1].evaluateDeBoorT(t_cur);
      acc = traj_[2].evaluateDeBoorT(t_cur);
      yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
      yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
      jer = traj_[5].evaluateDeBoorT(t_cur);
    } else if (t_cur >= traj_duration_) {
      pos = traj_[0].evaluateDeBoorT(traj_duration_);
      vel.setZero();
      acc.setZero();
      yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
      yawdot = 0.0;
    } else {
      std::cout << "[Traj server]: invalid time." << std::endl;
      return;
    }

    double len = calcPathLength(traj_cmd_);
    double flight_t = (end_time - start_time).seconds();
    auto clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000,
        "Drone %d, [%lf,%lf,%lf,%lf], [time, length, vel, energy]", drone_id_, flight_t, len,
        flight_t > 0 ? len / flight_t : 0.0, energy);

    if (isLoopCorrection) {
      pos = R_loop.transpose() * (pos - T_loop);
      vel = R_loop.transpose() * vel;
      acc = R_loop.transpose() * acc;

      Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
      yaw_dir = R_loop.transpose() * yaw_dir;
      yaw = atan2(yaw_dir[1], yaw_dir[0]);
    }

    cmd_.header.stamp = time_now;
    cmd_.trajectory_id = traj_id_;
    cmd_.position.x = pos(0);
    cmd_.position.y = pos(1);
    cmd_.position.z = pos(2);
    cmd_.velocity.x = vel(0);
    cmd_.velocity.y = vel(1);
    cmd_.velocity.z = vel(2);
    cmd_.acceleration.x = acc(0);
    cmd_.acceleration.y = acc(1);
    cmd_.acceleration.z = acc(2);
    cmd_.yaw = yaw;
    cmd_.yaw_dot = yawdot;
    pos_cmd_pub_->publish(cmd_);

    // Optional Twist output (body-frame, clipped).
    if (publish_twist_ && twist_pub_) {
      const double cy = std::cos(yaw), sy = std::sin(yaw);
      const double v_body_x =  cy * vel(0) + sy * vel(1);
      const double v_body_y = -sy * vel(0) + cy * vel(1);
      geometry_msgs::msg::Twist t;
      t.linear.x = std::clamp(v_body_x, -v_fwd_max_twist_, v_fwd_max_twist_);
      t.linear.y = std::clamp(v_body_y, -v_lat_max_twist_, v_lat_max_twist_);
      t.linear.z = 0.0;
      t.angular.x = 0.0;
      t.angular.y = 0.0;
      t.angular.z = std::clamp(yawdot, -omega_max_twist_, omega_max_twist_);
      twist_pub_->publish(t);
    }

    percep_utils_->setPose(pos, yaw);
    std::vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawFOV(l1, l2);

    if (traj_cmd_.size() == 0) {
      traj_cmd_.push_back(pos);
    } else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
      traj_cmd_.push_back(pos);
      double dt = (time_now - last_time).seconds();
      energy += jer.squaredNorm() * dt;
      end_time = this->now();
      if (energy > 10000) {
        std::cout << "jer: " << jer.transpose() << ", dt: " << dt << std::endl;
      }
    }
    last_time = time_now;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajServerNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <memory>
#include <string>

#include <armadillo>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pose_utils/pose_utils.h>

using namespace arma;

struct DisturbanceConfig {
  bool enable_drift_odom{false};
  bool enable_noisy_odom{false};
  double vdriftx{0}, vdrifty{0}, vdriftz{0}, vdriftyaw{0};
  double stdvdriftxyz{0}, stdvdriftyaw{0};
  double stdxyz{0}, stdyaw{0}, stdrp{0}, stdvxyz{0};
  double fxy{0}, fz{0}, stdfxy{0}, stdfz{0};
  double mrp{0}, myaw{0}, stdmrp{0}, stdmyaw{0};
};

static constexpr double kCorrectionRate = 1.0;

class SO3DisturbanceGenerator : public rclcpp::Node {
public:
  SO3DisturbanceGenerator() : rclcpp::Node("so3_disturbance_generator") {
    declareAll();

    auto odom_qos = rclcpp::QoS(10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", odom_qos,
        std::bind(&SO3DisturbanceGenerator::odomCb, this, std::placeholders::_1));
    pubo_ = this->create_publisher<nav_msgs::msg::Odometry>("noisy_odom", 10);
    pubc_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("correction", 10);
    pubf_ = this->create_publisher<geometry_msgs::msg::Vector3>("force_disturbance", 10);
    pubm_ = this->create_publisher<geometry_msgs::msg::Vector3>("moment_disturbance", 10);

    param_cb_ = this->add_on_set_parameters_callback(
        std::bind(&SO3DisturbanceGenerator::onParamSet, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
        std::bind(&SO3DisturbanceGenerator::setDisturbance, this));
  }

private:
  template <typename T>
  void declareAndRead(const std::string& name, T& field, T default_value) {
    field = this->declare_parameter<T>(name, default_value);
  }

  void declareAll() {
    declareAndRead("enable_drift_odom", cfg_.enable_drift_odom, false);
    declareAndRead("enable_noisy_odom", cfg_.enable_noisy_odom, false);
    declareAndRead("vdriftx", cfg_.vdriftx, 0.0);
    declareAndRead("vdrifty", cfg_.vdrifty, 0.0);
    declareAndRead("vdriftz", cfg_.vdriftz, 0.0);
    declareAndRead("vdriftyaw", cfg_.vdriftyaw, 0.0);
    declareAndRead("stdvdriftxyz", cfg_.stdvdriftxyz, 0.0);
    declareAndRead("stdvdriftyaw", cfg_.stdvdriftyaw, 0.0);
    declareAndRead("stdxyz", cfg_.stdxyz, 0.0);
    declareAndRead("stdyaw", cfg_.stdyaw, 0.0);
    declareAndRead("stdrp", cfg_.stdrp, 0.0);
    declareAndRead("stdvxyz", cfg_.stdvxyz, 0.0);
    declareAndRead("fxy", cfg_.fxy, 0.0);
    declareAndRead("fz", cfg_.fz, 0.0);
    declareAndRead("stdfxy", cfg_.stdfxy, 0.0);
    declareAndRead("stdfz", cfg_.stdfz, 0.0);
    declareAndRead("mrp", cfg_.mrp, 0.0);
    declareAndRead("myaw", cfg_.myaw, 0.0);
    declareAndRead("stdmrp", cfg_.stdmrp, 0.0);
    declareAndRead("stdmyaw", cfg_.stdmyaw, 0.0);
  }

  rcl_interfaces::msg::SetParametersResult onParamSet(
      const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      const auto& n = p.get_name();
      if (n == "enable_drift_odom") cfg_.enable_drift_odom = p.as_bool();
      else if (n == "enable_noisy_odom") cfg_.enable_noisy_odom = p.as_bool();
      else if (n == "vdriftx") cfg_.vdriftx = p.as_double();
      else if (n == "vdrifty") cfg_.vdrifty = p.as_double();
      else if (n == "vdriftz") cfg_.vdriftz = p.as_double();
      else if (n == "vdriftyaw") cfg_.vdriftyaw = p.as_double();
      else if (n == "stdvdriftxyz") cfg_.stdvdriftxyz = p.as_double();
      else if (n == "stdvdriftyaw") cfg_.stdvdriftyaw = p.as_double();
      else if (n == "stdxyz") cfg_.stdxyz = p.as_double();
      else if (n == "stdyaw") cfg_.stdyaw = p.as_double();
      else if (n == "stdrp") cfg_.stdrp = p.as_double();
      else if (n == "stdvxyz") cfg_.stdvxyz = p.as_double();
      else if (n == "fxy") cfg_.fxy = p.as_double();
      else if (n == "fz") cfg_.fz = p.as_double();
      else if (n == "stdfxy") cfg_.stdfxy = p.as_double();
      else if (n == "stdfz") cfg_.stdfz = p.as_double();
      else if (n == "mrp") cfg_.mrp = p.as_double();
      else if (n == "myaw") cfg_.myaw = p.as_double();
      else if (n == "stdmrp") cfg_.stdmrp = p.as_double();
      else if (n == "stdmyaw") cfg_.stdmyaw = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    noisy_odom_.header = msg->header;
    correction_.header = msg->header;

    colvec pose(6);
    colvec vel(3);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    colvec q = zeros<colvec>(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    if (!have_drift_state_) {
      drift_pose_ = pose;
      drift_vel_ = vel;
      correction_pose_ = zeros<colvec>(6);
      prev_pose_ = pose;
      prev_pose_t_ = rclcpp::Time(msg->header.stamp);
      prev_correction_t_ = prev_pose_t_;
      noisy_pose_ = drift_pose_;
      noisy_vel_ = drift_vel_;
      have_drift_state_ = true;
    }

    if (cfg_.enable_drift_odom) {
      auto stamp = rclcpp::Time(msg->header.stamp);
      double dt = (stamp - prev_pose_t_).seconds();
      prev_pose_t_ = stamp;
      colvec d = pose_update(pose_inverse(prev_pose_), pose);
      prev_pose_ = pose;
      d(0) += (cfg_.vdriftx + cfg_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(1) += (cfg_.vdrifty + cfg_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(2) += (cfg_.vdriftz + cfg_.stdvdriftxyz * as_scalar(randn(1))) * dt;
      d(3) += (cfg_.vdriftyaw + cfg_.stdvdriftyaw * as_scalar(randn(1))) * dt;
      drift_pose_ = pose_update(drift_pose_, d);
      drift_vel_ = ypr_to_R(drift_pose_.rows(3, 5)) * trans(ypr_to_R(pose.rows(3, 5))) * vel;
      correction_pose_ = pose_update(pose, pose_inverse(drift_pose_));
    } else {
      drift_pose_ = pose;
      drift_vel_ = vel;
      correction_pose_ = zeros<colvec>(6);
    }

    if (cfg_.enable_noisy_odom) {
      colvec noise_pose = zeros<colvec>(6);
      colvec noise_vel = zeros<colvec>(3);
      noise_pose(0) = cfg_.stdxyz * as_scalar(randn(1));
      noise_pose(1) = cfg_.stdxyz * as_scalar(randn(1));
      noise_pose(2) = cfg_.stdxyz * as_scalar(randn(1));
      noise_pose(3) = cfg_.stdyaw * as_scalar(randn(1));
      noise_pose(4) = cfg_.stdrp * as_scalar(randn(1));
      noise_pose(5) = cfg_.stdrp * as_scalar(randn(1));
      noise_vel(0) = cfg_.stdvxyz * as_scalar(randn(1));
      noise_vel(1) = cfg_.stdvxyz * as_scalar(randn(1));
      noise_vel(2) = cfg_.stdvxyz * as_scalar(randn(1));
      noisy_pose_ = drift_pose_ + noise_pose;
      noisy_vel_ = drift_vel_ + noise_vel;
      double sxyz = cfg_.stdxyz * cfg_.stdxyz;
      double syaw = cfg_.stdyaw * cfg_.stdyaw;
      double srp = cfg_.stdrp * cfg_.stdrp;
      double svxyz = cfg_.stdvxyz * cfg_.stdvxyz;
      noisy_odom_.pose.covariance[0 + 0 * 6] = sxyz;
      noisy_odom_.pose.covariance[1 + 1 * 6] = sxyz;
      noisy_odom_.pose.covariance[2 + 2 * 6] = sxyz;
      noisy_odom_.pose.covariance[(0 + 3) + (0 + 3) * 6] = syaw;
      noisy_odom_.pose.covariance[(1 + 3) + (1 + 3) * 6] = srp;
      noisy_odom_.pose.covariance[(2 + 3) + (2 + 3) * 6] = srp;
      noisy_odom_.twist.covariance[0 + 0 * 6] = svxyz;
      noisy_odom_.twist.covariance[1 + 1 * 6] = svxyz;
      noisy_odom_.twist.covariance[2 + 2 * 6] = svxyz;
    } else {
      noisy_pose_ = drift_pose_;
      noisy_vel_ = drift_vel_;
      for (int i = 0; i < 6; ++i) noisy_odom_.pose.covariance[i + i * 6] = 0;
      for (int i = 0; i < 3; ++i) noisy_odom_.twist.covariance[i + i * 6] = 0;
    }

    noisy_odom_.pose.pose.position.x = noisy_pose_(0);
    noisy_odom_.pose.pose.position.y = noisy_pose_(1);
    noisy_odom_.pose.pose.position.z = noisy_pose_(2);
    noisy_odom_.twist.twist.linear.x = noisy_vel_(0);
    noisy_odom_.twist.twist.linear.y = noisy_vel_(1);
    noisy_odom_.twist.twist.linear.z = noisy_vel_(2);
    colvec noisy_q = R_to_quaternion(ypr_to_R(noisy_pose_.rows(3, 5)));
    noisy_odom_.pose.pose.orientation.w = noisy_q(0);
    noisy_odom_.pose.pose.orientation.x = noisy_q(1);
    noisy_odom_.pose.pose.orientation.y = noisy_q(2);
    noisy_odom_.pose.pose.orientation.z = noisy_q(3);
    pubo_->publish(noisy_odom_);

    auto stamp = rclcpp::Time(msg->header.stamp);
    if ((stamp - prev_correction_t_).seconds() > 1.0 / kCorrectionRate) {
      prev_correction_t_ = stamp;
      correction_.pose.position.x = correction_pose_(0);
      correction_.pose.position.y = correction_pose_(1);
      correction_.pose.position.z = correction_pose_(2);
      colvec correction_q = R_to_quaternion(ypr_to_R(correction_pose_.rows(3, 5)));
      correction_.pose.orientation.w = correction_q(0);
      correction_.pose.orientation.x = correction_q(1);
      correction_.pose.orientation.y = correction_q(2);
      correction_.pose.orientation.z = correction_q(3);
      pubc_->publish(correction_);
    }
  }

  void setDisturbance() {
    geometry_msgs::msg::Vector3 f;
    geometry_msgs::msg::Vector3 m;
    f.x = cfg_.fxy + cfg_.stdfxy * as_scalar(randn(1));
    f.y = cfg_.fxy + cfg_.stdfxy * as_scalar(randn(1));
    f.z = cfg_.fz + cfg_.stdfz * as_scalar(randn(1));
    m.x = cfg_.mrp + cfg_.stdmrp * as_scalar(randn(1));
    m.y = cfg_.mrp + cfg_.stdmrp * as_scalar(randn(1));
    m.z = cfg_.myaw + cfg_.stdmyaw * as_scalar(randn(1));
    pubf_->publish(f);
    pubm_->publish(m);
  }

  DisturbanceConfig cfg_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubo_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubc_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubf_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubm_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  nav_msgs::msg::Odometry noisy_odom_;
  geometry_msgs::msg::PoseStamped correction_;
  colvec drift_pose_, drift_vel_, correction_pose_, prev_pose_, noisy_pose_, noisy_vel_;
  rclcpp::Time prev_pose_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_correction_t_{0, 0, RCL_ROS_TIME};
  bool have_drift_state_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SO3DisturbanceGenerator>());
  rclcpp::shutdown();
  return 0;
}

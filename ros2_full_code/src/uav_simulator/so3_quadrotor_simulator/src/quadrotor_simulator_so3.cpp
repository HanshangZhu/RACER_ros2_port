#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_simulator/Quadrotor.h>
#include <uav_utils/geometry_utils.h>

struct Control {
  double rpm[4];
};

struct Command {
  float force[3]{0, 0, 0};
  float qx{0}, qy{0}, qz{0}, qw{1};
  float k_r[3]{0, 0, 0};
  float k_om[3]{0, 0, 0};
  float corrections[3]{0, 0, 0};
  float current_yaw{0};
  bool use_external_yaw{false};
};

struct Disturbance {
  Eigen::Vector3d f{Eigen::Vector3d::Zero()};
  Eigen::Vector3d m{Eigen::Vector3d::Zero()};
};

static Control computeControl(const QuadrotorSimulator::Quadrotor& quad, const Command& cmd) {
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf = _kf - cmd.corrections[0];
  const double km = _km / _kf * kf;
  const double d = quad.getArmLength();
  const Eigen::Matrix3f J = quad.getInertia().cast<float>();
  const float I[3][3] = {{J(0, 0), J(0, 1), J(0, 2)}, {J(1, 0), J(1, 1), J(1, 2)},
      {J(2, 0), J(2, 1), J(2, 2)}};
  const auto state = quad.getState();

  Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);
  Eigen::Vector3d ypr = _ypr;
  if (cmd.use_external_yaw) ypr[0] = cmd.current_yaw;
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());
  float R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  float R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  float R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);
  float Om1 = state.omega(0), Om2 = state.omega(1), Om3 = state.omega(2);

  float Rd11 = cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  float Rd22 = cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  float Rd33 = cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;

  float Psi = 0.5f *
      (3.0f -
          (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 + Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
              Rd13 * R13 + Rd23 * R23 + Rd33 * R33));
  float force = 0;
  if (Psi < 1.0f)
    force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 + R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 - R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 + R31 * Rd32 - R32 * Rd31);

  float eOm1 = Om1, eOm2 = Om2, eOm3 = Om3;
  float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
      Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
      Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
      Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);

  float M1 = -cmd.k_r[0] * eR1 - cmd.k_om[0] * eOm1 + in1;
  float M2 = -cmd.k_r[1] * eR2 - cmd.k_om[1] * eOm2 + in2;
  float M3 = -cmd.k_r[2] * eR3 - cmd.k_om[2] * eOm3 + in3;

  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  Control control;
  for (int i = 0; i < 4; i++) {
    if (w_sq[i] < 0) w_sq[i] = 0;
    control.rpm[i] = std::sqrt(w_sq[i]);
  }
  return control;
}

static void stateToOdomMsg(
    const QuadrotorSimulator::Quadrotor::State& state, nav_msgs::msg::Odometry& odom) {
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);
  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);
  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

static void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::msg::Imu& imu) {
  auto state = quad.getState();
  Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();
  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);
  imu.linear_acceleration.x = quad.getAcc()[0];
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}

class QuadrotorSimulatorSO3 : public rclcpp::Node {
public:
  QuadrotorSimulatorSO3() : rclcpp::Node("quadrotor_simulator_so3") {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::SO3Command>("cmd", 100,
        std::bind(&QuadrotorSimulatorSO3::cmdCb, this, std::placeholders::_1));
    f_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("force_disturbance", 100,
        std::bind(&QuadrotorSimulatorSO3::forceCb, this, std::placeholders::_1));
    m_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("moment_disturbance", 100,
        std::bind(&QuadrotorSimulatorSO3::momentCb, this, std::placeholders::_1));

    double init_x = this->declare_parameter<double>("simulator.init_state_x", 0.0);
    double init_y = this->declare_parameter<double>("simulator.init_state_y", 0.0);
    double init_z = this->declare_parameter<double>("simulator.init_state_z", 1.0);
    quad_.setStatePos(Eigen::Vector3d(init_x, init_y, init_z));

    simulation_rate_ = this->declare_parameter<double>("rate.simulation", 1000.0);
    odom_rate_ = this->declare_parameter<double>("rate.odom", 100.0);
    quad_name_ = this->declare_parameter<std::string>("quadrotor_name", "quadrotor");

    odom_msg_.header.frame_id = "/simulator";
    odom_msg_.child_frame_id = "/" + quad_name_;
    imu_msg_.header.frame_id = "/simulator";

    next_odom_pub_time_ = this->now();
    dt_ = 1.0 / simulation_rate_;
    odom_period_ = rclcpp::Duration::from_seconds(1.0 / odom_rate_);

    auto period = std::chrono::duration<double>(dt_);
    sim_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&QuadrotorSimulatorSO3::step, this));
  }

private:
  void cmdCb(const quadrotor_msgs::msg::SO3Command::SharedPtr cmd) {
    command_.force[0] = cmd->force.x;
    command_.force[1] = cmd->force.y;
    command_.force[2] = cmd->force.z;
    command_.qx = cmd->orientation.x;
    command_.qy = cmd->orientation.y;
    command_.qz = cmd->orientation.z;
    command_.qw = cmd->orientation.w;
    for (int i = 0; i < 3; i++) {
      command_.k_r[i] = cmd->k_r[i];
      command_.k_om[i] = cmd->k_om[i];
    }
    command_.corrections[0] = cmd->aux.kf_correction;
    command_.corrections[1] = cmd->aux.angle_corrections[0];
    command_.corrections[2] = cmd->aux.angle_corrections[1];
    command_.current_yaw = cmd->aux.current_yaw;
    command_.use_external_yaw = cmd->aux.use_external_yaw;
  }

  void forceCb(const geometry_msgs::msg::Vector3::SharedPtr f) {
    disturbance_.f << f->x, f->y, f->z;
  }
  void momentCb(const geometry_msgs::msg::Vector3::SharedPtr m) {
    disturbance_.m << m->x, m->y, m->z;
  }

  void step() {
    auto last = control_;
    control_ = computeControl(quad_, command_);
    for (int i = 0; i < 4; ++i) {
      if (std::isnan(control_.rpm[i])) control_.rpm[i] = last.rpm[i];
    }
    quad_.setInput(control_.rpm[0], control_.rpm[1], control_.rpm[2], control_.rpm[3]);
    quad_.setExternalForce(disturbance_.f);
    quad_.setExternalMoment(disturbance_.m);
    quad_.step(dt_);

    auto tnow = this->now();
    if (tnow >= next_odom_pub_time_) {
      next_odom_pub_time_ = next_odom_pub_time_ + odom_period_;
      odom_msg_.header.stamp = tnow;
      stateToOdomMsg(quad_.getState(), odom_msg_);
      quadToImuMsg(quad_, imu_msg_);
      odom_pub_->publish(odom_msg_);
      imu_pub_->publish(imu_msg_);
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr f_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;

  QuadrotorSimulator::Quadrotor quad_;
  Command command_;
  Control control_{};
  Disturbance disturbance_;
  nav_msgs::msg::Odometry odom_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  rclcpp::Time next_odom_pub_time_;
  rclcpp::Duration odom_period_{0, 0};
  double simulation_rate_{1000.0};
  double odom_rate_{100.0};
  double dt_{0.001};
  std::string quad_name_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrotorSimulatorSO3>());
  rclcpp::shutdown();
  return 0;
}

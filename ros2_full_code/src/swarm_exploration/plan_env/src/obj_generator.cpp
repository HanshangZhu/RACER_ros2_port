#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <plan_env/linear_obj_model.hpp>

using namespace std;

class ObjGenerator : public rclcpp::Node {
public:
  ObjGenerator() : rclcpp::Node("dynamic_obj") {
    obj_num = this->declare_parameter<int>("obj_generator.obj_num", 10);
    _xy_size = this->declare_parameter<double>("obj_generator.xy_size", 15.0);
    _h_size = this->declare_parameter<double>("obj_generator.h_size", 5.0);
    _vel = this->declare_parameter<double>("obj_generator.vel", 5.0);
    _yaw_dot = this->declare_parameter<double>("obj_generator.yaw_dot", 5.0);
    _acc_r1 = this->declare_parameter<double>("obj_generator.acc_r1", 4.0);
    _acc_r2 = this->declare_parameter<double>("obj_generator.acc_r2", 6.0);
    _acc_z = this->declare_parameter<double>("obj_generator.acc_z", 3.0);
    _scale1 = this->declare_parameter<double>("obj_generator.scale1", 1.5);
    _scale2 = this->declare_parameter<double>("obj_generator.scale2", 2.5);
    _interval = this->declare_parameter<double>("obj_generator.interval", 2.5);

    obj_pub = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic/obj", 10);
    for (int i = 0; i < obj_num; ++i) {
      auto pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/dynamic/pose_" + to_string(i), 10);
      pose_pubs.push_back(pose_pub);
    }

    rand_color = uniform_real_distribution<double>(0.0, 1.0);
    rand_pos = uniform_real_distribution<double>(-_xy_size, _xy_size);
    rand_h = uniform_real_distribution<double>(0.0, _h_size);
    rand_vel = uniform_real_distribution<double>(-_vel, _vel);
    rand_acc_t = uniform_real_distribution<double>(0.0, 6.28);
    rand_acc_r = uniform_real_distribution<double>(_acc_r1, _acc_r2);
    rand_acc_z = uniform_real_distribution<double>(-_acc_z, _acc_z);
    rand_scale = uniform_real_distribution<double>(_scale1, _scale2);
    rand_yaw = uniform_real_distribution<double>(0.0, 2 * 3.141592);
    rand_yaw_dot = uniform_real_distribution<double>(-_yaw_dot, _yaw_dot);

    for (int i = 0; i < obj_num; ++i) {
      LinearObjModel model;
      Eigen::Vector3d pos(rand_pos(eng), rand_pos(eng), rand_h(eng));
      Eigen::Vector3d vel(rand_vel(eng), rand_vel(eng), 0.0);
      Eigen::Vector3d color(rand_color(eng), rand_color(eng), rand_color(eng));
      Eigen::Vector3d scale(rand_scale(eng), 1.5 * rand_scale(eng), rand_scale(eng));
      double yaw = rand_yaw(eng);
      double yaw_dot = rand_yaw_dot(eng);

      double r, t, z;
      r = rand_acc_r(eng);
      t = rand_acc_t(eng);
      z = rand_acc_z(eng);
      Eigen::Vector3d acc(r * cos(t), r * sin(t), z);

      model.initialize(pos, vel, acc, yaw, yaw_dot, color, scale);
      model.setLimits(Eigen::Vector3d(_xy_size, _xy_size, _h_size), Eigen::Vector2d(0.0, _vel),
          Eigen::Vector2d(0, 0));
      obj_models.push_back(model);
    }

    time_update = this->get_clock()->now();
    time_change = this->get_clock()->now();

    update_timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / 30.0)),
        std::bind(&ObjGenerator::updateCallback, this));

    RCLCPP_INFO(this->get_logger(), "[dynamic]: initialize with %d moving obj.", obj_num);
  }

private:
  int obj_num;
  double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obj_pub;
  vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs;
  vector<LinearObjModel> obj_models;

  random_device rd;
  default_random_engine eng{ rd() };
  uniform_real_distribution<double> rand_pos;
  uniform_real_distribution<double> rand_h;
  uniform_real_distribution<double> rand_vel;
  uniform_real_distribution<double> rand_acc_r;
  uniform_real_distribution<double> rand_acc_t;
  uniform_real_distribution<double> rand_acc_z;
  uniform_real_distribution<double> rand_color;
  uniform_real_distribution<double> rand_scale;
  uniform_real_distribution<double> rand_yaw_dot;
  uniform_real_distribution<double> rand_yaw;

  rclcpp::Time time_update, time_change;
  rclcpp::TimerBase::SharedPtr update_timer;

  void updateCallback() {
    rclcpp::Time time_now = this->get_clock()->now();

    double dtc = (time_now - time_change).seconds();
    if (dtc > _interval) {
      for (int i = 0; i < obj_num; ++i) {
        double vx, vy, vz, yd;
        vx = rand_vel(eng);
        vy = rand_vel(eng);
        vz = 0.0;
        yd = rand_yaw_dot(eng);

        obj_models[i].setInput(Eigen::Vector3d(vx, vy, vz));
        obj_models[i].setYawDot(yd);
      }
      time_change = time_now;
    }

    double dt = (time_now - time_update).seconds();
    time_update = time_now;
    for (int i = 0; i < obj_num; ++i) {
      obj_models[i].update(dt);
      visualizeObj(i);
    }

    for (int i = 0; i < obj_num; ++i)
      for (int j = i + 1; j < obj_num; ++j) {
        bool collision = LinearObjModel::collide(obj_models[i], obj_models[j]);
        if (collision) {
          double yd1 = rand_yaw_dot(eng);
          double yd2 = rand_yaw_dot(eng);
          obj_models[i].setYawDot(yd1);
          obj_models[j].setYawDot(yd2);
        }
      }
  }

  void visualizeObj(int id) {
    Eigen::Vector3d pos, color, scale;
    pos = obj_models[id].getPosition();
    color = obj_models[id].getColor();
    scale = obj_models[id].getScale();
    double yaw = obj_models[id].getYaw();

    Eigen::Matrix3d rot;
    rot << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

    Eigen::Quaterniond qua;
    qua = rot;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = this->get_clock()->now();
    mk.type = visualization_msgs::msg::Marker::CUBE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.id = id;

    mk.scale.x = scale(0);
    mk.scale.y = scale(1);
    mk.scale.z = scale(2);
    mk.color.a = 0.5;
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);

    mk.pose.orientation.w = qua.w();
    mk.pose.orientation.x = qua.x();
    mk.pose.orientation.y = qua.y();
    mk.pose.orientation.z = qua.z();

    mk.pose.position.x = pos(0);
    mk.pose.position.y = pos(1);
    mk.pose.position.z = pos(2);

    obj_pub->publish(mk);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    // NOTE(ros2-port): Header::seq removed in ROS 2; drop.
    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = pos(2);
    pose.pose.orientation.w = 1.0;
    pose_pubs[id]->publish(pose);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

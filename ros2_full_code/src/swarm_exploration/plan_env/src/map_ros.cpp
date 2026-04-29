#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>
#include <plan_env/multi_map_manager.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <chrono>
#include <fstream>
#include <functional>

namespace fast_planner {

using std::placeholders::_1;
using std::placeholders::_2;

MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {
  auto declare_or_get_d = [&](const std::string& name, double def) {
    if (!node_->has_parameter(name)) return node_->declare_parameter<double>(name, def);
    return node_->get_parameter(name).get_value<double>();
  };
  auto declare_or_get_i = [&](const std::string& name, int def) {
    if (!node_->has_parameter(name)) return node_->declare_parameter<int>(name, def);
    return node_->get_parameter(name).get_value<int>();
  };
  auto declare_or_get_b = [&](const std::string& name, bool def) {
    if (!node_->has_parameter(name)) return node_->declare_parameter<bool>(name, def);
    return node_->get_parameter(name).get_value<bool>();
  };
  auto declare_or_get_s = [&](const std::string& name, const std::string& def) {
    if (!node_->has_parameter(name)) return node_->declare_parameter<std::string>(name, def);
    return node_->get_parameter(name).get_value<std::string>();
  };

  fx_ = declare_or_get_d("map_ros.fx", -1.0);
  fy_ = declare_or_get_d("map_ros.fy", -1.0);
  cx_ = declare_or_get_d("map_ros.cx", -1.0);
  cy_ = declare_or_get_d("map_ros.cy", -1.0);
  depth_filter_maxdist_ = declare_or_get_d("map_ros.depth_filter_maxdist", -1.0);
  depth_filter_mindist_ = declare_or_get_d("map_ros.depth_filter_mindist", -1.0);
  depth_filter_margin_ = declare_or_get_i("map_ros.depth_filter_margin", -1);
  k_depth_scaling_factor_ = declare_or_get_d("map_ros.k_depth_scaling_factor", -1.0);
  skip_pixel_ = declare_or_get_i("map_ros.skip_pixel", -1);

  esdf_slice_height_ = declare_or_get_d("map_ros.esdf_slice_height", -0.1);
  visualization_truncate_height_ = declare_or_get_d("map_ros.visualization_truncate_height", -0.1);
  visualization_truncate_low_ = declare_or_get_d("map_ros.visualization_truncate_low", -0.1);
  show_occ_time_ = declare_or_get_b("map_ros.show_occ_time", false);
  show_esdf_time_ = declare_or_get_b("map_ros.show_esdf_time", false);
  show_all_map_ = declare_or_get_b("map_ros.show_all_map", false);
  frame_id_ = declare_or_get_s("map_ros.frame_id", std::string("world"));

  // Guard against skip_pixel_ <= 0 so resize doesn't blow up.
  int skip_safe = (skip_pixel_ > 0) ? skip_pixel_ : 1;
  proj_points_.resize(640 * 480 / (skip_safe * skip_safe));
  point_cloud_.points.resize(640 * 480 / (skip_safe * skip_safe));
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  std::random_device rd;
  eng_ = default_random_engine(rd());

  esdf_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&MapROS::updateESDFCallback, this));
  vis_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&MapROS::visCallback, this));

  map_all_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("/sdf_map/update_range", 10);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/depth_cloud", 10);

  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_.get(), "/map_ros/depth", rmw_qos_profile_sensor_data);
  cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      node_.get(), "/map_ros/cloud", rmw_qos_profile_sensor_data);
  pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
      node_.get(), "/map_ros/pose", rmw_qos_profile_sensor_data);

  sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
      MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  sync_image_pose_->registerCallback(
      std::bind(&MapROS::depthPoseCallback, this, _1, _2));
  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(
      std::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  map_start_time_ = node_->get_clock()->now();
}

void MapROS::visCallback() {
  publishMapLocal();
  if (show_all_map_) {
    // Limit the frequency of all map
    static rclcpp::Time last_time = node_->get_clock()->now();
    auto now = node_->get_clock()->now();
    double tpass = (now - last_time).seconds();
    if (tpass > 0.1) {
      publishMapAll();
      last_time = now;
    }
  }
  publishUpdateRange();
}

void MapROS::updateESDFCallback() {
  if (!esdf_need_update_) return;
  auto t1 = node_->get_clock()->now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = node_->get_clock()->now();
  double dt = (t2 - t1).seconds();
  esdf_time_ += dt;
  max_esdf_time_ = std::max(max_esdf_time_, dt);
  esdf_num_++;
  if (show_esdf_time_)
    RCLCPP_WARN(node_->get_logger(),
        "ESDF t: cur: %lf, avg: %lf, max: %lf", dt, esdf_time_ / esdf_num_, max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  if (!map_->isInMap(camera_pos_)) return;

  map_->mm_->drone_pos_ = camera_pos_;

  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
      pose->pose.orientation.y, pose->pose.orientation.z);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = node_->get_clock()->now();

  processDepthImage();
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = node_->get_clock()->now();
  double dt = (t2 - t1).seconds();
  fuse_time_ += dt;
  max_fuse_time_ = std::max(max_fuse_time_, dt);
  fuse_num_ += 1;
  if (show_occ_time_)
    RCLCPP_WARN(node_->get_logger(),
        "Fusion t: cur: %lf, avg: %lf, max: %lf", dt, fuse_time_ / fuse_num_, max_fuse_time_);
}

void MapROS::cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
      pose->pose.orientation.y, pose->pose.orientation.z);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  map_->inputPointCloud(cloud, num, camera_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

void MapROS::processDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishDepth();
}

void MapROS::publishMapAll() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;

  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);

  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx[0]; x <= max_idx[0]; ++x)
    for (int y = min_idx[1]; y <= max_idx[1]; ++y)
      for (int z = min_idx[2]; z <= max_idx[2]; ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  map_all_pub_->publish(cloud_msg);
}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
            map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  map_local_pub_->publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  map_local_inflate_pub_->publish(cloud_msg);
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);

  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx(0); x <= max_idx(0); ++x)
    for (int y = min_idx(1); y <= max_idx(1); ++y)
      for (int z = min_idx(2); z <= max_idx(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] <
            map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  unknown_pub_->publish(cloud_msg);
}

void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  depth_pub_->publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = node_->get_clock()->now();
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_->publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ -
      Eigen::Vector3i(map_->mp_->local_map_margin_,
          map_->mp_->local_map_margin_, map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ +
      Eigen::Vector3i(map_->mp_->local_map_margin_,
          map_->mp_->local_map_margin_, map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = std::min(dist, max_dist);
      dist = std::max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;

  esdf_pub_->publish(cloud_msg);
}
}  // namespace fast_planner

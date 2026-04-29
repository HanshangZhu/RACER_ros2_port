// ROS 2 port of exploration_manager/fast_exploration_manager.cpp
//
// Wave 7 changes:
//   - nh-style param reads replaced with declare_parameter + get_parameter
//     using dotted names.
//   - ros::Time::now() + ros::Time diff -> node_->now() / rclcpp::Time - rclcpp::Time.
//   - ROS 1 service clients -> rclcpp::Client<lkh_mtsp_solver::srv::SolveMTSP>,
//     with async_send_request().wait_for() instead of synchronous call().
//   - ROS_* logging -> RCLCPP_* using node logger.
//
// Service clients live in a Reentrant callback group so that waiting on a
// response inside a planner entry point (which is driven from the FSM's
// MutuallyExclusive timer group) does not deadlock a
// MultiThreadedExecutor. The FSM node constructs itself via
// std::make_shared, so shared_from_this() is available when initialize()
// is invoked from FSM::init().

#include <exploration_manager/fast_exploration_manager.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/hgrid.h>
#include <active_perception/perception_utils.h>
#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

using namespace Eigen;
using namespace std::chrono_literals;

namespace fast_planner {

namespace {
template <typename T>
void declare_or_get(const rclcpp::Node::SharedPtr& node, const std::string& name, T& value,
    const T& default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  node->get_parameter(name, value);
}
}  // namespace

// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(const rclcpp::Node::SharedPtr& node) {
  node_ = node;

  planner_manager_ = std::make_shared<FastPlannerManager>();
  planner_manager_->initPlanModules(node_);

  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_ = std::make_shared<FrontierFinder>(edt_environment_, node_);
  hgrid_ = std::make_shared<HGrid>(edt_environment_, node_);

  ed_ = std::make_shared<ExplorationData>();
  ep_ = std::make_shared<ExplorationParam>();

  declare_or_get<bool>(node_, "exploration.refine_local", ep_->refine_local_, true);
  declare_or_get<int>(node_, "exploration.refined_num", ep_->refined_num_, -1);
  declare_or_get<double>(node_, "exploration.refined_radius", ep_->refined_radius_, -1.0);
  declare_or_get<int>(node_, "exploration.top_view_num", ep_->top_view_num_, -1);
  declare_or_get<double>(node_, "exploration.max_decay", ep_->max_decay_, -1.0);
  declare_or_get<std::string>(node_, "exploration.tsp_dir", ep_->tsp_dir_, std::string("null"));
  declare_or_get<std::string>(node_, "exploration.mtsp_dir", ep_->mtsp_dir_, std::string("null"));
  declare_or_get<double>(node_, "exploration.relax_time", ep_->relax_time_, 1.0);
  declare_or_get<int>(node_, "exploration.drone_num", ep_->drone_num_, 1);
  declare_or_get<int>(node_, "exploration.drone_id", ep_->drone_id_, 1);
  declare_or_get<int>(node_, "exploration.init_plan_num", ep_->init_plan_num_, 2);

  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  declare_or_get<double>(node_, "exploration.vm", ViewNode::vm_, -1.0);
  declare_or_get<double>(node_, "exploration.am", ViewNode::am_, -1.0);
  declare_or_get<double>(node_, "exploration.yd", ViewNode::yd_, -1.0);
  declare_or_get<double>(node_, "exploration.ydd", ViewNode::ydd_, -1.0);
  declare_or_get<double>(node_, "exploration.w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(node_, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Reentrant callback group means async_send_request().wait_for() inside a
  // planner entry point (driven from the FSM's MutuallyExclusive group) does
  // not deadlock the executor waiting for its own service response.
  service_cbgrp_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tsp_client_ = node_->create_client<lkh_mtsp_solver::srv::SolveMTSP>(
      "/solve_tsp_" + std::to_string(ep_->drone_id_), rmw_qos_profile_services_default,
      service_cbgrp_);
  acvrp_client_ = node_->create_client<lkh_mtsp_solver::srv::SolveMTSP>(
      "/solve_acvrp_" + std::to_string(ep_->drone_id_), rmw_qos_profile_services_default,
      service_cbgrp_);

  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;
  ed_->plan_num_ = 0;
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  auto t_total_start = node_->now();
  auto t1 = t_total_start;

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  ed_->frontier_tour_.clear();
  Vector3d next_pos;
  double next_yaw;
  vector<int> grid_ids, frontier_ids;
  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

  if (grid_ids.empty()) {
    return NO_GRID;

    // (unreachable legacy fallback preserved from upstream)
    RCLCPP_WARN(node_->get_logger(), "Empty grid");
    double min_cost = 100000;
    int min_cost_id = -1;
    vector<Vector3d> tmp_path;
    for (size_t i = 0; i < ed_->averages_.size(); ++i) {
      auto tmp_cost =
          ViewNode::computeCost(pos, ed_->points_[i], yaw[0], ed_->yaws_[i], vel, yaw[1], tmp_path);
      if (tmp_cost < min_cost) {
        min_cost = tmp_cost;
        min_cost_id = i;
      }
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];
  } else if (frontier_ids.size() == 0) {
    Eigen::Vector3d grid_center = ed_->grid_tour_[1];

    double min_cost = 100000;
    int min_cost_id = -1;
    for (size_t i = 0; i < ed_->points_.size(); ++i) {
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCost(
          grid_center, ed_->averages_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    next_pos = ed_->points_[min_cost_id];
    next_yaw = ed_->yaws_[min_cost_id];
  } else if (frontier_ids.size() == 1) {
    if (ep_->refine_local_) {
      ed_->refined_ids_ = { frontier_ids[0] };
      ed_->unrefined_points_ = { ed_->points_[frontier_ids[0]] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { frontier_ids[0] }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      if (grid_ids.size() <= 1) {
        double min_cost = 100000;
        int min_cost_id = -1;
        vector<Vector3d> tmp_path;
        for (size_t i = 0; i < ed_->n_points_[0].size(); ++i) {
          auto tmp_cost = ViewNode::computeCost(
              pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
          if (tmp_cost < min_cost) {
            min_cost = tmp_cost;
            min_cost_id = i;
          }
        }
        next_pos = ed_->n_points_[0][min_cost_id];
        next_yaw = n_yaws[0][min_cost_id];
      } else {
        Eigen::Vector3d grid_pos;
        double grid_yaw;
        if (hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw)) {
          ed_->n_points_.push_back({ grid_pos });
          n_yaws.push_back({ grid_yaw });
        }

        ed_->refined_points_.clear();
        ed_->refined_views_.clear();
        vector<double> refined_yaws;
        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
        next_pos = ed_->refined_points_[0];
        next_yaw = refined_yaws[0];
      }
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    }
  } else {
    t1 = node_->now();

    ed_->refined_ids_.clear();
    ed_->unrefined_points_.clear();
    int knum = std::min(int(frontier_ids.size()), ep_->refined_num_);
    for (int i = 0; i < knum; ++i) {
      auto tmp = ed_->points_[frontier_ids[i]];
      ed_->unrefined_points_.push_back(tmp);
      ed_->refined_ids_.push_back(frontier_ids[i]);
      if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
    }

    ed_->n_points_.clear();
    vector<vector<double>> n_yaws;
    frontier_finder_->getViewpointsInfo(
        pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

    ed_->refined_points_.clear();
    ed_->refined_views_.clear();
    vector<double> refined_yaws;
    refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
    next_pos = ed_->refined_points_[0];
    next_yaw = refined_yaws[0];

    for (size_t i = 0; i < ed_->refined_points_.size(); ++i) {
      Vector3d view =
          ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
      ed_->refined_views_.push_back(view);
    }
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    for (size_t i = 0; i < ed_->refined_points_.size(); ++i) {
      vector<Vector3d> v1, v2;
      frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
      frontier_finder_->percep_utils_->getFOV(v1, v2);
      ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
      ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
    }
    double local_time = (node_->now() - t1).seconds();
    RCLCPP_INFO(node_->get_logger(), "Local refine time: %lf", local_time);
  }

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;
  ed_->next_pos_ = next_pos;
  ed_->next_yaw_ = next_yaw;

  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  double total = (node_->now() - t_total_start).seconds();
  RCLCPP_INFO(node_->get_logger(), "Total time: %lf", total);
  if (total > 0.1) RCLCPP_ERROR(node_->get_logger(), "Total time too long!!!");

  return SUCCEED;
}

int FastExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  auto t1 = node_->now();

  double diff0 = next_yaw - yaw[0];
  double diff1 = std::fabs(diff0);
  double time_lb = std::min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  (void)goal_unknown;
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END) {
    RCLCPP_ERROR(node_->get_logger(), "No path to next viewpoint");
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      RCLCPP_WARN(node_->get_logger(), "init plan.");
    }
  } else if (len > radius_far) {
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (size_t i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  } else {
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    RCLCPP_ERROR(node_->get_logger(), "Lower bound not satified!");

  double traj_plan_time = (node_->now() - t1).seconds();

  t1 = node_->now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

  return SUCCEED;
}

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d& pos) {

  auto t1 = node_->now();
  auto t2 = t1;
  ed_->views_.clear();

  frontier_finder_->searchFrontiers();

  double frontier_time = (node_->now() - t1).seconds();
  t1 = node_->now();

  frontier_finder_->computeFrontiersToVisit();

  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (size_t i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  if (ed_->frontiers_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No coverable frontier.");
    return 0;
  }

  double view_time = (node_->now() - t1).seconds();

  t1 = node_->now();
  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf",
      ep_->drone_id_, frontier_time, view_time, mat_time);

  RCLCPP_INFO(node_->get_logger(), "Total t: %lf", (node_->now() - t2).seconds());
  return ed_->frontiers_.size();
}

void FastExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids) {
  auto t1 = node_->now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };
  (void)yaws;

  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    return;
  }
  grid_ids = ego_ids;

  double grid_time = (node_->now() - t1).seconds();

  t1 = node_->now();

  vector<int> ftr_ids;
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
  RCLCPP_INFO(
      node_->get_logger(), "Find frontier tour, %zu involved------------", ftr_ids.size());

  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }

  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = { grid_pos };
  }

  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  double ftr_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Empty path to shorten");
    return;
  }
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && rclcpp::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d cur_yaw, vector<int>& indices) {
  auto t1 = node_->now();

  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();
  std::cout << "mat:   " << cost_mat.rows() << std::endl;

  double mat_time = (node_->now() - t1).seconds();
  t1 = node_->now();

  std::ofstream par_file(ep_->tsp_dir_ + "/drone_" + std::to_string(ep_->drone_id_) + ".par");
  par_file << "PROBLEM_FILE = "
           << ep_->tsp_dir_ + "/drone_" + std::to_string(ep_->drone_id_) + ".tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE ="
           << ep_->tsp_dir_ + "/drone_" + std::to_string(ep_->drone_id_) +
          ".tou"
          "r\n";
  par_file << "RUNS = 1\n";
  par_file.close();

  std::ofstream prob_file(ep_->tsp_dir_ + "/drone_" + std::to_string(ep_->drone_id_) + ".tsp");
  std::string prob_spec;
  prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + std::to_string(dimension) +
              "\nEDGE_WEIGHT_TYPE : "
              "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  const int scale = 100;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  auto req = std::make_shared<lkh_mtsp_solver::srv::SolveMTSP::Request>();
  auto future = tsp_client_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve TSP (timeout).");
    return;
  }

  std::ifstream res_file(ep_->tsp_dir_ + "/drone_" + std::to_string(ep_->drone_id_) + ".tour");
  std::string res;
  while (std::getline(res_file, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (std::getline(res_file, res)) {
    int id = std::stoi(res);
    if (id == 1) continue;
    if (id == -1) break;
    indices.push_back(id - 2);
  }
  res_file.close();

  std::cout << "Tour " << ep_->drone_id_ << ": ";
  for (auto id : indices) std::cout << id << ", ";
  std::cout << "" << std::endl;

  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

  double tsp_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
    const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = node_->now();

  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  std::cout << "Local refine graph size: 1, ";
  for (size_t i = 0; i < n_points.size(); ++i) {
    for (size_t j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (node_->now() - t1).seconds();
  (void)create_time;
  t1 = node_->now();

  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (node_->now() - t1).seconds();
  (void)search_time;
  t1 = node_->now();

  for (size_t i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path_seg;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path_seg))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path_seg.begin(), path_seg.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (node_->now() - t1).seconds();
  (void)parse_time;
}

void FastExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
    vector<int>& other_ids) {

  auto t1 = node_->now();
  auto t2 = t1;
  (void)t2;

  if (grid_ids.size() == 1) {
    auto pt = hgrid_->getCenter(grid_ids.front());
    vector<Eigen::Vector3d> path;
    double d1 = ViewNode::computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    double d2 = ViewNode::computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    if (d1 < d2) {
      ego_ids = grid_ids;
      other_ids = {};
    } else {
      ego_ids = {};
      other_ids = grid_ids;
    }
    return;
  }

  Eigen::MatrixXd mat;
  hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

  double mat_time = (node_->now() - t1).seconds();
  (void)mat_time;

  t1 = node_->now();
  const int dimension = mat.rows();
  const int drone_num = positions.size();

  vector<int> unknown_nums;
  int capacity = 0;
  for (size_t i = 0; i < grid_ids.size(); ++i) {
    int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
    unknown_nums.push_back(unum);
    capacity += unum;
  }
  capacity = capacity * 0.75 * 0.1;

  const int prob_type = 2;

  std::ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + std::to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : pairopt\n";

  if (prob_type == 1)
    file << "TYPE : ATSP\n";
  else if (prob_type == 2)
    file << "TYPE : ACVRP\n";

  file << "DIMENSION : " + std::to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (prob_type == 2) {
    file << "CAPACITY : " + std::to_string(capacity) + "\n";
    file << "VEHICLES : " + std::to_string(drone_num) + "\n";
  }

  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }

  if (prob_type == 2) {
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << std::to_string(i + 2) + " 0\n";
    }
    for (size_t i = 0; i < grid_ids.size(); ++i) {
      int grid_unknown = unknown_nums[i] * 0.1;
      file << std::to_string(i + 2 + drone_num) + " " + std::to_string(grid_unknown) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  int min_size = int(grid_ids.size()) / 2;
  int max_size = std::ceil(int(grid_ids.size()) / 2.0);
  file.open(ep_->mtsp_dir_ + "/amtsp3_" + std::to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + std::to_string(ep_->drone_id_) +
          ".atsp\n";
  if (prob_type == 1) {
    file << "SALESMEN = " << std::to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "MTSP_MIN_SIZE = " << std::to_string(min_size) << "\n";
    file << "MTSP_MAX_SIZE = " << std::to_string(max_size) << "\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (prob_type == 2) {
    file << "TRACE_LEVEL = 1\n";
    file << "SEED = 0\n";
  }
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + std::to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  t1 = node_->now();

  auto req = std::make_shared<lkh_mtsp_solver::srv::SolveMTSP::Request>();
  req->prob = 3;
  auto future = acvrp_client_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve ACVRP (timeout).");
    return;
  }

  double mtsp_time = (node_->now() - t1).seconds();
  std::cout << "Allocation time: " << mtsp_time << std::endl;

  t1 = node_->now();

  std::ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + std::to_string(ep_->drone_id_) + ".tour");
  std::string res;
  vector<int> ids;
  while (std::getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (std::getline(fin, res)) {
    int id = std::stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  for (size_t i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : ego_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  for (auto& id : other_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
}

double FastExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos,
    const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f) {
  (void)w_f;
  if (grid_ids.empty()) return 0.0;

  double cost = 0.0;
  cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
  for (size_t i = 0; i + 1 < grid_ids.size(); ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}

bool FastExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {

  RCLCPP_INFO(node_->get_logger(), "Find grid tour---------------");

  auto t1 = node_->now();

  auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

  vector<int> first_ids, second_ids;
  hgrid_->inputFrontiers(ed_->averages_);

  hgrid_->updateGridData(
      ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);

  if (grid_ids.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Empty dominance.");
    ed_->grid_tour_.clear();
    return false;
  }

  std::cout << "Allocated grid: ";
  for (auto id : grid_ids) std::cout << id << ", ";
  std::cout << "" << std::endl;

  Eigen::MatrixXd mat;
  if (!init)
    hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
  else
    hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

  double mat_time = (node_->now() - t1).seconds();
  (void)mat_time;

  t1 = node_->now();
  const int dimension = mat.rows();
  const int drone_num = 1;

  std::ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + std::to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + std::to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  file.open(ep_->mtsp_dir_ + "/amtsp2_" + std::to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + std::to_string(ep_->drone_id_) +
          ".atsp\n";
  file << "SALESMEN = " << std::to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + std::to_string(ep_->drone_id_) +
          ".tour\n";
  file.close();

  t1 = node_->now();

  auto req = std::make_shared<lkh_mtsp_solver::srv::SolveMTSP::Request>();
  req->prob = 2;
  auto future = tsp_client_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve ATSP (timeout).");
    return false;
  }

  t1 = node_->now();

  std::ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + std::to_string(ep_->drone_id_) + ".tour");
  std::string res;
  vector<int> ids;
  while (std::getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (std::getline(fin, res)) {
    int id = std::stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  others.resize(drone_num - 1);
  for (size_t i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  for (auto& other : others) {
    for (auto& id : other) id -= 1 + drone_num;
  }
  std::cout << "Grid tour: ";
  for (auto& id : indices) {
    id = grid_ids[id];
    std::cout << id << ", ";
  }
  std::cout << "" << std::endl;

  grid_ids = indices;
  hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;

  return true;
}

void FastExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

  auto t1 = node_->now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  Eigen::MatrixXd mat;
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();

  double mat_time = (node_->now() - t1).seconds();
  (void)mat_time;

  t1 = node_->now();

  std::ofstream file(ep_->mtsp_dir_ + "/amtsp_" + std::to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + std::to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + std::to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + std::to_string(ep_->drone_id_) +
          ".atsp\n";
  file << "SALESMEN = " << std::to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = "
       << std::to_string(std::min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << std::to_string(
              std::max(1, int(ed_->frontiers_.size()) / std::max(1, drone_num - 1)))
       << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + std::to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  t1 = node_->now();

  auto req = std::make_shared<lkh_mtsp_solver::srv::SolveMTSP::Request>();
  req->prob = 1;
  auto future = tsp_client_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Fail to solve ATSP (timeout).");
    return;
  }

  t1 = node_->now();

  std::ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + std::to_string(ep_->drone_id_) + ".tour");
  std::string res;
  vector<int> ids;
  while (std::getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (std::getline(fin, res)) {
    int id = std::stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  for (size_t i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }

  if (ed_->grid_tour_.size() > 2) {
    indices.pop_back();
  }
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }
}

}  // namespace fast_planner

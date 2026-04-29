// ROS 2 port of lkh_tsp_solver/tsp_node.
//
// The upstream ROS 1 package optionally bundled the full LKH algorithm C
// sources and linked them into the node. Since the project already depends on
// an external `/usr/local/bin/LKH` binary (see CLAUDE.md -- installed in the
// Docker image), this port simply shells out to that binary. This keeps the
// build self-contained and avoids dragging the LKH C code (heavy global
// state, 150+ .c files) into the ROS 2 tree.
//
// NOTE(ros2-port): On hosts where `/usr/local/bin/LKH` is not installed this
// node will still start, but service calls will log an error. Install LKH or
// override `lkh_binary` parameter to the correct path.

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lkh_tsp_solver/srv/solve_tsp.hpp>

class TspNode : public rclcpp::Node {
 public:
  TspNode() : rclcpp::Node("tsp_node") {
    // Parameters (declared with defaults matching upstream behaviour).
    tsp_dir_ = this->declare_parameter<std::string>("exploration.tsp_dir", std::string("null"));
    drone_id_ = this->declare_parameter<int>("exploration.drone_id", 1);
    lkh_binary_ = this->declare_parameter<std::string>("lkh_binary", std::string("/usr/local/bin/LKH"));

    tsp_dir_ = tsp_dir_ + "/drone_" + std::to_string(drone_id_) + ".par";

    const std::string service_name = "/solve_tsp_" + std::to_string(drone_id_);
    service_ = this->create_service<lkh_tsp_solver::srv::SolveTSP>(
        service_name,
        std::bind(&TspNode::tspCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_WARN(this->get_logger(), "TSP server %d is ready (par=%s, lkh=%s).",
                drone_id_, tsp_dir_.c_str(), lkh_binary_.c_str());
  }

 private:
  void tspCallback(const std::shared_ptr<lkh_tsp_solver::srv::SolveTSP::Request> /*req*/,
                   std::shared_ptr<lkh_tsp_solver::srv::SolveTSP::Response> res) {
    const std::string cmd = lkh_binary_ + " " + tsp_dir_;
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "TSP server %d: LKH invocation failed (rc=%d, cmd='%s').",
                   drone_id_, rc, cmd.c_str());
    }
    res->empty = 0;
    RCLCPP_WARN(this->get_logger(), "TSP server %d finish", drone_id_);
  }

  std::string tsp_dir_;
  std::string lkh_binary_;
  int drone_id_ = 1;
  rclcpp::Service<lkh_tsp_solver::srv::SolveTSP>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TspNode>());
  rclcpp::shutdown();
  return 0;
}

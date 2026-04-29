// ROS 2 port of lkh_mtsp_solver/mtsp_node.
//
// Like the TSP variant, this wrapper shells out to the external
// `/usr/local/bin/LKH` binary rather than bundling the LKH C sources. The
// upstream ROS 1 mtsp_node already fell back to `system("/usr/local/bin/LKH ...")`
// for prob==3, so this is a consistent extension across all three problem
// types.
//
// NOTE(ros2-port): If LKH is not installed at the default path, set the
// `lkh_binary` parameter. Service calls will log an error on non-zero exit.

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lkh_mtsp_solver/srv/solve_mtsp.hpp>

class MtspNode : public rclcpp::Node {
 public:
  MtspNode() : rclcpp::Node("mtsp_node") {
    const std::string mtsp_dir =
        this->declare_parameter<std::string>("exploration.mtsp_dir", std::string("null"));
    drone_id_ = this->declare_parameter<int>("exploration.drone_id", 1);
    problem_id_ = this->declare_parameter<int>("exploration.problem_id", 1);
    lkh_binary_ = this->declare_parameter<std::string>("lkh_binary", std::string("/usr/local/bin/LKH"));

    mtsp_dir1_ = mtsp_dir + "/amtsp_" + std::to_string(drone_id_) + ".par";
    mtsp_dir2_ = mtsp_dir + "/amtsp2_" + std::to_string(drone_id_) + ".par";
    mtsp_dir3_ = mtsp_dir + "/amtsp3_" + std::to_string(drone_id_) + ".par";

    std::string service_name;
    if (problem_id_ == 1) {
      service_name = "/solve_tsp_" + std::to_string(drone_id_);
    } else if (problem_id_ == 2) {
      service_name = "/solve_acvrp_" + std::to_string(drone_id_);
    } else {
      // Upstream only defines 1 and 2 as service-producing problem ids; fall
      // back to a unique name so multiple instances don't collide.
      service_name = "/solve_mtsp_" + std::to_string(drone_id_);
    }

    service_ = this->create_service<lkh_mtsp_solver::srv::SolveMTSP>(
        service_name,
        std::bind(&MtspNode::mtspCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_WARN(this->get_logger(),
                "MTSP server %d is ready (service=%s, lkh=%s).",
                drone_id_, service_name.c_str(), lkh_binary_.c_str());
  }

 private:
  void mtspCallback(const std::shared_ptr<lkh_mtsp_solver::srv::SolveMTSP::Request> req,
                    std::shared_ptr<lkh_mtsp_solver::srv::SolveMTSP::Response> res) {
    std::string par;
    if (req->prob == 1) {
      par = mtsp_dir1_;
    } else if (req->prob == 2) {
      par = mtsp_dir2_;
    } else if (req->prob == 3) {
      par = mtsp_dir3_;
    } else {
      RCLCPP_ERROR(this->get_logger(), "MTSP server %d: unknown prob %d", drone_id_, req->prob);
      res->empty = 0;
      return;
    }

    const std::string cmd = lkh_binary_ + " " + par;
    const int rc = std::system(cmd.c_str());
    if (rc != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "MTSP server %d: LKH invocation failed (rc=%d, cmd='%s').",
                   drone_id_, rc, cmd.c_str());
    }
    res->empty = 0;
  }

  std::string mtsp_dir1_;
  std::string mtsp_dir2_;
  std::string mtsp_dir3_;
  std::string lkh_binary_;
  int drone_id_ = 1;
  int problem_id_ = 1;
  rclcpp::Service<lkh_mtsp_solver::srv::SolveMTSP>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MtspNode>());
  rclcpp::shutdown();
  return 0;
}

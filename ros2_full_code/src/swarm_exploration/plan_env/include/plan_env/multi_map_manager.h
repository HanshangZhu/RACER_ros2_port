#ifndef _MULTI_MAP_MANAGER_H
#define _MULTI_MAP_MANAGER_H

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <plan_env/msg/chunk_stamps.hpp>
#include <plan_env/msg/chunk_data.hpp>
#include <plan_env/msg/idx_list.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Eigen>

#include <memory>
#include <random>
#include <vector>
#include <unordered_map>

using std::shared_ptr;
using std::vector;
using std::unordered_map;

namespace fast_planner {
class SDFMap;
class MapROS;

// A map chunk, the elementary exchange unit between robots
struct MapChunk {
  uint32_t idx_;  // Start from 1
  vector<uint32_t> voxel_adrs_;
  vector<uint8_t> voxel_occ_;

  bool need_query_;
  bool empty_;
};

struct ChunksData {
  vector<MapChunk> chunks_;
  vector<int> idx_list_;
};

struct ChunksBox {
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  bool valid_;
};

class MultiMapManager {
public:
  MultiMapManager();
  ~MultiMapManager();
  void setMap(SDFMap* map);
  void setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
  }
  void init();

  void updateMapChunk(const vector<uint32_t>& adrs);
  void getChunkBoxes(
      vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset = true);

private:
  void sendChunks(const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list);
  void getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs);
  void insertChunkToMap(const MapChunk& chunk, const int& chunk_drone_id);
  void adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx);

  void stampTimerCallback();
  void chunkTimerCallback();
  void stampMsgCallback(const plan_env::msg::ChunkStamps::ConstSharedPtr msg);
  void chunkCallback(const plan_env::msg::ChunkData::ConstSharedPtr msg);

  // Operations on the chunk idx list
  void findMissedChunkIds(
      const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids);
  bool findIntersect(
      const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr);
  void mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output);

  // data----------------

  int drone_id_, map_num_;
  int vis_drone_id_;
  int chunk_size_;

  SDFMap* map_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<plan_env::msg::ChunkStamps>::SharedPtr stamp_pub_;
  rclcpp::Publisher<plan_env::msg::ChunkData>::SharedPtr chunk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<plan_env::msg::ChunkStamps>::SharedPtr stamp_sub_;
  rclcpp::Subscription<plan_env::msg::ChunkData>::SharedPtr chunk_sub_;
  rclcpp::TimerBase::SharedPtr stamp_timer_, chunk_timer_;

  vector<ChunksData> multi_map_chunks_;                    // Main map data
  vector<uint32_t> adr_buffer_;                            // Buffer for chunks of this map
  vector<vector<plan_env::msg::ChunkData>> chunk_buffer_;  // Buffer for chunks of external map
  vector<unordered_map<int, char>> buffer_map_;
  vector<double> last_chunk_stamp_time_;

  // Bounding box of map chunks of swarm
  vector<ChunksBox> chunk_boxes_;

  vector<Eigen::Vector3i> tmp_ids_;
  Eigen::Vector3d drone_pos_;

  friend SDFMap;
  friend MapROS;
};
}  // namespace fast_planner

#endif

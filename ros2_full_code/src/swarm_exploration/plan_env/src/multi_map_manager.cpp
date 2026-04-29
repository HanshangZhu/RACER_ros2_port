#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <chrono>
#include <fstream>
#include <algorithm>
#include <limits>

namespace fast_planner {

MultiMapManager::MultiMapManager() {
}

MultiMapManager::~MultiMapManager() {
}

void MultiMapManager::setMap(SDFMap* map) {
  this->map_ = map;
}

void MultiMapManager::init() {
  auto declare_or_get_i = [&](const std::string& name, int def) {
    if (!node_->has_parameter(name)) return node_->declare_parameter<int>(name, def);
    return node_->get_parameter(name).get_value<int>();
  };

  drone_id_ = declare_or_get_i("exploration.drone_id", 1);
  vis_drone_id_ = declare_or_get_i("exploration.vis_drone_id", -1);
  map_num_ = declare_or_get_i("exploration.drone_num", 2);
  chunk_size_ = declare_or_get_i("multi_map_manager.chunk_size", 200);

  stamp_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MultiMapManager::stampTimerCallback, this));
  chunk_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MultiMapManager::chunkTimerCallback, this));

  stamp_pub_ = node_->create_publisher<plan_env::msg::ChunkStamps>(
      "/multi_map_manager/chunk_stamps_send", 10);
  chunk_pub_ = node_->create_publisher<plan_env::msg::ChunkData>(
      "/multi_map_manager/chunk_data_send", 5000);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/multi_map_manager/marker_" + std::to_string(drone_id_), 10);

  stamp_sub_ = node_->create_subscription<plan_env::msg::ChunkStamps>(
      "/multi_map_manager/chunk_stamps_recv", 10,
      std::bind(&MultiMapManager::stampMsgCallback, this, std::placeholders::_1));
  chunk_sub_ = node_->create_subscription<plan_env::msg::ChunkData>(
      "/multi_map_manager/chunk_data_recv", 5000,
      std::bind(&MultiMapManager::chunkCallback, this, std::placeholders::_1));

  multi_map_chunks_.resize(map_num_);
  for (auto& data : multi_map_chunks_) {
    data.idx_list_ = {};
  }
  chunk_boxes_.resize(map_num_);
  for (auto& box : chunk_boxes_) {
    box.valid_ = false;
  }
  chunk_buffer_.resize(map_num_);
  buffer_map_.resize(map_num_);
  last_chunk_stamp_time_.resize(map_num_);
  for (auto time : last_chunk_stamp_time_) time = 0.0;
}

void MultiMapManager::updateMapChunk(const vector<uint32_t>& adrs) {
  adr_buffer_.insert(adr_buffer_.end(), adrs.begin(), adrs.end());

  if (adr_buffer_.size() >= static_cast<size_t>(chunk_size_)) {
    // Insert chunk from too long buffer
    size_t i = 0;
    for (; i + chunk_size_ < adr_buffer_.size(); i += chunk_size_) {
      MapChunk chunk;
      chunk.voxel_adrs_.insert(
          chunk.voxel_adrs_.end(), adr_buffer_.begin() + i, adr_buffer_.begin() + i + chunk_size_);
      chunk.idx_ = multi_map_chunks_[drone_id_ - 1].chunks_.size() + 1;
      chunk.need_query_ = true;
      chunk.empty_ = false;
      multi_map_chunks_[drone_id_ - 1].chunks_.push_back(chunk);
    }
    if (multi_map_chunks_[drone_id_ - 1].idx_list_.empty()) {
      multi_map_chunks_[drone_id_ - 1].idx_list_ = { 1, 1 };
    }
    multi_map_chunks_[drone_id_ - 1].idx_list_.back() =
        multi_map_chunks_[drone_id_ - 1].chunks_.back().idx_;

    // Remove already inserted data
    vector<uint32_t> tmp;
    tmp.insert(tmp.end(), adr_buffer_.begin() + i, adr_buffer_.end());
    adr_buffer_ = tmp;
  }
}

void MultiMapManager::stampTimerCallback() {
  // Send stamp of chunks to other drones
  plan_env::msg::ChunkStamps msg;
  msg.from_drone_id = drone_id_;
  msg.time = node_->get_clock()->now().seconds();

  for (auto chunks : multi_map_chunks_) {
    plan_env::msg::IdxList idx_list;
    idx_list.ids = chunks.idx_list_;
    msg.idx_lists.push_back(idx_list);
  }
  stamp_pub_->publish(msg);

  return;

  // Test visualization is bypassed by early return above.
}

void MultiMapManager::stampMsgCallback(const plan_env::msg::ChunkStamps::ConstSharedPtr msg) {
  if (msg->from_drone_id == drone_id_) return;
  if (drone_id_ == map_num_) return;  // Ground node does not send chunk

  // Check msg time to avoid overwhelming
  if (msg->time - last_chunk_stamp_time_[msg->from_drone_id - 1] < 0.3) return;
  last_chunk_stamp_time_[msg->from_drone_id - 1] = msg->time;

  // Check others' stamp info and send chunks unknown by them
  for (size_t i = 0; i < multi_map_chunks_.size(); ++i) {
    if (static_cast<int>(i) == msg->from_drone_id - 1) continue;
    vector<int> missed;
    findMissedChunkIds(multi_map_chunks_[i].idx_list_, msg->idx_lists[i].ids, missed);
    sendChunks(i + 1, msg->from_drone_id, missed);
  }
}

void MultiMapManager::findMissedChunkIds(
    const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids) {
  if (other_idx_list.empty()) {
    miss_ids = self_idx_list;
    return;
  }

  vector<int> not_in_other;
  if (other_idx_list[0] > 1) {
    not_in_other.push_back(1);
    not_in_other.push_back(other_idx_list[0] - 1);
  }
  for (size_t i = 1; i < other_idx_list.size(); i += 2) {
    not_in_other.push_back(other_idx_list[i] + 1);
    if (i == other_idx_list.size() - 1) {
      int infinite = std::numeric_limits<int>::max();
      not_in_other.push_back(infinite);
    } else {
      not_in_other.push_back(other_idx_list[i + 1] - 1);
    }
  }

  for (size_t i = 0; i < self_idx_list.size(); i += 2) {
    for (size_t j = 0; j < not_in_other.size(); j += 2) {
      int minr, maxr;
      if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j],
              not_in_other[j + 1], minr, maxr)) {
        miss_ids.push_back(minr);
        miss_ids.push_back(maxr);
      }
    }
  }
}

bool MultiMapManager::findIntersect(
    const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr) {
  minr = std::max(min1, min2);
  maxr = std::min(max1, max2);
  if (minr <= maxr) return true;
  return false;
}

void MultiMapManager::sendChunks(
    const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list) {
  auto& data = multi_map_chunks_[chunk_drone_id - 1];

  for (size_t i = 0; i < idx_list.size(); i += 2) {
    for (int j = idx_list[i]; j <= idx_list[i + 1]; ++j) {
      plan_env::msg::ChunkData msg;
      msg.from_drone_id = drone_id_;
      msg.to_drone_id = to_drone_id;
      msg.chunk_drone_id = chunk_drone_id;
      msg.idx = data.chunks_[j - 1].idx_;
      msg.voxel_adrs = data.chunks_[j - 1].voxel_adrs_;
      if (chunk_drone_id == drone_id_ && data.chunks_[j - 1].need_query_) {
        getOccOfChunk(data.chunks_[j - 1].voxel_adrs_, data.chunks_[j - 1].voxel_occ_);
        data.chunks_[j - 1].need_query_ = false;
      }
      msg.voxel_occ = data.chunks_[j - 1].voxel_occ_;

      chunk_pub_->publish(msg);
    }
  }
}

void MultiMapManager::getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs) {
  for (auto adr : adrs) {
    uint8_t occ = map_->md_->occupancy_buffer_[adr] > map_->mp_->min_occupancy_log_ ? 1 : 0;
    occs.push_back(occ);
  }
}

void MultiMapManager::chunkCallback(const plan_env::msg::ChunkData::ConstSharedPtr msg) {
  if (msg->from_drone_id == drone_id_) return;

  if (buffer_map_[msg->chunk_drone_id - 1].find(msg->idx) !=
      buffer_map_[msg->chunk_drone_id - 1].end())
    return;

  chunk_buffer_[msg->chunk_drone_id - 1].push_back(*msg);
  buffer_map_[msg->chunk_drone_id - 1][msg->idx] = 1;

  return;
}

void MultiMapManager::chunkTimerCallback() {
  // Not process chunk until swarm basecoor transform is available
  Eigen::Vector4d tmp;
  (void)tmp;

  // Process chunks in buffers
  for (size_t i = 0; i < chunk_buffer_.size(); ++i) {
    auto& buffer = chunk_buffer_[i];
    if (buffer.empty()) continue;

    std::sort(buffer.begin(), buffer.end(),
        [](const plan_env::msg::ChunkData& chunk1, const plan_env::msg::ChunkData& chunk2) {
          return chunk1.idx < chunk2.idx;
        });
    vector<int> idx_list = { int(buffer.front().idx) };
    int last_idx = idx_list[0];
    for (size_t j = 1; j < buffer.size(); ++j) {
      if (static_cast<int>(buffer[j].idx) - last_idx > 1) {
        idx_list.push_back(last_idx);
        idx_list.push_back(buffer[j].idx);
      }
      last_idx = buffer[j].idx;
    }
    idx_list.push_back(last_idx);

    auto& chunks_data = multi_map_chunks_[i];

    // Add placeholder for chunks
    int len_inc = last_idx - static_cast<int>(chunks_data.chunks_.size());
    for (int j = 0; j < len_inc; ++j) {
      chunks_data.chunks_.push_back(MapChunk());
      auto& back_chunk = chunks_data.chunks_.back();
      back_chunk.idx_ = chunks_data.chunks_.size();
      back_chunk.empty_ = true;
      back_chunk.need_query_ = false;
    }

    for (auto msg : buffer) {
      auto& chunk = chunks_data.chunks_[msg.idx - 1];
      if (chunk.empty_) {
        chunk.voxel_adrs_ = msg.voxel_adrs;
        chunk.voxel_occ_ = msg.voxel_occ;
        insertChunkToMap(chunk, msg.chunk_drone_id);
        chunk.empty_ = false;
      }
    }

    vector<int> union_list;
    mergeChunkIds(idx_list, chunks_data.idx_list_, union_list);
    chunks_data.idx_list_ = union_list;

    buffer.clear();
    buffer_map_[i].clear();
  }
}

void MultiMapManager::mergeChunkIds(
    const vector<int>& list1, const vector<int>& list2, vector<int>& output) {
  if (list1.empty()) {
    output = list2;
    return;
  }

  output = list1;
  int tmp1, tmp2;
  for (size_t i = 0; i < list2.size(); i += 2) {
    bool intersect = false;
    for (size_t j = 0; j < output.size(); j += 2) {
      if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2)) {
        output[j] = std::min(output[j], list2[i]);
        output[j + 1] = std::max(output[j + 1], list2[i + 1]);
        intersect = true;
      }
    }
    if (!intersect) {
      vector<int> tmp = { list2[i], list2[i + 1] };
      if (list2[i + 1] < output.front()) {
        output.insert(output.begin(), tmp.begin(), tmp.end());
      } else if (list2[i] > output.back()) {
        output.insert(output.end(), tmp.begin(), tmp.end());
      } else {
        for (auto iter = output.begin() + 1; iter != output.end(); iter += 2) {
          if (*iter < list2[i] && *(iter + 1) > list2[i + 1]) {
            output.insert(iter + 1, tmp.begin(), tmp.end());
            break;
          }
        }
      }
    }
    for (auto iter = output.begin() + 1; iter != output.end() - 1;) {
      if (*iter >= *(iter + 1) - 1) {
        iter = output.erase(iter);
        iter = output.erase(iter);
      } else {
        iter += 2;
      }
    }
  }
}

void MultiMapManager::adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx) {
  uint32_t tmp_adr = adr;
  const int a = map_->mp_->map_voxel_num_[1] * map_->mp_->map_voxel_num_[2];
  const int b = map_->mp_->map_voxel_num_[2];

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

void MultiMapManager::insertChunkToMap(const MapChunk& chunk, const int& drone_id) {
  for (size_t i = 0; i < chunk.voxel_adrs_.size(); ++i) {
    auto& adr = chunk.voxel_adrs_[i];

    Eigen::Vector3i idx;
    adrToIndex(adr, idx);

    Eigen::Vector3d pos;
    map_->indexToPos(idx, pos);

    if (!map_->isInMap(pos)) continue;

    map_->posToIndex(pos, idx);
    auto adr_tf = map_->toAddress(idx);

    map_->md_->occupancy_buffer_[adr_tf] =
        chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;

    if (chunk_boxes_[drone_id - 1].valid_) {
      for (int k = 0; k < 3; ++k) {
        chunk_boxes_[drone_id - 1].min_[k] =
            std::min(chunk_boxes_[drone_id - 1].min_[k], pos[k]);
        chunk_boxes_[drone_id - 1].max_[k] =
            std::max(chunk_boxes_[drone_id - 1].max_[k], pos[k]);
      }
    } else {
      chunk_boxes_[drone_id - 1].min_ = chunk_boxes_[drone_id - 1].max_ = pos;
      chunk_boxes_[drone_id - 1].valid_ = true;
    }

    for (int k = 0; k < 3; ++k) {
      map_->md_->all_min_[k] = std::min(map_->md_->all_min_[k], pos[k]);
      map_->md_->all_max_[k] = std::max(map_->md_->all_max_[k], pos[k]);
    }

    if (chunk.voxel_occ_[i] == 1) {
      static const int inf_step =
          static_cast<int>(std::ceil(map_->mp_->obstacles_inflation_ / map_->mp_->resolution_));
      for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
        for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
          for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z) {
            Eigen::Vector3i inf_pt(idx[0] + inf_x, idx[1] + inf_y, idx[2] + inf_z);
            if (!map_->isInMap(inf_pt)) continue;
            int inf_adr = map_->toAddress(inf_pt);
            map_->md_->occupancy_buffer_inflate_[inf_adr] = 1;
          }
    }
  }
}

void MultiMapManager::getChunkBoxes(
    vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset) {
  for (auto& box : chunk_boxes_) {
    if (box.valid_) {
      mins.push_back(box.min_);
      maxs.push_back(box.max_);
      if (reset) box.valid_ = false;
    }
  }
}

}  // namespace fast_planner

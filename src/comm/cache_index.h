
#ifndef LIVOX_ROS_DRIVER_CACHE_INDEX_H_
#define LIVOX_ROS_DRIVER_CACHE_INDEX_H_

#include <mutex>
#include <array>
#include <map>
#include <string>

#include "comm/comm.h"

namespace livox_ros {

class CacheIndex {
 public:
  CacheIndex();
  void Clear();
  int8_t GetFreeIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  int8_t GetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  int8_t GenerateIndexKey(const uint8_t livox_lidar_type, const uint32_t handle, std::string& key);
  void ResetIndex(LidarDevice *lidar);

 private:
  std::mutex index_mutex_;
  std::map<std::string, uint8_t> map_index_; /* key:handle/slot, val:index */
  std::array<bool, kMaxSourceLidar> index_cache_;
};

} // namespace livox_ros

# endif // LIVOX_ROS_DRIVER_CACHE_INDEX_H_

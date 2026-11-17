
// livox lidar data source

#ifndef LIVOX_ROS_DRIVER_LDS_H_
#define LIVOX_ROS_DRIVER_LDS_H_

#include <map>
#include <atomic>

#include "comm/semaphore.h"
#include "comm/comm.h"
#include "comm/cache_index.h"

namespace livox_ros {
/**
 * Lidar data source abstract.
 */
class Lds {
 public:
  Lds(const double publish_freq);
  virtual ~Lds();

  void StorageImuData(ImuData* imu_data);
  void StoragePointData(PointFrame* frame);

  void PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time);

  static void ResetLidar(LidarDevice *lidar);
  void ResetLds();

  void RequestExit();

  bool IsRequestExit() { return request_exit_.load(std::memory_order_acquire); }
  virtual void PrepareExit(void);

  // get publishing frequency
  double GetLdsFrequency() { return publish_freq_; }

 public:
  uint8_t lidar_count_;                 /**< Lidar access handle. */
  LidarDevice lidars_[kMaxSourceLidar]; /**< The index is the handle */
  Semaphore pcd_semaphore_;
  Semaphore imu_semaphore_;
  static CacheIndex cache_index_;
 protected:
  double publish_freq_;
 private:
  std::atomic<bool> request_exit_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_H_

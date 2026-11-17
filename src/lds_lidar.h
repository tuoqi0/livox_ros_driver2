
/** Livox LiDAR data source, data from dependent lidar */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <memory>
#include <mutex>
#include <vector>

#include "lds.h"
#include "comm/comm.h"

#include "livox_lidar_def.h"

#include "rapidjson/document.h"

namespace livox_ros {

class LdsLidar final : public Lds {
 public:
  static LdsLidar *GetInstance(double publish_freq) {
    printf("LdsLidar *GetInstance\n");
    static LdsLidar lds_lidar(publish_freq);
    return &lds_lidar;
  }

  bool InitLdsLidar(const std::string& path_name);
  bool Start();

  int DeInitLdsLidar(void);
 private:
  LdsLidar(double publish_freq);
  LdsLidar(const LdsLidar &) = delete;
  ~LdsLidar();
  LdsLidar &operator=(const LdsLidar &) = delete;

  bool ParseSummaryConfig();

  bool InitLidars();
  bool InitLivoxLidar();    // for new SDK

  bool LivoxLidarStart();

  void ResetLdsLidar(void);

  void SetLidarPubHandle();

  virtual void PrepareExit(void);

 public:
  std::mutex config_mutex_;

 private:
  std::string path_;
  LidarSummaryInfo lidar_summary_info_;

  volatile bool is_initialized_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_LIDAR_H_

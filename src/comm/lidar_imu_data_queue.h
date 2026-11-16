
#ifndef LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>

namespace livox_ros {

// Based on the IMU Data Type in Livox communication protocol
// TODO: add a link to the protocol
typedef struct {
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} RawImuPoint;

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  uint8_t slot;
  // union {
  //   uint8_t handle;
  //   uint8_t slot;
  // };
  uint64_t time_stamp;
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} ImuData;

class LidarImuDataQueue {
 public:
  void Push(ImuData* imu_data);
  bool Pop(ImuData& imu_data);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  std::list<ImuData> imu_data_queue_;
};

} // namespace

#endif // LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_


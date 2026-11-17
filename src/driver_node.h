
#ifndef LIVOX_DRIVER_NODE_H
#define LIVOX_DRIVER_NODE_H

#include <livox_ros_driver2/ros2_headers.h>

namespace livox_ros {

class Lddc;

class DriverNode final : public rclcpp::Node {
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
};

} // namespace livox_ros

#endif // LIVOX_DRIVER_NODE_H

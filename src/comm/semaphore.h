
#ifndef LIVOX_ROS_DRIVER_SEMAPHORE_H_
#define LIVOX_ROS_DRIVER_SEMAPHORE_H_

#include <mutex>
#include <condition_variable>

namespace livox_ros {

class Semaphore {
 public:
  explicit Semaphore(int count = 0) : count_(count) {
  }
  void Signal();
  void Wait();
  int GetCount() {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  int count_;
};

} // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_SEMAPHORE_H_

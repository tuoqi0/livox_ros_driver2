
#include "driver_node.h"
#include "lddc.h"

namespace livox_ros {

DriverNode::~DriverNode() {
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

} // namespace livox_ros






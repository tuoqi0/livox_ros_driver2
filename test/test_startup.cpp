// Test the real source initialization and info/command callbacks. SDK entry
// points below are deterministic transport stubs; no SDK/device thread starts.
#include <robot_core/test/doctest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unistd.h>

#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "lds.h"
#include "rapidjson/document.h"
#define private public
#include "comm/pub_handler.h"
#include "lds_lidar.h"
#undef private
#include "call_back/livox_lidar_callback.h"

namespace {
using namespace livox_ros;
constexpr const char* kConfig = R"json({
  "lidar_summary_info": {"lidar_type": 8},
  "lidar_configs": [
    {"ip": "127.0.0.81", "pcl_data_type": 1, "pattern_mode": 0,
     "blind_spot_set": 42, "dual_emit_en": 0,
     "extrinsic_parameter": {"roll": 1.25, "pitch": -2.5, "yaw": 3.75,
                             "x": 101, "y": -202, "z": 303}},
    {"ip": "127.0.0.82",
     "extrinsic_parameter": {"roll": -4.5, "pitch": 5.25, "yaw": -6.75,
                             "x": -404, "y": 505, "z": -606}}
  ]
})json";

class ConfigFile {
 public:
  ConfigFile() {
    const int fd = mkstemp(path_);
    REQUIRE((fd >= 0));
    FILE* file = fdopen(fd, "wb");
    if (!file) close(fd);
    REQUIRE((file != nullptr));
    const size_t length = std::strlen(kConfig);
    const size_t written = std::fwrite(kConfig, 1, length, file);
    const int closed = std::fclose(file);
    REQUIRE((written == length));
    REQUIRE((closed == 0));
  }
  ~ConfigFile() { std::remove(path_); }
  const char* path() const { return path_; }
 private:
  char path_[64] = "/tmp/livox-startup-contract-XXXXXX";
};

enum class Kind { DataType, Pattern, Blind, Dual, Attitude, WorkMode, Imu };
struct Command {
  Kind kind;
  uint32_t handle;
  int64_t value;
  LivoxLidarInstallAttitude attitude{};
  LivoxLidarAsyncControlCallback callback;
  void* client;
};
struct Registration {
  LivoxLidarInfoChangeCallback callback;
  void* client;
};
struct Metadata {
  bool cache_found = false;
  bool extrinsic_found = false;
  uint8_t lidar_type = 0;
  uint32_t handle = 0;
  UserLivoxLidarConfig config{};
  ExtParameter extrinsic{};
};
struct InitObservation {
  bool callback_registered = false;
  bool correct_client = false;
  bool correct_path = false;
  bool default_optional_arguments = false;
  std::array<Metadata, 2> metadata;
  std::array<bool, 2> sampling_after_notification{{false, false}};
  unsigned notifications = 0;
};

LdsLidar* expected_source = nullptr;
std::string expected_path;
std::array<uint32_t, 2> handles{};
LivoxLidarInfoChangeCallback info_callback = nullptr;
void* info_client = nullptr;
std::vector<Registration> registrations;
std::vector<InitObservation> init_observations;
std::vector<Command> commands;
std::deque<Command> pending_commands;
unsigned acknowledged_commands = 0;
unsigned sdk_uninit_calls = 0;
unsigned observer_add_calls = 0;
unsigned observer_remove_calls = 0;
bool fail_next_init = true;
bool sdk_uninit_saw_stopped_worker = false;
uint16_t active_observer = 0;

Metadata metadataFor(uint32_t handle) {
  Metadata result;
  uint8_t index = 0;
  result.cache_found =
      expected_source->cache_index_.GetIndex(kLivoxLidarType, handle, index) == 0;
  if (result.cache_found) {
    const LidarDevice& device = expected_source->lidars_[index];
    result.lidar_type = device.lidar_type;
    result.handle = device.handle;
    result.config = device.livox_config;
  }
  const auto found = pub_handler().lidar_extrinsics_.find(handle);
  result.extrinsic_found = found != pub_handler().lidar_extrinsics_.end();
  if (result.extrinsic_found) result.extrinsic = found->second.param;
  return result;
}

livox_status queueCommand(Kind kind, uint32_t handle, int64_t value,
                           LivoxLidarAsyncControlCallback callback, void* client,
                           const LivoxLidarInstallAttitude* attitude = nullptr) {
  Command command{kind, handle, value, {}, callback, client};
  if (attitude) command.attitude = *attitude;
  commands.push_back(command);
  pending_commands.push_back(command);
  return kLivoxLidarStatusSuccess;
}

void acknowledgeCommands() {
  // InfoChange holds config_mutex_ while issuing configuration requests. Like
  // the SDK transport, complete these requests only after that callback exits.
  while (!pending_commands.empty()) {
    const Command command = pending_commands.front();
    pending_commands.pop_front();
    if (command.callback) {
      LivoxLidarAsyncControlResponse response{};
      command.callback(kLivoxLidarStatusSuccess, command.handle, &response,
                       command.client);
      ++acknowledged_commands;
    }
  }
}
}

extern "C" void DisableLivoxSdkConsoleLogger() {}

extern "C" void SetLivoxLidarInfoChangeCallback(
    LivoxLidarInfoChangeCallback callback, void* client) {
  info_callback = callback;
  info_client = client;
  registrations.push_back({callback, client});
}

extern "C" bool LivoxLidarSdkInit(const char* path, const char* host_ip,
                                   const LivoxLidarLoggerCfgInfo* log_config) {
  InitObservation observation;
  observation.callback_registered =
      info_callback == LivoxLidarCallback::LidarInfoChangeCallback;
  observation.correct_client = info_client == expected_source;
  observation.correct_path = path && expected_path == path;
  observation.default_optional_arguments = host_ip && host_ip[0] == '\0' && !log_config;
  for (size_t i = 0; i < handles.size(); ++i)
    observation.metadata[i] = metadataFor(handles[i]);

  if (fail_next_init) {
    // Exercise the existing SDK-init failure return without partially starting
    // a transport. This does not claim to cover SDK partial-init cleanup.
    fail_next_init = false;
    init_observations.push_back(observation);
    return false;
  }

  for (size_t i = 0; i < handles.size(); ++i) {
    if (info_callback) {
      LivoxLidarInfo info{};
      info.dev_type = kLivoxLidarTypeMid360;
      info_callback(handles[i], &info, info_client);
      ++observation.notifications;
      acknowledgeCommands();
    }
    uint8_t index = 0;
    if (expected_source->cache_index_.GetIndex(kLivoxLidarType, handles[i], index) == 0)
      observation.sampling_after_notification[i] =
          expected_source->lidars_[index].connect_state == kConnectStateSampling;
  }
  init_observations.push_back(observation);
  return true;
}

extern "C" void LivoxLidarSdkUninit() {
  ++sdk_uninit_calls;
  const auto& handler = pub_handler();
  sdk_uninit_saw_stopped_worker =
      handler.is_quit_.load() && !handler.point_process_thread_ && !active_observer;
  info_callback = nullptr;
  info_client = nullptr;
}

extern "C" uint16_t LivoxLidarAddPointCloudObserver(
    LivoxLidarPointCloudObserver, void*) {
  ++observer_add_calls;
  active_observer = 17;
  return active_observer;
}

extern "C" void LivoxLidarRemovePointCloudObserver(uint16_t id) {
  ++observer_remove_calls;
  if (id == active_observer) active_observer = 0;
}

extern "C" livox_status SetLivoxLidarPclDataType(uint32_t handle,
    LivoxLidarPointDataType type, LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::DataType, handle, static_cast<int64_t>(type), callback, client);
}
extern "C" livox_status SetLivoxLidarScanPattern(uint32_t handle,
    LivoxLidarScanPattern pattern, LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::Pattern, handle, static_cast<int64_t>(pattern), callback, client);
}
extern "C" livox_status SetLivoxLidarBlindSpot(uint32_t handle,
    uint32_t blind_spot, LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::Blind, handle, blind_spot, callback, client);
}
extern "C" livox_status SetLivoxLidarDualEmit(uint32_t handle,
    bool enabled, LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::Dual, handle, enabled ? 1 : 0, callback, client);
}
extern "C" livox_status SetLivoxLidarInstallAttitude(uint32_t handle,
    LivoxLidarInstallAttitude* attitude, LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::Attitude, handle, 0, callback, client, attitude);
}
extern "C" livox_status SetLivoxLidarWorkMode(uint32_t handle,
    LivoxLidarWorkMode mode, LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::WorkMode, handle, static_cast<int64_t>(mode), callback, client);
}
extern "C" livox_status EnableLivoxLidarImuData(uint32_t handle,
    LivoxLidarAsyncControlCallback callback, void* client) {
  return queueCommand(Kind::Imu, handle, 0, callback, client);
}

TEST_CASE("LivoxStartup.EarlyNotificationUsesPreparedDevices") {
  ConfigFile file;
  expected_path = file.path();
  handles = {{IpStringToNum("127.0.0.81"), IpStringToNum("127.0.0.82")}};
  expected_source = LdsLidar::GetInstance(10.0);
  REQUIRE((expected_source != nullptr));
  REQUIRE_FALSE((expected_source->is_initialized_));
  REQUIRE((pub_handler().point_process_thread_ == nullptr));

  const bool failed_init = expected_source->InitLdsLidar(file.path());
  CHECK_FALSE((failed_init));
  CHECK_FALSE((expected_source->is_initialized_));
  CHECK((info_callback == nullptr));
  CHECK((info_client == nullptr));
  CHECK((observer_add_calls == 0));
  CHECK((pub_handler().point_process_thread_ == nullptr));
  CHECK_FALSE((pub_handler().is_quit_.load()));
  CHECK((sdk_uninit_calls == 0));

  const bool initialized = expected_source->InitLdsLidar(file.path());
  const bool worker_started = pub_handler().point_process_thread_ &&
                              pub_handler().point_process_thread_->joinable();
  const bool source_initialized = expected_source->is_initialized_;
  // Clean the real worker before reporting an assertion failure. No restart,
  // retry event or direct test call to InfoChange repairs a missed notification.
  const int deinit_result = initialized ? expected_source->DeInitLdsLidar() : -1;
  CHECK((initialized));
  CHECK((source_initialized));
  CHECK((worker_started));
  CHECK((deinit_result == 0));
  CHECK((sdk_uninit_calls == 1));
  CHECK((sdk_uninit_saw_stopped_worker));
  CHECK((observer_add_calls == 1));
  CHECK((observer_remove_calls == 1));
  CHECK((pub_handler().point_process_thread_ == nullptr));
  REQUIRE((init_observations.size() == 2));
  CHECK((registrations.size() == 3));
  if (registrations.size() == 3) {
    CHECK((registrations[0].callback == LivoxLidarCallback::LidarInfoChangeCallback));
    CHECK((registrations[0].client == expected_source));
    CHECK((registrations[1].callback == nullptr));
    CHECK((registrations[1].client == nullptr));
    CHECK((registrations[2].callback == LivoxLidarCallback::LidarInfoChangeCallback));
    CHECK((registrations[2].client == expected_source));
  }

  const std::array<ExtParameter, 2> expected_extrinsics{{
      {1.25f, -2.5f, 3.75f, 101, -202, 303},
      {-4.5f, 5.25f, -6.75f, -404, 505, -606}}};
  for (size_t attempt = 0; attempt < init_observations.size(); ++attempt) {
    CAPTURE(attempt);
    const auto& observed = init_observations[attempt];
    CHECK((observed.callback_registered));
    CHECK((observed.correct_client));
    CHECK((observed.correct_path));
    CHECK((observed.default_optional_arguments));
    CHECK((observed.notifications == (attempt == 0 ? 0 : 2)));
    for (size_t device = 0; device < handles.size(); ++device) {
      CAPTURE(device);
      const Metadata& metadata = observed.metadata[device];
      const ExtParameter& extrinsic = expected_extrinsics[device];
      CHECK((metadata.cache_found));
      CHECK((metadata.extrinsic_found));
      CHECK((metadata.lidar_type == kLivoxLidarType));
      CHECK((metadata.handle == handles[device]));
      CHECK((metadata.config.handle == handles[device]));
      CHECK((metadata.config.pcl_data_type == (device == 0 ? 1 : -1)));
      CHECK((metadata.config.pattern_mode == (device == 0 ? 0 : -1)));
      CHECK((metadata.config.blind_spot_set == (device == 0 ? 42 : -1)));
      CHECK((metadata.config.dual_emit_en == (device == 0 ? 0 : -1)));
      CHECK((metadata.config.set_bits == 0));
      CHECK((metadata.config.get_bits == 0));
      CHECK((metadata.extrinsic.roll == extrinsic.roll));
      CHECK((metadata.extrinsic.pitch == extrinsic.pitch));
      CHECK((metadata.extrinsic.yaw == extrinsic.yaw));
      CHECK((metadata.extrinsic.x == extrinsic.x));
      CHECK((metadata.extrinsic.y == extrinsic.y));
      CHECK((metadata.extrinsic.z == extrinsic.z));
      CHECK((observed.sampling_after_notification[device] == (attempt == 1)));
    }
  }
  REQUIRE((commands.size() == 10));
  CHECK((acknowledged_commands == 10));
  CHECK((pending_commands.empty()));
  const std::array<Kind, 10> expected_kinds{{
      Kind::DataType, Kind::Pattern, Kind::Blind, Kind::Dual, Kind::Attitude,
      Kind::WorkMode, Kind::Imu, Kind::Attitude, Kind::WorkMode, Kind::Imu}};
  const std::array<int64_t, 10> expected_values{{
      1, 0, 42, 0, 0, kLivoxLidarNormal, 0, 0, kLivoxLidarNormal, 0}};
  for (size_t i = 0; i < commands.size(); ++i) {
    CAPTURE(i);
    const auto& command = commands[i];
    const size_t device = i < 7 ? 0 : 1;
    CHECK((command.kind == expected_kinds[i]));
    CHECK((command.handle == handles[device]));
    CHECK((command.value == expected_values[i]));
    CHECK((command.callback != nullptr));
    CHECK((command.client == (command.kind == Kind::WorkMode ? nullptr : expected_source)));
    if (command.kind == Kind::Attitude) {
      const auto& extrinsic = expected_extrinsics[device];
      CHECK((command.attitude.roll_deg == extrinsic.roll));
      CHECK((command.attitude.pitch_deg == extrinsic.pitch));
      CHECK((command.attitude.yaw_deg == extrinsic.yaw));
      CHECK((command.attitude.x == extrinsic.x));
      CHECK((command.attitude.y == extrinsic.y));
      CHECK((command.attitude.z == extrinsic.z));
    }
  }
}

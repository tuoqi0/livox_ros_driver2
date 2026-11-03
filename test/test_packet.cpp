// Exercises the real SDK-observer callback, packet processors and publisher
// worker without SDK initialization, sockets, ROS or hardware.
#include <robot_core/test/doctest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "lds.h"
#include "rapidjson/document.h"
// No production hook: expose the existing queue only to schedule callback and
// worker order deterministically. All tested transformations remain real code.
#define private public
#include "comm/pub_handler.h"
#include "lds_lidar.h"
#undef private

namespace {
LivoxLidarPointCloudObserver observer = nullptr;
void* observer_client = nullptr;
uint16_t observer_id = 0;
uint16_t next_observer_id = 1;
unsigned sdk_uninit_calls = 0;
bool sdk_uninit_saw_observer_removed = false;
bool sdk_uninit_saw_worker_cleared = false;
bool sdk_uninit_saw_quit = false;
}

extern "C" uint16_t LivoxLidarAddPointCloudObserver(
    LivoxLidarPointCloudObserver callback, void* client) {
  observer = callback;
  observer_client = client;
  observer_id = next_observer_id++;
  return observer_id;
}

extern "C" void LivoxLidarRemovePointCloudObserver(uint16_t id) {
  if (id == observer_id) {
    observer = nullptr;
    observer_client = nullptr;
    observer_id = 0;
  }
}

extern "C" void LivoxLidarSdkUninit() {
  ++sdk_uninit_calls;
  const livox_ros::PubHandler& handler = livox_ros::pub_handler();
  sdk_uninit_saw_observer_removed =
      observer == nullptr && observer_client == nullptr && observer_id == 0;
  sdk_uninit_saw_worker_cleared = handler.point_process_thread_ == nullptr;
  sdk_uninit_saw_quit = handler.is_quit_.load();
}

namespace {
using namespace livox_ros;
constexpr uint32_t kFirstHandle = 0x0100007f;
constexpr uint32_t kSecondHandle = 0x0200007f;
constexpr uint64_t kEpochNs = 1700000000123456789ULL;

template<class T>
void writeRecord(std::vector<uint8_t>& bytes, size_t index, const T& record) {
  std::memcpy(bytes.data() + index * sizeof(T), &record, sizeof(T));
}

std::vector<uint8_t> makePointData(uint8_t type, uint16_t count,
                                 uint8_t marker_base = 0) {
  size_t size = 0;
  switch (type) {
    case kLivoxLidarCartesianCoordinateHighData:
      size = sizeof(LivoxLidarCartesianHighRawPoint); break;
    case kLivoxLidarCartesianCoordinateLowData:
      size = sizeof(LivoxLidarCartesianLowRawPoint); break;
    case kLivoxLidarSphericalCoordinateData:
      size = sizeof(LivoxLidarSpherPoint); break;
    case kLivoxLidarDoubleEchoData:
      size = sizeof(LivoxLidarDoubleEchoRawPoint); break;
    default:
      FAIL("unsupported test point encoding");
  }
  std::vector<uint8_t> data(size * count);
  for (uint16_t i = 0; i < count; ++i) {
    const uint8_t intensity = static_cast<uint8_t>(marker_base + i);
    const uint8_t tag = static_cast<uint8_t>(0x10 + i % 3);
    switch (type) {
      case kLivoxLidarCartesianCoordinateHighData: {
        LivoxLidarCartesianHighRawPoint p{};
        p.x = (i + 1) * 1000; p.reflectivity = intensity; p.tag = tag;
        writeRecord(data, i, p); break;
      }
      case kLivoxLidarCartesianCoordinateLowData: {
        LivoxLidarCartesianLowRawPoint p{};
        p.x = (i + 1) * 100; p.reflectivity = intensity; p.tag = tag;
        writeRecord(data, i, p); break;
      }
      case kLivoxLidarSphericalCoordinateData: {
        LivoxLidarSpherPoint p{};
        p.depth = (i + 1) * 1000; p.theta = 9000;
        p.reflectivity = intensity; p.tag = tag;
        writeRecord(data, i, p); break;
      }
      case kLivoxLidarDoubleEchoData: {
        LivoxLidarDoubleEchoRawPoint p{};
        p.x1 = (i + 1) * 1000; p.x2 = (i + 1) * 2000;
        p.reflectivity1 = intensity; p.reflectivity2 = intensity;
        p.tag1 = tag; p.tag2 = static_cast<uint8_t>(tag + 4);
        writeRecord(data, i, p); break;
      }
    }
  }
  return data;
}

std::vector<uint8_t> makeWirePacket(uint8_t type, uint8_t time_type,
                                    uint64_t stamp, uint16_t count,
                                    uint16_t duration_units,
                                    const std::vector<uint8_t>& data) {
  const size_t header_size = offsetof(LivoxLidarEthernetPacket, data);
  std::vector<uint8_t> bytes(header_size + data.size());
  LivoxLidarEthernetPacket* packet =
      reinterpret_cast<LivoxLidarEthernetPacket*>(bytes.data());
  packet->version = 0;
  packet->length = static_cast<uint16_t>(bytes.size());
  packet->time_interval = duration_units;
  packet->dot_num = count;
  packet->data_type = type;
  packet->time_type = time_type;
  std::memcpy(packet->timestamp, &stamp, sizeof(stamp));
  if (!data.empty()) std::memcpy(packet->data, data.data(), data.size());
  return bytes;
}

struct FrameCopy {
  std::vector<uint32_t> handles;
  std::vector<uint64_t> bases;
  std::vector<std::vector<PointXyzlt>> clouds;
};

struct Fixture {
  PubHandler handler;
  std::vector<ImuData> imus;
  std::vector<FrameCopy> frames;
  std::function<void()> frame_received;

  Fixture() {
    handler.SetPointCloudsCallback([this](PointFrame* frame, void*) {
      FrameCopy copy;
      for (uint8_t i = 0; i < frame->lidar_num; ++i) {
        const PointPacket& p = frame->lidar_point[i];
        copy.handles.push_back(static_cast<uint32_t>(p.handle));
        copy.bases.push_back(static_cast<uint64_t>(frame->base_time[i]));
        const PointXyzlt* data = p.points;
        const size_t size = p.points_num;
        copy.clouds.emplace_back(data, data + size);
      }
      frames.push_back(std::move(copy));
      if (frame_received) frame_received();
    }, nullptr);
    handler.SetImuDataCallback([this](ImuData* imu, void*) {
      imus.push_back(*imu);
    }, nullptr);
  }

  void deliver(std::vector<uint8_t>& wire, uint32_t handle = kFirstHandle) {
    REQUIRE((observer != nullptr));
    REQUIRE((observer_client == &handler));
    observer(handle, kLivoxLidarTypeMid360,
             reinterpret_cast<LivoxLidarEthernetPacket*>(wire.data()),
             observer_client);
  }

  void points(uint64_t stamp, uint16_t count = 96,
              uint16_t duration_units = 9500,
              uint8_t type = kLivoxLidarCartesianCoordinateHighData,
              uint8_t time_type = kTimestampTypeGptpOrPtp,
              uint32_t handle = kFirstHandle, uint8_t marker_base = 0) {
    const auto data = makePointData(type, count, marker_base);
    auto wire = makeWirePacket(type, time_type, stamp, count, duration_units, data);
    deliver(wire, handle);
  }

  void imu(uint64_t stamp, uint8_t time_type,
           uint32_t handle = kFirstHandle) {
    LivoxLidarImuRawPoint value{1.25f, -2.5f, 0.125f, -0.75f, 0.5f, 1.0f};
    std::vector<uint8_t> data(sizeof(value));
    std::memcpy(data.data(), &value, sizeof(value));
    auto wire = makeWirePacket(kLivoxLidarImuData, time_type, stamp, 1, 0, data);
    deliver(wire, handle);
  }

  RawPacket pop() {
    std::lock_guard<std::mutex> lock(handler.packet_mutex_);
    REQUIRE((!handler.raw_packet_queue_.empty()));
    RawPacket packet = handler.raw_packet_queue_.front();
    handler.raw_packet_queue_.pop_front();
    return packet;
  }
};

const std::array<uint8_t, 4> point_types{{
    kLivoxLidarCartesianCoordinateHighData,
    kLivoxLidarCartesianCoordinateLowData,
    kLivoxLidarSphericalCoordinateData,
    kLivoxLidarDoubleEchoData}};
}

TEST_CASE("LivoxPacket.PtpAndGpsPreserveNanosecondsAndRawImuUnits") {
  for (uint8_t mode : {uint8_t(kTimestampTypeGptpOrPtp),
                       uint8_t(kTimestampTypeGps)}) {
    CAPTURE(mode);
    Fixture fixture;
    fixture.points(kEpochNs, 96, 9500,
                   kLivoxLidarCartesianCoordinateHighData, mode);
    fixture.imu(kEpochNs + 5000000, mode);
    const RawPacket packet = fixture.pop();
    CHECK((packet.time_stamp == kEpochNs));
    CHECK((packet.point_duration == 950000));
    CHECK((packet.timestamp_sync));
    CHECK((packet.handle == kFirstHandle));
    REQUIRE((fixture.imus.size() == 1));
    const ImuData& imu = fixture.imus.front();
    CHECK((imu.handle == kFirstHandle));
    CHECK((imu.time_stamp == kEpochNs + 5000000));
    CHECK((imu.gyro_x == 1.25f));
    CHECK((imu.gyro_y == -2.5f));
    CHECK((imu.gyro_z == 0.125f));
    CHECK((imu.acc_x == -0.75f));
    CHECK((imu.acc_y == 0.5f));
    CHECK((imu.acc_z == 1.0f));
    CHECK((fixture.frames.empty()));
  }
}

TEST_CASE("LivoxPacket.AllEncodingsPlace96SamplesAcross95Intervals") {
  // The official wire protocol defines time_interval as last minus first;
  // this fixture has 95 exact 10 us intervals, never 96 intervals.
  for (uint8_t type : point_types) {
    CAPTURE(type);
    Fixture fixture;
    fixture.points(kEpochNs, 96, 9500, type);
    RawPacket packet = fixture.pop();
    LidarPubHandler parser;
    parser.PointCloudProcess(packet);
    CHECK((parser.GetLidarBaseTime() == kEpochNs));
    CHECK((parser.GetRecentTimeStamp() == kEpochNs + 950000));
    std::vector<PointXyzlt> points;
    parser.GetLidarPointClouds(points);
    const size_t echoes = type == kLivoxLidarDoubleEchoData ? 2 : 1;
    REQUIRE((points.size() == 96 * echoes));
    for (size_t i = 0; i < 96; ++i) {
      for (size_t echo = 0; echo < echoes; ++echo) {
        const PointXyzlt& p = points[i * echoes + echo];
        CHECK((p.offset_time == kEpochNs + i * 10000));
        CHECK((p.line == i % 4));
        CHECK((p.tag == 0x10 + i % 3 + (echo ? 4 : 0)));
        CHECK((p.intensity == static_cast<float>(i)));
        CHECK((p.x == doctest::Approx((i + 1) * (echo ? 2.0 : 1.0)).epsilon(1e-6)));
        CHECK((p.y == doctest::Approx(0).epsilon(1e-6)));
        CHECK((p.z == doctest::Approx(0).epsilon(1e-6)));
      }
    }
  }
}

TEST_CASE("LivoxPacket.NonIntegralSpacingRetainsTheExactDeclaredLastPoint") {
  // 100 us / 95 is nonintegral. The first division must not truncate the
  // interval before multiplication; check independent fixed points as well.
  for (uint8_t type : point_types) {
    CAPTURE(type);
    Fixture fixture;
    fixture.points(kEpochNs, 96, 1000, type);
    RawPacket packet = fixture.pop();
    LidarPubHandler parser;
    parser.PointCloudProcess(packet);
    std::vector<PointXyzlt> points;
    parser.GetLidarPointClouds(points);
    const size_t echoes = type == kLivoxLidarDoubleEchoData ? 2 : 1;
    REQUIRE((points.size() == 96 * echoes));
    CHECK((points.front().offset_time == kEpochNs));
    CHECK((points[echoes].offset_time == kEpochNs + 1052));
    CHECK((points[47 * echoes].offset_time == kEpochNs + 49473));
    CHECK((points[94 * echoes].offset_time == kEpochNs + 98947));
    CHECK((points.back().offset_time == kEpochNs + 100000));
    for (size_t i = 1; i < points.size(); ++i)
      CHECK((points[i - 1].offset_time <= points[i].offset_time));
  }
}

TEST_CASE("LivoxPacket.SingleSampleHasItsPacketTimestampInEveryEncoding") {
  for (uint8_t type : point_types) {
    CAPTURE(type);
    Fixture fixture;
    fixture.points(kEpochNs, 1, 0, type);
    RawPacket packet = fixture.pop();
    LidarPubHandler parser;
    parser.PointCloudProcess(packet);
    CHECK((parser.GetLidarBaseTime() == kEpochNs));
    CHECK((parser.GetRecentTimeStamp() == kEpochNs));
    std::vector<PointXyzlt> points;
    parser.GetLidarPointClouds(points);
    REQUIRE((points.size() == (type == kLivoxLidarDoubleEchoData ? 2 : 1)));
    for (const PointXyzlt& point : points) CHECK((point.offset_time == kEpochNs));
  }
}

TEST_CASE("LivoxPacket.OutOfOrderPacketsUseTrueBoundsAndStablePointOrder") {
  Fixture fixture;
  // Four one-point packets arrive at 30, 10, 20, 20 us. Distinct intensity
  // identifies arrival order when timestamps tie, without copying sorting.
  fixture.points(kEpochNs + 30000, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeGptpOrPtp, kFirstHandle, 30);
  fixture.points(kEpochNs + 10000, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeGptpOrPtp, kFirstHandle, 10);
  fixture.points(kEpochNs + 20000, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeGptpOrPtp, kFirstHandle, 21);
  fixture.points(kEpochNs + 20000, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeGptpOrPtp, kFirstHandle, 22);
  LidarPubHandler parser;
  for (int i = 0; i < 4; ++i) {
    RawPacket packet = fixture.pop();
    parser.PointCloudProcess(packet);
  }
  CHECK((parser.GetLidarBaseTime() == kEpochNs + 10000));
  CHECK((parser.GetRecentTimeStamp() == kEpochNs + 30000));
  std::vector<PointXyzlt> points;
  parser.GetLidarPointClouds(points);
  REQUIRE((points.size() == 4));
  const std::array<uint64_t, 4> offsets{{10000, 20000, 20000, 30000}};
  const std::array<float, 4> intensities{{10, 21, 22, 30}};
  for (size_t i = 0; i < points.size(); ++i) {
    CHECK((points[i].offset_time == kEpochNs + offsets[i]));
    CHECK((points[i].intensity == intensities[i]));
  }
  CHECK((parser.GetLidarPointCloudsSize() == 0));
  CHECK((parser.GetLidarBaseTime() == 0));
  CHECK((parser.GetRecentTimeStamp() == 0));
}

TEST_CASE("LivoxPacket.QueuedPtpCloudIgnoresAnotherDevicesNoSyncImu") {
  Fixture fixture;
  fixture.points(900000000, 96, 9500);
  fixture.points(999050000, 96, 9500);
  fixture.imu(123000000, kTimestampTypeNoSync, kSecondHandle);
  REQUIRE((fixture.imus.size() == 1));
  CHECK((fixture.imus.front().handle == kSecondHandle));
  std::promise<void> published;
  auto done = published.get_future();
  fixture.frame_received = [&] {
    fixture.handler.RequestExit();
    published.set_value();
  };
  // A real worker consumes the original queue. Its production dispatch must
  // carry each packet's own mode to CheckTimer; test code never selects it.
  std::thread worker([&] { fixture.handler.RawDataProcess(); });
  const auto status = done.wait_for(std::chrono::seconds(1));
  fixture.handler.RequestExit();
  fixture.handler.packet_condition_.notify_all();
  worker.join();
  REQUIRE((status == std::future_status::ready));
  REQUIRE((fixture.frames.size() == 1));
  const FrameCopy& frame = fixture.frames.front();
  REQUIRE((frame.handles.size() == 1));
  CHECK((frame.handles[0] == kFirstHandle));
  CHECK((frame.bases[0] == 900000000));
  REQUIRE((frame.clouds[0].size() == 192));
  CHECK((frame.clouds[0].front().offset_time == frame.bases[0]));
  CHECK((frame.clouds[0].back().offset_time == 1000000000));
  for (size_t i = 1; i < frame.clouds[0].size(); ++i)
    CHECK((frame.clouds[0][i - 1].offset_time <= frame.clouds[0][i].offset_time));
}

TEST_CASE("LivoxPacket.NoSyncCadenceFlushesOnlyTheCurrentDevice") {
  constexpr uint32_t kThirdHandle = 0x0300007f;
  Fixture fixture;
  fixture.points(100000000, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeNoSync, kFirstHandle, 11);
  fixture.points(kEpochNs, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeGptpOrPtp, kSecondHandle, 22);
  fixture.points(300000000, 1, 0,
                 kLivoxLidarCartesianCoordinateHighData,
                 kTimestampTypeNoSync, kThirdHandle, 33);
  std::map<uint32_t, uint64_t> bases;
  for (int i = 0; i < 3; ++i) {
    RawPacket packet = fixture.pop();
    CHECK((packet.timestamp_sync == (packet.handle == kSecondHandle)));
    bases[packet.handle] = packet.time_stamp;
    auto& parser = fixture.handler.lidar_process_handlers_[packet.handle];
    parser.reset(new LidarPubHandler);
    parser->PointCloudProcess(packet);
  }
  const auto old_time = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  for (uint32_t handle : {kFirstHandle, kSecondHandle, kThirdHandle}) {
    fixture.handler.last_pub_times_[handle] = old_time;
    CHECK((fixture.handler.lidar_process_handlers_[handle]->GetLidarPointCloudsSize() == 1));
  }

  fixture.handler.CheckTimer(kFirstHandle, false);
  REQUIRE((fixture.frames.size() == 1));
  const FrameCopy& first = fixture.frames[0];
  REQUIRE((first.handles.size() == 1));
  CHECK((first.handles[0] == kFirstHandle));
  CHECK((first.bases[0] == bases[kFirstHandle]));
  REQUIRE((first.clouds[0].size() == 1));
  CHECK((first.clouds[0][0].intensity == 11.0f));
  CHECK((fixture.handler.lidar_process_handlers_[kFirstHandle]->GetLidarPointCloudsSize() == 0));
  CHECK((fixture.handler.last_pub_times_[kFirstHandle] ==
        old_time + std::chrono::nanoseconds(fixture.handler.publish_interval_)));
  for (uint32_t handle : {kSecondHandle, kThirdHandle}) {
    CHECK((fixture.handler.lidar_process_handlers_[handle]->GetLidarPointCloudsSize() == 1));
    CHECK((fixture.handler.lidar_process_handlers_[handle]->GetLidarBaseTime() == bases[handle]));
    CHECK((fixture.handler.lidar_process_handlers_[handle]->GetRecentTimeStamp() == bases[handle]));
    CHECK((fixture.handler.last_pub_times_[handle] == old_time));
  }

  fixture.handler.CheckTimer(kThirdHandle, false);
  REQUIRE((fixture.frames.size() == 2));
  const FrameCopy& third = fixture.frames[1];
  REQUIRE((third.handles.size() == 1));
  CHECK((third.handles[0] == kThirdHandle));
  CHECK((third.bases[0] == bases[kThirdHandle]));
  REQUIRE((third.clouds[0].size() == 1));
  CHECK((third.clouds[0][0].intensity == 33.0f));
  CHECK((fixture.handler.lidar_process_handlers_[kThirdHandle]->GetLidarPointCloudsSize() == 0));
  CHECK((fixture.handler.last_pub_times_[kThirdHandle] ==
        old_time + std::chrono::nanoseconds(fixture.handler.publish_interval_)));
  CHECK((fixture.handler.lidar_process_handlers_[kSecondHandle]->GetLidarPointCloudsSize() == 1));
  CHECK((fixture.handler.lidar_process_handlers_[kSecondHandle]->GetLidarBaseTime() == bases[kSecondHandle]));
  CHECK((fixture.handler.last_pub_times_[kSecondHandle] == old_time));
}

TEST_CASE("LivoxPacket.SourceDeinitJoinsItsRawWorkerBeforeSdkUninit") {
  // Construct only: InitLdsLidar/SDK initialization is deliberately never
  // called. This test owns the global publishing worker for its single life.
  LdsLidar* source = LdsLidar::GetInstance(10.0);
  REQUIRE((source != nullptr));
  CHECK_FALSE((source->is_initialized_));
  source->is_initialized_ = true;
  source->lidar_summary_info_.lidar_type = kLivoxLidarType;

  PubHandler& handler = pub_handler();
  REQUIRE_FALSE((handler.is_quit_.load()));
  REQUIRE((handler.point_process_thread_ == nullptr));
  handler.SetPointCloudsCallback([](PointFrame*, void*) {}, nullptr);
  REQUIRE((observer != nullptr));
  REQUIRE((observer_client == &handler));
  handler.SetPointCloudConfig(10.0);
  REQUIRE((handler.point_process_thread_ != nullptr));
  CHECK((handler.point_process_thread_->joinable()));
  CHECK((sdk_uninit_calls == 0));

  const int result = source->DeInitLdsLidar();
  CHECK((result == 0));
  CHECK((sdk_uninit_calls == 1));
  CHECK((sdk_uninit_saw_observer_removed));
  CHECK((sdk_uninit_saw_worker_cleared));
  CHECK((sdk_uninit_saw_quit));
  CHECK((handler.is_quit_.load()));
  CHECK((handler.point_process_thread_ == nullptr));
  CHECK((observer == nullptr));
}

TEST_CASE("LivoxPacket.ZeroPointNonImuPacketsNeverEnterTheWorkerQueue") {
  Fixture fixture;
  for (uint8_t type : point_types) {
    for (uint8_t mode : {uint8_t(kTimestampTypeNoSync),
                         uint8_t(kTimestampTypeGptpOrPtp),
                         uint8_t(kTimestampTypeGps)}) {
      CAPTURE(type);
      CAPTURE(mode);
      const auto data = makePointData(type, 0);
      auto wire = makeWirePacket(type, mode, kEpochNs, 0, 9500, data);
      fixture.deliver(wire);
      std::lock_guard<std::mutex> lock(fixture.handler.packet_mutex_);
      CHECK((fixture.handler.raw_packet_queue_.empty()));
      CHECK((fixture.handler.lidar_process_handlers_.empty()));
      CHECK((fixture.handler.last_pub_times_.empty()));
      CHECK((fixture.frames.empty()));
      CHECK((fixture.imus.empty()));
    }
  }
  fixture.points(kEpochNs, 1, 0);
  const RawPacket packet = fixture.pop();
  CHECK((packet.point_num == 1));
  CHECK((packet.time_stamp == kEpochNs));
}

TEST_CASE("LivoxPacket.ZeroDotHeaderDoesNotChangeImuDecoding") {
  Fixture fixture;
  LivoxLidarImuRawPoint value{1.25f, -2.5f, 0.125f, -0.75f, 0.5f, 1.0f};
  std::vector<uint8_t> data(sizeof(value));
  std::memcpy(data.data(), &value, sizeof(value));
  auto wire = makeWirePacket(kLivoxLidarImuData, kTimestampTypeGptpOrPtp,
                            kEpochNs, 0, 0, data);
  fixture.deliver(wire);
  REQUIRE((fixture.imus.size() == 1));
  const ImuData& imu = fixture.imus.front();
  CHECK((imu.handle == kFirstHandle));
  CHECK((imu.time_stamp == kEpochNs));
  CHECK((imu.gyro_x == 1.25f));
  CHECK((imu.gyro_y == -2.5f));
  CHECK((imu.gyro_z == 0.125f));
  CHECK((imu.acc_x == -0.75f));
  CHECK((imu.acc_y == 0.5f));
  CHECK((imu.acc_z == 1.0f));
  CHECK((fixture.handler.raw_packet_queue_.empty()));
}

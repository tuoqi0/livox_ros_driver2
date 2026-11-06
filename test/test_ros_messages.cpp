// Exercise the real driver packing functions without constructing DriverNode,
// opening a socket, or initializing the hardware SDK. Load all dependencies
// before exposing only Lddc's private packing methods to this test translation
// unit; no production declaration or implementation is copied.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
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

#include <robot_core/test/doctest.h>

#include "include/ros_headers.h"
#include "include/livox_ros_driver2.h"
#include "driver_node.h"
#include "lds.h"
#include "comm/pub_handler.h"
#include "comm/ldq.h"

#define private public
#include "lddc.h"
#undef private

namespace {
using namespace livox_ros;

void appendRawPoint(LidarPubHandler& handler, std::uint64_t stamp,
                    std::int32_t x_mm, std::uint8_t reflectivity) {
  LivoxLidarCartesianHighRawPoint point{};
  point.x = x_mm;
  point.y = -2000;
  point.z = 3000;
  point.reflectivity = reflectivity;
  point.tag = 0x10;
  RawPacket packet{};
  packet.lidar_type = kLivoxLidarType;
  packet.handle = 0x0c01a8c0u;
  packet.point_num = 1;
  packet.data_type = kLivoxLidarCartesianCoordinateHighData;
  packet.line_num = 4;
  packet.time_stamp = stamp;
  packet.raw_data.resize(sizeof(point));
  std::memcpy(packet.raw_data.data(), &point, sizeof(point));
  handler.PointCloudProcess(packet);
}

template <typename Value>
Value wireValue(const PointCloud2& cloud, std::size_t point,
                std::size_t offset) {
  Value value{};
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset,
              sizeof(value));
  return value;
}
}  // namespace

TEST_CASE("DriverRosMessage.OrderedNativePointsKeepCustomAndPointCloud2TimeContracts") {
  // The production LdsLidar singleton also has static storage. Ordinary Lds
  // keeps this test out of SDKInit and has the same zero-initialized queue
  // lifetime before its existing constructor calls ResetLds.
  static Lds source(10.0, kSourceRawLidar);
  std::string frame = "packet_contract_frame";
  Lddc control(kLivoxCustomMsg, 0, kSourceRawLidar, 10.0,
               frame);
  REQUIRE((control.RegisterLds(&source) == 0));
  source.lidars_[0].lidar_type = kLivoxLidarType;
  source.lidars_[0].handle = 0x0c01a8c0u;

  const std::uint64_t epoch = 1700000000000000000ULL;
  LidarPubHandler handler;
  appendRawPoint(handler, epoch + 1500, 1000, 11);
  appendRawPoint(handler, epoch + 500, 2000, 12);
  appendRawPoint(handler, epoch + 1000, 3000, 13);
  appendRawPoint(handler, epoch + 1000, 4000, 14);

  // min/max are checked before extraction, where CheckTimer consumes them.
  CHECK((handler.GetLidarBaseTime() == epoch + 500));
  CHECK((handler.GetRecentTimeStamp() == epoch + 1500));
  StoragePacket packet{};
  packet.base_time = handler.GetLidarBaseTime();
  handler.GetLidarPointClouds(packet.points);
  packet.points_num = static_cast<std::uint32_t>(packet.points.size());
  REQUIRE((packet.points.size() == 4));
  const std::array<std::uint64_t, 4> absolute{{
      epoch + 500, epoch + 1000, epoch + 1000, epoch + 1500}};
  const std::array<std::uint32_t, 4> offsets{{0, 500, 500, 1000}};
  const std::array<std::uint8_t, 4> intensities{{12, 13, 14, 11}};
  const std::array<float, 4> xs{{2.f, 3.f, 4.f, 1.f}};
  for (std::size_t i = 0; i < packet.points.size(); ++i) {
    CHECK((packet.points[i].offset_time == absolute[i]));
    CHECK((packet.points[i].intensity == intensities[i]));
    CHECK((packet.points[i].x == xs[i]));
  }
  CHECK((handler.GetLidarPointCloudsSize() == 0));

  CustomMsg custom;
  control.InitCustomMsg(custom, packet, 0);
  control.FillPointsToCustomMsg(custom, packet);
  CHECK((custom.header.frame_id == frame));
  CHECK((custom.timebase == epoch + 500));
  CHECK((rclcpp::Time(custom.header.stamp).nanoseconds() ==
        static_cast<std::int64_t>(custom.timebase)));
  CHECK((custom.point_num == packet.points_num));
  REQUIRE((custom.points.size() == packet.points.size()));
  for (std::size_t i = 0; i < custom.points.size(); ++i) {
    CHECK((custom.points[i].offset_time == offsets[i]));
    CHECK((custom.points[i].offset_time <= 1000));
    CHECK((custom.points[i].x == xs[i]));
    CHECK((custom.points[i].y == -2.f));
    CHECK((custom.points[i].z == 3.f));
    CHECK((custom.points[i].reflectivity == intensities[i]));
    CHECK((custom.points[i].tag == 0x10));
    CHECK((custom.points[i].line == 0));
  }

  PointCloud2 cloud;
  std::uint64_t stamp = 0;
  control.InitPointcloud2Msg(packet, cloud, stamp);
  CHECK((stamp == custom.timebase));
  CHECK((rclcpp::Time(cloud.header.stamp).nanoseconds() ==
        static_cast<std::int64_t>(stamp)));
  CHECK((cloud.header.frame_id == frame));
  CHECK((cloud.width == 4));
  CHECK((cloud.height == 1));
  CHECK_FALSE((cloud.is_bigendian));
  CHECK((cloud.is_dense));
  CHECK((cloud.point_step == sizeof(LivoxPointXyzrtlt)));
  CHECK((cloud.row_step == cloud.width * cloud.point_step));
  REQUIRE((cloud.data.size() == cloud.row_step));
  REQUIRE((cloud.fields.size() == 7));
  const std::array<std::string, 7> names{{
      "x", "y", "z", "intensity", "tag", "line", "timestamp"}};
  const std::array<std::uint32_t, 7> field_offsets{{0, 4, 8, 12, 16, 17, 18}};
  const std::array<std::uint8_t, 7> field_types{{
      PointField::FLOAT32, PointField::FLOAT32, PointField::FLOAT32,
      PointField::FLOAT32, PointField::UINT8, PointField::UINT8,
      PointField::FLOAT64}};
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    CHECK((cloud.fields[i].name == names[i]));
    CHECK((cloud.fields[i].offset == field_offsets[i]));
    CHECK((cloud.fields[i].datatype == field_types[i]));
    CHECK((cloud.fields[i].count == 1));
  }
  for (std::size_t i = 0; i < packet.points.size(); ++i) {
    CHECK((wireValue<float>(cloud, i, 0) == xs[i]));
    CHECK((wireValue<float>(cloud, i, 4) == -2.f));
    CHECK((wireValue<float>(cloud, i, 8) == 3.f));
    CHECK((wireValue<float>(cloud, i, 12) == intensities[i]));
    CHECK((wireValue<std::uint8_t>(cloud, i, 16) == 0x10));
    CHECK((wireValue<std::uint8_t>(cloud, i, 17) == 0));
    // PointCloud2's existing Float64 format stores absolute ns, while Custom
    // uses exact uint32 relative ns. Do not claim lossless epoch-ns precision
    // from Float64; check its actual specified double conversion instead.
    CHECK((wireValue<double>(cloud, i, 18) ==
          static_cast<double>(absolute[i])));
    CHECK((wireValue<double>(cloud, i, 18) > 1e15));
  }
}

TEST_CASE("DriverRosMessage.UnsupportedPclFormatConsumesQueuedPacket") {
  static Lds source(10.0, kSourceRawLidar);
  std::string frame = "unsupported_format_frame";
  Lddc control(static_cast<int>(99), 0, kSourceRawLidar, 10.0, frame);
  REQUIRE((control.RegisterLds(&source) == 0));
  source.lidars_[0].connect_state = kConnectStateSampling;
  REQUIRE((InitQueue(&source.lidars_[0].data, 8)));

  PointXyzlt point{};
  point.x = 1.0F;
  PointPacket packet{};
  packet.points = &point;
  packet.points_num = 1;
  REQUIRE((QueuePushAny(&source.lidars_[0].data,
                        reinterpret_cast<std::uint8_t *>(&packet), 1234) == 1));
  CHECK_FALSE((QueueIsEmpty(&source.lidars_[0].data)));

  control.PollingLidarPointCloudData(0, &source.lidars_[0]);
  CHECK((QueueIsEmpty(&source.lidars_[0].data)));
}

TEST_CASE("DriverRosMessage.ImuPackingPreservesRawRadiansAndGUnits") {
  std::string frame = "unused_lidar_frame";
  Lddc control(kLivoxCustomMsg, 0, kSourceRawLidar, 10.0,
               frame);
  ImuData raw{};
  raw.time_stamp = 1700000000000000123ULL;
  raw.gyro_x = .125f;
  raw.gyro_y = -.25f;
  raw.gyro_z = .5f;
  raw.acc_x = .25f;
  raw.acc_y = -.5f;
  raw.acc_z = 1.f;
  ImuMsg message;
  std::uint64_t stamp = 0;
  control.InitImuMsg(raw, message, stamp);
  CHECK((stamp == raw.time_stamp));
  CHECK((rclcpp::Time(message.header.stamp).nanoseconds() ==
        static_cast<std::int64_t>(raw.time_stamp)));
  CHECK((message.header.frame_id == frame));
  CHECK((message.angular_velocity.x == static_cast<double>(raw.gyro_x)));
  CHECK((message.angular_velocity.y == static_cast<double>(raw.gyro_y)));
  CHECK((message.angular_velocity.z == static_cast<double>(raw.gyro_z)));
  CHECK((message.linear_acceleration.x == static_cast<double>(raw.acc_x)));
  CHECK((message.linear_acceleration.y == static_cast<double>(raw.acc_y)));
  CHECK((message.linear_acceleration.z == 1.0));
}

TEST_CASE("DriverRosMessage.RequestExitReleasesBothColdDistributeWaits") {
  static Lds source(10.0, kSourceRawLidar);
  std::string frame = "cold_exit_contract";
  Lddc control(kLivoxCustomMsg, 0, kSourceRawLidar, 10.0,
               frame);
  REQUIRE((control.RegisterLds(&source) == 0));
  REQUIRE_FALSE((source.IsRequestExit()));
  REQUIRE((source.IsAllQueueEmpty()));
  std::promise<void> point_started, imu_started;
  auto point_start = point_started.get_future();
  auto imu_start = imu_started.get_future();
  auto point_done = std::async(std::launch::async, [&] {
    point_started.set_value();
    control.DistributePointCloudData();
  });
  auto imu_done = std::async(std::launch::async, [&] {
    imu_started.set_value();
    control.DistributeImuData();
  });
  const auto ready = std::future_status::ready;
  const auto timeout = std::future_status::timeout;
  CHECK((point_start.wait_for(std::chrono::seconds(2)) == ready));
  CHECK((imu_start.wait_for(std::chrono::seconds(2)) == ready));
  CHECK((point_done.wait_for(std::chrono::milliseconds(100)) == timeout));
  CHECK((imu_done.wait_for(std::chrono::milliseconds(100)) == timeout));
  source.RequestExit();
  const bool point_released =
      point_done.wait_for(std::chrono::seconds(2)) == ready;
  const bool imu_released =
      imu_done.wait_for(std::chrono::seconds(2)) == ready;
  CHECK((point_released));
  CHECK((imu_released));
  if (!point_released || !imu_released) {
    // Failure cleanup only: an old RequestExit that does not signal must fail
    // the checks above, but must not strand this test process in future joins.
    // This does not alter production Distribute or add an after-Wait guard.
    source.pcd_semaphore_.Signal();
    source.imu_semaphore_.Signal();
  }
  point_done.get();
  imu_done.get();
  CHECK((source.IsRequestExit()));
  CHECK((source.IsAllQueueEmpty()));
}

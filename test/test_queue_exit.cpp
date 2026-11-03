#include <robot_core/test/doctest.h>

#include "comm/ldq.h"
#include "lds.h"

#include <atomic>
#include <array>
#include "comm/cache_index.h"
#include <chrono>
#include <cstdint>
#include <future>
#include <limits>
#include <thread>
#include <vector>

namespace livox_queue_exit_contract {
namespace {
using namespace livox_ros;

std::vector<PointXyzlt> payload(std::uint32_t sequence) {
  std::vector<PointXyzlt> points(sequence % 7 + 1);
  for (std::size_t index = 0; index < points.size(); ++index) {
    auto& point = points[index];
    point.x = static_cast<float>(sequence);
    point.y = static_cast<float>(index) * .25f;
    point.z = -static_cast<float>(sequence) * .5f;
    point.intensity = static_cast<float>(sequence % 256);
    point.tag = static_cast<std::uint8_t>(index + 16);
    point.line = static_cast<std::uint8_t>(index % 4);
    point.offset_time = std::uint64_t(sequence) * 1000000 + index;
  }
  return points;
}

PointPacket packet(std::vector<PointXyzlt>& points) {
  PointPacket result{};
  result.handle = 0x0c01a8c0U;
  result.lidar_type = kLivoxLidarType;
  result.points_num = static_cast<std::uint32_t>(points.size());
  result.points = points.data();
  return result;
}

bool matches(const StoragePacket& result, std::uint32_t sequence) {
  const auto expected = payload(sequence);
  if (result.base_time != std::uint64_t(sequence) * 100000000 ||
      result.points_num != expected.size() || result.points.size() != expected.size()) return false;
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const auto& a = result.points[index];
    const auto& b = expected[index];
    if (a.x != b.x || a.y != b.y || a.z != b.z || a.intensity != b.intensity ||
        a.tag != b.tag || a.line != b.line || a.offset_time != b.offset_time) return false;
  }
  return true;
}

void push(LidarDataQueue& queue, std::uint32_t sequence) {
  auto points = payload(sequence);
  auto input = packet(points);
  QueuePushAny(&queue, reinterpret_cast<std::uint8_t*>(&input),
               std::uint64_t(sequence) * 100000000);
}
}  // namespace

TEST_CASE("Livox point queue preserves rounded capacity and full queue drop policy") {
  Lds source(10.0, 0);
  auto& queue = source.lidars_[0].data;
  REQUIRE((InitQueue(&queue, 3)));
  CHECK((queue.size == 4));
  CHECK((QueueUnusedSize(&queue) == 4));
  for (std::uint32_t sequence = 1; sequence <= 4; ++sequence) {
    auto points = payload(sequence);
    auto input = packet(points);
    source.PushLidarData(&input, 0, std::uint64_t(sequence) * 100000000);
  }
  CHECK((QueueIsFull(&queue)));
  CHECK((QueueUsedSize(&queue) == 4));
  CHECK((QueueUnusedSize(&queue) == 0));
  auto points = payload(99);
  auto dropped = packet(points);
  source.PushLidarData(&dropped, 0, 9900000000ULL);
  CHECK((QueueUsedSize(&queue) == 4));
  for (std::uint32_t sequence = 1; sequence <= 4; ++sequence) {
    StoragePacket output;
    REQUIRE((QueuePop(&queue, &output)));
    CHECK((matches(output, sequence)));
  }
  CHECK((QueueIsEmpty(&queue)));
  CHECK_FALSE((QueueIsFull(&queue)));
}

TEST_CASE("Livox point queue preserves slot and unsigned counter wrap order") {
  LidarDataQueue queue;
  REQUIRE((InitQueue(&queue, 4)));
  const auto near_wrap = std::numeric_limits<std::uint32_t>::max() - 1;
  queue.rd_idx.store(near_wrap, std::memory_order_relaxed);
  queue.wr_idx.store(near_wrap, std::memory_order_relaxed);
  for (std::uint32_t sequence = 1; sequence <= 4; ++sequence) push(queue, sequence);
  CHECK((QueueIsFull(&queue)));
  CHECK((QueueUsedSize(&queue) == 4));
  for (std::uint32_t sequence = 1; sequence <= 4; ++sequence) {
    StoragePacket output;
    REQUIRE((QueuePrePop(&queue, &output)));
    CHECK((matches(output, sequence)));
    CHECK((QueueUsedSize(&queue) == 5 - sequence));
    QueuePopUpdate(&queue);
  }
  CHECK((QueueIsEmpty(&queue)));
  CHECK((DeInitQueue(&queue)));
}

TEST_CASE("Livox point queue deinitialization and reinitialization retain ownership") {
  LidarDataQueue queue;
  CHECK((queue.storage_packet.load(std::memory_order_acquire) == nullptr));
  CHECK((DeInitQueue(&queue)));
  CHECK((DeInitQueue(&queue)));
  REQUIRE((InitQueue(&queue, 3)));
  push(queue, 1);
  CHECK((DeInitQueue(&queue)));
  CHECK((queue.storage_packet.load(std::memory_order_acquire) == nullptr));
  CHECK((queue.size == 0));
  CHECK((QueueIsEmpty(&queue)));
  CHECK((DeInitQueue(&queue)));
  REQUIRE((InitQueue(&queue, 8)));
  push(queue, 2);
  StoragePacket output;
  REQUIRE((QueuePop(&queue, &output)));
  CHECK((matches(output, 2)));
  push(queue, 3);
  ResetQueue(&queue);  // Reset is only legal after both worker threads stopped.
  CHECK((QueueIsEmpty(&queue)));
  push(queue, 4);
  REQUIRE((QueuePop(&queue, &output)));
  CHECK((matches(output, 4)));
  CHECK((DeInitQueue(&queue)));
}

TEST_CASE("Livox point queue publishes complete packets from one real producer to one real consumer") {
  LidarDataQueue queue;
  constexpr std::uint32_t count = 10000;
  std::atomic<bool> failed{false};
  std::atomic<bool> initialized{false};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  std::thread producer([&] {
    if (!InitQueue(&queue, 8)) { failed.store(true); return; }
    initialized.store(true, std::memory_order_release);
    for (std::uint32_t sequence = 1; sequence <= count; ++sequence) {
      while (QueueIsFull(&queue)) {
        if (failed.load() || std::chrono::steady_clock::now() >= deadline) {
          failed.store(true); return;
        }
        std::this_thread::yield();
      }
      if (failed.load()) return;
      push(queue, sequence);
    }
  });
  std::thread consumer([&] {
    // Like the real Lddc consumer, inspect the publication pointer before
    // consuming immutable dimensions or packet slots. No out-of-band ready
    // flag supplies the queue's initialization synchronization.
    while (queue.storage_packet.load(std::memory_order_acquire) == nullptr) {
      if (failed.load() || std::chrono::steady_clock::now() >= deadline) {
        failed.store(true); return;
      }
      std::this_thread::yield();
    }
    for (std::uint32_t sequence = 1; sequence <= count; ++sequence) {
      while (QueueIsEmpty(&queue)) {
        if (failed.load() || std::chrono::steady_clock::now() >= deadline) {
          failed.store(true); return;
        }
        std::this_thread::yield();
      }
      StoragePacket output;
      const bool read = QueuePrePop(&queue, &output);
      if (!read || !matches(output, sequence)) { failed.store(true); return; }
      QueuePopUpdate(&queue);
    }
  });
  producer.join();
  consumer.join();
  CHECK((initialized.load(std::memory_order_acquire)));
  CHECK_FALSE((failed.load()));
  CHECK((QueueIsEmpty(&queue)));
  CHECK((QueueUsedSize(&queue) == 0));
  CHECK((DeInitQueue(&queue)));
}

TEST_CASE("Livox cold idle exit wakes both existing point and IMU waiters") {
  Lds source(10.0, 0);
  CHECK_FALSE((source.IsRequestExit()));
  CHECK((source.pcd_semaphore_.GetCount() == 0));
  CHECK((source.imu_semaphore_.GetCount() == 0));
  std::promise<void> point_done, imu_done;
  auto point_result = point_done.get_future();
  auto imu_result = imu_done.get_future();
  std::atomic<int> entered{0};
  std::thread point([&] {
    entered.fetch_add(1);
    source.pcd_semaphore_.Wait();
    point_done.set_value();
  });
  std::thread imu([&] {
    entered.fetch_add(1);
    source.imu_semaphore_.Wait();
    imu_done.set_value();
  });
  const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (entered.load() != 2 && std::chrono::steady_clock::now() < start_deadline)
    std::this_thread::yield();
  const bool both_entered = entered.load() == 2;
  const bool point_blocked = point_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout;
  const bool imu_blocked = imu_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout;
  source.RequestExit();
  const bool point_woke = point_result.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  const bool imu_woke = imu_result.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  // Cleanup also makes a failed original implementation terminate normally;
  // these rescue signals do not change the recorded wake assertions.
  if (!point_woke) source.pcd_semaphore_.Signal();
  if (!imu_woke) source.imu_semaphore_.Signal();
  point.join();
  imu.join();
  CHECK((both_entered));
  CHECK((point_blocked));
  CHECK((imu_blocked));
  CHECK((point_woke));
  CHECK((imu_woke));
  CHECK((source.IsRequestExit()));
  source.CleanRequestExit();
  CHECK_FALSE((source.IsRequestExit()));
}
}  // namespace livox_queue_exit_contract


TEST_CASE("Livox device cache allocates each handle once under concurrent discovery and reads") {
  livox_ros::CacheIndex cache;
  std::array<std::atomic<unsigned>, 16> indices;
  for (auto& index : indices) index.store(255);
  std::atomic<bool> failed{false};
  auto discover = [&] {
    for (unsigned turn = 0; turn < 1000; ++turn) {
      for (unsigned handle = 0; handle < indices.size(); ++handle) {
        uint8_t index = 255, readback = 255;
        if (cache.GetFreeIndex(livox_ros::kLivoxLidarType, handle + 1, index) != 0 ||
            cache.GetIndex(livox_ros::kLivoxLidarType, handle + 1, readback) != 0 ||
            readback != index) {
          failed.store(true); return;
        }
        unsigned unset = 255;
        if (!indices[handle].compare_exchange_strong(unset, index) && unset != index) {
          failed.store(true); return;
        }
      }
    }
  };
  std::thread first(discover), second(discover);
  first.join(); second.join();
  CHECK_FALSE((failed.load()));
  std::array<bool, livox_ros::kMaxSourceLidar> used{};
  for (auto& stored : indices) {
    const unsigned index = stored.load();
    REQUIRE((index < used.size()));
    CHECK_FALSE((used[index]));
    used[index] = true;
  }
  livox_ros::LidarDevice device;
  device.lidar_type = livox_ros::kLivoxLidarType;
  device.handle = 1;
  cache.ResetIndex(&device);
  uint8_t recycled = 255;
  REQUIRE((cache.LvxGetIndex(livox_ros::kLivoxLidarType, 1, recycled) == 0));
  CHECK((recycled == indices[0].load()));
}

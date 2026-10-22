#include "aos/ipc_lib/memory_mapped_queue.h"

#if defined(__APPLE__)
#include <unistd.h>
#endif

#include <chrono>
#include <filesystem>
#include <string>

#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/tmpdir.h"

namespace aos::ipc_lib::testing {
namespace {

constexpr std::filesystem::perms kPermissions =
    static_cast<std::filesystem::perms>(0770);

// Runs with each kind of --shm_base the backend treats differently.
class MemoryMappedQueueTest : public ::testing::TestWithParam<std::string> {
 protected:
  MemoryMappedQueueTest()
      : config_(configuration::MergeConfiguration(
            FlatbufferDetachedBuffer<Configuration>(
                JsonToFlatbuffer<Configuration>(R"({
  "channels": [
    { "name": "/test", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 },
    { "name": "/other", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 }
  ]
})")))),
        shm_base_(GetParam()) {
    UnlinkQueues(shm_base_, &config_.message());
  }

  ~MemoryMappedQueueTest() override {
    UnlinkQueues(shm_base_, &config_.message());
  }

  const Channel *channel(const char *name) const {
    return configuration::GetChannel(&config_.message(), name,
                                     "aos.TestMessage", "", nullptr);
  }

  // Puts one message in the queue behind `channel`.
  void SendOn(const Channel *channel) {
    MemoryMappedQueue queue(shm_base_, kPermissions, &config_.message(),
                            channel);
    std::optional<LocklessQueueSender> sender =
        LocklessQueueSender::Make(queue.queue(), std::chrono::seconds(2));
    ASSERT_TRUE(sender.has_value());
    const char data[] = "hello";
    monotonic_clock::time_point monotonic_sent_time;
    realtime_clock::time_point realtime_sent_time;
    uint32_t queue_index;
    ASSERT_EQ(sender->Send(data, sizeof(data), monotonic_clock::min_time,
                           realtime_clock::min_time, monotonic_clock::min_time,
                           0xffffffffu, UUID::Zero(), &monotonic_sent_time,
                           &realtime_sent_time, &queue_index),
              LocklessQueueSender::Result::GOOD);
  }

  // Whether the queue behind `channel` holds anything.
  bool HasMessage(const Channel *channel) {
    MemoryMappedQueue queue(shm_base_, kPermissions, &config_.message(),
                            channel);
    return LocklessQueueReader(queue.queue()).LatestIndex().valid();
  }

  const FlatbufferDetachedBuffer<Configuration> config_;
  const std::string shm_base_;
};

// A message outlives the queue that sent it.
TEST_P(MemoryMappedQueueTest, MessagesPersist) {
  SendOn(channel("/test"));
  EXPECT_TRUE(HasMessage(channel("/test")));
  EXPECT_FALSE(HasMessage(channel("/other")));
}

// UnlinkQueues() empties the queues of the channels in the config it's given,
// and leaves the rest alone.
TEST_P(MemoryMappedQueueTest, UnlinkQueues) {
  SendOn(channel("/test"));
  SendOn(channel("/other"));

  const FlatbufferDetachedBuffer<Configuration> test_only =
      configuration::MergeConfiguration(FlatbufferDetachedBuffer<Configuration>(
          JsonToFlatbuffer<Configuration>(R"({
  "channels": [
    { "name": "/test", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 }
  ]
})")));
  UnlinkQueues(shm_base_, &test_only.message());

  EXPECT_FALSE(HasMessage(channel("/test")));
  EXPECT_TRUE(HasMessage(channel("/other")));
}

std::vector<std::string> ShmBases() {
  std::vector<std::string> result = {aos::testing::TestTmpDir() +
                                     "/memory_mapped_queue_test"};
#if defined(__APPLE__)
  // macOS keeps these in the POSIX shared memory namespace instead of in
  // files.  Everywhere else they're files like the one above, and tests
  // shouldn't write to the real /dev/shm.
  result.push_back(
      absl::StrCat("/dev/shm/memory_mapped_queue_test_", getpid()));
#endif
  return result;
}

INSTANTIATE_TEST_SUITE_P(ShmBases, MemoryMappedQueueTest,
                         ::testing::ValuesIn(ShmBases()));

}  // namespace
}  // namespace aos::ipc_lib::testing

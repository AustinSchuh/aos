#include "aos/ipc_lib/process_local_queue.h"

#include <chrono>
#include <string>

#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/json_to_flatbuffer.h"

namespace aos::ipc_lib::testing {
namespace {

class ProcessLocalQueueTest : public ::testing::Test {
 protected:
  ProcessLocalQueueTest()
      : config_(configuration::MergeConfiguration(
            FlatbufferDetachedBuffer<Configuration>(
                JsonToFlatbuffer<Configuration>(R"({
  "channels": [
    { "name": "/test", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 },
    { "name": "/other", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 }
  ]
})")))) {
    // Every test here starts from no queues, the way a shared-memory test
    // starts from an empty --shm_base.
    ProcessLocalQueue::ResetAll();
  }

  const Channel *channel(const char *name) const {
    return configuration::GetChannel(&config_.message(), name,
                                     "aos.TestMessage", "", nullptr);
  }

  // Puts one message in the queue behind `channel`.
  void SendOn(const Channel *channel) {
    ProcessLocalQueue queue(&config_.message(), channel);
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
    ProcessLocalQueue queue(&config_.message(), channel);
    return LocklessQueueReader(queue.queue()).LatestIndex().valid();
  }

  FlatbufferDetachedBuffer<Configuration> config_;
};

using ProcessLocalQueueDeathTest = ProcessLocalQueueTest;

// Every ProcessLocalQueue for a channel is the same memory, and different
// channels are different memory.
TEST_F(ProcessLocalQueueTest, SameChannelSameMemory) {
  ProcessLocalQueue first(&config_.message(), channel("/test"));
  ProcessLocalQueue second(&config_.message(), channel("/test"));
  ProcessLocalQueue other(&config_.message(), channel("/other"));

  EXPECT_EQ(first.memory(), second.memory());
  EXPECT_EQ(first.const_memory(), second.const_memory());
  EXPECT_NE(first.memory(), other.memory());

  // The two views are of the same pages.
  EXPECT_NE(static_cast<const void *>(first.memory()),
            static_cast<const void *>(first.const_memory()));
  EXPECT_EQ(first.GetMutableSharedMemory().size(),
            first.GetConstSharedMemory().size());
  EXPECT_EQ(first.GetMutableSharedMemory().size(),
            LocklessQueueMemorySize(first.config()));
}

// The queue outlives its users: what one sends, a later one finds.
TEST_F(ProcessLocalQueueTest, MessageOutlivesUsers) {
  EXPECT_FALSE(HasMessage(channel("/test")));
  SendOn(channel("/test"));
  EXPECT_TRUE(HasMessage(channel("/test")));
  EXPECT_FALSE(HasMessage(channel("/other")));
}

// ResetAll() is the only way to empty a queue, and it empties all
// of them.
TEST_F(ProcessLocalQueueTest, ResetEmptiesEverything) {
  SendOn(channel("/test"));
  SendOn(channel("/other"));
  ProcessLocalQueue::ResetAll();
  EXPECT_FALSE(HasMessage(channel("/test")));
  EXPECT_FALSE(HasMessage(channel("/other")));
}

// The read-only view really is read-only.
TEST_F(ProcessLocalQueueDeathTest, ReadOnlyViewFaultsOnWrite) {
  ProcessLocalQueue queue(&config_.message(), channel("/test"));
  char *const readonly =
      const_cast<char *>(queue.GetConstSharedMemory().data());
  EXPECT_DEATH({ readonly[0] = 'X'; }, "");
}

// A queue cannot be reset out from under a live user.
TEST_F(ProcessLocalQueueDeathTest, ResetWithLiveUserDies) {
  ProcessLocalQueue queue(&config_.message(), channel("/test"));
  EXPECT_DEATH(ProcessLocalQueue::ResetAll(),
               "live ProcessLocalQueue on /test aos.TestMessage");
}

// The same channel with a different queue configuration is a mistake, the
// way a shared-memory file of the wrong size is.
TEST_F(ProcessLocalQueueDeathTest, ChangedConfigurationDies) {
  ProcessLocalQueue queue(&config_.message(), channel("/test"));

  const FlatbufferDetachedBuffer<Configuration> bigger =
      configuration::MergeConfiguration(FlatbufferDetachedBuffer<Configuration>(
          JsonToFlatbuffer<Configuration>(R"({
  "channels": [
    { "name": "/test", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 1024 }
  ]
})")));
  const Channel *const bigger_channel = configuration::GetChannel(
      &bigger.message(), "/test", "aos.TestMessage", "", nullptr);
  EXPECT_DEATH(ProcessLocalQueue(&bigger.message(), bigger_channel),
               "already exists in this process with a different "
               "configuration");
}

}  // namespace
}  // namespace aos::ipc_lib::testing

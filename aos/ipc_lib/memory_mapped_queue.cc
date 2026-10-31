#include "aos/ipc_lib/memory_mapped_queue.h"

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/string.h"

#include "aos/ipc_lib/index.h"

ABSL_FLAG(std::string, shm_hugepage_base, "",
          "If non-empty, a directory on a hugetlbfs mount.  Queues whose "
          "memory size is >= --shm_hugepage_threshold are created there "
          "(backed by huge pages) and reached from --shm_base through a "
          "symlink.  Only matters for the process that creates a queue file; "
          "every other process follows the symlink and adapts.");
ABSL_FLAG(size_t, shm_hugepage_threshold, 500 * 1024 * 1024,
          "Queue memory size (bytes) at or above which a newly created queue "
          "is backed by --shm_hugepage_base.");

namespace aos::ipc_lib {

// Process-global cache of MemoryMappedQueue instances keyed by SHM path, so
// every sender, fetcher and watcher on a channel in this process shares one
// pair of mappings.  Entries are weak_ptrs: a mapping is released as soon as
// its last user drops it and re-created on the next request.
class MemoryMappedQueueCache {
 public:
  // Only ever reached from MemoryMappedQueue construction (never from
  // destruction), so an ordinary function-local static is fine.
  static MemoryMappedQueueCache &GetInstance() {
    static MemoryMappedQueueCache cache;
    return cache;
  }

  std::shared_ptr<MemoryMappedQueue> GetOrCreate(
      std::string_view shm_base, std::filesystem::perms permissions,
      const Configuration *config, const Channel *channel) {
    std::string path = ShmPath(shm_base, channel);
    std::lock_guard<std::mutex> lock(mutex_);
    auto &entry = cache_[path];
    auto existing = entry.lock();
    if (existing) {
      // Every user shares this instance's config, so a second configuration
      // with a different queue layout would otherwise silently get this one.
      ABSL_CHECK(existing->config() == MakeQueueConfiguration(config, channel))
          << ": " << path
          << " is already mapped in this process with a different queue "
             "configuration";
      return existing;
    }
    auto queue = std::make_shared<MemoryMappedQueue>(shm_base, permissions,
                                                     config, channel);
    entry = queue;
    return queue;
  }

 private:
  MemoryMappedQueueCache() = default;

  // Guards cache_.
  std::mutex mutex_;
  std::map<std::string, std::weak_ptr<MemoryMappedQueue>> cache_;
};

std::shared_ptr<MemoryMappedQueue> GetOrCreateMemoryMappedQueue(
    std::string_view shm_base, std::filesystem::perms permissions,
    const Configuration *config, const Channel *channel) {
  return MemoryMappedQueueCache::GetInstance().GetOrCreate(
      shm_base, permissions, config, channel);
}

std::string ShmFolder(std::string_view shm_base, const Channel *channel) {
  ABSL_CHECK(channel->has_name());
  ABSL_CHECK_EQ(channel->name()->string_view()[0], '/');
  return absl::StrCat(shm_base, channel->name()->string_view(), "/");
}

std::string ShmPath(std::string_view shm_base, const Channel *channel) {
  ABSL_CHECK(channel->has_type());
  return ShmFolder(shm_base, channel) + channel->type()->str() + ".v8";
}

LocklessQueueConfiguration MakeQueueConfiguration(
    const Configuration *configuration, const Channel *channel) {
  LocklessQueueConfiguration config;

  config.num_watchers = channel->num_watchers();
  config.num_senders = channel->num_senders();
  // The value in the channel will default to 0 if readers are configured to
  // copy.
  config.num_pinners = channel->num_readers();
  config.queue_size = configuration::QueueSize(configuration, channel);
  ABSL_CHECK_LT(config.queue_size,
                std::numeric_limits<QueueIndex::PackedIndexType>::max())
      << ": More messages/second configured than the queue can hold on "
      << configuration::CleanedChannelToString(channel) << ", "
      << channel->frequency() << "hz for "
      << std::chrono::duration<double>(
             configuration::ChannelStorageDuration(configuration, channel))
             .count()
      << "sec";
  config.message_data_size = channel->max_size();

  return config;
}

size_t HugePagesNeededForNode(const Configuration *config, const Node *node,
                              size_t threshold, size_t huge_page_size) {
  size_t pages = 0;
  if (!config->has_channels()) {
    return pages;
  }
  for (const aos::Channel *channel : *config->channels()) {
    if (!configuration::ChannelIsReadableOnNode(channel, node)) {
      continue;
    }
    const size_t size =
        LocklessQueueMemorySize(MakeQueueConfiguration(config, channel));
    if (size < threshold) {
      continue;
    }
    pages += (size + huge_page_size - 1) / huge_page_size;
  }
  return pages;
}

namespace {

// Returns the path on the hugetlbfs mount at which the queue file for channel
// should be created, or an empty string when the queue should live directly
// at the tmpfs path: large queues with a configured --shm_hugepage_base are
// created on the hugetlbfs mount with a symlink left at the tmpfs path (see
// WritableShmMapping); opening follows the symlink transparently either way.
std::string HugePagePath(const Channel *channel, size_t size) {
  const std::string hugepage_base = absl::GetFlag(FLAGS_shm_hugepage_base);
  if (hugepage_base.empty() ||
      size < absl::GetFlag(FLAGS_shm_hugepage_threshold)) {
    return "";
  }
  return ShmPath(hugepage_base, channel);
}

}  // namespace

MemoryMappedQueue::MemoryMappedQueue(std::string_view shm_base,
                                     std::filesystem::perms permissions,
                                     const Configuration *config,
                                     const Channel *channel)
    : config_(MakeQueueConfiguration(config, channel)),
      writable_mapping_(
          ShmPath(shm_base, channel), ipc_lib::LocklessQueueMemorySize(config_),
          permissions,
          HugePagePath(channel, ipc_lib::LocklessQueueMemorySize(config_))),
      readonly_mapping_(ShmPath(shm_base, channel),
                        ipc_lib::LocklessQueueMemorySize(config_),
                        permissions) {
  ipc_lib::InitializeLocklessQueueMemory(memory(), config_);
}

MemoryMappedQueue::~MemoryMappedQueue() {}

}  // namespace aos::ipc_lib

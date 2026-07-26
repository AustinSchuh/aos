#include "aos/events/aio_unix.h"

#include <pthread.h>

#include <atomic>
#include <mutex>

namespace aos::internal {
namespace {

// Private to this file: reached through the accessors below so no backend can
// take a reference to one and forget which memory order it wanted.
std::atomic<size_t> fork_count{0};
std::atomic<size_t> parent_fork_count{0};
std::once_flag fork_counters_once;

}  // namespace

size_t ForkCount() { return fork_count.load(std::memory_order_relaxed); }

size_t ParentForkCount() {
  return parent_fork_count.load(std::memory_order_relaxed);
}

void RegisterForkCounters() {
  std::call_once(fork_counters_once, []() {
    pthread_atfork(
        // No prepare handler: nothing here needs quiescing before the fork,
        // only fixing up after it.
        nullptr,
        // Parent: runs in the forking process once fork() returns.
        []() { parent_fork_count.fetch_add(1, std::memory_order_relaxed); },
        // Child: runs in the new process once fork() returns.
        []() { fork_count.fetch_add(1, std::memory_order_relaxed); });
  });
}

}  // namespace aos::internal

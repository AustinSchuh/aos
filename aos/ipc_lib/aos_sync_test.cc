// Cross-platform tests for aos_sync.  The OS-specific halves live in their
// own files and are conditionally compiled in by the build file
// (aos_sync_windows_test.cc today); this file holds what is true everywhere.

#include "aos/ipc_lib/aos_sync.h"

#if !defined(_WIN32)
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

namespace aos::testing {

// The futex word is the ABI everything else builds on: exactly the kernel's
// 32-bit futex on Linux, and the same layout driven by the backends
// elsewhere.
static_assert(sizeof(aos_futex) == sizeof(uint32_t));
static_assert(alignof(aos_futex) >= alignof(uint32_t));
static_assert(sizeof(aos_condition) == sizeof(uint32_t));

// The TID mask and flag bits must tile the word exactly (kernel-mandated
// values on Linux; everything decodes futex snapshots with them).
static_assert((FUTEX_TID_MASK | FUTEX_WAITERS | FUTEX_OWNER_DIED) ==
              0xffffffff);
static_assert((FUTEX_TID_MASK & (FUTEX_WAITERS | FUTEX_OWNER_DIED)) == 0);

// Basic set/unset semantics of the futex API, no blocking involved.
TEST(AosSyncTest, FutexSetUnset) {
  aos_futex futex{};
  EXPECT_EQ(1, futex_unset(&futex));  // Wasn't set.
  EXPECT_GE(futex_set(&futex), 0);
  EXPECT_EQ(0, futex_unset(&futex));  // Was set.
  EXPECT_EQ(1, futex_unset(&futex));
}

#if !defined(_WIN32)
namespace {

// Waits for futex to be set, for up to 10 seconds.  Returns how long that
// took.
std::chrono::steady_clock::duration WaitForSet(aos_futex *futex) {
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  struct timespec timeout = {.tv_sec = 1, .tv_nsec = 0};
  while (std::atomic_ref<uint32_t>(*futex).load() == 0 &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    EXPECT_GE(futex_wait_timeout(futex, &timeout), 0);
  }
  return std::chrono::steady_clock::now() - start;
}

}  // namespace

// A thread waiting on a futex in private memory is woken by futex_set() after
// the process forks.  The fork means the page the futex is on gets copied the
// next time the parent writes to it.
TEST(AosSyncTest, WakePrivateFutexAfterFork) {
  aos_futex futex{};
  std::chrono::steady_clock::duration waited;
  std::thread waiter([&futex, &waited]() { waited = WaitForSet(&futex); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Keep the child around until after the wake, so the page is still shared
  // with it when the parent writes to it.
  int pipe_fds[2];
  ASSERT_EQ(0, pipe(pipe_fds));
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    char c;
    close(pipe_fds[1]);
    (void)!read(pipe_fds[0], &c, 1);
    _exit(0);
  }
  close(pipe_fds[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_GE(futex_set(&futex), 0);
  waiter.join();
  close(pipe_fds[1]);
  ASSERT_EQ(child, waitpid(child, nullptr, 0));

  // A missed wake would be found by the 1 second timeout instead.
  EXPECT_LT(waited, std::chrono::milliseconds(900));
}

// A process waiting on a futex in shared memory is woken by another one
// setting it.
TEST(AosSyncTest, WakeSharedFutexFromAnotherProcess) {
  void *const memory = mmap(nullptr, sizeof(aos_futex), PROT_READ | PROT_WRITE,
                            MAP_SHARED | MAP_ANON, -1, 0);
  ASSERT_NE(MAP_FAILED, memory);
  aos_futex *const futex = new (memory) aos_futex{};

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    _exit(WaitForSet(futex) < std::chrono::milliseconds(900) ? 0 : 1);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_GE(futex_set(futex), 0);

  int status;
  ASSERT_EQ(child, waitpid(child, &status, 0));
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(0, WEXITSTATUS(status)) << ": The child wasn't woken.";
  ASSERT_EQ(0, munmap(memory, sizeof(aos_futex)));
}
#endif

}  // namespace aos::testing

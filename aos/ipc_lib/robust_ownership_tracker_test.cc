#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <errno.h>
#include <stdlib.h>

#include <atomic>
#include <cstdint>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/testing/test_child.h"
#include "aos/testing/test_shm.h"

namespace aos::ipc_lib::testing {
namespace {

// NoMatchingPID stores this into the futex's TID field and expects
// OwnerIsDefinitelyAbsolutelyDead() to report the owner gone.  It must be a PID
// that can't name a live process, so the platform's lookup of that TID fails --
// and each platform looks the TID up differently.
#if defined(__linux__)
uint32_t NonexistentPid() {
  // PID_MAX_LIMIT comes from the kernel's include/linux/threads.h, which isn't
  // exported to userspace, so we spell it out here.
  return 1 << 22;
}
#elif defined(__APPLE__)
// Darwin caps PIDs at PID_MAX, so PID_MAX + 1 can't name a live process.
// PID_MAX lives in the kernel-only header bsd/sys/proc_internal.h and isn't in
// the userspace SDK, so define it to its known value if it isn't already.  (The
// darwin check actually returns on a "futex TID != recorded owner thread id"
// mismatch before it ever does the liveness lookup, so any value other than our
// owner works here; PID_MAX + 1 just keeps the intent clear.)
#ifndef PID_MAX
#define PID_MAX 99999
#endif
uint32_t NonexistentPid() { return PID_MAX + 1; }
#else  // Windows
uint32_t NonexistentPid() {
  // Windows looks the PID up via the process API.  999999 is never a valid
  // Windows PID (process ids are always multiples of 4), so the lookup fails.
  return 999999;
}
#endif

}  // namespace

// Capture RobustOwnershipTracker in shared memory so it is shared across a
// fork (on Linux) or simply allocated/shared (on Windows).
//
// The tests never release ownership: the mutex lives in the block, and the
// block is simply freed when this goes out of scope (munmap on Linux, which
// makes the kernel's robust-list walk fault harmlessly; nothing walks the
// userspace robust list on Windows).  This matches how a lockless-queue slot is
// abandoned when its owner dies.
class SharedRobustOwnershipTracker {
 public:
  SharedRobustOwnershipTracker() : block_(sizeof(RobustOwnershipTracker)) {
    tracker_ = new (block_.get()) RobustOwnershipTracker();
  }

  RobustOwnershipTracker &tracker() const { return *tracker_; }

 private:
  aos::testing::SharedMemoryBlock block_;
  RobustOwnershipTracker *tracker_;
};

class RobustOwnershipTrackerTest : public ::testing::Test {
 public:
  // Runs a function in a child (a forked process, or a thread on platforms
  // without fork) and waits for it to finish before resuming.  The tests use
  // this to make a *different* owner claim the tracker and then die: process
  // death on POSIX, thread death on Windows -- each being the granularity at
  // which that platform detects owner death.
  template <typename T>
  void RunInChildAndBlockUntilComplete(T fn) {
    aos::testing::TestChild child;
    child.Start(std::move(fn));
    child.Join();
  }

  // Returns the robust mutex.
  aos_mutex &GetMutex(RobustOwnershipTracker &tracker) {
    return tracker.mutex_;
  }

#ifndef __APPLE__
  // Returns the current start time in ticks.
  uint64_t GetStartTimeTicks(RobustOwnershipTracker &tracker) {
    return tracker.start_time_ticks_.load();
  }

  // Sets the current start time in ticks.
  void SetStartTimeTicks(RobustOwnershipTracker &tracker, uint64_t start_time) {
    tracker.start_time_ticks_ = start_time;
  }
#endif
};

// Tests that acquiring the futex doesn't erroneously report the owner (i.e.
// "us") as dead.
TEST_F(RobustOwnershipTrackerTest, AcquireWorks) {
  SharedRobustOwnershipTracker shared_tracker;

  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // Run acquire in this process, and expect it should not be dead until
  // after the test finishes.
  shared_tracker.tracker().Acquire();

  // We have ownership. Since we are alive, the owner should not be marked as
  // dead. We can use relaxed ordering since we are the only ones touching the
  // data here.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

// Tests that child death without unlocking results in the futex being marked as
// dead, and the owner being very dead.
TEST_F(RobustOwnershipTrackerTest, FutexRecovers) {
  SharedRobustOwnershipTracker shared_tracker;

  RunInChildAndBlockUntilComplete(
      [&]() { shared_tracker.tracker().Acquire(); });

  // Since the child that took ownership died, we expect that death to be
  // reported.
  EXPECT_TRUE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

// Tests that a PID which doesn't exist results in the process being noticed as
// dead when we inspect /proc.
TEST_F(RobustOwnershipTrackerTest, NoMatchingPID) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
  std::atomic_ref<uint32_t>(GetMutex(shared_tracker.tracker()).futex)
      .store(NonexistentPid(), std::memory_order_relaxed);

  // Since we're only pretending that the owner died (by changing the TID in the
  // futex), we only notice that the owner is dead when spending the time
  // walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

#ifndef __APPLE__
// Tests that a mismatched start time results in the process being marked as
// dead.
TEST_F(RobustOwnershipTrackerTest, NoMatchingStartTime) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  EXPECT_NE(GetStartTimeTicks(shared_tracker.tracker()), 0);
  EXPECT_NE(GetStartTimeTicks(shared_tracker.tracker()),
            RobustOwnershipTracker::kNoStartTimeTicks);
  SetStartTimeTicks(shared_tracker.tracker(), 1);

  // Since we're only pretending that the owner died (by changing the tracked
  // start time ticks in the tracker), we only notice that the owner is dead
  // when spending the time walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}
#endif

}  // namespace aos::ipc_lib::testing

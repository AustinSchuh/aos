#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <assert.h>
#include <process.h>
#include <windows.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {

namespace {

uint64_t ReadStartTimeTicks(pid_t tid) {
  if (tid == 0) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  HANDLE hThread = OpenThread(THREAD_QUERY_LIMITED_INFORMATION, FALSE,
                              static_cast<DWORD>(tid));
  if (hThread == NULL) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  FILETIME creation_time, exit_time, kernel_time, user_time;
  if (!GetThreadTimes(hThread, &creation_time, &exit_time, &kernel_time,
                      &user_time)) {
    CloseHandle(hThread);
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  CloseHandle(hThread);
  ULARGE_INTEGER li;
  li.LowPart = creation_time.dwLowDateTime;
  li.HighPart = creation_time.dwHighDateTime;
  return li.QuadPart;
}

}  // namespace

bool RobustOwnershipTracker::OwnerIsDefinitelyAbsolutelyDead() const {
  auto loaded = LoadAcquire();
  if (loaded.OwnerIsDead()) {
    return true;
  }
  if (loaded.IsUnclaimed()) {
    return false;
  }
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(loaded.tid());
  if (proc_start_time_ticks == kNoStartTimeTicks) {
    ABSL_LOG(ERROR) << "Detected that PID " << loaded.tid() << " died.";
    return true;
  }

  if (proc_start_time_ticks != start_time_ticks_) {
    ABSL_LOG(ERROR) << "Detected that PID " << loaded.tid()
                    << " died from a starttime missmatch.";
    return true;
  }
  return false;
}

void RobustOwnershipTracker::Acquire() {
  // There is a subtle ordering of operations here: the metadata
  // (start_time_ticks_) must be fully written and visible BEFORE the futex is
  // claimed by death_notification_init. Otherwise, another process inspecting
  // the tracker concurrently could see that the futex is claimed but find the
  // metadata is still unset (or contains stale values), incorrectly concluding
  // that the owner is dead.
  //
  // Note that if two processes concurrently attempt to call Acquire() on the
  // same tracker, they could overwrite each other's metadata before either
  // claims the futex, leading to state corruption.  Callers must serialize
  // calls to Acquire() (e.g., using a higher-level lock, as is done in the
  // lockless queue implementation via the queue setup lock).
  pid_t tid = static_cast<pid_t>(GetCurrentThreadId());
  assert(tid > 0);
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(tid);
  ABSL_CHECK_NE(proc_start_time_ticks, kNoStartTimeTicks);

  start_time_ticks_ = proc_start_time_ticks;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib

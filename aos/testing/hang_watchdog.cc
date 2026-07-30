#include "aos/testing/hang_watchdog.h"

#ifdef __linux__

#include <dirent.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "absl/debugging/stacktrace.h"
#include "absl/debugging/symbolize.h"

namespace aos::testing {
namespace {

// Set by the dumping thread before each tgkill(), read by the handler.
// The handler acknowledges by flipping dump_done_ so the threads' output
// doesn't interleave.
std::atomic<bool> dump_done_{false};

// Everything in the handler sticks to write(2) plus absl's
// signal-safe-in-practice stack machinery, mirroring
// absl::InstallFailureSignalHandler()'s approach.  Buffered stdio is
// deliberately avoided.
void RawWrite(const char *buf) {
  (void)!write(STDERR_FILENO, buf, strlen(buf));
}

void DumpThisThread(int /*signo*/) {
  char line[256];
  void *stack[64];
  const int depth = absl::GetStackTrace(stack, 64, /*skip_count=*/1);
  snprintf(line, sizeof(line), "--- thread %ld (%d frames) ---\n",
           static_cast<long>(syscall(SYS_gettid)), depth);
  RawWrite(line);
  for (int i = 0; i < depth; ++i) {
    char symbol[192];
    if (!absl::Symbolize(stack[i], symbol, sizeof(symbol))) {
      snprintf(symbol, sizeof(symbol), "(unknown)");
    }
    snprintf(line, sizeof(line), "  @ %p  %s\n", stack[i], symbol);
    RawWrite(line);
  }
  dump_done_.store(true, std::memory_order_release);
}

void DumpAllThreadsAndAbort() {
  char line[256];
  snprintf(line, sizeof(line),
           "\n*** HANG WATCHDOG: test still running as bazel's TEST_TIMEOUT "
           "approaches; dumping all threads then aborting. ***\n");
  RawWrite(line);

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = DumpThisThread;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGUSR2, &sa, nullptr);

  const pid_t self = static_cast<pid_t>(syscall(SYS_gettid));
  DIR *dir = opendir("/proc/self/task");
  if (dir != nullptr) {
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
      const long tid = atol(entry->d_name);
      if (tid <= 0 || tid == self) continue;
      dump_done_.store(false, std::memory_order_release);
      if (syscall(SYS_tgkill, getpid(), tid, SIGUSR2) != 0) continue;
      // Give the target thread a bounded window to finish printing before
      // signaling the next one, so the dumps don't interleave.  A thread
      // stuck in an uninterruptible state just forfeits its dump.
      for (int spin = 0;
           spin < 100 && !dump_done_.load(std::memory_order_acquire); ++spin) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
    closedir(dir);
  }

  // This thread's own stack last (it's just the watchdog, but completeness
  // is cheap), then die loudly enough that nobody mistakes this for a pass.
  DumpThisThread(0);
  RawWrite("*** HANG WATCHDOG: dump complete, aborting. ***\n");
  abort();
}

}  // namespace

void MaybeStartHangWatchdog() {
  const char *timeout_env = getenv("TEST_TIMEOUT");
  if (timeout_env == nullptr) return;
  const long timeout_sec = atol(timeout_env);
  if (timeout_sec <= 0) return;

  // Fire far enough ahead of the external kill that the dump lands in the
  // log, but never so early that a legitimately slow test trips it.
  const long margin = timeout_sec > 60 ? 15 : timeout_sec / 4;
  const long fire_after = timeout_sec - margin;

  // The watchdog thread must not be eligible to receive process-directed
  // signals.  It starts before any test code runs, so it would otherwise be
  // the one thread with everything unblocked after a test's signalfd-based
  // listener masks its signals on the main thread -- and the kernel hands a
  // process-directed signal to any thread that doesn't block it.  For a
  // signal whose default disposition is ignore (SIGCHLD), that DISCARDS the
  // signal before the signalfd can observe it: found live as starterd
  // silently never reaping its children in starter_test, leaving zombies
  // and a stuck restart state machine, at ~50% per run on loaded CI.
  // Block everything here (inherited by the thread), then restore.
  sigset_t all, old;
  sigfillset(&all);
  if (pthread_sigmask(SIG_BLOCK, &all, &old) != 0) abort();
  std::thread([fire_after]() {
    std::this_thread::sleep_for(std::chrono::seconds(fire_after));
    // If the tests finished, the process exited and this never runs.
    DumpAllThreadsAndAbort();
  }).detach();
  if (pthread_sigmask(SIG_SETMASK, &old, nullptr) != 0) abort();
}

}  // namespace aos::testing

#else  // !__linux__

namespace aos::testing {
void MaybeStartHangWatchdog() {}
}  // namespace aos::testing

#endif  // __linux__

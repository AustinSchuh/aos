#include "aos/realtime.h"

#ifndef _WIN32
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>

#include "absl/base/internal/raw_logging.h"
#include "absl/debugging/failure_signal_handler.h"
#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/init.h"
#include "aos/sanitizers.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);

namespace aos::testing {

// Tests that ScopedRealtime handles the simple case.
TEST(RealtimeTest, ScopedRealtime) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that ScopedRealtime handles nesting.
TEST(RealtimeTest, DoubleScopedRealtime) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
    {
      ScopedRealtime rt2;
      CheckRealtime();
    }
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that ScopedRealtime handles nesting with ScopedNotRealtime.
TEST(RealtimeTest, ScopedNotRealtime) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
    {
      ScopedNotRealtime nrt;
      CheckNotRealtime();
    }
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that ScopedRealtimeRestorer works both when starting RT and nonrt.
TEST(RealtimeTest, ScopedRealtimeRestorer) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
    {
      ScopedRealtimeRestorer restore;
      CheckRealtime();

      MarkRealtime(false);
      CheckNotRealtime();
    }
    CheckRealtime();
  }
  CheckNotRealtime();

  {
    ScopedRealtimeRestorer restore;
    CheckNotRealtime();

    MarkRealtime(true);
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that getters and setters properly interact with thread realtime
// priority.
TEST(RealtimeTest, GetSetRealtimePriority) {
  UnsetCurrentThreadRealtimePriority();
  EXPECT_EQ(GetCurrentThreadRealtimePriority(), 0);
  SetCurrentThreadRealtimePriority(30);
  EXPECT_EQ(GetCurrentThreadRealtimePriority(), 30);
  UnsetCurrentThreadRealtimePriority();
}

// Tests that getters and setters properly interact with thread scheduling
// policy.
TEST(RealtimeTest, GetSetSchedulingPolicy) {
  UnsetCurrentThreadRealtimePriority();
  EXPECT_EQ(GetCurrentThreadSchedulingPolicy(), SCHED_OTHER);
  SetCurrentThreadRealtimePriority(1, SCHED_FIFO);
  EXPECT_EQ(GetCurrentThreadSchedulingPolicy(), SCHED_FIFO);
  SetCurrentThreadRealtimePriority(1, SCHED_RR);
  EXPECT_EQ(GetCurrentThreadSchedulingPolicy(), SCHED_RR);
  UnsetCurrentThreadRealtimePriority();
}

#ifdef __linux__
// Tests that the fatal-path unset drops every thread in the process, not just
// the one which is dying.  Sibling threads are about to be torn down with the
// rest of the process, and nothing they do in the meantime is worth preempting
// healthy realtime work elsewhere on the system.
//
// This also covers the /proc/self/task walk, which uses raw open()/getdents64()
// rather than opendir() so that it stays async-signal-safe for the fatal signal
// handler.
TEST(RealtimeTest, FatalUnsetRealtimePriorityDropsEveryThread) {
  std::atomic<pid_t> sibling_tid{0};
  std::atomic<bool> sibling_is_realtime{false};
  std::atomic<bool> done{false};

  std::thread sibling([&]() {
    sibling_tid = GetThreadId();
    sibling_is_realtime =
        SetCurrentThreadRealtimePriorityLowLevel(1, SCHED_FIFO) == 0;
    while (!done.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  while (sibling_tid.load() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  while (!sibling_is_realtime.load() &&
         sched_getscheduler(sibling_tid.load()) != SCHED_FIFO) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // NO_MODE keeps the malloc hooks off, so gtest can keep allocating while we
  // are on the realtime scheduler.
  SetCurrentThreadRealtimePriority(1, SCHED_FIFO, RealtimePolicy::NO_MODE);
  ASSERT_EQ(sched_getscheduler(0), SCHED_FIFO);
  ASSERT_EQ(sched_getscheduler(sibling_tid.load()), SCHED_FIFO);

  aos_FatalUnsetRealtimePriority();

  EXPECT_EQ(sched_getscheduler(0), SCHED_OTHER);
  EXPECT_EQ(sched_getscheduler(sibling_tid.load()), SCHED_OTHER);

  done = true;
  sibling.join();
}
#endif  // __linux__

// Malloc hooks don't work with asan/msan.
#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)

// Tests that CHECK statements give real error messages rather than die on
// malloc.
TEST(RealtimeDeathTest, Check) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        CHECK_EQ(1, 2) << ": Numbers aren't equal.";
      },
      "Numbers aren't equal");
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        CHECK_GT(1, 2) << ": Cute error message";
      },
      "Cute error message");
}

// Tests that CHECK statements give real error messages rather than die on
// malloc.
TEST(RealtimeDeathTest, Fatal) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        LOG(FATAL) << "Cute message here";
      },
      "Cute message here");
}

TEST(RealtimeDeathTest, Malloc) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = reinterpret_cast<volatile int *>(malloc(sizeof(int)));
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, Realloc) {
  EXPECT_DEATH(
      {
        void *a = malloc(sizeof(int));
        ScopedRealtime rt;
        volatile int *b =
            reinterpret_cast<volatile int *>(realloc(a, sizeof(int) * 2));
        *b = 5;
        EXPECT_EQ(*b, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, Calloc) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a =
            reinterpret_cast<volatile int *>(calloc(1, sizeof(int)));
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, New) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = new int;
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, NewArray) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = new int[3];
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

#ifndef _WIN32
// Tests that the signal handler drops RT permission and prints out a real
// backtrace instead of crashing on the resulting mallocs.
TEST(RealtimeDeathTest, SignalHandler) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        int x = reinterpret_cast<const volatile int *>(0)[0];
        LOG(INFO) << x;
      },
      "\\*\\*\\* SIGSEGV received at .*");
}
#endif

// Tests that ABSL_RAW_LOG(FATAL) explodes properly.
TEST(RealtimeDeathTest, RawFatal) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        ABSL_RAW_LOG(FATAL, "Cute message here\n");
      },
      "Cute message here");
}

#ifndef _WIN32

namespace {

// Async-signal-safe report of the current scheduling policy, so the death
// tests below can observe it from contexts where the process is already dying
// and nothing which allocates is safe to call.
void ReportSchedulingPolicy() {
  char buffer[64];
  const int size =
      snprintf(buffer, sizeof(buffer), "\npolicy while dying: %d\n",
               sched_getscheduler(0));
  if (size > 0) {
    // Nothing useful to do if this fails; we are on our way out.
    [[maybe_unused]] const ssize_t written =
        write(STDERR_FILENO, buffer, static_cast<size_t>(size));
  }
}

void SchedulingPolicyAbortHook(const char * /*file*/, int /*line*/,
                               const char * /*buf_start*/,
                               const char * /*prefix_end*/,
                               const char * /*buf_end*/) {
  ReportSchedulingPolicy();
}

void SchedulingPolicySignalHandler(int /*signo*/) {
  ReportSchedulingPolicy();
  _exit(1);
}

}  // namespace

// Tests that ABSL_RAW_LOG(FATAL) leaves the realtime scheduler before it
// formats and writes anything.  It aborts without going through LogMessage, so
// it needs its own hook to do this.
TEST(RealtimeDeathTest, RawFatalDropsRealtimePriority) {
  EXPECT_DEATH(
      {
        // The abort hook runs after the message is written, which is after the
        // drop we are checking for.
        absl::raw_log_internal::RegisterAbortHook(SchedulingPolicyAbortHook);

        SetCurrentThreadRealtimePriority(1, SCHED_FIFO,
                                         RealtimePolicy::NO_MODE);
        ABSL_RAW_LOG(FATAL, "Dying now\n");
      },
      absl::StrCat("policy while dying: ", SCHED_OTHER));
}

// Tests that a fatal signal drops us off the realtime scheduler before abseil
// symbolizes the backtrace.  That work is unbounded, and doing it at realtime
// priority lets a dying process starve healthy realtime work elsewhere.
TEST(RealtimeDeathTest, SignalHandlerDropsRealtimePriority) {
  EXPECT_DEATH(
      {
        // Install our handler first, then re-install abseil's asking it to
        // chain.  Abseil's handler then runs first (dropping us off RT) and
        // ours observes the result.
        struct sigaction action;
        memset(&action, 0, sizeof(action));
        action.sa_handler = SchedulingPolicySignalHandler;
        PCHECK(sigaction(SIGSEGV, &action, nullptr) == 0);

        absl::FailureSignalHandlerOptions options;
        options.call_previous_handler = true;
        absl::InstallFailureSignalHandler(options);

        SetCurrentThreadRealtimePriority(1, SCHED_FIFO,
                                         RealtimePolicy::NO_MODE);
        int x = reinterpret_cast<const volatile int *>(0)[0];
        LOG(INFO) << x;
      },
      absl::StrCat("policy while dying: ", SCHED_OTHER));
}

#endif  // !_WIN32

#endif  // !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)

#if !defined(__APPLE__) && !defined(_WIN32)
// Tests that we see which CPUs we tried to set when it fails. This can be
// useful for debugging.
TEST(RealtimeDeathTest, SetAffinityErrorMessage) {
  EXPECT_DEATH(
      { SetCurrentThreadAffinity(MakeCpusetFromCpus({1000})); },
      "sched_setaffinity\\(0, sizeof\\(cpu_set_t\\), "
      "cpuset\\.native_handle\\(\\)\\) == 0 "
      "\\{CPUs 1000\\}: Invalid argument");
  EXPECT_DEATH(
      { SetCurrentThreadAffinity(MakeCpusetFromCpus({1000, 1001})); },
      "sched_setaffinity\\(0, sizeof\\(cpu_set_t\\), "
      "cpuset\\.native_handle\\(\\)\\) == 0 "
      "\\{CPUs 1000, 1001\\}: Invalid argument");
}
#endif

// Tests CpuSet functionality.
TEST(CpuSetTest, BasicFunctionality) {
  CpuSet s;
  EXPECT_TRUE(s.Empty());
  for (int i = 0; i < static_cast<int>(CpuSet::kSize); ++i) {
    EXPECT_FALSE(s.IsSet(i));
  }

  s.Set(1);
  EXPECT_FALSE(s.Empty());
  EXPECT_TRUE(s.IsSet(1));
  EXPECT_FALSE(s.IsSet(0));

  s.Set(10);
  EXPECT_TRUE(s.IsSet(1));
  EXPECT_TRUE(s.IsSet(10));
  EXPECT_FALSE(s.IsSet(9));

  s.Clear(1);
  EXPECT_FALSE(s.IsSet(1));
  EXPECT_TRUE(s.IsSet(10));
  EXPECT_FALSE(s.Empty());

  s.Clear();
  EXPECT_TRUE(s.Empty());
  EXPECT_FALSE(s.IsSet(10));
}

TEST(CpuSetTest, Equality) {
  CpuSet s1;
  CpuSet s2;

  EXPECT_EQ(s1, s2);

  s1.Set(1);
  EXPECT_NE(s1, s2);

  s2.Set(1);
  EXPECT_EQ(s1, s2);

  s1.Set(2);
  EXPECT_NE(s1, s2);
}

TEST(CpuSetTest, Stringify) {
  CpuSet s;
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs }");
  s.Set(1);
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs 1}");
  s.Set(3);
  // Iteration order usually 0..N
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs 1, 3}");
}

TEST(CpuSetTest, MakeCpusetFromCpus) {
  CpuSet s = MakeCpusetFromCpus({1, 3});
  EXPECT_TRUE(s.IsSet(1));
  EXPECT_TRUE(s.IsSet(3));
  EXPECT_FALSE(s.IsSet(2));
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs 1, 3}");
}

}  // namespace aos::testing

// We need a special gtest main to force die_on_malloc support on.  Otherwise
// we can't test CHECK statements before turning die_on_malloc on globally.
GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)
  absl::SetFlag(&FLAGS_die_on_malloc, true);
#endif

  aos::InitGoogle(&argc, &argv);

  return RUN_ALL_TESTS();
}

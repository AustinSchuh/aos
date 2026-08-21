#include "aos/events/aio.h"

#include <fcntl.h>
#include <signal.h>
#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif
#if defined(__linux__)
#include <sys/epoll.h>
#else
#define EPOLLIN 0x01
#define EPOLLOUT 0x04
#define EPOLLERR 0x08
#define EPOLLHUP 0x10
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/pipe.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/realtime.h"

ABSL_DECLARE_FLAG(std::string, aio_backend);
// Defined only by the backends with a submission queue (io_uring, kqueue); the
// Windows IOCP backend has no equivalent, so uses of this flag are guarded.
ABSL_DECLARE_FLAG(uint32_t, aio_queue_depth);

namespace aos::testing {

// Builds a FileDescriptor from a raw integer, for tests that intentionally use
// synthetic or invalid descriptors.  On Windows a FileDescriptor is an opaque
// pointer, so the value must be reinterpret_cast rather than implicitly
// converted from an int.
inline FileDescriptor FakeFd(intptr_t value) {
#if defined(_WIN32)
  return reinterpret_cast<FileDescriptor>(value);
#else
  return static_cast<FileDescriptor>(value);
#endif
}

// Scoped watchdog: arms a SIGALRM fuse so a wedged test dies loudly instead
// of hanging until bazel's timeout, and disarms it on scope exit so the fuse
// cannot leak into subsequent tests in this process (alarm(2) keeps exactly
// one pending fuse per process, and nothing else would ever clear it).
// SIGALRM's default disposition -- terminate -- is the whole watchdog; no
// handler is installed on purpose.  Note the fuse lives in the *parent*
// test process even when armed around a death test: alarm() timers are not
// inherited across fork(), so the child never sees it -- the parent's fuse
// covers waiting on a wedged child.  alarm() is POSIX-only; on Windows the
// child re-execs and there is no equivalent, so this is a no-op there.
class ScopedDeathTestWatchdog {
 public:
  ScopedDeathTestWatchdog() {
#ifndef _WIN32
    alarm(30);
#endif
  }
  ~ScopedDeathTestWatchdog() {
#ifndef _WIN32
    alarm(0);
#endif
  }
};

// Self-imposed backpressure for ring-churning loops.  Closing an io_uring fd
// is fire-and-forget: the kernel frees the ring asynchronously on a
// workqueue, and IORING_SETUP_DEFER_TASKRUN rings additionally block that
// work on a full RCU grace period each (io_ring_exit_work).  Teardown
// throughput is therefore capped -- ~2500 rings/s on an idle machine,
// collapsing under CPU load as grace periods stretch -- while creation is
// effectively unbounded.  A loop that creates rings faster than the kernel
// retires them accumulates gigabytes of unreclaimable slab (pinned ctx,
// request, and ring-buffer memory), which is exactly what a heavily parallel
// CI run did to the whole build cluster.  There is no API to wait for a
// specific ring's teardown, but /proc/meminfo's SUnreclaim tracks the
// backlog well: capture a baseline, and whenever growth exceeds a slack
// threshold, sleep until the kernel catches back up.  Backpressure only,
// never an assertion -- the counter is machine-global, so a noisy neighbor
// can only ever make this throttle extra, not pass wrongly.  Linux-only;
// no-op elsewhere.
inline void ThrottleOnKernelRingTeardown() {
#ifdef __linux__
  static const auto read_sunreclaim_kb = []() -> long {
    FILE *f = fopen("/proc/meminfo", "r");
    if (f == nullptr) return -1;
    char line[128];
    long kb = -1;
    while (fgets(line, sizeof(line), f) != nullptr) {
      if (sscanf(line, "SUnreclaim: %ld kB", &kb) == 1) break;
    }
    fclose(f);
    return kb;
  };
  static const long baseline_kb = read_sunreclaim_kb();
  if (baseline_kb < 0) return;
  // Small slack: with per-process baselines, every concurrently-running
  // process effectively grants itself this much headroom, so fleet-wide
  // growth scales with (slack x process count).  (An absolute threshold
  // would avoid that but can't be chosen portably -- idle SUnreclaim
  // varies by gigabytes across machines.)
  constexpr long kSlackKb = 64 * 1024;  // 64 MB over baseline
  // Whole-process throttle budget, not per-checkpoint: on a node where
  // *other* processes keep the backlog elevated, a per-checkpoint deadline
  // multiplies across a loop's checkpoints (observed: 20 x 10s = 200s of
  // stalling inside one test, blowing its 300s timeout).  The budget bounds
  // the total slowdown this helper can ever add to a process; when it's
  // spent, the loop runs unthrottled and the machine-level pressure is the
  // neighbors' to shed.
  static std::chrono::milliseconds budget{15000};
  while (budget > std::chrono::milliseconds(0) &&
         read_sunreclaim_kb() > baseline_kb + kSlackKb) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    budget -= std::chrono::milliseconds(20);
  }
#endif
}

// A repeating timer, built the way consumers have to build one now that
// Aio::Timer is deliberately one-shot (see Aio::Timer::Schedule()):
// re-schedule from the callback against an absolute grid the caller owns.
//
// This is ShmTimerHandler's pattern minus its policy choice -- it skips
// straight to the next *future* deadline when it falls behind, where this
// delivers every elapsed period in turn.  Tests want the latter, since
// "did every period get delivered" is a thing several of them measure.
// Having the policy be visibly the caller's is the point: it is why Aio
// itself does not implement repeating timers.
class RepeatingTimer {
 public:
  RepeatingTimer(Aio *aio, std::function<void(Completion)> callback)
      : timer_(aio), callback_(std::move(callback)) {}

  void Start(aos::monotonic_clock::time_point base,
             aos::monotonic_clock::duration period) {
    base_ = base;
    period_ = period;
    timer_.Schedule(base_, &OnFire, this);
  }

  // Retargets a running timer, exactly as an external Schedule() would.
  void Reschedule(aos::monotonic_clock::time_point base,
                  aos::monotonic_clock::duration period) {
    Start(base, period);
  }

  void Cancel() { timer_.Cancel(); }

 private:
  static void OnFire(Completion completion, void *ctx) {
    auto *self = static_cast<RepeatingTimer *>(ctx);
    // Re-arm before dispatching, so a callback that cancels or reschedules
    // wins over this arm rather than racing it.
    self->base_ += self->period_;
    self->timer_.Schedule(self->base_, &OnFire, self);
    self->callback_(completion);
  }

  Aio::Timer timer_;
  std::function<void(Completion)> callback_;
  aos::monotonic_clock::time_point base_;
  aos::monotonic_clock::duration period_{};
};

// Parameterized by backend name, the same string --aio_backend takes, so a
// failing test names the backend it failed on instead of an index -- and so
// adding a backend does not mean re-teaching this a new boolean.
class AioTest : public ::testing::TestWithParam<std::string> {
 protected:
  // Whether the backend under test is io_uring.  Several tests below cover
  // behavior only it has (SINGLE_ISSUER enforcement, orphaned destruction);
  // they skip elsewhere rather than assert something epoll never promised.
  //
  // Only Linux has an io_uring backend at all.  Everywhere else --aio_backend
  // is accepted and ignored -- macOS always runs kqueue, Windows always IOCP --
  // so the "io_uring" parameter is a second pass over the single backend that
  // platform has, and none of the io_uring-specific behavior applies to it.
  static bool IsIoUring() {
#if defined(__linux__)
    return GetParam() == "io_uring";
#else
    return false;
#endif
  }

  // The name the backend under test calls itself by in its messages.  On
  // Linux the parameter is that name; everywhere else the parameter is
  // ignored along with --aio_backend, so both passes run the one backend
  // that platform has and it is that one's name that surfaces.
  static std::string BackendName() {
#if defined(__linux__)
    return GetParam();
#elif defined(_WIN32)
    return "iocp";
#else
    return "kqueue";
#endif
  }

  void SetUp() override {
    // Pace ring creation across the whole suite, not just the
    // ring-churning tests -- see ThrottleOnKernelRingTeardown().
    ThrottleOnKernelRingTeardown();
    ::absl::SetFlag(&FLAGS_aio_backend, GetParam());
    ABSL_LOG(INFO) << "Testing Aio with the " << GetParam() << " backend.";
  }

  // Restores every flag SetUp() (or the test body) touched.
  absl::FlagSaver flag_saver_;
};

// Fixture for io_uring-specific regression tests; the loops inside these
// churn rings aggressively, so they also call ThrottleOnKernelRingTeardown()
// periodically themselves.
class AioReproTest : public ::testing::Test {
 protected:
  void SetUp() override {
#if defined(__linux__)
    ThrottleOnKernelRingTeardown();
    ::absl::SetFlag(&FLAGS_aio_backend, "io_uring");
#else
    // These pin down io_uring's internals -- SQEs staged against
    // --aio_queue_depth before the ring is enabled, CQ overflow terminating a
    // multishot poll -- and no other backend has those shapes at all: kqueue
    // submits each kevent() immediately and IOCP has no completion-queue
    // overflow, so --aio_queue_depth is not even read off Linux.
    //
    // The SetFlag above is a no-op here (--aio_backend is accepted and ignored
    // off Linux; see AioTest::IsIoUring()), so without this these would quietly
    // run a second pass over kqueue/IOCP and assert behavior those backends
    // never promised.  Skip rather than lie about what ran.
    GTEST_SKIP() << "io_uring-specific regression tests; Linux only.";
#endif
  }

  absl::FlagSaver flag_saver_;
};

// Tests that we can push basic strings through a pipe with io_uring.
TEST_P(AioTest, BasicPipeReadWrite) {
  Aio aio;
  Pipe pipe;

  char write_buf[] = "Hello io_uring!";
  char read_buf[64] = {0};

  AsyncRequest write_req;
  write_req.callback = [](Completion completion, void *) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    EXPECT_GT(completion.result, 0);
  };

  AsyncRequest read_req;
  read_req.callback = [](Completion completion, void *) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    EXPECT_GT(completion.result, 0);
  };

  size_t count = 0;
  {
    ScopedRealtime rt;

    aio.AsyncWrite(pipe.write_fd(), write_buf, &write_req);
    aio.AsyncRead(pipe.read_fd(), read_buf, &read_req);

    while ((!write_req.done || !read_req.done) && aio.Poll(true)) {
      ++count;
    }
  }

  // The epoll/kqueue backend processes exactly one event per Poll() call.
#if defined(__linux__)
  EXPECT_EQ(count, IsIoUring() ? 1 : 2);
#else
  EXPECT_EQ(count, 2);
#endif
  EXPECT_STREQ(read_buf, "Hello io_uring!");
}

// A deadline already in the past fires as soon as the loop is driven, and
// aos::monotonic_clock::epoch() is just the earliest such deadline -- not a
// special value.  aos::TimerHandler::Schedule() lets callers pass it to mean
// "now", and the simulated event loop runs on a timeline whose origin *is*
// the epoch, so a ShmEventLoop that treated it differently would diverge
// from sim for the same application code.
//
// The Linux backends need help here: they arm with timerfd_settime(2), and
// an itimerspec whose it_value is exactly zero means "disarm", not "expire
// immediately" -- so the epoch is the one past deadline that would silently
// never fire.  See IoUringTimerState::Schedule().
TEST_P(AioTest, ScheduleTimerAtEpochFiresTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  int fires = 0;
  timer.Schedule(
      aos::monotonic_clock::epoch(),
      [](Completion completion, void *ctx) {
        if (aos::IsOk(completion.status)) {
          ++*static_cast<int *>(ctx);
        }
      },
      &fires);

  // Bounded rather than a blocking drain: a dropped deadline means nothing
  // ever wakes the loop, so Poll(true) would hang here instead of failing.
  const auto give_up =
      aos::monotonic_clock::now() + std::chrono::milliseconds(500);
  while (fires == 0 && aos::monotonic_clock::now() < give_up) {
    aio.Poll(false);
  }
  EXPECT_EQ(fires, 1)
      << "A timer scheduled at the monotonic epoch never fired.";
}

// Tests that we can have 2 timers going.
TEST_P(AioTest, AsyncTimerTest) {
  Aio aio;

  Aio::Timer timer1(&aio);
  Aio::Timer timer2(&aio);

  aos::monotonic_clock::time_point timer_fired1 =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point timer_fired2 =
      aos::monotonic_clock::min_time;

  size_t count = 0;
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;

    timer1.Schedule(
        start_time + std::chrono::milliseconds(100),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<aos::monotonic_clock::time_point *>(ctx) =
              aos::monotonic_clock::now();
        },
        &timer_fired1);

    timer2.Schedule(
        start_time + std::chrono::milliseconds(500),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<aos::monotonic_clock::time_point *>(ctx) =
              aos::monotonic_clock::now();
        },
        &timer_fired2);

    while ((timer_fired1 == monotonic_clock::min_time ||
            timer_fired2 == monotonic_clock::min_time) &&
           aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_EQ(count, 2);
  // The not-early bounds stay exact: an absolute timer firing before its
  // deadline is a kernel-level guarantee violation, never scheduling noise.
  // The late bounds only need to discriminate a *wrong* deadline (unit
  // errors and deadline/interval mix-ups miss by at least the whole
  // deadline, i.e. 100ms/500ms or more) from ordinary dispatch latency on
  // a loaded machine -- observed up to ~175ms late under a saturated CI
  // node (20+ pegged containers), which a tight 100ms slack flaked on at
  // ~0.02% while proving nothing.  400ms sits well clear of measured noise
  // while still catching every real failure mode by a wide margin.
  constexpr std::chrono::milliseconds kLateSlack(400);
  EXPECT_GT(timer_fired1, start_time + std::chrono::milliseconds(100));
  EXPECT_LT(timer_fired1,
            start_time + std::chrono::milliseconds(100) + kLateSlack);
  EXPECT_GT(timer_fired2, start_time + std::chrono::milliseconds(500));
  EXPECT_LT(timer_fired2,
            start_time + std::chrono::milliseconds(500) + kLateSlack);
}

// Tests that ThreadSignal events trigger the registered SignalFd wakeup
// callback in the event loop.
TEST_P(AioTest, ThreadSignalTest) {
  Aio aio;

  aos::ipc_lib::ThreadSignalReceiver sfd;

  bool signal_fired = false;
  aio.RegisterThreadSignalReceiver(&sfd,
                                   [&signal_fired]() { signal_fired = true; });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();

  std::thread signaler([pid, tid]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    aos::ipc_lib::ThreadSignalSender signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  size_t count = 0;
  {
    ScopedRealtime rt;

    // Poll until the read request completes.
    while (!signal_fired && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_TRUE(signal_fired);

  signaler.join();
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// Tests that a registered SignalFd callback is successfully invoked multiple
// times when multiple signals are sent sequentially, using multishot poll.
TEST_P(AioTest, MultiThreadSignalTest) {
  Aio aio;
  // A lost wakeup leaves the main thread blocked in Poll(true) with the
  // signaler waiting on signal_count -- turn that hang into a clean death.
  ScopedDeathTestWatchdog watchdog;

  aos::ipc_lib::ThreadSignalReceiver sfd;

  std::atomic<size_t> signal_count{0};
  std::vector<aos::monotonic_clock::time_point> callback_times;
  callback_times.reserve(3);
  aio.RegisterThreadSignalReceiver(&sfd, [&signal_count, &callback_times]() {
    callback_times.push_back(aos::monotonic_clock::now());
    ++signal_count;
  });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();

  auto start = aos::monotonic_clock::now();
  // Each send waits for its callback before the next: wakeups coalesce by
  // contract (see Aio::RegisterThreadSignalReceiver()), so free-running
  // sends could merge into fewer callbacks and a wait for exactly three
  // would hang instead of failing.  Serialized sends pin what this test is
  // actually about -- the multishot registration keeps delivering, one
  // callback per wakeup, without re-arming.
  std::atomic<bool> give_up{false};
  std::thread signaler([pid, tid, &signal_count, &give_up]() {
    for (size_t i = 0; i < 3; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      aos::ipc_lib::ThreadSignalSender signaler_signal;
      signaler_signal.Signal(pid, tid);
      while (signal_count <= i && !give_up) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (give_up) {
        return;
      }
    }
  });

  {
    ScopedRealtime rt;

    // Poll until we receive all 3 signals.
    while (signal_count < 3 && aio.Poll(true)) {
    }
  }

  // Join before the first assertion.  An ASSERT_* returns from the test
  // body, and destroying a still-joinable std::thread is std::terminate();
  // give_up bounds the signaler's wait so the join cannot hang if a
  // callback never arrived.
  give_up = true;
  signaler.join();

  EXPECT_EQ(signal_count, 3);

  ASSERT_EQ(callback_times.size(), 3);
  // The first signal shouldn't show up until the thread finishes its first 50ms
  // wait. We use 40ms of slack to account for non-realtime OS scheduler
  // variations.
  EXPECT_GE(callback_times[0], start + std::chrono::milliseconds(40));
  // The subsequent callbacks must be received in order.
  EXPECT_GE(callback_times[1], callback_times[0]);
  EXPECT_GE(callback_times[2], callback_times[1]);
  EXPECT_LT(callback_times[2], start + std::chrono::seconds(1));

  aio.UnregisterThreadSignalReceiver(&sfd);
}

// Tests that we can cancel a timer immediately after scheduling it.
TEST_P(AioTest, CancelTimerTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  bool callback_invoked = false;

  auto start = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;
    // Schedule a timer for 10 seconds in the future.
    timer.Schedule(
        start + std::chrono::seconds(10),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<bool *>(ctx) = true;
        },
        &callback_invoked);

    // Immediately cancel the timer.
    timer.Cancel();

    // Poll to ensure it never fires.
    aio.Poll(false);
  }

  EXPECT_FALSE(callback_invoked);
  EXPECT_LT(aos::monotonic_clock::now(), start + std::chrono::seconds(1));
}

// Tests that canceling a pending AsyncRead request executes the callback
// asynchronously inside Poll(), never nested/synchronously inside Cancel().
TEST_P(AioTest, CancelAsyncReadTest) {
  Aio aio;
  Pipe pipe;

  AsyncRequest request;
  bool callback_invoked = false;
  std::optional<aos::Status> fired_status;

  char buf[10];

  {
    ScopedRealtime rt;
    request.callback = [](Completion completion, void *ctx) {
      auto *state =
          static_cast<std::pair<std::optional<aos::Status> *, bool *> *>(ctx);
      state->first->emplace(std::move(completion.status));
      *state->second = true;
    };
    std::pair<std::optional<aos::Status> *, bool *> ctx(&fired_status,
                                                        &callback_invoked);
    request.context = &ctx;

    aio.AsyncRead(pipe.read_fd(), buf, &request);

    // Verify it hasn't run yet.
    EXPECT_FALSE(callback_invoked);

    // Cancel the request.
    aio.Cancel(&request);

    // The callback must NOT run synchronously inside Cancel().
    EXPECT_FALSE(callback_invoked);

    // Now poll, which should execute the canceled callback.
    while (!callback_invoked && aio.Poll(true)) {
    }
  }

  EXPECT_TRUE(callback_invoked);
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_FALSE(aos::IsOk(*fired_status));
  EXPECT_EQ(fired_status->error().message(), "Canceled");
}

// Contract test for aio.h's constraint 2: a canceled request only ever has
// to live until its callback runs.  The cancel's kernel acknowledgment
// deliberately does not name the request (it carries the loop-owned
// cancel_ack_sentinel_ identity -- see IoUringImpl::Cancel()), so freeing
// the request immediately after the Canceled callback and continuing to
// poll must be clean.  A regression here is a read of freed memory in
// DrainCompletions()'s ack handling, which needs ASAN to fail reliably.
TEST_P(AioTest, FreeCanceledRequestAfterCallback) {
  Aio aio;
  Pipe pipe;

  char buf[8];
  bool invoked = false;
  auto request = std::make_unique<AsyncRequest>();
  request->callback = [](Completion completion, void *ctx) {
    EXPECT_FALSE(aos::IsOk(completion.status));
    *static_cast<bool *>(ctx) = true;
  };
  request->context = &invoked;

  aio.AsyncRead(pipe.read_fd(), buf, request.get());
  aio.Cancel(request.get());
  while (!invoked && aio.Poll(true)) {
  }
  ASSERT_TRUE(invoked);

  // Free immediately after the callback -- the documented earliest legal
  // point -- then keep polling.  Any late traffic for the cancel must not
  // touch the freed request.
  request.reset();
  for (int i = 0; i < 10; ++i) {
    aio.Poll(false);
  }
}

// Tests that we can change a timer's deadline by canceling and rescheduling it.
TEST_P(AioTest, ChangeTimerDeadlineTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  std::optional<aos::Status> fired_status;
  size_t invocations = 0;

  std::pair<std::optional<aos::Status> *, size_t *> ctx(&fired_status,
                                                        &invocations);

  size_t count = 0;
  auto start = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;
    // Schedule a timer for 10 seconds in the future.
    timer.Schedule(
        start + std::chrono::seconds(10),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          auto state =
              static_cast<std::pair<std::optional<aos::Status> *, size_t *> *>(
                  ctx);
          state->first->emplace(std::move(completion.status));
          (*state->second)++;
        },
        &ctx);

    // Re-schedule for 100 milliseconds (implicitly cancels the first).
    timer.Schedule(
        start + std::chrono::milliseconds(100),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          auto state =
              static_cast<std::pair<std::optional<aos::Status> *, size_t *> *>(
                  ctx);
          state->first->emplace(std::move(completion.status));
          (*state->second)++;
        },
        &ctx);

    // Poll until the rescheduled timer fires (1 completion).
    while (invocations < 1 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(invocations, 1);
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_TRUE(aos::IsOk(*fired_status));
  auto end = aos::monotonic_clock::now();
  EXPECT_GT(end, start + std::chrono::milliseconds(100));
  EXPECT_LT(end, start + std::chrono::seconds(1));
}

// Tests that scheduling a timer with a deadline in the past fires immediately
// on the next poll.
TEST_P(AioTest, PastTimerTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  bool callback_invoked = false;

  size_t count;
  aos::monotonic_clock::time_point start;
  {
    ScopedRealtime rt;
    start = aos::monotonic_clock::now();
    // Schedule a timer with a deadline in the past.
    timer.Schedule(
        start - std::chrono::seconds(5),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<bool *>(ctx) = true;
        },
        &callback_invoked);

    // Poll once. It should fire immediately.
    count = 0;
    while (!callback_invoked && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_EQ(count, 1);
  EXPECT_TRUE(callback_invoked);
  EXPECT_LT(aos::monotonic_clock::now(),
            start + std::chrono::milliseconds(500));
}

// Tests that a repeating timer can be implemented by rescheduling from the
// completion callback.
TEST_P(AioTest, RepeatingTimerTest) {
  Aio aio;

  struct TimerContext {
    Aio::Timer timer;
    size_t count = 0;
    bool done = false;

    TimerContext(Aio *aio) : timer(aio) {}

    static void OnTimer(Completion completion, void *ctx) {
      auto state = static_cast<TimerContext *>(ctx);
      EXPECT_TRUE(aos::IsOk(completion.status));
      state->count++;
      if (state->count < 3) {
        // Re-schedule for 50 milliseconds in the future.
        state->timer.Schedule(
            aos::monotonic_clock::now() + std::chrono::milliseconds(50),
            &OnTimer, state);
      } else {
        state->done = true;
      }
    }
  };

  TimerContext timer_ctx(&aio);

  size_t count = 0;
  auto start = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;
    timer_ctx.timer.Schedule(start + std::chrono::milliseconds(50),
                             &TimerContext::OnTimer, &timer_ctx);

    // Poll until repeating timer triggers all iterations.
    while (!timer_ctx.done && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(timer_ctx.count, 3);
  auto end = aos::monotonic_clock::now();
  EXPECT_GT(end, start + std::chrono::milliseconds(150));
  EXPECT_LT(end, start + std::chrono::seconds(1));
}

// Tests that we can cancel a timer after it has been active for some time.
TEST_P(AioTest, CancelTimerAfterDelayTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  bool callback_invoked = false;

  aos::monotonic_clock::time_point start;
  {
    ScopedRealtime rt;
    start = aos::monotonic_clock::now();
    // Schedule a timer for 10 seconds in the future.
    timer.Schedule(
        start + std::chrono::seconds(10),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<bool *>(ctx) = true;
        },
        &callback_invoked);

    // Let it run for 100 milliseconds.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    aio.Poll(false);

    // Verify the callback has NOT been invoked yet.
    EXPECT_FALSE(callback_invoked);

    // Cancel the timer.
    timer.Cancel();

    // Poll to ensure it never fires.
    aio.Poll(false);
  }

  EXPECT_FALSE(callback_invoked);
  EXPECT_LT(aos::monotonic_clock::now(), start + std::chrono::seconds(1));
}

// Tests that duplicate registrations for legacy fds and signalfds die.
TEST_P(AioTest, DuplicateRegistrationDeathTest) {
  Aio aio;
  Pipe pipe;

  aio.OnReadable(pipe.read_fd(), []() {});
  EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), []() {}),
               "Duplicate in functions for");

  aio.OnWritable(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.OnWritable(pipe.write_fd(), []() {}),
               "Duplicate out functions for");

  aio.OnError(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.OnError(pipe.write_fd(), []() {}),
               "Duplicate error functions for");

  aos::ipc_lib::ThreadSignalReceiver sfd;
  aio.RegisterThreadSignalReceiver(&sfd, []() {});
  EXPECT_DEATH(aio.RegisterThreadSignalReceiver(&sfd, []() {}), "Duplicate.*");

  // Clean up registered resources before loop destruction.
  aio.DeleteFd(pipe.read_fd());
  aio.DeleteFd(pipe.write_fd());
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// Tests that unregistering untracked legacy fds or signalfds dies.
TEST_P(AioTest, UntrackedUnregistrationDeathTest) {
  Aio aio;

  EXPECT_DEATH(aio.DeleteFd(FakeFd(999)), "fd .* not found");
  aos::ipc_lib::ThreadSignalReceiver sfd2;
  // Each backend words this differently.  Match with a matcher rather than an
  // alternation regex: gtest only has POSIX regexes where <regex.h> exists, and
  // falls back to its own engine (which has no alternation) on Windows.
  EXPECT_DEATH(
      aio.UnregisterThreadSignalReceiver(&sfd2),
      ::testing::AnyOf(::testing::HasSubstr("ThreadSignalReceiver not found"),
                       ::testing::ContainsRegex("fd .* not found")));
}

// Tests that calling Poll() from inside a callback dies (constraint 3 in
// aio.h).
TEST_P(AioTest, NestedPollDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  Aio aio;

  Aio::Timer timer(&aio);
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) { static_cast<Aio *>(ctx)->Poll(false); },
      &aio);
  EXPECT_DEATH(
      {
        while (aio.Poll(true)) {
        }
      },
      "reentered");
}

// Tests that mixing OnEvents and other legacy hooks or calling invalid methods
// triggers assertions.
TEST_P(AioTest, MixedRegistrationAndInvalidHookDeathTest) {
  Aio aio;
  Pipe pipe;

  // OnEvents, then OnReadable fails.
  aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
  EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), []() {}),
               "Cannot mix OnEvents and OnReadable");

  // OnEvents, then EnableWritable fails.
  EXPECT_DEATH(aio.EnableWritable(pipe.read_fd()),
               "EnableWritable is only for fds registered using OnWritable");

  // OnEvents, then DisableWritable fails.
  EXPECT_DEATH(aio.DisableWritable(pipe.read_fd()),
               "DisableWritable is only for fds registered using OnWritable");

  aio.DeleteFd(pipe.read_fd());

  // OnWritable, then SetEvents fails.
  aio.OnWritable(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.SetEvents(pipe.write_fd(), 0x04),
               "SetEvents is only for fds registered using OnEvents");

  aio.DeleteFd(pipe.write_fd());

  if (!IsIoUring()) {
    // Under epoll, verify that mixing AsyncRead/AsyncWrite and
    // OnReadable/OnWritable/OnEvents fails.
    AsyncRequest req;
    char buf[10];

    // AsyncRead, then OnReadable fails.
    // The raw request has to be submitted inside the death test: forking
    // with one in flight is itself fatal now (ForkWithPendingAsyncWriteDies),
    // and EXPECT_DEATH forks.  The child does both halves and dies on the
    // second, which is what this is checking either way.
    EXPECT_DEATH(
        {
          aio.AsyncRead(pipe.read_fd(), buf, &req);
          aio.OnReadable(pipe.read_fd(), []() {});
        },
        "Cannot mix OnReadable and AsyncRead");

    // OnReadable, then AsyncRead fails.
    aio.OnReadable(pipe.read_fd(), []() {});
    EXPECT_DEATH(aio.AsyncRead(pipe.read_fd(), buf, &req),
                 "Cannot mix OnReadable and AsyncRead");
    aio.DeleteFd(pipe.read_fd());

    // AsyncWrite, then OnWritable fails.  Submitted inside, as above.
    EXPECT_DEATH(
        {
          aio.AsyncWrite(pipe.write_fd(), buf, &req);
          aio.OnWritable(pipe.write_fd(), []() {});
        },
        "Cannot mix OnWritable and AsyncWrite");

    // OnWritable, then AsyncWrite fails.
    aio.OnWritable(pipe.write_fd(), []() {});
    EXPECT_DEATH(aio.AsyncWrite(pipe.write_fd(), buf, &req),
                 "Cannot mix OnWritable and AsyncWrite");
    aio.DeleteFd(pipe.write_fd());

    // AsyncRead, then OnEvents fails.  Submitted inside, as above.
    EXPECT_DEATH(
        {
          aio.AsyncRead(pipe.read_fd(), buf, &req);
          aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
        },
        "Cannot mix OnEvents and AsyncRead/AsyncWrite");

    // OnEvents, then AsyncRead fails.
    aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
    EXPECT_DEATH(aio.AsyncRead(pipe.read_fd(), buf, &req),
                 "Cannot mix OnEvents and AsyncRead/AsyncWrite");
    aio.DeleteFd(pipe.read_fd());
  }
}

// Tests that a failed I/O operation (like reading from an invalid fd)
// is correctly captured as an error status and the raw errno is populated.
TEST_P(AioTest, FailedIoErrorTest) {
  Aio aio;

  AsyncRequest read_req;
  std::optional<aos::Status> fired_status;
  int32_t result_code = 0;

  read_req.callback = [](Completion completion, void *ctx) {
    auto self_ctx =
        static_cast<std::pair<std::optional<aos::Status> *, int32_t *> *>(ctx);
    self_ctx->first->emplace(std::move(completion.status));
    *self_ctx->second = completion.result;
  };
  std::pair<std::optional<aos::Status> *, int32_t *> ctx(&fired_status,
                                                         &result_code);
  read_req.context = &ctx;

  char buf[8];
  size_t count = 0;
  {
    ScopedRealtime rt;
    // Schedule a read on an invalid file descriptor (-1).
    aio.AsyncRead(FakeFd(-1), buf, &read_req);

    // Poll until it executes.
    while (!read_req.done && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_FALSE(aos::IsOk(*fired_status));
  // Each backend names itself in its operational-failure message.
  EXPECT_EQ(fired_status->error().message(), BackendName() + " error");
  EXPECT_EQ(result_code, EBADF);
}

// Tests that async registrations recycle: every request here fully completes
// before the next distinct fd is used, so completed requests must return
// their slot to the pool.  Uses more simultaneously-open fds than the epoll
// backend's default 16-slot pool (one of which the wakeup eventfd occupies),
// which used to abort with "Async registration pool exhausted" on the 16th
// distinct fd.
TEST_P(AioTest, AsyncRegistrationPoolRecycleTest) {
  Aio aio;

  std::array<Pipe, 20> pipes;
  char buf[8];
  for (auto &pipe : pipes) {
    AsyncRequest req;
    bool fired = false;
    req.callback = [](Completion completion, void *ctx) {
      EXPECT_TRUE(aos::IsOk(completion.status));
      *static_cast<bool *>(ctx) = true;
    };
    req.context = &fired;

    aio.AsyncRead(pipe.read_fd(), buf, &req);
    pipe.Write("x");
    while (!fired && aio.Poll(true)) {
    }
    EXPECT_TRUE(fired);
  }
}

// Tests that a callback which deletes its own fd can keep using its captures
// afterwards.  Deleting the fd releases the registration that owns the
// std::function currently executing; destroying that function mid-call frees
// the lambda's captures out from under the running code (a heap-use-after-free
// under ASAN).  The capture is sized past any small-buffer optimization so it
// actually lives on the heap.
TEST_P(AioTest, DeleteOwnFdFromCallbackKeepsCapturesTest) {
  Aio aio;
  Pipe pipe;

  bool fired = false;
  const std::string marker(128, 'm');
  aio.OnReadable(pipe.read_fd(), [&aio, &pipe, &fired, marker]() {
    aio.DeleteFd(pipe.read_fd());
    // The registration owning this lambda was just released; the captures
    // must stay intact until dispatch is over.
    EXPECT_EQ(marker, std::string(128, 'm'));
    fired = true;
  });
  pipe.Write("x");

  while (!fired && aio.Poll(true)) {
  }
  EXPECT_TRUE(fired);

  // The retired registration is reclaimed on the next Poll(), once no
  // dispatch is in flight.
  aio.Poll(false);
}

// Tests AsyncRead()/AsyncWrite() on a regular file.  epoll_ctl(ADD) rejects
// regular files with EPERM -- they have no wait queue and are simply always
// ready -- which used to abort the epoll backend on a perfectly valid
// mkstemp() descriptor.
TEST_P(AioTest, RegularFileAsyncReadWriteTest) {
  Aio aio;

  const char *tmpdir = getenv("TEST_TMPDIR");
  std::string path =
      std::string(tmpdir != nullptr ? tmpdir : "/tmp") + "/aio_regular_XXXXXX";
  int fd = mkstemp(path.data());
  ASSERT_GE(fd, 0);
  ASSERT_EQ(unlink(path.c_str()), 0);

  const char data[] = "regular file data";

  AsyncRequest write_req;
  std::optional<int32_t> write_result;
  write_req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    static_cast<std::optional<int32_t> *>(ctx)->emplace(completion.result);
  };
  write_req.context = &write_result;
  aio.AsyncWrite(fd, std::span<const char>(data, sizeof(data) - 1), &write_req);
  while (!write_req.done && aio.Poll(true)) {
  }
  ASSERT_TRUE(write_result.has_value());
  EXPECT_EQ(*write_result, static_cast<int32_t>(sizeof(data) - 1));

  ASSERT_EQ(lseek(fd, 0, SEEK_SET), 0);

  char buf[64] = {};
  AsyncRequest read_req;
  std::optional<int32_t> read_result;
  read_req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    static_cast<std::optional<int32_t> *>(ctx)->emplace(completion.result);
  };
  read_req.context = &read_result;
  aio.AsyncRead(fd, buf, &read_req);
  while (!read_req.done && aio.Poll(true)) {
  }
  ASSERT_TRUE(read_result.has_value());
  EXPECT_EQ(*read_result, static_cast<int32_t>(sizeof(data) - 1));
  EXPECT_STREQ(buf, "regular file data");

  ABSL_PCHECK(close(fd) == 0);
}

// Tests that a pending AsyncRead() completes with EOF when the write end of
// an empty pipe closes.  That hangup surfaces as EPOLLHUP with no EPOLLIN;
// the epoll backend used to route it only to the (absent) error callback, so
// the request stayed pending forever while the level-triggered EPOLLHUP spun
// the loop with no progress.
TEST_P(AioTest, AsyncReadEofOnHangupTest) {
  Aio aio;
  Pipe pipe;

  AsyncRequest req;
  std::optional<aos::Status> fired_status;
  int32_t result = -1;
  req.callback = [](Completion completion, void *ctx) {
    auto *self_ctx =
        static_cast<std::pair<std::optional<aos::Status> *, int32_t *> *>(ctx);
    self_ctx->first->emplace(std::move(completion.status));
    *self_ctx->second = completion.result;
  };
  std::pair<std::optional<aos::Status> *, int32_t *> ctx(&fired_status,
                                                         &result);
  req.context = &ctx;

  char buf[8];
  aio.AsyncRead(pipe.read_fd(), buf, &req);
  // Make sure the read is armed and pending before hanging up.
  aio.Poll(false);
  pipe.close_write_fd();

  // Bounded non-blocking polling: on regression this hangs (or spins), and a
  // deadline turns that into a loud failure instead of a test timeout.
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!req.done && std::chrono::steady_clock::now() < deadline) {
    aio.Poll(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(req.done) << "AsyncRead never completed after writer hangup";
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_TRUE(aos::IsOk(*fired_status));
  EXPECT_EQ(result, 0);
}

// Tests that a timer destroyed by its own callback stops delivery cleanly
// instead of touching the freed timer state.  Both backends dispatch the
// user callback as the last thing they do with the state, specifically so
// that this is safe; this holds them to it.
TEST_P(AioTest, TimerDestroyedByOwnCallbackTest) {
  Aio aio;

  struct Ctx {
    std::unique_ptr<Aio::Timer> timer;
    size_t count = 0;
  } ctx;
  ctx.timer = std::make_unique<Aio::Timer>(&aio);

  const auto start = aos::monotonic_clock::now();
  ctx.timer->Schedule(
      start + std::chrono::milliseconds(10),
      [](Completion completion, void *raw) {
        EXPECT_TRUE(aos::IsOk(completion.status));
        auto *ctx = static_cast<Ctx *>(raw);
        ++ctx->count;
        ctx->timer.reset();
      },
      &ctx);

  // Let the deadline pass well before servicing it, so the callback runs
  // out of a completion the loop had already been sitting on.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  aio.Poll(true);
  while (aio.Poll(false)) {
  }
  EXPECT_EQ(ctx.count, 1u);
}

// Two timers with the same deadline both fire, so both completions are
// extracted into one dispatch batch.  The first callback then acts on the
// second timer -- which has already fired, kernel-side and as far as the
// backend's queue is concerned, but has NOT been dispatched yet.
//
// This is a genuine hole in any batch-completion design: between "the
// completion exists" and "the callback ran" there is a window where the
// owner can legitimately change its mind, and the backend has to honor the
// newer intent rather than deliver the stale firing.  The three things an
// owner can do in that window are covered separately below, because they
// have different right answers.
//
// Both timers are scheduled for a deadline already in the past and then
// serviced with a single Poll(), which is what guarantees one batch rather
// than two.
namespace {
struct TwoTimerBatch {
  Aio aio;
  std::unique_ptr<Aio::Timer> first;
  std::unique_ptr<Aio::Timer> second;
  int second_fires = 0;
  void *first_context = nullptr;

  TwoTimerBatch()
      : first(std::make_unique<Aio::Timer>(&aio)),
        second(std::make_unique<Aio::Timer>(&aio)) {}

  // Arms both for the same already-past deadline, with `on_first` running
  // as the first timer's callback.
  void Arm(CompletionCallback on_first) {
    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    first->Schedule(deadline, on_first, this);
    second->Schedule(
        deadline,
        [](Completion completion, void *ctx) {
          if (aos::IsOk(completion.status)) {
            ++static_cast<TwoTimerBatch *>(ctx)->second_fires;
          }
        },
        this);
    // Both deadlines are well past by the time anything polls.
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
};
}  // namespace

// Control for the three tests below, pinning down that the window they
// describe actually exists.  Without this they could pass for the wrong
// reason -- the second timer's completion simply not existing yet when the
// first callback ran.
//
// Both timers are due, so both are ready as far as the backend is
// concerned, but Poll() delivers exactly one completion (see Aio::Poll()).
// That is what creates the window the three tests below exercise: the first
// callback runs while the second timer has already fired and has not been
// delivered yet, and it is free to cancel, reschedule, or destroy it.
//
// Identical on every backend by construction, which is the point -- a
// consumer cannot tell io_uring from epoll by counting callbacks.  io_uring
// reaches this by draining its whole completion queue and dispatching one
// entry off it; epoll by asking the kernel for one event at a time.  Same
// contract, and the same window, out of different mechanisms.
TEST_P(AioTest, OneCompletionPerPollTest) {
  TwoTimerBatch batch;
  int first_fires = 0;
  batch.first_context = &first_fires;
  batch.Arm([](Completion, void *ctx) {
    ++*static_cast<int *>(static_cast<TwoTimerBatch *>(ctx)->first_context);
  });

  // Exactly one Poll(), no drain loop.
  batch.aio.Poll(true);
  EXPECT_EQ(first_fires, 1);
  EXPECT_EQ(batch.second_fires, 0)
      << "Poll() delivered two completions; the same-batch tests below rely "
         "on the second timer still being undelivered when the first "
         "callback runs, and prove nothing without that.";

  // ...and the second one arrives on a subsequent Poll().
  while (batch.second_fires == 0 && batch.aio.Poll(true)) {
  }
  EXPECT_EQ(batch.second_fires, 1);
}

// Canceling the second timer from the first's callback must suppress its
// pending firing entirely -- Timer::Cancel() is documented as silent, and
// "already fired but not yet delivered" is still pending.
TEST_P(AioTest, CancelATimerFiringInTheSameBatchTest) {
  TwoTimerBatch batch;
  batch.Arm([](Completion, void *ctx) {
    static_cast<TwoTimerBatch *>(ctx)->second->Cancel();
  });

  batch.aio.Poll(true);
  while (batch.aio.Poll(false)) {
  }
  EXPECT_EQ(batch.second_fires, 0)
      << "A timer canceled before its already-queued firing was dispatched "
         "delivered it anyway.";
}

// Rescheduling the second timer from the first's callback must move its
// firing to the new deadline, not deliver the old one immediately.  Getting
// this wrong is how a consumer that filters early wakeups (ShmEventLoop
// does) ends up with a timer that is armed everywhere in userspace and
// absent from the kernel.
TEST_P(AioTest, RescheduleATimerFiringInTheSameBatchTest) {
  TwoTimerBatch batch;
  batch.Arm([](Completion, void *ctx) {
    auto *self = static_cast<TwoTimerBatch *>(ctx);
    self->second->Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(50),
        [](Completion completion, void *inner) {
          if (aos::IsOk(completion.status)) {
            ++static_cast<TwoTimerBatch *>(inner)->second_fires;
          }
        },
        self);
  });

  batch.aio.Poll(true);
  while (batch.aio.Poll(false)) {
  }
  EXPECT_EQ(batch.second_fires, 0)
      << "The superseded firing was delivered instead of being replaced by "
         "the new schedule.";

  // ...and the new schedule must still be live.
  while (batch.second_fires == 0 && batch.aio.Poll(true)) {
  }
  EXPECT_EQ(batch.second_fires, 1) << "The rescheduled timer never fired.";
}

// Destroying the second timer from the first's callback must not leave the
// dispatch loop walking into freed memory when it reaches that timer's
// already-queued completion.
TEST_P(AioTest, DestroyATimerFiringInTheSameBatchTest) {
  TwoTimerBatch batch;
  batch.Arm([](Completion, void *ctx) {
    static_cast<TwoTimerBatch *>(ctx)->second.reset();
  });

  batch.aio.Poll(true);
  while (batch.aio.Poll(false)) {
  }
  EXPECT_EQ(batch.second_fires, 0);
}

// Tests the normal behavior of OnEvents (scheduling, callback delivery,
// persistent one-shot re-submission, and unregistering).
TEST_P(AioTest, LegacyFdTest) {
  Aio aio;
  Pipe pipe;

  size_t callback_count = 0;
  uint32_t active_events = 0;

  // Register for input readiness (0x01 = POLLIN / Epoll In).
  aio.OnEvents(pipe.read_fd(),
               [&callback_count, &active_events](uint32_t events) {
                 ++callback_count;
                 active_events = events;
               });
  aio.SetEvents(pipe.read_fd(), 0x01);

  size_t count = 0;

  {
    ScopedRealtime rt;
    // Verify that since the pipe is empty, no callback fires.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 0);

  // Write some data to make the pipe readable.
  pipe.Write("a");

  {
    ScopedRealtime rt;
    // Poll.  The callback should fire.
    while (callback_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(active_events & 0x01);

  // Read the data to consume it and clear readability.
  EXPECT_EQ(pipe.Read(1), "a");

  {
    ScopedRealtime rt;
    // Verify that polling now does not trigger additional callbacks.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 1);

  // Write again.  Since it is persistently re-submitted, it should fire again.
  pipe.Write("a");
  count = 0;
  {
    ScopedRealtime rt;
    while (callback_count == 1 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 2);

  // Read data to clear it.
  EXPECT_EQ(pipe.Read(1), "a");

  // Unregister the descriptor.
  aio.DeleteFd(pipe.read_fd());

  // Write more data.
  pipe.Write("a");

  {
    ScopedRealtime rt;
    // Poll.  The callback should NOT fire anymore.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 2);

  // Clean up.
  EXPECT_EQ(pipe.Read(1), "a");
}

// Tests the writable readiness behavior of OnEvents (using 0x04 / POLLOUT).
TEST_P(AioTest, LegacyFdWritableTest) {
  Aio aio;
  Pipe pipe;

  size_t callback_count = 0;
  uint32_t active_events = 0;

  // Register for output readiness (0x04 = POLLOUT / Epoll Out).
  aio.OnEvents(pipe.write_fd(),
               [&callback_count, &active_events](uint32_t events) {
                 ++callback_count;
                 active_events = events;
               });
  aio.SetEvents(pipe.write_fd(), 0x04);

  size_t count = 0;
  {
    ScopedRealtime rt;
    // Poll.  Since the pipe is empty and has space, it should be writable
    // immediately.
    while (callback_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(active_events & 0x04);

  // Clean up.
  aio.DeleteFd(pipe.write_fd());
}

// Tests the error/hangup readiness behavior of OnEvents (using 0x08 / POLLERR |
// POLLHUP).
TEST_P(AioTest, LegacyFdErrorTest) {
  Aio aio;
  Pipe pipe;

  size_t callback_count = 0;
  uint32_t active_events = 0;

  // Register the read end for hangup/error events (0x08 = POLLERR | POLLHUP).
  aio.OnEvents(pipe.read_fd(),
               [&callback_count, &active_events](uint32_t events) {
                 ++callback_count;
                 active_events = events;
               });
  aio.SetEvents(pipe.read_fd(), 0x08);

  {
    ScopedRealtime rt;
    // Verify no callback fires initially.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 0);

  // Close the write end of the pipe to trigger POLLHUP.
  pipe.close_write_fd();

  size_t count = 0;
  {
    ScopedRealtime rt;
    // Poll.  The callback should fire with the hangup/error event.
    while (callback_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(active_events & 0x08);

  // Clean up.
  aio.DeleteFd(pipe.read_fd());
}

// Tests that we can schedule, cancel, and schedule again before polling,
// and it does not hang and correctly processes the rescheduled timer.
TEST_P(AioTest, ScheduleCancelScheduleTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  size_t canceled_count = 0;
  size_t fired_count = 0;

  struct TestState {
    size_t *canceled_count;
    size_t *fired_count;
  };
  TestState state{&canceled_count, &fired_count};

  // 1. Schedule timer.
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion /*completion*/, void *ctx) {
        auto *s = static_cast<TestState *>(ctx);
        (*s->canceled_count)++;
      },
      &state);

  // 2. Cancel timer.
  timer.Cancel();
  EXPECT_EQ(canceled_count, 0);
  EXPECT_EQ(fired_count, 0);

  // 3. Schedule timer again.
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(100),
      [](Completion completion, void *ctx) {
        auto *s = static_cast<TestState *>(ctx);
        EXPECT_TRUE(aos::IsOk(completion.status));
        (*s->fired_count)++;
      },
      &state);

  // The first callback (canceled) should NEVER run.
  EXPECT_EQ(canceled_count, 0);
  EXPECT_EQ(fired_count, 0);

  // 4. Poll. We expect the new timer to fire.
  while (fired_count == 0 && aio.Poll(true)) {
  }

  EXPECT_EQ(canceled_count, 0);
  EXPECT_EQ(fired_count, 1);
}

struct NoNestedCallbackState {
  bool timer1_fired = false;
  bool timer3_fired = false;
  // Set only while timer1's callback is inside Schedule().  Timer 3's callback
  // running while this is set is what "nested" means; timer 3 running before
  // timer 1 at all is just an ordering the API never promised either way.
  bool in_timer1_schedule = false;
  bool nested_callback_detected = false;
  Aio::Timer *timer2 = nullptr;
};

// Tests that cancelling or rescheduling a timer does not trigger other
// pending completion callbacks nested inside the current callback context.
TEST_P(AioTest, NoNestedCallbackTest) {
  Aio aio;

  Aio::Timer timer1(&aio);
  Aio::Timer timer2(&aio);
  Aio::Timer timer3(&aio);

  NoNestedCallbackState test_state;
  test_state.timer2 = &timer2;

  timer1.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(100),
      [](Completion, void *ctx) {
        auto *s = static_cast<NoNestedCallbackState *>(ctx);
        s->timer1_fired = true;
        // Reschedule timer2 (which was scheduled for 10s).
        // This must NOT execute timer3's callback nested inside here.
        s->in_timer1_schedule = true;
        s->timer2->Schedule(
            aos::monotonic_clock::now() + std::chrono::seconds(20),
            [](Completion, void *) {}, nullptr);
        s->in_timer1_schedule = false;
      },
      &test_state);

  // Timer 2 is scheduled for 10s.
  timer2.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion, void *) {}, nullptr);

  // Timer 3 is scheduled to expire at the same time as Timer 1.
  timer3.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(100),
      [](Completion, void *ctx) {
        auto *s = static_cast<NoNestedCallbackState *>(ctx);
        // Getting here from inside timer1's Schedule() is the failure; getting
        // here from Poll() is fine no matter which timer got there first.
        if (s->in_timer1_schedule) {
          s->nested_callback_detected = true;
        }
        s->timer3_fired = true;
      },
      &test_state);

  // Run the event loop until both Timer 1 and Timer 3 have fired.
  while ((!test_state.timer1_fired || !test_state.timer3_fired) &&
         aio.Poll(true)) {
  }

  EXPECT_TRUE(test_state.timer1_fired);
  EXPECT_TRUE(test_state.timer3_fired);
  EXPECT_FALSE(test_state.nested_callback_detected);
}

// Tests the OnReadable/OnWritable/OnError legacy EPoll-like APIs.
TEST_P(AioTest, EPollLikeLegacyFdTest) {
  Aio aio;
  Pipe pipe;

  size_t readable_count = 0;
  size_t writable_count = 0;

  aio.OnReadable(pipe.read_fd(), [&readable_count]() { ++readable_count; });
  aio.OnWritable(pipe.write_fd(), [&writable_count]() { ++writable_count; });

  // Initially, writable should fire because the pipe is empty.
  size_t count = 0;
  {
    ScopedRealtime rt;
    while (writable_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(writable_count, 1);
  EXPECT_EQ(readable_count, 0);

  // Disable writability.
  {
    ScopedRealtime rt;
    aio.DisableWritable(pipe.write_fd());
  }

  // Write a byte to make readable fire.
  pipe.Write("a");

  // Poll; readable should fire. Writable should NOT fire again since disabled.
  count = 0;
  {
    ScopedRealtime rt;
    while (readable_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(readable_count, 1);
  EXPECT_EQ(writable_count, 1);

  // Re-enable writability.
  {
    ScopedRealtime rt;
    aio.EnableWritable(pipe.write_fd());
  }
  count = 0;
  {
    ScopedRealtime rt;
    while (writable_count == 1 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(writable_count, 2);

  // Clean up.
  aio.DeleteFd(pipe.read_fd());
  aio.DeleteFd(pipe.write_fd());
}

// Tests the OnEvents/SetEvents legacy EPoll-like APIs.
TEST_P(AioTest, EPollLikeOnEventsTest) {
  Aio aio;
  Pipe pipe;

  size_t event_count = 0;
  uint32_t active_events = 0;

  aio.OnEvents(pipe.read_fd(), [&event_count, &active_events](uint32_t events) {
    ++event_count;
    active_events = events;
  });

  // Schedule events (0x01 = POLLIN / Epoll In).
  {
    ScopedRealtime rt;
    aio.SetEvents(pipe.read_fd(), 0x01);
  }

  // Write a byte.
  pipe.Write("x");

  // Poll; callback should fire.
  size_t count = 0;
  {
    ScopedRealtime rt;
    while (event_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(event_count, 1);
  EXPECT_TRUE(active_events & 0x01);

  // Clean up.
  aio.DeleteFd(pipe.read_fd());
}

// Tests DeleteFd and ForgetClosedFd APIs.
TEST_P(AioTest, EPollLikeDeleteAndForgetTest) {
  Aio aio;
  Pipe pipe;

  size_t readable_count = 0;
  aio.OnReadable(pipe.read_fd(), [&readable_count]() { ++readable_count; });

  // Delete fd.
  aio.DeleteFd(pipe.read_fd());

  // Write a byte and verify callback is not called.
  pipe.Write("x");

  {
    ScopedRealtime rt;
    aio.Poll(false);
  }
  EXPECT_EQ(readable_count, 0);

  // Register the write end and verify the registration is live -- an empty
  // pipe's write end is immediately writable.
  size_t writable_count = 0;
  FileDescriptor w_fd = pipe.write_fd();
  aio.OnWritable(w_fd, [&writable_count]() { ++writable_count; });
  {
    ScopedRealtime rt;
    while (writable_count == 0 && aio.Poll(true)) {
    }
  }
  EXPECT_GT(writable_count, 0u);

  // Close it and forget it: the callback must never fire again, and the
  // loop must keep polling cleanly with the registration gone.
  const size_t writable_count_before = writable_count;
  pipe.close_write_fd();
  aio.ForgetClosedFd(w_fd);
  {
    ScopedRealtime rt;
    aio.Poll(false);
    aio.Poll(false);
  }
  EXPECT_EQ(writable_count, writable_count_before);
}

// The one test covering the caller-driven repeating pattern end to end --
// which is to say, the pattern ShmTimerHandler uses in production, since
// Aio::Timer is one-shot.  RepeatingTimer (above) is the same three lines
// ShmTimerHandler writes: re-arm from the callback against an absolute
// grid the caller owns.
//
// The property: firing i is due at base + i*period for every i, forever.
// The deadlines are a fixed grid that neither scheduling delay nor a slow
// callback may shift.  Everything else about repeating timers is now
// caller code, so this is the piece Aio is still responsible for.
//
// This is also the property that motivated removing repeating timers from
// Aio altogether.  The io_uring backend used to implement them with
// IORING_TIMEOUT_MULTISHOT, which cannot express an absolute deadline (the
// kernel rejects MULTISHOT|ABS) and so re-armed relatively, from whenever
// task work ran -- losing the wakeup latency every single period, measured
// at 4-10us each and accumulating without bound.  Measured through this
// test against that backend: 1.5ms of accumulated error across 500 firings
// of a 2ms timer.
//
// Note what this test does and does not cover now.  It cannot fail against
// the old backend anymore, because the drifting construct is gone from the
// API: with no interval parameter, the only thing left to schedule is an
// absolute deadline, and absolute deadlines never drifted.  What it guards
// going forward is that Schedule() really honors the absolute deadline it
// is given, and that re-arming from inside the callback -- the pattern
// every periodic consumer now uses -- accumulates nothing.  That is the
// property the whole design now rests on, so it is worth pinning even
// though no current bug can violate it.
//
// The discriminator is the *minimum* error across the final window, not an
// average: scheduling noise can only ever make a firing late, so a
// phase-locked timer is guaranteed some near-exact firing in any window,
// while a drifting one is uniformly late by however much it has
// accumulated.  That makes the check immune to hiccups without needing a
// loose bound.
TEST_P(AioTest, RepeatingTimerHoldsPhaseTest) {
  constexpr auto kPeriod = std::chrono::milliseconds(2);
  constexpr size_t kFirings = 500;
  constexpr size_t kWindow = 50;

  Aio aio;

  size_t count = 0;
  std::vector<aos::monotonic_clock::duration> errors;
  errors.reserve(kFirings);
  const auto base = aos::monotonic_clock::now() + std::chrono::milliseconds(20);

  RepeatingTimer timer(&aio, [&](Completion completion) {
    if (!aos::IsOk(completion.status)) {
      return;
    }
    // Firing i (0-based) is due at base + i*period.  Every elapsed period
    // is delivered in turn, so the index and the grid stay in step even if
    // the loop falls behind.
    errors.push_back(aos::monotonic_clock::now() -
                     (base + kPeriod * static_cast<int64_t>(count)));
    ++count;
  });
  timer.Start(base, kPeriod);

  while (count < kFirings && aio.Poll(true)) {
  }
  timer.Cancel();
  ASSERT_GE(errors.size(), kFirings);

  const auto best_early =
      *std::min_element(errors.begin(), errors.begin() + kWindow);
  const auto best_late =
      *std::min_element(errors.end() - kWindow, errors.end());

  // Every firing is at-or-after its deadline; a timer that fired *early*
  // would mean the grid moved backwards.
  EXPECT_GE(best_early, aos::monotonic_clock::duration::zero());
  EXPECT_GE(best_late, aos::monotonic_clock::duration::zero());

  // The grid must not have moved between the two windows.  Phase-locked
  // this is flat, bounded only by wakeup jitter; anything that reintroduced
  // a relative re-arm would show up here as a gap that grows with the run
  // length.
  EXPECT_LT(best_late - best_early, std::chrono::milliseconds(1))
      << "Best-case lateness grew from " << best_early << " over the first "
      << kWindow << " firings to " << best_late << " over the last " << kWindow
      << " of " << kFirings
      << ".  A repeating timer must not accumulate "
         "phase error.";
}

// Cancel, reschedule, and destruction against a timer whose firing the loop
// has not observed yet.
//
// Historically this was two separate tests and the sharpest pair in the
// file: with the timer living in an IORING_OP_TIMEOUT, the kernel's cancel
// lookup went blind to the target for a window around every firing, so both
// a cancel and a "shape-changing" reschedule landing in that window were
// silently discarded -- leaving a timer that either would not stop or was
// wedged with nothing armed at all.  Both are one timerfd_settime() now and
// cannot miss, which collapses the two into one scenario.
//
// Kept, merged, because the failures it caught were silent, and because a
// single settime() replacing both paths is exactly the kind of claim worth
// holding down.  It still exercises the orphan path: destruction with a
// completion in flight.  A raw sleep with no Poll() in between is what
// leaves the firing unobserved.
TEST_F(AioReproTest, CancelRescheduleAndDestroyPastUnobservedFirings) {
  for (int iter = 0; iter < 50; ++iter) {
    if (iter % 25 == 0) {
      ThrottleOnKernelRingTeardown();
    }
    Aio aio;
    int fire_count = 0;
    RepeatingTimer timer(&aio, [&fire_count](Completion completion) {
      if (aos::IsOk(completion.status)) {
        ++fire_count;
      }
    });
    timer.Start(aos::monotonic_clock::now() + std::chrono::milliseconds(5),
                std::chrono::milliseconds(5));
    while (fire_count < 2 && aio.Poll(true)) {
    }
    ASSERT_GE(fire_count, 2) << "iter " << iter;

    // Let it fire unobserved, then land a reschedule into that churn.  It
    // must take effect rather than being lost or leaving nothing armed.
    std::this_thread::sleep_for(std::chrono::milliseconds(7));
    int single_count = 0;
    Aio::Timer single(&aio);
    single.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(5),
        [](Completion completion, void *ctx) {
          if (aos::IsOk(completion.status)) {
            ++*static_cast<int *>(ctx);
          }
        },
        &single_count);
    while (single_count < 1 && aio.Poll(true)) {
    }
    ASSERT_EQ(single_count, 1) << "iter " << iter;

    // Again unobserved, then cancel.  Both timers' destructors run at the
    // end of this iteration's scope -- the orphan path, with completions
    // possibly still in flight.
    std::this_thread::sleep_for(std::chrono::milliseconds(7));
    timer.Cancel();
  }
}

// Regression test for DestroyTimerState()'s "nothing in flight, free it
// right here" fast path, which used to skip unlinking the request from
// pending_dispatch_.
//
// Three things have to be true at once to reach it.  A timer's multishot
// poll must have been terminated by the kernel (CQ overflow -- see
// TimerPollsSurviveCqOverflow below), which is what sets request.done and
// leaves no cancel ack outstanding, so the fast path applies.  That timer's
// completion must still be sitting on pending_dispatch_, queued by
// DrainCompletions() but not yet dispatched.  And something must destroy it
// in that window -- which an earlier callback in the same dispatch batch
// can legitimately do.  Freeing it there leaves the dispatch loop holding a
// pointer into freed memory, which it walks into as soon as it pops that
// node.
//
// Constructed here by overflowing a tiny CQ with many simultaneous timers
// (producing terminated polls and queued completions in bulk) and giving
// every timer a callback that destroys all the others.  Without the
// unconditional UnlinkPendingDispatch() in DestroyTimerState() this fails
// as a use-after-free under ASAN, or as the intrusive list's own "removing
// a node that is not on this list" CHECK on a normal build.
//
// Note the shape predates the timerfd redesign: the old backend had the
// identical fast path, and reached it far more easily, because there a
// plain fired single-shot timer was `done` while queued.  CancelRequest()
// has always unlinked unconditionally for exactly this reason; the fast
// path just never did.
TEST_F(AioReproTest, DestroyTimerWithTerminatedPollAndQueuedCompletion) {
  constexpr int kTimers = 32;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);  // CQ = 8 slots.
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before constructing timers; each construction arms a
    // poll, and 32 of them would otherwise outrun the 4-entry SQ.
    aio.Poll(false);

    std::vector<std::unique_ptr<Aio::Timer>> timers(kTimers);
    int fired = 0;
    // Destroying is held off until the second round of dispatch.  The first
    // Poll() drains the per-firing (F_MORE) completions, which leaves the
    // terminated polls' *terminal* completions still in the kernel's
    // overflow list; only the round after that carries them, and a
    // terminal is what sets request.done and arms the fast path.
    // Destroying during round one would just orphan everything, which is
    // the safe path and proves nothing.
    bool destroy_enabled = false;
    struct Ctx {
      std::vector<std::unique_ptr<Aio::Timer>> *timers = nullptr;
      int *fired = nullptr;
      const bool *destroy_enabled = nullptr;
      int self = 0;
    };
    std::vector<Ctx> ctxs(kTimers);

    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    for (int i = 0; i < kTimers; ++i) {
      timers[i] = std::make_unique<Aio::Timer>(&aio);
      ctxs[i] = Ctx{&timers, &fired, &destroy_enabled, i};
      timers[i]->Schedule(
          deadline,
          [](Completion, void *raw) {
            auto *ctx = static_cast<Ctx *>(raw);
            ++*ctx->fired;
            if (!*ctx->destroy_enabled) return;
            // Destroy every other timer -- including any whose completion
            // is queued behind this one in the very same dispatch batch.
            for (int j = 0; j < static_cast<int>(ctx->timers->size()); ++j) {
              if (j == ctx->self) continue;
              (*ctx->timers)[j].reset();
            }
          },
          &ctxs[i]);
    }

    // All 32 expire with nothing draining, so the 8-slot CQ overflows and
    // the kernel terminates polls in bulk.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    // Round one: exactly one Poll(), which drains the CQ's worth of
    // per-firing completions and no more.  Draining to empty here would
    // also consume the terminals, and dispatching a terminal re-arms the
    // poll (clearing request.done) -- which is precisely the state the fast
    // path needs, so it must still be pending when destroying starts.
    aio.Poll(true);
    // Round two onward: the terminated polls' terminal completions arrive,
    // several to a batch, and the first callback frees the rest
    // mid-dispatch-loop.
    destroy_enabled = true;
    const auto deadline_stop =
        aos::monotonic_clock::now() + std::chrono::seconds(2);
    while (aos::monotonic_clock::now() < deadline_stop) {
      if (!aio.Poll(false)) break;
    }
    EXPECT_GE(fired, 1);
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// Regression test: the kernel terminates any multishot op whose auxiliary
// CQE cannot be posted -- a full CQ ends the op with a terminal completion
// (io_req_post_cqe() returning false; aux CQEs get no overflow-list
// fallback, unlike ordinary completions).  "Persistent" ops are therefore
// revocable at the kernel's convenience, so every multishot consumer must
// re-arm on an unexpected terminal completion.  Each timer's poll on its
// timerfd is one such consumer: a poll killed by an overflow and never
// re-armed leaves that timer silently dead, armed kernel-side with nobody
// watching.
//
// Forcing the overflow takes many timers rather than one fast one.  A
// single repeating timer cannot overflow anything now: an undrained
// timerfd stays readable without generating a new wait-queue edge, so a
// hundred banked expirations still produce exactly one CQE (and one read()
// reporting all of them).  Independent timers do produce independent CQEs,
// so 24 of them against an 8-slot CQ, left undrained, overflows it.  Every
// timer must still be firing afterward.
TEST_F(AioReproTest, TimerPollsSurviveCqOverflow) {
  constexpr int kTimers = 24;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before building the timers.  Each one arms a poll at
    // construction, and MaybeSubmit() cannot flush those until the ring is
    // enabled -- so without this, 24 constructions would queue 24 SQEs
    // against a 4-entry submission queue and CHECK-fail before the test got
    // anywhere near the CQ.
    aio.Poll(false);
    std::vector<int> fire_counts(kTimers, 0);
    std::vector<std::unique_ptr<RepeatingTimer>> timers;
    for (int i = 0; i < kTimers; ++i) {
      timers.push_back(std::make_unique<RepeatingTimer>(
          &aio, [&fire_counts, i](Completion completion) {
            if (aos::IsOk(completion.status)) {
              ++fire_counts[i];
            }
          }));
      // Staggered so their CQEs arrive spread out rather than as one batch
      // the ring might absorb between polls.
      timers.back()->Start(
          aos::monotonic_clock::now() + std::chrono::milliseconds(5 + i % 5),
          std::chrono::milliseconds(5));
    }

    // Overflow the 8-slot CQ: 24 timers firing every 5ms for 100ms, with
    // nothing draining.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Drain the backlog, then require every timer to fire well past
    // anything the pre-termination backlog alone could deliver.  A timer
    // whose poll died in the overflow stalls the loop here instead (caught
    // by the watchdog).
    while (aio.Poll(false)) {
    }
    std::vector<int> target(kTimers);
    for (int i = 0; i < kTimers; ++i) {
      target[i] = fire_counts[i] + 5;
    }
    const auto reached_targets = [&]() {
      for (int i = 0; i < kTimers; ++i) {
        if (fire_counts[i] < target[i]) return false;
      }
      return true;
    };
    while (!reached_targets() && aio.Poll(true)) {
    }
    for (int i = 0; i < kTimers; ++i) {
      EXPECT_GE(fire_counts[i], target[i])
          << "timer " << i << " stopped firing after the CQ overflow";
    }

    for (auto &timer : timers) {
      timer->Cancel();
    }
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// Queue exhaustion must be predictable and front-loaded, not a bare "Out
// of SQEs" somewhere downstream: nothing can flush the submission queue
// before the first Run()/Poll() enables the ring, so every op armed before
// then holds a staged SQE, and exceeding --aio_queue_depth there is
// deterministic at the arming call.  ArmSqe()'s message must name the flag
// and the cause.  (The wakeup read and the legacy-epoll poll hold two of
// the four slots; the third timer's poll is the one that cannot stage.)
TEST_F(AioReproTest, PreRunArmingExceedsQueueDepthDeathTest) {
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Aio::Timer t1(&aio);
        Aio::Timer t2(&aio);
        Aio::Timer t3(&aio);
      },
      "aio_queue_depth");
}

// Regression test for UnregisterThreadSignalReceiver()'s "nothing in
// flight, free it right here" fast path, which used to skip unlinking the
// request from pending_dispatch_ -- the receiver twin of
// DestroyTimerWithTerminatedPollAndQueuedCompletion above, with the same
// three-way setup: the receiver's multishot poll terminated by CQ overflow
// (request.done set, no cancel acks outstanding, so the fast path applies),
// its terminal completion queued by DrainCompletions() but not yet
// dispatched, and an earlier callback in the same dispatch batch
// unregistering the receiver.  Freeing there leaves the dispatch loop
// holding a pointer into freed memory.  A regression fails as a
// use-after-free under ASAN, or as the intrusive list's own CHECK on a
// normal build.
TEST_F(AioReproTest, UnregisterReceiverWithTerminatedPollAndQueuedCompletion) {
  // 28 timers against an 8-slot CQ: 8 fit as auxiliary CQEs, the other 20
  // terminate their polls, and those 20 terminals plus the receiver's --
  // fired last, so ordered last -- land on the kernel's overflow list, 21
  // entries total.  The overflow refills the freed CQ 8 at a time, so the
  // final refill batch is [4 timer terminals, receiver terminal]: exactly
  // the shape where DrainCompletions() queues the receiver's terminal
  // (setting request.done) *behind* user-visible work in one batch.
  constexpr int kTimers = 28;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);  // CQ = 8 slots.
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before arming everything -- see the sibling tests.
    aio.Poll(false);

    aos::ipc_lib::ThreadSignalReceiver sfd;
    aio.RegisterThreadSignalReceiver(&sfd, []() {});

    int fired = 0;
    std::vector<std::unique_ptr<Aio::Timer>> timers(kTimers);
    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    for (int i = 0; i < kTimers; ++i) {
      timers[i] = std::make_unique<Aio::Timer>(&aio);
      timers[i]->Schedule(
          deadline, [](Completion, void *raw) { ++*static_cast<int *>(raw); },
          &fired);
    }

    // All 28 timers expire with nothing draining.  Only then fire the
    // receiver's poll, so its completion is ordered after every timer's.
    // (Under DEFER_TASKRUN nothing posts until the next Poll() enters the
    // kernel; the CQ overflows there, in wake order.)
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Poll() dispatches one user-visible completion (one timer callback)
    // per call.  The 25th is the first entry of that final refill batch --
    // when it has been dispatched, the receiver's terminal has been drained
    // (request.done set, no cancel acks outstanding: the fast path's exact
    // condition) but is still sitting queued on pending_dispatch_.
    while (fired < 25 && aio.Poll(true)) {
    }
    // The exact count is derived from the kernel's CQ-overflow refill
    // batching; a kernel that batches differently lands `fired` elsewhere.
    // EXPECT, not ASSERT: an early return here would leave the receiver
    // registered and turn a batching drift into ~IoUringImpl()'s
    // still-registered CHECK -- an abort -- instead of a clean failure.
    // The unregister below is safe in any state; off the exact window the
    // use-after-free assertion just degrades to a smoke test.
    EXPECT_EQ(fired, 25);

    // Unregister in that window.  The fast path frees the state here; if it
    // fails to unlink the queued terminal first, the dispatch loop below
    // walks into the freed node.
    aio.UnregisterThreadSignalReceiver(&sfd);

    const auto deadline_stop =
        aos::monotonic_clock::now() + std::chrono::seconds(2);
    while (aos::monotonic_clock::now() < deadline_stop) {
      if (!aio.Poll(false)) break;
    }
    EXPECT_EQ(fired, kTimers);
    // The unregistered receiver never read the signal; don't leave it
    // pending.
    sfd.ConsumeWakeup();
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// Regression test: unregistering the receiver from inside its own
// dispatched callback.  The receiver trampoline keeps using its state
// *after* invoking the user callback -- the signalfd drain loop continues,
// then the revival check runs -- so UnregisterThreadSignalReceiver()'s
// nothing-in-flight fast path must not free the state mid-dispatch; it
// parks it as a drained orphan instead, recycled at the end of the
// outermost dispatch after every frame has unwound.  The fast path is only
// reachable mid-dispatch via a terminal completion (a live multishot is
// never `done`), so the CQ-overflow shape from the previous test forces
// one, with a signal pending so the trampoline actually runs the user
// callback.  A regression is a read of freed memory in the trampoline's
// drain loop, which needs ASAN to fail reliably.
TEST_F(AioReproTest, UnregisterReceiverFromOwnCallbackDuringTerminalDispatch) {
  constexpr int kTimers = 28;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);  // CQ = 8 slots.
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before arming everything -- see the sibling tests.
    aio.Poll(false);

    aos::ipc_lib::ThreadSignalReceiver sfd;
    bool unregistered = false;
    aio.RegisterThreadSignalReceiver(&sfd, [&aio, &sfd, &unregistered]() {
      if (unregistered) return;
      unregistered = true;
      aio.UnregisterThreadSignalReceiver(&sfd);
    });

    std::vector<std::unique_ptr<Aio::Timer>> timers(kTimers);
    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    for (int i = 0; i < kTimers; ++i) {
      timers[i] = std::make_unique<Aio::Timer>(&aio);
      timers[i]->Schedule(deadline, [](Completion, void *) {}, nullptr);
    }

    // Overflow the CQ with the timer firings, then wake the receiver's
    // poll against the full CQ so the kernel terminates the multishot --
    // its terminal completion is what makes the in-callback unregister
    // take the fast path.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const auto deadline_stop =
        aos::monotonic_clock::now() + std::chrono::seconds(2);
    while (aos::monotonic_clock::now() < deadline_stop) {
      if (!aio.Poll(false)) break;
    }
    EXPECT_TRUE(unregistered);
    if (!unregistered) {
      // Kernel batching drift kept the terminal dispatch (and so the
      // in-callback unregister) from happening: clean up so the failure
      // stays a clean EXPECT instead of tripping ~IoUringImpl()'s
      // still-registered CHECK.
      aio.UnregisterThreadSignalReceiver(&sfd);
    }
    sfd.ConsumeWakeup();
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// aio.h's constraint 2 lets a caller destroy the Aio with requests still
// pending -- including one whose completion was already drained and queued
// but never dispatched.  That AsyncRequest outlives the Aio and is legal
// to reuse with another one; stale internal dispatch-queue state would
// make the second Aio silently drop its callback forever.  Two readable
// AsyncReads and a single Poll() construct the queued-but-undispatched
// state deterministically on io_uring: the drain resolves both
// completions, the one-user-visible budget delivers only one.  (On epoll
// the second request is simply still pending at destruction -- the same
// contract, a different internal state.)
TEST_P(AioTest, ReuseRequestPendingAtDestruction) {
  // The regression mode is a silently-dropped callback: the poll loop below
  // then blocks forever, and the watchdog turns that into a clean death.
  ScopedDeathTestWatchdog watchdog;
  Pipe pipe_a;
  Pipe pipe_b;
  pipe_a.Write("a");
  pipe_b.Write("b");

  bool fired_a = false;
  bool fired_b = false;
  AsyncRequest request_a;
  AsyncRequest request_b;
  request_a.callback = [](Completion, void *ctx) {
    *static_cast<bool *>(ctx) = true;
  };
  request_a.context = &fired_a;
  request_b.callback = request_a.callback;
  request_b.context = &fired_b;

  char buf_a[8];
  char buf_b[8];
  {
    Aio aio;
    aio.AsyncRead(pipe_a.read_fd(), buf_a, &request_a);
    aio.AsyncRead(pipe_b.read_fd(), buf_b, &request_b);
    aio.Poll(true);
  }
  // At most one callback ran; at least one request is left over.
  ASSERT_FALSE(fired_a && fired_b);

  // Reuse a leftover request with a fresh Aio: its callback must fire.
  AsyncRequest *reuse = fired_a ? &request_b : &request_a;
  Pipe *reuse_pipe = fired_a ? &pipe_b : &pipe_a;
  bool *reuse_fired = fired_a ? &fired_b : &fired_a;
  // The first Aio may have consumed the byte before being destroyed; make
  // the fd readable again either way.
  reuse_pipe->Write("x");

  Aio aio2;
  char buf2[8];
  aio2.AsyncRead(reuse_pipe->read_fd(), buf2, reuse);
  while (!*reuse_fired && aio2.Poll(true)) {
  }
  EXPECT_TRUE(*reuse_fired);
}

// The construct-here/run-there downgrade rebuilds the ring and re-arms
// every persistent registration -- including the multishot poll each live
// timer holds on its timerfd (ReArmPersistentRegistrations()'s timer loop,
// via GetSqeForRingReconstruction()).  No other downgrade test carries a
// live timer, so this is that path's only coverage; a rebuild that loses a
// timer's poll is the same silent never-fires-again failure
// TimerPollsSurviveCqOverflow pins on the overflow-termination path.
TEST_P(AioTest, DowngradeReArmsActiveTimers) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "The SINGLE_ISSUER downgrade is io_uring-specific.";
  }
  ScopedDeathTestWatchdog watchdog;

  auto aio = std::make_unique<Aio>();
  constexpr int kTimers = 3;
  std::vector<std::unique_ptr<Aio::Timer>> timers;
  int fired = 0;
  const auto deadline =
      aos::monotonic_clock::now() + std::chrono::milliseconds(30);
  for (int i = 0; i < kTimers; ++i) {
    timers.push_back(std::make_unique<Aio::Timer>(aio.get()));
    timers.back()->Schedule(
        deadline, [](Completion, void *ctx) { ++*static_cast<int *>(ctx); },
        &fired);
  }

  // First drive from a different thread: EnsureBound() sees the
  // construction-thread mismatch, downgrades, and must re-arm all three
  // timer polls on the rebuilt ring for them to ever fire.
  std::thread driver([&aio, &fired]() {
    while (fired < kTimers && aio->Poll(true)) {
    }
  });
  driver.join();
  EXPECT_EQ(fired, kTimers);
}

#if defined(__linux__)
// A Timer must be destroyed before its Aio: ~Timer dereferences the Aio's
// impl, so a Timer outliving its Aio is a use-after-free.  The destructor
// CHECKs active timers so the bug dies loudly at the Aio instead.
TEST_P(AioTest, TimerOutlivesAioDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        auto aio = std::make_unique<Aio>();
        Aio::Timer timer(aio.get());
        aio.reset();
      },
      "destroyed before its Aio");
}

// aio.h documents "All Fds must be cleaned up before this class is
// destroyed"; EPoll::~EPoll() has always CHECKed it.  So does Aio now.
TEST_P(AioTest, DestroyWithFdRegisteredDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Pipe pipe;
        Aio aio;
        aio.OnReadable(pipe.read_fd(), []() {});
      },
      "before destroying the Aio");
}

TEST_P(AioTest, DestroyWithReceiverRegisteredDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        aos::ipc_lib::ThreadSignalReceiver sfd;
        Aio aio;
        aio.RegisterThreadSignalReceiver(&sfd, []() {});
      },
      "unregistered before destroying");
}

// Completion::user_data is documented as "the opaque pointer supplied by
// the caller", and Timer::Schedule() takes no user_data -- so a timer
// completion must carry nullptr, not some internal state pointer.
TEST_P(AioTest, TimerCompletionUserDataIsNull) {
  Aio aio;
  Aio::Timer timer(&aio);

  struct Result {
    bool fired = false;
    void *user_data;
  } result;
  result.user_data = &result;  // Anything non-null.
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion completion, void *ctx) {
        auto *r = static_cast<Result *>(ctx);
        r->fired = true;
        r->user_data = completion.user_data;
      },
      &result);
  while (!result.fired && aio.Poll(true)) {
  }
  EXPECT_TRUE(result.fired);
  EXPECT_EQ(result.user_data, nullptr);
}

// A SetEvents() mask made only of bits the epoll translation drops (like
// EPOLLHUP, which the kernel always reports and never accepts in a mask)
// must keep the fd registered, exactly as EPoll::DoEpollCtl() keyed its
// add-vs-remove decision on the caller's untranslated mask.  Keying it on
// the translated mask unregistered the fd entirely, so the hangup below
// would never be delivered.
TEST_P(AioTest, SetEventsUntranslatedMaskKeepsRegistration) {
  Aio aio;
  Pipe pipe;

  int events_seen = 0;
  aio.OnEvents(pipe.read_fd(), [&aio, &pipe, &events_seen](uint32_t events) {
    EXPECT_TRUE(events & EPOLLERR);
    ++events_seen;
    // Stop the level-triggered hangup from re-firing.
    aio.DeleteFd(pipe.read_fd());
  });
  aio.SetEvents(pipe.read_fd(), EPOLLHUP);

  pipe.close_write_fd();
  // Non-blocking with a deadline: the regression mode is "the fd got
  // unregistered, no event will ever arrive", which must fail the EXPECT
  // below rather than park forever in a blocking Poll().
  const auto deadline_stop =
      aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (events_seen == 0 && aos::monotonic_clock::now() < deadline_stop) {
    aio.Poll(false);
  }
  EXPECT_EQ(events_seen, 1);
}

// Pins the coalescing contract from Aio::RegisterThreadSignalReceiver()'s
// docs: every pending wakeup is consumed first, then the callback runs
// exactly once.  kWakeupSignal is a realtime signal, so the three sends
// below genuinely queue three pending siginfos; a per-signal dispatch
// would invoke the callback three times.
TEST_P(AioTest, PendingWakeupsCoalesceIntoOneCallback) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  // Bind and enable the loop before queueing the signals, so delivery
  // happens through the armed receiver rather than at registration time.
  aio.Poll(false);
  int count = 0;
  aio.RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });

  for (int i = 0; i < 3; ++i) {
    pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  }

  while (count == 0 && aio.Poll(true)) {
  }
  // Drain any further deliveries the backend queued for the same burst.
  const auto deadline_stop =
      aos::monotonic_clock::now() + std::chrono::milliseconds(200);
  while (aos::monotonic_clock::now() < deadline_stop) {
    if (!aio.Poll(false)) break;
  }
  EXPECT_EQ(count, 1);
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// Pins the contract from Aio::RegisterThreadSignalReceiver()'s docs: once
// UnregisterThreadSignalReceiver() returns, the signalfd is the caller's
// again -- a wakeup that was still kernel-side at unregister time is
// discarded without reading the fd, so a successor receiver registered on
// the same fd owns every pending signal.
TEST_P(AioTest, SuccessorReceiverOwnsPendingWakeups) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  // Enable the loop, then arm a wakeup while nothing polls: the first
  // receiver's kernel-side poll fires, but its completion is never
  // dispatched before the unregister.
  aio.Poll(false);
  aio.RegisterThreadSignalReceiver(&sfd, []() {});
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  aio.UnregisterThreadSignalReceiver(&sfd);

  int count = 0;
  aio.RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });
  const auto deadline_stop =
      aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (count == 0 && aos::monotonic_clock::now() < deadline_stop) {
    aio.Poll(false);
  }
  EXPECT_EQ(count, 1) << "the unregistered receiver consumed the wakeup";
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// A receiver callback that unregisters its own receiver keeps executing
// after UnregisterThreadSignalReceiver() returns -- so the std::function
// (and its captures) must not be destroyed out from under it.  The old
// implementation assigned nullptr to the executing function; with a
// heap-allocated capture this is a use-after-free ASAN catches.
TEST_P(AioTest, ReceiverCallbackCapturesSurviveSelfUnregister) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  const std::string canary(64, 'x');  // Big enough to defeat SSO.
  bool checked = false;
  aio.RegisterThreadSignalReceiver(&sfd, [&aio, &sfd, canary, &checked]() {
    if (checked) return;
    checked = true;
    aio.UnregisterThreadSignalReceiver(&sfd);
    // The capture must still be alive after the unregister.
    EXPECT_EQ(canary, std::string(64, 'x'));
  });

  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  while (!checked && aio.Poll(true)) {
  }
  EXPECT_TRUE(checked);
}

// A callback that deletes its own fd, closes it, and registers a fresh fd
// that reuses the same number -- all within one firing -- must not have
// the old event's remaining bits dispatched into the new registration.
// The dispatch holds the original registration and stops on its fd = -1
// tombstone; it never consults the fd number again, so reuse cannot
// mislead it.
TEST_P(AioTest, StaleEventBitsDoNotReachReusedFdNumber) {
  Aio aio;

  int fds[2];
  ABSL_PCHECK(pipe(fds) == 0);
  ABSL_PCHECK(fcntl(fds[0], F_SETFL, O_NONBLOCK) == 0);
  int new_fds[2] = {-1, -1};
  bool fired = false;
  int spurious_new_errors = 0;

  aio.OnReadable(fds[0], [&]() {
    if (fired) return;
    fired = true;
    char buf[8];
    ABSL_PCHECK(read(fds[0], buf, sizeof(buf)) >= 0);
    aio.DeleteFd(fds[0]);
    ABSL_PCHECK(close(fds[0]) == 0);
    // POSIX hands out the lowest free descriptor: the new pipe's read end
    // reuses the number we just closed.
    ABSL_PCHECK(pipe(new_fds) == 0);
    ABSL_CHECK_EQ(new_fds[0], fds[0]);
    aio.OnError(new_fds[0],
                [&spurious_new_errors]() { ++spurious_new_errors; });
  });

  // Data plus a closed write end: the firing carries readable and error
  // bits together, and only the readable one belongs to the callback
  // above -- the error bit must die with the old registration.
  ABSL_PCHECK(write(fds[1], "x", 1) == 1);
  ABSL_PCHECK(close(fds[1]) == 0);

  for (int i = 0; i < 100 && !fired; ++i) {
    aio.Poll(true);
  }
  EXPECT_TRUE(fired);
  aio.Poll(false);
  EXPECT_EQ(spurious_new_errors, 0)
      << "stale event bits reached the reused fd number's new registration";

  aio.DeleteFd(new_fds[0]);
  ABSL_PCHECK(close(new_fds[0]) == 0);
  ABSL_PCHECK(close(new_fds[1]) == 0);
}
#endif  // defined(__linux__)

// Helper function to poll Aio for a given duration.
void RunAioFor(Aio &aio, std::chrono::nanoseconds duration) {
  Aio::Timer timer(&aio);
  bool done = false;
  {
    ScopedRealtime rt;
    timer.Schedule(
        aos::monotonic_clock::now() + duration,
        [](Completion, void *ctx) { *static_cast<bool *>(ctx) = true; }, &done);
    while (!done && aio.Poll(true)) {
    }
  }
}

// Helper function to fill up a pipe using OnWritable callbacks.
// It runs the event loop until the pipe stops reporting its write end as
// ready, i.e. its buffer is full.
void FillPipe(Aio &aio, Pipe &pipe) {
  while (pipe.write_ready()) {
    ScopedRealtime rt;
    aio.Poll(true);
  }
}

// Test that the basics of OnWritable work, filling the pipe's buffer.
TEST_P(AioTest, EPollLikeBasicWritable) {
  Aio aio;
  Pipe pipe;
  int number_writes = 0;
  aio.OnWritable(pipe.write_fd(), [&]() {
    pipe.Write(" ");
    ++number_writes;
  });

  // First, fill up the pipe's write buffer.
  FillPipe(aio, pipe);
  EXPECT_GT(number_writes, 0);

  // Now, if we try again, we shouldn't do anything because buffer is full.
  const int bytes_in_pipe = number_writes;
  number_writes = 0;
  FillPipe(aio, pipe);
  EXPECT_EQ(number_writes, 0);

  // Empty the pipe, then fill it up again.
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }
  number_writes = 0;
  FillPipe(aio, pipe);
#if defined(_WIN32)
  // Unlike a real pipe, this "pipe" is a loopback TCP socket pair, and the
  // usable send window after a full drain-and-refill cycle doesn't
  // reliably reopen to the exact same byte count it started at (this is a
  // property of the TCP stack's buffer/window bookkeeping, not of Aio) --
  // so just check that a substantial refill happened rather than requiring
  // exact parity with the original fill.
  EXPECT_GT(number_writes, bytes_in_pipe / 2);
#else
  EXPECT_EQ(number_writes, bytes_in_pipe);
#endif

  aio.DeleteFd(pipe.write_fd());
}

// Test that the basics of OnError work by closing the read end.
TEST_P(AioTest, EPollLikeBasicError) {
  Aio aio;
  Pipe pipe;
  int number_errors = 0;
  aio.OnError(pipe.write_fd(), [&]() { ++number_errors; });

  // Sanity check that we don't get any errors before anything has happened.
  RunAioFor(aio, std::chrono::milliseconds(50));
  EXPECT_EQ(number_errors, 0);

  pipe.close_read_fd();

  // Poll once to consume the hangup/error.
  {
    ScopedRealtime rt;
    aio.Poll(false);
  }

  EXPECT_EQ(number_errors, 1);

  aio.DeleteFd(pipe.write_fd());
}

// Tests that removing an event before scheduling any events works.
TEST_P(AioTest, EPollLikeRemoveWithoutEvents) {
  Aio aio;
  Pipe pipe;
  aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
  aio.DeleteFd(pipe.read_fd());
}

// Tests that a callback deleting its own fd doesn't leave the dispatch code
// executing out of a freed registration.
//
// DeleteFd() from inside one of the fd's own callbacks means one of that
// LegacyState's std::functions is the code currently executing -- freeing it
// inline would free the lambda's captures out from under the running
// callback -- the state must stay allocated, not merely tombstoned.  So
// DeleteFd() parks the state on retired_legacy_states_ instead, and it is
// freed at the end of that same outermost dispatch, after every callback
// has returned.  A regression is a read of freed memory, which needs ASAN
// to fail reliably; without it this test can pass while still being wrong.
//
// Deleting an fd from inside its own callback is an ordinary path -- a timer
// callback that destroys its timer lands in ~TimerState -> DeleteFd().
TEST_P(AioTest, DeleteFdFromOwnCallback) {
  Aio aio;

  Pipe pipe;
  bool fired = false;
  aio.OnReadable(pipe.read_fd(), [&aio, &pipe, &fired]() {
    fired = true;
    aio.DeleteFd(pipe.read_fd());
  });
  pipe.Write("x");

  aio.Poll(true);
  EXPECT_TRUE(fired);

  // Nothing left registered; a further Poll() finds no events.
  aio.Poll(false);
}

// Regression test: a bare EPOLLHUP must reach the readable handler when no
// error handler is registered.  After a pipe's write end closes and its
// remaining data is drained, the kernel reports plain EPOLLHUP with no
// EPOLLIN -- level-triggered state that cannot be consumed.  This used to
// dispatch nothing (EPOLLHUP folds into the error bit, and with only
// OnReadable registered there was no eligible handler) while the legacy
// poll re-armed unconditionally, so blocking Poll(true) returned true
// forever without ever running a callback: a silent busy loop.
// DrainLegacyEpoll() now routes the error bit to in_fn, whose read()
// observes EOF and can DeleteFd() -- which is what actually ends the
// re-firing.
TEST_P(AioTest, LegacyReadableSeesHangup) {
  Aio aio;
  Pipe pipe;

  bool saw_eof = false;
  aio.OnReadable(pipe.read_fd(), [&aio, &pipe, &saw_eof]() {
    char buf[16];
    const ssize_t n = read(pipe.read_fd(), buf, sizeof(buf));
    if (n == 0) {
      saw_eof = true;
      aio.DeleteFd(pipe.read_fd());
    } else {
      ABSL_PCHECK(n > 0) << "read failed";
    }
  });

  pipe.Write("x");
  pipe.close_write_fd();

  // First firing: EPOLLIN|EPOLLHUP, the callback reads the byte.  Second
  // firing: bare EPOLLHUP, the callback's read() returns 0 and deletes the
  // fd.  Bounded so a regression fails the EXPECT below instead of spinning
  // in Poll(true) forever.
  for (int i = 0; i < 100 && !saw_eof; ++i) {
    aio.Poll(true);
  }
  EXPECT_TRUE(saw_eof);
}

// Registering a before-wait function from inside one is disallowed: the
// push_back could reallocate the vector out from under the executing
// std::function.  Pinned as a CHECK rather than left as silent UB (which
// is what EPoll's range-for did).
TEST_P(AioTest, BeforeWaitFromBeforeWaitDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        aio.BeforeWait([&aio]() {
          aio.BeforeWait([]() {});
          aio.Quit();
        });
        aio.Run();
      },
      "may not be called from a before-wait function");
}

// Tests that calling Quit from a BeforeWait callback successfully stops the
// loop.
TEST_P(AioTest, QuitInBeforeWait) {
  Aio aio;
  aio.BeforeWait([&aio]() { aio.Quit(); });
  aio.Run();
}

// Tests that Run() flushes what is already queued before returning from a
// Quit().
//
// EPoll::Run() polled with a zero timeout once Quit() landed, so events already
// sitting in the queue still ran before Run() returned ("This lets us flush the
// event queue before quitting", 6b6dfa5a9).  Wrapping EPoll around Aio dropped
// that, and nothing caught it because no test had ever pinned it -- so pin it.
//
// The ordering here is deliberate: only `trigger` is readable when Run()
// starts, so its callback is guaranteed to run first, and it is that callback
// which both makes `target` readable and asks to quit.  A non-draining Run()
// leaves target_count at 0.
TEST_P(AioTest, RunDrainsQueuedEventsAfterQuit) {
  Aio aio;
  Pipe trigger;
  Pipe target;

  int target_count = 0;
  aio.OnReadable(trigger.read_fd(), [&aio, &trigger, &target]() {
    EXPECT_EQ(trigger.Read(1), "x");
    // Queue up work and ask to stop in the same breath.
    target.Write("x");
    aio.Quit();
  });
  aio.OnReadable(target.read_fd(), [&target, &target_count]() {
    EXPECT_EQ(target.Read(1), "x");
    ++target_count;
  });

  trigger.Write("x");
  aio.Run();

  EXPECT_EQ(target_count, 1)
      << "Run() returned without draining the event queue; work that was "
         "already pending when Quit() was called got dropped.";

  aio.DeleteFd(trigger.read_fd());
  aio.DeleteFd(target.read_fd());
}

// Tests that a Quit() concurrent with Run() startup always stops the loop, and
// that the loop is left stopped afterwards.
//
// Quit() is documented async-safe, so it may be called from another thread (or
// a signal handler) at any point relative to Run().  This covers the two
// interleavings that are reachable in practice: Quit() landing entirely before
// Run() starts, and Quit() landing once Run() is already blocked in Poll().
//
// It does NOT reliably reach the narrowest interleaving, where Quit() lands
// between Run() reading quit_requested_ and Run() storing to run_ -- that
// window is a couple of instructions wide and did not reproduce here even with
// a barrier and 2000 attempts.  Run()'s loop consults should_run() rather than
// run_ alone precisely so that interleaving stays harmless: the store would
// clobber Quit()'s `run_ = false`, leaving quit_requested_ as the only record
// that a shutdown was asked for.  A regression there would hang rather than
// fail an assertion.
TEST_P(AioTest, QuitRacingWithRunStartup) {
  // 200 iterations, deliberately not more: every fresh Aio is a kernel ring
  // whose teardown costs at least one RCU grace period on a workqueue capped
  // at 64 concurrent teardowns (io_ring_exit_work + percpu_ref_kill's
  // call_rcu -- see the ADR's teardown-throughput section), regardless of
  // SINGLE_ISSUER.  This loop is the suite's dominant ring creator; at an
  // earlier 2000 iterations, massively parallel --runs_per_test invocations
  // outran kernel teardown fleet-wide and accumulated tens of GB of
  // unreclaimable slab per node.  Interleaving coverage lives in the
  // barrier below, not the count -- and at stress-run scale the count
  // multiplies out anyway (200 x 10000 runs = 2M interleavings).
  for (int i = 0; i < 200; ++i) {
    if (i % 50 == 0) {
      ThrottleOnKernelRingTeardown();
    }
    Aio aio;
    // Spin both threads up against a barrier so Quit() and Run() start
    // together; just spawning the thread lets Run() reach Poll() first every
    // time, which is the easy interleaving and not the interesting one.
    std::atomic<bool> go{false};
    std::thread quitter([&aio, &go]() {
      while (!go.load(std::memory_order_acquire)) {
      }
      aio.Quit();
    });
    go.store(true, std::memory_order_release);
    aio.Run();
    quitter.join();
    EXPECT_FALSE(aio.should_run());
  }
}

// Tests that unregistering a signalfd correctly cancels the underlying
// multishot poll request and prevents any use-after-free or extra callbacks.
TEST_P(AioTest, UnregisterThreadSignalReceiverTest) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  int count = 0;
  aio.RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();
  aos::ipc_lib::ThreadSignalSender signaler_signal;

  // Send kWakeupSignal.
  signaler_signal.Signal(pid, tid);

  // Poll until the signal is handled.
  while (count == 0 && aio.Poll(true)) {
  }
  EXPECT_EQ(count, 1);

  // Unregister the signalfd.
  aio.UnregisterThreadSignalReceiver(&sfd);

  // Send the signal again.
  signaler_signal.Signal(pid, tid);

  // Clean up signal so it doesn't stay pending.
#if defined(__linux__)
  struct signalfd_siginfo siginfo;
  ssize_t res = read(sfd.fd(), &siginfo, sizeof(siginfo));
  EXPECT_EQ(res, sizeof(siginfo));
#endif

  // Poll non-blockingly a few times to ensure the callback is NOT invoked.
  for (int i = 0; i < 5; ++i) {
    aio.Poll(false);
  }
  EXPECT_EQ(count, 1);
}

TEST_P(AioTest, DuplicateEventOnCancel) {
  // Test that clearing events on an active file descriptor from its callback
  // does not result in duplicate events due to recursive cancel handling.
  Aio aio;
  Pipe pipe;
  int count = 0;

  aio.OnEvents(pipe.write_fd(), [&](uint32_t events) {
    EXPECT_TRUE(events & EPOLLOUT);
    ++count;
    aio.SetEvents(pipe.write_fd(), 0);
  });

  aio.SetEvents(pipe.write_fd(), EPOLLOUT);
  aio.Poll(true);
  aio.Poll(false);

  EXPECT_EQ(count, 1);

  aio.DeleteFd(pipe.write_fd());
}

TEST_P(AioTest, ForkDeathTest) {
  // Test that an Aio instance constructed in the parent process remains
  // fully functional inside a forked child process by successfully detecting
  // the fork and recreating the io_uring ring.  We register both a standard
  // file descriptor event and a SignalFd to exercise all re-registration loops
  // in HandleFork.
  //
  // On Windows there is no fork: the death-test child re-executes the binary
  // and rebuilds all of this state from scratch.  That exercises less (no
  // inherited state to recover), but it is exactly what every death test in the
  // tree relies on, so run it there too to pin down that death tests work with
  // a live event loop.
  Aio aio;
  Pipe pipe;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  int signal_count = 0;
  int fd_count = 0;

  aio.RegisterThreadSignalReceiver(&sfd, [&]() { ++signal_count; });

  aio.OnEvents(pipe.write_fd(), [&](uint32_t events) {
    EXPECT_TRUE(events & EPOLLOUT);
    ++fd_count;
    aio.SetEvents(pipe.write_fd(), 0);
  });

  aio.SetEvents(pipe.write_fd(), EPOLLOUT);

  EXPECT_EXIT(
      {
        const auto pid = aos::GetProcessId();
        const auto tid = aos::GetThreadId();
        aos::ipc_lib::ThreadSignalSender signaler_signal;
        signaler_signal.Signal(pid, tid);
        while ((signal_count == 0 || fd_count == 0) && aio.Poll(true)) {
        }
        if (signal_count == 1 && fd_count == 1) {
          exit(42);
        }
        exit(1);
      },
      ::testing::ExitedWithCode(42), "");

  aio.UnregisterThreadSignalReceiver(&sfd);
  aio.DeleteFd(pipe.write_fd());
}

// Regression test for detecting a fork too late.  Fork detection used to run
// only inside Poll(): an operation issued by a forked child *before* its first
// Poll() went into the stale inherited ring/epoll and was silently discarded
// when Poll() finally rebuilt the backend, so the request never completed and
// Poll() blocked forever.
//
// This one really is about inherited state, which only exists where death tests
// fork -- on Windows the death-test child starts from a fresh process, so there
// is nothing stale to detect (and no alarm() to watchdog the hang with).
#ifndef _WIN32
//
// The child here issues an AsyncRead and makes data available -- all before the
// first Poll() -- and the read must complete.  If the backend isn't rebuilt
// eagerly the read is lost and Poll() hangs; the alarm() then kills us so the
// ExitedWithCode(42) check fails loudly instead of wedging the whole suite.
TEST_P(AioTest, ForkOperationBeforePollDeathTest) {
  Aio aio;
  Pipe pipe;

  EXPECT_EXIT(
      {
        alarm(30);

        AsyncRequest read_req;
        read_req.callback = [](Completion completion, void *) {
          EXPECT_TRUE(aos::IsOk(completion.status));
        };

        char read_buf[8] = {0};
        aio.AsyncRead(pipe.read_fd(), read_buf, &read_req);

        const char msg = 'x';
        PCHECK(write(pipe.write_fd(), &msg, 1) == 1);

        while (!read_req.done && aio.Poll(true)) {
        }
        exit(read_req.done ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}
#endif  // !_WIN32

TEST_P(AioTest, ShouldRunTest) {
  Aio aio;
  // should_run() starts true (matching the original EPoll's run_{true}), before
  // any Run() or Quit() has happened.
  EXPECT_TRUE(aio.should_run());

  // Set a timer to check should_run() while running and then quit.
  Aio::Timer timer(&aio);
  struct Context {
    Aio *aio;
    bool checked_running;
  };
  Context context{&aio, false};

  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) {
        auto *c = static_cast<Context *>(ctx);
        EXPECT_TRUE(c->aio->should_run());
        c->checked_running = true;
        c->aio->Quit();
        EXPECT_FALSE(c->aio->should_run());
      },
      &context);

  aio.Run();
  EXPECT_FALSE(aio.should_run());
  EXPECT_TRUE(context.checked_running);
}

// Quit() before Run() is remembered: Run() returns without entering the loop
// instead of blocking forever, and consumes the request rather than leaving it
// to strand the *next* Run().
//
// This is the exact shape that broke EPoll when it was converted to wrap Aio:
// it kept a run_ flag of its own, set it in Run(), never cleared it on exit,
// and the impl's early return swallowed the quit -- so should_run() answered
// true forever afterward.  Nothing pinned the behavior at this level, which is
// why that went unnoticed; it is pinned here now.
TEST_P(AioTest, QuitBeforeRunTest) {
  Aio aio;
  EXPECT_TRUE(aio.should_run());

  aio.Quit();
  EXPECT_FALSE(aio.should_run());

  // Returns rather than blocking: the loop body never executes.
  aio.Run();
  EXPECT_FALSE(aio.should_run());

  // ...and the request was consumed, not left pending.  A stranded quit shows
  // up here, as a second Run() that returns immediately without servicing
  // anything.
  Aio::Timer timer(&aio);
  bool fired = false;
  struct Context {
    Aio *aio;
    bool *fired;
  } context{&aio, &fired};
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) {
        auto *c = static_cast<Context *>(ctx);
        *c->fired = true;
        c->aio->Quit();
      },
      &context);

  aio.Run();
  EXPECT_TRUE(fired)
      << "The second Run() returned without servicing the loop; the earlier "
         "Quit() was never consumed.";
  EXPECT_FALSE(aio.should_run());
}

TEST_P(AioTest, TimerForkTest) {
  // Test that an active timer scheduled in the parent process remains fully
  // functional and fires inside a forked child process.  This verifies that the
  // backend correctly re-registers pending timeouts when recreating the loop.
  Aio aio;
  Aio::Timer timer(&aio);

  int timer_count = 0;
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *context) {
        auto *counter = static_cast<int *>(context);
        ++(*counter);
      },
      &timer_count);

  EXPECT_EXIT(
      {
        while (timer_count == 0 && aio.Poll(true)) {
        }
        if (timer_count == 1) {
          exit(42);
        }
        exit(1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test: constructing a Timer is the first thing a forked child
// does in some real call paths (EventSchedulerScheduler::RunFor(), reached
// under gtest death tests), and Timer construction arms a poll -- so
// Initialize() has to run the same fork check every other ring-touching
// entry point does.  Without it the child stages an SQE into the stale
// inherited ring and dies with -EEXIST out of io_uring_submit().  Found via
// logger_test's LoggerDeathTest.CrashOnFallBehind, whose failure named
// MaybeSubmit() under Aio::Timer::Timer() rather than anything timer-shaped.
TEST_P(AioTest, ConstructTimerInForkedChildTest) {
  Aio aio;
  // Drive the loop once in the parent, so the ring is bound and enabled
  // before the fork -- an unenabled ring would decline the submission on
  // its own and hide the bug.
  aio.Poll(false);

  EXPECT_EXIT(
      {
        // The child's very first Aio interaction is building a timer.
        Aio::Timer timer(&aio);
        int fired = 0;
        timer.Schedule(
            aos::monotonic_clock::now(),
            [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
        while (fired == 0 && aio.Poll(true)) {
        }
        exit(fired == 1 ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test: a repeating timer armed before a fork must keep
// repeating correctly in the child afterward, not silently revert to firing
// once and stopping.
TEST_P(AioTest, ForkDuringRepeatingTimerDeathTest) {
  Aio aio;

  int fire_count = 0;
  RepeatingTimer timer(&aio, [&fire_count](Completion completion) {
    if (aos::IsOk(completion.status)) {
      ++fire_count;
    }
  });
  timer.Start(aos::monotonic_clock::now() + std::chrono::milliseconds(10),
              std::chrono::milliseconds(10));

  // Let it fire a few times before forking.
  while (fire_count < 3 && aio.Poll(true)) {
  }
  ASSERT_GE(fire_count, 3);
  const int fires_before_fork = fire_count;

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        // If HandleFork() silently demoted this timer back to single-shot
        // (or dropped it entirely), fire_count would stall right where the
        // parent left it instead of continuing to climb.
        while (fire_count < fires_before_fork + 3 && aio.Poll(true)) {
        }
        exit(fire_count >= fires_before_fork + 3 ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test for a kernel quirk in IORING_SETUP_DEFER_TASKRUN rings
// (see global_parent_fork_count's comment in aio_linux.cc): a repeating
// timer outstanding in the parent becomes uncancelable after any fork() the
// parent was party to.  The kernel's cancel/remove lookup returns -ENOENT
// for it, even though the op is demonstrably still alive and firing.  This
// happens even when the child never touches this Aio, or this process, at
// all -- a plain fork()+exec() (starterd's normal child-spawning pattern)
// is enough.  CheckForParentFork() is the fix: a single
// io_uring_get_events() resync, lazily triggered on the next entry point
// touched after a fork, keeps the parent's own still-outstanding ops
// cancelable.
TEST_P(AioTest, ForkChildNeverTouchesAioTest) {
  // Bounds worst-case runtime if this ever regresses: the reap loop's own
  // kMaxReapAttempts bound would otherwise take on the order of a minute to
  // trip (10000 iterations at this timer's 10ms period) before crashing
  // with an actionable message -- this just gets there faster.
  ScopedDeathTestWatchdog watchdog;

  Aio aio;

  int fire_count = 0;
  RepeatingTimer timer(&aio, [&fire_count](Completion completion) {
    if (aos::IsOk(completion.status)) {
      ++fire_count;
    }
  });
  timer.Start(aos::monotonic_clock::now() + std::chrono::milliseconds(10),
              std::chrono::milliseconds(10));

  while (fire_count < 3 && aio.Poll(true)) {
  }
  ASSERT_GE(fire_count, 3);

  pid_t pid = fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    // Deliberately never touches `aio` (or anything io_uring-related) --
    // just enough elapsed time for a few more periods to have passed, to
    // match the shape that reproduced the bug.
    usleep(40000);
    _exit(0);
  }
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);

  // Canceling this (via ~Timer() below) must not hang.
}

// Regression test for IoUringImpl::CheckSubmitterThread(): destroying an Aio
// from a different thread than the one that first called Run()/Poll() on it
// must die loudly.  io_uring-specific -- IORING_SETUP_SINGLE_ISSUER is what
// makes this a real constraint; the epoll backend has no such thing.
TEST_P(AioTest, DestroyFromWrongThreadDeathTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "Same-thread destructor enforcement is io_uring-specific.";
  }
  EXPECT_DEATH(
      {
        // Construct and first-Poll() on the same thread (this one) -- no
        // auto-downgrade (see the next test) should be triggered, so
        // SINGLE_ISSUER's binding, and thus the enforcement, stays live.
        auto aio = std::make_unique<Aio>();
        aio->Poll(false);
        std::thread t([&aio]() { aio.reset(); });
        t.join();
      },
      "different thread");
}

// Regression test for the SINGLE_ISSUER-vs-PI-futex conflict: see
// documentation/adr/0001-aio-io-uring-single-issuer.md.  In short,
// IORING_SETUP_SINGLE_ISSUER wants destruction on Run()'s thread, but AOS's
// shared-memory queues (aos/ipc_lib/lockless_queue.cc's
// RobustOwnershipTracker) require every sender/watcher/pinner to be
// destroyed on its construction thread instead.  When an Aio's construction
// thread differs from the thread that first calls Run()/Poll() on it, the
// io_uring backend must downgrade away from SINGLE_ISSUER for that instance
// rather than enforce same-thread destruction.  Exercised here by
// constructing on this thread, Run()-ing on a worker thread, and destroying
// back on this thread again -- unlike the previous test, this must not
// crash.
TEST_P(AioTest, ConstructOnOneThreadRunOnAnotherTest) {
  auto aio = std::make_unique<Aio>();
  std::thread t([&aio]() { aio->Poll(false); });
  t.join();
  aio.reset();
}

// A raw AsyncRead/AsyncWrite submitted before the loop is first driven
// cannot survive the construct-here/run-there downgrade's ring rebuild:
// unlike the persistent registrations, there is no registry to re-arm it
// from.  The downgrade must refuse loudly rather than drop the request
// silently (it would otherwise just never complete).
TEST_P(AioTest, DowngradeWithRawRequestInFlightDeathTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "The SINGLE_ISSUER downgrade is io_uring-specific.";
  }
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest request;
        char buf[8];
        aio.AsyncRead(pipe.read_fd(), buf, &request);
        // First drive from a different thread triggers the downgrade.
        std::thread t([&aio]() { aio.Poll(false); });
        t.join();
      },
      "AsyncRead/AsyncWrite requests in flight");
}

// Regression test: legacy fd registration (OnReadable/OnWritable/OnError/
// OnEvents/EnableWritable/DisableWritable/SetEvents/DeleteFd) is backed by
// one shared, embedded epoll instance, not a per-fd io_uring request -- see
// documentation/adr/0001-aio-io-uring-single-issuer.md.  Mask changes and
// removal are plain epoll_ctl() calls, which never touch the ring at all,
// so none of them can be "the first call into an Aio" in the sense that
// triggers EnsureBound()/DowngradeFromSingleIssuer().  Exercised here the
// same way as the previous test (construct on this thread, touch the Aio
// from a different one) specifically via DeleteFd(), to pin down that this
// path in particular has no thread-binding dependency left at all.
TEST_P(AioTest, DeleteFdFromDifferentThreadTest) {
  Pipe pipe;
  auto aio = std::make_unique<Aio>();
  aio->OnReadable(pipe.read_fd(), []() {});

  std::thread t([&aio, &pipe]() { aio->DeleteFd(pipe.read_fd()); });
  t.join();
}

// Regression test: an orphaned receiver must survive the
// construct-here/run-there downgrade's ring rebuild.
// UnregisterThreadSignalReceiver() before the first Poll() orphans the
// receiver's state with a cancel SQE staged locally -- the ring is still
// disabled, so nothing has reached the kernel.  When the first drive then
// comes from a thread other than the constructing one,
// DowngradeFromSingleIssuer() replaces the ring: the staged cancel and any
// completions the orphan was waiting for die with it, and
// ReArmPersistentRegistrations()'s scrub must recycle such orphans rather
// than leave them parked forever waiting for CQEs that can never arrive.
// The re-registration afterward reuses the recycled state (see
// RegisterThreadSignalReceiver()'s freelist pop) and must still deliver
// wakeups.  The scoped watchdog turns a wedge into a clean failure if
// this regresses.
TEST_P(AioTest, UnregisterThreadSignalReceiverTriggersDowngradeTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "SINGLE_ISSUER downgrade is io_uring-specific.";
  }
  ScopedDeathTestWatchdog watchdog;

  auto aio = std::make_unique<Aio>();
  aos::ipc_lib::ThreadSignalReceiver sfd;
  aio->RegisterThreadSignalReceiver(&sfd, []() {});

  std::thread t([&aio, &sfd]() {
    aio->UnregisterThreadSignalReceiver(&sfd);
    // The first drive of the ring, from a non-construction thread: this is
    // what actually reaches EnsureBound() -> DowngradeFromSingleIssuer()
    // (Unregister itself never drives the ring), with the orphan parked.
    aio->Poll(false);
  });
  t.join();

  // The downgraded ring has no thread binding, so this thread may drive it
  // now.  Re-register -- reusing the recycled orphan -- and verify wakeups
  // still flow end-to-end on the rebuilt ring.
  int count = 0;
  aio->RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  while (count == 0 && aio->Poll(true)) {
  }
  EXPECT_EQ(count, 1);
  aio->UnregisterThreadSignalReceiver(&sfd);
}

// Regression test: a timer canceled asynchronously (via Timer::Cancel()) whose
// completion has not been reaped yet must not be left pending across a fork.
//
// With the io_uring backend Timer::Cancel() is asynchronous -- it submits a
// cancel and reaps the -ECANCELED later, inside Poll().  If HandleFork()
// doesn't resolve that in-flight cancellation, request.done stays false in the
// child (its completion died with the old ring) and the next reap-cancel -- the
// re-Schedule() below, or the Timer destructor -- waits forever for a
// completion that will never arrive.
//
// On Windows the death-test child re-execs from scratch rather than inheriting
// state, so there is nothing stale to recover; the code should still work, so
// we run it there too.  alarm() (the watchdog that turns a hang into a loud
// failure instead of wedging the suite) is the only POSIX-only piece.
TEST_P(AioTest, CancelTimerBeforeForkDeathTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  bool fired = false;
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion, void *ctx) { *static_cast<bool *>(ctx) = true; }, &fired);

  // Cancel asynchronously and deliberately do NOT Poll, so the cancellation is
  // still in flight when we fork.
  timer.Cancel();

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        // Re-Schedule in the child.  This triggers HandleFork() and then the
        // internal reap-cancel that would hang if the canceled timer weren't
        // resolved across the fork.  The fresh timer must then fire.
        timer.Schedule(
            aos::monotonic_clock::now(),
            [](Completion, void *ctx) { *static_cast<bool *>(ctx) = true; },
            &fired);
        while (!fired && aio.Poll(true)) {
        }
        exit(fired ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test: HandleFork() re-arms every active timer in a single pass.
// With a shallow ring (--aio_queue_depth below the number of live timers) that
// pass queues more submission entries than the ring is deep; without draining
// the queue mid-pass, io_uring_get_sqe() returns null and the child aborts with
// "Out of SQEs".  Schedule more timers than the depth, fork, and require the
// child to rebuild and fire them all.
//
// The --aio_queue_depth flag exists only on the submission-queue backends
// (io_uring, kqueue), so this test is Linux/macOS-only; the Windows IOCP
// backend has no submission queue to overflow.
#ifndef _WIN32
TEST_P(AioTest, ForkWithManyTimersDeathTest) {
  absl::FlagSaver flag_saver;

  // kNumTimers plus the wakeup read exceeds the kQueueDepth-entry submission
  // queue, so HandleFork()'s single reconstruction pass has to drain as it
  // fills.
  constexpr int kQueueDepth = 4;
  constexpr int kNumTimers = 6;
  static_assert(kNumTimers + 1 > kQueueDepth, "must overflow the submit queue");

  absl::SetFlag(&FLAGS_aio_queue_depth, kQueueDepth);

  Aio aio;
  std::vector<std::unique_ptr<Aio::Timer>> timers;
  for (int i = 0; i < kNumTimers; ++i) {
    timers.push_back(std::make_unique<Aio::Timer>(&aio));
    // Far-future so every timer is still pending at the fork, giving
    // HandleFork() all kNumTimers to re-arm.  Poll() flushes each submission so
    // the parent's own scheduling never overflows the queue -- only the
    // single-pass reconstruction does.
    timers.back()->Schedule(
        aos::monotonic_clock::now() + std::chrono::hours(1),
        [](Completion, void *) {}, nullptr);
    aio.Poll(false);
  }

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        // This Poll() rebuilds the ring and re-arms every pending request in a
        // single pass -- more submission entries than the ring is deep.
        // Surviving that pass, rather than aborting with "Out of SQEs", is the
        // assertion.
        aio.Poll(false);
        exit(42);
      },
      ::testing::ExitedWithCode(42), "");

  // Tear the timers down one at a time, draining between each.  Cancelling
  // costs two completions (the cancel itself, plus the timeout's -ECANCELED)
  // and the reap path deliberately doesn't advance the completion queue, so
  // destroying all kNumTimers back-to-back would overflow a queue this shallow.
  // That teardown limit is a separate concern from the reconstruction under
  // test, so keep it out of the way rather than tuning kNumTimers around it.
  for (auto &timer : timers) {
    timer.reset();
    aio.Poll(false);
  }
}
#endif  // !_WIN32

// Regression test for rescheduling an already-armed timer from an RT
// thread: it must not block, must not allocate, and the superseded
// schedule's callback must never reach the user -- only the new one.
//
// This is ordinary production usage rather than an edge case.
// ShmTimerHandler re-arms from inside its own callback on every firing, and
// external code reschedules live timers routinely; the historical failures
// here (TooBigConnect/StarterChainTest/TimerChangeParameters all hitting
// aos::CheckNotRealtime()) came from a reschedule path that could block.
// It is now a single timerfd_settime(2) on every backend, which is the
// strongest form of this guarantee available -- so the test is really
// pinning down that no future change reintroduces an asynchronous
// reschedule underneath it.
TEST_P(AioTest, RescheduleArmedTimerWhileRealtimeTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  bool old_fired = false;
  int new_fire_count = 0;

  {
    ScopedRealtime rt;
    // Arm far enough out that it's still armed when superseded below.
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::seconds(10),
        [](Completion, void *ctx) { *static_cast<bool *>(ctx) = true; },
        &old_fired);

    // Retarget the still-armed op to a much nearer deadline, with a
    // different callback.
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(20),
        [](Completion completion, void *ctx) {
          if (aos::IsOk(completion.status)) {
            ++(*static_cast<int *>(ctx));
          }
        },
        &new_fire_count);

    while (new_fire_count < 1 && aio.Poll(true)) {
    }
  }

  EXPECT_FALSE(old_fired) << "Superseded schedule's callback ran anyway.";
  EXPECT_EQ(new_fire_count, 1);

  {
    ScopedRealtime rt;
    timer.Cancel();
  }
}

// Destroying a timer from an RT thread dies deterministically:
// DestroyTimerState() can free (its nothing-in-flight fast path), and a
// sometimes-frees function under the malloc hook would be a data-dependent
// crash -- so it CheckNotRealtime()s up front, pending state or not.
TEST_P(AioTest, DeleteTimerWhileRealtimeDeathTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "CheckNotRealtime() enforcement is io_uring-specific.";
  }
  Aio aio;

  EXPECT_DEATH(
      {
        std::optional<Aio::Timer> timer;
        timer.emplace(&aio);
        timer->Schedule(
            aos::monotonic_clock::now() + std::chrono::seconds(10),
            [](Completion, void *) {}, nullptr);
        ScopedRealtime rt;
        timer.reset();  // Destructor runs here, while marked realtime.
      },
      // Pin the death to CheckNotRealtime()'s CHECK, not just any abort --
      // an empty matcher would pass on unrelated crashes.
      "GetIsRealtime");
}

// Destroying an armed timer (off RT) orphans its state -- the poll on its
// timerfd cancelled, user callback stripped -- and continued polling drains
// and recycles it; a new timer then reuses the freelist, including its
// already-created timerfd.  ASAN checks the lifetime story end to end.
TEST_P(AioTest, DeleteArmedTimerOrphansAndRecycles) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "Orphaned destruction is io_uring-specific.";
  }
  Aio aio;

  {
    Aio::Timer timer(&aio);
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(5),
        [](Completion, void *) {}, nullptr);
    // Drive the loop so the poll is really armed kernel-side before the
    // destructor has to cancel it.
    aio.Poll(false);
  }  // Orphaned here.

  // Drain the orphan's terminal completion; the sweep recycles it.
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (aos::monotonic_clock::now() < deadline) {
    aio.Poll(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // A fresh timer picks the state back up off the freelist and must work.
  Aio::Timer reused(&aio);
  int fired = 0;
  reused.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(5),
      [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
  while (fired == 0 && aio.Poll(true)) {
  }
  EXPECT_EQ(fired, 1);
}

// Confirms Schedule()/Cancel() (async) are unaffected by the enforcement
// above: both must keep working, unblocked, from an RT thread.
TEST_P(AioTest, AsyncCancelWhileRealtimeDoesNotDie) {
  Aio aio;
  Aio::Timer timer(&aio);
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion, void *) {}, nullptr);

  {
    ScopedRealtime rt;
    timer.Cancel();
  }
  // Reaching here without dying is the assertion.
}

// An unrecognized --aio_backend is fatal rather than a silent fallback.  The
// whole point of naming a backend is that a deployment states which one it
// gets -- quietly picking one after a typo would defeat that, and would do it
// on exactly the machines least able to notice.  Linux-only: it is the only
// platform with more than one backend to choose between, so it is the only
// one that parses the name at all.
#ifdef __linux__
TEST(AioBackendFlagTest, UnknownBackendDies) {
  absl::FlagSaver flag_saver;
  ::absl::SetFlag(&FLAGS_aio_backend, "nonsense");
  EXPECT_DEATH({ Aio aio; }, "Unknown --aio_backend");
}
#endif

// A forked child that touches an Aio which had a caller-submitted AsyncWrite
// in flight at the fork dies, rather than continuing into a shape with no
// correct answer: io_uring's child could never complete the write (the ring
// it was submitted to died with the fork, and there is no registry to re-arm
// raw requests from), while epoll's and kqueue's registrations survive, so
// the child would re-arm and send the same bytes the parent is also still
// sending.
//
// Deliberately checked where the child rebuilds, not in a pthread_atfork
// handler: fork() itself is fine, and fork()+exec() -- which is most of what
// forking is for around here -- must stay fine.  Only a child that goes on to
// use the loop has a problem.
//
// EXPECT_DEATH's own fork() is the fork under test.  The write is aimed at a
// pipe nobody drains, which is what keeps it outstanding.
#ifndef _WIN32
TEST_P(AioTest, ForkedChildWithPendingAsyncWriteDies) {
  Aio aio;
  Pipe pipe;

  std::vector<char> filler(1 << 16, 'x');
  while (write(pipe.write_fd(), filler.data(), filler.size()) > 0) {
  }

  AsyncRequest request;
  request.callback = [](Completion, void *) {};
  std::vector<char> payload(64, 'y');
  aio.AsyncWrite(pipe.write_fd(), std::span<const char>(payload), &request);
  aio.Poll(false);
  ASSERT_FALSE(request.done) << "The write completed; nothing is pending.";

  EXPECT_DEATH({ aio.Poll(false); }, "in flight at the fork");

  // Drain the parent's own copy, which is unaffected by the fork.  No
  // DeleteFd(): a raw AsyncWrite never creates a legacy registration on the
  // io_uring backend, so there would be nothing to delete.
  aio.Cancel(&request);
  while (!request.done && aio.Poll(false)) {
  }
}
#endif

INSTANTIATE_TEST_SUITE_P(AioBackends, AioTest,
                         ::testing::Values("io_uring", "epoll"),
                         [](const ::testing::TestParamInfo<std::string> &info) {
                           return info.param;
                         });

}  // namespace aos::testing

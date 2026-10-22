#include "aos/ipc_lib/signalfd.h"

#include <poll.h>
#include <pthread.h>
#include <unistd.h>

#include <memory>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/testing/test_logging.h"

namespace aos::ipc_lib::testing {

// Tests in this file use separate threads to isolate all manipulation of signal
// masks between test cases.

// Verify that SignalFd will leave signals unblocked if we ask it to.
TEST(SignalFdTest, LeaveSignalBlocked) {
  ::aos::testing::EnableTestLogging();
  std::thread thread([]() {
    {
      sigset_t test_mask;
      ABSL_CHECK_EQ(0, sigemptyset(&test_mask));
      ABSL_CHECK_EQ(0, sigaddset(&test_mask, SIGUSR1));
      ABSL_PCHECK(pthread_sigmask(SIG_BLOCK, &test_mask, nullptr) == 0);
    }
    SignalFd({SIGUSR1});
    {
      sigset_t blocked_now;
      ABSL_PCHECK(pthread_sigmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_TRUE(sigismember(&blocked_now, SIGUSR1));
    }
  });
  thread.join();
}

// Verify that SignalFd actually blocks the requested signals, and unblocks them
// afterwards.
TEST(SignalFdTest, BlockSignal) {
  ::aos::testing::EnableTestLogging();
  std::thread thread([]() {
    {
      sigset_t blocked_now;
      ABSL_PCHECK(pthread_sigmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_FALSE(sigismember(&blocked_now, SIGUSR1));
    }
    {
      SignalFd signalfd({SIGUSR1});
      sigset_t blocked_now;
      ABSL_PCHECK(pthread_sigmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_TRUE(sigismember(&blocked_now, SIGUSR1));
    }
    {
      sigset_t blocked_now;
      ABSL_PCHECK(pthread_sigmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_FALSE(sigismember(&blocked_now, SIGUSR1));
    }
  });
  thread.join();
}

namespace {
// Waits up to a second for the fd to be readable.
bool WaitReadable(int fd) {
  struct pollfd pfd = {.fd = fd, .events = POLLIN, .revents = 0};
  return poll(&pfd, 1, 1000) == 1 && (pfd.revents & POLLIN);
}

bool IsPending(int signal) {
  sigset_t pending;
  ABSL_PCHECK(sigpending(&pending) == 0);
  return sigismember(&pending, signal);
}
}  // namespace

// The tests which send a signal run on the test's main thread rather than
// their own.  A signal sent to the process goes to any thread which hasn't
// blocked it, and is otherwise left pending on one of them, so the only way to
// be sure which thread gets it is to have just one.

// Verify that a signal sent to us makes the fd readable, comes out of Read(),
// and is no longer pending afterwards.
TEST(SignalFdTest, ReadSignal) {
  ::aos::testing::EnableTestLogging();
  SignalFd signalfd({SIGUSR1, SIGUSR2});
  EXPECT_EQ(0u, signalfd.Read().ssi_signo);

  ABSL_PCHECK(kill(getpid(), SIGUSR2) == 0);
  ASSERT_TRUE(WaitReadable(signalfd.fd()));
  EXPECT_EQ(static_cast<uint32_t>(SIGUSR2), signalfd.Read().ssi_signo);
  EXPECT_FALSE(IsPending(SIGUSR2));
  EXPECT_EQ(0u, signalfd.Read().ssi_signo);
}

// Verify that a signal which was already pending when the SignalFd was made is
// reported.
TEST(SignalFdTest, ReadAlreadyPendingSignal) {
  ::aos::testing::EnableTestLogging();
  sigset_t test_mask;
  ABSL_CHECK_EQ(0, sigemptyset(&test_mask));
  ABSL_CHECK_EQ(0, sigaddset(&test_mask, SIGUSR1));
  ABSL_PCHECK(pthread_sigmask(SIG_BLOCK, &test_mask, nullptr) == 0);
  ABSL_PCHECK(kill(getpid(), SIGUSR1) == 0);
  ASSERT_TRUE(IsPending(SIGUSR1));
  {
    SignalFd signalfd({SIGUSR1});
    ASSERT_TRUE(WaitReadable(signalfd.fd()));
    EXPECT_EQ(static_cast<uint32_t>(SIGUSR1), signalfd.Read().ssi_signo);
    EXPECT_FALSE(IsPending(SIGUSR1));
  }
  ABSL_PCHECK(pthread_sigmask(SIG_UNBLOCK, &test_mask, nullptr) == 0);
}

// Verify that SignalFd responds correctly when some other code unblocks one of
// its signals.
TEST(SignalFdDeathTest, ExternalUnblockSignal) {
  ::aos::testing::EnableTestLogging();
  auto t = []() {
    SignalFd signalfd({SIGUSR1});
    sigset_t test_mask;
    ABSL_CHECK_EQ(0, sigemptyset(&test_mask));
    ABSL_CHECK_EQ(0, sigaddset(&test_mask, SIGUSR1));
    ABSL_PCHECK(pthread_sigmask(SIG_UNBLOCK, &test_mask, nullptr) == 0);
  };
  EXPECT_DEATH(
      { t(); }, "Some other code unblocked one or more of our signals");
}

}  // namespace aos::ipc_lib::testing

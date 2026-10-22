#include "aos/ipc_lib/signalfd.h"

#include <signal.h>
#include <sys/event.h>
#include <sys/types.h>
#include <unistd.h>

#include <csignal>
#include <cstring>
#include <initializer_list>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {
namespace {

// Signals the kernel raises at the instruction which caused them.  Blocking
// one of these, or returning from a handler for it, runs the instruction again
// forever on macOS, so they are left alone.
bool IsFaultSignal(int signal) {
  switch (signal) {
    case SIGILL:
    case SIGTRAP:
    case SIGFPE:
    case SIGBUS:
    case SIGSEGV:
    case SIGSYS:
      return true;
    default:
      return false;
  }
}

// EVFILT_SIGNAL has already recorded the signal by the time this runs, so
// there is nothing left to do.
void IgnoreSignal(int) {}

}  // namespace

SignalFd::SignalFd(::std::initializer_list<unsigned int> signal_list) {
  ABSL_PCHECK((fd_ = kqueue()) >= 0) << ": Failed to create a kqueue";

  // Watch the signals before blocking them, so one which arrives in between
  // is seen.  EVFILT_SIGNAL records every attempt to deliver the signal,
  // whether or not it is blocked, and is cleared when it is read.
  for (int signal : signal_list) {
    struct kevent ev;
    EV_SET(&ev, signal, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0, nullptr);
    ABSL_PCHECK(kevent(fd_, &ev, 1, nullptr, 0, nullptr) == 0)
        << ": Failed to watch signal " << signal;
  }

  ABSL_CHECK_EQ(0, sigemptyset(&blocked_mask_));
  ABSL_CHECK_EQ(0, sigemptyset(&handled_mask_));
  for (int signal : signal_list) {
    if (IsFaultSignal(signal)) {
      continue;
    }
    ABSL_CHECK_EQ(0, sigaddset(&blocked_mask_, signal));
    ABSL_CHECK_EQ(0, sigaddset(&handled_mask_, signal));

    struct sigaction action;
    std::memset(&action, 0, sizeof(action));
    action.sa_handler = IgnoreSignal;
    ABSL_CHECK_EQ(0, sigemptyset(&action.sa_mask));
    action.sa_flags = SA_RESTART;
    ABSL_PCHECK(sigaction(signal, &action, &old_actions_[signal]) == 0);
  }

  sigset_t old_mask;
  ABSL_CHECK_EQ(0, pthread_sigmask(SIG_BLOCK, &blocked_mask_, &old_mask));
  for (int signal : signal_list) {
    if (sigismember(&old_mask, signal)) {
      // It was blocked before, so leave it blocked.  Unlike
      // LeaveSignalBlocked(), restore the handler.
      ABSL_CHECK_EQ(0, sigdelset(&blocked_mask_, signal));
    }
  }

  // A signalfd reports signals which were already pending when it was made.
  // EVFILT_SIGNAL only sees the ones delivered after it was added, so stand in
  // for the others with a user event.  Read() handles both the same way.
  sigset_t pending;
  ABSL_PCHECK(sigpending(&pending) == 0);
  for (int signal : signal_list) {
    if (sigismember(&pending, signal)) {
      struct kevent ev;
      EV_SET(&ev, signal, EVFILT_USER, EV_ADD | EV_ONESHOT, NOTE_TRIGGER, 0,
             nullptr);
      ABSL_PCHECK(kevent(fd_, &ev, 1, nullptr, 0, nullptr) == 0)
          << ": Failed to report pending signal " << signal;
    }
  }
}

SignalFd::~SignalFd() {
  // Unwind the constructor. Unblock the signals, restore the handlers, and
  // close the fd. Verify nobody else unblocked the signals we're supposed to
  // unblock in the meantime.
  sigset_t old_mask;
  ABSL_CHECK_EQ(0, pthread_sigmask(SIG_UNBLOCK, &blocked_mask_, &old_mask));
  for (int signal = 1; signal < NSIG; ++signal) {
    if (sigismember(&blocked_mask_, signal) &&
        !sigismember(&old_mask, signal)) {
      ABSL_LOG(FATAL) << "Some other code unblocked one or more of our signals";
    }
  }
  for (int signal = 1; signal < NSIG; ++signal) {
    if (sigismember(&handled_mask_, signal)) {
      ABSL_PCHECK(sigaction(signal, &old_actions_[signal], nullptr) == 0);
    }
  }
  ABSL_PCHECK(close(fd_) == 0);
}

SignalInfo SignalFd::Read() {
  SignalInfo result;

  struct kevent ev;
  const struct timespec zero = {};
  if (kevent(fd_, nullptr, 0, &ev, 1, &zero) != 1) {
    result.ssi_signo = 0;
    return result;
  }
  result.ssi_signo = static_cast<uint32_t>(ev.ident);
  ABSL_CHECK_NE(0u, result.ssi_signo);

  // Reading a signalfd takes the signal off the pending set, and reading a
  // kqueue does not.  Take it off with sigwait() so it isn't delivered when it
  // is unblocked.  Only wait when it is pending on this thread, since sigwait()
  // blocks if it isn't.  It may be pending on another thread instead, or not at
  // all if a thread which doesn't block it took it.
  sigset_t pending;
  ABSL_PCHECK(sigpending(&pending) == 0);
  if (sigismember(&pending, result.ssi_signo)) {
    sigset_t wait_mask;
    ABSL_CHECK_EQ(0, sigemptyset(&wait_mask));
    ABSL_CHECK_EQ(0, sigaddset(&wait_mask, result.ssi_signo));
    int signal;
    ABSL_CHECK_EQ(0, sigwait(&wait_mask, &signal));
  }
  return result;
}

void SignalFd::LeaveSignalBlocked(unsigned int signal) {
  ABSL_CHECK_EQ(0, sigdelset(&blocked_mask_, signal));
  ABSL_CHECK_EQ(0, sigdelset(&handled_mask_, signal));
}

}  // namespace aos::ipc_lib

#ifndef AOS_IPC_LIB_SIGNALFD_H_
#define AOS_IPC_LIB_SIGNALFD_H_

#include <signal.h>
#include <stdint.h>

#include <initializer_list>

#if defined(__linux__)
#include <sys/signalfd.h>
#endif

namespace aos::ipc_lib {

#if defined(__linux__)
using SignalInfo = signalfd_siginfo;
#else
// What a kqueue tells us about a signal.  Named like signalfd_siginfo so code
// which only looks at the signal number reads the same on both.
struct SignalInfo {
  uint32_t ssi_signo;
};
#endif

// Class to manage a signalfd.
//
// This is the general "turn a set of signals into a readable fd" primitive
// (used e.g. by starter to watch child-process signals).  On Linux it is a
// signalfd.  On macOS it is a kqueue with an EVFILT_SIGNAL filter for each
// signal, which goes readable the same way.
//
// On macOS, only signals sent to the process (kill(2)) are seen, not ones sent
// to a thread (pthread_kill(3) or raise(3)).  EVFILT_SIGNAL doesn't see those.
//
// On macOS, a signal sent to the process while every thread blocks it is left
// pending on one of the threads, and only that thread can take it with
// sigwait(2).  So a handler which does nothing is installed for each signal as
// well, so that a signal left on another thread does nothing when that thread
// unblocks it rather than taking its default action.  The handler is restored
// by the destructor.
//
// On macOS, signals which the kernel raises for a faulting instruction (SIGILL,
// SIGTRAP, SIGFPE, SIGBUS, SIGSEGV and SIGSYS) are watched but not blocked or
// handled.  Linux kills a process which faults with that signal blocked, and
// macOS instead runs the faulting instruction again forever.
//
// For the thread-wakeup mechanism, use ThreadSignalReceiver instead.
class SignalFd {
 public:
  // Constructs a SignalFd for the provided list of signals.  Blocks the signals
  // at the same time in this thread.
  SignalFd(::std::initializer_list<unsigned int> signal_list);

  SignalFd(const SignalFd &) = delete;
  SignalFd &operator=(const SignalFd &) = delete;
  ~SignalFd();

  // Reads the next signal.  If there was an error, or no signal was waiting,
  // the resulting ssi_signo will be 0.
  SignalInfo Read();

  // Ensures the destructor will leave the specific signal blocked.  This can be
  // helpful if the signal is sent asynchronously, such that it may arrive after
  // this object is destroyed, to ensure that doesn't kill the process.  On
  // macOS the handler which does nothing is left installed too.
  void LeaveSignalBlocked(unsigned int signal);

  // Returns the file descriptor to wait on for it to be readable.
  int fd() { return fd_; }

 private:
  int fd_ = -1;

  // The signals we blocked in the constructor.
  sigset_t blocked_mask_;

#if !defined(__linux__)
  // The signals we installed a handler for, and what they had before.
  sigset_t handled_mask_;
  struct sigaction old_actions_[NSIG];
#endif
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_SIGNALFD_H_

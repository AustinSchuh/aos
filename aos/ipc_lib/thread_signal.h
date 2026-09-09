#ifndef AOS_IPC_LIB_THREAD_SIGNAL_H_
#define AOS_IPC_LIB_THREAD_SIGNAL_H_

#include <functional>
#include <memory>

#include "aos/realtime.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <signal.h>
#else
#include <signal.h>
#include <sys/signalfd.h>
#endif

namespace aos::ipc_lib {

namespace internal {

// The one-receiver-per-thread rule, shared by every platform so it reads the
// same everywhere; see ThreadSignalReceiver::BindToCurrentThread().  The
// binding itself lives in thread_signal.cc -- a thread_local defined here
// would be one per translation unit including this header.
void BindReceiverToThread(const void *receiver);
void UnbindReceiverFromThread(const void *receiver);

}  // namespace internal

#ifdef _WIN32
const static unsigned int kWakeupSignal = 0;
#elif defined(__APPLE__)
const static unsigned int kWakeupSignal = SIGUSR1;
#else
const static unsigned int kWakeupSignal = SIGRTMIN + 2;
#endif

// Sends a wakeup to a specific thread in a (possibly different) process.
class ThreadSignalSender {
 public:
  ThreadSignalSender();
  ~ThreadSignalSender();

  ThreadSignalSender(const ThreadSignalSender &) = delete;
  ThreadSignalSender &operator=(const ThreadSignalSender &) = delete;
  ThreadSignalSender(ThreadSignalSender &&other);
  ThreadSignalSender &operator=(ThreadSignalSender &&other);

  // Wakes up the target thread.
  // On Linux, this uses rt_tgsigqueueinfo.
  // On Windows, this opens and signals the thread's named event.
  // On macOS, since cross-process thread-directed signals are not supported,
  // this sends a process-directed signal (kill) using kWakeupSignal, which
  // may trigger spurious wakeups on other threads waiting in the same process.
  void Signal(pid_t pid, pid_t tid);

#ifdef _WIN32
  HANDLE event_handle() const { return event_handle_; }
#endif

 private:
#ifdef _WIN32
  HANDLE event_handle_ = NULL;
#else
  const pid_t pid_;
  const uid_t uid_;
#endif
};

// The receiving half of the thread wakeup mechanism.
//
// This wraps the platform primitive used to receive kWakeupSignal (a signalfd
// on Linux, an ignored signal + kqueue EVFILT_SIGNAL on macOS, a named Event on
// Windows) and exposes it to the event loop, so callers can watch it with their
// event loop (e.g. via Aio::RegisterThreadSignalReceiver).  It replaces the old
// separate SignalFd class -- if raw signalfd-of-arbitrary-signals behavior is
// ever needed again, add a dedicated (Linux-only) class for it rather than
// generalizing this one.
class ThreadSignalReceiver {
 public:
  ThreadSignalReceiver();
  ~ThreadSignalReceiver();

  ThreadSignalReceiver(const ThreadSignalReceiver &) = delete;
  ThreadSignalReceiver &operator=(const ThreadSignalReceiver &) = delete;

#ifdef _WIN32
  // The Event which ThreadSignalSender::Signal() signals.  Windows has no
  // signalfd equivalent, so the Aio (IOCP) backend watches this handle instead
  // of an fd.
  HANDLE event_handle() const { return event_handle_; }
#elif defined(__APPLE__)
  // The signalfd's file descriptor (Linux).  -1 on platforms that don't back
  // the receiver with an fd (macOS).
  int fd() const { return -1; }
#else
  int fd() const { return fd_; }
#endif

  // Binds this receiver to the calling thread -- the thread whose wakeups it
  // delivers.  Aio::RegisterThreadSignalReceiver() calls this, because
  // registration is the first moment the receiver learns which thread it
  // serves: it runs on the polling thread, while construction may not.
  //
  // At most one receiver may be bound to a thread at a time; a second CHECKs.
  void BindToCurrentThread();

  // Releases the binding, so another receiver may take this thread.  Called
  // by Aio::UnregisterThreadSignalReceiver().  Safe to call unbound.
  void UnbindFromCurrentThread();

  // Drains any pending wakeups so we don't immediately wake again.
  void ConsumeWakeup();

  // Leaves kWakeupSignal blocked (Linux) / ignored (macOS) when this receiver
  // is destroyed, rather than restoring the previous disposition.  This closes
  // a shutdown race: a sender can signal us between when we stop being watched
  // and when we're destroyed, and without this the stray signal would hit the
  // default action and kill the process.  There is no signal disposition to
  // restore on Windows, so it does nothing there.
  void LeaveSignalBlocked();

 private:
#if defined(_WIN32)
  // The Event the sender opens by name and signals.  Manual-reset, so a
  // successful wait leaves it set and ConsumeWakeup() is what clears it --
  // see the constructor for why the consumption point has to be ours.
  HANDLE event_handle_ = NULL;

  // The thread event_handle_ is currently named for, so BindToCurrentThread()
  // can tell a real thread change from a re-registration on the same thread.
  pid_t bound_tid_ = 0;

  // Nothing backs the receiver on macOS, so it has no state at all.
#elif !defined(__APPLE__)
  // Reads a single signalfd_siginfo.  On error/EAGAIN the resulting ssi_signo
  // is 0.
  signalfd_siginfo Read();

  int fd_ = -1;
  // Whether the destructor unblocks kWakeupSignal.  False if it was already
  // blocked before we got here, or if LeaveSignalBlocked() was called.
  bool should_unblock_ = true;
#endif
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_THREAD_SIGNAL_H_

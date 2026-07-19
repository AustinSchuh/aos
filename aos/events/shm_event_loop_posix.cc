#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <initializer_list>
#include <mutex>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/shm_event_loop.h"
#include "aos/ipc_lib/thread_signal.h"

namespace aos {
namespace {

// RAII class to mask signals.
class ScopedSignalMask {
 public:
  ScopedSignalMask(std::initializer_list<int> signals) {
    sigset_t sigset;
    ABSL_PCHECK(sigemptyset(&sigset) == 0);
    for (int signal : signals) {
      ABSL_PCHECK(sigaddset(&sigset, signal) == 0);
    }

    ABSL_PCHECK(sigprocmask(SIG_BLOCK, &sigset, &old_) == 0);
  }

  ~ScopedSignalMask() {
    ABSL_PCHECK(sigprocmask(SIG_SETMASK, &old_, nullptr) == 0);
  }

 private:
  sigset_t old_;
};

class SignalHandler {
 public:
  static SignalHandler *global() {
    static SignalHandler loop;
    return &loop;
  }

  static void HandleSignal(int) { global()->DoHandleSignal(); }

  void Register(ShmEventLoop *event_loop) {
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    std::unique_lock<stl_mutex> locker(mutex_);
    if (event_loops_.size() == 0) {
      struct sigaction new_action;
      sigemptyset(&new_action.sa_mask);
      new_action.sa_flags = SA_RESETHAND;
      new_action.sa_handler = &HandleSignal;

      ABSL_PCHECK(sigaction(SIGINT, &new_action, &old_action_int_) == 0);
      ABSL_PCHECK(sigaction(SIGHUP, &new_action, &old_action_hup_) == 0);
      ABSL_PCHECK(sigaction(SIGTERM, &new_action, &old_action_term_) == 0);
    }

    event_loops_.push_back(event_loop);
  }

  void Unregister(ShmEventLoop *event_loop) {
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    std::unique_lock<stl_mutex> locker(mutex_);

    event_loops_.erase(
        std::find(event_loops_.begin(), event_loops_.end(), event_loop));

    if (event_loops_.size() == 0u) {
      ABSL_PCHECK(sigaction(SIGINT, &old_action_int_, nullptr) == 0);
      ABSL_PCHECK(sigaction(SIGHUP, &old_action_hup_, nullptr) == 0);
      ABSL_PCHECK(sigaction(SIGTERM, &old_action_term_, nullptr) == 0);
    }
  }

 private:
  void DoHandleSignal() {
    ABSL_CHECK(mutex_.try_lock())
        << ": sigprocmask failed to block signals while "
           "modifing the event loop list.";
    for (ShmEventLoop *event_loop : event_loops_) {
      event_loop->Exit();
    }
    mutex_.unlock();
  }

  stl_mutex mutex_;
  std::vector<ShmEventLoop *> event_loops_;
  struct sigaction old_action_int_;
  struct sigaction old_action_hup_;
  struct sigaction old_action_term_;
};

}  // namespace

void ShmEventLoop::IgnoreWakeupSignal() {
  struct sigaction action;
  action.sa_handler = SIG_IGN;
  ABSL_PCHECK(sigemptyset(&action.sa_mask) == 0);
  action.sa_flags = 0;
  ABSL_PCHECK(sigaction(ipc_lib::kWakeupSignal, &action, nullptr) == 0);
}

void ShmEventLoop::RegisterSignalHandler() {
  SignalHandler::global()->Register(this);
}

void ShmEventLoop::UnregisterSignalHandler() {
  SignalHandler::global()->Unregister(this);
}

void ShmEventLoop::InitializeRealtime() { ::aos::InitRT(); }

}  // namespace aos

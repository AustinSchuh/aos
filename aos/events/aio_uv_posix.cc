// The POSIX half of the libuv backend; see aio_uv_internal.h for the seam.
//
// libuv can only wait on descriptors, so each of the two things AOS needs
// waking up for has to be one: a deadline, and a thread wakeup signal.  Linux
// has a purpose-built descriptor for each -- a timerfd and the receiver's own
// signalfd -- which is what makes this half short here.  A platform with
// neither has to build them; that is what the seam is for.  Below these
// helpers there is only ever an fd, and the core watches it through its own
// OnReadable().

#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <uv.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/numeric/int128.h"

#include "aos/events/aio_uv_internal.h"
#include "aos/ipc_lib/thread_signal.h"

namespace aos::uv_internal {

int UvPlatform::Read(FileDescriptor fd, char *ptr, size_t size) {
  return static_cast<int>(::read(fd, ptr, size));
}

int UvPlatform::Write(FileDescriptor fd, const char *ptr, size_t size) {
  return static_cast<int>(::write(fd, ptr, size));
}

int UvPlatform::LastIoError() { return errno; }

bool UvPlatform::IsInvalidFd(FileDescriptor fd) { return fd < 0; }

int UvPlatform::PollInit(uv_loop_t *loop, uv_poll_t *poll, FileDescriptor fd) {
  return uv_poll_init(loop, poll, fd);
}

void UvPlatform::Close(FileDescriptor fd) { close(fd); }

// libuv's readiness is the kernel's own here, so what the core asks for and
// what it is told both stand as they are.
int UvPlatform::AdjustWatch(uint32_t /*subscribed*/, int wanted) {
  return wanted;
}

namespace {

int CreateDeadlineFd() {
  const int result =
      timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);
  ABSL_PCHECK(result >= 0) << ": Failed to create timerfd";
  return result;
}

void ArmDeadline(int fd, aos::monotonic_clock::time_point deadline) {
  struct itimerspec its;
  std::memset(&its, 0, sizeof(its));
  // The epoch itself has to still arm the timer, so a deadline of exactly zero
  // becomes one nanosecond -- an it_value of all zeroes disarms instead.
  const auto since_epoch = deadline.time_since_epoch();
  const int64_t nanoseconds = std::max<int64_t>(
      1, std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch)
             .count());
  its.it_value.tv_sec = nanoseconds / 1000000000;
  its.it_value.tv_nsec = nanoseconds % 1000000000;
  ABSL_PCHECK(timerfd_settime(fd, TFD_TIMER_ABSTIME, &its, nullptr) == 0)
      << ": timerfd_settime failed";
}

void DisarmDeadline(int fd) {
  struct itimerspec its;
  std::memset(&its, 0, sizeof(its));
  timerfd_settime(fd, 0, &its, nullptr);
}

bool DrainDeadline(int fd) {
  uint64_t expirations = 0;
  const ssize_t res = read(fd, &expirations, sizeof(expirations));
  return res == static_cast<ssize_t>(sizeof(expirations));
}

// The receiver is a signalfd here, which libuv can wait on directly, so there
// is nothing of our own to close.
constexpr bool kOwnsWakeupFd = false;

int CreateWakeupFd(ipc_lib::ThreadSignalReceiver *receiver) {
  ABSL_CHECK_GE(receiver->fd(), 0)
      << ": This platform's ThreadSignalReceiver is not a descriptor, so "
         "libuv cannot wait on it";
  return receiver->fd();
}

void DrainWakeupFd(int fd) {
  // Batched, the way EpollImpl drains this same descriptor: signalfd hands
  // back as many siginfos as fit the buffer, so the ordinary one-signal wakeup
  // costs a single read() and a short count is what says the queue is empty --
  // no EAGAIN bounce through the kernel just to end the loop.
  struct signalfd_siginfo siginfos[16];
  while (true) {
    const ssize_t res = read(fd, siginfos, sizeof(siginfos));
    if (res > 0) {
      if (static_cast<size_t>(res) < sizeof(siginfos)) {
        return;  // Short read: nothing was left pending.
      }
      continue;
    }
    if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }
    if (res < 0 && errno == EINTR) {
      continue;
    }
    ABSL_PCHECK(false) << ": Failed to drain the wakeup signalfd";
  }
}

// A timer backed by a deadline descriptor watched with uv_poll -- a timerfd
// here.
//
// libuv's own uv_timer_t has millisecond resolution, which would quietly
// coarsen every AOS timer by up to a millisecond.  The descriptor keeps the
// nanosecond deadline AOS schedules with, and is just another descriptor for
// libuv to poll -- the same trade EpollImpl makes.
class PosixTimerState : public Aio::TimerState {
 public:
  explicit PosixTimerState(UvCore *core) : core_(core) {}
  ~PosixTimerState() override {
    core_->TimerDestroyed();
    if (timer_fd_ >= 0) {
      // The registration closes it, once libuv is done polling it.
      core_->DeleteAndCloseFd(timer_fd_);
      timer_fd_ = -1;
    }
  }

  void Initialize() override {
    timer_fd_ = CreateDeadlineFd();
    core_->OnReadable(timer_fd_, [this]() { HandleExpiration(); });
  }

  void Schedule(aos::monotonic_clock::time_point deadline,
                CompletionCallback callback, void *context) override {
    ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
    Cancel(true);

    this->deadline = deadline;
    this->user_callback = callback;
    this->user_context = context;
    this->request.done = false;

    ArmDeadline(timer_fd_, deadline);
  }

  void Cancel(bool /*reap*/) override {
    if (timer_fd_ < 0) {
      return;
    }
    DisarmDeadline(timer_fd_);
    request.done = true;
    user_callback = nullptr;
  }

 private:
  void HandleExpiration() {
    if (!DrainDeadline(timer_fd_)) {
      // Spurious readability, or the timer was disarmed between the wakeup
      // and this drain. Either way there is nothing to report.
      return;
    }
    if (user_callback == nullptr) {
      return;
    }
    CompletionCallback callback = user_callback;
    void *const context = user_context;
    user_callback = nullptr;
    request.done = true;
    callback(
        Completion{.status = aos::Status{}, .result = 0, .user_data = nullptr},
        context);
  }

  UvCore *const core_;
  int timer_fd_ = -1;
};

class PosixPlatform : public UvPlatform {
 public:
  explicit PosixPlatform(UvCore *core) : core_(core) {}

  // Everything is a descriptor here, and libuv polls every one of them.
  bool Adopt(FdRegistration * /*reg*/) override { return false; }
  void Update(FdRegistration * /*reg*/) override {}
  void Release(std::unique_ptr<FdRegistration> /*reg*/) override {}
  void Quit() override {}
  bool DispatchReadiness(FdRegistration * /*reg*/, int /*status*/,
                         int /*uv_events*/) override {
    return false;
  }

  std::unique_ptr<Aio::TimerState> MakeTimerState() override {
    return std::make_unique<PosixTimerState>(core_);
  }

  void RegisterReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                        std::function<void()> callback) override {
    wakeup_fd_ = CreateWakeupFd(receiver);
    core_->OnReadable(wakeup_fd_, [this, callback = std::move(callback)]() {
      // Draining is what consumes the wakeup on both platforms; see
      // DrainWakeupFd().
      DrainWakeupFd(wakeup_fd_);
      if (callback) callback();
    });
  }

  void UnregisterReceiver(
      ipc_lib::ThreadSignalReceiver * /*receiver*/) override {
    if (kOwnsWakeupFd) {
      // Ours to close, but not before libuv has stopped polling it.
      core_->DeleteAndCloseFd(wakeup_fd_);
    } else {
      core_->DeleteFd(wakeup_fd_);
    }
    wakeup_fd_ = -1;
  }

  void ConsumeReceiver(ipc_lib::ThreadSignalReceiver * /*receiver*/) override {
    // Nothing registered means nothing to drain.  The other backends can reach
    // for receiver->fd() unconditionally here because the receiver owns their
    // descriptor; ours may be one we built at registration and have since
    // closed, so this has to go through wakeup_fd_ instead.
    if (wakeup_fd_ < 0) {
      return;
    }
    DrainWakeupFd(wakeup_fd_);
  }

 private:
  UvCore *const core_;
  // The descriptor libuv waits on for the receiver's wakeups.  Borrowed from
  // the receiver or owned by us, depending on the platform -- see
  // kOwnsWakeupFd.
  int wakeup_fd_ = -1;
};

}  // namespace

std::unique_ptr<UvPlatform> MakeUvPlatform(UvCore *core) {
  return std::make_unique<PosixPlatform>(core);
}

}  // namespace aos::uv_internal

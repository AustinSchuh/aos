// The POSIX half of the libuv backend; see aio_uv_internal.h for the seam.
//
// libuv can only wait on descriptors, so each of the two things AOS needs
// waking up for has to be one: a deadline, and a thread wakeup signal.  Linux
// has a purpose-built descriptor for each (timerfd, signalfd).  macOS has
// neither -- but a kqueue is itself a descriptor, it goes readable when one of
// its filters fires, and it nests inside the kqueue libuv is polling with, so
// a kqueue holding a single EVFILT_TIMER or EVFILT_SIGNAL is the same object
// under a different name.  Below these helpers there is only ever an fd, and
// the core watches it through its own OnReadable().

#include <unistd.h>
#include <uv.h>

#if defined(__APPLE__)
#include <mach/mach_time.h>
#include <sys/event.h>
#else
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#endif

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

// libuv's readiness is the kernel's own here, so on epoll what the core asks
// for and what it is told both stand as they are.
//
// kqueue is the exception, and it hangs rather than merely missing an event.
// libuv spells UV_DISCONNECT as UV__POLLRDHUP, and its kqueue backend only
// ever adds a knote for POLLIN or POLLOUT (uv__io_poll() in libuv's
// src/unix/kqueue.c: the EVFILT_READ add is guarded on `w->pevents & POLLIN`,
// the EVFILT_WRITE add on `w->pevents & POLLOUT`).  A subscription to
// UV_DISCONNECT alone therefore arms nothing at all, and uv_run() then blocks
// in kevent() with an empty change list -- forever, since the hangup it is
// waiting for has nothing to arrive on.  The RDHUP report is itself only
// reachable from inside that same EVFILT_READ branch, gated on EV_EOF.
//
// So a registration that wants errors is watched for readability too, exactly
// as the Windows platform does and for a kindred reason.  EV_EOF then fires
// the read filter and libuv reports UV_READABLE|UV_DISCONNECT together.
// LegacyFdErrorTest pins this; without it that test hangs until its timeout.
int UvPlatform::AdjustWatch(uint32_t subscribed, int wanted) {
#if defined(__APPLE__)
  if ((subscribed & kErr) != 0) {
    wanted |= UV_READABLE;
  }
#else
  (void)subscribed;
#endif
  return wanted;
}

namespace {

#if defined(__APPLE__)

// Each timer gets its own kqueue holding exactly one filter, so the ident it
// is keyed by is arbitrary.
constexpr uintptr_t kDeadlineIdent = 1;

// EVFILT_TIMER takes Mach ticks when the deadline is given as
// NOTE_MACHTIME | NOTE_ABSOLUTE.  Same conversion as aio_darwin.cc.
uint64_t ToMachTicks(aos::monotonic_clock::time_point time) {
  static mach_timebase_info_data_t timebase_info = []() {
    mach_timebase_info_data_t info;
    mach_timebase_info(&info);
    return info;
  }();
  const uint64_t nanos =
      std::chrono::nanoseconds(time.time_since_epoch()).count();
  return static_cast<uint64_t>((absl::uint128(nanos) * timebase_info.denom) /
                               timebase_info.numer);
}

int CreateDeadlineFd() {
  const int result = kqueue();
  ABSL_PCHECK(result >= 0) << ": Failed to create a kqueue for a timer";
  return result;
}

void ArmDeadline(int fd, aos::monotonic_clock::time_point deadline) {
  // NOTE_MACHTIME | NOTE_ABSOLUTE keeps the nanosecond deadline AOS scheduled
  // with, where uv_timer_t would round it to a millisecond.  EV_ONESHOT
  // because AOS re-arms every repeating timer itself.
  struct kevent ev;
  EV_SET(&ev, kDeadlineIdent, EVFILT_TIMER, EV_ADD | EV_ENABLE | EV_ONESHOT,
         NOTE_MACHTIME | NOTE_ABSOLUTE, ToMachTicks(deadline), nullptr);
  ABSL_PCHECK(kevent(fd, &ev, 1, nullptr, 0, nullptr) == 0)
      << ": Failed to arm EVFILT_TIMER";
}

void DisarmDeadline(int fd) {
  struct kevent ev;
  EV_SET(&ev, kDeadlineIdent, EVFILT_TIMER, EV_DELETE, 0, 0, nullptr);
  // ENOENT here just means EV_ONESHOT already retired the filter by firing.
  kevent(fd, &ev, 1, nullptr, 0, nullptr);
}

bool DrainDeadline(int fd) {
  struct kevent ev;
  const struct timespec zero = {};
  return kevent(fd, nullptr, 0, &ev, 1, &zero) == 1;
}

// This platform's ThreadSignalReceiver is backed by nothing at all: it only
// sets kWakeupSignal to ignored and leaves observing it to whoever cares.  So
// the descriptor libuv waits on is one we build and own.
constexpr bool kOwnsWakeupFd = true;

int CreateWakeupFd(ipc_lib::ThreadSignalReceiver * /*receiver*/) {
  const int result = kqueue();
  ABSL_PCHECK(result >= 0) << ": Failed to create a kqueue for wakeups";
  struct kevent ev;
  EV_SET(&ev, ipc_lib::kWakeupSignal, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0,
         nullptr);
  ABSL_PCHECK(kevent(result, &ev, 1, nullptr, 0, nullptr) == 0)
      << ": Failed to watch for wakeup signals";
  return result;
}

void DrainWakeupFd(int fd) {
  // ThreadSignalReceiver::ConsumeWakeup() is a no-op here -- it expects
  // whoever owns the kqueue to consume the event by reading it, which is
  // exactly this.  Leaving it unread would leave the fd readable forever, and
  // libuv would spin on it.
  struct kevent ev;
  const struct timespec zero = {};
  while (kevent(fd, nullptr, 0, &ev, 1, &zero) == 1) {
  }
}

#else

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

#endif

// A timer backed by a deadline descriptor watched with uv_poll -- a timerfd on
// Linux, a kqueue holding one EVFILT_TIMER on macOS.
//
// libuv's own uv_timer_t has millisecond resolution, which would quietly
// coarsen every AOS timer by up to a millisecond.  Either descriptor keeps the
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

  ~PosixPlatform() override {
    // Only reachable for a fd we built; a borrowed one is the receiver's.
    if (kOwnsWakeupFd && wakeup_fd_ >= 0) {
      close(wakeup_fd_);
    }
  }

  void RegisterReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                        std::function<void()> callback) override {
    // A wakeup fd kept from an earlier registration is reused rather than
    // rebuilt; see UnregisterReceiver() for why it is still here.
    if (wakeup_fd_ < 0) {
      wakeup_fd_ = CreateWakeupFd(receiver);
    }
    receiver_registered_ = true;
    core_->OnReadable(wakeup_fd_, [this, callback = std::move(callback)]() {
      // Draining is what consumes the wakeup on both platforms; see
      // DrainWakeupFd().
      DrainWakeupFd(wakeup_fd_);
      if (callback) callback();
    });
  }

  void UnregisterReceiver(
      ipc_lib::ThreadSignalReceiver * /*receiver*/) override {
    core_->DeleteFd(wakeup_fd_);
    receiver_registered_ = false;
    // Where the fd is ours it stays open, unpolled, until the next
    // registration claims it.  Closing it here is what the signalfd platforms
    // get away with and this one cannot: our fd is a kqueue whose
    // EVFILT_SIGNAL knote *is* the pending count, so closing it discards
    // every wakeup that arrived while it was armed -- and those belong to the
    // successor receiver, per Aio::RegisterThreadSignalReceiver()'s contract.
    // KqueueImpl keeps its own knote armed across an unregister for exactly
    // this reason; retaining the fd is the same decision.  A borrowed signalfd
    // needs none of it: it simply stops being polled and keeps what is queued.
    if (!kOwnsWakeupFd) {
      wakeup_fd_ = -1;
    }
  }

  void ConsumeReceiver(ipc_lib::ThreadSignalReceiver * /*receiver*/) override {
    // Nothing registered means nothing to drain.  The other backends can reach
    // for receiver->fd() unconditionally here because the receiver owns their
    // descriptor; ours may be one we built at registration and have since
    // closed, so this has to go through wakeup_fd_ instead.
    //
    // receiver_registered_ rather than the fd alone: a retained fd outlives
    // the registration, and draining it while nothing is registered would
    // consume the very wakeups being held for the successor.
    if (wakeup_fd_ < 0 || !receiver_registered_) {
      return;
    }
    DrainWakeupFd(wakeup_fd_);
  }

 private:
  UvCore *const core_;
  // The descriptor libuv waits on for the receiver's wakeups.  Borrowed from
  // the receiver or owned by us, depending on the platform -- see
  // kOwnsWakeupFd.  An owned one outlives the registration it was made for.
  int wakeup_fd_ = -1;
  // Whether a receiver is registered right now, which a live wakeup_fd_ no
  // longer answers on its own.
  bool receiver_registered_ = false;
};

}  // namespace

std::unique_ptr<UvPlatform> MakeUvPlatform(UvCore *core) {
  return std::make_unique<PosixPlatform>(core);
}

}  // namespace aos::uv_internal

#include "aos/events/aio.h"

#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"

#include "aos/events/aio_internal.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/libc/aos_strerror.h"
#include "aos/time/time.h"

ABSL_FLAG(size_t, aio_epoll_pool_size, 16,
          "Initial size of the pre-allocated epoll FdRegistration pool.");

ABSL_FLAG(uint32_t, aio_queue_depth, 1024,
          "Depth of the io_uring submission and completion queues.");

ABSL_FLAG(std::string, aio_backend, "kqueue",
          "Which Aio backend to use.  macOS has only the kqueue backend, so "
          "this exists to keep the flag's name and meaning the same on every "
          "platform; it is accepted and ignored.");

namespace aos {

class KqueueImpl;

namespace {

// Open kqueue file descriptors are not valid inside a forked child, and
// their event registrations have to be rebuilt before the child can use the
// loop again.
//
// Detected the same way the Linux backends do it (see aio_linux.cc): a
// pthread_atfork child handler bumps a counter, and each loop compares it at
// the top of every public entry point (CheckForFork()).  Deliberately lazy
// rather than resetting every live loop from inside the handler, which is
// what this used to do:
//
//   * A child that never touches the loop -- fork()+exec(), which is most of
//     what forking is for around here -- pays nothing and is never affected
//     by a rebuild it does not need.
//   * It is the only way the "a child may not inherit raw I/O" rule can hold
//     without making fork()+exec() fatal; see
//     Aio::Impl::CheckNoRawRequestsInFlightOnFork().
//   * Reconstruction runs on the thread that goes on to drive the loop,
//     rather than inside an atfork handler, where async-signal-safety rules
//     make almost everything it does questionable.
//
// A relaxed atomic load never syscalls, so the per-entry-point check is
// cheap.  size_t for the same reason as aio_linux.cc's copy: this only ever
// answers "did a fork happen since I last looked".
std::atomic<size_t> global_fork_count{0};
std::once_flag atfork_once;

void RegisterAtFork() {
  std::call_once(atfork_once, []() {
    pthread_atfork(nullptr, nullptr, []() {
      global_fork_count.fetch_add(1, std::memory_order_relaxed);
    });
  });
}

// RAII wrapper around standard pipe file descriptors to simulate Linux eventfd.
//
// macOS lacks support for eventfd(). Since Aio needs a way to asynchronously
// wake up a blocking event loop (for example, to interrupt a sleeping kevent()
// call when Aio::Quit() is called from another thread), we use a pipe.
//
// The read end of the pipe is registered with kqueue. When Quit() is invoked,
// it triggers Wakeup() which writes a single byte to the write end of the pipe,
// forcing kevent() to wake up immediately so the event loop thread can check
// should_run() and terminate clean.
class EventFD {
 public:
  EventFD() {
    int pipefd[2];
    ABSL_PCHECK(pipe(pipefd) == 0) << "Failed to create pipe";
    ABSL_PCHECK(fcntl(pipefd[0], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(pipefd[1], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(pipefd[0], F_SETFD, FD_CLOEXEC) == 0);
    ABSL_PCHECK(fcntl(pipefd[1], F_SETFD, FD_CLOEXEC) == 0);
    fd_ = pipefd[0];
    write_fd_ = pipefd[1];
  }

  ~EventFD() {
    if (fd_ >= 0) {
      close(fd_);
    }
    if (write_fd_ >= 0) {
      close(write_fd_);
    }
  }

  EventFD(const EventFD &) = delete;
  EventFD &operator=(const EventFD &) = delete;

  int fd() const { return fd_; }
  int write_fd() const { return write_fd_; }

  void Write() {
    char val = 1;
    // Quit() reaches this from ShmEventLoop's SIGINT/SIGHUP/SIGTERM handler, so
    // everything here has to be async-signal-safe.  write() is.  Overwriting
    // errno is not: we interrupted a thread which may be partway through
    // checking its own errno, so put back whatever was there.
    const int saved_errno = errno;
    const ssize_t ret = write(write_fd_, &val, sizeof(val));
    const int write_errno = errno;
    errno = saved_errno;
    if (ret < 0 && write_errno != EAGAIN && write_errno != EWOULDBLOCK) {
      // ABSL_RAW_LOG formats into a stack buffer and writes the result out
      // directly, where ABSL_LOG would allocate and take locks.  The errno goes
      // out as a bare number for the same reason -- aos_strerror() formats
      // through thread_local storage with snprintf().
      ABSL_RAW_LOG(FATAL, "Failed to write to pipe: errno %d", write_errno);
    }
  }

  uint64_t eventfd_buf = 0;
  AsyncRequest wakeup_req;

 private:
  int fd_ = -1;
  int write_fd_ = -1;
};

// Internal helper union overlayed on top of AsyncRequest's opaque state buffer
// (`internal_state`).
//
// To prevent exporting backend-specific types in the public `aio.h` header,
// AsyncRequest provides a generic 16-byte buffer. This union maps it to:
// - `timespec`: Used for absolute/relative deadline timepoints.
// - `buffer`: Stores transient buffer pointer and size for simulated
// read/write.
// - `link`: Tracks next/result fields for completed queues or cancellations.
union AioState {
  struct {
    int64_t tv_sec;
    int64_t tv_nsec;
  } timespec;
  struct {
    void *ptr;
    size_t size;
  } buffer;
  struct {
    AsyncRequest *next;
    int64_t result;
  } link;
};

inline AioState &State(AsyncRequest *req) {
  static_assert(sizeof(AioState) <= sizeof(req->internal_state),
                "AioState too large");
  static_assert(alignof(AioState) <= 8, "AioState alignment mismatch");
  return *reinterpret_cast<AioState *>(req->internal_state);
}

[[maybe_unused]] inline const AioState &State(const AsyncRequest *req) {
  return *reinterpret_cast<const AioState *>(req->internal_state);
}

// Converts a monotonic clock timepoint to Mach absolute time ticks.
// kevent timer filters (`EVFILT_TIMER`) on macOS require deadline ticks
// when specified with `NOTE_MACHTIME` and `NOTE_ABSOLUTE`.
uint64_t ToMachTicks(aos::monotonic_clock::time_point time) {
  static mach_timebase_info_data_t timebase_info = []() {
    mach_timebase_info_data_t info;
    mach_timebase_info(&info);
    return info;
  }();

  uint64_t nanos = std::chrono::nanoseconds(time.time_since_epoch()).count();
  return static_cast<uint64_t>((absl::uint128(nanos) * timebase_info.denom) /
                               timebase_info.numer);
}

}  // namespace

class KqueueImpl;

struct KqueueTimerState : public Aio::TimerState {
  explicit KqueueTimerState(KqueueImpl *impl) : impl_(impl) {}
  ~KqueueTimerState() override;

  void Initialize() override;
  void Schedule(aos::monotonic_clock::time_point deadline,
                CompletionCallback callback, void *context) override;
  void Cancel(bool reap) override;

  // Order this timer was armed in, used to break deadline ties.  kqueue
  // cannot do it for us -- see KqueueImpl::EarliestExpiredTimer().
  uint64_t schedule_sequence = 0;

 private:
  KqueueImpl *impl_;
};

// KqueueImpl implements the Aio::Impl interface using macOS kqueue.
//
// Since kqueue is readiness-based, KqueueImpl emulates completion-based I/O
// (AsyncRead/AsyncWrite) by:
// 1. Stashing user buffers/contexts in the opaque AsyncRequest state.
// 2. Registering read/write filters on kqueue.
// 3. Performing the actual read() or write() system call inside kqueue callback
//    when the file descriptor is ready.
// 4. Executing the user completion callback.
//
// To satisfy real-time safety constraints, KqueueImpl uses a pre-allocated pool
// of FdRegistration objects, avoiding dynamic memory allocation on hot paths.
class KqueueImpl : public Aio::Impl {
  friend struct KqueueTimerState;

 public:
  KqueueImpl();
  ~KqueueImpl() override;

  std::unique_ptr<Aio::TimerState> MakeTimerState() override;

  void Run() override;
  bool Poll(bool block) override;
  void Quit() override;
  bool should_run() const override;
  void Wakeup();

  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) override;
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) override;
  void Cancel(AsyncRequest *request) override;
  void BeforeWait(std::function<void()> function) override;

  // Legacy Readiness API implementations (used by the Epoll interface class).
  void OnReadable(FileDescriptor fd, std::function<void()> callback) override;
  void OnError(FileDescriptor fd, std::function<void()> callback) override;
  void OnWritable(FileDescriptor fd, std::function<void()> callback) override;
  void OnEvents(FileDescriptor fd,
                std::function<void(uint32_t)> callback) override;
  void DeleteFd(FileDescriptor fd) override;
  void ForgetClosedFd(FileDescriptor fd) override;
  void EnableWritable(FileDescriptor fd) override;
  void DisableWritable(FileDescriptor fd) override;
  void SetEvents(FileDescriptor fd, uint32_t events) override;

  void RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                    std::function<void()> callback) override;
  void UnregisterThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override;
  void ConsumeThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override;

  bool HasRawRequestsInFlight() const override;

 private:
  // Rebuilds the kqueue and every registration on it, in a forked child.
  void ResetOnFork();
  // Runs ResetOnFork() exactly once per fork, the first time this loop is
  // touched after one.  Called at the top of every public entry point --
  // same shape as IoUringImpl/EpollImpl's CheckForFork() on Linux.
  void CheckForFork();
  size_t last_fork_count_ = 0;

  // Tracks active event handlers and request status for a registered file
  // descriptor.
  struct FdRegistration {
    int fd = -1;
    // Pointers to currently active completion-based asynchronous requests.
    AsyncRequest *read_req = nullptr;
    AsyncRequest *write_req = nullptr;

    // Callbacks for legacy readiness-based handlers.
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    uint32_t events = 0;

    // kqueue registration tracking state.
    bool registered = false;
    uint32_t epoll_events = 0;
    // True while this registration exists only to carry AsyncRead/AsyncWrite
    // requests (created by GetOrCreateAsyncRegistration(), no legacy
    // OnReadable/... state ever attached).  Such registrations are retired
    // back to the pool automatically when their last request finishes --
    // see MaybeRetireAsyncRegistration().  Cleared the moment a legacy
    // registration attaches, since legacy state has no "final operation"
    // and lives until DeleteFd()/ForgetClosedFd().
    bool async_only = false;
  };

  // Readiness bitfield flags mapped to mock epoll event structures.
  static constexpr uint32_t kIn = 0x01;
  static constexpr uint32_t kPri = 0x02;
  static constexpr uint32_t kOut = 0x04;
  static constexpr uint32_t kErr = 0x08;

  static constexpr uint32_t kInEvents = kIn | kPri;
  static constexpr uint32_t kOutEvents = kOut;
  static constexpr uint32_t kErrorEvents = kErr;

  // Returns the registration for fd if it is currently registered, or nullptr.
  FdRegistration *GetActiveRegistration(FileDescriptor fd) const;

  // Retrieves the registration for fd, creating a new one from the
  // pre-allocated pool if not present.
  FdRegistration &GetOrCreateLegacyRegistration(FileDescriptor fd);
  FdRegistration &GetOrCreateAsyncRegistration(FileDescriptor fd);

  // Returns a registration to the pool when it is fully cleaned up and no
  // longer needed.
  void ReleaseRegistration(FdRegistration *reg);

  // If reg is async-only and its last request just finished (completed or
  // canceled), returns it to the pool -- otherwise idle registrations
  // accumulate one pool slot per fd ever touched by AsyncRead/AsyncWrite
  // and the default 16-slot pool exhausts on the 16th distinct fd.
  // Deregisters from kqueue too (the DeleteFd() shape).  Safe to call
  // mid-dispatch: ReleaseRegistration() only parks the state on retired_.
  void MaybeRetireAsyncRegistration(FdRegistration *reg);

  // Moves everything on retired_ to free_list_, destroying the callbacks
  // parked there, and trims the pool back to its cap.  Must only run with
  // no dispatch in flight (dispatch_depth_ == 0): a parked callback can be
  // the very function whose invocation triggered the release.
  void ScrubRetiredRegistrations();

  // Syncs registration's requested events (epoll_events) with the actual kqueue
  // filters.
  // Returns false if fd turned out to be closed (kevent reported EBADF), in
  // which case nothing ends up registered for it.
  bool UpdateKqueue(FileDescriptor fd);

  // Registers the wakeup pipe read filter with kqueue.
  void SubmitWakeupRead();

  // Cancels a pending request, moving it to the pending cancels queue.
  void CancelRequest(AsyncRequest *request);

  bool RemoveFromCancels(AsyncRequest *r) {
    AsyncRequest **curr = &pending_cancels_head;
    while (*curr != nullptr) {
      if (*curr == r) {
        *curr = State(*curr).link.next;
        return true;
      }
      curr = &State(*curr).link.next;
    }
    return false;
  }

  bool RemoveFromSync(AsyncRequest *r) {
    AsyncRequest **curr = &pending_sync_completions_head;
    while (*curr != nullptr) {
      if (*curr == r) {
        *curr = State(*curr).link.next;
        return true;
      }
      curr = &State(*curr).link.next;
    }
    return false;
  }

  int kqueue_fd_ = -1;
  EventFD event_fd_;

  // Pre-allocated registration pool to guarantee no-allocation on hot paths.
  std::vector<std::unique_ptr<FdRegistration>> registrations_;
  std::vector<std::unique_ptr<FdRegistration>> free_list_;
  // Registrations released while a dispatch was in flight, still holding
  // their std::functions.  One of those functions can be the exact
  // function currently executing (a callback that deleted its own fd), so
  // they are destroyed later, by ScrubRetiredRegistrations(), never here.
  std::vector<std::unique_ptr<FdRegistration>> retired_;
  size_t initial_pool_size_ = 16;
  // Non-zero while Poll() is dispatching callbacks; see the
  // reclaim loop at the top of Poll().
  int dispatch_depth_ = 0;
  // True only while Poll() is running the before-wait functions.  Separate
  // from dispatch_depth_ because the two guard different things: BeforeWait()
  // is legal from a completion callback (where dispatch_depth_ is also
  // non-zero) and illegal only from another before-wait function.
  bool in_before_wait_ = false;
  std::vector<std::function<void()>> before_wait_functions_;

  // Quit() is documented in aio.h as callable from any thread, so these are
  // read by the polling thread while another thread writes them.  Plain bools
  // would be a data race, and the shutdown request could simply not be seen --
  // Wakeup() would break Poll() out of its wait only for should_run() to read a
  // stale value and go back in.
  // Starts true so should_run() reports "running" before the first Run(), which
  // is how the original EPoll (run_{true}) behaved.  Run() clears it on exit
  // and Quit() clears it on shutdown.
  std::atomic<bool> run_ = true;
  std::atomic<bool> quit_requested_ = false;

  // Linked list heads for delayed cancel/sync callbacks executed outside the
  // Wait.
  AsyncRequest *pending_cancels_head = nullptr;
  AsyncRequest *pending_sync_completions_head = nullptr;

  struct SignalRegistration {
    ipc_lib::ThreadSignalReceiver *receiver = nullptr;
    int signal_number = -1;
    std::function<void()> callback = nullptr;
    // Set by ResetOnFork(), delivered by the next Poll().  A signal that
    // arrives between the fork and the rebuild below is unrecoverable on
    // macOS: the receiver leaves kWakeupSignal at SIG_IGN, so the kernel
    // discards it on delivery rather than leaving it pending, and
    // EVFILT_SIGNAL only counts what is delivered while its knote exists --
    // and after a fork the whole kqueue, knote included, is gone.  Linux has
    // no such window; its signalfd is an ordinary fd that survives the fork
    // with anything already queued on it.
    //
    // ResetOnFork() cannot tell whether a wakeup was actually missed, so it
    // assumes one was.  These wakeups are explicitly allowed to be spurious
    // (see thread_signal.h -- macOS signals the whole process, so every
    // receiver in it already wakes on a wakeup meant for one thread), while a
    // dropped one hangs the loop outright.
    bool wakeup_pending_after_fork = false;
  };
  std::unordered_map<int, SignalRegistration> signals_;

  // Double-linked list of scheduled timers.
  void InsertActiveTimer(Aio::TimerState *state);
  void RemoveActiveTimer(Aio::TimerState *state);
  Aio::TimerState *active_timers_head_ = nullptr;

  // The armed timer that is due first, ties broken by the order the timers
  // were armed in, or nullptr if none is due yet.  See its definition for why
  // Poll() picks the timer to dispatch this way instead of trusting the
  // identity kqueue reported.
  Aio::TimerState *EarliestExpiredTimer();
  // Adds/removes a timer's one-shot EVFILT_TIMER knote.  Poll() needs both
  // when it dispatches a timer other than the one kqueue handed it.
  void ArmTimer(Aio::TimerState *state);
  void DisarmTimer(Aio::TimerState *state);
  // Handed out by Schedule() to order timers armed for the same deadline.
  uint64_t next_timer_sequence_ = 0;
};

// A member definition cannot sit in the anonymous namespace the old
// AtForkHandler lived in -- that namespace does not enclose KqueueImpl.
void KqueueImpl::CheckForFork() {
  size_t current_fork_count = global_fork_count.load(std::memory_order_relaxed);
  if (last_fork_count_ != current_fork_count) {
    // Record the new count *before* ResetOnFork(): it re-submits work through
    // the same public entry points (e.g. the wakeup read via AsyncRead), which
    // call CheckForFork() again -- if the count weren't updated first, that
    // re-entrant call would fire ResetOnFork() once more and recurse forever.
    last_fork_count_ = current_fork_count;
    ResetOnFork();
  }
}

std::unique_ptr<Aio::TimerState> KqueueImpl::MakeTimerState() {
  return std::make_unique<KqueueTimerState>(this);
}

KqueueImpl::KqueueImpl() {
  kqueue_fd_ = kqueue();
  ABSL_PCHECK(kqueue_fd_ >= 0) << "Failed to create kqueue instance";
  ABSL_PCHECK(fcntl(kqueue_fd_, F_SETFD, FD_CLOEXEC) == 0);

  initial_pool_size_ = absl::GetFlag(FLAGS_aio_epoll_pool_size);
  free_list_.reserve(initial_pool_size_ * 2);
  for (size_t i = 0; i < initial_pool_size_; ++i) {
    free_list_.push_back(std::make_unique<FdRegistration>());
  }
  registrations_.reserve(initial_pool_size_ * 2);
  // Retiring happens on the dispatch path, which runs under RT -- the push
  // must never grow the vector there.
  retired_.reserve(initial_pool_size_ * 2);

  // When wakeup event fd read completes, re-schedule it.
  event_fd_.wakeup_req.callback = [](Completion completion, void *context) {
    auto *impl = static_cast<KqueueImpl *>(context);
    if (aos::IsOk(completion.status)) {
      impl->SubmitWakeupRead();
    }
  };
  event_fd_.wakeup_req.context = this;

  RegisterAtFork();
  last_fork_count_ = global_fork_count.load(std::memory_order_relaxed);

  SubmitWakeupRead();
}

KqueueImpl::~KqueueImpl() {
  run_ = false;
  if (kqueue_fd_ >= 0) {
    close(kqueue_fd_);
  }
}

void KqueueImpl::InsertActiveTimer(Aio::TimerState *state) {
  if (state->is_active) return;
  state->is_active = true;
  state->next_active = active_timers_head_;
  state->prev_active = nullptr;
  if (active_timers_head_) {
    active_timers_head_->prev_active = state;
  }
  active_timers_head_ = state;
}

bool KqueueImpl::HasRawRequestsInFlight() const {
  for (const auto &reg : registrations_) {
    if (!reg) continue;
    // The loop's own wakeup read goes through the public AsyncRead()
    // (SubmitWakeupRead()) and is outstanding for the loop's whole life, so
    // counting it would refuse every fork.  ResetOnFork() re-arms it, so it
    // is not a caller's request to lose.  Both Linux backends draw the same
    // line.
    if (reg->read_req != nullptr && reg->read_req != &event_fd_.wakeup_req) {
      return true;
    }
    if (reg->write_req != nullptr) {
      return true;
    }
  }
  return false;
}

void KqueueImpl::RemoveActiveTimer(Aio::TimerState *state) {
  if (!state->is_active) return;
  state->is_active = false;
  if (state->prev_active) {
    state->prev_active->next_active = state->next_active;
  } else {
    active_timers_head_ = state->next_active;
  }
  if (state->next_active) {
    state->next_active->prev_active = state->prev_active;
  }
  state->prev_active = nullptr;
  state->next_active = nullptr;
}

void KqueueImpl::ResetOnFork() {
  // Shared rule; see Aio::Impl::CheckNoRawRequestsInFlightOnFork().  Here
  // the failure it prevents is a duplicated write: the registrations re-armed
  // below survive a fork, so without this the child would repeat a read or
  // write the parent is also still doing.  Reached only from CheckForFork(),
  // so only a child that actually uses the loop can trip it.
  CheckNoRawRequestsInFlightOnFork();

  close(kqueue_fd_);
  kqueue_fd_ = kqueue();
  ABSL_PCHECK(kqueue_fd_ >= 0) << "Failed to recreate kqueue on fork";
  ABSL_PCHECK(fcntl(kqueue_fd_, F_SETFD, FD_CLOEXEC) == 0);

  // Re-register all signals
  for (auto &pair : signals_) {
    struct kevent sig_ev;
    EV_SET(&sig_ev, pair.first, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0, NULL);
    ABSL_PCHECK(kevent(kqueue_fd_, &sig_ev, 1, NULL, 0, NULL) == 0);
    // Nothing tells us whether a wakeup landed in the window this rebuild
    // closes, so assume one did -- see wakeup_pending_after_fork.
    pair.second.wakeup_pending_after_fork = true;
  }

  // Re-register all active timers
  Aio::TimerState *timer = active_timers_head_;
  while (timer != nullptr) {
    if (!timer->request.done) {
      struct kevent ev;
      uint64_t mach_ticks = ToMachTicks(timer->deadline);
      EV_SET(&ev, reinterpret_cast<uintptr_t>(timer), EVFILT_TIMER,
             EV_ADD | EV_ENABLE | EV_ONESHOT, NOTE_MACHTIME | NOTE_ABSOLUTE,
             mach_ticks, timer);
      ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
    }
    timer = timer->next_active;
  }

  // Re-register all active fds
  for (const auto &reg : registrations_) {
    if (!reg) continue;
    reg->registered = false;
    reg->epoll_events = 0;
    UpdateKqueue(reg->fd);
  }
}

KqueueImpl::FdRegistration *KqueueImpl::GetActiveRegistration(
    FileDescriptor fd) const {
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  if (it != registrations_.end() && (*it)->fd == fd) {
    return it->get();
  }
  return nullptr;
}

KqueueImpl::FdRegistration &KqueueImpl::GetOrCreateLegacyRegistration(
    FileDescriptor fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    // Legacy state is attaching to a registration that may have been created
    // for an async request: it now lives until DeleteFd()/ForgetClosedFd(),
    // so it must no longer auto-retire when an async request finishes.
    reg->async_only = false;
    return *reg;
  }
  auto new_reg = std::make_unique<FdRegistration>();
  auto *ptr = new_reg.get();
  ptr->fd = fd;
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  registrations_.insert(it, std::move(new_reg));
  return *ptr;
}

KqueueImpl::FdRegistration &KqueueImpl::GetOrCreateAsyncRegistration(
    FileDescriptor fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    return *reg;
  }
  ABSL_CHECK(!free_list_.empty()) << "Async registration pool exhausted";
  auto owned_reg = std::move(free_list_.back());
  free_list_.pop_back();
  auto *ptr = owned_reg.get();
  ptr->fd = fd;
  ptr->async_only = true;
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  registrations_.insert(it, std::move(owned_reg));
  return *ptr;
}

void KqueueImpl::ReleaseRegistration(FdRegistration *reg) {
  auto it =
      std::lower_bound(registrations_.begin(), registrations_.end(), reg->fd,
                       [](const std::unique_ptr<FdRegistration> &r, int value) {
                         return r->fd < value;
                       });
  ABSL_CHECK(it != registrations_.end() && it->get() == reg);

  std::unique_ptr<FdRegistration> owned_reg = std::move(*it);
  registrations_.erase(it);

  owned_reg->fd = -1;
  owned_reg->read_req = nullptr;
  owned_reg->write_req = nullptr;
  owned_reg->events = 0;
  owned_reg->registered = false;
  owned_reg->epoll_events = 0;

  // The std::functions are deliberately NOT cleared here: one of them can
  // be the very function currently executing -- a callback that calls
  // DeleteFd() on its own fd lands here with its own lambda on the stack,
  // and destroying that std::function frees the lambda's captures out
  // from under the running code (a heap-use-after-free the moment it
  // touches one, confirmed under ASAN).  Park the registration on
  // retired_ instead; ScrubRetiredRegistrations() destroys the callbacks
  // and recycles the slot once no dispatch is in flight.  For the same
  // reason nothing is freed or reused here: Poll()'s dispatch may still
  // be holding a pointer to this registration and re-reading reg->fd
  // after each callback to notice exactly this deletion.
  retired_.push_back(std::move(owned_reg));
  if (dispatch_depth_ == 0) {
    ScrubRetiredRegistrations();
  }
}

void KqueueImpl::MaybeRetireAsyncRegistration(FdRegistration *reg) {
  if (!reg->async_only || reg->read_req != nullptr ||
      reg->write_req != nullptr) {
    return;
  }
  if (reg->registered) {
    struct kevent events[2];
    int n = 0;
    if (reg->epoll_events & (kIn | kPri | kErr)) {
      EV_SET(&events[n++], reg->fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    if (reg->epoll_events & (kOut | kErr)) {
      EV_SET(&events[n++], reg->fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    if (n > 0) {
      // A closed fd has had its filters dropped by the kernel already, which
      // is not an error worth dying on -- same reasoning as DeleteFd().
      kevent(kqueue_fd_, events, n, NULL, 0, NULL);
    }
  }
  ReleaseRegistration(reg);
}

void KqueueImpl::ScrubRetiredRegistrations() {
  ABSL_CHECK_EQ(dispatch_depth_, 0);
  for (auto &reg : retired_) {
    reg->in_fn = nullptr;
    reg->out_fn = nullptr;
    reg->err_fn = nullptr;
    reg->events_fn = nullptr;
    reg->async_only = false;
    free_list_.push_back(std::move(reg));
  }
  retired_.clear();
  while (free_list_.size() > 2 * initial_pool_size_) {
    free_list_.pop_back();
  }
}

void KqueueImpl::Run() {
  if (quit_requested_) {
    quit_requested_ = false;
    return;
  }
  run_ = true;
  // Block while we are running; once Quit() lands, switch to non-blocking
  // polls so whatever is already queued gets flushed before Run() returns.
  //
  // This is what EPoll::Run() did from 2019 (6b6dfa5a9, "This lets us flush the
  // event queue before quitting") until it was rewritten to wrap Aio, which
  // silently dropped it.  It survived two refactors with the comment intact,
  // and when a repeated-Quit() hang was found in 2021 (f74daa655) it was the
  // Quit() side that got guarded rather than the drain removed -- so the
  // behavior is deliberate and worth keeping.
  //
  // should_run() rather than run_ alone is what makes a concurrent Quit() safe.
  // Quit() is async-safe, so it can run in its entirety between the
  // quit_requested_ check above and the `run_ = true` just above here.  Quit()
  // stores run_ = false, and that store is then overwritten here -- so run_
  // no longer records that a shutdown was asked for.  quit_requested_ still
  // does, because nothing clears it between Quit() setting it and this loop
  // reading it.  Reading run_ by itself would drop the request and then block
  // here forever: the wakeup Quit() sent has already been spent, so nothing is
  // left to bring Poll() back.
  //
  // Quit()'s own store order matters for the same reason -- it sets both flags
  // before calling Wakeup(), so a Poll() woken by that wakeup is guaranteed to
  // see them.
  while (true) {
    if (!Poll(should_run())) {
      // Poll() found nothing to do.  If a shutdown was requested, the queue is
      // drained now and we are done; otherwise Poll() just came back early
      // (EINTR) and we go back to waiting.
      if (!should_run()) {
        break;
      }
    }
  }
  // run_ first: should_run() must not still report "running" once Run() has
  // returned.  A Quit() landing between these two stores is cleared along with
  // them, which cannot hang anything -- the loop has already stopped -- it only
  // means the next Run() won't return early.
  run_ = false;
  quit_requested_ = false;
}

bool KqueueImpl::Poll(bool block) {
  // Not reentrant, matching IoUringImpl::Poll().  dispatch_depth_ covers
  // the whole body below, before-wait functions included.
  ABSL_CHECK_EQ(dispatch_depth_, 0)
      << "Aio::Poll() reentered from inside a completion callback or "
         "before-wait function; wait by returning to the event loop instead";

  CheckForFork();
  // Reclaim registrations retired by earlier dispatches.
  // ReleaseRegistration() deliberately does not scrub or free them itself: a
  // callback is allowed to delete its own fd (a timer callback destroying its
  // timer reaches ~KqueueTimerState -> DeleteFd()), and the dispatch below
  // keeps re-reading reg->fd after each callback to notice exactly that.
  // Recycling inside the callback turns those reads into use-after-frees, and
  // the reg->fd != -1 guard cannot help -- it is reading the recycled memory to
  // make its decision.  Destroying the parked std::functions there is just as
  // bad: one of them can be the very function running.  Here no dispatch is in
  // flight (the reentrancy CHECK above), so nothing holds a registration
  // pointer and no retired callback is on the stack.
  ScrubRetiredRegistrations();
  struct DispatchDepth {
    int *depth;
    explicit DispatchDepth(int *d) : depth(d) { ++*depth; }
    ~DispatchDepth() { --*depth; }
  } dispatch_depth_guard(&dispatch_depth_);

  // Registering a before-wait function from inside one is disallowed --
  // see KqueueImpl::BeforeWait().
  in_before_wait_ = true;
  for (const auto &fn : before_wait_functions_) {
    fn();
  }
  in_before_wait_ = false;

  bool processed = false;

  // At most one completion callback per Poll() -- see Aio::Poll().  Each of
  // these delivers one and returns; the rest of the queue waits for the next
  // Poll(), the same way io_uring's pending_dispatch_ does.  The loops keep
  // popping only past entries that deliver nothing (already resolved, or no
  // callback at all), which is not an observable dispatch.

  // Handle any pending cancels first to complete them.
  while (pending_cancels_head != nullptr) {
    auto *req = pending_cancels_head;
    pending_cancels_head = State(req).link.next;

    if (req->done) {
      continue;
    }
    req->done = true;
    if (req->callback) {
      req->callback(Completion{aos::MakeError("Canceled"), 0, req->user_data},
                    req->context);
      return true;
    }
  }

  // Process synchronous completions.
  while (pending_sync_completions_head != nullptr) {
    auto *req = pending_sync_completions_head;
    pending_sync_completions_head = State(req).link.next;

    if (req->done) {
      continue;
    }
    req->done = true;
    if (req->callback) {
      int64_t res = State(req).link.result;
      Completion completion;
      completion.user_data = req->user_data;
      if (res >= 0) {
        completion.status = aos::Ok();
        completion.result = static_cast<int32_t>(res);
      } else {
        completion.status = aos::MakeError("kqueue error");
        completion.result = static_cast<int32_t>(-res);
      }
      req->callback(completion, req->context);
      return true;
    }
  }

  // Deliver the post-fork signal wakeups ResetOnFork() queued.  Same shape as
  // the EVFILT_SIGNAL dispatch below, so a consumer cannot tell the two apart.
  for (auto &pair : signals_) {
    if (!pair.second.wakeup_pending_after_fork) {
      continue;
    }
    pair.second.wakeup_pending_after_fork = false;
    if (pair.second.callback) {
      pair.second.callback();
      return true;
    }
  }

  struct kevent event;
  struct timespec timeout;
  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;
  struct timespec *timeout_ptr = block ? nullptr : &timeout;
  int num_events;
  do {
    num_events = kevent(kqueue_fd_, NULL, 0, &event, 1, timeout_ptr);
  } while (num_events == -1 && errno == EINTR && block);

  if (num_events == -1) {
    if (errno == EINTR) {
      return processed;
    }
    ABSL_PCHECK(num_events != -1);
  }

  if (num_events > 0) {
    processed = true;
    if (event.filter == EVFILT_TIMER) {
      auto *reported = static_cast<Aio::TimerState *>(event.udata);
      // EV_ONESHOT already took `reported` off the kqueue, but it is not
      // necessarily the one that is due first -- see EarliestExpiredTimer().
      auto *state = EarliestExpiredTimer();
      if (state == nullptr) {
        // Nothing looks due by aos::monotonic_clock even though kqueue says
        // otherwise (the two clocks can straddle a deadline by a hair).  Go
        // with what the kernel reported.
        state = reported;
      } else if (state != reported) {
        // Dispatching out of the order kqueue offered: take the timer we
        // picked off the kqueue, since it is being delivered now, and put
        // `reported` back, since it is not.  Its deadline is already past, so
        // re-arming delivers it on a later Poll().
        DisarmTimer(state);
        if (reported != nullptr && !reported->request.done) {
          ArmTimer(reported);
        }
      }
      if (state && !state->request.done) {
        // Resolve before dispatching -- the callback may destroy this timer.
        state->request.done = true;
        RemoveActiveTimer(state);
        if (state->user_callback) {
          Completion completion;
          completion.user_data = state->request.user_data;
          completion.status = aos::Ok();
          completion.result = 0;
          state->user_callback(completion, state->user_context);
        }
      }
    } else if (event.filter == EVFILT_SIGNAL) {
      int sig_num = event.ident;
      auto it = signals_.find(sig_num);
      if (it != signals_.end() && it->second.callback) {
        it->second.callback();
      }
    } else {
      auto *reg = static_cast<FdRegistration *>(event.udata);
      uint32_t got_events = 0;
      if (event.filter == EVFILT_READ) {
        got_events |= kIn;
        if (event.flags & EV_EOF) {
          got_events |= kErr;
        }
      } else if (event.filter == EVFILT_WRITE) {
        got_events |= kOut;
        if (event.flags & EV_EOF) {
          got_events |= kErr;
        }
      }
      if (event.flags & EV_ERROR) {
        got_events |= kErr;
      }

      uint32_t events = 0;
      if (got_events & kIn) events |= kIn;
      if (got_events & kOut) events |= kOut;
      if (got_events & kErr) events |= kErr;

      if (events & kInEvents) {
        if (reg->in_fn) {
          reg->in_fn();
        }
      }

      if (reg->fd != -1 && (events & kOutEvents)) {
        if (reg->out_fn) {
          reg->out_fn();
        }
      }

      if (reg->fd != -1 && (events & kErrorEvents)) {
        if (reg->err_fn) {
          reg->err_fn();
        }
      }

      if (reg->fd != -1 && reg->events_fn) {
        // Report only what SetEvents() asked for.  Our kqueue registration is
        // broader than the caller's interest mask -- a read filter stays on to
        // report errors even when kIn wasn't requested -- and epoll would never
        // hand back a readiness bit that wasn't subscribed to.  Error bits are
        // unconditional there, so they pass through here too.
        //
        // kPri widens to kIn exactly as UpdateKqueue() does when registering:
        // kqueue has no way to tell priority data apart, so a kPri subscriber
        // gets the read filter and has to be told when it fires.  Dropping
        // that instead would leave the level-triggered filter reporting
        // readiness nobody ever consumes, and Poll() would spin.
        uint32_t interest = reg->events;
        if (interest & kPri) {
          interest |= kIn;
        }
        const uint32_t requested_events = events & (interest | kErrorEvents);
        if (requested_events != 0) {
          reg->events_fn(requested_events);
        }
      }
    }
  }

  return processed;
}

void KqueueImpl::Quit() {
  // Already asked to stop.  Bail out rather than re-arming the wakeup: once
  // Run() is draining it polls with a zero timeout, so a Quit() called from a
  // BeforeWait callback (or any other per-Poll path) would refill the queue
  // every time around and the drain would never finish.  This is the 2021
  // EPoll::Quit() guard -- f74daa655, "Make EPoll actually return from Run even
  // if you call Quit repeatedly" -- which the drain has always needed.
  //
  // Suppressing the wakeup is safe: quit_requested_ is only cleared by Run() on
  // its way out, so while it is set the loop has either already been woken or
  // is in the non-blocking drain and cannot block again.
  if (quit_requested_) {
    return;
  }

  quit_requested_ = true;
  run_ = false;
  Wakeup();
}

bool KqueueImpl::should_run() const { return run_ && !quit_requested_; }

void KqueueImpl::Wakeup() { event_fd_.Write(); }

void KqueueImpl::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                           AsyncRequest *request) {
  CheckForFork();
  request->done = false;
  if (fd < 0) {
    State(request).link.next = pending_sync_completions_head;
    State(request).link.result = -EBADF;
    pending_sync_completions_head = request;
    return;
  }
  auto &reg = GetOrCreateAsyncRegistration(fd);

  ABSL_CHECK(reg.events_fn == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(reg.in_fn == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  ABSL_CHECK(reg.read_req == nullptr) << "Duplicate AsyncRead on fd " << fd;
  reg.read_req = request;

  State(request).buffer.ptr = buffer.data();
  State(request).buffer.size = buffer.size();

  reg.in_fn = [this, fd]() {
    // Copy the captures onto the stack up front.  The paths below can destroy
    // this very closure (clearing r->in_fn, or retiring the whole
    // registration), and with it `this` and `fd`; everything after that point
    // has to run off locals (and plain pointees like req).
    KqueueImpl *const impl = this;
    const FileDescriptor read_fd = fd;

    auto *r = impl->GetActiveRegistration(read_fd);
    if (r == nullptr) return;
    auto *req = r->read_req;
    if (!req) return;
    char *data = static_cast<char *>(State(req).buffer.ptr);
    size_t size = State(req).buffer.size;
    ssize_t res = read(read_fd, data, size);
    if (res >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      // Before any kevent() below can clobber it.
      const int read_errno = errno;
      r->read_req = nullptr;
      req->done = true;
      if (r->async_only && r->write_req == nullptr) {
        // Final operation on an async-only registration: recycle the pool
        // slot.  This only parks the registration (this executing closure
        // included) on retired_ -- nothing is destroyed until dispatch is
        // over.
        impl->MaybeRetireAsyncRegistration(r);
      } else {
        // Legacy state shares this registration; just detach the read.  The
        // assignment destroys this closure -- locals only from here.
        r->in_fn = nullptr;
        impl->UpdateKqueue(read_fd);
      }
      if (req->callback) {
        Completion completion;
        completion.user_data = req->user_data;
        if (res >= 0) {
          completion.status = aos::Ok();
          completion.result = static_cast<int32_t>(res);
        } else {
          completion.status = aos::MakeError("kqueue error");
          completion.result = static_cast<int32_t>(read_errno);
        }
        req->callback(completion, req->context);
      }
    }
  };

  if (!UpdateKqueue(fd)) {
    // fd is open as far as the caller knows but closed as far as the kernel is
    // concerned.  Report that the same way a negative fd does, rather than
    // aborting: a completion-based caller expects an EBADF completion.
    reg.read_req = nullptr;
    reg.in_fn = nullptr;
    MaybeRetireAsyncRegistration(&reg);
    State(request).link.next = pending_sync_completions_head;
    State(request).link.result = -EBADF;
    pending_sync_completions_head = request;
  }
}

void KqueueImpl::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                            AsyncRequest *request) {
  CheckForFork();
  request->done = false;
  if (fd < 0) {
    State(request).link.next = pending_sync_completions_head;
    State(request).link.result = -EBADF;
    pending_sync_completions_head = request;
    return;
  }
  auto &reg = GetOrCreateAsyncRegistration(fd);

  ABSL_CHECK(reg.events_fn == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(reg.out_fn == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  ABSL_CHECK(reg.write_req == nullptr) << "Duplicate AsyncWrite on fd " << fd;
  reg.write_req = request;

  State(request).buffer.ptr = const_cast<char *>(buffer.data());
  State(request).buffer.size = buffer.size();

  reg.out_fn = [this, fd]() {
    // Same capture discipline as AsyncRead()'s closure -- see there.
    KqueueImpl *const impl = this;
    const FileDescriptor write_fd = fd;

    auto *r = impl->GetActiveRegistration(write_fd);
    if (r == nullptr) return;
    auto *req = r->write_req;
    if (!req) return;
    const char *data = static_cast<const char *>(State(req).buffer.ptr);
    size_t size = State(req).buffer.size;
    ssize_t res = write(write_fd, data, size);
    if (res >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      const int write_errno = errno;
      r->write_req = nullptr;
      req->done = true;
      if (r->async_only && r->read_req == nullptr) {
        impl->MaybeRetireAsyncRegistration(r);
      } else {
        r->out_fn = nullptr;
        impl->UpdateKqueue(write_fd);
      }
      if (req->callback) {
        Completion completion;
        completion.user_data = req->user_data;
        if (res >= 0) {
          completion.status = aos::Ok();
          completion.result = static_cast<int32_t>(res);
        } else {
          completion.status = aos::MakeError("kqueue error");
          completion.result = static_cast<int32_t>(write_errno);
        }
        req->callback(completion, req->context);
      }
    }
  };

  if (!UpdateKqueue(fd)) {
    // See the matching comment in AsyncRead().
    reg.write_req = nullptr;
    reg.out_fn = nullptr;
    MaybeRetireAsyncRegistration(&reg);
    State(request).link.next = pending_sync_completions_head;
    State(request).link.result = -EBADF;
    pending_sync_completions_head = request;
  }
}

void KqueueImpl::Cancel(AsyncRequest *request) {
  CheckForFork();
  CancelRequest(request);
}

void KqueueImpl::BeforeWait(std::function<void()> function) {
  // Same-thread only, like every other registration call: Poll() iterates
  // before_wait_functions_, so a push_back from another thread would race it.
  // (CheckForFork() also enforces that -- see there.)
  CheckForFork();
  // Not from inside a before-wait function: the push_back can reallocate the
  // vector while Poll()'s iteration is executing an element in the old
  // storage.  Deterministically illegal rather than sometimes-corrupting --
  // unguarded, this is a use-after-free of the running std::function, and it
  // reproduces as a SIGSEGV the moment the callback touches a capture.
  // Same contract, and the same message, as IoUringImpl::BeforeWait().
  ABSL_CHECK(!in_before_wait_)
      << ": BeforeWait() may not be called from a before-wait function";
  before_wait_functions_.push_back(std::move(function));
}

void KqueueImpl::OnReadable(FileDescriptor fd, std::function<void()> callback) {
  CheckForFork();
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  ABSL_CHECK(reg.read_req == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  if (reg.in_fn) {
    ABSL_CHECK(!callback) << "Duplicate in functions for " << fd;
  }
  reg.in_fn = std::move(callback);

  reg.events |= kInEvents;
  UpdateKqueue(fd);
}

void KqueueImpl::OnError(FileDescriptor fd, std::function<void()> callback) {
  CheckForFork();
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn) << "Cannot mix OnEvents and OnError for fd " << fd;
  if (reg.err_fn) {
    ABSL_CHECK(!callback) << "Duplicate error functions for " << fd;
  }
  reg.err_fn = std::move(callback);

  reg.events |= kErrorEvents;
  UpdateKqueue(fd);
}

void KqueueImpl::OnWritable(FileDescriptor fd, std::function<void()> callback) {
  CheckForFork();
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  ABSL_CHECK(reg.write_req == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  if (reg.out_fn) {
    ABSL_CHECK(!callback) << "Duplicate out functions for " << fd;
  }
  reg.out_fn = std::move(callback);

  reg.events |= kOutEvents;
  UpdateKqueue(fd);
}

void KqueueImpl::OnEvents(FileDescriptor fd,
                          std::function<void(uint32_t)> callback) {
  CheckForFork();
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(reg.read_req == nullptr && reg.write_req == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!reg.in_fn && !reg.out_fn && !reg.err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!reg.events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  reg.events_fn = std::move(callback);
}

void KqueueImpl::DeleteFd(FileDescriptor fd) {
  CheckForFork();
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";

  if (reg->registered) {
    struct kevent events[2];
    int n = 0;
    if (reg->epoll_events & (kIn | kPri | kErr)) {
      EV_SET(&events[n++], fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    if (reg->epoll_events & (kOut | kErr)) {
      EV_SET(&events[n++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    if (n > 0) {
      kevent(kqueue_fd_, events, n, NULL, 0, NULL);
    }
  }

  ReleaseRegistration(reg);
}

void KqueueImpl::ForgetClosedFd(FileDescriptor fd) {
  CheckForFork();
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";

  ReleaseRegistration(reg);
}

void KqueueImpl::EnableWritable(FileDescriptor fd) {
  CheckForFork();
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn) << "EnableWritable is only for fds registered "
                                 "using OnWritable, not OnEvents";

  uint32_t new_events = reg->events | kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateKqueue(fd);
  }
}

void KqueueImpl::DisableWritable(FileDescriptor fd) {
  CheckForFork();
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn) << "DisableWritable is only for fds registered "
                                 "using OnWritable, not OnEvents";

  uint32_t new_events = reg->events & ~kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateKqueue(fd);
  }
}

void KqueueImpl::SetEvents(FileDescriptor fd, uint32_t events) {
  CheckForFork();
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(reg->events_fn)
      << "SetEvents is only for fds registered using OnEvents";

  if (reg->events != events) {
    reg->events = events;
    UpdateKqueue(fd);
  }
}

void KqueueImpl::RegisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver, std::function<void()> callback) {
  CheckForFork();
  int sig_num = ipc_lib::kWakeupSignal;
  auto [it, inserted] = signals_.try_emplace(sig_num);
  ABSL_CHECK(inserted) << "Duplicate signal registration for receiver "
                       << receiver;

  it->second.receiver = receiver;
  it->second.signal_number = sig_num;
  it->second.callback = std::move(callback);

  struct kevent ev;
  EV_SET(&ev, sig_num, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0, NULL);
  ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
}

void KqueueImpl::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  CheckForFork();
  int sig_num = ipc_lib::kWakeupSignal;
  auto it = signals_.find(sig_num);
  ABSL_CHECK(it != signals_.end() && it->second.receiver == receiver)
      << "ThreadSignalReceiver not found";

  struct kevent ev;
  EV_SET(&ev, sig_num, EVFILT_SIGNAL, EV_DELETE, 0, 0, NULL);
  kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL);

  signals_.erase(it);
}

void KqueueImpl::ConsumeThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  CheckForFork();
  (void)receiver;
  // kqueue automatically consumes the signal event when returning it.
}

KqueueTimerState::~KqueueTimerState() { Cancel(true); }

void KqueueTimerState::Initialize() { request.done = true; }

void KqueueTimerState::Schedule(aos::monotonic_clock::time_point deadline,
                                CompletionCallback callback, void *context) {
  impl_->CheckForFork();
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  Cancel(true);

  this->deadline = deadline;
  this->user_callback = callback;
  this->user_context = context;
  this->request.user_data = this;
  this->request.done = false;
  this->schedule_sequence = impl_->next_timer_sequence_++;
  impl_->InsertActiveTimer(this);

  impl_->ArmTimer(this);
}

void KqueueTimerState::Cancel(bool /*reap*/) {
  impl_->CheckForFork();
  impl_->DisarmTimer(this);

  impl_->RemoveActiveTimer(this);
  request.done = true;
  user_callback = nullptr;
}

void KqueueImpl::ArmTimer(Aio::TimerState *state) {
  struct kevent ev;
  uint64_t mach_ticks = ToMachTicks(state->deadline);
  EV_SET(&ev, reinterpret_cast<uintptr_t>(state), EVFILT_TIMER,
         EV_ADD | EV_ENABLE | EV_ONESHOT, NOTE_MACHTIME | NOTE_ABSOLUTE,
         mach_ticks, state);

  ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
}

void KqueueImpl::DisarmTimer(Aio::TimerState *state) {
  struct kevent ev;
  EV_SET(&ev, reinterpret_cast<uintptr_t>(state), EVFILT_TIMER, EV_DELETE, 0, 0,
         NULL);
  // ENOENT just means the one-shot already fired and took itself off.
  kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL);
}

// kqueue hands back timers that came due together most-recently-armed first --
// its knote list is a stack -- so the raw event order is the reverse of the
// order they were scheduled in.  Both Linux backends deliver the earliest
// deadline first (each timer is its own timerfd, and epoll/io_uring report
// them in the order the kernel made them ready), and callers get to rely on
// that: a callback firing for the earlier deadline may cancel, reschedule, or
// destroy a later timer whose firing is already queued.
//
// So pick the timer to dispatch here rather than trusting the identity kqueue
// reported.  Every deadline is absolute and recorded on the timer, which makes
// "already due" a property of the clock rather than of which knote the kernel
// happened to hand back.
Aio::TimerState *KqueueImpl::EarliestExpiredTimer() {
  const aos::monotonic_clock::time_point now = aos::monotonic_clock::now();
  Aio::TimerState *best = nullptr;
  for (Aio::TimerState *timer = active_timers_head_; timer != nullptr;
       timer = timer->next_active) {
    if (timer->request.done || timer->deadline > now) {
      continue;
    }
    if (best == nullptr || timer->deadline < best->deadline ||
        (timer->deadline == best->deadline &&
         static_cast<KqueueTimerState *>(timer)->schedule_sequence <
             static_cast<KqueueTimerState *>(best)->schedule_sequence)) {
      best = timer;
    }
  }
  return best;
}

bool KqueueImpl::UpdateKqueue(FileDescriptor fd) {
  auto *reg = GetActiveRegistration(fd);
  if (!reg) return true;

  uint32_t desired_events = 0;
  if (reg->read_req) desired_events |= kIn;
  if (reg->write_req) desired_events |= kOut;

  if (reg->events & kIn) desired_events |= kIn;
  if (reg->events & kPri) desired_events |= kIn;
  if (reg->events & kOut) desired_events |= kOut;
  if (reg->events & kErr) desired_events |= kErr;

  bool want_read = (desired_events & kIn) || (desired_events & kErr);
  bool has_read = reg->registered &&
                  ((reg->epoll_events & kIn) || (reg->epoll_events & kErr));

  // EV_CLEAR is only set when the filter exists purely to report errors, so it
  // has to be re-evaluated whenever that changes -- not just when the filter is
  // added or removed.  Going from error-only to readable keeps want_read true,
  // and without this the filter would stay edge-triggered and we'd miss
  // readiness; going the other way would leave it level-triggered and spin.
  // EV_ADD on an already-registered filter updates its flags in place.
  const bool want_read_clear = want_read && !(desired_events & kIn);
  const bool has_read_clear = has_read && !(reg->epoll_events & kIn);

  if (want_read != has_read ||
      (want_read && want_read_clear != has_read_clear)) {
    struct kevent ev;
    if (want_read) {
      int flags = EV_ADD | EV_ENABLE;
      if (want_read_clear) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, fd, EVFILT_READ, flags, 0, 0, reg);
    } else {
      EV_SET(&ev, fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    if (kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) != 0) {
      // A descriptor closed behind our back is the caller's problem to report,
      // not ours to die on -- io_uring answers a request on a closed fd with an
      // EBADF completion, and this backend has to match.  (The kernel drops a
      // closed fd's filters itself, so ENOENT on the EV_DELETE path just means
      // it got there first.)
      if (errno == EBADF || errno == ENOENT) {
        reg->registered = false;
        reg->epoll_events = 0;
        return false;
      }
      ABSL_PLOG(FATAL) << "kevent failed for fd " << fd;
    }
  }

  bool want_write = (desired_events & kOut) || (desired_events & kErr);
  bool has_write = reg->registered &&
                   ((reg->epoll_events & kOut) || (reg->epoll_events & kErr));

  // Same EV_CLEAR re-evaluation as the read filter above.
  const bool want_write_clear = want_write && !(desired_events & kOut);
  const bool has_write_clear = has_write && !(reg->epoll_events & kOut);

  if (want_write != has_write ||
      (want_write && want_write_clear != has_write_clear)) {
    struct kevent ev;
    if (want_write) {
      int flags = EV_ADD | EV_ENABLE;
      if (want_write_clear) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, fd, EVFILT_WRITE, flags, 0, 0, reg);
    } else {
      EV_SET(&ev, fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    if (kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) != 0) {
      // A descriptor closed behind our back is the caller's problem to report,
      // not ours to die on -- io_uring answers a request on a closed fd with an
      // EBADF completion, and this backend has to match.  (The kernel drops a
      // closed fd's filters itself, so ENOENT on the EV_DELETE path just means
      // it got there first.)
      if (errno == EBADF || errno == ENOENT) {
        reg->registered = false;
        reg->epoll_events = 0;
        return false;
      }
      ABSL_PLOG(FATAL) << "kevent failed for fd " << fd;
    }
  }

  reg->registered = (want_read || want_write);
  reg->epoll_events = desired_events;
  return true;
}

void KqueueImpl::SubmitWakeupRead() {
  AsyncRead(event_fd_.fd(),
            std::span<char>(reinterpret_cast<char *>(&event_fd_.eventfd_buf),
                            sizeof(event_fd_.eventfd_buf)),
            &event_fd_.wakeup_req);
}

void KqueueImpl::CancelRequest(AsyncRequest *request) {
  if (request->done) return;

  RemoveFromCancels(request);
  RemoveFromSync(request);

  bool is_read = false;
  bool is_write = false;
  int found_fd = -1;
  for (const auto &reg : registrations_) {
    if (reg->read_req == request) {
      is_read = true;
      found_fd = reg->fd;
      break;
    }
    if (reg->write_req == request) {
      is_write = true;
      found_fd = reg->fd;
      break;
    }
  }

  if (is_read) {
    auto *reg = GetActiveRegistration(found_fd);
    if (reg) {
      reg->read_req = nullptr;
      reg->in_fn = nullptr;
      if (reg->async_only && reg->write_req == nullptr) {
        MaybeRetireAsyncRegistration(reg);
      } else {
        UpdateKqueue(found_fd);
      }
    }
  } else if (is_write) {
    auto *reg = GetActiveRegistration(found_fd);
    if (reg) {
      reg->write_req = nullptr;
      reg->out_fn = nullptr;
      if (reg->async_only && reg->read_req == nullptr) {
        MaybeRetireAsyncRegistration(reg);
      } else {
        UpdateKqueue(found_fd);
      }
    }
  }

  State(request).link.next = pending_cancels_head;
  pending_cancels_head = request;
}

Aio::Aio() { impl_ = std::make_unique<KqueueImpl>(); }
}  // namespace aos

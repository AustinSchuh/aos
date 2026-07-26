#ifndef AOS_EVENTS_AIO_READINESS_BACKEND_H_
#define AOS_EVENTS_AIO_READINESS_BACKEND_H_

#include <functional>
#include <span>
#include <string>
#include <vector>

#include "aos/events/aio_internal.h"
#include "aos/events/aio_registrations.h"
#include "aos/events/aio_state.h"
#include "aos/events/file_descriptor.h"

namespace aos::internal {

// Everything a readiness-driven Aio backend does that is not a syscall.
//
// epoll, kqueue and IOCP all work the same way: keep a table of what each fd
// is registered for, ask the kernel which one is ready, and dispatch.  They
// differ only in how interest is expressed to the kernel and how the kernel
// answers.  That difference is the five virtuals below; everything else --
// the public entry points, the raw request machinery, the drain loop, the
// fork check, and the dispatch rules EPoll established -- lives here once.
//
// Sharing this is mostly about the subtle parts rather than the long ones.
// Independent copies of it have already drifted into three separate bugs:
// a hangup reaching the wrong handler, a use-after-free when a callback
// deletes its own fd, and a fork going unnoticed because one backend never
// registered the atfork handler.
class ReadinessBackend : public Aio::Impl {
 public:
  ~ReadinessBackend() override;

  void Run() final;
  bool should_run() const final;
  void Quit() final;
  bool Poll(bool block) final;

  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) final;
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) final;
  void Cancel(AsyncRequest *request) final;
  void BeforeWait(std::function<void()> function) final;

  void OnReadable(FileDescriptor fd, std::function<void()> callback) final;
  void OnError(FileDescriptor fd, std::function<void()> callback) final;
  void OnWritable(FileDescriptor fd, std::function<void()> callback) final;
  void OnEvents(FileDescriptor fd,
                std::function<void(uint32_t)> callback) final;
  void DeleteFd(FileDescriptor fd) final;
  void ForgetClosedFd(FileDescriptor fd) final;
  void EnableWritable(FileDescriptor fd) final;
  void DisableWritable(FileDescriptor fd) final;
  void SetEvents(FileDescriptor fd, uint32_t events) final;

  bool HasRawRequestsInFlight() const final;

 protected:
  // backend_name is what this backend calls itself in operational-failure
  // messages ("epoll", "kqueue", "iocp"); aio_test asserts on it.
  ReadinessBackend(size_t pool_size, std::string backend_name);

  // --- The seam.  Everything below is a syscall or close to one. ---

  // Arms or disarms reg's kernel registration to match what it is now
  // interested in.  False means the fd went away underneath us (EBADF), in
  // which case the caller completes the request with an error rather than
  // waiting for an event that cannot come.
  virtual bool UpdateRegistration(FdRegistration *reg) = 0;

  // Drops reg's kernel registration entirely.  Tolerates the fd having been
  // closed already -- the kernel drops those registrations itself.
  virtual void RemoveRegistration(FdRegistration *reg) = 0;

  enum class WaitResult {
    // Nothing was ready, or the wait was interrupted.
    kNothing,
    // The backend handled something itself -- a kqueue timer or signal
    // filter, which epoll delivers as an ordinary fd instead.
    kHandled,
    // *out is filled in.
    kFdReady,
  };
  struct FdReady {
    FdRegistration *reg = nullptr;
    // The readiness bits, already translated to kIn/kPri/kOut/kErr and
    // already masked down to what the caller subscribed to.
    uint32_t events = 0;
    // Whether the condition was terminal -- a hangup or an error.  A raw
    // request has to observe one by running, since it cannot be consumed.
    bool terminal = false;
  };
  // Waits for at most one event.
  virtual WaitResult WaitForOne(bool block, FdReady *out) = 0;

  // Rebuilds the kernel object in a forked child.  Called with the
  // registration table intact; the caller re-arms every registration
  // afterwards through UpdateRegistration().
  virtual void RecreateKernelState() = 0;

  // Runs after RecreateKernelState() and ReArmAllRegistrations(), for work a
  // backend can only do once the table is armed on the new kernel object --
  // EpollImpl replaces its inherited timerfds here, which registers new fds
  // that the re-arm above must not try to add a second time.
  virtual void AfterForkReArm() {}

  // Breaks a blocked WaitForOne() out.  Reached from Quit(), which is
  // async-signal-safe, so this must be too.
  virtual void WakeLoop() = 0;

  // The fd the loop's own wakeup read is on, so it can be told apart from a
  // caller's request.
  virtual FileDescriptor wakeup_fd() const = 0;

  // --- Shared state the backends still need to reach. ---
  FdRegistrationTable registrations_;
  const std::string backend_name_;

  // Re-arms every live registration on a freshly rebuilt kernel object.
  void ReArmAllRegistrations();
  // The common half of the forked-child rebuild.
  void HandleForkInternal();
  // Rebuilds if the process forked since we last looked.  Every public
  // entry point calls this first, so a forked child that submits work
  // before its first Poll() operates on live kernel state.
  void CheckForFork();

  bool in_before_wait_ = false;
  std::vector<std::function<void()>> before_wait_functions_;

  // Quit() writes these from other threads and from a signal handler
  // (ShmEventLoop's SIGINT/SIGHUP/SIGTERM handler), and should_run() reads
  // them; plain bools would be a data race that can miss the shutdown
  // outright.  A handler may only touch lock-free atomics, so assert that;
  // a non-lock-free atomic could deadlock against the interrupted thread.
  static_assert(std::atomic<bool>::is_always_lock_free,
                "Quit() runs in a signal handler, so these have to be usable "
                "from one");
  // Starts true, so should_run() is true before the first Run().  Run()
  // clears it on exit, Quit() on shutdown.
  std::atomic<bool> run_ = true;
  std::atomic<bool> quit_requested_ = false;

  size_t last_fork_count_ = 0;

  // Requests waiting to be resolved as Canceled / with an
  // already-determined synchronous result on the next Poll(), singly linked
  // through AioState::link.next.
  struct PendingLinkTraits {
    static AsyncRequest *&next(AsyncRequest *request) {
      return State(request).link.next;
    }
    static AsyncRequest *&prev(AsyncRequest *request) {
      return State(request).link.prev;
    }
  };
  // link.queued values, so membership is O(1) and testing it does not
  // reorder the list the way a remove-and-reinsert would.
  static constexpr int32_t kQueuedSync = 1;
  static constexpr int32_t kQueuedCancel = 2;
  // FIFO, matching io_uring: completions are delivered in the order they
  // were resolved.  These were stacks, which delivered them backwards -- an
  // ordering difference a caller driving two requests can see.
  IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits> pending_cancels_;
  IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits>
      pending_sync_completions_;
  // Push helpers that keep link.queued in step with the lists.
  void QueueSyncCompletion(AsyncRequest *request) {
    State(request).link.queued = kQueuedSync;
    pending_sync_completions_.PushBack(request);
  }
  void QueueCancel(AsyncRequest *request) {
    State(request).link.queued = kQueuedCancel;
    pending_cancels_.PushBack(request);
  }

  void CancelRequest(AsyncRequest *request);
  // Drops the fd claim a canceled request held, once its completion is out.
  void ReleaseCancelClaim(AsyncRequest *request);
  void MaybeRetireAsyncRegistration(FdRegistration *reg);

  // Completes `request` with the errno UpdateRegistration() rejected the fd
  // with, and retires the registration.  See EpollImpl::UpdateRegistration().
  void CompleteWithRegistrationError(FdRegistration *reg,
                                     AsyncRequest *request);
  // UpdateRegistration(), fatal if it refuses the fd.  For every caller with
  // no request to hand the refusal to: a registration the kernel is not
  // holding never fires again, so the alternative is an fd that silently
  // stops waking.  The two submit paths are the exception -- they have a
  // request, and turn a refusal into the error completion aio.h promises
  // (CompleteWithRegistrationError()).
  void UpdateRegistrationOrDie(FdRegistration *reg);
  // Dies if request is already armed on this loop.  See the definition.
  void CheckNotAlreadyInFlight(AsyncRequest *request) const;
};

}  // namespace aos::internal

#endif  // AOS_EVENTS_AIO_READINESS_BACKEND_H_

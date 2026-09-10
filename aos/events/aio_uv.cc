#include "aos/events/aio_uv.h"

#include <uv.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/die_if_null.h"

#include "aos/events/aio_internal.h"
#include "aos/events/aio_state.h"
#include "aos/events/aio_uv_internal.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/realtime.h"

// The half of the libuv backend that is the same on every platform.  What
// libuv can wait on, and what has to be built around it where it cannot, is
// the other half: aio_uv_posix.cc and aio_uv_windows.cc, behind the seam in
// aio_uv_internal.h.

namespace aos {

namespace uv_internal {

ScopedCallbackRealtime::ScopedCallbackRealtime(const UvCore *core)
    : prior_(core->callback_realtime().has_value()
                 ? std::make_optional(MarkRealtime(*core->callback_realtime()))
                 : std::nullopt) {}

ScopedCallbackRealtime::~ScopedCallbackRealtime() {
  if (prior_.has_value()) {
    MarkRealtime(*prior_);
  }
}

}  // namespace uv_internal

namespace {

using uv_internal::FdRegistration;
using uv_internal::kErr;
using uv_internal::kIn;
using uv_internal::kOut;
using uv_internal::kPri;
using uv_internal::MakeUvPlatform;
using uv_internal::ScopedCallbackRealtime;
using uv_internal::UvCore;
using uv_internal::UvPlatform;

int UvEventsFor(uint32_t events) {
  int result = 0;
  if (events & kIn) result |= UV_READABLE;
  if (events & kOut) result |= UV_WRITABLE;
  if (events & kPri) result |= UV_PRIORITIZED;
  if (events & kErr) result |= UV_DISCONNECT;
  return result;
}

uint32_t EventsFromUv(int uv_events) {
  uint32_t result = 0;
  if (uv_events & UV_READABLE) result |= kIn;
  if (uv_events & UV_WRITABLE) result |= kOut;
  if (uv_events & UV_PRIORITIZED) result |= kPri;
  if (uv_events & UV_DISCONNECT) result |= kErr;
  return result;
}

class UvImpl : public Aio::Impl, public UvCore {
 public:
  UvImpl(uv_loop_s *loop, UvQuitBehavior quit_behavior)
      : loop_(ABSL_DIE_IF_NULL(loop)), quit_behavior_(quit_behavior) {
    // Heap allocated, and freed by its own close callback, because libuv runs
    // that callback on some later turn of a loop this object does not drive.
    // A handle owned as a member could not survive that long.
    prepare_ = new uv_prepare_t;
    ABSL_CHECK_EQ(uv_prepare_init(loop_, prepare_), 0);
    prepare_->data = this;

    // Created in both quit behaviours, because every part of Quit() -- not
    // just the optional uv_stop() -- has to happen on the loop thread.  aio.h
    // documents Quit() as callable from any thread and from a signal handler,
    // and no libuv handle operation is either of those except
    // uv_async_send().  So Quit() sends, and this callback does the work.
    quit_async_ = new uv_async_t;
    ABSL_CHECK_EQ(
        uv_async_init(
            loop_, quit_async_,
            [](uv_async_t *handle) {
              static_cast<UvImpl *>(handle->data)->HandleQuitOnLoopThread();
            }),
        0);
    quit_async_->data = this;
    // Nothing should be kept alive merely because AOS might quit later.
    uv_unref(reinterpret_cast<uv_handle_t *>(quit_async_));

    // An idle handle rather than the prepare above, for completions that have
    // no descriptor to wait on: while one is active libuv polls with a zero
    // timeout, so the turn that delivers the completion is a turn that ends.
    // Running them from prepare_ instead would deliver them and then block the
    // loop anyway, waiting for an event nobody is going to send.
    completions_idle_ = new uv_idle_t;
    ABSL_CHECK_EQ(uv_idle_init(loop_, completions_idle_), 0);
    completions_idle_->data = this;

    // Last, so everything it may ask of this object is already there.
    platform_ = MakeUvPlatform(this);
  }

  ~UvImpl() override {
    // A live Aio::Timer's destructor dereferences this impl, so one outliving
    // its Aio is a use-after-free -- on a borrowed loop exactly as much as on
    // one this backend drives.  Every other backend CHECKs it, and aio.h
    // documents it; without this the same mistake is a segfault in ~Timer
    // rather than a message naming what the caller did.
    ABSL_CHECK_EQ(active_timer_count_, 0)
        << ": An Aio::Timer must be destroyed before its Aio";
    while (!registrations_.empty()) {
      DeleteFd(registrations_.begin()->first);
    }
    // After the registrations, which it may have been watching, and while
    // the loop is still this object's to put handles on.
    platform_.reset();
    if (completions_idle_started_) {
      uv_idle_stop(completions_idle_);
      completions_idle_started_ = false;
    }
    uv_close(reinterpret_cast<uv_handle_t *>(completions_idle_),
             [](uv_handle_t *handle) {
               delete reinterpret_cast<uv_idle_t *>(handle);
             });
    completions_idle_ = nullptr;
    if (prepare_started_) {
      uv_prepare_stop(prepare_);
      prepare_started_ = false;
    }
    uv_close(reinterpret_cast<uv_handle_t *>(prepare_),
             [](uv_handle_t *handle) {
               delete reinterpret_cast<uv_prepare_t *>(handle);
             });
    uv_close(reinterpret_cast<uv_handle_t *>(quit_async_),
             [](uv_handle_t *handle) {
               delete reinterpret_cast<uv_async_t *>(handle);
             });
    quit_async_ = nullptr;
    // Deliberately no uv_run() here.  The loop belongs to whoever handed it
    // to UvAio, and the close callbacks above run on their next turn of it --
    // which is why nothing closed here may be owned by this object.
  }

  uv_loop_s *loop() override { return loop_; }

  // Called by every callback that did user-visible work, so Poll() can report
  // whether anything happened.
  void DidWork() override { did_work_ = true; }

  // Called by every platform timer's destructor; see the CHECK in ~UvImpl().
  void TimerDestroyed() override { --active_timer_count_; }

  // What the realtime malloc check was set to outside this turn of the loop,
  // or nullopt when nothing is holding it off; see UvAio::ScopedLoopTurn.
  void set_callback_realtime(std::optional<bool> realtime) {
    callback_realtime_ = realtime;
  }
  std::optional<bool> callback_realtime() const override {
    return callback_realtime_;
  }

  bool TakeDidWork() {
    const bool result = did_work_;
    did_work_ = false;
    return result;
  }

  bool HasRawRequestsInFlight() const override { return false; }

  std::unique_ptr<Aio::TimerState> MakeTimerState() override {
    ++active_timer_count_;
    return platform_->MakeTimerState();
  }

  void Run() override {
    ABSL_LOG(FATAL)
        << ": This Aio runs on a borrowed libuv loop, which is its owner's to "
           "drive.  Run their loop instead; everything registered here runs "
           "when they do.";
  }

  bool should_run() const override {
    return should_run_.load(std::memory_order_relaxed);
  }

  bool Poll(bool /*block*/) override {
    ABSL_LOG(FATAL)
        << ": Polling drives the loop, which a borrowed libuv loop is its "
           "owner's to do.  Run their loop instead; everything registered "
           "here runs when they do.";
  }

  void Quit() override {
    // Everything this does beyond the flag is libuv handle work, which is
    // the loop thread's alone -- so it is deferred to HandleQuitOnLoopThread()
    // rather than done here.  aio.h promises Quit() from any thread and from
    // a signal handler, and this pair of lines is all that promise allows:
    // a relaxed store, and uv_async_send(), which is the one libuv call
    // documented safe from both.
    should_run_.store(false, std::memory_order_relaxed);
    uv_async_send(quit_async_);
  }

  // The other half of Quit(), on the thread that owns the handles.
  //
  // Coalescing is libuv's: any number of uv_async_send()s before the loop
  // turns produce one callback, which is what Quit()'s own documented
  // stickiness wants anyway.
  void HandleQuitOnLoopThread() {
    for (auto &pair : registrations_) {
      if (pair.second->closing || pair.second->platform_watched) {
        continue;
      }
      uv_poll_stop(&pair.second->poll);
      pair.second->started_uv_events = 0;
    }
    platform_->Quit();
    if (prepare_started_) {
      uv_prepare_stop(prepare_);
      prepare_started_ = false;
    }
    if (quit_behavior_ == UvQuitBehavior::kStopLoop) {
      // Deliberately not reached in kLeaveLoopRunning: the loop belongs to
      // somebody else and may well have work of their own left.  Stopping
      // what AOS registered is the most that can mean.
      uv_stop(loop_);
    }
  }

  // libuv is readiness-based, so the completion API is emulated the way the
  // other readiness backends emulate it: park a handler that does the I/O
  // when the descriptor reports ready, and complete the request from there.
  //
  // Where this differs from ReadinessBackend is who runs the callback.  There
  // the completion is handed back through Poll(), one per call; here there is
  // no Poll() to hand it to -- the loop belongs to its owner -- so a
  // completion runs from the loop callback that produced it, and one turn of
  // the loop can deliver several.  Callers who need them rationed have to
  // poll, which means owning the loop.
  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) override {
    SubmitRaw(fd, request, buffer.data(), buffer.size(), /*write=*/false);
  }

  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) override {
    // The span is const because a write does not modify it; the staging slot
    // is shared with AsyncRead, which does.
    SubmitRaw(fd, request, const_cast<char *>(buffer.data()), buffer.size(),
              /*write=*/true);
  }

  void Cancel(AsyncRequest *request) override {
    ABSL_CHECK(request != nullptr);
    // Disarming goes back through libuv too -- see SubmitRaw().
    ScopedNotRealtime nrt;
    if (request->done) {
      // Nothing in flight.  Cancel() of a finished request is a no-op
      // everywhere, so that a caller racing a completion need not check.
      return;
    }
    for (auto &pair : registrations_) {
      FdRegistration *reg = pair.second.get();
      const bool is_read = reg->read_req == request;
      if (!is_read && reg->write_req != request) {
        continue;
      }
      DetachRaw(reg, is_read);
      QueueCompletion(request, kQueuedCanceled, 0);
      return;
    }
    // Already detached from its registration -- a submit that failed outright
    // is parked on pending_completions_ waiting for the loop, and cancelling
    // it changes nothing about the completion it is already going to get.
  }

  void BeforeWait(std::function<void()> function) override {
    // Registering one from inside one would mutate the list being walked.
    ABSL_CHECK(!in_before_wait_)
        << ": BeforeWait() may not be called from a before-wait function";
    before_wait_.push_back(std::move(function));
    EnsurePrepareStarted();
  }

  // The prepare handle runs once per turn of the loop, before it waits, which
  // is where both the before-wait functions and any completion with no
  // callback of its own get their turn.
  void EnsurePrepareStarted() {
    if (prepare_started_) return;
    ScopedNotRealtime nrt;
    ABSL_CHECK_EQ(
        uv_prepare_start(prepare_,
                         [](uv_prepare_t *handle) {
                           static_cast<UvImpl *>(handle->data)->RunBeforeWait();
                         }),
        0);
    prepare_started_ = true;
  }

  void OnReadable(FileDescriptor fd, std::function<void()> callback) override {
    FdRegistration *reg = GetOrCreate(fd);
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": Cannot mix OnEvents and OnReadable for fd " << fd;
    ABSL_CHECK(reg->read_req == nullptr)
        << ": Cannot mix OnReadable and AsyncRead on fd " << fd;
    ABSL_CHECK(!reg->async_only)
        << ": Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
    ABSL_CHECK(reg->in_fn == nullptr) << ": Duplicate in functions for " << fd;
    reg->in_fn = std::move(callback);
    Update(reg);
  }

  void OnError(FileDescriptor fd, std::function<void()> callback) override {
    FdRegistration *reg = GetOrCreate(fd);
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": Cannot mix OnEvents and OnError for fd " << fd;
    ABSL_CHECK(!reg->async_only)
        << ": Cannot mix AsyncRead/AsyncWrite and OnError on fd " << fd;
    ABSL_CHECK(reg->err_fn == nullptr)
        << ": Duplicate error functions for " << fd;
    reg->err_fn = std::move(callback);
    Update(reg);
  }

  void OnWritable(FileDescriptor fd, std::function<void()> callback) override {
    FdRegistration *reg = GetOrCreate(fd);
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": Cannot mix OnEvents and OnWritable for fd " << fd;
    ABSL_CHECK(reg->write_req == nullptr)
        << ": Cannot mix OnWritable and AsyncWrite on fd " << fd;
    ABSL_CHECK(!reg->async_only)
        << ": Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
    ABSL_CHECK(reg->out_fn == nullptr)
        << ": Duplicate out functions for " << fd;
    reg->out_fn = std::move(callback);
    reg->writable_enabled = true;
    Update(reg);
  }

  void OnEvents(FileDescriptor fd,
                std::function<void(uint32_t)> callback) override {
    FdRegistration *reg = GetOrCreate(fd);
    // Before the handler check: the raw path parks its own lambda in
    // in_fn/out_fn, so a descriptor carrying a request would otherwise be
    // reported as having handlers the caller never registered.
    ABSL_CHECK(!reg->async_only)
        << ": Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
    ABSL_CHECK(reg->in_fn == nullptr && reg->out_fn == nullptr &&
               reg->err_fn == nullptr)
        << ": May not replace OnEvents handlers for fd " << fd;
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": May not replace OnEvents handlers for fd " << fd;
    reg->events_fn = std::move(callback);
    Update(reg);
  }

  void DeleteFd(FileDescriptor fd) override {
    auto it = registrations_.find(fd);
    ABSL_CHECK(it != registrations_.end()) << "fd " << fd << " not found";
    std::unique_ptr<FdRegistration> reg = std::move(it->second);
    registrations_.erase(it);

    reg->closing = true;
    if (reg->platform_watched) {
      // Nothing of libuv's to close; the platform frees it, on a later turn.
      platform_->Release(std::move(reg));
      return;
    }
    uv_poll_stop(&reg->poll);
    // libuv frees nothing itself, and the handle must stay alive until its
    // close callback runs, so the registration deletes itself from there.
    uv_close(reinterpret_cast<uv_handle_t *>(&reg.release()->poll),
             [](uv_handle_t *handle) {
               FdRegistration *reg =
                   static_cast<FdRegistration *>(handle->data);
               if (reg->owns_fd) {
                 UvPlatform::Close(reg->fd);
               }
               delete reg;
             });
  }

  // Removes fd and closes it once libuv has finished with the handle.
  void DeleteAndCloseFd(FileDescriptor fd) override {
    auto it = registrations_.find(fd);
    ABSL_CHECK(it != registrations_.end()) << "fd " << fd << " not found";
    it->second->owns_fd = true;
    DeleteFd(fd);
  }

  void ForgetClosedFd(FileDescriptor fd) override {
    auto it = registrations_.find(fd);
    if (it == registrations_.end()) {
      return;
    }
    // The descriptor is already closed, so uv_poll_stop would be operating on
    // a stale fd. Closing the handle is still required to free it.
    DeleteFd(fd);
  }

  void EnableWritable(FileDescriptor fd) override {
    FdRegistration *reg = GetLegacy(fd);
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": EnableWritable is only for fds registered using OnWritable, not "
        << fd;
    // Deliberately not checked for a handler here: the other backends accept
    // this and die when the event they were told to watch for arrives with
    // nothing to deliver it to.  Checking at enable time instead moves the
    // death out of the caller's Poll() -- which is where a death test puts
    // its expectation -- and into the enable call, taking the process with it.
    reg->writable_enabled = true;
    Update(reg);
  }

  void DisableWritable(FileDescriptor fd) override {
    FdRegistration *reg = GetLegacy(fd);
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": DisableWritable is only for fds registered using OnWritable, not "
        << fd;
    ABSL_CHECK(reg->out_fn != nullptr)
        << ": DisableWritable is only for fds registered using OnWritable, not "
        << fd;
    reg->writable_enabled = false;
    Update(reg);
  }

  void SetEvents(FileDescriptor fd, uint32_t events) override {
    FdRegistration *reg = GetLegacy(fd);
    ABSL_CHECK(reg->events_fn != nullptr)
        << ": SetEvents is only for fds registered using OnEvents, not " << fd;
    reg->events = events;
    Update(reg);
  }

  void RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                    std::function<void()> callback) override {
    ABSL_CHECK(receiver_ == nullptr)
        << ": Duplicate ThreadSignalReceiver registration: only one receiver "
           "may be active at a time (see Aio::RegisterThreadSignalReceiver)";
    receiver_ = receiver;
    platform_->RegisterReceiver(receiver, std::move(callback));
  }

  void UnregisterThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override {
    ABSL_CHECK(receiver_ == receiver) << ": ThreadSignalReceiver not found";
    receiver_ = nullptr;
    platform_->UnregisterReceiver(receiver);
  }

  void ConsumeThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override {
    // Nothing registered means nothing to drain.
    if (receiver_ == nullptr) {
      return;
    }
    ABSL_CHECK(receiver_ == receiver) << ": ThreadSignalReceiver not found";
    platform_->ConsumeReceiver(receiver);
  }

 private:
  FdRegistration *Get(FileDescriptor fd) {
    auto it = registrations_.find(fd);
    ABSL_CHECK(it != registrations_.end()) << "fd " << fd << " not found";
    return it->second.get();
  }

  FdRegistration *GetOrCreate(FileDescriptor fd) {
    auto it = registrations_.find(fd);
    if (it != registrations_.end()) {
      return it->second.get();
    }
    auto reg = std::make_unique<FdRegistration>();
    reg->impl = this;
    reg->fd = fd;
    // The platform first: a descriptor libuv cannot poll may still be one
    // the platform can wait on, and only it knows.
    if (platform_->Adopt(reg.get())) {
      reg->platform_watched = true;
    } else {
      ABSL_CHECK_EQ(UvPlatform::PollInit(loop_, &reg->poll, fd), 0)
          << ": uv_poll_init failed for fd " << fd;
      reg->poll.data = reg.get();
    }
    FdRegistration *result = reg.get();
    registrations_.emplace(fd, std::move(reg));
    return result;
  }

  // GetOrCreate() for the raw path, which has a request to fail rather than a
  // caller to abort: libuv rejects a descriptor it cannot poll (a closed one,
  // or a regular file on some platforms), and aio.h promises that arrives as
  // an error completion.  Never offered to the platform: the emulation below
  // needs libuv's own readiness.
  FdRegistration *GetOrCreateForAsync(FileDescriptor fd, int *error) {
    auto it = registrations_.find(fd);
    if (it != registrations_.end()) {
      return it->second.get();
    }
    auto reg = std::make_unique<FdRegistration>();
    reg->impl = this;
    reg->fd = fd;
    const int result = UvPlatform::PollInit(loop_, &reg->poll, fd);
    if (result != 0) {
      *error = -result;
      return nullptr;
    }
    reg->poll.data = reg.get();
    FdRegistration *out = reg.get();
    registrations_.emplace(fd, std::move(reg));
    return out;
  }

  FdRegistration *GetLegacy(FileDescriptor fd) {
    FdRegistration *reg = Get(fd);
    ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
    return reg;
  }

  FdRegistration *Find(FileDescriptor fd) {
    auto it = registrations_.find(fd);
    return it == registrations_.end() ? nullptr : it->second.get();
  }

  // Recomputes what libuv should be watching for and re-arms if it changed.
  void Update(FdRegistration *reg) {
    if (reg->closing || !should_run_.load(std::memory_order_relaxed)) {
      return;
    }
    if (reg->platform_watched) {
      platform_->Update(reg);
      return;
    }
    int wanted = 0;
    if (reg->events_fn != nullptr) {
      wanted = UvEventsFor(reg->events);
      // A mask of nothing libuv has a bit for -- a caller asking only about
      // hangups, say -- still has to leave the descriptor watched, or the
      // condition it is waiting for can never be reported.  libuv delivers a
      // hangup on whatever it is already watching, and UV_DISCONNECT is the
      // cheapest thing to be watching.
      if (wanted == 0) wanted = UV_DISCONNECT;
    } else {
      if (reg->in_fn != nullptr) wanted |= UV_READABLE;
      // Armed on writable_enabled alone: asking for output events without a
      // handler is a caller error, and it is caught when the event arrives,
      // which cannot happen if the event is never asked for.
      if (reg->writable_enabled) wanted |= UV_WRITABLE;
      // libuv reports errors on whatever it is already watching, so OnError
      // alone still needs something armed to be delivered through.
      if (reg->err_fn != nullptr && wanted == 0) wanted |= UV_DISCONNECT;
    }
    wanted =
        UvPlatform::AdjustWatch(uv_internal::SubscribedEvents(reg), wanted);

    if (wanted == reg->started_uv_events) {
      return;
    }
    reg->started_uv_events = wanted;
    // libuv allocates inside these, and a realtime caller arming a watch is
    // not the allocation the malloc hook is there to catch.  The callbacks it
    // eventually runs are, so this is scoped to the calls themselves -- see
    // the file comment.
    ScopedNotRealtime nrt;
    if (wanted == 0) {
      uv_poll_stop(&reg->poll);
      return;
    }
    ABSL_CHECK_EQ(uv_poll_start(&reg->poll, wanted, &UvImpl::PollCallback), 0)
        << ": uv_poll_start failed for fd " << reg->fd;
  }

  static void PollCallback(uv_poll_t *handle, int status, int uv_events) {
    FdRegistration *reg = static_cast<FdRegistration *>(handle->data);
    UvImpl *impl = static_cast<UvImpl *>(reg->impl);
    impl->DidWork();

    // A platform that has to read libuv's report before believing it takes
    // over here; see UvPlatform::DispatchReadiness().
    if (impl->platform_->DispatchReadiness(reg, status, uv_events)) {
      return;
    }

    ScopedCallbackRealtime callback_realtime(impl);

    if (reg->events_fn != nullptr) {
      uint32_t events = EventsFromUv(uv_events);
      if (status < 0) events |= kErr;
      // Drop readiness nobody subscribed to, the way epoll never reports an
      // unsubscribed bit and KqueueImpl::DesiredReadiness() masks to stay a
      // faithful mirror of it.  This only bites where AdjustWatch() widened
      // the watch beyond what was asked: on Linux libuv is only ever asked
      // for the subscription, and Windows leaves through DispatchReadiness()
      // above without reaching here.  kErr is never masked out -- like epoll,
      // an error is reported whether or not it was asked for.
      events &= uv_internal::SubscribedEvents(reg) | kErr;
      // Nothing left to say.  Returning without calling back is what makes
      // the widened watch invisible rather than a stream of empty callbacks.
      if (events == 0) {
        return;
      }
      reg->events_fn(events);
      return;
    }

    // An error arrives as a negative status or as UV_DISCONNECT, and it is
    // reported first because the readable/writable callbacks below would
    // otherwise act on a descriptor that is already broken.
    //
    // Both spellings, because which one a hangup takes is the platform's
    // choice rather than the caller's.  epoll turns POLLERR into a status of
    // UV_EBADF (uv__poll_io() in libuv's src/unix/poll.c), which is the only
    // way this ever fired.  kqueue has no such translation -- EV_EOF is not
    // EV_ERROR, and libuv maps it to UV_DISCONNECT with a status of 0 -- so on
    // macOS an OnError() registration was told nothing at all.  The events_fn
    // path above has always folded UV_DISCONNECT into kErr; this is the same
    // rule for the callback-per-condition form.
    if (status < 0 || (uv_events & UV_DISCONNECT) != 0) {
      if (reg->err_fn != nullptr) {
        reg->err_fn();
        return;
      }
    }
    if ((uv_events & UV_READABLE) && reg->in_fn != nullptr) {
      reg->in_fn();
      // The callback may have deleted this registration.
      if (reg->closing) return;
    }
    if ((uv_events & UV_WRITABLE) && reg->writable_enabled) {
      ABSL_CHECK(reg->out_fn != nullptr)
          << ": No handler registered for output events on descriptor "
          << reg->fd;
      reg->out_fn();
    }
  }

  // --- The raw (AsyncRead/AsyncWrite) emulation. ---

  // Stages a request against fd and arms the direction it needs.
  void SubmitRaw(FileDescriptor fd, AsyncRequest *request, char *ptr,
                 size_t size, bool write) {
    ABSL_CHECK(request != nullptr);
    // Arming a watch allocates here -- libuv's handle bookkeeping, and the
    // registration this backend keeps beside it -- and there is no
    // preallocated ring to submit to instead, the way io_uring has.  The
    // other backends promise a realtime caller that submitting allocates
    // nothing; this one cannot, so it says so by turning the check off for
    // the submit rather than by quietly tripping it.  Completion callbacks
    // are deliberately outside this: what a caller does with its own data is
    // still held to the promise.
    ScopedNotRealtime nrt;
    CheckNotAlreadyInFlight(request);
    request->done = false;
    if (UvPlatform::IsInvalidFd(fd)) {
      // Not a descriptor libuv can be asked about, so this resolves now and is
      // delivered from the loop like any other completion.
      QueueCompletion(request, kQueuedError, EBADF);
      return;
    }

    int error = 0;
    FdRegistration *reg = GetOrCreateForAsync(fd, &error);
    if (reg == nullptr) {
      // libuv will not poll this descriptor.  A regular file is the ordinary
      // reason -- it is always ready, so it never produces the event a poll
      // would wait for -- and the answer there is to do the I/O now and hand
      // the result back with the loop, which is what EpollImpl does for the
      // same case.  A descriptor that is merely invalid fails this syscall
      // too, with the errno aio.h asks for.
      const int result = write ? UvPlatform::Write(fd, ptr, size)
                               : UvPlatform::Read(fd, ptr, size);
      if (result >= 0) {
        QueueCompletion(request, kQueuedOk, result);
      } else {
        QueueCompletion(request, kQueuedError, UvPlatform::LastIoError());
      }
      return;
    }
    ABSL_CHECK(reg->events_fn == nullptr)
        << ": Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
    // The duplicate check comes first in each direction: the raw path parks
    // its own lambda in in_fn/out_fn, so a second request on this fd would
    // otherwise trip the mixing check below and blame a handler the caller
    // never registered.
    if (write) {
      ABSL_CHECK(reg->write_req == nullptr)
          << ": Duplicate AsyncWrite on fd " << fd;
      ABSL_CHECK(reg->out_fn == nullptr)
          << ": Cannot mix OnWritable and AsyncWrite on fd " << fd;
    } else {
      ABSL_CHECK(reg->read_req == nullptr)
          << ": Duplicate AsyncRead on fd " << fd;
      ABSL_CHECK(reg->in_fn == nullptr)
          << ": Cannot mix OnReadable and AsyncRead on fd " << fd;
    }
    // The specific checks above name the direction that collides; this
    // catches the cross-direction pairing.  async_only is the honest
    // discriminator rather than "is some handler set", for the same reason.
    ABSL_CHECK(reg->async_only ||
               (reg->in_fn == nullptr && reg->out_fn == nullptr &&
                reg->err_fn == nullptr))
        << ": Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
    reg->async_only = true;

    State(request).raw.ptr = ptr;
    State(request).raw.size = size;

    if (write) {
      reg->write_req = request;
      reg->writable_enabled = true;
      reg->out_fn = [this, fd]() { CompleteRaw(fd, /*write=*/true); };
    } else {
      reg->read_req = request;
      reg->in_fn = [this, fd]() { CompleteRaw(fd, /*write=*/false); };
    }
    Update(reg);
  }

  // Runs the I/O the request came for, now that libuv says the descriptor is
  // ready, and completes it.
  void CompleteRaw(FileDescriptor fd, bool write) {
    FdRegistration *reg = Find(fd);
    if (reg == nullptr) return;
    AsyncRequest *request = write ? reg->write_req : reg->read_req;
    if (request == nullptr) return;

    char *const ptr = static_cast<char *>(State(request).raw.ptr);
    const size_t size = State(request).raw.size;
    const int result = write ? UvPlatform::Write(fd, ptr, size)
                             : UvPlatform::Read(fd, ptr, size);
    // Taken before anything below can clobber it.
    const int error = result < 0 ? UvPlatform::LastIoError() : 0;
    if (result < 0 && (error == EAGAIN || error == EWOULDBLOCK)) {
      // A spurious readiness report.  Leave the request armed for the next.
      return;
    }

    DetachRaw(reg, !write);
    if (result >= 0) {
      RunCompletion(request, aos::Ok(), result);
    } else {
      RunCompletion(request, aos::MakeError("uv error"), error);
    }
  }

  // Unstages a request and stops watching for what it needed.
  void DetachRaw(FdRegistration *reg, bool read) {
    if (read) {
      reg->read_req = nullptr;
      reg->in_fn = nullptr;
    } else {
      reg->write_req = nullptr;
      reg->out_fn = nullptr;
      reg->writable_enabled = false;
    }
    if (reg->read_req == nullptr && reg->write_req == nullptr) {
      reg->async_only = false;
    }
    Update(reg);
  }

  // aio.h calls resubmitting a request that is still in flight a caller error,
  // and it is one worth naming: the staging slot below would be overwritten
  // and the first completion lost.
  void CheckNotAlreadyInFlight(const AsyncRequest *request) const {
    for (const auto &pair : registrations_) {
      const FdRegistration *reg = pair.second.get();
      ABSL_CHECK(reg->read_req != request && reg->write_req != request)
          << ": AsyncRead()/AsyncWrite() on a request that is still in "
             "flight; wait for its completion or Cancel() it first";
    }
    for (const AsyncRequest *queued = pending_head_; queued != nullptr;
         queued = State(queued).link.next) {
      ABSL_CHECK(queued != request)
          << ": AsyncRead()/AsyncWrite() on a request that is still in "
             "flight; wait for its completion or Cancel() it first";
    }
  }

  // Which status a queued completion is waiting to be given.  Kept as a tag
  // rather than a built Status because the queue below has to be walkable and
  // drainable without allocating -- the callbacks run under whatever realtime
  // promise the caller made, and a std::vector of Status would malloc on the
  // way in and free on the way out.
  enum QueuedStatus : int32_t {
    kNotQueued = 0,
    kQueuedCanceled = 1,
    kQueuedError = 2,
    kQueuedOk = 3,
  };

  // Completions with no loop callback of their own to run from -- a submit
  // that resolved before libuv saw it, and Cancel().  Threaded through the
  // request's own link, so queueing one allocates nothing, and delivered from
  // the prepare handle so every completion still reaches the caller from the
  // loop rather than from inside the call that made it.
  void QueueCompletion(AsyncRequest *request, QueuedStatus status,
                       int32_t result) {
    State(request).link.result = result;
    State(request).link.queued = status;
    State(request).link.next = nullptr;
    request->done = false;
    if (pending_tail_ == nullptr) {
      pending_head_ = request;
    } else {
      State(pending_tail_).link.next = request;
    }
    pending_tail_ = request;
    if (!completions_idle_started_) {
      ScopedNotRealtime nrt;
      ABSL_CHECK_EQ(
          uv_idle_start(
              completions_idle_,
              [](uv_idle_t *handle) {
                static_cast<UvImpl *>(handle->data)->DrainPendingCompletions();
              }),
          0);
      completions_idle_started_ = true;
    }
  }

  void DrainPendingCompletions() {
    // Detached first: a callback may queue another completion, and that one
    // belongs to the next turn of the loop rather than to this drain.
    AsyncRequest *request = pending_head_;
    if (request != nullptr) DidWork();
    pending_head_ = nullptr;
    pending_tail_ = nullptr;
    while (request != nullptr) {
      AsyncRequest *const next = State(request).link.next;
      const int32_t queued = State(request).link.queued;
      State(request).link.queued = kNotQueued;
      State(request).link.next = nullptr;
      RunCompletion(request,
                    queued == kQueuedOk         ? aos::Ok()
                    : queued == kQueuedCanceled ? aos::MakeError("Canceled")
                                                : aos::MakeError("uv error"),
                    State(request).link.result);
      request = next;
    }
    if (pending_head_ == nullptr && completions_idle_started_) {
      // Nothing left to deliver, so stop asking the loop not to sleep.
      ScopedNotRealtime nrt;
      uv_idle_stop(completions_idle_);
      completions_idle_started_ = false;
    }
  }

  void RunCompletion(AsyncRequest *request, aos::Status status,
                     int32_t result) {
    request->done = true;
    if (request->callback == nullptr) return;
    ScopedCallbackRealtime callback_realtime(this);
    request->callback(Completion{std::move(status), result, request->user_data},
                      request->context);
  }

  void RunBeforeWait() {
    in_before_wait_ = true;
    for (const std::function<void()> &function : before_wait_) {
      function();
    }
    in_before_wait_ = false;
  }

  uv_loop_s *const loop_;
  const UvQuitBehavior quit_behavior_;
  uv_prepare_t *prepare_ = nullptr;
  uv_idle_t *completions_idle_ = nullptr;
  bool completions_idle_started_ = false;
  uv_async_t *quit_async_ = nullptr;
  bool prepare_started_ = false;
  // Written by Quit() from any thread, read on the loop thread.  Relaxed
  // throughout: it carries no data, and the uv_async_send() that follows the
  // store is what actually orders the loop's view of it.
  std::atomic<bool> should_run_{true};
  bool did_work_ = false;
  std::optional<bool> callback_realtime_;
  bool in_before_wait_ = false;

  int active_timer_count_ = 0;

  std::map<FileDescriptor, std::unique_ptr<FdRegistration>> registrations_;
  std::vector<std::function<void()>> before_wait_;
  // Completions with no loop callback of their own; see QueueCompletion().
  AsyncRequest *pending_head_ = nullptr;
  AsyncRequest *pending_tail_ = nullptr;
  ipc_lib::ThreadSignalReceiver *receiver_ = nullptr;
  // The other half; see aio_uv_internal.h.  Last, so it is destroyed first.
  std::unique_ptr<UvPlatform> platform_;
};

}  // namespace

UvAio::UvAio(uv_loop_s *loop, UvQuitBehavior quit_behavior)
    : Aio(std::make_unique<UvImpl>(loop, quit_behavior)) {}

UvAio::~UvAio() = default;

bool UvAio::TakeDidWork() {
  return static_cast<UvImpl *>(impl_.get())->TakeDidWork();
}

UvAio::ScopedLoopTurn::ScopedLoopTurn(UvAio *aio)
    : aio_(ABSL_DIE_IF_NULL(aio)), prior_(MarkRealtime(false)) {
  static_cast<UvImpl *>(aio_->impl_.get())->set_callback_realtime(prior_);
}

UvAio::ScopedLoopTurn::~ScopedLoopTurn() {
  static_cast<UvImpl *>(aio_->impl_.get())->set_callback_realtime(std::nullopt);
  MarkRealtime(prior_);
}

}  // namespace aos

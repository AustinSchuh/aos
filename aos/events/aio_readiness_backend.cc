#include "aos/events/aio_readiness_backend.h"

#include <cerrno>
#include <cstring>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/aio_state.h"
#include "aos/events/aio_unix.h"
#include "aos/realtime.h"

namespace aos::internal {

ReadinessBackend::ReadinessBackend(size_t pool_size, std::string backend_name)
    : registrations_(pool_size), backend_name_(std::move(backend_name)) {
  RegisterForkCounters();
  last_fork_count_ = ForkCount();
}

ReadinessBackend::~ReadinessBackend() = default;

void ReadinessBackend::ReArmAllRegistrations() {
  for (const auto &reg : registrations_) {
    if (reg->registered) {
      reg->registered = false;
      UpdateRegistrationOrDie(reg);
    }
  }
}

void ReadinessBackend::Run() {
  if (quit_requested_) {
    quit_requested_ = false;
    return;
  }
  run_ = true;
  // Blocking polls while running, non-blocking once Quit() lands -- see
  // IoUringImpl::Run(), which this mirrors, and Aio::Run() for the contract.
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

bool ReadinessBackend::should_run() const { return run_ && !quit_requested_; }

// The common half of rebuilding in a forked child.  The kernel object
// itself is per-backend (RecreateKernelState()); re-arming what was
// registered on the old one is not.
void ReadinessBackend::HandleForkInternal() {
  // Shared rule; see Aio::Impl::CheckNoRawRequestsInFlightOnFork().  The
  // failure it prevents is a duplicated write: a readiness registration
  // survives a fork, so the re-arm below would otherwise hand the child work
  // the parent is still doing.
  CheckNoRawRequestsInFlightOnFork();

  RecreateKernelState();
  ReArmAllRegistrations();
  AfterForkReArm();
}

void ReadinessBackend::Quit() {
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
  WakeLoop();
}

void ReadinessBackend::CheckForFork() {
  size_t current_fork_count = internal::ForkCount();
  if (last_fork_count_ != current_fork_count) {
    // Record the new count *before* HandleFork(): it re-submits work
    // through these same entry points (e.g. the wakeup read via
    // AsyncRead()), so a count left stale would recurse forever.
    last_fork_count_ = current_fork_count;
    HandleForkInternal();
  }
}

// A request may only be armed once at a time.  aio.h's constraint 2 lets a
// request outlive the Aio it was armed on, so `done` alone cannot answer this
// -- a request left pending by a destroyed Aio also carries done == false.
// Ask this loop instead, the same way IoUringImpl::ClearStaleRawState() asks
// raw_in_flight_: the registrations and the two pending lists are exactly what
// this loop is holding.  link.queued would answer the membership question in
// O(1), and deliberately is not asked -- it is the flag that can be stale from
// a destroyed instance, which is the case this exists for.
//
// The per-fd "Duplicate AsyncRead on fd" check catches re-arming on the *same*
// fd.  This is the cross-fd case, which used to be silent and left two
// registrations pointing at one request: whichever completed first ran the
// callback, and the other kept a pointer to a request the caller was then free
// to reuse.
//
// `done` is the cheap filter, so a fresh or completed request never walks
// anything.  A request that reaches the walk is one a destroyed Aio left
// pending, which is rare.
void ReadinessBackend::CheckNotAlreadyInFlight(AsyncRequest *request) const {
  if (request->done) {
    return;
  }
  for (const auto &reg : registrations_) {
    ABSL_CHECK(reg->read_req != request && reg->write_req != request)
        << ": AsyncRead()/AsyncWrite() on a request that is still in flight; "
           "wait for its completion or Cancel() it first";
  }
  // The two ways a request can be in flight without being attached to any
  // registration: a submit that failed outright (an invalid fd, resolved here
  // and parked for the next Poll()), and one whose Cancel() detached it from
  // its registration but whose Canceled completion has not been dispatched
  // yet.  Neither has run the caller's callback, so both are still in flight
  // as far as aio.h is concerned, and neither is reachable from the walk
  // above.
  //
  // Checked rather than left to the "duplicate on this fd" tests because both
  // lists are intrusive and threaded through AsyncRequest::internal_state:
  // pushing a request that is already on one clears the link to whatever
  // followed it, so that request and everything behind it drop out of the
  // list and their completions never arrive.  They also share one set of
  // links (see AioState::link), so being on the other list is just as fatal.
  const auto check = [request](const AsyncRequest *queued) {
    ABSL_CHECK(queued != request)
        << ": AsyncRead()/AsyncWrite() on a request that is still in flight; "
           "wait for its completion or Cancel() it first";
  };
  pending_sync_completions_.ForEach(check);
  pending_cancels_.ForEach(check);
}

void ReadinessBackend::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                                 AsyncRequest *request) {
  CheckForFork();
  CheckNotAlreadyInFlight(request);
  // Cleared rather than trusted: constraint 2 lets a request outlive the Aio
  // it was armed on, so it can arrive still marked queued on a list belonging
  // to an instance that is gone.  See IoUringImpl::ClearStaleRawState().
  State(request).link.queued = 0;
  request->done = false;
  if (fd < 0) {
    State(request).link.result = -EBADF;
    QueueSyncCompletion(request);
    return;
  }
  internal::FdRegistration *reg = registrations_.GetOrCreateAsync(fd);

  ABSL_CHECK(reg->events_fn == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  // Before the handler checks below: the raw submit path parks its own lambda
  // in in_fn, so a second AsyncRead() on this fd would otherwise trip "Cannot
  // mix OnReadable and AsyncRead" and blame a handler the caller never
  // registered.
  ABSL_CHECK(reg->read_req == nullptr) << "Duplicate AsyncRead on fd " << fd;
  ABSL_CHECK(reg->in_fn == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  // Legacy handlers and raw requests are exclusive on an fd, in both
  // directions.  The specific checks above name the direction that
  // collides; this catches the cross-direction pairing, which used to be
  // legal.  in_fn/out_fn are also where the raw submit paths park their
  // own lambdas, so async_only is the honest discriminator rather than
  // "is some handler set".
  ABSL_CHECK(reg->async_only)
      << "Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
  reg->read_req = request;

  State(request).raw.ptr = buffer.data();
  State(request).raw.size = buffer.size();

  reg->in_fn = [this, fd]() {
    // Local copies of the captures: the paths below can destroy this very
    // lambda, and only these locals (and plain pointees like req) may be
    // touched after that.
    ReadinessBackend *const impl = this;
    const int read_fd = fd;

    auto *r = impl->registrations_.GetActive(read_fd);
    if (r == nullptr) return;
    AsyncRequest *req = r->read_req;
    if (!req) return;
    char *data = static_cast<char *>(State(req).raw.ptr);
    size_t size = State(req).raw.size;
    ssize_t res = read(read_fd, data, size);
    if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }
    // Before any kernel call below can clobber it.
    const int read_errno = errno;
    r->read_req = nullptr;
    req->done = true;
    if (r->async_only && r->write_req == nullptr) {
      // Last operation: recycle the pool slot (parked on retired_, not
      // destroyed until dispatch is over).
      impl->MaybeRetireAsyncRegistration(r);
    } else {
      // Legacy state shares this registration; just detach the read.  The
      // assignment destroys the executing lambda -- locals only from here.
      r->in_fn = nullptr;
      impl->UpdateRegistrationOrDie(r);
    }
    if (req->callback) {
      Completion completion;
      completion.user_data = req->user_data;
      if (res >= 0) {
        completion.status = aos::Ok();
        completion.result = static_cast<int32_t>(res);
      } else {
        completion.status = aos::MakeError(backend_name_ + " error");
        completion.result = static_cast<int32_t>(read_errno);
      }
      req->callback(completion, req->context);
    }
  };

  if (!UpdateRegistration(reg)) {
    // The kernel refused the registration (see EpollImpl::UpdateRegistration).
    // Deliver it as the error completion aio.h promises instead of aborting.
    CompleteWithRegistrationError(reg, request);
    return;
  }
  if (reg->unpollable) {
    // Regular file (see UpdateRegistration()): always ready, never delivers an
    // event, and never returns EAGAIN -- so read now.  The callback still
    // only runs inside Poll(), via the sync-completion list.
    const ssize_t res = read(fd, buffer.data(), buffer.size());
    const int read_errno = errno;
    reg->read_req = nullptr;
    reg->in_fn = nullptr;
    MaybeRetireAsyncRegistration(reg);
    State(request).link.result = res >= 0 ? res : -read_errno;
    QueueSyncCompletion(request);
  }
}

void ReadinessBackend::AsyncWrite(FileDescriptor fd,
                                  std::span<const char> buffer,
                                  AsyncRequest *request) {
  CheckForFork();
  CheckNotAlreadyInFlight(request);
  // Cleared rather than trusted: constraint 2 lets a request outlive the Aio
  // it was armed on, so it can arrive still marked queued on a list belonging
  // to an instance that is gone.  See IoUringImpl::ClearStaleRawState().
  State(request).link.queued = 0;
  request->done = false;
  if (fd < 0) {
    State(request).link.result = -EBADF;
    QueueSyncCompletion(request);
    return;
  }
  internal::FdRegistration *reg = registrations_.GetOrCreateAsync(fd);

  ABSL_CHECK(reg->events_fn == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  // Before the handler checks below: the raw submit path parks its own lambda
  // in out_fn, so a second AsyncWrite() on this fd would otherwise trip "Cannot
  // mix OnWritable and AsyncWrite" and blame a handler the caller never
  // registered.
  ABSL_CHECK(reg->write_req == nullptr) << "Duplicate AsyncWrite on fd " << fd;
  ABSL_CHECK(reg->out_fn == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  // Legacy handlers and raw requests are exclusive on an fd, in both
  // directions.  The specific checks above name the direction that
  // collides; this catches the cross-direction pairing, which used to be
  // legal.  in_fn/out_fn are also where the raw submit paths park their
  // own lambdas, so async_only is the honest discriminator rather than
  // "is some handler set".
  ABSL_CHECK(reg->async_only)
      << "Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
  reg->write_req = request;

  State(request).raw.ptr = const_cast<char *>(buffer.data());
  State(request).raw.size = buffer.size();

  reg->out_fn = [this, fd]() {
    // Same capture discipline as AsyncRead()'s lambda -- see there.
    ReadinessBackend *const impl = this;
    const int write_fd = fd;

    auto *r = impl->registrations_.GetActive(write_fd);
    if (r == nullptr) return;
    AsyncRequest *req = r->write_req;
    if (!req) return;
    const char *data = static_cast<const char *>(State(req).raw.ptr);
    size_t size = State(req).raw.size;
    ssize_t res = write(write_fd, data, size);
    if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return;
    }
    const int write_errno = errno;
    r->write_req = nullptr;
    req->done = true;
    if (r->async_only && r->read_req == nullptr) {
      impl->MaybeRetireAsyncRegistration(r);
    } else {
      r->out_fn = nullptr;
      impl->UpdateRegistrationOrDie(r);
    }
    if (req->callback) {
      Completion completion;
      completion.user_data = req->user_data;
      if (res >= 0) {
        completion.status = aos::Ok();
        completion.result = static_cast<int32_t>(res);
      } else {
        completion.status = aos::MakeError(backend_name_ + " error");
        completion.result = static_cast<int32_t>(write_errno);
      }
      req->callback(completion, req->context);
    }
  };

  if (!UpdateRegistration(reg)) {
    // The kernel refused the registration (see EpollImpl::UpdateRegistration).
    // Deliver it as the error completion aio.h promises instead of aborting.
    CompleteWithRegistrationError(reg, request);
    return;
  }
  if (reg->unpollable) {
    // Regular file -- always ready; same shape as AsyncRead()'s inline
    // completion, see there.
    const ssize_t res = write(fd, buffer.data(), buffer.size());
    const int write_errno = errno;
    reg->write_req = nullptr;
    reg->out_fn = nullptr;
    MaybeRetireAsyncRegistration(reg);
    State(request).link.result = res >= 0 ? res : -write_errno;
    QueueSyncCompletion(request);
  }
}

void ReadinessBackend::CompleteWithRegistrationError(
    internal::FdRegistration *reg, AsyncRequest *request) {
  const int err = reg->registration_errno;
  reg->registration_errno = 0;
  reg->read_req = nullptr;
  reg->write_req = nullptr;
  reg->in_fn = nullptr;
  reg->out_fn = nullptr;
  MaybeRetireAsyncRegistration(reg);
  // Negative means "errno" to the sync-completion drain, the same encoding
  // the unpollable inline reads use.
  State(request).link.result = -err;
  QueueSyncCompletion(request);
}

void ReadinessBackend::Cancel(AsyncRequest *request) {
  CheckForFork();
  CancelRequest(request);
}

void ReadinessBackend::ReleaseCancelClaim(AsyncRequest *request) {
  auto *reg = registrations_.GetActive(State(request).link.result);
  if (reg == nullptr) {
    return;
  }
  // Whichever direction this was; one request cannot be both.
  if (reg->cancel_pending_read) {
    reg->cancel_pending_read = false;
  } else {
    reg->cancel_pending_write = false;
  }
  // Retired only now, and only once nothing else holds the fd.
  if (reg->async_only && reg->read_req == nullptr &&
      reg->write_req == nullptr && !reg->cancel_pending_read &&
      !reg->cancel_pending_write) {
    MaybeRetireAsyncRegistration(reg);
  }
}

void ReadinessBackend::CancelRequest(AsyncRequest *request) {
  if (request->done) return;

  // A request already on the sync-completion list finished at submit: the
  // cancel lost the race and the completion stands, like io_uring's cancel
  // against an already-posted CQE.
  // Tested rather than removed and re-added, which on a FIFO queue would move
  // it to the back and reorder completions a caller can observe.
  if (State(request).link.queued == kQueuedSync) {
    return;
  }

  // Remove from the pending-cancel list if already there (re-canceling).
  // Already queued to be canceled: leave it where it is, for the same reason
  // -- re-canceling must not change delivery order.
  if (State(request).link.queued == kQueuedCancel) {
    return;
  }

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

  // The trampoline goes now -- the request must not also complete normally --
  // but the fd stays claimed until the Canceled completion is dispatched, and
  // the registration stays alive to hold that claim.  See
  // FdRegistration::cancel_pending_read.
  if (is_read) {
    auto *reg = registrations_.GetActive(found_fd);
    if (reg) {
      reg->read_req = nullptr;
      reg->in_fn = nullptr;
      reg->cancel_pending_read = true;
      UpdateRegistrationOrDie(reg);
    }
  } else if (is_write) {
    auto *reg = registrations_.GetActive(found_fd);
    if (reg) {
      reg->write_req = nullptr;
      reg->out_fn = nullptr;
      reg->cancel_pending_write = true;
      UpdateRegistrationOrDie(reg);
    }
  }

  // Which fd to release the claim on when this completion is dispatched.
  // link.result is unused for a cancel -- the Completion carries 0.
  State(request).link.result = found_fd;

  QueueCancel(request);
}

void ReadinessBackend::BeforeWait(std::function<void()> function) {
  // Stores a std::function, and grows the vector holding them.
  aos::CheckNotRealtime();
  ABSL_CHECK(!in_before_wait_)
      << ": BeforeWait() may not be called from a before-wait function";
  before_wait_functions_.push_back(std::move(function));
}

void ReadinessBackend::OnReadable(FileDescriptor fd,
                                  std::function<void()> callback) {
  CheckForFork();
  // Stores a std::function and may grow the registration table; both
  // allocate.  Registering a handler is a startup-time operation.
  aos::CheckNotRealtime();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  ABSL_CHECK(reg->read_req == nullptr && !reg->cancel_pending_read)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  ABSL_CHECK(reg->write_req == nullptr && !reg->cancel_pending_write)
      << "Cannot mix AsyncWrite and OnReadable on fd " << fd;
  // Unconditional, as EPoll and IoUringImpl have it: a null `callback` is
  // no exception, since it would silently clear the handler while leaving
  // the events subscribed -- which Poll()'s dispatch then dies on.
  ABSL_CHECK(!reg->in_fn) << "Duplicate in functions for " << fd;
  reg->in_fn = std::move(callback);

  reg->events |= kInEvents;
  UpdateRegistrationOrDie(reg);
}

void ReadinessBackend::OnError(FileDescriptor fd,
                               std::function<void()> callback) {
  CheckForFork();
  // Stores a std::function and may grow the registration table; both
  // allocate.  Registering a handler is a startup-time operation.
  aos::CheckNotRealtime();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  ABSL_CHECK(reg->read_req == nullptr && reg->write_req == nullptr &&
             !reg->cancel_pending_read && !reg->cancel_pending_write)
      << "Cannot mix AsyncRead/AsyncWrite and OnError on fd " << fd;
  // Unconditional -- see OnReadable().
  ABSL_CHECK(!reg->err_fn) << "Duplicate error functions for " << fd;
  reg->err_fn = std::move(callback);

  reg->events |= kErrorEvents;
  UpdateRegistrationOrDie(reg);
}

void ReadinessBackend::OnWritable(FileDescriptor fd,
                                  std::function<void()> callback) {
  CheckForFork();
  // Stores a std::function and may grow the registration table; both
  // allocate.  Registering a handler is a startup-time operation.
  aos::CheckNotRealtime();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  ABSL_CHECK(reg->write_req == nullptr && !reg->cancel_pending_write)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  ABSL_CHECK(reg->read_req == nullptr && !reg->cancel_pending_read)
      << "Cannot mix AsyncRead and OnWritable on fd " << fd;
  // Unconditional -- see OnReadable().
  ABSL_CHECK(!reg->out_fn) << "Duplicate out functions for " << fd;
  reg->out_fn = std::move(callback);

  reg->events |= kOutEvents;
  UpdateRegistrationOrDie(reg);
}

void ReadinessBackend::OnEvents(FileDescriptor fd,
                                std::function<void(uint32_t)> callback) {
  CheckForFork();
  // Stores a std::function and may grow the registration table; both
  // allocate.  Registering a handler is a startup-time operation.
  aos::CheckNotRealtime();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
  ABSL_CHECK(reg->read_req == nullptr && reg->write_req == nullptr &&
             !reg->cancel_pending_read && !reg->cancel_pending_write)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!reg->in_fn && !reg->out_fn && !reg->err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!reg->events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  reg->events_fn = std::move(callback);
}

void ReadinessBackend::DeleteFd(FileDescriptor fd) {
  CheckForFork();
  // Deterministic rather than data-dependent, matching
  // IoUringImpl::DeleteFd(): Release() can free the registration and destroy
  // its std::functions, so this would only *sometimes* trip the RT malloc
  // hook.  Nothing legitimately removes an fd registration from an RT thread
  // -- EPoll::DeleteFd() has always freed, so this was never legal.
  aos::CheckNotRealtime();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Async-only registrations are not the caller's to delete.  DeleteFd()
  // undoes an On*() registration; an fd carrying only AsyncRead/AsyncWrite
  // has none, and Release()ing it here would Detach() the requests without
  // ever completing them -- leaving req.done false forever, which aio.h's
  // constraint 2 says the caller may then never free or reuse.  A silent
  // hang, where io_uring gives a loud "fd not found" for the same sequence
  // because its legacy state lives in a separate table.
  //
  // Raw requests are retired by completing or Cancel()ing them, which is
  // what a caller wanting this fd gone should do.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";

  RemoveRegistration(reg);

  registrations_.Release(reg);
}

void ReadinessBackend::ForgetClosedFd(FileDescriptor fd) {
  // Deterministic, like DeleteFd() -- see there.
  aos::CheckNotRealtime();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Async-only registrations are not the caller's to delete.  DeleteFd()
  // undoes an On*() registration; an fd carrying only AsyncRead/AsyncWrite
  // has none, and Release()ing it here would Detach() the requests without
  // ever completing them -- leaving req.done false forever, which aio.h's
  // constraint 2 says the caller may then never free or reuse.  A silent
  // hang, where io_uring gives a loud "fd not found" for the same sequence
  // because its legacy state lives in a separate table.
  //
  // Raw requests are retired by completing or Cancel()ing them, which is
  // what a caller wanting this fd gone should do.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";

  registrations_.Release(reg);
}

void ReadinessBackend::EnableWritable(FileDescriptor fd) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // A raw-only registration is not an fd the caller registered a handler on,
  // so this is the same misuse io_uring reports as "fd not found" -- it looks
  // in a table that only holds legacy state, while this one holds both.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events | kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateRegistrationOrDie(reg);
  }
}

void ReadinessBackend::DisableWritable(FileDescriptor fd) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // A raw-only registration is not an fd the caller registered a handler on,
  // so this is the same misuse io_uring reports as "fd not found" -- it looks
  // in a table that only holds legacy state, while this one holds both.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events & ~kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateRegistrationOrDie(reg);
  }
}

void ReadinessBackend::SetEvents(FileDescriptor fd, uint32_t events) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(reg->events_fn)
      << "SetEvents is only for fds registered using OnEvents";

  if (reg->events != events) {
    reg->events = events;
    UpdateRegistrationOrDie(reg);
  }
}

void ReadinessBackend::UpdateRegistrationOrDie(internal::FdRegistration *reg) {
  if (UpdateRegistration(reg)) {
    return;
  }
  const int err = reg->registration_errno;
  reg->registration_errno = 0;
  ABSL_LOG(FATAL) << "The kernel refused the registration for fd " << reg->fd
                  << ": " << std::strerror(err)
                  << ".  A descriptor closed behind this loop's back has to be "
                     "removed with ForgetClosedFd() before it is closed; there "
                     "is no request here to report this on.";
}

void ReadinessBackend::MaybeRetireAsyncRegistration(
    internal::FdRegistration *reg) {
  if (!internal::FdRegistrationTable::ShouldRetire(reg)) {
    return;
  }
  RemoveRegistration(reg);
  registrations_.Release(reg);
}

bool ReadinessBackend::HasRawRequestsInFlight() const {
  for (const auto &reg : registrations_) {
    // No carve-out for the loop's own wakeup: it is a legacy OnReadable()
    // registration now, not a raw request, so it cannot land here at all.
    if (reg->read_req != nullptr) {
      return true;
    }
    if (reg->write_req != nullptr) {
      return true;
    }
  }
  return false;
}

bool ReadinessBackend::Poll(bool block) {
  // Not reentrant, matching IoUringImpl::Poll().  The dispatch guard covers
  // the whole body below, before-wait functions included.
  ABSL_CHECK_EQ(registrations_.dispatch_depth(), 0)
      << "Aio::Poll() reentered from inside a completion callback or "
         "before-wait function; wait by returning to the event loop instead";

  CheckForFork();

  // Reclaim registrations retired by earlier dispatches -- deferred to here
  // because a callback may delete its own fd while the dispatch below still
  // holds the registration pointer (see FdRegistrationTable::Release()).  At
  // this point no dispatch is in flight and no retired callback is on the
  // stack.
  registrations_.ScrubRetired();
  FdRegistrationTable::DispatchGuard dispatch_guard(&registrations_);

  // Registering a before-wait function from inside one is disallowed --
  // see BeforeWait().
  in_before_wait_ = true;
  for (const auto &fn : before_wait_functions_) {
    fn();
  }
  in_before_wait_ = false;

  // At most one completion callback per Poll() -- see Aio::Poll().  Each of
  // these delivers one and returns; entries that deliver nothing (already
  // resolved, or no callback) are popped without counting as a dispatch.

  // Handle any pending cancels first to complete them.
  // Set by a request that resolved with no callback to run; see below.
  bool progressed = false;
  while (AsyncRequest *req = pending_cancels_.PopFront()) {
    State(req).link.queued = 0;
    if (req->done) {
      continue;
    }
    req->done = true;
    // The claim CancelRequest() held goes here, which is the point the request
    // is finished -- io_uring releases its at the terminal CQE, and this is
    // the same moment.
    ReleaseCancelClaim(req);
    if (req->callback) {
      req->callback(Completion{aos::MakeError("Canceled"), 0, req->user_data},
                    req->context);
      return true;
    }
    // Resolved with nothing to deliver.  A request with no callback is not
    // a user-visible completion, so it does not spend the one-per-Poll()
    // budget -- but it is progress, so Poll() has to report it and must
    // not go on to wait.  io_uring gets this for free: its resolution is a
    // CQE, which both wakes the wait and counts as processed.
    progressed = true;
  }

  // Process synchronous completions (e.g. invalid FDs).
  while (AsyncRequest *req = pending_sync_completions_.PopFront()) {
    State(req).link.queued = 0;
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
        completion.status = aos::MakeError(backend_name_ + " error");
        completion.result = static_cast<int32_t>(-res);
      }
      req->callback(completion, req->context);
      return true;
    }
    // Resolved with nothing to deliver.  A request with no callback is not
    // a user-visible completion, so it does not spend the one-per-Poll()
    // budget -- but it is progress, so Poll() has to report it and must
    // not go on to wait.  io_uring gets this for free: its resolution is a
    // CQE, which both wakes the wait and counts as processed.
    progressed = true;
  }

  // Something was resolved above with no callback to run.  Report it and stop
  // here rather than waiting: blocking would strand a caller driving the
  // documented `while (!req.done && Poll(true))` drain inside this call,
  // where it can never re-read req.done.  Anything still ready is delivered
  // by the next Poll(), the same way a queued completion is.
  if (progressed) {
    return true;
  }

  FdReady ready;
  switch (WaitForOne(block, &ready)) {
    case WaitResult::kNothing:
      return false;
    case WaitResult::kHandled:
      return true;
    case WaitResult::kFdReady:
      break;
  }

  FdRegistration *reg = ready.reg;
  if (reg->events_fn) {
    // OnEvents delivers everything, hangups included -- see the OnEvents()
    // contract in aio.h.  SetEvents(fd, <hangup bit>) is a supported way to
    // watch for one, which is why this does not follow EPoll here.
    reg->events_fn(ready.terminal ? (ready.events | kErr) : ready.events);
    return true;
  }

  // Legacy handlers see exactly the bits the kernel reported, and the CHECKs
  // below are EPoll::InOutEventData::DoCallbacks() verbatim, message text
  // included -- callers depend on those semantics.
  //
  // Raw AsyncRead/AsyncWrite requests are the one addition, and only because
  // EPoll has no equivalent to match: such a request observes a terminal
  // condition by running.
  // Captured before any callback runs: a completion can retire this
  // registration, which is why the branches below re-read reg->fd.
  const bool raw_only = reg->async_only;

  uint32_t dispatch = ready.events;
  if (ready.terminal) {
    if (reg->read_req != nullptr) dispatch |= kIn;
    if (reg->write_req != nullptr) dispatch |= kOut;
  }

  if (dispatch & kInEvents) {
    ABSL_CHECK(reg->in_fn)
        << ": No handler registered for input events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << ready.events << std::dec;
    reg->in_fn();
    // A raw completion is one user-visible completion, so it is all this
    // Poll() delivers.  The legacy path is the documented exception -- EPoll
    // ran one fd's readable, writable and error handlers together and callers
    // rely on that -- but a raw AsyncRead and AsyncWrite on one fd are two
    // independent completions and are rationed like any others.  Readiness is
    // level-triggered, so the write side is reported again on the next Poll().
    if (raw_only) {
      return true;
    }
  }

  if (reg->fd != kInvalidFd && (dispatch & kOutEvents)) {
    ABSL_CHECK(reg->out_fn)
        << ": No handler registered for output events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << ready.events << std::dec;
    reg->out_fn();
  }

  // Gated on reg->events -- the mask the caller subscribed to through On*()
  // -- rather than firing for every registration: one carrying only raw
  // requests is not a shape EPoll could hold, and it consumed the error
  // above through its own handler.
  if (reg->fd != kInvalidFd && (ready.events & kErrorEvents) &&
      reg->events != 0) {
    ABSL_CHECK(reg->err_fn)
        << ": No handler registered for error events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << ready.events << std::dec
        << ". " << GetSocketErrorStr(reg->fd);
    reg->err_fn();
  }
  return true;
}

}  // namespace aos::internal

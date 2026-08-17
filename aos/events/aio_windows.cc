#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "aos/events/aio.h"

#include <mswsock.h>
#include <timeapi.h>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/containers/sized_array.h"
#include "aos/events/aio_internal.h"
#include "aos/events/winsock_init.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

ABSL_FLAG(std::string, aio_backend, "iocp",
          "Which Aio backend to use.  Windows has only the IOCP backend, so "
          "this exists to keep the flag's name and meaning the same on every "
          "platform; it is accepted and ignored.");

namespace aos {

#include <errno.h>

int TranslateWinsockError(int err) {
  switch (err) {
    case WSAENOTSOCK:
    case WSAEBADF:
      return EBADF;
    case WSAEACCES:
      return EACCES;
    case WSAEADDRINUSE:
      return EADDRINUSE;
    case WSAEADDRNOTAVAIL:
      return EADDRNOTAVAIL;
    case WSAEAFNOSUPPORT:
      return EAFNOSUPPORT;
    case WSAEALREADY:
      return EALREADY;
    case WSAECONNABORTED:
      return ECONNABORTED;
    case WSAECONNREFUSED:
      return ECONNREFUSED;
    case WSAECONNRESET:
      return ECONNRESET;
    case WSAEDESTADDRREQ:
      return EDESTADDRREQ;
    case WSAEFAULT:
      return EFAULT;
    case WSAEHOSTUNREACH:
      return EHOSTUNREACH;
    case WSAEINPROGRESS:
      return EINPROGRESS;
    case WSAEINTR:
      return EINTR;
    case WSAEINVAL:
      return EINVAL;
    case WSAEISCONN:
      return EISCONN;
    case WSAEMFILE:
      return EMFILE;
    case WSAEMSGSIZE:
      return EMSGSIZE;
    case WSAENETDOWN:
      return ENETDOWN;
    case WSAENETRESET:
      return ENETRESET;
    case WSAENETUNREACH:
      return ENETUNREACH;
    case WSAENOBUFS:
      return ENOBUFS;
    case WSAENOPROTOOPT:
      return ENOPROTOOPT;
    case WSAENOTCONN:
      return ENOTCONN;
    case WSAEOPNOTSUPP:
      return EOPNOTSUPP;
    case WSAEPROTONOSUPPORT:
      return EPROTONOSUPPORT;
    case WSAEPROTOTYPE:
      return EPROTOTYPE;
    case WSAETIMEDOUT:
      return ETIMEDOUT;
    case WSAEWOULDBLOCK:
      return EWOULDBLOCK;
    default:
      return err;
  }
}

// On Windows a FileDescriptor is an opaque handle that holds the SOCKET
// directly (see aio.h), so no lookup table is needed to recover it.
inline SOCKET ToSocket(FileDescriptor fd) {
  return reinterpret_cast<SOCKET>(fd);
}

// Raises the process's timer resolution to 1ms, once, for the life of the
// process.
//
// Poll() waits by handing GetQueuedCompletionStatus() a millisecond timeout,
// and that timeout is rounded up to the system timer tick -- ~15.6ms by
// default.  Every timer shorter than that therefore fires late by up to a
// full tick, which is not a jitter problem so much as a correctness one for
// a repeating timer: it is late every single cycle, so it spends the whole
// run catching up on missed deadlines and firing back-to-back.  Linux gives
// us hrtimers with no such request, so this is purely about matching the
// resolution the rest of AOS already assumes.
//
// There is no matching timeEndPeriod(): the resolution is wanted for as long
// as any event loop might run, and since Windows 10 2004 the setting is
// scoped to the calling process, so leaving it raised costs other processes
// nothing.
void EnsureHighResolutionTimers() {
  static const bool initialized = []() {
    // TIMERR_NOERROR is 0.  A failure here is not fatal -- it only means
    // timers keep the coarse default -- so log rather than die.
    const MMRESULT result = timeBeginPeriod(1);
    if (result != TIMERR_NOERROR) {
      ABSL_LOG(WARNING) << "timeBeginPeriod(1) failed with " << result
                        << "; timers will be quantized to the system tick";
    }
    return true;
  }();
  (void)initialized;
}

namespace {
// Per-request state for the Windows IOCP backend.  It lives inside
// AsyncRequest::internal_state so the public AsyncRequest struct stays platform
// agnostic (see aio.h).  Every member is trivially constructible, so the
// zero-initialized internal_state bytes are a valid default-state record and no
// explicit construction is needed (matching the Linux/macOS backends).
struct WindowsAsyncState {
  AsyncRequest *next = nullptr;
  AsyncRequest *prev = nullptr;
  std::span<char> buffer;
  std::span<const char> const_buffer;
  int op_type = 0;                     // 1: Read, 2: Write, 3: Accept
  FileDescriptor client_fd = nullptr;  // For Accept
  int64_t deadline_ns = 0;             // Monotonic timer deadline in ns
};

inline WindowsAsyncState &State(AsyncRequest *request) {
  static_assert(sizeof(WindowsAsyncState) <= sizeof(request->internal_state),
                "WindowsAsyncState too large for AsyncRequest::internal_state");
  static_assert(
      alignof(WindowsAsyncState) <= 8,
      "WindowsAsyncState over-aligned for AsyncRequest::internal_state");
  return *reinterpret_cast<WindowsAsyncState *>(request->internal_state);
}
}  // namespace

struct WindowsTimerState;
struct IocpImpl;

// Thread pool callbacks
VOID CALLBACK CustomHandleCallback(PVOID lpParameter, BOOLEAN TimerOrWaitFired);

struct IocpImpl : public Aio::Impl {
  IocpImpl();
  ~IocpImpl() override;

  HANDLE iocp_handle = INVALID_HANDLE_VALUE;
  // Starts true so should_run() reports "running" before the first Run(), which
  // is how the original EPoll (run_{true}) behaved.  Run() clears it on exit
  // and Quit() clears it on shutdown.
  std::atomic<bool> run{true};
  std::atomic<bool> quit_requested{false};
  std::vector<std::function<void()>> before_wait_functions;

  // Nonzero while inside Poll(), to enforce that it is not reentrant.  See
  // the CHECK at the top of Poll(), and EpollImpl's dispatch_depth_.
  int dispatch_depth_ = 0;
  // True while running the before-wait functions; see BeforeWait().
  bool in_before_wait_ = false;

  // Intrusive timer queue
  AsyncRequest *timers_head = nullptr;

  void RemoveTimer(AsyncRequest *req);
  void InsertTimer(AsyncRequest *req);

  // Custom Handle Registry
  struct HandleState {
    HANDLE handle = NULL;
    std::function<void()> callback;
    HANDLE wait_handle = NULL;
    std::pair<IocpImpl *, HANDLE> *wait_context = nullptr;
  };
  std::mutex handle_mutex;
  std::unordered_map<HANDLE, HandleState> handle_states;

  // Socket Registry
  struct FdState {
    FileDescriptor fd = nullptr;
    bool associated_with_iocp = false;

    AsyncRequest *read_request = nullptr;
    AsyncRequest *write_request = nullptr;
    AsyncRequest *accept_request = nullptr;

    OVERLAPPED read_overlapped = {};
    OVERLAPPED write_overlapped = {};
    OVERLAPPED accept_overlapped = {};
    char accept_buffer[288] = {};

    bool is_legacy = false;
    uint32_t legacy_events = 0;
    bool read_pending = false;
    bool write_pending = false;

    bool has_events_fn = false;
    bool has_in_fn = false;
    bool has_out_fn = false;
    bool has_err_fn = false;

    std::function<void()> in_fn;
    std::function<void()> out_fn;
    std::function<void()> err_fn;
    std::function<void(uint32_t)> events_fn;
  };
  // Recursive because legacy callbacks (in_fn/out_fn/err_fn/events_fn) are
  // invoked synchronously from within Poll() while this is held, and it is
  // legal for those callbacks to call back into Aio (e.g. SetEvents) from
  // the same thread.
  mutable std::recursive_mutex fd_states_mutex;
  // FdState objects are kept in a vector sorted by fd (binary searched via
  // GetActiveRegistration) rather than a std::unordered_map, and new
  // registrations used by the Async* completion-based API are drawn from a
  // pre-allocated free_list_ pool.  This is required so that AsyncRead/
  // AsyncWrite/AsyncAccept never allocate memory, matching the no-malloc
  // guarantee documented on Aio (see aio.h) and required by ScopedRealtime.
  std::vector<std::unique_ptr<FdState>> registrations_;
  std::vector<std::unique_ptr<FdState>> free_list_;
  // Released but not yet scrubbed; see ReleaseRegistration().
  std::vector<std::unique_ptr<FdState>> retired_;
  size_t initial_pool_size_ = 16;

  // Orders registrations_ by fd.  FileDescriptor is an opaque handle (SOCKET)
  // on Windows, so compare the underlying integer values for a well-defined
  // total order.
  static bool FdLess(const std::unique_ptr<FdState> &reg, FileDescriptor value);

  FdState *GetActiveRegistration(FileDescriptor fd) const;

  // Used by the legacy readiness-based API (OnReadable/OnWritable/OnEvents/
  // RegisterLegacyFd).  Not guaranteed to avoid allocation; these are not
  // meant to be called from realtime code.
  FdState &GetOrCreateLegacyRegistration(FileDescriptor fd);

  // Used by the completion-based API (AsyncRead/AsyncWrite/AsyncAccept),
  // which must not allocate.  Draws from free_list_ instead of the heap.
  FdState &GetOrCreateAsyncRegistration(FileDescriptor fd);

  void ReleaseRegistration(FdState *reg);

  // Finishes what ReleaseRegistration() deferred: destroys the retired
  // states' callbacks and returns their slots to free_list_.  Only legal
  // with dispatch_depth_ == 0.
  void ScrubRetiredRegistrations();

  // CancelIoEx()es whichever readiness watches this state has armed.  The
  // cancelled operations still post their (aborted) completions, and those
  // are what return pending_io_count to zero.
  void CancelPendingWatches(FdState &state);

  // Clears the pending flag a retired state is still holding for `overlapped`,
  // if any, and reports whether it matched.  A completion whose registration
  // has already been retired has nothing to dispatch, but the state must not
  // be recycled until the kernel is finished with the OVERLAPPED living
  // inside it -- otherwise the next AsyncRead() to draw that slot would share
  // its OVERLAPPED with an operation the kernel still owns.
  bool ClearRetiredPending(OVERLAPPED *overlapped);

  // Returns pool slots held by registrations that AsyncRead/AsyncWrite/
  // AsyncAccept created and that now have nothing outstanding.  Without
  // this the pool is a high-water mark of distinct fds ever used
  // asynchronously rather than of simultaneously-outstanding requests, and
  // the 17th one CHECK-fails in GetOrCreateAsyncRegistration().  Matches
  // EpollImpl::MaybeRetireAsyncRegistration().
  //
  // Swept at the end of Poll() rather than inline at each completion: the
  // completion handlers hold an FdState & across the user callback (and
  // that callback may itself start new work on the same fd), so releasing
  // mid-dispatch would hand the slot out from under a live reference.
  void RetireIdleAsyncRegistrations();

  aos::SizedArray<AsyncRequest *, MAXIMUM_WAIT_OBJECTS> cancelled_requests;
  struct FailedRequest {
    AsyncRequest *request;
    int error_code;
  };
  std::mutex failed_requests_mutex;
  // AsyncRead/AsyncWrite/AsyncAccept push onto failed_requests when the
  // syscall fails synchronously (e.g. an invalid fd), and must not
  // allocate.  Pre-reserved once at construction; Poll() takes entries off
  // the front with erase(), which shifts within the existing storage rather
  // than reallocating, so capacity is retained and no push_back ever has to
  // grow it again.
  std::vector<FailedRequest> failed_requests;
  std::atomic<int> pending_io_count{0};

  void AssociateSocket(FdState &state);
  void UpdateSocketLocked(FdState &state);
  void UpdateSocket(FileDescriptor fd);
  void Wakeup();

  std::unique_ptr<Aio::TimerState> MakeTimerState() override;
  // Windows has no fork(), so nothing ever consults this.  Answered honestly
  // rather than hardcoded false, so it stays right if that changes.
  bool HasRawRequestsInFlight() const override;
  void Run() override;
  bool should_run() const override;
  bool Poll(bool block) override;
  void Quit() override;

  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) override;
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) override;
  LPFN_ACCEPTEX LoadAcceptEx(SOCKET listen_sock);
  void AsyncAccept(FileDescriptor listen_fd, FileDescriptor client_fd,
                   AsyncRequest *request);
  void AsyncTimer(aos::monotonic_clock::time_point deadline,
                  AsyncRequest *request);
  void AsyncTimer(uint64_t timeout_ms, AsyncRequest *request);
  void Cancel(AsyncRequest *request) override;
  void BeforeWait(std::function<void()> function) override;

  // Legacy Readiness Hooks.
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

  void OnHandleSignaled(void *handle, std::function<void()> function);
  void DeleteHandle(void *handle);

  // The Event handle each registered receiver is being watched through, so
  // unregistering can find the handle again after the fact.
  std::unordered_map<ipc_lib::ThreadSignalReceiver *, HANDLE> receiver_handles_;
};

IocpImpl::IocpImpl() {
  free_list_.reserve(initial_pool_size_ * 2);
  for (size_t i = 0; i < initial_pool_size_; ++i) {
    free_list_.push_back(std::make_unique<FdState>());
  }
  registrations_.reserve(initial_pool_size_ * 2);
  // Retiring happens on the completion path, which runs under ScopedRealtime
  // and must not allocate.  Sized like registrations_ because in the worst
  // case every active registration retires before anything is scrubbed.
  retired_.reserve(initial_pool_size_ * 2);
  failed_requests.reserve(initial_pool_size_);
}

void IocpImpl::RemoveTimer(AsyncRequest *req) {
  WindowsAsyncState &s = State(req);
  if (s.prev) {
    State(s.prev).next = s.next;
  } else if (timers_head == req) {
    timers_head = s.next;
  }
  if (s.next) {
    State(s.next).prev = s.prev;
  }
  s.next = nullptr;
  s.prev = nullptr;
}

void IocpImpl::InsertTimer(AsyncRequest *req) {
  RemoveTimer(req);
  WindowsAsyncState &s = State(req);
  if (!timers_head) {
    timers_head = req;
    s.next = nullptr;
    s.prev = nullptr;
    return;
  }
  AsyncRequest *curr = timers_head;
  AsyncRequest *prev_node = nullptr;
  while (curr && State(curr).deadline_ns <= s.deadline_ns) {
    prev_node = curr;
    curr = State(curr).next;
  }
  if (!prev_node) {
    s.next = timers_head;
    State(timers_head).prev = req;
    s.prev = nullptr;
    timers_head = req;
  } else {
    s.next = curr;
    s.prev = prev_node;
    State(prev_node).next = req;
    if (curr) {
      State(curr).prev = req;
    }
  }
}

bool IocpImpl::FdLess(const std::unique_ptr<FdState> &reg,
                      FileDescriptor value) {
  return reinterpret_cast<uintptr_t>(reg->fd) <
         reinterpret_cast<uintptr_t>(value);
}

IocpImpl::FdState *IocpImpl::GetActiveRegistration(FileDescriptor fd) const {
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             FdLess);
  if (it != registrations_.end() && (*it)->fd == fd) {
    return it->get();
  }
  return nullptr;
}

IocpImpl::FdState &IocpImpl::GetOrCreateLegacyRegistration(FileDescriptor fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    return *reg;
  }
  auto new_reg = std::make_unique<FdState>();
  auto *ptr = new_reg.get();
  ptr->fd = fd;
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             FdLess);
  registrations_.insert(it, std::move(new_reg));
  return *ptr;
}

IocpImpl::FdState &IocpImpl::GetOrCreateAsyncRegistration(FileDescriptor fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    return *reg;
  }
  ABSL_CHECK(!free_list_.empty())
      << "Async registration pool exhausted for fd " << fd;
  auto owned_reg = std::move(free_list_.back());
  free_list_.pop_back();
  auto *ptr = owned_reg.get();
  ptr->fd = fd;
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             FdLess);
  registrations_.insert(it, std::move(owned_reg));
  return *ptr;
}

void IocpImpl::ReleaseRegistration(FdState *reg) {
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(),
                             reg->fd, FdLess);
  ABSL_CHECK(it != registrations_.end() && it->get() == reg);

  std::unique_ptr<FdState> owned_reg = std::move(*it);
  registrations_.erase(it);

  // Erasing above is enough to make the registration invisible to lookups.
  // The reset -- which destroys in_fn/out_fn/err_fn/events_fn -- has to wait
  // until no dispatch is in flight: a callback is allowed to delete its own
  // fd, and that callback is one of these std::functions, so destroying it
  // here would free the running lambda's captures out from under it.  Park
  // the state on retired_ and let ScrubRetiredRegistrations() finish the job
  // once the stack is clear.  Matches EpollImpl::ReleaseRegistration().
  retired_.push_back(std::move(owned_reg));
  if (dispatch_depth_ == 0) {
    ScrubRetiredRegistrations();
  }
}

void IocpImpl::CancelPendingWatches(FdState &state) {
  if (state.read_pending) {
    CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
               &state.read_overlapped);
  }
  if (state.write_pending) {
    CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
               &state.write_overlapped);
  }
}

bool IocpImpl::ClearRetiredPending(OVERLAPPED *overlapped) {
  for (auto &reg : retired_) {
    if (overlapped == &reg->read_overlapped) {
      reg->read_pending = false;
      reg->read_request = nullptr;
      return true;
    }
    if (overlapped == &reg->write_overlapped) {
      reg->write_pending = false;
      reg->write_request = nullptr;
      return true;
    }
  }
  return false;
}

void IocpImpl::ScrubRetiredRegistrations() {
  ABSL_CHECK_EQ(dispatch_depth_, 0);
  size_t kept = 0;
  for (size_t i = 0; i < retired_.size(); ++i) {
    std::unique_ptr<FdState> reg = std::move(retired_[i]);
    // Still owed a completion: the kernel holds a pointer into this object,
    // so it stays parked until that lands (ClearRetiredPending() clears the
    // flag, ~IocpImpl() forces the issue by cancelling).
    if (reg->read_pending || reg->write_pending || reg->read_request ||
        reg->write_request || reg->accept_request) {
      retired_[kept++] = std::move(reg);
      continue;
    }
    *reg = FdState();
    if (free_list_.size() < 2 * initial_pool_size_) {
      free_list_.push_back(std::move(reg));
    }
  }
  retired_.resize(kept);
}

void IocpImpl::RetireIdleAsyncRegistrations() {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  // Walk backwards: ReleaseRegistration() erases from registrations_, so
  // indices at or past the erased one shift.
  for (size_t i = registrations_.size(); i > 0; --i) {
    FdState *reg = registrations_[i - 1].get();
    // Legacy registrations belong to the caller until DeleteFd(); only the
    // ones the Async* API created on demand are ours to reclaim.
    if (reg->is_legacy || reg->has_events_fn || reg->has_in_fn ||
        reg->has_out_fn || reg->has_err_fn) {
      continue;
    }
    if (reg->read_request != nullptr || reg->write_request != nullptr ||
        reg->accept_request != nullptr || reg->read_pending ||
        reg->write_pending) {
      continue;
    }
    // Note the socket stays associated with the completion port -- that is
    // not undoable -- so a later AsyncRead() on this same fd builds a fresh
    // registration whose associated_with_iocp starts false.  AssociateSocket()
    // handles that: re-associating an already-associated socket fails with
    // ERROR_INVALID_PARAMETER, which it treats as success.
    ReleaseRegistration(reg);
  }
  // Also finishes off anything a callback retired mid-dispatch (DeleteFd
  // from inside its own handler), which ReleaseRegistration() could only
  // park at the time.
  ScrubRetiredRegistrations();
}

void IocpImpl::AssociateSocket(FdState &state) {
  if (!state.associated_with_iocp) {
    SOCKET s = ToSocket(state.fd);
    if (s == INVALID_SOCKET) {
      return;
    }
    HANDLE socket_handle = reinterpret_cast<HANDLE>(s);
    HANDLE res = CreateIoCompletionPort(
        socket_handle, iocp_handle, reinterpret_cast<ULONG_PTR>(state.fd), 0);
    if (res == iocp_handle) {
      state.associated_with_iocp = true;
    } else {
      int err = GetLastError();
      if (err == ERROR_INVALID_PARAMETER) {
        // Already associated with a completion port.  Normally that port is
        // ours -- this is a re-registration of an fd we retired earlier, and
        // re-associating is both harmless and unavoidable, since the state
        // carrying associated_with_iocp went back to the pool.
        //
        // It can also mean the socket belongs to a *different* Aio's port,
        // which is unrecoverable: a socket's completion-port association is
        // permanent for the life of the socket, so this Aio will never see a
        // completion for it and any Poll() waiting on one blocks forever.
        // The two cases are indistinguishable here -- Windows offers no way
        // to ask a socket which port it is on -- so this stays permissive.
        // Callers must not hand the same socket to two Aio instances.
        state.associated_with_iocp = true;
      } else {
        // AssociateSocket is reachable from AsyncRead/AsyncWrite/
        // AsyncAccept, which must not allocate (see aio.h).  ABSL_LOG
        // streams through the full logging pipeline, which can allocate
        // (especially on first use); ABSL_RAW_LOG does not.
        ABSL_RAW_LOG(WARNING,
                     "Failed to associate socket fd=%p (handle=%p, "
                     "iocp=%p) with IOCP: %d",
                     state.fd, socket_handle, iocp_handle, err);
      }
    }
  }
}

void IocpImpl::UpdateSocketLocked(FdState &state) {
  if (state.is_legacy) {
    // kErr (0x08) is folded into want_read: a socket is essentially always
    // immediately writable, so treating kErr as wanting a write watch too
    // would fire a spurious "writable" completion the instant the watch is
    // armed.  A read watch, on the other hand, only completes once there
    // is real data, an orderly shutdown, or an actual error -- so it is
    // safe (and sufficient) to use it for error/hangup detection too.
    bool want_read = (state.legacy_events & (0x01 | 0x02 | 0x08)) != 0;
    bool want_write = (state.legacy_events & 0x04) != 0;

    if (want_read) {
      if (!state.read_request && !state.read_pending) {
        state.read_pending = true;
        WSABUF buf = {0, nullptr};
        DWORD flags = 0;
        std::memset(&state.read_overlapped, 0, sizeof(state.read_overlapped));
        int res = WSARecv(ToSocket(state.fd), &buf, 1, nullptr, &flags,
                          &state.read_overlapped, nullptr);
        if (res == 0 || WSAGetLastError() == WSA_IO_PENDING) {
          pending_io_count++;
        } else {
          state.read_pending = false;
        }
      }
    } else {
      if (state.read_pending) {
        CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
                   &state.read_overlapped);
      }
    }

    if (want_write) {
      if (!state.write_request && !state.write_pending) {
        state.write_pending = true;
        WSABUF buf = {0, nullptr};
        std::memset(&state.write_overlapped, 0, sizeof(state.write_overlapped));
        int res = WSASend(ToSocket(state.fd), &buf, 1, nullptr, 0,
                          &state.write_overlapped, nullptr);
        if (res == 0 || WSAGetLastError() == WSA_IO_PENDING) {
          pending_io_count++;
        } else {
          state.write_pending = false;
        }
      }
    } else {
      if (state.write_pending) {
        CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
                   &state.write_overlapped);
      }
    }
  }
}

void IocpImpl::UpdateSocket(FileDescriptor fd) {
  auto &state = GetOrCreateLegacyRegistration(fd);
  state.fd = fd;
  AssociateSocket(state);
  UpdateSocketLocked(state);
}

void IocpImpl::Wakeup() {
  PostQueuedCompletionStatus(iocp_handle, 0, static_cast<ULONG_PTR>(-3),
                             nullptr);
}

IocpImpl::~IocpImpl() {
  // Matches IoUringImpl/EpollImpl/KqueueImpl.  Nothing can observe it after
  // this point, but leaving one backend's teardown different from the other
  // three is a difference someone has to re-derive as harmless later.
  run = false;

  while (timers_head) {
    Cancel(timers_head);
  }

  // Unregister wait handles for custom handles
  {
    std::lock_guard<std::mutex> lock(handle_mutex);
    for (auto &pair : handle_states) {
      if (pair.second.wait_handle != NULL) {
        UnregisterWait(pair.second.wait_handle);
      }
      if (pair.second.wait_context != nullptr) {
        delete pair.second.wait_context;
      }
    }
  }

  // Unregister wait handles for legacy sockets
  std::vector<FileDescriptor> active_fds;
  {
    std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
    for (const auto &reg : registrations_) {
      active_fds.push_back(reg->fd);
    }
  }
  for (FileDescriptor fd : active_fds) {
    AsyncRequest *read_req = nullptr;
    AsyncRequest *write_req = nullptr;
    AsyncRequest *accept_req = nullptr;
    {
      std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
      if (auto *state = GetActiveRegistration(fd)) {
        read_req = state->read_request;
        write_req = state->write_request;
        accept_req = state->accept_request;
      }
    }
    if (read_req) Cancel(read_req);
    if (write_req) Cancel(write_req);
    if (accept_req) Cancel(accept_req);
    // The Cancel()s above only cover the completion-based requests.  A
    // legacy fd's readiness watch is an overlapped operation too, and it
    // counts toward pending_io_count just the same, so the drain below hangs
    // forever unless it is cancelled as well.
    {
      std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
      if (auto *state = GetActiveRegistration(fd)) {
        CancelPendingWatches(*state);
      }
    }
  }
  // Same for anything retired but not yet recycled.
  {
    std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
    for (auto &reg : retired_) {
      CancelPendingWatches(*reg);
    }
  }

  // Wait for all pending overlapped operations to complete.  Everything that
  // incremented pending_io_count has been cancelled by this point -- the
  // requests above, and the readiness watches via CancelPendingWatches() --
  // so each one still outstanding owes exactly one (aborted) completion.
  while (pending_io_count > 0) {
    DWORD bytes = 0;
    ULONG_PTR key = 0;
    LPOVERLAPPED overlapped = nullptr;
    GetQueuedCompletionStatus(iocp_handle, &bytes, &key, &overlapped, 10);
    if (overlapped) {
      pending_io_count--;
    }
  }

  if (iocp_handle != INVALID_HANDLE_VALUE) {
    CloseHandle(iocp_handle);
    iocp_handle = INVALID_HANDLE_VALUE;
  }
  // No WSACleanup() to match the startup in Aio::Aio(): see
  // EnsureWinsockInitialized().
}

void IocpImpl::Run() {
  if (quit_requested) {
    quit_requested = false;
    return;
  }
  run = true;
  // Block while we are running; once Quit() lands, switch to non-blocking polls
  // so whatever is already queued gets flushed before Run() returns.  This
  // mirrors the Linux and macOS backends -- see the commentary on
  // IoUringImpl::Run() for why the drain exists and why this reads should_run()
  // rather than run.
  while (true) {
    if (!Poll(should_run())) {
      // Poll() found nothing to do.  If a shutdown was requested, the queue is
      // drained now and we are done; otherwise it just came back early and we
      // go back to waiting.
      if (!should_run()) {
        break;
      }
    }
  }
  run = false;
  quit_requested = false;
}

bool IocpImpl::should_run() const { return run && !quit_requested; }

void IocpImpl::Quit() {
  // Already asked to stop.  Bail out rather than re-arming the wakeup: once
  // Run() is draining it polls without blocking, so a Quit() called from a
  // BeforeWait callback (or any other per-Poll path) would refill the
  // completion queue every time around and the drain would never finish.  Same
  // guard as the other backends.
  if (quit_requested) {
    return;
  }

  quit_requested = true;
  run = false;
  Wakeup();
}

void IocpImpl::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                         AsyncRequest *request) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateAsyncRegistration(fd);
  state.fd = fd;
  ABSL_CHECK(!state.has_events_fn)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!state.has_in_fn)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  ABSL_CHECK(state.read_request == nullptr)
      << "Duplicate AsyncRead on fd " << fd;

  AssociateSocket(state);

  State(request).buffer = buffer;
  State(request).op_type = 1;
  request->done = false;
  state.read_request = request;

  std::memset(&state.read_overlapped, 0, sizeof(state.read_overlapped));

  WSABUF wsa_buf;
  wsa_buf.buf = buffer.data();
  wsa_buf.len = static_cast<ULONG>(buffer.size());

  DWORD flags = 0;
  int res = WSARecv(ToSocket(fd), &wsa_buf, 1, nullptr, &flags,
                    &state.read_overlapped, nullptr);
  if (res == 0 || WSAGetLastError() == WSA_IO_PENDING) {
    pending_io_count++;
  } else {
    int err = WSAGetLastError();
    state.read_request = nullptr;
    {
      std::lock_guard<std::mutex> lock_failed(failed_requests_mutex);
      failed_requests.push_back({request, err});
    }
    Wakeup();
  }
}

void IocpImpl::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                          AsyncRequest *request) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateAsyncRegistration(fd);
  state.fd = fd;
  ABSL_CHECK(!state.has_events_fn)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!state.has_out_fn)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  ABSL_CHECK(state.write_request == nullptr)
      << "Duplicate AsyncWrite on fd " << fd;

  AssociateSocket(state);

  State(request).const_buffer = buffer;
  State(request).op_type = 2;
  request->done = false;
  state.write_request = request;

  std::memset(&state.write_overlapped, 0, sizeof(state.write_overlapped));

  WSABUF wsa_buf;
  wsa_buf.buf = const_cast<char *>(buffer.data());
  wsa_buf.len = static_cast<ULONG>(buffer.size());

  int res = WSASend(ToSocket(fd), &wsa_buf, 1, nullptr, 0,
                    &state.write_overlapped, nullptr);
  if (res == 0 || WSAGetLastError() == WSA_IO_PENDING) {
    pending_io_count++;
  } else {
    int err = WSAGetLastError();
    state.write_request = nullptr;
    {
      std::lock_guard<std::mutex> lock_failed(failed_requests_mutex);
      failed_requests.push_back({request, err});
    }
    Wakeup();
  }
}

LPFN_ACCEPTEX IocpImpl::LoadAcceptEx(SOCKET listen_sock) {
  LPFN_ACCEPTEX lpfnAcceptEx = nullptr;
  GUID GuidAcceptEx = WSAID_ACCEPTEX;
  DWORD dwBytes = 0;
  WSAIoctl(listen_sock, SIO_GET_EXTENSION_FUNCTION_POINTER, &GuidAcceptEx,
           sizeof(GuidAcceptEx), &lpfnAcceptEx, sizeof(lpfnAcceptEx), &dwBytes,
           nullptr, nullptr);
  return lpfnAcceptEx;
}

void IocpImpl::AsyncAccept(FileDescriptor listen_fd, FileDescriptor client_fd,
                           AsyncRequest *request) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateAsyncRegistration(listen_fd);
  state.fd = listen_fd;
  ABSL_CHECK(!state.has_events_fn)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << listen_fd;
  ABSL_CHECK(state.accept_request == nullptr)
      << "Duplicate AsyncAccept on fd " << listen_fd;

  AssociateSocket(state);

  State(request).op_type = 3;
  State(request).client_fd = client_fd;
  request->done = false;
  state.accept_request = request;

  std::memset(&state.accept_overlapped, 0, sizeof(state.accept_overlapped));

  LPFN_ACCEPTEX lpfnAcceptEx = LoadAcceptEx(ToSocket(listen_fd));
  ABSL_CHECK(lpfnAcceptEx != nullptr) << "Failed to load AcceptEx";

  DWORD bytes_received = 0;
  BOOL res = lpfnAcceptEx(ToSocket(listen_fd), ToSocket(client_fd),
                          state.accept_buffer, 0, sizeof(sockaddr_storage) + 16,
                          sizeof(sockaddr_storage) + 16, &bytes_received,
                          &state.accept_overlapped);
  if (res || WSAGetLastError() == ERROR_IO_PENDING) {
    pending_io_count++;
  } else {
    int err = WSAGetLastError();
    state.accept_request = nullptr;
    {
      std::lock_guard<std::mutex> lock_failed(failed_requests_mutex);
      failed_requests.push_back({request, err});
    }
    Wakeup();
  }
}

void IocpImpl::AsyncTimer(aos::monotonic_clock::time_point deadline,
                          AsyncRequest *request) {
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  State(request).deadline_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          deadline.time_since_epoch())
          .count();
  InsertTimer(request);
}

void IocpImpl::AsyncTimer(uint64_t timeout_ms, AsyncRequest *request) {
  AsyncTimer(monotonic_clock::now() + std::chrono::milliseconds(timeout_ms),
             request);
}

void IocpImpl::Cancel(AsyncRequest *request) {
  // Cancel from active timer queue
  AsyncRequest *curr = timers_head;
  while (curr) {
    if (curr == request) {
      RemoveTimer(request);
      request->done = true;
      {
        std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
        cancelled_requests.push_back(request);
      }
      Wakeup();
      return;
    }
    curr = State(curr).next;
  }

  // Cancel from active I/O operation queue
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  for (auto &reg : registrations_) {
    auto &state = *reg;
    if (state.read_request == request) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
                 &state.read_overlapped);
      return;
    }
    if (state.write_request == request) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
                 &state.write_overlapped);
      return;
    }
    if (state.accept_request == request) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state.fd)),
                 &state.accept_overlapped);
      return;
    }
  }
}

void IocpImpl::BeforeWait(std::function<void()> function) {
  // Not from inside a before-wait function: the push_back can reallocate the
  // vector while Poll()'s iteration is executing an element in the old
  // storage.  Deterministically illegal rather than sometimes-corrupting,
  // matching IoUringImpl/EpollImpl.
  ABSL_CHECK(!in_before_wait_)
      << ": BeforeWait() may not be called from a before-wait function";
  before_wait_functions.push_back(std::move(function));
}

void IocpImpl::OnReadable(FileDescriptor fd, std::function<void()> callback) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!state.has_events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  ABSL_CHECK(state.read_request == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  ABSL_CHECK(!state.has_in_fn) << "Duplicate in functions for " << fd;
  state.fd = fd;
  state.is_legacy = true;
  state.has_in_fn = true;
  state.legacy_events |= 1;  // kIn
  state.in_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::OnError(FileDescriptor fd, std::function<void()> callback) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!state.has_events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  ABSL_CHECK(!state.has_err_fn) << "Duplicate error functions for " << fd;
  state.fd = fd;
  state.is_legacy = true;
  state.has_err_fn = true;
  state.legacy_events |= 8;  // kErr
  state.err_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::OnWritable(FileDescriptor fd, std::function<void()> callback) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!state.has_events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  ABSL_CHECK(state.write_request == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  ABSL_CHECK(!state.has_out_fn) << "Duplicate out functions for " << fd;
  state.fd = fd;
  state.is_legacy = true;
  state.has_out_fn = true;
  state.legacy_events |= 4;  // kOut
  state.out_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::OnEvents(FileDescriptor fd,
                        std::function<void(uint32_t)> callback) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto &state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(state.read_request == nullptr && state.write_request == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!state.has_in_fn && !state.has_out_fn && !state.has_err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!state.has_events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  state.fd = fd;
  state.is_legacy = true;
  state.has_events_fn = true;
  state.events_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::DeleteFd(FileDescriptor fd) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  auto &state = *reg;
  state.is_legacy = false;
  state.legacy_events = 0;
  state.has_events_fn = false;
  state.has_in_fn = false;
  state.has_out_fn = false;
  state.has_err_fn = false;
  // The std::functions themselves are deliberately left alone here: a
  // callback deleting its own fd is legal, and it is one of them, so
  // clearing it would destroy the running lambda's captures mid-call.  The
  // has_*_fn flags above already stop anything from dispatching to it, and
  // ReleaseRegistration() -> ScrubRetiredRegistrations() destroys them once
  // no dispatch is in flight.

  // Cancel the readiness watches explicitly rather than letting
  // UpdateSocket() notice that legacy_events is now empty: is_legacy was
  // just cleared, and UpdateSocketLocked() returns immediately for a
  // non-legacy state, so its cancel branch is unreachable from here.  A
  // watch left armed keeps its pending_io_count reference forever, which
  // wedges ~IocpImpl()'s drain.
  CancelPendingWatches(state);
  UpdateSocket(fd);
  ReleaseRegistration(reg);
}

void IocpImpl::ForgetClosedFd(FileDescriptor fd) { DeleteFd(fd); }

void IocpImpl::EnableWritable(FileDescriptor fd) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  auto &state = *reg;
  ABSL_CHECK(!state.has_events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  state.legacy_events |= 4;  // kOut
  UpdateSocket(fd);
}

void IocpImpl::DisableWritable(FileDescriptor fd) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  auto &state = *reg;
  ABSL_CHECK(!state.has_events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  state.legacy_events &= ~4;  // kOut
  UpdateSocket(fd);
}

void IocpImpl::SetEvents(FileDescriptor fd, uint32_t events) {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  auto &state = *reg;
  ABSL_CHECK(state.has_events_fn)
      << "SetEvents is only for fds registered using OnEvents";
  state.legacy_events = events;
  UpdateSocket(fd);
}

// Windows has no signalfd, so instead of watching an fd we watch the receiver's
// auto-reset Event through the same handle-waiting path used for any other
// Win32 object.
void IocpImpl::RegisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver, std::function<void()> callback) {
  ABSL_CHECK(receiver != nullptr);
  ABSL_CHECK(receiver_handles_.empty())
      << "Duplicate ThreadSignalReceiver registration: only one receiver "
         "may be active at a time (see Aio::RegisterThreadSignalReceiver)";
  HANDLE handle = receiver->event_handle();
  auto [it, inserted] = receiver_handles_.try_emplace(receiver, handle);
  ABSL_CHECK(inserted) << "Duplicate ThreadSignalReceiver registration";
  OnHandleSignaled(handle, std::move(callback));
}

void IocpImpl::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  auto it = receiver_handles_.find(receiver);
  ABSL_CHECK(it != receiver_handles_.end()) << "ThreadSignalReceiver not found";
  DeleteHandle(it->second);
  receiver_handles_.erase(it);
}

void IocpImpl::ConsumeThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  receiver->ConsumeWakeup();
}

void IocpImpl::OnHandleSignaled(void *handle, std::function<void()> function) {
  std::lock_guard<std::mutex> lock(handle_mutex);
  auto &state = handle_states[handle];
  state.handle = handle;
  state.callback = std::move(function);

  if (state.wait_handle == NULL) {
    state.wait_context = new std::pair<IocpImpl *, HANDLE>(this, handle);
    RegisterWaitForSingleObject(&state.wait_handle, handle,
                                CustomHandleCallback, state.wait_context,
                                INFINITE, WT_EXECUTEONLYONCE);
  }
}

void IocpImpl::DeleteHandle(void *handle) {
  std::lock_guard<std::mutex> lock(handle_mutex);
  auto it = handle_states.find(handle);
  if (it != handle_states.end()) {
    if (it->second.wait_handle != NULL) {
      UnregisterWait(it->second.wait_handle);
    }
    if (it->second.wait_context != nullptr) {
      delete it->second.wait_context;
    }
    handle_states.erase(it);
  }
}

VOID CALLBACK CustomHandleCallback(PVOID lpParameter,
                                   BOOLEAN TimerOrWaitFired) {
  auto *context = static_cast<std::pair<IocpImpl *, HANDLE> *>(lpParameter);
  PostQueuedCompletionStatus(context->first->iocp_handle, 0,
                             static_cast<ULONG_PTR>(-1),
                             reinterpret_cast<LPOVERLAPPED>(context->second));
}

struct WindowsTimerState : public Aio::TimerState {
  explicit WindowsTimerState(IocpImpl *impl) : impl_(impl) {}
  ~WindowsTimerState() override { Cancel(false); }

  void Initialize() override { request.done = true; }

  void Schedule(aos::monotonic_clock::time_point deadline,
                CompletionCallback callback, void *context) override {
    // Match the Linux backends (IoUringTimerState/EpollTimerState): a timer
    // scheduled before the monotonic epoch is a caller bug, not something to
    // silently clamp.
    ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
    Cancel(false);

    this->deadline = deadline;
    this->user_callback = callback;
    this->user_context = context;

    request.callback = [](Completion completion, void *ctx) {
      auto *state = static_cast<WindowsTimerState *>(ctx);
      if (completion.status.has_value()) {
        // One-shot: resolve before dispatching -- the callback may destroy
        // this timer.
        state->request.done = true;
        state->user_callback({aos::Ok(), 1, state->user_context},
                             state->user_context);
      } else {
        state->request.done = true;
        state->user_callback(completion, state->user_context);
      }
    };
    request.context = this;
    request.done = false;
    State(&request).deadline_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            deadline.time_since_epoch())
            .count();

    impl_->InsertTimer(&request);
  }

  void Cancel(bool /*reap*/) override {
    if (request.done) return;
    impl_->RemoveTimer(&request);
    request.done = true;
    user_callback = nullptr;
  }

 private:
  IocpImpl *impl_;
};

std::unique_ptr<Aio::TimerState> IocpImpl::MakeTimerState() {
  return std::make_unique<WindowsTimerState>(this);
}

bool IocpImpl::HasRawRequestsInFlight() const {
  std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
  for (const auto &reg : registrations_) {
    if (!reg) continue;
    if (reg->read_request != nullptr || reg->write_request != nullptr) {
      return true;
    }
  }
  return false;
}

bool IocpImpl::Poll(bool block) {
  // Not reentrant, matching IoUringImpl::Poll() and EpollImpl::Poll().
  // dispatch_depth_ covers the whole body below, before-wait functions
  // included.
  ABSL_CHECK_EQ(dispatch_depth_, 0)
      << "Aio::Poll() reentered from inside a completion callback or "
         "before-wait function; wait by returning to the event loop instead";
  // Declared before dispatch_depth_guard so it destructs after it: the sweep
  // must see dispatch_depth_ back at 0, with no callback frame live.
  struct RetireGuard {
    IocpImpl *impl;
    ~RetireGuard() { impl->RetireIdleAsyncRegistrations(); }
  } retire_guard{this};
  struct DispatchDepth {
    int *depth;
    explicit DispatchDepth(int *d) : depth(d) { ++*depth; }
    ~DispatchDepth() { --*depth; }
  } dispatch_depth_guard(&dispatch_depth_);

  // NOTE: Do not drop realtime here.  Poll() must run at whatever realtime
  // level the caller was at, exactly like the Linux backends: user callbacks
  // (timers, watchers, before-wait hooks) are dispatched from inside this
  // function and are expected to observe the caller's realtime state.  The
  // blocking wait itself is fine to perform while realtime (a sleeping thread
  // allocates nothing), so there is nothing to bracket with ScopedNotRealtime.
  in_before_wait_ = true;
  for (const auto &fn : before_wait_functions) {
    fn();
  }
  in_before_wait_ = false;

  bool processed = false;

  // At most one completion callback per Poll() -- see Aio::Poll().  Each of
  // the three sources below delivers one and returns; the rest wait for the
  // next Poll(), the same way io_uring's pending_dispatch_ does.  The loops
  // keep popping only past entries that deliver nothing (no callback at
  // all), which is not an observable dispatch.

  {
    std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
    for (auto &reg : registrations_) {
      if (reg->is_legacy) {
        UpdateSocketLocked(*reg);
      }
    }
  }

  // Handle cancelled requests asynchronously.  Taken one at a time rather
  // than drained into a local: erase(begin()) shifts within the existing
  // storage, so this keeps the never-allocate property the batch drain had
  // without needing a scratch buffer to preserve capacity.
  while (true) {
    AsyncRequest *req = nullptr;
    {
      std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
      if (cancelled_requests.empty()) {
        break;
      }
      req = cancelled_requests.front();
      cancelled_requests.erase(cancelled_requests.begin());
    }
    if (req->callback) {
      req->callback({aos::MakeError("Canceled"), 0, req->user_data},
                    req->context);
      return true;
    }
  }

  // Handle failed requests asynchronously.  Erasing the front in place keeps
  // failed_requests' capacity, which is what the pre-reserved scratch buffer
  // used to be for -- see the comment on failed_requests.
  while (true) {
    FailedRequest fr;
    {
      std::lock_guard<std::mutex> lock_failed(failed_requests_mutex);
      if (failed_requests.empty()) {
        break;
      }
      fr = failed_requests.front();
      failed_requests.erase(failed_requests.begin());
    }
    fr.request->done = true;
    if (fr.request->callback) {
      fr.request->callback(
          {aos::MakeError("iocp error"), TranslateWinsockError(fr.error_code),
           fr.request->user_data},
          fr.request->context);
      return true;
    }
  }

  while (true) {
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      monotonic_clock::now().time_since_epoch())
                      .count();
    DWORD timeout_dw = INFINITE;
    if (!block || processed) {
      timeout_dw = 0;
    } else if (timers_head) {
      const int64_t head_deadline_ns = State(timers_head).deadline_ns;
      if (head_deadline_ns <= now_ns) {
        timeout_dw = 0;
      } else {
        timeout_dw =
            static_cast<DWORD>((head_deadline_ns - now_ns + 999999) / 1000000);
      }
    }

    DWORD bytes_transferred = 0;
    ULONG_PTR completion_key = 0;
    LPOVERLAPPED lp_overlapped = nullptr;

    BOOL success =
        GetQueuedCompletionStatus(iocp_handle, &bytes_transferred,
                                  &completion_key, &lp_overlapped, timeout_dw);

    if (success || lp_overlapped != nullptr) {
      processed = true;

      if (completion_key == static_cast<ULONG_PTR>(-3)) {
        // Wakeup signal
      } else if (completion_key == static_cast<ULONG_PTR>(-1)) {
        // Custom handle signaled
        HANDLE handle = reinterpret_cast<HANDLE>(lp_overlapped);
        std::function<void()> cb;
        {
          std::lock_guard<std::mutex> lock(handle_mutex);
          auto it = handle_states.find(handle);
          if (it != handle_states.end()) {
            cb = it->second.callback;
            // Re-register wait handle
            UnregisterWait(it->second.wait_handle);
            it->second.wait_handle = NULL;
            RegisterWaitForSingleObject(&it->second.wait_handle,
                                        it->second.handle, CustomHandleCallback,
                                        it->second.wait_context, INFINITE,
                                        WT_EXECUTEONLYONCE);
          }
        }
        if (cb) {
          cb();
        }
      } else {
        // Otherwise, it is an asynchronous overlapped socket completion
        pending_io_count--;
        FileDescriptor fd = reinterpret_cast<FileDescriptor>(completion_key);
        std::lock_guard<std::recursive_mutex> lock(fd_states_mutex);
        // A completion for an fd that has since been retired (DeleteFd, or a
        // request cancelled at teardown) still has to release the hold the
        // kernel had on that state before it can be recycled.
        ClearRetiredPending(lp_overlapped);
        if (auto *state_ptr = GetActiveRegistration(fd)) {
          auto &state = *state_ptr;
          if (lp_overlapped == &state.read_overlapped) {
            if (state.read_request) {
              auto *req = state.read_request;
              state.read_request = nullptr;
              if (req) {
                req->done = true;
                if (req->callback) {
                  if (success) {
                    // Note a zero-byte completion on a non-empty buffer is
                    // EOF -- the peer hung up -- and that is a *successful*
                    // zero-length read, not a failure.  It is what read(2)
                    // reports as 0, and what the io_uring and kqueue backends
                    // pass through as Ok(); reporting an error here instead
                    // made a hangup indistinguishable from real I/O failure.
                    req->callback(
                        {aos::Ok(), static_cast<int32_t>(bytes_transferred),
                         req->user_data},
                        req->context);
                  } else {
                    DWORD dw_flags = 0;
                    DWORD err = 0;
                    WSAGetOverlappedResult(ToSocket(fd), lp_overlapped,
                                           &bytes_transferred, FALSE,
                                           &dw_flags);
                    err = GetLastError();
                    if (err == ERROR_OPERATION_ABORTED) {
                      req->callback(
                          {aos::MakeError("Canceled"), 0, req->user_data},
                          req->context);
                    } else {
                      req->callback(
                          {aos::MakeError("iocp error"),
                           TranslateWinsockError(static_cast<int32_t>(err)),
                           req->user_data},
                          req->context);
                    }
                  }
                }
              }
            } else if (state.read_pending) {
              state.read_pending = false;
              // The readiness watch is a 0-byte WSARecv, so a successful
              // completion cannot itself distinguish "data is available"
              // from "the peer performed an orderly shutdown" (both
              // complete the same way).  Peek a byte (without consuming it)
              // to tell the two apart, matching epoll's behavior of
              // reporting HUP/ERR alongside or instead of IN.
              uint32_t events = 0;
              if (success) {
                char peek_byte;
                int peek_res = recv(ToSocket(fd), &peek_byte, 1, MSG_PEEK);
                if (peek_res > 0) {
                  events |= 0x01;  // kIn
                } else if (peek_res == 0) {
                  events |= 0x08;  // kErr (EOF/HUP)
                } else if (WSAGetLastError() != WSAEWOULDBLOCK) {
                  events |= 0x08;  // kErr
                }
                // If peek_res < 0 with WSAEWOULDBLOCK, this was a spurious
                // wakeup; the watch is re-armed on the next Poll() call.
              } else {
                events |= 0x08;  // kErr
              }
              // Only report bits the caller actually asked for, except
              // kErr, which (like epoll's EPOLLERR/EPOLLHUP) is always
              // reported since this watch may have been armed purely to
              // detect it.
              events &= state.legacy_events | 0x08;
              if (events != 0) {
                if (state.has_events_fn && state.events_fn) {
                  state.events_fn(events);
                } else {
                  // A bare kErr with no err_fn is routed to in_fn (failing
                  // that, out_fn), matching EpollImpl and DrainLegacyEpoll().
                  // A hangup cannot be consumed by acknowledging it -- only
                  // by a read() that observes the EOF and unregisters -- so
                  // dispatching nothing here would re-fire the watch forever
                  // with no callback ever running.
                  const bool err_like = (events & 0x08) != 0;
                  const bool err_unhandled =
                      err_like && !(state.has_err_fn && state.err_fn);
                  const bool has_in = state.has_in_fn && state.in_fn != nullptr;

                  if ((events & 0x01) || (err_unhandled && has_in)) {
                    if (has_in) {
                      state.in_fn();
                    }
                  }
                  if ((events & 0x04) || (err_unhandled && !has_in)) {
                    if (state.has_out_fn && state.out_fn) {
                      state.out_fn();
                    }
                  }
                  if (err_like && state.has_err_fn && state.err_fn) {
                    state.err_fn();
                  }
                }
              }
            }
          } else if (lp_overlapped == &state.write_overlapped) {
            if (state.write_request) {
              auto *req = state.write_request;
              state.write_request = nullptr;
              if (req) {
                req->done = true;
                if (req->callback) {
                  if (success) {
                    req->callback(
                        {aos::Ok(), static_cast<int32_t>(bytes_transferred),
                         req->user_data},
                        req->context);
                  } else {
                    DWORD dw_flags = 0;
                    DWORD err = 0;
                    WSAGetOverlappedResult(ToSocket(fd), lp_overlapped,
                                           &bytes_transferred, FALSE,
                                           &dw_flags);
                    err = GetLastError();
                    if (err == ERROR_OPERATION_ABORTED) {
                      req->callback(
                          {aos::MakeError("Canceled"), 0, req->user_data},
                          req->context);
                    } else {
                      req->callback(
                          {aos::MakeError("iocp error"),
                           TranslateWinsockError(static_cast<int32_t>(err)),
                           req->user_data},
                          req->context);
                    }
                  }
                }
              }
            } else if (state.write_pending) {
              state.write_pending = false;
              uint32_t events = success ? 0x04 /* kOut */ : 0x08 /* kErr */;
              events &= state.legacy_events | 0x08;
              if (events != 0) {
                if (state.has_events_fn && state.events_fn) {
                  state.events_fn(events);
                } else {
                  if ((events & 0x04) && state.has_out_fn && state.out_fn) {
                    state.out_fn();
                  }
                  if ((events & 0x08) && state.has_err_fn && state.err_fn) {
                    state.err_fn();
                  }
                }
              }
            }
          } else if (lp_overlapped == &state.accept_overlapped) {
            auto *req = state.accept_request;
            state.accept_request = nullptr;
            if (req) {
              req->done = true;
              if (req->callback) {
                if (success) {
                  SOCKET listen_sock = ToSocket(fd);
                  SOCKET client_sock = ToSocket(State(req).client_fd);
                  setsockopt(client_sock, SOL_SOCKET, SO_UPDATE_ACCEPT_CONTEXT,
                             reinterpret_cast<char *>(&listen_sock),
                             sizeof(listen_sock));
                  // The accepted client fd was supplied by the caller in
                  // AsyncAccept, so success just needs the Ok status; the
                  // FileDescriptor does not fit in the int32_t result field.
                  req->callback({aos::Ok(), 0, req->user_data}, req->context);
                } else {
                  DWORD dw_flags = 0;
                  DWORD err = 0;
                  WSAGetOverlappedResult(ToSocket(fd), lp_overlapped,
                                         &bytes_transferred, FALSE, &dw_flags);
                  err = GetLastError();
                  if (err == ERROR_OPERATION_ABORTED) {
                    req->callback(
                        {aos::MakeError("Canceled"), 0, req->user_data},
                        req->context);
                  } else {
                    req->callback(
                        {aos::MakeError("iocp error"),
                         TranslateWinsockError(static_cast<int32_t>(err)),
                         req->user_data},
                        req->context);
                  }
                }
              }
            }
          }
        }
      }
      break;
    } else {
      int err = GetLastError();
      if (err == WAIT_TIMEOUT) {
        if (!block || processed) {
          break;
        }
        auto check_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             monotonic_clock::now().time_since_epoch())
                             .count();
        if (!timers_head || State(timers_head).deadline_ns <= check_now) {
          break;
        }
      } else {
        return processed;
      }
    }
  }

  // Check for expired timers.  One per Poll(), and only if nothing was
  // dispatched above -- see Aio::Poll().  A timer left over here is already
  // past its deadline, so the very next Poll() (Run() calls one immediately)
  // fires it without waiting on anything.
  if (processed) {
    return true;
  }
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    monotonic_clock::now().time_since_epoch())
                    .count();
  while (timers_head && State(timers_head).deadline_ns <= now_ns) {
    auto *req = timers_head;
    RemoveTimer(req);
    req->done = true;
    if (req->callback) {
      req->callback({aos::Ok(), 0, req->user_data}, req->context);
      return true;
    }
  }

  return processed;
}

Aio::Aio() : impl_(std::make_unique<IocpImpl>()) {
  EnsureWinsockInitialized();
  EnsureHighResolutionTimers();

  auto *w_impl = static_cast<IocpImpl *>(impl_.get());
  w_impl->iocp_handle =
      CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 0);
  ABSL_CHECK(w_impl->iocp_handle != NULL) << "Failed to create IOCP";
}

}  // namespace aos

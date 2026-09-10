#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "aos/events/aio.h"

#include <mswsock.h>
#include <timeapi.h>
#include <windows.h>
#include <winsock2.h>
#include <winternl.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/call_once.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/aio_internal.h"
#include "aos/events/aio_windows_internal.h"
#include "aos/events/timer_queue.h"
#include "aos/events/winsock_init.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

ABSL_FLAG(std::string, aio_backend, "iocp",
          "Which Aio backend to use.  Windows has only the IOCP backend, so "
          "this exists to keep the flag's name and meaning the same on every "
          "platform; it is accepted and ignored.");

// Defined by every backend so tests can name it without an #ifdef, the same
// way --aio_backend is.  IOCP has neither a submission queue to exhaust nor a
// completion queue to overflow -- completions are kernel-managed and
// unbounded -- so there is no depth to configure and this is accepted and
// ignored.
ABSL_FLAG(uint32_t, aio_queue_depth, 1024,
          "Unused on Windows: IOCP has no submission or completion queue to "
          "size.  Present so the flag means the same thing on every "
          "platform.");

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

// Whether a Winsock error describes the state of the connection rather than
// a mistake in the call.  These are what a read watch armed on a socket
// whose peer has gone gets back synchronously, and they are the socket's to
// report as kErr; anything else -- a handle that is not a socket, a bad
// argument -- is a bug, and dies where it is found.
inline bool IsSocketConditionError(int err) {
  switch (err) {
    case WSAECONNRESET:
    case WSAECONNABORTED:
    case WSAENETRESET:
    case WSAESHUTDOWN:
    case WSAENOTCONN:
    case WSAEDISCON:
    case WSAETIMEDOUT:
    case WSAENETDOWN:
    case WSAEHOSTUNREACH:
    case WSAENETUNREACH:
      return true;
    default:
      return false;
  }
}

// Raises the process's timer resolution to 1ms, once, for the life of the
// process.
//
// Poll() itself no longer needs this: its deadline is a high-resolution
// waitable timer (see DeadlineTimer), which is exempt from the
// system tick.  It stays for two measured reasons.  The high-resolution
// timer is still tighter with it -- a 500us deadline was 0.18ms late at the
// median with the 1ms tick and 0.47ms without -- and every other wait in the
// process is Sleep()-based and tick-bound, aos::SleepFor() included, so
// dropping it would coarsen those from 1ms back to ~15.6ms in any process
// that hosts an event loop.  Linux gives hrtimers with no such request.
//
// There is no matching timeEndPeriod(): the resolution is wanted for as long
// as any event loop might run, and since Windows 10 2004 the setting is
// scoped to the calling process, so leaving it raised costs other processes
// nothing.
void EnsureHighResolutionTimers() {
  static absl::once_flag once;
  absl::call_once(once, []() {
    // TIMERR_NOERROR is 0.  A failure here is not fatal -- it only means
    // timers keep the coarse default -- so log rather than die.
    const MMRESULT result = timeBeginPeriod(1);
    if (result != TIMERR_NOERROR) {
      ABSL_LOG(WARNING) << "timeBeginPeriod(1) failed with " << result
                        << "; timers will be quantized to the system tick";
    }
  });
}

namespace {
// Per-request state for the Windows IOCP backend.  It lives inside
// AsyncRequest::internal_state so the public AsyncRequest struct stays platform
// agnostic (see aio.h).  Every member is trivially constructible, so the
// zero-initialized internal_state bytes are a valid default-state record and no
// explicit construction is needed (matching the Linux/macOS backends).
struct WindowsAsyncState {
  // A raw AsyncRead()/AsyncWrite() that reaches the kernel keeps nothing here:
  // it hands its buffer over through the WSABUF at submit time and is
  // identified on completion by which OVERLAPPED came back.  That is what
  // leaves room for a whole tree node inside the 64 bytes every backend gets;
  // the fields below are the timer's, and the two after them belong to a raw
  // request that never reached the kernel at all.

  // Nanoseconds since aos::monotonic_clock's epoch; Deadline() below is how
  // everything else reads it.
  int64_t deadline_ns;
  // IocpImpl::timers_' tree node; see TimerQueueTraits.
  AsyncRequest *left;
  AsyncRequest *right;
  AsyncRequest *parent;
  uint64_t sequence;

  // Link and already-translated errno for IocpImpl::failed_requests_.  A raw
  // request whose WSARecv()/WSASend() failed outright still has to be resolved
  // from Poll(), since that is where aio.h delivers every completion, so it
  // waits on that list carrying the result it will be completed with, so
  // dispatching one looks nothing up.  The errno rather than the whole
  // Completion: aos::Status is not trivially constructible, and these bytes
  // have to stay a valid record when they are merely zeroed.  Never set on a
  // timer request -- only AsyncRead()/AsyncWrite() can fail this way, and
  // neither is ever handed a timer's request.
  AsyncRequest *failed_next;
  int32_t failed_errno;

  // The narrow fields last, so the whole record still fits inside the 64
  // bytes every backend gets.
  bool red;
  // Whether this request is in timers_.  A tree cannot answer that the way
  // walking a list could, and Remove() on a node that is not in the tree is
  // not safe -- so the membership question is answered here, in O(1).
  bool queued;
};

static_assert(sizeof(WindowsAsyncState) <= sizeof(AsyncRequest::internal_state),
              "WindowsAsyncState too large for AsyncRequest::internal_state");
static_assert(
    alignof(WindowsAsyncState) <= 8,
    "WindowsAsyncState over-aligned for AsyncRequest::internal_state");

inline WindowsAsyncState *State(AsyncRequest *request) {
  return reinterpret_cast<WindowsAsyncState *>(request->internal_state);
}
inline const WindowsAsyncState *State(const AsyncRequest *request) {
  return reinterpret_cast<const WindowsAsyncState *>(request->internal_state);
}

// Link and key accessors for IocpImpl::timers_.
struct TimerQueueTraits {
  static AsyncRequest *&left(AsyncRequest *node) { return State(node)->left; }
  static AsyncRequest *&right(AsyncRequest *node) { return State(node)->right; }
  static AsyncRequest *&parent(AsyncRequest *node) {
    return State(node)->parent;
  }
  static bool &red(AsyncRequest *node) { return State(node)->red; }
  static uint64_t &sequence(AsyncRequest *node) {
    return State(node)->sequence;
  }
  static aos::monotonic_clock::time_point deadline(const AsyncRequest *node) {
    return aos::monotonic_clock::time_point(
        std::chrono::nanoseconds(State(node)->deadline_ns));
  }
};

// What a timer request is armed for.  deadline_ns is a bare int64 only
// because WindowsAsyncState has to stay trivially constructible and fit in 64
// bytes; everything outside it works in time_points.
inline aos::monotonic_clock::time_point Deadline(const AsyncRequest *request) {
  return TimerQueueTraits::deadline(request);
}

// The FIFO of raw requests whose submit failed outright; see
// IocpImpl::failed_requests_.
struct FailedRequestTraits {
  static AsyncRequest *&next(AsyncRequest *request) {
    return State(request)->failed_next;
  }
};

// The legacy readiness bits, in Aio::OnEvents()'s encoding (the low epoll
// bits; see aio.h).  Spelled out so the masks below read as what they mean.
constexpr uint32_t kIn = 0x01;
constexpr uint32_t kPri = 0x02;
constexpr uint32_t kOut = 0x04;
constexpr uint32_t kErr = 0x08;
}  // namespace

struct WindowsTimerState;
class IocpImpl;

// The wait completion packet API is described in aio_windows_internal.h.
const WaitPacketApi &WaitPackets() {
  static const WaitPacketApi api = []() {
    HMODULE ntdll = GetModuleHandleW(L"ntdll.dll");
    ABSL_CHECK(ntdll != NULL) << ": ntdll.dll is not loaded";
    WaitPacketApi result;
    result.create = reinterpret_cast<NtCreateWaitCompletionPacketFn>(
        GetProcAddress(ntdll, "NtCreateWaitCompletionPacket"));
    result.associate = reinterpret_cast<NtAssociateWaitCompletionPacketFn>(
        GetProcAddress(ntdll, "NtAssociateWaitCompletionPacket"));
    result.cancel = reinterpret_cast<NtCancelWaitCompletionPacketFn>(
        GetProcAddress(ntdll, "NtCancelWaitCompletionPacket"));
    ABSL_CHECK(result.create != nullptr && result.associate != nullptr &&
               result.cancel != nullptr)
        << ": ntdll does not export the wait completion packet API; this "
           "backend needs Windows 8 or later";
    return result;
  }();
  return api;
}

HANDLE CreateWaitPacket() {
  HANDLE packet = NULL;
  const NTSTATUS status = WaitPackets().create(&packet, GENERIC_ALL, nullptr);
  ABSL_CHECK(NtOk(status)) << ": NtCreateWaitCompletionPacket failed: 0x"
                           << std::hex << static_cast<uint32_t>(status);
  return packet;
}

// Completion keys for packets and posts that are not socket I/O.  Poll()
// tells them from an fd's completion by key; the destructor's drain skips
// them because they carry no pending_io_count reference.  A kHandleKey or
// kWritableKey packet carries the FdState it belongs to as its ApcContext,
// which GetQueuedCompletionStatus() hands back as lpOverlapped.
constexpr ULONG_PTR kHandleKey = static_cast<ULONG_PTR>(-1);
constexpr ULONG_PTR kTimerKey = static_cast<ULONG_PTR>(-2);
constexpr ULONG_PTR kWakeupKey = static_cast<ULONG_PTR>(-3);
constexpr ULONG_PTR kWritableKey = static_cast<ULONG_PTR>(-4);

inline bool IsInternalKey(ULONG_PTR key) {
  return key == kHandleKey || key == kTimerKey || key == kWakeupKey ||
         key == kWritableKey;
}

// How long Poll() will block while a write watch is armed; see WriteWatch
// for the one case this covers.  Long enough to be cheap when it is never
// needed, short enough that a wedged writer recovers in well under a
// human-visible pause.
constexpr DWORD kWriteWatchBackstop = 100;

namespace {
// WaitPacket and DeadlineTimer -- the packet, and the one high-resolution
// kernel timer behind IocpImpl::timers_ that is delivered through one with
// kTimerKey -- live in aio_windows_internal.h, shared with the libuv backend.

// A legacy registration whose "fd" is a plain waitable HANDLE rather than a
// socket: what a glib GPollFD is on this platform, and what a
// ThreadSignalReceiver's wakeup Event is, since Windows has no signalfd.
// Neither can join a completion port, so the handle is watched through a
// wait completion packet instead.
//
// One-shot.  Delivery disarms the packet, and it is re-armed only once the
// callback has returned and so had its chance to consume the handle (see
// IocpImpl::DispatchHandleSignal()).  Re-arming a still-signalled handle
// first would queue the packet again at once, faster than Poll() can drain
// -- and this backend may not consume the handle on the owner's behalf: it
// does not own a GPollFD's, and for the wakeup Event, consuming is the
// registered callback's job, as ConsumeWakeup() is on every other backend.
class HandleWatch {
 public:
  // owner is the registration the handle belongs to; it comes back out of
  // the port as the packet's context.
  HandleWatch(HANDLE handle, void *owner) : handle_(handle), owner_(owner) {}
  HandleWatch(const HandleWatch &) = delete;
  HandleWatch &operator=(const HandleWatch &) = delete;

  void Arm(HANDLE port) {
    if (!packet_.armed()) {
      packet_.Arm(port, handle_, kHandleKey, owner_);
    }
  }
  void MarkDelivered() { packet_.MarkDelivered(); }

 private:
  const HANDLE handle_;
  void *const owner_;
  WaitPacket packet_;
};

// The legacy write watch: where OnWritable()'s kOut comes from.  The level
// oracle it asks, SocketWritable(), is in aio_windows_internal.h.
//
// Not the mirror image of the read watch.  The read watch is a zero-byte
// WSARecv, which genuinely pends until data arrives.  A zero-byte WSASend
// has nothing to buffer, so it completes at once no matter how full the
// socket is; there is no state in which it waits.  Written that way by
// symmetry, OnWritable() reported writability that was not there on every
// single Poll(), where epoll and kqueue stay quiet.
//
// So this is WSAEventSelect(FD_WRITE), delivered on a WSAEVENT.  That is a
// waitable handle, so it reaches Poll() through a wait completion packet
// like everything else here.  Measured rather than assumed: it coexists
// with the overlapped zero-byte WSARecv on the same socket, so the read
// watch and its MSG_PEEK are untouched; the stack re-signals it after *any*
// send on the socket fails with WSAEWOULDBLOCK, the caller's own included;
// and it signals immediately if the socket is already writable when it is
// registered.
//
// Two things it cannot do, and what covers each:
//
//   - FD_WRITE is edge-triggered where EPOLLOUT is level-triggered.  Once
//     consumed it does not fire again merely because the socket is still
//     writable, and re-registering does not re-record it (measured).  So
//     Poll() asks Writable() -- a select() -- before every wait, and that
//     answer alone decides whether kOut is dispatched.  FD_WRITE only
//     decides when a wait ends.
//   - FD_WRITE is only issued after a send on the socket has failed with
//     WSAEWOULDBLOCK.  A caller whose data runs out exactly as the buffer
//     fills never sees that error -- a non-blocking send() returns a short
//     count instead -- so the stack owes it no edge and will never signal,
//     however much room the peer then makes.  Confirmed on Windows: the peer
//     drained the socket and FD_WRITE stayed silent while select() called it
//     writable.
//
//     Asking on the way into the wait does not cover this, because the
//     transition happens during the wait.  Poll() checks Writable() at the
//     top of its loop, finds the socket full, and blocks with nothing left
//     that will ever wake it -- reaching that check again is the very thing
//     it is blocked on.  Nor can the edge be manufactured: that takes a
//     failed send, and the only thing we could send would land in the
//     caller's stream.  So the wait has to end on its own, which is what
//     capping it at kWriteWatchBackstop does.  Every send that does fail
//     still wakes the loop through FD_WRITE.
class WriteWatch {
 public:
  // Registers for FD_WRITE on s.  Two documented Winsock side effects: the
  // socket becomes non-blocking, and any WSAEventSelect() somebody else had
  // on it is replaced.  Both are part of why a socket may only belong to
  // one Aio (see IocpImpl::AssociateSocket()).  owner is the registration
  // the socket belongs to, handed back as the packet's context.
  WriteWatch(SOCKET s, void *owner) : socket_(s), owner_(owner) {
    event_ = WSACreateEvent();
    ABSL_CHECK(event_ != WSA_INVALID_EVENT)
        << ": WSACreateEvent failed: " << WSAGetLastError();
    const int res = WSAEventSelect(socket_, event_, FD_WRITE | FD_CLOSE);
    ABSL_CHECK(res == 0) << ": WSAEventSelect(FD_WRITE) failed on socket "
                         << socket_ << ": " << WSAGetLastError();
  }
  ~WriteWatch() {
    packet_.Disarm();
    // Deregister before closing, so the socket is not left pointing at a
    // dead event.  The socket stays non-blocking; WSAEventSelect() cannot
    // undo that, and callers of the legacy API are expected to cope, as
    // they must on epoll.
    WSAEventSelect(socket_, NULL, 0);
    WSACloseEvent(event_);
  }
  WriteWatch(const WriteWatch &) = delete;
  WriteWatch &operator=(const WriteWatch &) = delete;

  void Arm(HANDLE port) {
    if (!packet_.armed()) {
      packet_.Arm(port, event_, kWritableKey, owner_);
    }
  }

  // Poll() dequeued a delivered FD_WRITE.  Clears it, so the stack can
  // record the next one.  WSAEnumNetworkEvents() rather than ResetEvent():
  // it clears the socket's internal record of the network events as well as
  // resetting the WSAEVENT, and it is that record, not the handle, which
  // decides whether the stack will signal FD_WRITE again.  Resetting the
  // handle alone would leave FD_WRITE latched and the watch permanently
  // silent.
  //
  // The result is deliberately dropped, errors included: a failure recorded
  // against FD_WRITE is the socket's, not the watch's, and the read watch
  // reports it as kErr the way every other error on this fd is reported, so
  // there is one path for it rather than two.
  void Consume() {
    packet_.MarkDelivered();
    WSANETWORKEVENTS events;
    std::memset(&events, 0, sizeof(events));
    WSAEnumNetworkEvents(socket_, event_, &events);
  }

  bool Writable() const { return SocketWritable(socket_); }

 private:
  const SOCKET socket_;
  void *const owner_;
  WSAEVENT event_ = WSA_INVALID_EVENT;
  WaitPacket packet_;
};
}  // namespace

class IocpImpl : public Aio::Impl {
  // Reaches active_timer_count_ and the timer queue; the rest of this class
  // is nobody else's business.  Matches KqueueTimerState/KqueueImpl.
  friend struct WindowsTimerState;

 public:
  IocpImpl();
  ~IocpImpl() override;

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

 private:
  HANDLE iocp_handle_ = INVALID_HANDLE_VALUE;
  // Starts true so should_run() reports "running" before the first Run(), which
  // is how the original EPoll (run_{true}) behaved.  Run() clears it on exit
  // and Quit() clears it on shutdown.
  std::atomic<bool> run_{true};
  std::atomic<bool> quit_requested_{false};
  std::vector<std::function<void()>> before_wait_functions_;

  // Nonzero while inside Poll(), to enforce that it is not reentrant.  See
  // the CHECK at the top of Poll(), and EpollImpl's dispatch_depth_.
  int dispatch_depth_ = 0;
  // True while running the before-wait functions; see BeforeWait().
  bool in_before_wait_ = false;

  // The armed timers, ordered by (deadline, arming sequence).  An
  // IntrusiveRbTree rather than the sorted list this used to be: arming is
  // O(log n) instead of O(n), and Cancel() answers membership from a flag
  // instead of walking.  Same container and same ordering rule as the kqueue
  // backend's active_timers_ -- see timer_queue.h.
  TimerQueue<AsyncRequest, TimerQueueTraits> timers_;
  // Live WindowsTimerStates, counted so ~IocpImpl() can CHECK that none
  // outlive it -- ~WindowsTimerState dereferences this impl.  Not the same as
  // timers_.size(): a Timer that exists but is not currently scheduled is
  // counted here and is not in the queue.
  int active_timer_count_ = 0;

  void RemoveTimer(AsyncRequest *request);
  void InsertTimer(AsyncRequest *request);

  // Delivers the head of timers_ to the port; see DeadlineTimer.  Poll()
  // re-arms it only when the head deadline actually changes.  Auto-reset,
  // because Poll() sees the packet and OnDelivered() is what consumes it.
  DeadlineTimer deadline_timer_{kTimerKey};

  // Everything this loop knows about one registered fd: the caller's
  // outstanding raw requests, its legacy readiness handlers, and the
  // OVERLAPPEDs both are driven through.
  struct FdState {
    FileDescriptor fd = nullptr;
    bool associated_with_iocp = false;

    AsyncRequest *read_request = nullptr;
    AsyncRequest *write_request = nullptr;

    OVERLAPPED read_overlapped = {};
    OVERLAPPED write_overlapped = {};

    bool is_legacy = false;
    uint32_t legacy_events = 0;
    bool read_pending = false;

    // Whether fd is a socket, cached on first use, and whether we have asked
    // yet.  A non-socket handle takes the wait path instead of the zero-byte
    // WSARecv, and must never be handed to CreateIoCompletionPort().
    bool socket_known = false;
    bool is_socket = true;

    // The kernel-side watches a legacy subscription can hold, each present
    // exactly while it is wanted (see UpdateSocketState()).  A socket
    // subscribed to kOut holds a WriteWatch; a non-socket handle subscribed
    // to anything holds a HandleWatch.  The read watch is the zero-byte
    // WSARecv on read_overlapped above, tracked by read_pending.  Both are
    // owned here so that a state going back to the pool cannot leak one.
    std::optional<WriteWatch> write_watch;
    std::optional<HandleWatch> handle_watch;
    // Links for writable_watches_, the round-robin of states holding a
    // WriteWatch.  On that list exactly while write_watch is engaged.
    FdState *writable_prev = nullptr;
    FdState *writable_next = nullptr;

    bool has_events_fn = false;
    bool has_in_fn = false;
    bool has_out_fn = false;
    bool has_err_fn = false;

    std::function<void()> in_fn;
    std::function<void()> out_fn;
    std::function<void()> err_fn;
    std::function<void(uint32_t)> events_fn;
    // Link for free_list_ or retired_.  One field serves both because the
    // memberships are exclusive: a state is either waiting to be handed out
    // or waiting to be scrubbed, never both.
    FdState *next_free = nullptr;
    // Link for the list that owns every allocation for this loop's lifetime,
    // so the tree and the free/retired stacks can all be non-owning.
    FdState *next_all = nullptr;
    // Links for registrations_, keyed on fd.  Owned by that tree; nothing
    // else may touch them while the state is live.
    FdState *tree_left = nullptr;
    FdState *tree_right = nullptr;
    FdState *tree_parent = nullptr;
    bool tree_red = false;
  };
  struct FreeLinkTraits {
    static FdState *&next(FdState *state) { return state->next_free; }
  };
  struct AllLinkTraits {
    static FdState *&next(FdState *state) { return state->next_all; }
  };
  struct WritableLinkTraits {
    static FdState *&next(FdState *state) { return state->writable_next; }
    static FdState *&prev(FdState *state) { return state->writable_prev; }
  };
  // Orders the live registrations by fd.  FileDescriptor is an opaque handle
  // (SOCKET) on Windows, so compare the underlying integer values for a
  // well-defined total order.  Compare() lets GetActiveRegistration() look one
  // up from a bare fd without building a state to compare against.
  struct LiveTreeTraits {
    static FdState *&left(FdState *state) { return state->tree_left; }
    static FdState *&right(FdState *state) { return state->tree_right; }
    static FdState *&parent(FdState *state) { return state->tree_parent; }
    static bool &red(FdState *state) { return state->tree_red; }
    static uintptr_t Key(FileDescriptor fd) {
      return reinterpret_cast<uintptr_t>(fd);
    }
    static bool Less(FdState *a, FdState *b) { return Key(a->fd) < Key(b->fd); }
    static int Compare(const FdState *state, FileDescriptor fd) {
      if (Key(state->fd) < Key(fd)) return -1;
      if (Key(fd) < Key(state->fd)) return 1;
      return 0;
    }
  };
  // Owns every FdState this loop ever allocates.  registrations_, free_list_
  // and retired_ below are all non-owning views onto it, so a state moves
  // between them by pointer assignment -- which is what lets AsyncRead() and
  // AsyncWrite() register an fd without allocating, as the no-malloc
  // guarantee documented on Aio (see aio.h) and required by ScopedRealtime.
  //
  // Declared before them deliberately: members are destroyed in reverse, so
  // the owner outlives every view onto it.  Same shape and same order as the
  // readiness backends' FdRegistrationTable.
  OwningIntrusiveStack<FdState, AllLinkTraits> all_;
  // Keyed on fd, like the readiness backends' live_ tree.  A sorted vector
  // would reallocate on insert, and inserting happens on the arming path --
  // which must not allocate -- while legacy registrations are unbounded, so
  // no reservation could be relied on.  O(log n) lookup, no allocation.
  IntrusiveRbTree<FdState, LiveTreeTraits> registrations_;
  // Intrusive, like the epoll and kqueue backends': drawing from the pool and
  // returning to it are then pointer assignment, which is what the no-malloc
  // guarantee above actually needs.
  IntrusiveStack<FdState, FreeLinkTraits> free_list_;
  // Released but not yet scrubbed; see ReleaseRegistration().  Intrusive
  // like free_list_ and sharing its link, so parking a state is pointer
  // assignment rather than a vector push that can reallocate on an RT path.
  // Searching it (ClearRetiredPending()), walking it at teardown, and scrubbing
  // it selectively are ForEach() and MoveMatchingTo().
  IntrusiveStack<FdState, FreeLinkTraits> retired_;
  static constexpr size_t kInitialPoolSize = 16;

  FdState *GetActiveRegistration(FileDescriptor fd) const;
  // The same, but only if fd carries a *legacy* registration.  The legacy
  // hooks (DeleteFd, SetEvents, Enable/DisableWritable) address that
  // namespace: an fd carrying only raw requests is "not found" to them, on
  // every backend.
  FdState *GetActiveLegacyRegistration(FileDescriptor fd) const;

  // Used by the legacy readiness-based API (OnReadable/OnWritable/OnError/
  // OnEvents).  Not guaranteed to avoid allocation; these are not meant to be
  // called from realtime code.
  FdState *GetOrCreateLegacyRegistration(FileDescriptor fd);

  // Used by the completion-based API (AsyncRead/AsyncWrite),
  // which must not allocate.  Draws from free_list_ instead of the heap.
  FdState *GetOrCreateAsyncRegistration(FileDescriptor fd);

  void ReleaseRegistration(FdState *state);

  // Hands back the pool slot of a registration AsyncRead()/AsyncWrite()
  // created, once it has nothing outstanding.  "Maybe" because it is a no-op
  // for a legacy registration -- those belong to the caller until DeleteFd()
  // -- and for one that still has a request or a watch in flight.  Called at
  // each completion, so the pool tracks simultaneously-outstanding requests
  // rather than distinct fds ever used.  Safe from inside a dispatch because
  // ReleaseRegistration() only parks the state; nothing is recycled until
  // ScrubRetiredRegistrations() runs with no callback on the stack.
  void MaybeRetireAsyncRegistration(FdState *state);

  // Finishes what ReleaseRegistration() deferred: destroys the retired
  // states' callbacks and returns their slots to free_list_.  Only legal
  // with dispatch_depth_ == 0.
  void ScrubRetiredRegistrations();

  // CancelIoEx()es whichever readiness watches this state has armed.  The
  // cancelled operations still post their (aborted) completions, and those
  // are what return pending_io_count_ to zero.
  void CancelPendingWatches(FdState *state);

  // Releases the hold a retired state was keeping for `overlapped`, and
  // reports whether one matched.  The kernel writes into an OVERLAPPED until
  // its completion has been dequeued, so a state with one still out may not
  // be recycled before then.  Each outstanding operation holds the state
  // separately: this clears the one whose completion just landed, and
  // ScrubRetiredRegistrations() waits for the last.
  bool ClearRetiredPending(OVERLAPPED *overlapped);

  // Raw requests whose WSARecv()/WSASend() failed outright, waiting for the
  // Poll() that will complete them -- aio.h delivers every completion from
  // there, never from the submit call.  Intrusive, threaded through the
  // requests themselves (see WindowsAsyncState::failed_next), so parking one
  // allocates nothing.  Each entry already carries the errno it will be
  // completed with.
  IntrusiveFifo<AsyncRequest, FailedRequestTraits> failed_requests_;
  // Parks request on failed_requests_ with `winsock_error` translated.
  void QueueFailedRequest(AsyncRequest *request, int winsock_error);
  std::atomic<int> pending_io_count_{0};

  // Joins state's socket to this loop's completion port so that every
  // overlapped operation started on it reports back through
  // GetQueuedCompletionStatus() with the fd as the completion key.  That is
  // all "associate" means to Windows, and it is a property of the socket
  // rather than of any one operation: it happens once, and it cannot be
  // undone for the life of the socket.
  //
  // A no-op for a handle that is not a socket.  Only a kernel file object can
  // join a port; a bare waitable handle is watched through a HandleWatch
  // instead (see UpdateSocketState()).
  void AssociateSocket(FdState *state);
  // Arms or disarms the kernel-side watches state's legacy subscription now
  // calls for, which is this backend's UpdateRegistration().  Idempotent, and
  // driven by change rather than by a sweep: every mutator runs it through
  // UpdateSocket(), and the two dispatch paths run it again on the way out,
  // because delivering a readiness watch is what disarms it.
  void UpdateSocketState(FdState *state);

  // Gives state a WriteWatch if it has none and arms it; idempotent.  The
  // release is what DeleteFd() and a subscription dropping kOut call.
  void ArmWriteWatch(FdState *state);
  void ReleaseWriteWatch(FdState *state);
  // Dispatches kOut to one write watch whose socket is writable right now,
  // like every other source here.  Returns whether it dispatched.
  bool DispatchWritableWatches();
  // Every state holding a WriteWatch, in the order DispatchWritableWatches()
  // will consider them: it rotates each one it looks at to the back, so
  // every watch gets its turn.  Writability is a level -- a socket nobody
  // fills stays writable indefinitely -- so a scan that restarted from the
  // same place every Poll() would dispatch that one watch forever.  The
  // other one-per-Poll() sources are fair for free because they dequeue the
  // port's FIFO; this one is a scan, so it carries its own order.
  IntrusiveDoublyLinkedList<FdState, WritableLinkTraits> writable_watches_;
  // AssociateSocket() and UpdateSocketState(), for a legacy fd named rather
  // than held.
  void UpdateSocket(FileDescriptor fd);
  void Wakeup();

  // The two halves of Poll()'s completion dispatch, split out because the
  // function was doing three unrelated jobs at once.  Both run with
  // dispatch_depth_ already raised, and Poll() returns true after either: the
  // caller has decided a user-visible completion is happening; these decide
  // what it is.
  // Resolves one raw AsyncRead()/AsyncWrite() from its completion.  `slot` is
  // the state's read_request or write_request, cleared before the callback so
  // the callback may re-arm it.
  void CompleteRawRequest(AsyncRequest **slot, FileDescriptor fd,
                          LPOVERLAPPED lp_overlapped, DWORD bytes_transferred,
                          BOOL success);
  // EPoll's dispatch rules, shared by the readiness watch, the write watch
  // and a signalled non-socket handle.  `terminal` is a hangup: OnEvents sees
  // it as the error bit, the in/out/err trio does not.
  void DispatchLegacyEvents(FdState *state, FileDescriptor fd, uint32_t events,
                            bool terminal);
  // Dispatches a signalled non-socket handle to its legacy handlers, then
  // re-arms the watch.  See UpdateSocketState() for why the reported mask is
  // the subscribed one.
  void DispatchHandleSignal(FdState *state);
  void DispatchSocketCompletion(ULONG_PTR completion_key,
                                LPOVERLAPPED lp_overlapped,
                                DWORD bytes_transferred, BOOL success);

  // Dies if request is already armed on this loop.  See the definition.
  void CheckNotAlreadyInFlight(AsyncRequest *request) const;

  // The one registered ThreadSignalReceiver, and the Event handle it is
  // registered under -- as a legacy OnReadable() on that handle, the way it
  // is an OnReadable() on the signalfd on Linux -- so unregistering can name
  // the same registration again.  One at a time, as
  // Aio::RegisterThreadSignalReceiver() documents and as the other three
  // backends also enforce.
  ipc_lib::ThreadSignalReceiver *signal_receiver_ = nullptr;
  HANDLE signal_receiver_handle_ = NULL;
};

IocpImpl::IocpImpl() {
  EnsureWinsockInitialized();
  EnsureHighResolutionTimers();

  iocp_handle_ = CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 0);
  ABSL_CHECK(iocp_handle_ != NULL) << "Failed to create IOCP";

  // The pool the arming path draws from; see all_.  Sized to make running
  // dry not happen, since the fallback allocation is what the realtime malloc
  // hook exists to catch.  failed_requests_ needs no equivalent reserve --
  // it, registrations_ and retired_ are all intrusive, so adding to any of
  // them is pointer assignment rather than a push that can reallocate.
  for (size_t i = 0; i < kInitialPoolSize; ++i) {
    auto *state = new FdState();
    all_.Push(state);
    free_list_.Push(state);
  }
}

void IocpImpl::RemoveTimer(AsyncRequest *request) {
  WindowsAsyncState *state = State(request);
  if (!state->queued) {
    return;
  }
  timers_.Remove(request);
  state->queued = false;
}

void IocpImpl::InsertTimer(AsyncRequest *request) {
  // Re-arming is legal, so drop any previous position first.  Insert() stamps
  // the sequence, which is what keeps equal deadlines in arming order.
  RemoveTimer(request);
  timers_.Insert(request);
  State(request)->queued = true;
}

IocpImpl::FdState *IocpImpl::GetActiveRegistration(FileDescriptor fd) const {
  return registrations_.Find(fd);
}

IocpImpl::FdState *IocpImpl::GetActiveLegacyRegistration(
    FileDescriptor fd) const {
  FdState *state = GetActiveRegistration(fd);
  if (state == nullptr || !state->is_legacy) {
    return nullptr;
  }
  return state;
}

IocpImpl::FdState *IocpImpl::GetOrCreateLegacyRegistration(FileDescriptor fd) {
  FdState *state = GetActiveRegistration(fd);
  if (state != nullptr) {
    return state;
  }
  // Not from the pool: legacy registrations live until DeleteFd(), so they
  // would hold a slot the arming path needs.  Registering a handler is a
  // startup operation and may allocate; see the comment on registrations_.
  state = new FdState();
  all_.Push(state);
  state->fd = fd;
  registrations_.Insert(state);
  return state;
}

IocpImpl::FdState *IocpImpl::GetOrCreateAsyncRegistration(FileDescriptor fd) {
  FdState *state = GetActiveRegistration(fd);
  if (state != nullptr) {
    return state;
  }
  state = free_list_.Pop();
  if (state == nullptr) {
    // Running dry is not fatal -- the pool is sized to make it not happen,
    // and this allocation is what the realtime malloc hook is there to catch
    // (realtime_windows.cc detours ucrtbase's malloc for exactly this).
    // Aborting instead would turn a sizing miss into a dead process, which
    // is what FdRegistrationTable::Allocate() deliberately avoids on the
    // other backends.
    state = new FdState();
    all_.Push(state);
  }
  state->fd = fd;
  registrations_.Insert(state);
  return state;
}

void IocpImpl::ReleaseRegistration(FdState *state) {
  ABSL_CHECK_EQ(registrations_.Find(state->fd), state);
  // A watch is a packet the kernel may still queue, and it is on lists the
  // scrub does not walk; whoever releases a registration takes its watches
  // down first (DeleteFd() does, and an async-only state never has any).
  ABSL_CHECK(!state->write_watch.has_value() &&
             !state->handle_watch.has_value())
      << ": fd " << state->fd << " released with a watch still armed";
  registrations_.Remove(state);

  // Erasing above is enough to make the registration invisible to lookups.
  // The reset -- which destroys in_fn/out_fn/err_fn/events_fn -- has to wait
  // until no dispatch is in flight: a callback is allowed to delete its own
  // fd, and that callback is one of these std::functions, so destroying it
  // here would free the running lambda's captures out from under it.  So
  // the state is parked on retired_, and ScrubRetiredRegistrations() finishes
  // the job with no callback on the stack: right now when this was called
  // from outside a dispatch (a DeleteFd() between Poll()s, with no Poll()
  // coming to do it), and otherwise at the top of the next Poll().  Matches
  // EpollImpl::ReleaseRegistration().
  retired_.Push(state);
  if (dispatch_depth_ == 0) {
    ScrubRetiredRegistrations();
  }
}

void IocpImpl::MaybeRetireAsyncRegistration(FdState *state) {
  // Legacy registrations belong to the caller until DeleteFd(); only the ones
  // the Async* API created on demand are ours to reclaim.
  if (state->is_legacy || state->has_events_fn || state->has_in_fn ||
      state->has_out_fn || state->has_err_fn) {
    return;
  }
  // Still owed something.  A pending flag means the kernel holds the
  // OVERLAPPED inside this state, cancelled operations included.
  if (state->read_request != nullptr || state->write_request != nullptr ||
      state->read_pending) {
    return;
  }
  // Note the socket stays associated with the completion port -- that is not
  // undoable -- so a later AsyncRead() on this same fd builds a fresh
  // registration whose associated_with_iocp starts false.  AssociateSocket()
  // handles that: re-associating an already-associated socket fails with
  // ERROR_INVALID_PARAMETER, which it treats as success.
  ReleaseRegistration(state);
}

void IocpImpl::CancelPendingWatches(FdState *state) {
  if (state->read_pending) {
    CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state->fd)),
               &state->read_overlapped);
  }
  // No write half: the legacy write watch is a WSAEventSelect() registration
  // rather than an overlapped operation, and ReleaseWriteWatch() tears it
  // down.  Raw AsyncWrite()s are cancelled by Cancel() and by ~IocpImpl().
}

bool IocpImpl::ClearRetiredPending(OVERLAPPED *overlapped) {
  bool cleared = false;
  retired_.ForEach([overlapped, &cleared](FdState *state) {
    if (cleared) {
      return;
    }
    if (overlapped == &state->read_overlapped) {
      state->read_pending = false;
      state->read_request = nullptr;
      cleared = true;
    } else if (overlapped == &state->write_overlapped) {
      state->write_request = nullptr;
      cleared = true;
    }
  });
  return cleared;
}

void IocpImpl::ScrubRetiredRegistrations() {
  ABSL_CHECK_EQ(dispatch_depth_, 0);
  // Split the finished states off in one pass.  A state still owed a
  // completion stays parked: the kernel holds a pointer into it, so it may
  // not be recycled until that lands (ClearRetiredPending() clears the flag,
  // ~IocpImpl() forces the issue by cancelling).
  IntrusiveStack<FdState, FreeLinkTraits> finished;
  retired_.MoveMatchingTo(&finished, [](FdState *state) {
    return !(state->read_pending || state->read_request ||
             state->write_request);
  });
  while (FdState *state = finished.Pop()) {
    // Back to pool state, by destroying it and constructing a fresh one in
    // the same storage -- rather than listing the fields, so a field added
    // later cannot be forgotten here, and rather than assigning, since the
    // watches it may hold are not movable.  next_all has to survive it: the
    // state is still on all_, which is what owns it, and a fresh FdState
    // would null that link and drop it out of the list -- a leak the
    // destructor could not find.
    FdState *const saved_next_all = state->next_all;
    state->~FdState();
    new (state) FdState();
    state->next_all = saved_next_all;
    // Every one of them, with no cap: all_ owns the allocation either way, so
    // holding a surplus slot on free_list_ costs nothing and dropping it
    // would only make the next AsyncRead() allocate a replacement.  Same rule
    // as FdRegistrationTable::ScrubRetired().
    free_list_.Push(state);
  }
}

void IocpImpl::AssociateSocket(FdState *state) {
  if (!state->associated_with_iocp) {
    SOCKET s = ToSocket(state->fd);
    if (s == INVALID_SOCKET) {
      return;
    }
    if (!state->socket_known) {
      state->socket_known = true;
      state->is_socket = IsSocket(state->fd);
    }
    if (!state->is_socket) {
      // Only a socket can join a completion port.  Handing anything else to
      // CreateIoCompletionPort() fails with ERROR_INVALID_HANDLE, and since
      // associated_with_iocp stays false it used to retry -- and log -- on
      // every single Poll().  Non-sockets take the wait path instead; see
      // UpdateSocketState().
      return;
    }
    HANDLE socket_handle = reinterpret_cast<HANDLE>(s);
    HANDLE res = CreateIoCompletionPort(
        socket_handle, iocp_handle_, reinterpret_cast<ULONG_PTR>(state->fd), 0);
    const DWORD err = (res == iocp_handle_) ? 0 : GetLastError();
    // ERROR_INVALID_PARAMETER is "already associated with a completion
    // port".  Normally that port is ours -- this is a re-registration of an
    // fd we retired earlier, and re-associating is both harmless and
    // unavoidable, since the state carrying associated_with_iocp went back
    // to the pool.
    //
    // It can also mean the socket belongs to a *different* Aio's port, which
    // is unrecoverable: a socket's completion-port association is permanent
    // for the life of the socket, so this Aio will never see a completion
    // for it and any Poll() waiting on one blocks forever.  The two cases
    // are indistinguishable here -- Windows offers no way to ask a socket
    // which port it is on -- so this stays permissive.  Callers must not
    // hand the same socket to two Aio instances.
    //
    // Anything else means the socket cannot join the port at all, and so no
    // operation on it will ever complete: the request just submitted would
    // wait forever.  Fatal, as failing to arm the read watch is.
    ABSL_CHECK(err == 0 || err == ERROR_INVALID_PARAMETER)
        << ": failed to associate socket fd=" << state->fd
        << " with the completion port: error " << err;
    state->associated_with_iocp = true;
  }
}

void IocpImpl::UpdateSocketState(FdState *state) {
  if (state->is_legacy) {
    if (!state->socket_known) {
      state->socket_known = true;
      state->is_socket = IsSocket(state->fd);
    }
    if (!state->is_socket) {
      // A waitable object rather than a socket: there is no readiness to
      // sample, only "has it been signalled".  That is exactly what glib
      // means by a GPollFD on Windows -- g_poll() waits on the handle and
      // sets revents = events, with no discrimination between the bits --
      // so reporting the subscribed mask on signal is this platform's own
      // semantics, not an approximation of epoll's.
      if (state->legacy_events != 0) {
        if (!state->handle_watch.has_value()) {
          // In place: this runs from the re-arm after a dispatch too, under
          // whatever realtime state the caller was in, and a kernel object
          // is not a malloc.
          state->handle_watch.emplace(reinterpret_cast<HANDLE>(state->fd),
                                      state);
        }
        state->handle_watch->Arm(iocp_handle_);
      } else {
        state->handle_watch.reset();
      }
      return;
    }
    // kErr rides on the read watch.  An error or a hangup surfaces there --
    // the zero-byte WSARecv completes with the error, or completes cleanly
    // and the peek that follows says EOF (see DispatchSocketCompletion()) --
    // while the write watch is FD_WRITE, which says nothing about either.
    // So an OnError()-only registration arms the read watch and no write
    // watch.
    bool want_read = (state->legacy_events & (kIn | kPri | kErr)) != 0;
    bool want_write = (state->legacy_events & kOut) != 0;

    if (want_read) {
      if (!state->read_request && !state->read_pending) {
        state->read_pending = true;
        WSABUF buf = {0, nullptr};
        DWORD flags = 0;
        std::memset(&state->read_overlapped, 0, sizeof(state->read_overlapped));
        const int res = WSARecv(ToSocket(state->fd), &buf, 1, nullptr, &flags,
                                &state->read_overlapped, nullptr);
        const int err = (res == 0) ? 0 : WSAGetLastError();
        if (res != 0 && err != WSA_IO_PENDING && IsSocketConditionError(err)) {
          // The socket itself is what is wrong -- the peer reset it, say --
          // and that is a readiness event, not a failure to register: epoll
          // reports it as EPOLLERR on every wait until the fd is removed.
          // Nothing is armed, so deliver it the way an armed watch would
          // have: post the completion the WSARecv would have produced, and
          // the peek in DispatchSocketCompletion() finds the error and
          // reports kErr.  Re-arming after that dispatch lands here again,
          // which is the level.  Accounted for like any completion, so the
          // destructor's drain sees it too.
          ABSL_PCHECK(PostQueuedCompletionStatus(
              iocp_handle_, 0, reinterpret_cast<ULONG_PTR>(state->fd),
              &state->read_overlapped))
              << "PostQueuedCompletionStatus failed";
        } else {
          // Nothing was queued, so no completion is owed and no change will
          // come along to re-arm this: the fd would simply stop waking, which
          // the caller sees as a loop that goes quiet.  Die rather than
          // degrade, as EpollImpl::UpdateRegistration()'s PCHECK and
          // KqueueImpl's ABSL_PLOG(FATAL) do for the same failure.
          ABSL_CHECK(res == 0 || err == WSA_IO_PENDING)
              << ": failed to arm the read watch on fd " << state->fd
              << ": WSA error " << err;
        }
        pending_io_count_++;
      }
    } else {
      if (state->read_pending) {
        CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state->fd)),
                   &state->read_overlapped);
      }
    }

    // Not an overlapped operation, so not part of pending_io_count_ either;
    // see WriteWatch.
    if (want_write) {
      ArmWriteWatch(state);
    } else {
      ReleaseWriteWatch(state);
    }
  }
}

void IocpImpl::ArmWriteWatch(FdState *state) {
  if (!state->write_watch.has_value()) {
    // In place, for the same reason as the HandleWatch in
    // UpdateSocketState(): EnableWritable() and the re-arm after a dispatch
    // both reach here, and neither may malloc.
    state->write_watch.emplace(ToSocket(state->fd), state);
    writable_watches_.PushBack(state);
  }
  state->write_watch->Arm(iocp_handle_);
}

void IocpImpl::ReleaseWriteWatch(FdState *state) {
  if (!state->write_watch.has_value()) {
    return;
  }
  writable_watches_.Remove(state);
  state->write_watch.reset();
}

bool IocpImpl::DispatchWritableWatches() {
  // Each watch looked at moves to the back, dispatched or not, so the next
  // scan starts with the one after it; see writable_watches_.
  for (size_t i = 0; i < writable_watches_.size(); ++i) {
    FdState *const state = writable_watches_.PopFront();
    writable_watches_.PushBack(state);
    if (!state->write_watch->Writable()) {
      continue;
    }
    // kOut only, which is all a state on this list is subscribed to that
    // this watch can answer for: an error on the fd comes back through the
    // read watch.
    DispatchLegacyEvents(state, state->fd, kOut, /*terminal=*/false);
    return true;
  }
  return false;
}

void IocpImpl::UpdateSocket(FileDescriptor fd) {
  FdState *state = GetOrCreateLegacyRegistration(fd);
  AssociateSocket(state);
  UpdateSocketState(state);
}

void IocpImpl::Wakeup() {
  PostQueuedCompletionStatus(iocp_handle_, 0, kWakeupKey, nullptr);
}

IocpImpl::~IocpImpl() {
  // Owner-facing state must be gone first, as EPoll::~EPoll() CHECKed and
  // aio.h documents.  A live Aio::Timer's destructor dereferences this impl,
  // so one outliving its Aio is a use-after-free -- and this destructor goes
  // on to cancel timers and free wait contexts, so it is not a quiet one.
  // Same set of checks as ~EpollImpl() and ~KqueueImpl().
  ABSL_CHECK_EQ(active_timer_count_, 0)
      << ": An Aio::Timer must be destroyed before its Aio";
  ABSL_CHECK(signal_receiver_ == nullptr)
      << ": The ThreadSignalReceiver must be unregistered before destroying "
         "the Aio";
  {
    // Only async-only registrations may remain: aio.h's constraint 2 permits
    // destroying the Aio with raw requests still pending.  Caller fd
    // registrations must be gone, as EPoll always CHECKed.
    for (FdState *state : registrations_) {
      // Windows spells async-only as "not is_legacy, and no legacy handler
      // attached" -- the same predicate MaybeRetireAsyncRegistration() uses.
      const bool is_caller_registration =
          state->is_legacy || state->has_events_fn || state->has_in_fn ||
          state->has_out_fn || state->has_err_fn;
      ABSL_CHECK(!is_caller_registration)
          << ": fd " << state->fd
          << " must be removed (DeleteFd()/ForgetClosedFd()) before "
             "destroying the Aio";
    }
  }

  // Matches IoUringImpl/EpollImpl/KqueueImpl.  Nothing can observe it after
  // this point, but leaving one backend's teardown different from the other
  // three is a difference someone has to re-derive as harmless later.
  run_ = false;

  // Nothing can be left: active_timer_count_ was CHECKed zero above, every
  // ~WindowsTimerState calls RemoveTimer() on the way out, and nothing else
  // inserts.  Disarmed here, while the port is still open, rather than left
  // to the member's destructor.
  ABSL_CHECK(timers_.empty());
  deadline_timer_.Disarm();

  // Likewise nothing can hold a write watch or a handle watch: only a legacy
  // registration ever has one, DeleteFd() releases them, and the loop above
  // CHECKed that every legacy registration is gone.  Asserted rather than
  // swept, so a path that grows one later says so here instead of leaking
  // an event and a packet.
  ABSL_CHECK(writable_watches_.empty());

  // Cancel the raw requests and readiness watches still out on the sockets.
  for (FdState *state : registrations_) {
    // Directly rather than through Cancel(), which would re-scan the tree
    // for a registration already in hand.
    if (state->read_request != nullptr) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state->fd)),
                 &state->read_overlapped);
    }
    if (state->write_request != nullptr) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state->fd)),
                 &state->write_overlapped);
    }
    // The Cancel()s above only cover the completion-based requests.  A
    // legacy fd's readiness watch is an overlapped operation too, and it
    // counts toward pending_io_count_ just the same, so the drain below hangs
    // forever unless it is cancelled as well.
    CancelPendingWatches(state);
  }
  // Same for anything retired but not yet recycled.
  retired_.ForEach([this](FdState *state) { CancelPendingWatches(state); });

  // Wait for all pending overlapped operations to complete.  Everything that
  // incremented pending_io_count_ has been cancelled by this point -- the
  // requests above, and the readiness watches via CancelPendingWatches() --
  // so each one still outstanding owes exactly one (aborted) completion.
  while (pending_io_count_ > 0) {
    DWORD bytes = 0;
    ULONG_PTR key = 0;
    LPOVERLAPPED overlapped = nullptr;
    GetQueuedCompletionStatus(iocp_handle_, &bytes, &key, &overlapped, 10);
    // A wakeup or handle packet carries a non-null lpOverlapped too, but no
    // pending_io_count_ reference.  None should remain after the disarm
    // above; skipping by key keeps a late one from being miscounted as I/O.
    if (overlapped && !IsInternalKey(key)) {
      pending_io_count_--;
    }
  }

  if (iocp_handle_ != INVALID_HANDLE_VALUE) {
    CloseHandle(iocp_handle_);
    iocp_handle_ = INVALID_HANDLE_VALUE;
  }
  // deadline_timer_ closes its timer and packet after this, as a member: a
  // packet outlives the port it was associated with (probed), and closing in
  // this order keeps the drain above the last thing that could dequeue from
  // it.
  //
  // No WSACleanup() to match the startup in Aio::Aio(): see
  // EnsureWinsockInitialized().
}

void IocpImpl::Run() {
  if (quit_requested_) {
    quit_requested_ = false;
    return;
  }
  run_ = true;
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
  run_ = false;
  quit_requested_ = false;
}

bool IocpImpl::should_run() const { return run_ && !quit_requested_; }

void IocpImpl::Quit() {
  // Already asked to stop.  Bail out rather than re-arming the wakeup: once
  // Run() is draining it polls without blocking, so a Quit() called from a
  // BeforeWait callback (or any other per-Poll path) would refill the
  // completion queue every time around and the drain would never finish.  Same
  // guard as the other backends.
  if (quit_requested_) {
    return;
  }

  quit_requested_ = true;
  run_ = false;
  Wakeup();
}

// A request may only be armed once at a time.  aio.h's constraint 2 lets a
// request outlive the Aio it was armed on, so `done` alone cannot answer this
// -- a request left pending by a destroyed Aio also carries done == false.
// Ask this loop instead, exactly as the Linux and macOS backends do: the
// registrations are what this loop has armed.
//
// The per-fd "Duplicate AsyncRead on fd" check catches re-arming on the *same*
// fd.  This is the cross-fd case, which used to be silent and left two
// FdStates pointing at one request: whichever completed first ran the
// callback, and the other kept a pointer to a request the caller was then free
// to reuse -- and here the kernel holds that pointer too, through the
// OVERLAPPED.
//
// `done` is the cheap filter, so a fresh or completed request never walks
// anything.
void IocpImpl::CheckNotAlreadyInFlight(AsyncRequest *request) const {
  if (request->done) {
    return;
  }
  for (FdState *state : registrations_) {
    ABSL_CHECK(state->read_request != request &&
               state->write_request != request)
        << ": AsyncRead()/AsyncWrite() on a request that is still in flight; "
           "wait for its completion or Cancel() it first";
  }
  // A request whose submit failed is in flight too, as far as the caller is
  // concerned: its callback has not run yet.  Checked because
  // failed_requests_ is threaded through the requests themselves -- pushing
  // one on twice links it to itself, and the list never recovers -- where the
  // {request, error} vector this replaced would merely have completed it
  // twice.  Nothing else can reach here with a request on the list: an
  // AsyncRequest starts done, AsyncRead()/AsyncWrite() clear that only after
  // this check, and the drain in Poll() sets it again before the callback.
  failed_requests_.ForEach([request](const AsyncRequest *queued) {
    ABSL_CHECK(queued != request)
        << ": AsyncRead()/AsyncWrite() on a request that is still in flight; "
           "wait for its completion or Cancel() it first";
  });
}

void IocpImpl::QueueFailedRequest(AsyncRequest *request, int winsock_error) {
  // Translated here rather than at dispatch, so what is queued is the result
  // the request will be completed with and Poll() has nothing left to work
  // out.  No Wakeup(): the list is drained at the top of Poll(), before it
  // waits on anything, and a submit only ever happens between Poll()s on
  // the loop's own thread.
  State(request)->failed_errno = TranslateWinsockError(winsock_error);
  failed_requests_.PushBack(request);
}

void IocpImpl::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                         AsyncRequest *request) {
  CheckNotAlreadyInFlight(request);
  FdState *state = GetOrCreateAsyncRegistration(fd);
  ABSL_CHECK(!state->has_events_fn)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!state->has_in_fn)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  // Exclusive in both directions, as IoUringImpl::ClaimRawFd() is: the
  // opposite-direction handler collides too, and so does an OnError-only
  // registration.  The narrower checks above run first only because naming
  // the specific collision is friendlier than the general rule.
  ABSL_CHECK(!state->has_in_fn && !state->has_out_fn && !state->has_err_fn)
      << "Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(state->read_request == nullptr)
      << "Duplicate AsyncRead on fd " << fd;

  AssociateSocket(state);

  request->done = false;
  state->read_request = request;

  std::memset(&state->read_overlapped, 0, sizeof(state->read_overlapped));

  WSABUF wsa_buf;
  wsa_buf.buf = buffer.data();
  wsa_buf.len = static_cast<ULONG>(buffer.size());

  DWORD flags = 0;
  int res = WSARecv(ToSocket(fd), &wsa_buf, 1, nullptr, &flags,
                    &state->read_overlapped, nullptr);
  if (res == 0 || WSAGetLastError() == WSA_IO_PENDING) {
    pending_io_count_++;
  } else {
    state->read_request = nullptr;
    QueueFailedRequest(request, WSAGetLastError());
    // Nothing reached the kernel, so the completion that would normally
    // retire this registration is not coming.
    MaybeRetireAsyncRegistration(state);
  }
}

void IocpImpl::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                          AsyncRequest *request) {
  CheckNotAlreadyInFlight(request);
  FdState *state = GetOrCreateAsyncRegistration(fd);
  ABSL_CHECK(!state->has_events_fn)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!state->has_out_fn)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  // Exclusive in both directions -- see AsyncRead().
  ABSL_CHECK(!state->has_in_fn && !state->has_out_fn && !state->has_err_fn)
      << "Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(state->write_request == nullptr)
      << "Duplicate AsyncWrite on fd " << fd;

  AssociateSocket(state);

  request->done = false;
  state->write_request = request;

  std::memset(&state->write_overlapped, 0, sizeof(state->write_overlapped));

  WSABUF wsa_buf;
  wsa_buf.buf = const_cast<char *>(buffer.data());
  wsa_buf.len = static_cast<ULONG>(buffer.size());

  int res = WSASend(ToSocket(fd), &wsa_buf, 1, nullptr, 0,
                    &state->write_overlapped, nullptr);
  if (res == 0 || WSAGetLastError() == WSA_IO_PENDING) {
    pending_io_count_++;
  } else {
    state->write_request = nullptr;
    QueueFailedRequest(request, WSAGetLastError());
    // See AsyncRead().
    MaybeRetireAsyncRegistration(state);
  }
}

void IocpImpl::Cancel(AsyncRequest *request) {
  // Only raw I/O reaches here.  A timer is cancelled through
  // WindowsTimerState::Cancel(), which calls RemoveTimer() directly, and the
  // request an Aio::Timer embeds is private -- so nothing a caller can pass
  // to Aio::Cancel() is ever in timers_.
  for (FdState *state : registrations_) {
    if (state->read_request == request) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state->fd)),
                 &state->read_overlapped);
      return;
    }
    if (state->write_request == request) {
      CancelIoEx(reinterpret_cast<HANDLE>(ToSocket(state->fd)),
                 &state->write_overlapped);
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
  before_wait_functions_.push_back(std::move(function));
}

void IocpImpl::OnReadable(FileDescriptor fd, std::function<void()> callback) {
  FdState *state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!state->has_events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  ABSL_CHECK(state->read_request == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  // The opposite direction collides too, as it does in the shared readiness
  // backend -- exclusivity runs both ways, and checking only the matching
  // direction let AsyncWrite()-then-OnReadable() through here.
  ABSL_CHECK(state->write_request == nullptr)
      << "Cannot mix AsyncWrite and OnReadable on fd " << fd;
  ABSL_CHECK(!state->has_in_fn) << "Duplicate in functions for " << fd;
  state->is_legacy = true;
  state->has_in_fn = true;
  state->legacy_events |= kIn;
  state->in_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::OnError(FileDescriptor fd, std::function<void()> callback) {
  FdState *state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!state->has_events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  // OnError has no direction of its own, so any raw request on the fd
  // collides.  This checked nothing at all before, which made it the widest
  // of the three holes.
  ABSL_CHECK(state->read_request == nullptr && state->write_request == nullptr)
      << "Cannot mix AsyncRead/AsyncWrite and OnError on fd " << fd;
  ABSL_CHECK(!state->has_err_fn) << "Duplicate error functions for " << fd;
  state->is_legacy = true;
  state->has_err_fn = true;
  state->legacy_events |= kErr;
  state->err_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::OnWritable(FileDescriptor fd, std::function<void()> callback) {
  FdState *state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!state->has_events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  ABSL_CHECK(state->write_request == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  // See OnReadable(): both directions collide.
  ABSL_CHECK(state->read_request == nullptr)
      << "Cannot mix AsyncRead and OnWritable on fd " << fd;
  ABSL_CHECK(!state->has_out_fn) << "Duplicate out functions for " << fd;
  state->is_legacy = true;
  state->has_out_fn = true;
  state->legacy_events |= kOut;
  state->out_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::OnEvents(FileDescriptor fd,
                        std::function<void(uint32_t)> callback) {
  FdState *state = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(state->read_request == nullptr && state->write_request == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!state->has_in_fn && !state->has_out_fn && !state->has_err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!state->has_events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  state->is_legacy = true;
  state->has_events_fn = true;
  state->events_fn = std::move(callback);
  UpdateSocket(fd);
}

void IocpImpl::DeleteFd(FileDescriptor fd) {
  FdState *state = GetActiveLegacyRegistration(fd);
  ABSL_CHECK(state != nullptr) << "fd " << fd << " not found";
  state->is_legacy = false;
  state->legacy_events = 0;
  state->has_events_fn = false;
  state->has_in_fn = false;
  state->has_out_fn = false;
  state->has_err_fn = false;
  // The std::functions themselves are deliberately left alone here: a
  // callback deleting its own fd is legal, and it is one of them, so
  // clearing it would destroy the running lambda's captures mid-call.  The
  // has_*_fn flags above already stop anything from dispatching to it, and
  // ReleaseRegistration() -> ScrubRetiredRegistrations() destroys them once
  // no dispatch is in flight.

  // Take the watches down explicitly rather than letting UpdateSocket()
  // notice that legacy_events is now empty: is_legacy was just cleared, and
  // UpdateSocketState() returns immediately for a non-legacy state, so its
  // release branches are unreachable from here.  A read watch left armed
  // keeps its pending_io_count_ reference forever, which wedges
  // ~IocpImpl()'s drain; the other two are objects ReleaseRegistration()
  // CHECKs are gone.
  state->handle_watch.reset();
  ReleaseWriteWatch(state);
  CancelPendingWatches(state);
  ReleaseRegistration(state);
}

void IocpImpl::ForgetClosedFd(FileDescriptor fd) { DeleteFd(fd); }

void IocpImpl::EnableWritable(FileDescriptor fd) {
  FdState *state = GetActiveLegacyRegistration(fd);
  ABSL_CHECK(state != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!state->has_events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  state->legacy_events |= kOut;
  UpdateSocket(fd);
}

void IocpImpl::DisableWritable(FileDescriptor fd) {
  FdState *state = GetActiveLegacyRegistration(fd);
  ABSL_CHECK(state != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!state->has_events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  state->legacy_events &= ~kOut;
  UpdateSocket(fd);
}

void IocpImpl::SetEvents(FileDescriptor fd, uint32_t events) {
  FdState *state = GetActiveLegacyRegistration(fd);
  ABSL_CHECK(state != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(state->has_events_fn)
      << "SetEvents is only for fds registered using OnEvents";
  state->legacy_events = events;
  UpdateSocket(fd);
}

// Windows has no signalfd, so the receiver's manual-reset Event stands in
// for it: a legacy OnReadable() registration on that handle, the same shape
// EpollImpl gives the signalfd, taking the non-socket path through
// HandleWatch.  The callback consumes before it notifies, as it does there,
// which is what makes a burst collapse into one callback and keeps a signal
// that lands during the callback pending for the next one.
void IocpImpl::RegisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver, std::function<void()> callback) {
  ABSL_CHECK(receiver != nullptr);
  // Stores a std::function, which allocates; the rule is stated here as it
  // is in EpollImpl rather than left to the OnReadable() below.
  aos::CheckNotRealtime();
  ABSL_CHECK(signal_receiver_ == nullptr)
      << "Duplicate ThreadSignalReceiver registration: only one receiver "
         "may be active at a time (see Aio::RegisterThreadSignalReceiver)";
  // Registration runs on the polling thread, which is the thread whose
  // wakeups this receiver serves.  Construction may not have.
  receiver->BindToCurrentThread();
  signal_receiver_ = receiver;
  // Remembered rather than asked for again at unregister time: the receiver
  // is entitled to forget the handle once it is unbound, and this is the fd
  // the registration is held under.
  signal_receiver_handle_ = receiver->event_handle();
  OnReadable(reinterpret_cast<FileDescriptor>(signal_receiver_handle_),
             [receiver, callback = std::move(callback)]() {
               receiver->ConsumeWakeup();
               if (callback) callback();
             });
}

void IocpImpl::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  ABSL_CHECK_EQ(receiver, signal_receiver_)
      << ": ThreadSignalReceiver not found";
  DeleteFd(reinterpret_cast<FileDescriptor>(signal_receiver_handle_));
  signal_receiver_ = nullptr;
  signal_receiver_handle_ = NULL;
  receiver->UnbindFromCurrentThread();
}

void IocpImpl::ConsumeThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  receiver->ConsumeWakeup();
}

void IocpImpl::DispatchHandleSignal(FdState *state) {
  // The packet was one-shot; see HandleWatch.  Delivery disarmed it, and it
  // is not re-armed until after the callback below.
  state->handle_watch->MarkDelivered();
  // A signalled handle reports the mask the caller subscribed to, because
  // that is all a waitable object can say; see UpdateSocketState().
  const FileDescriptor fd = state->fd;
  const uint32_t events = state->legacy_events;
  // Same dispatch rules as the socket path, which are EPoll's.  Never
  // terminal: a signalled object reports the subscribed mask, and there is no
  // hangup to discover.
  DispatchLegacyEvents(state, fd, events, /*terminal=*/false);
  // Re-arm, now that the callback has had its chance to consume the handle.
  // Looked up again because that callback may have deleted the fd.
  if (FdState *live = GetActiveLegacyRegistration(fd)) {
    UpdateSocketState(live);
  }
}

struct WindowsTimerState : public Aio::TimerState {
  explicit WindowsTimerState(IocpImpl *impl) : impl_(impl) {}
  ~WindowsTimerState() override {
    Cancel(false);
    --impl_->active_timer_count_;
  }

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
      // One-shot: resolve before dispatching -- the callback may destroy
      // this timer.
      state->request.done = true;
      // Timer::Schedule() takes no user_data, so a timer completion carries
      // nullptr rather than an internal pointer, and result 0 -- see
      // Completion::user_data in aio.h, and the other three backends.
      // AioTest.TimerCompletionUserDataIsNull pins it, on every backend.
      Completion timer_completion = completion;
      timer_completion.user_data = nullptr;
      if (completion.status.has_value()) {
        timer_completion.status = aos::Ok();
        timer_completion.result = 0;
      }
      state->user_callback(timer_completion, state->user_context);
    };
    request.context = this;
    request.done = false;
    State(&request)->deadline_ns =
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
  ++active_timer_count_;
  return std::make_unique<WindowsTimerState>(this);
}

// The socket detail EPoll has always added to an unhandled error event.
// aos/events/aio_unix.h has the POSIX spelling; this is the winsock one,
// which needs no S_ISSOCK check because a FileDescriptor here is a SOCKET.
std::string SocketErrorStr(FileDescriptor fd) {
  int error = 0;
  int errlen = sizeof(error);
  if (getsockopt(ToSocket(fd), SOL_SOCKET, SO_ERROR,
                 reinterpret_cast<char *>(&error), &errlen) != 0) {
    return "";
  }
  if (error == 0) {
    return "";
  }
  return "Socket error: " + std::to_string(error);
}

bool IocpImpl::HasRawRequestsInFlight() const {
  // O(n) over the registrations, exactly as ReadinessBackend answers it.
  // Fine because of when it is asked: once per fork on the platforms that
  // have one, and never on this one.  No internal request to filter out,
  // unlike the three backends that exclude their own wakeup read: Wakeup()
  // here is PostQueuedCompletionStatus(), which owns no AsyncRequest.
  for (FdState *state : registrations_) {
    if (state->read_request != nullptr || state->write_request != nullptr) {
      return true;
    }
  }
  return false;
}

void IocpImpl::CompleteRawRequest(AsyncRequest **slot, FileDescriptor fd,
                                  LPOVERLAPPED lp_overlapped,
                                  DWORD bytes_transferred, BOOL success) {
  AsyncRequest *const request = *slot;
  *slot = nullptr;
  request->done = true;
  if (request->callback == nullptr) {
    return;
  }
  if (success) {
    // A zero-byte completion on a non-empty buffer is EOF -- the peer hung up
    // -- and that is a *successful* zero-length read, not a failure.  It is
    // what read(2) reports as 0, and what the io_uring and kqueue backends
    // pass through as Ok(); reporting an error here instead made a hangup
    // indistinguishable from real I/O failure.
    request->callback({aos::Ok(), static_cast<int32_t>(bytes_transferred),
                       request->user_data},
                      request->context);
    return;
  }
  DWORD dw_flags = 0;
  WSAGetOverlappedResult(ToSocket(fd), lp_overlapped, &bytes_transferred, FALSE,
                         &dw_flags);
  const DWORD err = GetLastError();
  if (err == ERROR_OPERATION_ABORTED) {
    request->callback({aos::MakeError("Canceled"), 0, request->user_data},
                      request->context);
  } else {
    request->callback(
        {aos::MakeError("iocp error"),
         TranslateWinsockError(static_cast<int32_t>(err)), request->user_data},
        request->context);
  }
}

void IocpImpl::DispatchLegacyEvents(FdState *state, FileDescriptor fd,
                                    uint32_t events, bool terminal) {
  if (events == 0 && !terminal) {
    return;
  }
  if (state->has_events_fn && state->events_fn) {
    // OnEvents sees a hangup, as EPOLLHUP does on Linux:
    // `terminal ? (ready | kErr) : ready`.  The trio below does not -- see
    // the peek in DispatchSocketCompletion().
    state->events_fn(terminal ? (events | kErr) : events);
    return;
  }
  // EPoll::InOutEventData::DoCallbacks()'s rules, message text included --
  // callers depend on those semantics, and the Linux and kqueue backends were
  // brought to them.  A readiness bit with no handler is a silent busy loop
  // rather than something to skip, and an error with no err_fn is fatal.
  if (events & kIn) {
    ABSL_CHECK(state->has_in_fn && state->in_fn)
        << ": No handler registered for input events on descriptor " << fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    state->in_fn();
  }
  if (events & kOut) {
    ABSL_CHECK(state->has_out_fn && state->out_fn)
        << ": No handler registered for output events on descriptor " << fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    state->out_fn();
  }
  // Gated on legacy_events -- the mask the caller subscribed to through On*()
  // -- as the other backends are: a registration carrying only raw requests is
  // not a shape EPoll could hold.
  if ((events & kErr) && state->legacy_events != 0) {
    ABSL_CHECK(state->has_err_fn && state->err_fn)
        << ": No handler registered for error events on descriptor " << fd
        << ". Received events = 0x" << std::hex << events << std::dec << ". "
        << SocketErrorStr(fd);
    state->err_fn();
  }
}

void IocpImpl::DispatchSocketCompletion(ULONG_PTR completion_key,
                                        LPOVERLAPPED lp_overlapped,
                                        DWORD bytes_transferred, BOOL success) {
  // An overlapped socket operation completed.
  pending_io_count_--;
  FileDescriptor fd = reinterpret_cast<FileDescriptor>(completion_key);
  // If the registration was retired with this operation still out (DeleteFd,
  // or a request cancelled at teardown), the state has been waiting on this
  // completion to be recyclable.  Each outstanding operation holds it
  // separately, so this releases one hold; a state with a read and a write
  // out waits for both.
  ClearRetiredPending(lp_overlapped);
  if (FdState *state = GetActiveRegistration(fd)) {
    if (lp_overlapped == &state->read_overlapped) {
      if (state->read_request) {
        CompleteRawRequest(&state->read_request, fd, lp_overlapped,
                           bytes_transferred, success);
      } else if (state->read_pending) {
        state->read_pending = false;
        // The readiness watch is a 0-byte WSARecv, so a successful
        // completion cannot itself distinguish "data is available"
        // from "the peer performed an orderly shutdown" (both
        // complete the same way).  Peek a byte (without consuming it)
        // to tell the two apart.
        //
        // A hangup is NOT an error, and conflating them here used to
        // abort the process.  epoll separates EPOLLHUP from EPOLLERR
        // and keeps HUP out of the in/out/err trio entirely
        // (aio_linux.cc); kqueue reports EVFILT_READ|EV_EOF, which
        // aio_darwin.cc translates to kIn alone.  Reporting kErr for
        // an orderly shutdown meant an ordinary OnReadable()-only
        // registration hit the err_fn CHECK the moment its peer
        // closed -- fatal here, silent on the other two backends.
        // ADR 0002's decision 3 is about exactly this trap.
        //
        // So `terminal` is tracked separately: it reaches OnEvents as
        // kErr, matching epoll's `terminal ? (ready | kErr) : ready`,
        // and never reaches err_fn on its own.  kIn is what lets the
        // reader run, observe the zero-length read and unregister,
        // which is the only way a hangup is ever consumed.
        uint32_t events = 0;
        bool terminal = false;
        if (success) {
          char peek_byte;
          int peek_res = recv(ToSocket(fd), &peek_byte, 1, MSG_PEEK);
          if (peek_res > 0) {
            events |= kIn;
          } else if (peek_res == 0) {
            // The peer is gone.  Which of kqueue's two EV_EOF cases
            // that is depends on what this registration subscribed
            // to, because kErr folds into want_read (see
            // UpdateSocketState()) and so one read watch stands in
            // for both filters here.
            //
            //   subscribed to kIn -- the *read* side hung up.  That
            //   is EVFILT_READ|EV_EOF, which aio_darwin.cc reports as
            //   kIn alone, and EPOLLHUP, which aio_linux.cc keeps out
            //   of the trio.  Not an error.
            //
            //   not subscribed to kIn -- the caller is watching for
            //   errors, so the condition that matters is the *write*
            //   side: the reader is gone and every later send fails.
            //   That is EVFILT_WRITE|EV_EOF -> kErr, and EPOLLERR.
            events |= (state->legacy_events & kIn) ? kIn : kErr;
            terminal = true;
          } else if (WSAGetLastError() != WSAEWOULDBLOCK) {
            events |= kErr;  // epoll's EPOLLERR.
            terminal = true;
          }
          // If peek_res < 0 with WSAEWOULDBLOCK, this was a spurious
          // wakeup; events stays 0, nothing is dispatched, and the re-arm
          // on the way out of this function puts the watch back.
        } else {
          events |= kErr;
          terminal = true;
        }
        // Only report bits the caller actually asked for, except
        // kErr, which (like epoll's EPOLLERR) is always reported
        // since this watch may have been armed purely to detect it.
        events &= state->legacy_events | kErr;
        DispatchLegacyEvents(state, fd, events, terminal);
      }
    } else if (lp_overlapped == &state->write_overlapped) {
      // Only a raw AsyncWrite() reaches here now.  A legacy write watch is
      // no longer an overlapped operation at all, so it has no OVERLAPPED to
      // come back on; DispatchWritableWatches() is where kOut comes from.
      if (state->write_request) {
        CompleteRawRequest(&state->write_request, fd, lp_overlapped,
                           bytes_transferred, success);
      }
    }
  }
  // Looked up again rather than carried across the dispatch above: the
  // callback is allowed to re-arm this fd, attach a legacy handler to it, or
  // (through DeleteFd()) retire it outright, and each of those changes both
  // answers below.
  if (FdState *state = GetActiveRegistration(fd)) {
    // A readiness watch is a single overlapped operation, so delivering it
    // disarmed it.  Re-arm here, after the callback -- before it would report
    // data the callback is about to consume.  The two calls are exclusive:
    // a registration is legacy or async-only, and each of these does nothing
    // to the other kind.
    UpdateSocketState(state);
    MaybeRetireAsyncRegistration(state);
  }
}

bool IocpImpl::Poll(bool block) {
  // Not reentrant, matching IoUringImpl::Poll() and EpollImpl::Poll().
  // dispatch_depth_ covers the whole body below, before-wait functions
  // included.
  ABSL_CHECK_EQ(dispatch_depth_, 0)
      << "Aio::Poll() reentered from inside a completion callback or "
         "before-wait function; wait by returning to the event loop instead";

  // Reclaim whatever the last Poll() retired from inside a dispatch, which
  // ReleaseRegistration() could only park at the time.  Here, where no
  // callback is on the stack and none of the std::functions about to be
  // destroyed is executing.  Matches ReadinessBackend::Poll().
  ScrubRetiredRegistrations();

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
  for (const auto &fn : before_wait_functions_) {
    fn();
  }
  in_before_wait_ = false;

  // At most one completion callback per Poll() -- see Aio::Poll().  Each of
  // the sources below delivers one and returns; the rest wait for the next
  // Poll(), the same way io_uring's pending_dispatch_ does.

  // Raw requests whose submit failed outright, each carrying the errno it is
  // to be completed with; see failed_requests_.  Popped past the ones with
  // no callback, which deliver nothing -- but retiring them is still
  // something that happened, so a Poll() that did only that reports true,
  // as io_uring does when it drains their CQEs.
  bool retired = false;
  while (AsyncRequest *request = failed_requests_.PopFront()) {
    request->done = true;
    retired = true;
    if (request->callback) {
      request->callback({aos::MakeError("iocp error"),
                         State(request)->failed_errno, request->user_data},
                        request->context);
      return true;
    }
  }
  if (retired) {
    return true;
  }

  // One clock reading per pass round the loop, and none when there is no
  // timer to compare it against.  A pass ends in a wait, which is the only
  // place time passes without this function knowing about it, so whatever
  // ends that wait -- the timer fired, a write watch's edge landed, the
  // write-watch cap expired -- goes round again and judges the head
  // deadline against a fresh reading.  The expired-timer check after the
  // loop uses the last one; nothing waits between the two.
  monotonic_clock::time_point now = monotonic_clock::min_time;
  bool head_due = false;
  while (true) {
    // Writability is level-triggered, so it is asked rather than waited for;
    // see WriteWatch.  At the top of the loop so that every re-entry asks
    // again.
    if (DispatchWritableWatches()) {
      return true;
    }
    if (!timers_.empty()) {
      now = monotonic_clock::now();
      head_due = Deadline(timers_.front()) <= now;
    }
    // The wait is either a poll (0) or unbounded (INFINITE): a pending
    // deadline arrives as the timer's packet, never as a timeout.  See
    // DeadlineTimer for why.
    DWORD timeout_dw = INFINITE;
    if (!block || head_due) {
      // A due timer still polls the port once, so I/O already queued goes
      // first; the timer fires below when nothing is.
      timeout_dw = 0;
    } else if (!timers_.empty()) {
      deadline_timer_.Arm(iocp_handle_, Deadline(timers_.front()), now);
    } else {
      // Nothing to wait for but I/O.  A timer armed for a deadline that has
      // since been cancelled would otherwise wake this wait for nothing.
      deadline_timer_.Disarm();
    }

    // The backstop for the one case FD_WRITE cannot signal: a socket filled
    // exactly, by a caller whose sends therefore never failed, so the stack
    // owes no edge and nothing will ever wake this wait.  Measured and
    // unavoidable -- see WriteWatch.
    //
    // Gated on there being a live writable subscription, not merely on this
    // being a socket loop: writable_watches_ holds the fds currently
    // subscribed to kOut.  With no OnWritable() anywhere -- every caller on
    // this platform today -- this is dead code and the wait stays unbounded.
    //
    // It also only ever shortens a wait that was going to be indefinite: if a
    // timer or any other completion is due first, that arrives and the check
    // at the top of this loop picks the socket up, so this never fires at
    // all.  What it costs is bounded by how long a full socket stays full.
    if (timeout_dw == INFINITE && !writable_watches_.empty()) {
      timeout_dw = kWriteWatchBackstop;
    }

    DWORD bytes_transferred = 0;
    ULONG_PTR completion_key = 0;
    LPOVERLAPPED lp_overlapped = nullptr;
    const BOOL success =
        GetQueuedCompletionStatus(iocp_handle_, &bytes_transferred,
                                  &completion_key, &lp_overlapped, timeout_dw);

    if (!success && lp_overlapped == nullptr) {
      // Nothing was dequeued.  WAIT_TIMEOUT is the wait ending on its own:
      // the zero timeout of a non-blocking poll or a due timer, which are
      // settled below, or the write-watch cap, which goes round again for
      // the check at the top.  Anything else is the port itself failing.
      const DWORD err = GetLastError();
      if (err != WAIT_TIMEOUT) {
        return false;
      }
      if (!block || head_due) {
        break;
      }
      continue;
    }

    switch (completion_key) {
      case kTimerKey:
        // The head deadline's timer fired.  Nothing is dispatched here: the
        // loop goes round, judges the head against a fresh clock reading,
        // and -- if it really is due -- lands in the expired-timer check
        // below.  A fire that is not quite due yet (the two clocks differ)
        // re-arms for the remainder instead.
        deadline_timer_.OnDelivered();
        continue;
      case kWritableKey: {
        // A write watch's FD_WRITE landed.  Not a dispatch on its own -- it
        // says only that writability may have changed, and the level check
        // at the top of the loop is what decides.  Consumed so the next edge
        // is not lost, re-armed, then round again.  The state is live: a
        // watch torn down since the kernel queued this took the packet with
        // it (see WaitPacket::Disarm()).
        FdState *const state =
            static_cast<FdState *>(static_cast<void *>(lp_overlapped));
        state->write_watch->Consume();
        state->write_watch->Arm(iocp_handle_);
        continue;
      }
      case kWakeupKey:
        // Quit(), or a nudge from a previous life of the port.  Nothing to
        // run, but the wait ended, which is what the caller asked about.
        return true;
      case kHandleKey:
        // A watched non-socket handle signalled; the state is live for the
        // same reason as above.
        DispatchHandleSignal(
            static_cast<FdState *>(static_cast<void *>(lp_overlapped)));
        return true;
      default:
        DispatchSocketCompletion(completion_key, lp_overlapped,
                                 bytes_transferred, success);
        return true;
    }
  }

  // Check for expired timers.  One per Poll(), and only if nothing was
  // dispatched above -- see Aio::Poll().  A timer left over here is already
  // past its deadline, so the very next Poll() (Run() calls one immediately)
  // fires it without waiting on anything.
  while (!timers_.empty() && Deadline(timers_.front()) <= now) {
    AsyncRequest *const request = timers_.front();
    RemoveTimer(request);
    request->done = true;
    if (request->callback) {
      request->callback({aos::Ok(), 0, request->user_data}, request->context);
      return true;
    }
  }

  return false;
}

Aio::Aio() { impl_ = std::make_unique<IocpImpl>(); }

}  // namespace aos

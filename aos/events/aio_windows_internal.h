#ifndef AOS_EVENTS_AIO_WINDOWS_INTERNAL_H_
#define AOS_EVENTS_AIO_WINDOWS_INTERNAL_H_

// What the two Windows backends share: the IOCP backend that owns its port
// (aio_windows.cc) and the libuv backend on a borrowed one (aio_uv.cc).  Both
// wait on kernel objects that cannot join a completion port -- a timer, an
// Event -- by attaching wait completion packets to whichever port they have.
// Private to this package, as aio_internal.h is; nothing outside the backends
// includes it.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <winternl.h>

#include <algorithm>
#include <chrono>
#include <cstdint>

#include "absl/log/absl_check.h"

#include "aos/events/aio.h"
#include "aos/time/time.h"

namespace aos {

// Maps a WSAE* code onto the errno every other platform reports, so a
// Completion's result means the same thing everywhere.  Defined in
// aio_windows.cc.
int TranslateWinsockError(int err);

// On Windows a FileDescriptor is an opaque handle that holds the SOCKET
// directly (see aio.h), so no lookup table is needed to recover it.
inline SOCKET ToSocket(FileDescriptor fd) {
  return reinterpret_cast<SOCKET>(fd);
}

// Whether fd names a socket rather than some other waitable Win32 object.
//
// ToSocket() above is a bare reinterpret_cast, so nothing downstream can tell
// the difference on its own -- which is why a non-socket handle used to reach
// CreateIoCompletionPort() and fail with ERROR_INVALID_HANDLE on every Poll().
// getsockopt(SO_TYPE) is the cheap, side-effect-free question: it answers for
// any socket and fails with WSAENOTSOCK for anything that is not one.
//
// Cached per registration rather than asked per Poll(): the answer cannot
// change for a given handle, and this sits on the arming path.
inline bool IsSocket(FileDescriptor fd) {
  int type = 0;
  int length = sizeof(type);
  return getsockopt(ToSocket(fd), SOL_SOCKET, SO_TYPE,
                    reinterpret_cast<char *>(&type), &length) == 0;
}

// The level oracle for writability: whether a send would go through right
// now, which is the question EPOLLOUT answers.  Both backends need it,
// because neither of the edges this platform offers -- FD_WRITE, or a poll
// libuv re-submits the moment it reports -- says whether the level still
// holds by the time the caller is told.
inline bool SocketWritable(SOCKET s) {
  fd_set writable;
  FD_ZERO(&writable);
  FD_SET(s, &writable);
  TIMEVAL immediately = {0, 0};
  // nfds is ignored on Windows; the fd_set carries its own count.
  return select(0, nullptr, &writable, nullptr, &immediately) == 1;
}

// Wait completion packets.
//
// The kernel's own way of turning "this object became signaled" into a
// completion on a port: a packet is associated with (port, object, key,
// context), and when the object signals the kernel queues that packet to the
// port.
//
// Nt* rather than Win32: there is no documented Win32 wrapper.  Resolved by
// name from ntdll at first use and CHECKed, since the alternative is silent.
// Present since Windows 8.
//
// Semantics, established by probe rather than documentation:
//   - One-shot.  A delivered packet is no longer associated; re-associate to
//     watch again.  Associating an already-associated packet fails with
//     STATUS_INVALID_PARAMETER_1.
//   - Associating with an already-signaled object queues the packet at once
//     and reports AlreadySignaled.
//   - Cancel with RemoveSignaledPacket=TRUE also pulls a packet that is
//     queued but not yet dequeued.  Cancel with nothing associated is
//     STATUS_CANCELLED and otherwise harmless.
//   - Closing the target object, or the port, while associated delivers
//     nothing and leaves the packet cancellable and closable.
typedef NTSTATUS(NTAPI *NtCreateWaitCompletionPacketFn)(PHANDLE, ACCESS_MASK,
                                                        POBJECT_ATTRIBUTES);
typedef NTSTATUS(NTAPI *NtAssociateWaitCompletionPacketFn)(
    HANDLE WaitCompletionPacketHandle, HANDLE IoCompletionHandle,
    HANDLE TargetObjectHandle, PVOID KeyContext, PVOID ApcContext,
    NTSTATUS IoStatus, ULONG_PTR IoStatusInformation, PBOOLEAN AlreadySignaled);
typedef NTSTATUS(NTAPI *NtCancelWaitCompletionPacketFn)(
    HANDLE WaitCompletionPacketHandle, BOOLEAN RemoveSignaledPacket);

struct WaitPacketApi {
  NtCreateWaitCompletionPacketFn create;
  NtAssociateWaitCompletionPacketFn associate;
  NtCancelWaitCompletionPacketFn cancel;
};

// Both defined in aio_windows.cc.
const WaitPacketApi &WaitPackets();
HANDLE CreateWaitPacket();

inline bool NtOk(NTSTATUS status) { return status >= 0; }

// One wait completion packet -- see WaitPackets() for what the kernel
// promises about them -- plus the one fact about it the kernel does not
// report back: whether it is currently associated.  Owns the handle.
class WaitPacket {
 public:
  WaitPacket() : packet_(CreateWaitPacket()) {}
  ~WaitPacket() {
    Disarm();
    ABSL_PCHECK(CloseHandle(packet_)) << "CloseHandle failed";
  }
  WaitPacket(const WaitPacket &) = delete;
  WaitPacket &operator=(const WaitPacket &) = delete;

  // Associates the packet with target on port, so that when target signals,
  // whoever dequeues from port gets key as the completion key and context as
  // lpOverlapped.  A target that is already signaled queues the packet right
  // here.  Not legal while armed: the kernel refuses a second association.
  //
  // A null context is a wake with nothing attached.  libuv treats a dequeued
  // packet with no lpOverlapped as exactly that, so it is how the libuv
  // backend rides a borrowed port; the IOCP backend always attaches one.
  void Arm(HANDLE port, HANDLE target, ULONG_PTR key, void *context) {
    ABSL_CHECK(!armed_) << ": wait packet armed twice";
    BOOLEAN already_signaled = FALSE;
    const NTSTATUS status = WaitPackets().associate(
        packet_, port, target, reinterpret_cast<PVOID>(key), context,
        /*IoStatus=*/0, /*IoStatusInformation=*/0, &already_signaled);
    ABSL_CHECK(NtOk(status)) << ": NtAssociateWaitCompletionPacket failed: 0x"
                             << std::hex << static_cast<uint32_t>(status);
    armed_ = true;
  }

  // Withdraws the association.  RemoveSignaledPacket: a packet the kernel
  // has already queued but nobody has yet dequeued goes too, so nothing for
  // this packet arrives after Disarm() returns.  That is what lets a dequeued
  // context be used as a live pointer, and what lets ~IocpImpl()'s drain
  // treat everything it dequeues as socket I/O.  A no-op when not armed.
  void Disarm() {
    if (!armed_) {
      return;
    }
    WaitPackets().cancel(packet_, /*RemoveSignaledPacket=*/TRUE);
    armed_ = false;
  }

  // The packet was dequeued.  A delivered packet is no longer associated;
  // whoever owns it decides whether to Arm() again.
  void MarkDelivered() { armed_ = false; }

  bool armed() const { return armed_; }

 private:
  const HANDLE packet_;
  bool armed_ = false;
};

// One high-resolution kernel timer, armed for a deadline and delivered to a
// port as a wait completion packet, so the loop on that port waits with no
// timeout of its own.
//
// Not the port's own timeout, which is whole milliseconds rounded up: a 500us
// timer waited a full millisecond, and a repeating one drifted late every
// cycle.  This takes its due time in 100ns units instead.  Measured lateness
// on an idle machine: floor 35-100us, median 0.2-1.0ms, worst 1.2ms, down
// from 4.5ms.  See EnsureHighResolutionTimers() in aio_windows.cc for the
// system tick underneath all of those.
class DeadlineTimer {
 public:
  // What a delivered packet carries: `key` as the completion key and no
  // context.  Manual-reset keeps the timer signalled after it fires until it
  // is set again, so a loop that never sees the packet itself -- libuv's
  // dequeues it as a bare wake -- can still ask Fired().  A synchronization
  // (auto-reset) timer is the right one where the packet is seen: the packet
  // consumes the signal on delivery, and SetWaitableTimer() clears a fire
  // nobody consumed, so a stale expiry cannot leak into the next arming as an
  // immediate delivery (probed).
  explicit DeadlineTimer(ULONG_PTR key, bool manual_reset = false) : key_(key) {
    // The high-resolution flag needs Windows 10 1803, older than anything
    // this runs on; refusing to start beats quietly quantising every deadline
    // to the tick.
    timer_ = CreateWaitableTimerExW(
        NULL, NULL,
        CREATE_WAITABLE_TIMER_HIGH_RESOLUTION |
            (manual_reset ? CREATE_WAITABLE_TIMER_MANUAL_RESET : 0),
        TIMER_ALL_ACCESS);
    ABSL_PCHECK(timer_ != NULL)
        << "CreateWaitableTimerExW(HIGH_RESOLUTION) failed; this backend "
           "needs Windows 10 1803 or later";
  }
  ~DeadlineTimer() {
    // Before the timer the packet is watching goes, not after.
    packet_.Disarm();
    ABSL_PCHECK(CloseHandle(timer_)) << "CloseHandle failed";
  }
  DeadlineTimer(const DeadlineTimer &) = delete;
  DeadlineTimer &operator=(const DeadlineTimer &) = delete;

  // Arms for deadline on aos::monotonic_clock, the clock every deadline is
  // expressed in.  A no-op if that is what it is armed for already.
  void Arm(HANDLE port, aos::monotonic_clock::time_point deadline,
           aos::monotonic_clock::time_point now) {
    if (packet_.armed() && armed_deadline_ == deadline) {
      return;
    }
    // A different deadline: withdraw the old one, queued packet included, so
    // a stale expiry cannot wake the wait.
    Disarm();
    // Negative because that is how SetWaitableTimer() is told a time is
    // relative: a positive lpDueTime is an absolute FILETIME, a negative one
    // is 100ns units from now, which is what deadline-minus-now gives us.
    // Rounded up so the rounding can never fire it early.
    //
    // Set before associate, never after: setting the timer clears a fire
    // nobody consumed, and associating with an object that is already
    // signalled delivers at once.
    const int64_t delta_ns = std::max<int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now)
            .count(),
        1);
    LARGE_INTEGER due;
    due.QuadPart = -((delta_ns + 99) / 100);
    ABSL_PCHECK(SetWaitableTimer(timer_, &due, 0, nullptr, nullptr, FALSE))
        << "SetWaitableTimer failed";
    packet_.Arm(port, timer_, key_, /*context=*/nullptr);
    armed_deadline_ = deadline;
  }

  void Disarm() {
    packet_.Disarm();
    armed_deadline_ = aos::monotonic_clock::min_time;
  }

  // The timer's packet was dequeued.  Whether the deadline is really due is
  // the caller's to decide: the timer may run a little ahead of
  // monotonic_clock, since the two are read from different clocks.
  void OnDelivered() {
    packet_.MarkDelivered();
    armed_deadline_ = aos::monotonic_clock::min_time;
  }

  // Whether the timer has fired since it was last set.  Only a manual-reset
  // timer can answer; an auto-reset one was consumed by the packet.
  bool Fired() const { return WaitForSingleObject(timer_, 0) == WAIT_OBJECT_0; }

  bool armed() const { return packet_.armed(); }

 private:
  const ULONG_PTR key_;
  HANDLE timer_ = NULL;
  WaitPacket packet_;
  // Meaningless while the packet is not armed.
  aos::monotonic_clock::time_point armed_deadline_ =
      aos::monotonic_clock::min_time;
};

}  // namespace aos

#endif  // AOS_EVENTS_AIO_WINDOWS_INTERNAL_H_

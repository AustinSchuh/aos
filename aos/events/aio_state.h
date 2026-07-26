#ifndef AOS_EVENTS_AIO_STATE_H_
#define AOS_EVENTS_AIO_STATE_H_

#include <cstdint>

#include "aos/events/aio.h"

namespace aos {

// Which raw operation a request is carrying, for IoUringImpl.
enum class RawIo : uint8_t {
  kNone = 0,
  kRead,
  kWrite,
};

// Backend-private per-request state, overlaid on AsyncRequest's opaque
// internal_state buffer so that aio.h stays free of backend types.
//
// One definition for every backend rather than one each: the members are
// deliberately separate rather than a union, because a request can be
// staged for I/O and queued on a list at the same time.  An earlier
// per-backend union overlapped those two and only luck kept it from
// corrupting the list.
struct AioState {
  // List node, used by every backend: IoUringImpl::pending_dispatch_ uses
  // all four fields; the readiness backends' pending lists are singly
  // linked and use only next/result.
  struct {
    AsyncRequest *next;
    // Doubly linked so IoUringImpl::UnlinkPendingDispatch() can splice a
    // request out of pending_dispatch_ in O(1).
    AsyncRequest *prev;
    int32_t result;
    // Set while linked on IoUringImpl::pending_dispatch_ (see
    // QueuePendingDispatch()).
    int32_t queued;
  } link;
  // Readiness backends only (epoll, kqueue): read/write staging for
  // AsyncRead()/AsyncWrite(), which do the I/O themselves when the fd
  // reports ready.  io_uring hands its span straight to the SQE and never
  // touches this.  Deliberately NOT overlapped with `link` -- a request can
  // be staged and queued at the same time.
  struct {
    void *ptr;
    size_t size;
  } raw;
  // IoUringImpl only.
  struct {
    // List node and fd for raw_in_flight_, valid while raw_io is set --
    // this is what lets legacy registration CHECK the no-mixing contract
    // (see ClaimRawFd()).  Separate from `link` -- see the struct comment.
    AsyncRequest *raw_prev;
    AsyncRequest *raw_next;
    int32_t raw_fd;
    // Incarnation counter: incremented per fresh kernel op submitted under
    // this request's identity and encoded into that op's user_data.  See
    // EncodeUserData() for why 16 bits cannot wrap into ambiguity.
    uint16_t generation;
    // Set while a caller-submitted AsyncRead/AsyncWrite is in flight (the
    // internal wakeup read is excluded).  Backs raw_requests_in_flight_
    // (see DowngradeFromSingleIssuer()) and tells the terminal completion
    // which per-fd no-mixing claim to release.
    RawIo raw_io;
    // Set at queue time when this request's callback is the caller's own
    // rather than one of this file's trampolines, so ReapCompletions() can
    // tell whether dispatching it spends the one-user-completion-per-Poll()
    // budget.  The trampolines report for themselves.
    //
    // Note that this is a boolean flag, but uses uint8_t to ensure
    // struct alignment/packing without complications around the "bool"
    // type.
    uint8_t user_visible;
  } uring;
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

}  // namespace aos

#endif  // AOS_EVENTS_AIO_STATE_H_

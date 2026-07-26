#ifndef AOS_EVENTS_AIO_REGISTRATIONS_H_
#define AOS_EVENTS_AIO_REGISTRATIONS_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/events/aio_internal.h"
#include "aos/events/file_descriptor.h"
#include "aos/events/intrusive_rb_tree.h"

namespace aos::internal {

// AOS's fd-readiness event encoding, shared by every backend and by the
// public SetEvents()/OnEvents() contract.  Numerically identical to the low
// 4 bits of the real epoll event flags (the encoding documented on
// Aio::OnEvents()).
inline constexpr uint32_t kIn = 0x01;   // EPOLLIN
inline constexpr uint32_t kPri = 0x02;  // EPOLLPRI
inline constexpr uint32_t kOut = 0x04;  // EPOLLOUT
inline constexpr uint32_t kErr = 0x08;  // EPOLLERR

inline constexpr uint32_t kInEvents = kIn | kPri;
inline constexpr uint32_t kOutEvents = kOut;
inline constexpr uint32_t kErrorEvents = kErr;

// What one fd is registered for, and the table that owns them.
//
// Every readiness-driven backend -- epoll, kqueue, IOCP -- keeps exactly
// this state and recycles it exactly this way.  Only the syscall that arms
// and disarms the kernel differs, and that stays in the backend.  Sharing
// the bookkeeping is mostly about the parts that are subtle rather than the
// parts that are long: the deferred teardown below took an ASAN run to find
// once already, and there is no reason for each backend to rediscover it.
struct FdRegistration {
  FileDescriptor fd = kInvalidFd;
  // The caller's outstanding AsyncRead()/AsyncWrite(), if any.
  AsyncRequest *read_req = nullptr;
  AsyncRequest *write_req = nullptr;

  // The legacy readiness handlers.
  std::function<void()> in_fn = nullptr;
  std::function<void()> out_fn = nullptr;
  std::function<void()> err_fn = nullptr;
  std::function<void(uint32_t)> events_fn = nullptr;
  // The mask the caller subscribed to through On*()/SetEvents().
  uint32_t events = 0;

  // What the kernel currently has, so a backend can skip a no-op syscall.
  bool registered = false;
  uint32_t epoll_events = 0;

  // True while this registration exists only to carry AsyncRead/AsyncWrite
  // requests; retired back to the pool when the last one finishes (see
  // ShouldRetire()).  Cleared when a legacy registration attaches, which
  // lives until DeleteFd()/ForgetClosedFd().
  bool async_only = false;
  // Set when the kernel refused to poll this fd because it is always ready
  // (epoll_ctl(ADD) answering EPERM for a regular file).  The submit paths
  // do the I/O inline instead of waiting for an event that cannot arrive.
  bool unpollable = false;
  // Positive errno from a registration the kernel rejected outright (a
  // closed fd answering EBADF).  UpdateRegistration() returns false and
  // leaves it here; the submit paths turn it into the error completion
  // aio.h promises, rather than taking the process down.
  int registration_errno = 0;
  // A raw request that was Cancel()ed but whose Canceled completion has not
  // been dispatched yet still owns this fd.  aio.h documents Cancel() as
  // asynchronous -- the request completes through Poll() -- so the fd is not
  // free until then, which is where io_uring releases its claim (at the
  // terminal CQE, not at Cancel()).
  bool cancel_pending_read = false;
  bool cancel_pending_write = false;

  // Link for the free list and the retired list.  One link covers both: a
  // registration is only ever on one of them, moving free -> live ->
  // retired -> free.
  FdRegistration *next_free = nullptr;
  // Link for the list that owns every allocation for the table's lifetime,
  // so the tree and the free/retired stacks can all be non-owning.
  FdRegistration *next_all = nullptr;

  // Links for the live tree, keyed on fd.  Owned by FdRegistrationTable;
  // nothing else may read or write them while the registration is live.
  FdRegistration *tree_left = nullptr;
  FdRegistration *tree_right = nullptr;
  FdRegistration *tree_parent = nullptr;
  bool tree_red = false;

  // Drops everything tying this registration to an fd -- but deliberately
  // not the callbacks.  Release() can run underneath one of them (a
  // callback which DeleteFd()s its own fd), and destroying a std::function
  // there would free the frame that is executing.  Reset() does that half,
  // once it is safe.
  void Detach() {
    fd = kInvalidFd;
    read_req = nullptr;
    write_req = nullptr;
    cancel_pending_read = false;
    cancel_pending_write = false;
    events = 0;
    registered = false;
    epoll_events = 0;
  }

  // Drops what Detach() left behind, returning this to pool state.  Only
  // safe with no dispatch in flight; see ScrubRetired().
  void Reset() {
    in_fn = nullptr;
    out_fn = nullptr;
    err_fn = nullptr;
    events_fn = nullptr;
    async_only = false;
    unpollable = false;
  }
};

struct FreeLinkTraits {
  static FdRegistration *&next(FdRegistration *reg) { return reg->next_free; }
};

struct AllLinkTraits {
  static FdRegistration *&next(FdRegistration *reg) { return reg->next_all; }
};

// Orders the live registrations by fd.  Compare() lets GetActive() look one
// up from a bare fd without building a registration to compare against.
struct LiveTreeTraits {
  static FdRegistration *&left(FdRegistration *reg) { return reg->tree_left; }
  static FdRegistration *&right(FdRegistration *reg) { return reg->tree_right; }
  static FdRegistration *&parent(FdRegistration *reg) {
    return reg->tree_parent;
  }
  static bool &red(FdRegistration *reg) { return reg->tree_red; }
  static bool Less(FdRegistration *a, FdRegistration *b) {
    return a->fd < b->fd;
  }
  static int Compare(const FdRegistration *reg, FileDescriptor fd) {
    if (reg->fd < fd) return -1;
    if (fd < reg->fd) return 1;
    return 0;
  }
};

class FdRegistrationTable {
 public:
  // pool_size registrations are allocated up front so the hot paths do not.
  // Running dry is not fatal -- Allocate() falls back to new -- but that
  // allocation is what the realtime malloc hook is there to catch, so the
  // pool is sized to make it not happen.
  explicit FdRegistrationTable(size_t pool_size) {
    for (size_t i = 0; i < pool_size; ++i) {
      auto *reg = new FdRegistration();
      all_.Push(reg);
      free_list_.Push(reg);
    }
  }

  ~FdRegistrationTable() {
    // Cleared here rather than left to the members' destructors so that the
    // std::functions a parked registration still holds are destroyed while
    // the owning backend is still intact -- a capture's destructor can
    // reenter it.
    all_.Clear();
  }

  FdRegistrationTable(const FdRegistrationTable &) = delete;
  FdRegistrationTable &operator=(const FdRegistrationTable &) = delete;

  // Live registrations, in a tree keyed on fd.
  FdRegistration *GetActive(FileDescriptor fd) const { return live_.Find(fd); }

  // For a legacy On*() registration, which outlives any single request and
  // so is allocated outside the pool.
  FdRegistration *GetOrCreateLegacy(FileDescriptor fd) {
    if (auto *reg = GetActive(fd)) {
      // Legacy state lives until DeleteFd()/ForgetClosedFd() now, so this
      // must no longer auto-retire when a request finishes.
      reg->async_only = false;
      return reg;
    }
    return Insert(Allocate(), fd, false);
  }

  // For a raw AsyncRead()/AsyncWrite(), which is retired again as soon as
  // its last request completes.
  FdRegistration *GetOrCreateAsync(FileDescriptor fd) {
    if (auto *reg = GetActive(fd)) {
      return reg;
    }
    return Insert(Allocate(), fd, true);
  }

  // Whether reg exists only for raw requests and its last one just
  // finished, so the caller should deregister it from the kernel and
  // Release() it.
  static bool ShouldRetire(const FdRegistration *reg) {
    return reg->async_only && reg->read_req == nullptr &&
           reg->write_req == nullptr;
  }

  // Detaches reg from its fd and parks it.  Nothing is destroyed while a
  // dispatch is in flight: one of reg's std::functions can be the very
  // function calling this (a callback that DeleteFd()s its own fd), and the
  // dispatch loop may still re-read reg->fd afterwards to notice the
  // deletion.  ScrubRetired() finishes the job.
  void Release(FdRegistration *reg) {
    ABSL_CHECK(live_.Find(reg->fd) == reg);
    live_.Remove(reg);
    reg->Detach();
    // Parked, not freed: one of the std::functions it still holds can be the
    // very callback that called this (a callback that DeleteFd()s its own
    // fd), and the dispatch loop may still re-read reg->fd afterwards to
    // notice the deletion.  ScrubRetired() frees it once neither is true.
    retired_.Push(reg);
    if (dispatch_depth_ == 0) {
      ScrubRetired();
    }
  }

  // Moves everything parked by Release() back to the pool, destroying the
  // callbacks it was still holding.  Only legal with no dispatch in flight.
  //
  // No trimming and no free(): all_ owns every registration for the table's
  // lifetime, so the free list only ever grows to the peak concurrent fd
  // count and hands the same objects back out.  The sorted-vector version
  // had to give surplus back with delete, which put a free() inside the
  // caller's realtime section.
  //
  // Reset() destroys the registration's std::functions.  The only ones a
  // realtime section can reach are the submit paths' own [this, fd] lambdas,
  // which have to fit std::function's inline buffer already -- AsyncRead()
  // assigns them on a path aio.h requires to be allocation-free, so if they
  // ever heap allocated the submit would be the bug, not the scrub.  A
  // caller's own handler is only ever released by DeleteFd() or
  // ForgetClosedFd(), both aos::CheckNotRealtime().
  void ScrubRetired() {
    ABSL_CHECK_EQ(dispatch_depth_, 0);
    while (FdRegistration *reg = retired_.Pop()) {
      reg->Reset();
      free_list_.Push(reg);
    }
  }

  // Non-zero while the owner is running callbacks.  Release() consults it,
  // and the backend's Poll() reentrancy CHECK reads it.
  int dispatch_depth() const { return dispatch_depth_; }

  // Raises dispatch_depth() for as long as it is alive.
  class DispatchGuard {
   public:
    explicit DispatchGuard(FdRegistrationTable *table) : table_(table) {
      ++table_->dispatch_depth_;
    }
    ~DispatchGuard() { --table_->dispatch_depth_; }
    DispatchGuard(const DispatchGuard &) = delete;
    DispatchGuard &operator=(const DispatchGuard &) = delete;

   private:
    FdRegistrationTable *table_;
  };

  // Iteration over the live registrations, for a backend's destructor
  // checks and for rebuilding them all after a fork.
  auto begin() const { return live_.begin(); }
  auto end() const { return live_.end(); }

 private:
  // A registration from the pool, or a fresh one if it has run dry.  The
  // fallback keeps an unusually large fd count working rather than turning
  // it into a CHECK; on a realtime thread the malloc hook is what objects,
  // which is the right place for that to surface.
  FdRegistration *Allocate() {
    if (FdRegistration *reg = free_list_.Pop()) {
      return reg;
    }
    auto *reg = new FdRegistration();
    all_.Push(reg);
    return reg;
  }

  FdRegistration *Insert(FdRegistration *reg, FileDescriptor fd,
                         bool async_only) {
    reg->fd = fd;
    reg->async_only = async_only;
    live_.Insert(reg);
    return reg;
  }

  // Owns every registration for this table's lifetime, so the tree and the
  // two stacks below can all be non-owning -- moving a registration between
  // them is then pointer assignment, which is what makes Release() safe on
  // the dispatch path under realtime.
  OwningIntrusiveStack<FdRegistration, AllLinkTraits> all_;
  // Live registrations, keyed on fd.  Intrusive, so inserting one allocates
  // nothing -- which the sorted vector this replaced could not promise, since
  // it held the wakeup, every legacy fd and every timer fd and so outgrew its
  // reserve on the submit path.
  IntrusiveRbTree<FdRegistration, LiveTreeTraits> live_;
  IntrusiveStack<FdRegistration, FreeLinkTraits> free_list_;
  IntrusiveStack<FdRegistration, FreeLinkTraits> retired_;
  int dispatch_depth_ = 0;
};

}  // namespace aos::internal

#endif  // AOS_EVENTS_AIO_REGISTRATIONS_H_

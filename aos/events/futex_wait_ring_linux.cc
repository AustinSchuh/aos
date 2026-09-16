#include "aos/events/futex_wait_ring.h"

#include <errno.h>
#include <liburing.h>
#include <linux/futex.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/libc/aos_strerror.h"
#include "aos/realtime.h"

// See aio_linux.cc for why this may be missing from the sysroot's headers.
#ifndef FUTEX2_SIZE_U32
#define FUTEX2_SIZE_U32 0x02
#endif

ABSL_DECLARE_FLAG(uint32_t, aio_queue_depth);

namespace aos::internal {
namespace {

// Completion user_data values that are not waiters.  Waiters are identified
// by their pointer, which is 8-byte aligned, so small odd numbers never
// collide.
constexpr uint64_t kCancelToken = 1;
constexpr uint64_t kNopToken = 3;

// The ring itself: arm, cancel, drain.  Every method runs on the loop thread,
// which is the ring's single issuer.
class FutexWaitRing {
 public:
  FutexWaitRing() { Create(); }
  ~FutexWaitRing() { io_uring_queue_exit(&ring_); }

  FutexWaitRing(const FutexWaitRing &) = delete;
  FutexWaitRing &operator=(const FutexWaitRing &) = delete;

  int fd() const { return ring_.ring_fd; }

  // Stages and submits one FUTEX_WAIT on word for its value to leave
  // expected, completing under token.  RT-safe: an SQE and one syscall.
  void Arm(const std::atomic<uint32_t> *word, uint32_t expected,
           uint64_t token) {
    struct io_uring_sqe *sqe = GetSqe();
    // Shared, not FUTEX2_PRIVATE: the word is in a MAP_SHARED queue and the
    // waker is another process.
    io_uring_prep_futex_wait(sqe, reinterpret_cast<const uint32_t *>(word),
                             expected, FUTEX_BITSET_MATCH_ANY, FUTEX2_SIZE_U32,
                             0);
    io_uring_sqe_set_data64(sqe, token);
    Submit();
  }

  // Cancels token's wait and blocks until its completion has been seen, so
  // the caller may free whatever token names.  Completions for other tokens
  // met on the way are held for the next Drain(), and a NOP is queued so
  // the fd reports readable for them.  Not RT: the hold list may grow.
  void Cancel(uint64_t token) {
    aos::CheckNotRealtime();
    struct io_uring_sqe *sqe = GetSqe();
    io_uring_prep_cancel64(sqe, token, 0);
    io_uring_sqe_set_data64(sqe, kCancelToken);
    Submit();

    bool held_any = false;
    while (true) {
      struct io_uring_cqe *cqe = nullptr;
      const int ret = io_uring_wait_cqe(&ring_, &cqe);
      if (ret == -EINTR) {
        continue;
      }
      ABSL_CHECK_EQ(ret, 0)
          << ": io_uring_wait_cqe failed: " << aos_strerror(-ret);
      const uint64_t data = io_uring_cqe_get_data64(cqe);
      const int32_t res = cqe->res;
      io_uring_cqe_seen(&ring_, cqe);
      if (data == token) {
        // -ECANCELED from our cancel, or a wake that beat it.  Either way
        // the wait is over and nothing names the token any more.
        break;
      }
      if (data == kCancelToken) {
        // A non-negative result is a cancel that took: 0 for most ops, and
        // for a futex wait the number of waits removed, 1.  -ENOENT /
        // -EALREADY: the wait was already completing; its completion is
        // what this loop is waiting for.
        ABSL_CHECK(res >= 0 || res == -ENOENT || res == -EALREADY)
            << ": unexpected async-cancel result " << aos_strerror(-res);
        continue;
      }
      if (data == kNopToken) {
        continue;
      }
      held_.emplace_back(data, res);
      held_any = true;
    }
    if (held_any) {
      // The CQ may now be empty while completions sit in held_, and an
      // empty CQ is not readable.  A NOP completes immediately and makes
      // the fd readable, so the next turn of the loop drains them.
      struct io_uring_sqe *nop = GetSqe();
      io_uring_prep_nop(nop);
      io_uring_sqe_set_data64(nop, kNopToken);
      Submit();
    }
  }

  // Hands every completion -- held ones first -- to fn(token, res).  Called
  // from the fd's readable callback.
  void Drain(const std::function<void(uint64_t, int32_t)> &fn) {
    for (const auto &[token, res] : held_) {
      fn(token, res);
    }
    held_.clear();
    unsigned head;
    struct io_uring_cqe *cqe = nullptr;
    unsigned count = 0;
    io_uring_for_each_cqe(&ring_, head, cqe) {
      ++count;
      const uint64_t data = io_uring_cqe_get_data64(cqe);
      if (data == kCancelToken || data == kNopToken) {
        continue;
      }
      fn(data, cqe->res);
    }
    if (count > 0) {
      io_uring_cq_advance(&ring_, count);
    }
  }

  // In a forked child: the inherited ring is the parent's (its mappings are
  // shared, not copied), so build a fresh one.  Every wait dies with the
  // old ring; the caller re-arms.
  void Recreate() {
    io_uring_queue_exit(&ring_);
    held_.clear();
    Create();
  }

 private:
  void Create() {
    struct io_uring_params params;
    memset(&params, 0, sizeof(params));
    // A plain ring, deliberately -- not the SINGLE_ISSUER | DEFER_TASKRUN
    // configuration the io_uring backend uses.  This ring is never waited
    // on; its owner sits in epoll_wait() on its fd, and what has to reach
    // that owner is a kick.  A plain ring posts a completion through task
    // work added with TWA_SIGNAL, which interrupts epoll_wait(), runs on
    // the way back to userspace, and leaves the fd readable for the retry.
    // On a DEFER_TASKRUN ring the completion is only posted when the owner
    // enters the ring, and queueing the deferred work does not wake a
    // poller already asleep on the fd -- so an owner blocked in
    // epoll_wait() never learns that there is something to enter for.
    // (Tried; FutexWaitWakesTest hung on every fd-polling backend.)
    // COOP_TASKRUN drops the kick for the same reason.
    params.flags = 0;
    const int ret = io_uring_queue_init_params(
        absl::GetFlag(FLAGS_aio_queue_depth), &ring_, &params);
    ABSL_CHECK_EQ(ret, 0) << ": io_uring_queue_init for futex wakeups failed: "
                          << aos_strerror(-ret)
                          << ".  Futex wakeups need Linux 6.7 or later; use "
                             "--queue_wakeup=signal on older kernels.";
  }

  struct io_uring_sqe *GetSqe() {
    struct io_uring_sqe *sqe = io_uring_get_sqe(&ring_);
    // Every arm submits immediately, so the local SQ holds at most one
    // entry; this cannot fill on any depth that io_uring_setup accepts.
    ABSL_CHECK(sqe != nullptr) << ": futex wait ring out of SQEs";
    return sqe;
  }

  void Submit() {
    const int ret = io_uring_submit(&ring_);
    ABSL_CHECK_GE(ret, 0) << ": io_uring_submit failed: " << aos_strerror(-ret);
  }

  struct io_uring ring_;
  std::vector<std::pair<uint64_t, int32_t>> held_;
};

class RingFutexWaitGroup;

class RingFutexWaiter : public FutexWaiter {
 public:
  RingFutexWaiter(RingFutexWaitGroup *group, std::atomic<uint32_t> *word)
      : FutexWaiter(reinterpret_cast<FutexWaitGroup *>(group), word),
        ring_group_(group) {}
  ~RingFutexWaiter() override = default;

  RingFutexWaitGroup *ring_group() const { return ring_group_; }
  uint64_t token() const { return reinterpret_cast<uint64_t>(this); }

  void DoArm(FutexWaitRing *ring, uint32_t expected) {
    ABSL_CHECK(!armed_) << ": Arming a FutexWaiter that is already armed";
    expected_ = expected;
    armed_ = true;
    ring->Arm(word_, expected_, token());
  }

  // Re-arms with the value it was last armed with, after a ring rebuild.
  void ReArm(FutexWaitRing *ring) {
    if (armed_) {
      ring->Arm(word_, expected_, token());
    }
  }

  // The wait completed.  0: woken.  -EAGAIN: the word already differed.
  // Anything else means the registration cannot work, and dying names it.
  void Completed(int32_t res) {
    if (res != 0 && res != -EAGAIN) {
      ABSL_LOG(FATAL) << "IORING_OP_FUTEX_WAIT failed: " << aos_strerror(-res)
                      << (res == -EINVAL
                              ? " -- the kernel lacks io_uring futex "
                                "operations (Linux 6.7); use "
                                "--queue_wakeup=signal"
                              : "");
    }
    armed_ = false;
  }

  bool is_armed() const { return armed_; }

 private:
  RingFutexWaitGroup *const ring_group_;
};

class RingFutexWaitGroup : public FutexWaitGroup {
 public:
  RingFutexWaitGroup(
      std::function<void()> callback,
      std::function<void(FileDescriptor, std::function<void()>)> on_readable,
      std::function<void(FileDescriptor)> delete_fd)
      : FutexWaitGroup(std::move(callback)),
        on_readable_(std::move(on_readable)),
        delete_fd_(std::move(delete_fd)) {
    Watch();
  }

  ~RingFutexWaitGroup() override {
    ABSL_CHECK(waiters_.empty())
        << ": Remove every FutexWaiter before unregistering its group";
    delete_fd_(ring_.fd());
  }

  RingFutexWaiter *Add(std::atomic<uint32_t> *word) {
    auto *waiter = new RingFutexWaiter(this, word);
    waiters_.push_back(waiter);
    return waiter;
  }

  // Legal from inside callback_(), which is what a receiver unregistering
  // itself from its own callback does.  The drain that produced the callback
  // has already finished, so nothing is walking waiters_.
  void Remove(RingFutexWaiter *waiter) {
    auto it = std::find(waiters_.begin(), waiters_.end(), waiter);
    ABSL_CHECK(it != waiters_.end()) << ": FutexWaiter not found";
    waiters_.erase(it);
    if (waiter->is_armed()) {
      ring_.Cancel(waiter->token());
    }
    delete waiter;
  }

  void Arm(RingFutexWaiter *waiter, uint32_t expected) {
    waiter->DoArm(&ring_, expected);
  }

  bool in_callback() const { return in_callback_; }
  FileDescriptor fd() const { return ring_.fd(); }
  void EraseWhenCallbackReturns(std::function<void(RingFutexWaitGroup *)> f) {
    erase_when_callback_returns_ = std::move(f);
  }

  void AfterFork() {
    delete_fd_(ring_.fd());
    ring_.Recreate();
    for (RingFutexWaiter *waiter : waiters_) {
      waiter->ReArm(&ring_);
    }
    Watch();
  }

 private:
  void Watch() {
    on_readable_(ring_.fd(), [this]() { OnReadable(); });
  }

  // The fd reported readable: drain, then one callback if anything
  // completed.  Every completed waiter is disarmed before the callback, so
  // it re-arms whatever it finds disarmed; a completion that lands after
  // the drain makes the fd readable again and produces another callback on
  // a later turn of the loop.
  void OnReadable() {
    bool fired = false;
    ring_.Drain([&](uint64_t token, int32_t res) {
      // Cancel() has already seen the terminal completion of every removed
      // waiter, so a token here names a live one.
      reinterpret_cast<RingFutexWaiter *>(token)->Completed(res);
      fired = true;
    });
    if (fired) {
      in_callback_ = true;
      callback_();
      in_callback_ = false;
    }
    if (erase_when_callback_returns_ != nullptr) {
      // callback_() unregistered this group.  Erasing it there would have
      // freed the std::function that was running; doing it here, with the
      // callback returned and nothing below touching this, is the fd-polling
      // equivalent of the io_uring backend's end-of-dispatch sweep.  Deletes
      // this, so nothing may follow.
      erase_when_callback_returns_(this);
      return;
    }
  }

  const std::function<void(FileDescriptor, std::function<void()>)> on_readable_;
  const std::function<void(FileDescriptor)> delete_fd_;
  FutexWaitRing ring_;
  std::vector<RingFutexWaiter *> waiters_;
  bool in_callback_ = false;
  // Set by RingFutexWaitGroups::Unregister() when it is called from inside
  // callback_(); OnReadable() runs it once the callback has returned.
  std::function<void(RingFutexWaitGroup *)> erase_when_callback_returns_;
};

}  // namespace

class FutexWaitGroups::Impl {
 public:
  Impl(std::function<void(FileDescriptor, std::function<void()>)> on_readable,
       std::function<void(FileDescriptor)> delete_fd)
      : on_readable_(std::move(on_readable)),
        delete_fd_(std::move(delete_fd)) {}

  ~Impl() {
    ABSL_CHECK(groups_.empty())
        << ": Every FutexWaitGroup must be unregistered before destroying "
           "the Aio";
  }

  FutexWaitGroup *Register(std::function<void()> callback) {
    // Allocates: the group, its ring, and the std::functions.
    aos::CheckNotRealtime();
    groups_.push_back(std::make_unique<RingFutexWaitGroup>(
        std::move(callback), on_readable_, delete_fd_));
    return groups_.back().get();
  }

  void Unregister(FutexWaitGroup *group) {
    aos::CheckNotRealtime();
    auto it = std::find_if(groups_.begin(), groups_.end(),
                           [group](const auto &g) { return g.get() == group; });
    ABSL_CHECK(it != groups_.end()) << ": FutexWaitGroup not found";
    auto *ring_group = static_cast<RingFutexWaitGroup *>(group);
    if (ring_group->in_callback()) {
      ring_group->EraseWhenCallbackReturns(
          [this](RingFutexWaitGroup *g) { Erase(g); });
      return;
    }
    groups_.erase(it);
  }

  FutexWaiter *Add(FutexWaitGroup *group, std::atomic<uint32_t> *word) {
    aos::CheckNotRealtime();
    return static_cast<RingFutexWaitGroup *>(group)->Add(word);
  }

  void Remove(FutexWaiter *waiter) {
    aos::CheckNotRealtime();
    auto *ring_waiter = static_cast<RingFutexWaiter *>(waiter);
    ring_waiter->ring_group()->Remove(ring_waiter);
  }

  void Arm(FutexWaiter *waiter, uint32_t expected) {
    auto *ring_waiter = static_cast<RingFutexWaiter *>(waiter);
    ring_waiter->ring_group()->Arm(ring_waiter, expected);
  }

  void AfterFork() {
    for (const std::unique_ptr<RingFutexWaitGroup> &group : groups_) {
      group->AfterFork();
    }
  }

  bool empty() const { return groups_.empty(); }

  bool OwnsFd(FileDescriptor fd) const {
    for (const std::unique_ptr<RingFutexWaitGroup> &group : groups_) {
      if (group->fd() == fd) {
        return true;
      }
    }
    return false;
  }

  void ForEachFd(const std::function<void(FileDescriptor)> &fn) const {
    for (const std::unique_ptr<RingFutexWaitGroup> &group : groups_) {
      fn(group->fd());
    }
  }

 private:
  void Erase(RingFutexWaitGroup *group) {
    auto it = std::find_if(groups_.begin(), groups_.end(),
                           [group](const auto &g) { return g.get() == group; });
    ABSL_CHECK(it != groups_.end()) << ": deferred FutexWaitGroup lost";
    groups_.erase(it);
  }

 public:
 private:
  const std::function<void(FileDescriptor, std::function<void()>)> on_readable_;
  const std::function<void(FileDescriptor)> delete_fd_;
  std::vector<std::unique_ptr<RingFutexWaitGroup>> groups_;
};

FutexWaitGroups::FutexWaitGroups(
    std::function<void(FileDescriptor, std::function<void()>)> on_readable,
    std::function<void(FileDescriptor)> delete_fd)
    : impl_(std::make_unique<Impl>(std::move(on_readable),
                                   std::move(delete_fd))) {}

FutexWaitGroups::~FutexWaitGroups() = default;

FutexWaitGroup *FutexWaitGroups::Register(std::function<void()> callback) {
  return impl_->Register(std::move(callback));
}

void FutexWaitGroups::Unregister(FutexWaitGroup *group) {
  impl_->Unregister(group);
}

FutexWaiter *FutexWaitGroups::Add(FutexWaitGroup *group,
                                  std::atomic<uint32_t> *word) {
  return impl_->Add(group, word);
}

void FutexWaitGroups::Remove(FutexWaiter *waiter) { impl_->Remove(waiter); }

void FutexWaitGroups::Arm(FutexWaiter *waiter, uint32_t expected) {
  impl_->Arm(waiter, expected);
}

void FutexWaitGroups::AfterFork() { impl_->AfterFork(); }

bool FutexWaitGroups::empty() const { return impl_->empty(); }

bool FutexWaitGroups::OwnsFd(FileDescriptor fd) const {
  return impl_->OwnsFd(fd);
}

void FutexWaitGroups::ForEachFd(
    const std::function<void(FileDescriptor)> &fn) const {
  impl_->ForEachFd(fn);
}

}  // namespace aos::internal

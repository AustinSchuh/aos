#ifndef AOS_EVENTS_FUTEX_WAIT_RING_H_
#define AOS_EVENTS_FUTEX_WAIT_RING_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

#include "aos/events/aio.h"
#include "aos/events/aio_internal.h"
#include "aos/events/file_descriptor.h"

namespace aos::internal {

// The futex wait groups of a backend that waits on file descriptors -- epoll,
// and the libuv guest loop.  The io_uring backend needs none of this; its
// waits ride its own ring.
//
// Each group owns a minimal io_uring -- SINGLE_ISSUER | DEFER_TASKRUN, created
// on the loop thread when the group is registered -- holding nothing but that
// group's IORING_OP_FUTEX_WAIT operations, and hands the backend the ring's
// fd to watch through `on_readable`.  The fd is readable when a completion is
// available or, on a DEFER_TASKRUN ring, when the task work that will produce
// one is pending; the readable callback runs that work, drains every
// completion into disarmed waiters, and invokes the group's callback once.
// That is one syscall per wakeup more than the io_uring backend pays, and it
// is the price of a descriptor.
//
// `on_readable` and `delete_fd` are the backend's own legacy-fd registration,
// which is what makes the ring fd just another descriptor to it.  Both hooks
// are invoked from the loop thread only.
//
// Only Linux (kernel 6.7 and later) has an implementation; everywhere else
// futex_wait_ring_unsupported.cc supplies just enough for a backend to hold
// and destroy one without ever building one.  That is why this is a concrete
// class over a pointer rather than an interface: the platforms that cannot
// implement it still have to name the type.
class FutexWaitGroups {
 public:
  FutexWaitGroups(
      std::function<void(FileDescriptor, std::function<void()>)> on_readable,
      std::function<void(FileDescriptor)> delete_fd);
  ~FutexWaitGroups();

  FutexWaitGroups(const FutexWaitGroups &) = delete;
  FutexWaitGroups &operator=(const FutexWaitGroups &) = delete;

  FutexWaitGroup *Register(std::function<void()> callback);
  void Unregister(FutexWaitGroup *group);
  FutexWaiter *Add(FutexWaitGroup *group, std::atomic<uint32_t> *word);
  void Remove(FutexWaiter *waiter);
  // Arms a one-shot wait for *word != expected.  Arming an armed waiter is a
  // CHECK.  RT-safe.
  void Arm(FutexWaiter *waiter, uint32_t expected);
  // In a forked child: rebuild the kernel state every group holds and re-arm
  // every armed waiter.  The words are shared memory and need nothing.
  void AfterFork();
  bool empty() const;
  // Whether fd is one of the groups' own kernel objects, which the backend
  // registers like any other fd but the caller never did.
  bool OwnsFd(FileDescriptor fd) const;
  // Calls fn with each group's own kernel object.  Fork-time checks only.
  void ForEachFd(const std::function<void(FileDescriptor)> &fn) const;

 private:
  class Impl;
  const std::unique_ptr<Impl> impl_;
};

}  // namespace aos::internal

#endif  // AOS_EVENTS_FUTEX_WAIT_RING_H_

// The platforms with no futex to wait on, and so no wait ring to hold.
//
// A backend that polls descriptors holds its FutexWaitGroups for its whole
// life (UvAio::futex_groups_), so every platform has to be able to name the
// type and destroy one even where nothing can ever build one.  That is all
// this file is: the definitions that link, and a message for anyone who gets
// here anyway.
//
// Nothing reaches these.  ThreadSignalReceiver's constructor refuses the
// futex kind off Linux, QueueEventLoop never asks for one, and the backends
// check before they build the groups -- so each of these is the fourth line
// of defense, not the first.

#include "absl/log/absl_log.h"

#include "aos/events/futex_wait_ring.h"

namespace aos::internal {

// Empty rather than absent: ~FutexWaitGroups() needs it complete.
class FutexWaitGroups::Impl {};

FutexWaitGroups::FutexWaitGroups(
    std::function<void(FileDescriptor, std::function<void()>)> /*on_readable*/,
    std::function<void(FileDescriptor)> /*delete_fd*/) {
  ABSL_LOG(FATAL) << "Futex wakeups need Linux 6.7 or later; this platform "
                     "has no futex to wait on.  Use --queue_wakeup=signal.";
}

FutexWaitGroups::~FutexWaitGroups() = default;

FutexWaitGroup *FutexWaitGroups::Register(std::function<void()> /*callback*/) {
  ABSL_LOG(FATAL) << "No futex wait groups on this platform.";
}

void FutexWaitGroups::Unregister(FutexWaitGroup * /*group*/) {
  ABSL_LOG(FATAL) << "No futex wait groups on this platform.";
}

FutexWaiter *FutexWaitGroups::Add(FutexWaitGroup * /*group*/,
                                  std::atomic<uint32_t> * /*word*/) {
  ABSL_LOG(FATAL) << "No futex wait groups on this platform.";
}

void FutexWaitGroups::Remove(FutexWaiter * /*waiter*/) {
  ABSL_LOG(FATAL) << "No futex wait groups on this platform.";
}

void FutexWaitGroups::Arm(FutexWaiter * /*waiter*/, uint32_t /*expected*/) {
  ABSL_LOG(FATAL) << "No futex wait groups on this platform.";
}

void FutexWaitGroups::AfterFork() {
  ABSL_LOG(FATAL) << "No futex wait groups on this platform.";
}

bool FutexWaitGroups::empty() const { return true; }

bool FutexWaitGroups::OwnsFd(FileDescriptor /*fd*/) const { return false; }

void FutexWaitGroups::ForEachFd(
    const std::function<void(FileDescriptor)> & /*fn*/) const {}

}  // namespace aos::internal

#include "aos/ipc_lib/thread_signal.h"

#include "absl/log/absl_check.h"

namespace aos::ipc_lib::internal {
namespace {

// The receiver bound to this thread, or nullptr.  Not in the header: a
// thread_local there is a definition every translation unit carries, and
// nothing outside these two functions has any business reading it.
thread_local const void *bound_receiver = nullptr;

}  // namespace

void BindReceiverToThread(const void *receiver) {
  // One at a time.  Two on one thread are broken on every platform, just
  // differently -- on Windows they share an Event, so one signal wakes one of
  // them and starves the other; on Linux they are two signalfds carrying the
  // same mask, either of which can dequeue a thread-directed signal, and the
  // second destructor unblocks kWakeupSignal while the first still needs it
  // blocked.  Cheaper to refuse than to make either case work.
  ABSL_CHECK(bound_receiver == nullptr || bound_receiver == receiver)
      << "Another ThreadSignalReceiver is already bound to this thread; only "
         "one may be bound at a time (see BindToCurrentThread)";
  bound_receiver = receiver;
}

void UnbindReceiverFromThread(const void *receiver) {
  // Safe to call unbound, and safe to call from a receiver that never bound:
  // UnbindFromCurrentThread() is documented that way.
  if (bound_receiver == receiver) {
    bound_receiver = nullptr;
  }
}

}  // namespace aos::ipc_lib::internal

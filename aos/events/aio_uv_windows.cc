// The Windows half of the libuv backend; see aio_uv_internal.h for the seam.
//
// libuv can poll a socket here and nothing else -- "on windows only sockets
// can be polled with poll handles" -- and neither a deadline nor a thread
// wakeup is a socket.  What the loop *is*, though, is an I/O completion port,
// and a wait completion packet (see aio_windows_internal.h) can hand any
// waitable object's signal to any port, this borrowed one included.
//
// A packet with no OVERLAPPED wakes the loop and nothing more: libuv ends the
// wait and carries on.  That is a path libuv relies on itself, in
// uv__wake_all_loops(), so it is not a trick played on it.
//
// Because the wake says nothing about what caused it, a uv_check_t -- which
// libuv runs after every wait -- asks each thing that could have: the kernel
// timer behind AOS's timer queue, and each waitable handle registered through
// the legacy API.  Those handles are a glib GPollFD and the
// ThreadSignalReceiver's Event, which are handles rather than descriptors
// because Windows has no signalfd.
//
// Sockets are left to libuv, which polls them itself.  Everything here is the
// same use of a completion port the IOCP backend makes, on a port this
// backend borrows instead of owning.

#include <uv.h>

#include <cerrno>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/events/aio_uv_internal.h"
#include "aos/events/aio_windows_internal.h"
#include "aos/events/timer_queue.h"
#include "aos/ipc_lib/thread_signal.h"

namespace aos::uv_internal {

// Raw I/O on a socket, with the errno every other platform would report;
// see TranslateWinsockError().
int UvPlatform::Read(FileDescriptor fd, char *ptr, size_t size) {
  return recv(ToSocket(fd), ptr, static_cast<int>(size), 0);
}

int UvPlatform::Write(FileDescriptor fd, const char *ptr, size_t size) {
  return send(ToSocket(fd), ptr, static_cast<int>(size), 0);
}

int UvPlatform::LastIoError() {
  return TranslateWinsockError(WSAGetLastError());
}

bool UvPlatform::IsInvalidFd(FileDescriptor fd) {
  return ToSocket(fd) == INVALID_SOCKET;
}

int UvPlatform::PollInit(uv_loop_t *loop, uv_poll_t *poll, FileDescriptor fd) {
  return uv_poll_init_socket(loop, poll, ToSocket(fd));
}

void UvPlatform::Close(FileDescriptor fd) { closesocket(ToSocket(fd)); }

// A peer going away abruptly is not something libuv will say here unless it
// is also watching for data.  Its poll asks AFD for the events it was given,
// and a subscription to UV_DISCONNECT alone becomes AFD_POLL_DISCONNECT
// alone -- the graceful close.  AFD_POLL_ABORT, the reset, is only asked for
// alongside UV_READABLE (uv__fast_poll_submit_poll_req() in libuv's
// src/win/poll.c), and then reported as readable.  So a registration that
// wants errors is watched for readability too, and DispatchReadiness()
// below peeks to tell data, EOF and error apart -- the same peek the IOCP
// backend makes on its zero-byte read.  Measured: with this an OnError()-only
// registration sees a peer's abortive reset; without it, nothing arrives.
// AbortiveResetReachesAnErrorOnlyRegistration pins that.
int UvPlatform::AdjustWatch(uint32_t subscribed, int wanted) {
  if ((subscribed & kErr) != 0) {
    wanted |= UV_READABLE;
  }
  return wanted;
}

namespace {

class WindowsPlatform;

// A timer on this platform is a node in the platform's TimerQueue -- the
// container the IOCP and kqueue backends use, with the same
// (deadline, sequence) order -- and one high-resolution kernel timer, armed
// for the head of that queue, is what wakes the loop.  Same reason as the
// POSIX half's descriptors: uv_timer_t is milliseconds, and AOS schedules in
// nanoseconds.
class WindowsTimerState : public Aio::TimerState {
 public:
  explicit WindowsTimerState(WindowsPlatform *platform) : platform_(platform) {}
  ~WindowsTimerState() override;

  void Initialize() override { request.done = true; }
  void Schedule(aos::monotonic_clock::time_point deadline,
                CompletionCallback callback, void *context) override;
  void Cancel(bool reap) override;

  // Already out of the queue when this is called.
  void HandleExpiration() {
    if (user_callback == nullptr) {
      return;
    }
    CompletionCallback callback = user_callback;
    void *const context = user_context;
    user_callback = nullptr;
    request.done = true;
    callback(
        Completion{.status = aos::Status{}, .result = 0, .user_data = nullptr},
        context);
  }

  // The queue's tree node; see TimerQueueTraits.  The deadline it is keyed on
  // is TimerState::deadline.
  WindowsTimerState *left = nullptr;
  WindowsTimerState *right = nullptr;
  WindowsTimerState *parent = nullptr;
  bool red = false;
  uint64_t sequence = 0;
  // Whether this timer is in the queue -- the tree cannot answer that, and
  // Remove() on a node that is not in it is not safe.
  bool queued = false;

 private:
  WindowsPlatform *const platform_;
};

struct TimerQueueTraits {
  static WindowsTimerState *&left(WindowsTimerState *node) {
    return node->left;
  }
  static WindowsTimerState *&right(WindowsTimerState *node) {
    return node->right;
  }
  static WindowsTimerState *&parent(WindowsTimerState *node) {
    return node->parent;
  }
  static bool &red(WindowsTimerState *node) { return node->red; }
  static uint64_t &sequence(WindowsTimerState *node) { return node->sequence; }
  static aos::monotonic_clock::time_point deadline(
      const WindowsTimerState *node) {
    return node->deadline;
  }
};

// A legacy registration on a waitable HANDLE, watched through a packet on
// the loop's port.  Same semantics as the IOCP backend's HandleWatch:
// one-shot, reporting the subscribed mask, re-armed only after the callback
// has had its chance to consume the handle.
// Owned by the list it is on: Adopt() news one onto
// WindowsPlatform::watches_, and SweepReleasedWatches() is the only thing
// that takes one off and deletes it.  ~WindowsPlatform() CHECKs the list
// empty after a final sweep, so a watch that never got released is a loud
// failure rather than a leak.
struct HandleWatch {
  explicit HandleWatch(FdRegistration *reg_in) : reg(reg_in) {}
  FdRegistration *const reg;
  WaitPacket packet;

  // Links for WindowsPlatform::watches_.
  HandleWatch *next = nullptr;
  HandleWatch *prev = nullptr;
  // Set by Release() and swept by OnCheck().  A watch is unlinked only
  // between passes, never during one, so a pass can hold a next pointer
  // across a callback -- see OnCheck().
  bool released = false;
};

class WindowsPlatform : public UvPlatform {
 public:
  explicit WindowsPlatform(UvCore *core) : core_(core) {
    // Runs after every wait; see OnCheck().  Heap allocated and freed from
    // its own close callback, like every handle the core puts on the loop,
    // because that callback runs on a later turn of a loop this object does
    // not drive.  Started for good, and referenced only while it has
    // something to wait for (UpdateCheckRef()), so AOS with nothing
    // scheduled keeps the owner's loop alive no more here than elsewhere.
    check_ = new uv_check_t;
    ABSL_CHECK_EQ(uv_check_init(core_->loop(), check_), 0);
    check_->data = this;
    ABSL_CHECK_EQ(uv_check_start(
                      check_,
                      [](uv_check_t *handle) {
                        static_cast<WindowsPlatform *>(handle->data)->OnCheck();
                      }),
                  0);
    uv_unref(reinterpret_cast<uv_handle_t *>(check_));
  }

  ~WindowsPlatform() override {
    // The core has deleted every registration by now, so nothing of AOS's is
    // on the stack and what Release() parked can go.  Release() only marks;
    // OnCheck() is what unlinks, and the loop need not have turned since the
    // last one went away, so sweep here as well.  Whatever is left after that
    // is a registration the core never released, which is the bug the CHECK
    // is for.  The timers were CHECKed gone before that.
    SweepReleasedWatches();
    ABSL_CHECK(timers_.empty());
    ABSL_CHECK(watches_.empty());
    retired_.clear();
    deadline_timer_.Disarm();
    uv_check_stop(check_);
    uv_close(reinterpret_cast<uv_handle_t *>(check_), [](uv_handle_t *handle) {
      delete reinterpret_cast<uv_check_t *>(handle);
    });
  }

  bool Adopt(FdRegistration *reg) override {
    if (IsSocket(reg->fd)) {
      return false;
    }
    HandleWatch *const watch = new HandleWatch(reg);
    reg->platform_state = watch;
    watches_.PushBack(watch);
    return true;
  }

  void Update(FdRegistration *reg) override {
    HandleWatch *const watch = WatchFor(reg);
    // A waitable object has no bits to subscribe to: it is signalled or it
    // is not, and OnCheck() reports the whole subscribed mask when it is.  So
    // the only question is whether to watch at all.
    if (SubscribedEvents(reg) == 0) {
      watch->packet.Disarm();
    } else {
      Arm(watch);
    }
    UpdateCheckRef();
  }

  void Release(std::unique_ptr<FdRegistration> reg) override {
    HandleWatch *const watch = WatchFor(reg.get());
    // The packet goes now, a queued signal with it.  The watch stays on the
    // list until OnCheck() sweeps it, because Release() can be reached from
    // inside a callback OnCheck() is in the middle of dispatching, and a pass
    // that unlinked nodes under itself could not hold a next pointer.  The
    // registration waits with it, so nothing of AOS's is freed with its own
    // frame still on the stack.
    watch->packet.Disarm();
    watch->released = true;
    reg->platform_state = nullptr;
    retired_.push_back(std::move(reg));
    UpdateCheckRef();
  }

  void Quit() override {
    // Quiet the same way the core's polls go quiet: nothing wakes the loop
    // for AOS any more, and the check that would dispatch is stopped.
    for (HandleWatch *watch = watches_.front(); watch != nullptr;
         watch = watch->next) {
      watch->packet.Disarm();
    }
    deadline_timer_.Disarm();
    uv_check_stop(check_);
  }

  // libuv's report is read before it is believed, for two reasons that are
  // both this platform's.
  //
  // It is a snapshot.  libuv re-submits its poll the moment it reports, and
  // on a socket that is still readable the new poll completes at once and
  // waits in the port; if the caller consumes the data before the loop turns
  // again, the next turn delivers readiness that is no longer there.  The
  // IOCP backend has the same shape -- its zero-byte read re-arms the same
  // way -- and answers it by peeking at dispatch, so this does too.
  //
  // And readable is where a peer's departure arrives, graceful or not (see
  // AdjustWatch()), so readable is either data, EOF or an error, and the
  // peek is what says which.  A hangup is reported the way the IOCP backend
  // and kqueue report it: kIn to a registration that reads, so it can
  // observe the zero-length read and unregister, and kErr to one that does
  // not, since for it the condition that matters is that every later send
  // fails.  Writability is asked about the same way, with the same select()
  // the IOCP backend's write watch uses.
  bool DispatchReadiness(FdRegistration *reg, int status,
                         int uv_events) override {
    const SOCKET socket = ToSocket(reg->fd);
    const uint32_t subscribed = SubscribedEvents(reg);
    uint32_t events = 0;
    bool terminal = false;
    if (status < 0) {
      // libuv itself has given up on the socket, and stopped watching it.
      events = kErr;
      terminal = true;
    } else {
      if (uv_events & UV_READABLE) {
        char byte;
        const int peeked = recv(socket, &byte, 1, MSG_PEEK);
        if (peeked > 0) {
          events |= kIn;
        } else if (peeked == 0) {
          events |= (subscribed & kIn) ? kIn : kErr;
          terminal = true;
        } else if (WSAGetLastError() != WSAEWOULDBLOCK) {
          events |= kErr;
          terminal = true;
        }
        // WSAEWOULDBLOCK: the snapshot was stale, and nothing is reported.
      }
      if ((uv_events & UV_WRITABLE) && SocketWritable(socket)) {
        events |= kOut;
      }
      if (uv_events & UV_DISCONNECT) {
        // Fast poll can say this after all, for the sockets it covers.
        events |= kErr;
        terminal = true;
      }
    }
    // Only what the caller asked for, except kErr, which is always reported
    // since the watch may have been armed purely to detect it.
    events &= subscribed | kErr;
    if (reg->async_only) {
      // A raw request has no err_fn; the handler parked for it does the I/O
      // and reports whatever the socket says, error included.
      if (events & kErr) {
        events = (reg->in_fn != nullptr ? kIn : 0) |
                 (reg->out_fn != nullptr ? kOut : 0);
      }
    } else if (reg->events_fn != nullptr && terminal) {
      // OnEvents sees a hangup, as EPOLLHUP does on Linux; the trio does not.
      events |= kErr;
    }
    if (events != 0) {
      core_->DispatchEvents(reg, events);
    }
    return true;
  }

  std::unique_ptr<Aio::TimerState> MakeTimerState() override {
    return std::make_unique<WindowsTimerState>(this);
  }

  // Windows has no signalfd, so the receiver's manual-reset Event stands in
  // for it: a legacy registration on that handle, which Adopt() takes, whose
  // callback consumes before it notifies -- the shape the IOCP backend gives
  // it, and EpollImpl gives the signalfd.  The Event's name carries the
  // thread it serves, and registration is the first moment that thread is
  // known.
  void RegisterReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                        std::function<void()> callback) override {
    receiver->BindToCurrentThread();
    wakeup_fd_ = reinterpret_cast<FileDescriptor>(receiver->event_handle());
    core_->OnReadable(wakeup_fd_, [receiver, callback = std::move(callback)]() {
      receiver->ConsumeWakeup();
      if (callback) callback();
    });
  }

  void UnregisterReceiver(ipc_lib::ThreadSignalReceiver *receiver) override {
    core_->DeleteFd(wakeup_fd_);
    wakeup_fd_ = nullptr;
    receiver->UnbindFromCurrentThread();
  }

  void ConsumeReceiver(ipc_lib::ThreadSignalReceiver *receiver) override {
    receiver->ConsumeWakeup();
  }

  // --- Timers, for WindowsTimerState. ---

  void InsertTimer(WindowsTimerState *timer) {
    Unlink(timer);
    timers_.Insert(timer);
    timer->queued = true;
    ArmDeadline();
    UpdateCheckRef();
  }

  void RemoveTimer(WindowsTimerState *timer) {
    Unlink(timer);
    ArmDeadline();
    UpdateCheckRef();
  }

  void TimerDestroyed() { core_->TimerDestroyed(); }

 private:
  void Unlink(WindowsTimerState *timer) {
    if (!timer->queued) {
      return;
    }
    timers_.Remove(timer);
    timer->queued = false;
  }

  // Keeps the kernel timer pointed at the head of the queue.  A no-op when
  // the head has not moved, so re-arming a repeating timer from its own
  // callback costs nothing when it lands behind another.
  void ArmDeadline() {
    if (timers_.empty()) {
      deadline_timer_.Disarm();
      return;
    }
    deadline_timer_.Arm(core_->loop()->iocp, timers_.front()->deadline,
                        aos::monotonic_clock::now());
  }

  // The check handle keeps the loop alive exactly while AOS has something a
  // wake could be for.  A socket registration is a poll handle and counts on
  // its own; a timer or a watched handle is not, so this stands in for them.
  void UpdateCheckRef() {
    // Sets the level it wants without tracking what it set last time, which
    // is only right because uv_ref()/uv_unref() set and clear a flag on the
    // handle rather than counting: uv__handle_ref() in libuv's uv-common.h
    // returns early when the flag already says what it was asked.  The
    // documentation does not promise that, so it was measured -- two unrefs
    // and one ref leave the loop alive, two refs and one unref leave it not.
    if (timers_.empty() && watches_.empty()) {
      uv_unref(reinterpret_cast<uv_handle_t *>(check_));
    } else {
      uv_ref(reinterpret_cast<uv_handle_t *>(check_));
    }
  }

  // Unlinks and frees everything Release() marked.  The only place a watch
  // leaves the list, so that a pass through it can hold a next pointer across
  // a callback.
  void SweepReleasedWatches() {
    for (HandleWatch *watch = watches_.front(); watch != nullptr;) {
      HandleWatch *const next = watch->next;
      if (watch->released) {
        watches_.Remove(watch);
        delete watch;
      }
      watch = next;
    }
  }

  // Adopt() hung the watch off the registration, so this is a cast rather
  // than a search.
  static HandleWatch *WatchFor(const FdRegistration *reg) {
    HandleWatch *const watch = static_cast<HandleWatch *>(reg->platform_state);
    ABSL_CHECK(watch != nullptr)
        << ": fd " << reg->fd << " is not a watched handle";
    return watch;
  }

  void Arm(HandleWatch *watch) {
    if (!watch->packet.armed()) {
      watch->packet.Arm(core_->loop()->iocp,
                        reinterpret_cast<HANDLE>(watch->reg->fd), /*key=*/0,
                        /*context=*/nullptr);
    }
  }

  // After every wait.  libuv has dequeued whatever woke it -- its own I/O,
  // or one of AOS's bare wakes -- and cannot say which, so this asks the
  // objects themselves.
  void OnCheck() {
    // What Release() parked since the last turn.
    retired_.clear();

    // The kernel timer.  Manual-reset, so it stays signalled after firing
    // until it is set again; that is how a fire is told from a wake that was
    // somebody else's.  Withdrawn rather than marked delivered: the fire may
    // have landed after the wake this check is running for, with its packet
    // still queued in the port, and a queued packet is still associated as
    // far as the kernel is concerned -- re-arming over it is refused.
    // Disarm() takes a queued packet with it and is harmless on a delivered
    // one.  Then every timer that is due, in order, unlinked one at a time
    // rather than swept because a callback may schedule or cancel others;
    // the kernel timer is pointed at whatever is left, once.
    if (deadline_timer_.Fired()) {
      deadline_timer_.Disarm();
    }
    const aos::monotonic_clock::time_point now = aos::monotonic_clock::now();
    while (!timers_.empty() && timers_.front()->deadline <= now) {
      WindowsTimerState *const timer = timers_.front();
      Unlink(timer);
      core_->DidWork();
      ScopedCallbackRealtime callback_realtime(core_);
      timer->HandleExpiration();
    }
    ArmDeadline();

    // The watched handles.  A callback may register or unregister any of
    // them, so the pass holds a next pointer across a dispatch and needs that
    // pointer to stay good: Release() unlinks nothing, it marks, and the
    // sweep below is the only thing that unlinks.  A watch a callback
    // registered goes on the back, and whether this pass reaches it depends
    // on where the pass had got to; either way it has just been armed and
    // has not been signalled, so there is nothing for a check to find.
    for (HandleWatch *watch = watches_.front(); watch != nullptr;) {
      HandleWatch *const next = watch->next;
      if (watch->released) {
        watch = next;
        continue;
      }
      FdRegistration *const reg = watch->reg;
      const uint32_t subscribed = SubscribedEvents(reg);
      const bool signaled =
          subscribed != 0 &&
          WaitForSingleObject(reinterpret_cast<HANDLE>(reg->fd), 0) ==
              WAIT_OBJECT_0;
      if (signaled) {
        // The packet has delivered, or is queued and about to.  Withdrawn
        // either way before the callback runs, so the re-arm below starts
        // from nothing.
        watch->packet.Disarm();
        core_->DidWork();
        core_->DispatchEvents(reg, subscribed);
        // The callback may have released this watch, or any other.  Both are
        // still linked; released says which.
      }
      if (!watch->released) {
        // Re-armed from nothing rather than trusting the armed flag: a packet
        // whose handle was signalled and consumed by its owner between two
        // checks has delivered without anything here seeing it, and the flag
        // would keep saying armed while the loop slept through the next
        // signal.  Disarm() on a delivered packet is harmless, and this is
        // what makes the watch level-triggered for a handle that is set again
        // before the next turn.
        watch->packet.Disarm();
        if (SubscribedEvents(watch->reg) != 0) {
          Arm(watch);
        }
      }
      watch = next;
    }

    // Nothing of AOS's is on the stack any more, so the marked watches can
    // go.
    SweepReleasedWatches();
    UpdateCheckRef();
  }

  UvCore *const core_;
  // The armed timers, and the one kernel timer that wakes the loop for the
  // head of them.  Manual-reset, because the loop dequeues its packet as a
  // bare wake and OnCheck() has to ask the timer itself whether it fired.
  TimerQueue<WindowsTimerState, TimerQueueTraits> timers_;
  DeadlineTimer deadline_timer_{/*key=*/0, /*manual_reset=*/true};
  uv_check_t *check_ = nullptr;
  // The registrations on waitable handles, and the ones Release() has parked
  // for OnCheck() to free.
  struct WatchLinkTraits {
    static HandleWatch *&next(HandleWatch *watch) { return watch->next; }
    static HandleWatch *&prev(HandleWatch *watch) { return watch->prev; }
  };
  IntrusiveDoublyLinkedList<HandleWatch, WatchLinkTraits> watches_;
  std::vector<std::unique_ptr<FdRegistration>> retired_;
  // The receiver's Event handle, as the fd it is registered under.
  FileDescriptor wakeup_fd_ = nullptr;
};

WindowsTimerState::~WindowsTimerState() {
  platform_->RemoveTimer(this);
  platform_->TimerDestroyed();
}

void WindowsTimerState::Schedule(aos::monotonic_clock::time_point deadline,
                                 CompletionCallback callback, void *context) {
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  Cancel(true);

  this->deadline = deadline;
  this->user_callback = callback;
  this->user_context = context;
  this->request.done = false;

  platform_->InsertTimer(this);
}

void WindowsTimerState::Cancel(bool /*reap*/) {
  platform_->RemoveTimer(this);
  request.done = true;
  user_callback = nullptr;
}

}  // namespace

std::unique_ptr<UvPlatform> MakeUvPlatform(UvCore *core) {
  return std::make_unique<WindowsPlatform>(core);
}

}  // namespace aos::uv_internal

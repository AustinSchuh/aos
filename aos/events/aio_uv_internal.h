#ifndef AOS_EVENTS_AIO_UV_INTERNAL_H_
#define AOS_EVENTS_AIO_UV_INTERNAL_H_

#include <uv.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "aos/events/aio.h"
#include "aos/events/aio_internal.h"
#include "aos/ipc_lib/thread_signal.h"

// The seam between the two halves of the libuv backend.
//
// The core, in aio_uv.cc, is everything that does not depend on the platform:
// registrations, readiness dispatch, the AsyncRead()/AsyncWrite() emulation on
// top of it, Quit(), BeforeWait(), and the rules about who may touch a libuv
// handle and when.
//
// The platform half supplies what AOS needs to wait on.  libuv waits on
// descriptors; a deadline and a thread wakeup are not descriptors everywhere,
// so each platform makes them into whatever its libuv can poll.
// aio_uv_posix.cc is that half here.
//
// The core reaches its platform through UvPlatform, the platform reaches back
// through UvCore, and neither includes the other's definition.  Private to
// this package, like aio_internal.h.
namespace aos::uv_internal {

// Backend-local readiness flags, matching the other backends' spelling of the
// same idea.  Aio::SetEvents()/OnEvents() are documented in terms of epoll
// bits, and these are the same values EpollImpl uses.
constexpr uint32_t kIn = 0x01;
constexpr uint32_t kPri = 0x02;
constexpr uint32_t kOut = 0x04;
constexpr uint32_t kErr = 0x08;

class UvCore;

// One libuv poll handle per descriptor, plus whichever callbacks are attached
// to it.  A descriptor is used either through OnReadable/OnWritable/OnError or
// through OnEvents, never both -- the same rule the other backends enforce.
struct FdRegistration {
  uv_poll_t poll;
  UvCore *impl = nullptr;
  // Always set by whoever creates the registration.
  FileDescriptor fd{};

  std::function<void()> in_fn;
  std::function<void()> out_fn;
  std::function<void()> err_fn;
  std::function<void(uint32_t)> events_fn;

  // What OnEvents/SetEvents asked for.
  uint32_t events = 0;
  bool writable_enabled = false;
  // What is currently handed to uv_poll_start, so a no-op update stays a
  // no-op.
  int started_uv_events = 0;
  // Set while the handle is being closed, so nothing re-arms it.
  bool closing = false;
  // Whether the platform, rather than uv_poll, is watching this descriptor;
  // see UvPlatform::Adopt().  `poll` is never initialised for one of these.
  bool platform_watched = false;
  // Whatever the platform hung off this registration in Adopt(), so it can
  // find it again without a lookup.  Opaque here on purpose: what it points
  // at is the platform's business, and the core never touches it.
  void *platform_state = nullptr;
  // Whether the registration closes its descriptor once libuv is done with
  // the handle.  Closing it any earlier would leave libuv polling a stale
  // descriptor.
  bool owns_fd = false;

  // Caller-submitted AsyncRead/AsyncWrite requests, at most one of each per
  // descriptor.  An fd is used either through the legacy handlers or through
  // these, never both -- async_only says which, and is what the mixing checks
  // key off, since the raw paths park their own lambdas in in_fn/out_fn.
  AsyncRequest *read_req = nullptr;
  AsyncRequest *write_req = nullptr;
  bool async_only = false;
};

// The mask a registration has asked to be told about, in AOS's bits.  For a
// descriptor the platform watches itself this is also what it reports: a
// waitable object is signalled or it is not, so the whole subscription is
// the only thing it can say.  Zero means nothing is wanted.
inline uint32_t SubscribedEvents(const FdRegistration *reg) {
  if (reg->events_fn != nullptr) {
    return reg->events;
  }
  uint32_t events = 0;
  if (reg->in_fn != nullptr) events |= kIn;
  if (reg->writable_enabled) events |= kOut;
  if (reg->err_fn != nullptr) events |= kErr;
  return events;
}

// What the platform half may ask of the core.
class UvCore {
 public:
  virtual ~UvCore() = default;

  virtual uv_loop_t *loop() = 0;

  // The legacy registration API, for the descriptors the platform builds
  // itself -- a timer's, or the wakeup's.
  virtual void OnReadable(FileDescriptor fd,
                          std::function<void()> callback) = 0;
  virtual void DeleteFd(FileDescriptor fd) = 0;
  // DeleteFd(), and UvPlatform::Close() the descriptor once libuv has
  // finished with the handle.
  virtual void DeleteAndCloseFd(FileDescriptor fd) = 0;

  // Called by every callback that did user-visible work, so the loop's owner
  // can be told whether a turn did any of AOS's.
  virtual void DidWork() = 0;
  // Called by every platform timer's destructor; the core counts them.
  virtual void TimerDestroyed() = 0;

  // What the realtime malloc check was set to outside this turn of the loop,
  // or nullopt when nothing is holding it off; see UvAio::ScopedLoopTurn.
  virtual std::optional<bool> callback_realtime() const = 0;

  // EPoll's dispatch rules for a registration the platform is watching itself,
  // reporting `events` -- SubscribedEvents(reg), in practice.  Only a platform
  // that classifies its own reports needs this; see DispatchReadiness().
  virtual void DispatchEvents(FdRegistration *reg, uint32_t events) = 0;
};

// Restores the caller's realtime state around one AOS callback, so libuv's
// own allocations stay exempt while the caller's do not.
//
// Nothing happens when the loop's owner is not suspending the check.  The
// state a callback runs under is then the caller's own -- an ShmEventLoop
// that went realtime is still realtime when its handlers run -- and the
// backend has not touched it, so it has nothing to put back.
class ScopedCallbackRealtime {
 public:
  explicit ScopedCallbackRealtime(const UvCore *core);
  ~ScopedCallbackRealtime();

  ScopedCallbackRealtime(const ScopedCallbackRealtime &) = delete;
  ScopedCallbackRealtime &operator=(const ScopedCallbackRealtime &) = delete;

 private:
  const std::optional<bool> prior_;
};

// What differs by platform.  One implementation each, in aio_uv_posix.cc and
// aio_uv_windows.cc.
class UvPlatform {
 public:
  virtual ~UvPlatform() = default;

  // Raw I/O on a descriptor libuv reported ready, in one shape: a count, or
  // -1 with LastIoError() the errno every platform reports.
  static int Read(FileDescriptor fd, char *ptr, size_t size);
  static int Write(FileDescriptor fd, const char *ptr, size_t size);
  static int LastIoError();
  // A value that cannot name a descriptor at all, as opposed to one libuv
  // merely refuses.
  static bool IsInvalidFd(FileDescriptor fd);
  // uv_poll_init() in the spelling this platform's descriptors need.
  static int PollInit(uv_loop_t *loop, uv_poll_t *poll, FileDescriptor fd);
  static void Close(FileDescriptor fd);

  // The last word on what to ask libuv to watch, given what the registration
  // subscribed to (`subscribed`, AOS's bits) and what the core worked out
  // from that (`wanted`, libuv's).  Most platforms return `wanted`; Windows
  // has to watch for more than was asked, because the only thing that says a
  // peer is gone on a socket libuv polls with select() is that it reads as
  // EOF.
  static int AdjustWatch(uint32_t subscribed, int wanted);

  // Offered every readiness report libuv makes, before the core acts on it.
  //
  // Return false to let the core handle the report, which is what every
  // platform but Windows does.  Return true to say this platform handled it
  // instead, having worked out the real events and passed them to
  // UvCore::DispatchEvents().  Windows needs that because libuv's report
  // there can be out of date by the time it arrives, and because it never
  // says a socket hung up.
  virtual bool DispatchReadiness(FdRegistration *reg, int status,
                                 int uv_events) = 0;

  // Offered every new legacy registration before the core hands it to
  // uv_poll.  A platform that can wait on the descriptor some other way --
  // and has to, because libuv cannot -- takes it and returns true, and the
  // core then routes Update() and Release() here instead.  A registration
  // for a raw request is never offered: the emulation needs libuv's own
  // readiness.
  virtual bool Adopt(FdRegistration *reg) = 0;
  // reg's subscription changed; SubscribedEvents(reg) is what it wants now.
  virtual void Update(FdRegistration *reg) = 0;
  // reg is being deleted.  The platform stops watching it and frees it --
  // not now, since this may be running from the very callback reg holds,
  // but on a later turn of the loop, the way a poll handle's close callback
  // does for the core.
  virtual void Release(std::unique_ptr<FdRegistration> reg) = 0;
  // Quit() has reached the loop thread: go quiet, the way the core stops its
  // poll handles.
  virtual void Quit() = 0;

  virtual std::unique_ptr<Aio::TimerState> MakeTimerState() = 0;

  // The thread wakeup, registered on whatever this platform can watch it
  // through.  The core has already checked there is only one.
  virtual void RegisterReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                std::function<void()> callback) = 0;
  virtual void UnregisterReceiver(ipc_lib::ThreadSignalReceiver *receiver) = 0;
  virtual void ConsumeReceiver(ipc_lib::ThreadSignalReceiver *receiver) = 0;
};

std::unique_ptr<UvPlatform> MakeUvPlatform(UvCore *core);

}  // namespace aos::uv_internal

#endif  // AOS_EVENTS_AIO_UV_INTERNAL_H_

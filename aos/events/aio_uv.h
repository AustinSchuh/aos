#ifndef AOS_EVENTS_AIO_UV_H_
#define AOS_EVENTS_AIO_UV_H_

#include <memory>

#include "aos/events/aio.h"

// Forward-declared so that this header, and everything that includes it, stays
// free of <uv.h>.  Only aio_uv.cc needs libuv's definitions.
struct uv_loop_s;

namespace aos {

// An Aio backed by somebody else's libuv loop.
//
// The libuv loop is borrowed, never owned: it is not run, stopped, or closed
// here.  Its owner drives it, and everything AOS registered runs whenever they
// do.  Quit() therefore stops AOS's own work rather than the loop, which would
// not be AOS's to stop.
//
// Only what libuv can wait on is available here.  On Linux that is everything
// AOS needs, because the wakeup an ShmEventLoop watcher uses is a signalfd and
// its timers are timerfds -- both ordinary descriptors.  macOS has neither
// (its ThreadSignalReceiver has no descriptor at all) and libuv on Windows
// cannot poll a descriptor, so this target is Linux-only for now.
// What Quit() should do to the borrowed loop.
enum class UvQuitBehavior {
  // Stop AOS's own handles and leave the loop running.  Right for a guest:
  // the loop has its owner's work on it too, and an unexpected return from
  // their uv_run() is not something AOS can decide for them.
  kLeaveLoopRunning,
  // Also uv_stop() the loop, so their uv_run() returns.  Right when AOS is
  // the only thing on the loop and its Exit() is meant to end the program.
  // The stop is routed through a uv_async_t, so it is safe from any thread --
  // which an ExitHandle needs.
  kStopLoop,
};

class UvAio : public Aio {
 public:
  // The loop must outlive this.
  explicit UvAio(uv_loop_s *loop, UvQuitBehavior quit_behavior =
                                      UvQuitBehavior::kLeaveLoopRunning);
  ~UvAio() override;

  // Whether AOS dispatched anything since this was last called, and clears
  // the flag.  Poll() is how a caller learns that on the backends that own
  // their loop; here the loop's owner runs it, so this is what tells them
  // whether a turn did any of AOS's work -- enough to drain until it has
  // none left before going to sleep on something else.
  bool TakeDidWork();

  // Held by the loop's owner across uv_run(), by a caller that runs the loop
  // under AOS's realtime malloc check.
  //
  // libuv allocates and frees inside uv_run() -- its own handle and request
  // bookkeeping -- and none of that is what the check is there to catch, so
  // it would abort a realtime caller for something it did not do.  This
  // suspends the check for the turn and puts it back around AOS's own
  // callbacks, where what the caller does with its own data is still held to
  // the promise it made.
  class ScopedLoopTurn {
   public:
    explicit ScopedLoopTurn(UvAio *aio);
    ~ScopedLoopTurn();

    ScopedLoopTurn(const ScopedLoopTurn &) = delete;
    ScopedLoopTurn &operator=(const ScopedLoopTurn &) = delete;

   private:
    UvAio *const aio_;
    const bool prior_;
  };
};

}  // namespace aos

#endif  // AOS_EVENTS_AIO_UV_H_

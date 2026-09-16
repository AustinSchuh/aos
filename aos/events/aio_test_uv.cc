// The Aio contract suite, run against the libuv backend.
//
// The bodies are in aio_test_lib.cc, compiled once into
// //aos/events:aio_test_lib, and this is the only file in that binary that
// includes <uv.h>.  Keeping the instantiation separate is what lets a test
// binary with no libuv link the same bodies -- see aio_test_native.cc.

#include <uv.h>

#include <memory>
#include <vector>

#include "absl/log/absl_check.h"
#include "gtest/gtest.h"

#include "aos/events/aio_test_lib.h"
#include "aos/events/aio_uv.h"

namespace aos::testing {
namespace {

// A UvAio cannot be default-constructed: it borrows a loop somebody else owns.
// Here that somebody is the test, so this owns one loop per Aio and hands it
// over.
//
// The loop is declared before the Aio so it outlives it, which is the order
// the borrow requires.
class UvAioDriver : public TestAioDriver {
 public:
  UvAioDriver() {
    ABSL_CHECK_EQ(uv_loop_init(&loop_), 0);
    // kStopLoop because the loop is ours alone here, so Quit() ending the run
    // is what these tests mean by it.
    aio_ = std::make_unique<UvAio>(&loop_, UvQuitBehavior::kStopLoop);
  }

  ~UvAioDriver() override {
    aio_.reset();
    // ~UvAio has asked libuv to close its handles, but a handle is not closed
    // until the loop runs its close callback, and uv_loop_close() refuses
    // while any remain.  Turn the loop over until they are gone.
    while (uv_loop_close(&loop_) == UV_EBUSY) {
      uv_run(&loop_, UV_RUN_NOWAIT);
    }
  }

  Aio *aio() override { return aio_.get(); }

  // UvAio refuses to Poll() or Run(), because driving a loop is the job of
  // whoever owns it and it cannot know what else is on theirs.  Here the owner
  // is this driver, so it can say what the tests mean: turn our loop over.
  //
  // Poll() reports whether it is worth calling again, which for the bodies
  // means "the loop is still usable" -- they stop on the request they are
  // waiting for, not on this.  uv_run()'s own return counts active handles,
  // which goes to zero on the very turn that delivers the last completion, so
  // returning it would report failure for the poll that succeeded.
  //
  // A blocking Poll() on the other backends sleeps until something for AOS
  // happens, and one UV_RUN_ONCE turn need not: while a handle waits on its
  // close callback (a just-destroyed Aio::Timer, say) libuv polls without
  // blocking, and a turn that only ran that callback did none of our work.
  // Reporting it as a dead loop ended the bodies' wait loops before the event
  // they were waiting for arrived, so keep turning while the loop still has
  // something that could deliver one.
  bool Poll(bool block) override {
    UvAio::ScopedLoopTurn turn(aio_.get());
    if (!block) {
      uv_run(&loop_, UV_RUN_NOWAIT);
      return aio_->TakeDidWork();
    }
    while (true) {
      const bool alive = uv_run(&loop_, UV_RUN_ONCE) != 0;
      if (aio_->TakeDidWork()) return true;
      if (!alive) return false;
    }
  }

  // Quit() with kStopLoop uv_stop()s the loop, so uv_run() returns as Run()
  // promises.
  void Run() override {
    UvAio::ScopedLoopTurn turn(aio_.get());
    uv_run(&loop_, UV_RUN_DEFAULT);
  }

 private:
  uv_loop_t loop_;
  std::unique_ptr<UvAio> aio_;
};

std::unique_ptr<TestAioDriver> MakeUv() {
  return std::make_unique<UvAioDriver>();
}

}  // namespace

const AioBackend kUvBackend{.name = "uv",
                            .make = &MakeUv,
                            // A borrowed loop is somebody else's to
                            // drive, to rebuild after fork(), and to
                            // bound the lifetime of.
                            .drives_its_own_loop = false,
                            .survives_fork = false,
                            .checks_teardown = false,
                            .selected_by_flag = false};

INSTANTIATE_TEST_SUITE_P(AioBackends, AioTest, ::testing::Values(kUvBackend),
                         &BackendTestName);

std::vector<AioReceiverParam> ReceiverParams() {
  std::vector<AioReceiverParam> result;
  result.push_back(AioReceiverParam{.backend = kUvBackend,
                                    .kind = ipc_lib::WakeupKind::kSignal});
#if defined(__linux__)
  result.push_back(AioReceiverParam{.backend = kUvBackend,
                                    .kind = ipc_lib::WakeupKind::kFutex});
#endif
  return result;
}

INSTANTIATE_TEST_SUITE_P(AioBackends, AioReceiverTest,
                         ::testing::ValuesIn(ReceiverParams()),
                         &ReceiverTestName);

// io_uring's own contract, which this backend has nothing to do with.
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AioIoUringTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AioIoUringReceiverTest);

}  // namespace aos::testing

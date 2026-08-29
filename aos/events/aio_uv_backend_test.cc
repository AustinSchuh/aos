#include <uv.h>

#include <chrono>
#include <memory>
#include <thread>

#include "absl/log/absl_check.h"
#include "gtest/gtest.h"

#include "aos/events/aio.h"
#include "aos/events/aio_uv.h"
#include "aos/events/pipe.h"
#include "aos/time/time.h"

namespace aos::testing {
namespace chrono = std::chrono;

// UvAio with nothing built on top of it: the loop is libuv's, the work
// registered on it is Aio's, and uv_run() is the only thing turning the crank.
//
// //aos/events:aio_test_uv is what holds this backend to the Aio contract --
// the same aio_test_lib.cc every other backend runs.  What is here is what that
// suite structurally cannot reach.  Its TestAio supplies Run() and Poll(), so
// the refusals have to be tested somewhere nothing hides them; it stops the
// loop on Quit(), where a real host's keeps running; and it has no reason to
// cycle a registration through the state where libuv still owns the handle.
// aio_uv_test.cc covers the backend through an ShmEventLoop, which is how it
// is actually used.
class AioUvBackendTest : public ::testing::Test {
 public:
  AioUvBackendTest() {
    ABSL_CHECK_EQ(uv_loop_init(&uv_loop_), 0);
    aio_ = std::make_unique<UvAio>(&uv_loop_);
  }

  ~AioUvBackendTest() {
    aio_.reset();
    // Destroying the Aio only *starts* closing the handles it put on the loop;
    // libuv runs the close callbacks on a later turn, and turning the loop is
    // its owner's job -- here, ours.  Without this uv_loop_close() refuses,
    // because the loop still owns live handles.  That refusal is what makes
    // this destructor a real check on every test below.
    ABSL_CHECK_EQ(uv_run(&uv_loop_, UV_RUN_DEFAULT), 0);
    ABSL_CHECK_EQ(uv_loop_close(&uv_loop_), 0);
  }

  uv_loop_t *uv_loop() { return &uv_loop_; }
  Aio *aio() { return aio_.get(); }

  // Turns the loop until done, or until a libuv timer gives up on it, so that
  // work which never happens fails an expectation instead of hanging.
  void RunUntil(const bool &done) {
    bool timed_out = false;
    uv_timer_t backstop;
    ABSL_CHECK_EQ(uv_timer_init(&uv_loop_, &backstop), 0);
    backstop.data = &timed_out;
    ABSL_CHECK_EQ(uv_timer_start(
                      &backstop,
                      [](uv_timer_t *handle) {
                        *static_cast<bool *>(handle->data) = true;
                        uv_stop(handle->loop);
                      },
                      5000, 0),
                  0);

    while (!done && !timed_out) {
      uv_run(&uv_loop_, UV_RUN_ONCE);
    }

    uv_close(reinterpret_cast<uv_handle_t *>(&backstop), nullptr);
    // The backstop is a stack handle, so it has to be off the loop before it
    // goes out of scope.
    uv_run(&uv_loop_, UV_RUN_NOWAIT);
  }

 private:
  uv_loop_t uv_loop_;
  std::unique_ptr<UvAio> aio_;
};

// DeleteFd() cannot free the registration when it is called: libuv owns the
// poll handle until its close callback runs, and that is on some later turn of
// a loop this code does not drive.  Cycling a registration through that state
// is what leaks or double-frees if the deferral is gotten wrong, and the
// fixture's uv_loop_close() is what notices.
TEST_F(AioUvBackendTest, DeleteFdDefersFreeingItsHandle) {
  Pipe pipe;

  for (int i = 0; i < 4; ++i) {
    aio()->OnReadable(pipe.read_fd(), []() {});
    aio()->DeleteFd(pipe.read_fd());
    // Nothing is freed until the owner turns the loop.
    uv_run(uv_loop(), UV_RUN_NOWAIT);
  }

  // And the loop still works afterwards.
  Aio::Timer timer(aio());
  bool fired = false;
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *context) { *static_cast<bool *>(context) = true; },
      &fired);

  RunUntil(fired);
  EXPECT_TRUE(fired) << ": the loop stopped working after a DeleteFd cycle";
}

// Quit() from another thread *while the loop is turning on a different one*,
// which aio.h documents as legal: "Quit() is thread-safe and may be called
// from any thread".
//
// This is the shape that made the old implementation wrong, and the reason it
// has to be this shape: Quit() used to call uv_poll_stop() and
// uv_prepare_stop() inline, and every libuv handle operation except
// uv_async_send() belongs to the thread running the loop.  Quitting from a
// second thread that is *not* racing the loop touches the same handles and is
// still a contract violation, but nothing observes it.  Racing a live
// uv_run() is what turns it into a data race on libuv's own watcher queues,
// which is why this test is worth its complexity -- run it under --config=tsan
// and the old code is what TSAN has something to say about.
//
// Its own loop rather than the fixture's, because the loop has to be driven
// from the second thread and kStopLoop is what lets uv_run() return.
TEST(AioUvBackendQuitTest, QuitRacesALiveLoopSafely) {
  uv_loop_t loop;
  ASSERT_EQ(uv_loop_init(&loop), 0);
  {
    UvAio aio(&loop, UvQuitBehavior::kStopLoop);
    Pipe pipe;
    aio.OnReadable(pipe.read_fd(), []() {});

    // Something of the host's to keep the loop genuinely turning, so the
    // Quit() below lands while uv_run() is live rather than idle.
    std::atomic<bool> turning{false};
    uv_timer_t ticker;
    ASSERT_EQ(uv_timer_init(&loop, &ticker), 0);
    ticker.data = &turning;
    ASSERT_EQ(uv_timer_start(
                  &ticker,
                  [](uv_timer_t *handle) {
                    static_cast<std::atomic<bool> *>(handle->data)
                        ->store(true, std::memory_order_relaxed);
                  },
                  1, 1),
              0);

    std::thread runner([&loop]() { uv_run(&loop, UV_RUN_DEFAULT); });
    while (!turning.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(chrono::milliseconds(1));
    }

    aio.Quit();
    runner.join();

    EXPECT_FALSE(aio.should_run());
    uv_close(reinterpret_cast<uv_handle_t *>(&ticker), nullptr);
    aio.DeleteFd(pipe.read_fd());
  }
  // The Aio is gone; finish its closes the way the fixture does.
  ASSERT_EQ(uv_run(&loop, UV_RUN_DEFAULT), 0);
  EXPECT_EQ(uv_loop_close(&loop), 0);
}

// The default behaviour: a guest stops its own work and leaves the loop for
// its owner.  uv_run() returning here is the loop running out of AOS work,
// not AOS having stopped it -- which is why the timer below, owned by the
// test rather than by AOS, still runs afterwards.
TEST_F(AioUvBackendTest, QuitLeavesTheBorrowedLoopUsable) {
  aio()->Quit();
  uv_run(uv_loop(), UV_RUN_ONCE);
  EXPECT_FALSE(aio()->should_run());

  bool host_work_ran = false;
  uv_timer_t host_timer;
  ABSL_CHECK_EQ(uv_timer_init(uv_loop(), &host_timer), 0);
  host_timer.data = &host_work_ran;
  ABSL_CHECK_EQ(
      uv_timer_start(
          &host_timer,
          [](uv_timer_t *handle) { *static_cast<bool *>(handle->data) = true; },
          1, 0),
      0);
  RunUntil(host_work_ran);
  EXPECT_TRUE(host_work_ran) << "Quit() left the owner's loop unusable";

  uv_close(reinterpret_cast<uv_handle_t *>(&host_timer), nullptr);
  uv_run(uv_loop(), UV_RUN_NOWAIT);
}

// Run() and Poll() drive the loop, which a borrowed one is its owner's to do.
// aio_uv_test.cc has the ShmEventLoop-level version of this; the refusal
// itself lives here.
TEST_F(AioUvBackendTest, RunIsRefusedDeathTest) {
  EXPECT_DEATH(aio()->Run(), "borrowed libuv loop");
}

TEST_F(AioUvBackendTest, PollIsRefusedDeathTest) {
  EXPECT_DEATH((void)aio()->Poll(true), "borrowed libuv loop");
}

}  // namespace aos::testing

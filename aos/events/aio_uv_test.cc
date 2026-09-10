#include "aos/events/aio_uv.h"

#include <uv.h>

#include <chrono>
#include <memory>
#include <vector>

#include "absl/log/absl_check.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/test_message_generated.h"
#include "aos/events/test_message_static.h"
#include "aos/ipc_lib/shm_base.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"

namespace aos::testing {

namespace chrono = std::chrono;

const FlatbufferDetachedBuffer<Configuration> &Config() {
  static const FlatbufferDetachedBuffer<Configuration> result =
      configuration::ReadConfig(
          ArtifactPath("aos/events/aio_uv_test_config.json"));
  return result;
}

// An AOS event loop scheduled onto a libuv loop, with libuv doing the driving.
class AioUvTest : public ::testing::Test {
 public:
  AioUvTest() {
    SetShmBase(aos::testing::TestTmpDir());
    ABSL_CHECK_EQ(uv_loop_init(&uv_loop_), 0);
    aio_ = std::make_unique<UvAio>(&uv_loop_);
  }

  ~AioUvTest() {
    event_loop_.reset();
    aio_.reset();
    // Tearing those down only *starts* closing the handles they put on the
    // loop; libuv runs the close callbacks on a later turn, and running the
    // loop is the owner's job -- here, this test's. Without this the loop
    // still owns live handles and uv_loop_close refuses.
    ABSL_CHECK_EQ(uv_run(&uv_loop_, UV_RUN_DEFAULT), 0);
    ABSL_CHECK_EQ(uv_loop_close(&uv_loop_), 0);
  }

  // Builds the event loop on the borrowed Aio.
  ShmEventLoop *MakeEventLoop() {
    event_loop_ =
        std::make_unique<ShmEventLoop>(&Config().message(), aio_.get());
    event_loop_->SkipTimingReport();
    event_loop_->SkipAosLog();
    return event_loop_.get();
  }

  uv_loop_t *uv_loop() { return &uv_loop_; }

  void DestroyEventLoop() { event_loop_.reset(); }

 private:
  uv_loop_t uv_loop_;
  std::unique_ptr<UvAio> aio_;
  std::unique_ptr<ShmEventLoop> event_loop_;
};

// The whole point: an AOS timer firing because libuv's loop is being run.
TEST_F(AioUvTest, AosTimerRunsOnUvLoop) {
  ShmEventLoop *const event_loop = MakeEventLoop();

  int aos_ticks = 0;
  TimerHandler *const timer = event_loop->AddTimer([&]() { ++aos_ticks; });
  event_loop->OnRun([&]() {
    timer->Schedule(event_loop->monotonic_now(), chrono::milliseconds(5));
  });

  // A libuv timer both keeps the loop alive and stops it, so libuv is
  // unambiguously the one in charge.
  uv_timer_t stopper;
  ABSL_CHECK_EQ(uv_timer_init(uv_loop(), &stopper), 0);
  ABSL_CHECK_EQ(uv_timer_start(
                    &stopper,
                    [](uv_timer_t *handle) {
                      uv_stop(handle->loop);
                      uv_close(reinterpret_cast<uv_handle_t *>(handle),
                               nullptr);
                    },
                    200, 0),
                0);

  event_loop->Startup();
  uv_run(uv_loop(), UV_RUN_DEFAULT);
  // uv_stop() returns from uv_run() before the closing pass, so give the loop
  // one more turn to finish with `stopper` while it is still in scope.
  uv_run(uv_loop(), UV_RUN_NOWAIT);
  const Status status = event_loop->Shutdown();

  EXPECT_TRUE(status.has_value());
  // 200 ms of a 5 ms timer is about 40. Half of that still shows the timer was
  // running on something like the right schedule rather than firing once.
  EXPECT_GT(aos_ticks, 20) << ": AOS timers did not run on the libuv loop";
}

// A watcher waking from shared memory, which is the path that goes through
// ThreadSignalReceiver's signalfd rather than a timerfd.
TEST_F(AioUvTest, WatcherWakesOnUvLoop) {
  ShmEventLoop *const event_loop = MakeEventLoop();

  std::vector<int> received;
  event_loop->MakeWatcher("/test", [&](const TestMessage &message) {
    received.push_back(message.value());
  });

  // The sender needs its own event loop -- AOS refuses a sender and a watcher
  // on one channel in one loop -- but it needs no Aio of its own to run on,
  // since sending does not require a running loop.
  ShmEventLoop sender_loop(&Config().message());
  sender_loop.SkipTimingReport();
  sender_loop.SkipAosLog();
  aos::Sender<TestMessageStatic> sender =
      sender_loop.MakeSender<TestMessageStatic>("/test");

  constexpr int kMessages = 10;
  int sent = 0;
  uv_timer_t send_timer;
  ABSL_CHECK_EQ(uv_timer_init(uv_loop(), &send_timer), 0);

  // Driven from libuv so that the sending, the waking, and the receiving are
  // all on the one loop.
  struct SendState {
    aos::Sender<TestMessageStatic> *sender;
    int *sent;
  } send_state{&sender, &sent};
  send_timer.data = &send_state;
  ABSL_CHECK_EQ(uv_timer_start(
                    &send_timer,
                    [](uv_timer_t *handle) {
                      SendState *state = static_cast<SendState *>(handle->data);
                      aos::Sender<TestMessageStatic>::StaticBuilder builder =
                          state->sender->MakeStaticBuilder();
                      builder->set_value(*state->sent);
                      builder.CheckOk(builder.Send());
                      if (++*state->sent == kMessages) {
                        uv_stop(handle->loop);
                        uv_close(reinterpret_cast<uv_handle_t *>(handle),
                                 nullptr);
                      }
                    },
                    5, 5),
                0);

  event_loop->Startup();
  uv_run(uv_loop(), UV_RUN_DEFAULT);
  // One more turn, for the last message's wakeup and to finish closing
  // `send_timer` while it is still in scope.
  uv_run(uv_loop(), UV_RUN_NOWAIT);
  const Status status = event_loop->Shutdown();

  EXPECT_TRUE(status.has_value());
  ASSERT_GE(received.size(), static_cast<size_t>(kMessages - 1))
      << ": watcher did not wake on the libuv loop";
  for (size_t i = 0; i < received.size(); ++i) {
    EXPECT_EQ(received[i], static_cast<int>(i));
  }
}

// Neither Startup() nor Shutdown() is called here.  The event loop brings
// itself up on the first turn of the libuv loop and tears itself down when it
// is destroyed, so embedding it is just constructing it.
TEST_F(AioUvTest, StartsAndStopsItself) {
  int aos_ticks = 0;
  {
    ShmEventLoop *const event_loop = MakeEventLoop();
    TimerHandler *const timer = event_loop->AddTimer([&]() { ++aos_ticks; });
    event_loop->OnRun([&]() {
      timer->Schedule(event_loop->monotonic_now(), chrono::milliseconds(5));
    });

    uv_timer_t stopper;
    ABSL_CHECK_EQ(uv_timer_init(uv_loop(), &stopper), 0);
    ABSL_CHECK_EQ(uv_timer_start(
                      &stopper,
                      [](uv_timer_t *handle) {
                        uv_stop(handle->loop);
                        uv_close(reinterpret_cast<uv_handle_t *>(handle),
                                 nullptr);
                      },
                      100, 0),
                  0);

    uv_run(uv_loop(), UV_RUN_DEFAULT);
    uv_run(uv_loop(), UV_RUN_NOWAIT);
    DestroyEventLoop();
  }

  // OnRun ran and the timer fired, both without anyone calling Startup().
  EXPECT_GT(aos_ticks, 5) << ": the event loop never started itself";
}

// Run() drives the Aio, which a borrowed libuv loop is not this event loop's
// to do.
TEST_F(AioUvTest, RunIsRefusedOnABorrowedAio) {
  ShmEventLoop *const event_loop = MakeEventLoop();
  EXPECT_DEATH((void)event_loop->Run(), "borrowed Aio");
}

}  // namespace aos::testing

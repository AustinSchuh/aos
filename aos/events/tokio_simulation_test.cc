#include "aos/events/tokio_simulation.h"

#include <chrono>

#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/testing/path.h"
#include "aos/testing/ping_pong/ping_generated.h"

extern "C" {

struct PingSeen {
  int32_t value;
  int64_t monotonic_now_ns;
};

// From tokio_simulation_test_lib.rs.
void tokio_simulation_test_watch_pings(aos_tokio_simulation_t *simulation,
                                       aos_event_loop_t *event_loop,
                                       PingSeen *seen);
}

namespace aos::testing {
namespace {

namespace chrono = std::chrono;

class TokioSimulationTest : public ::testing::Test {
 protected:
  TokioSimulationTest()
      : config_(configuration::ReadConfig(
            ArtifactPath("aos/testing/ping_pong/pingpong_config.json"))),
        factory_(&config_.message()) {}

  FlatbufferDetachedBuffer<Configuration> config_;
  SimulatedEventLoopFactory factory_;
};

// A simulation assembled in C++, with a C++ application sending a ping one
// simulated second in, runs a Rust application's task on tokio, which gets
// the ping at exactly that simulated time.
TEST_F(TokioSimulationTest, RunsRustTasksOnSimulatedTime) {
  std::unique_ptr<EventLoop> sender_loop = factory_.MakeEventLoop("sender");
  Sender<examples::Ping> sender =
      sender_loop->MakeSender<examples::Ping>("/test");
  TimerHandler *timer = sender_loop->AddTimer([&]() {
    auto builder = sender.MakeBuilder();
    examples::Ping::Builder ping = builder.MakeBuilder<examples::Ping>();
    ping.add_value(7);
    builder.CheckOk(builder.Send(ping.Finish()));
  });
  // Timing reports keep a simulation going forever; end it after the ping.
  TimerHandler *exit = sender_loop->AddTimer([&]() { factory_.Exit(); });
  sender_loop->OnRun([&]() {
    timer->Schedule(sender_loop->monotonic_now() + chrono::seconds(1));
    exit->Schedule(sender_loop->monotonic_now() + chrono::seconds(2));
  });

  std::unique_ptr<EventLoop> rust_loop = factory_.MakeEventLoop("rust");
  PingSeen seen{.value = -1, .monotonic_now_ns = -1};
  TokioSimulation simulation(&factory_);
  tokio_simulation_test_watch_pings(
      simulation.get(), TokioSimulation::ForRust(rust_loop.get()), &seen);
  simulation.Run();

  EXPECT_EQ(seen.value, 7);
  EXPECT_EQ(seen.monotonic_now_ns,
            (monotonic_clock::epoch() + chrono::seconds(1))
                .time_since_epoch()
                .count());
}

// A simulation destroyed without running cleans up the Rust side's objects.
TEST_F(TokioSimulationTest, DestroysUnrunSimulations) {
  std::unique_ptr<EventLoop> rust_loop = factory_.MakeEventLoop("rust");
  PingSeen seen{.value = -1, .monotonic_now_ns = -1};
  {
    TokioSimulation simulation(&factory_);
    tokio_simulation_test_watch_pings(
        simulation.get(), TokioSimulation::ForRust(rust_loop.get()), &seen);
  }
  factory_.RunFor(chrono::seconds(1));
  EXPECT_EQ(seen.value, -1);
}

}  // namespace
}  // namespace aos::testing

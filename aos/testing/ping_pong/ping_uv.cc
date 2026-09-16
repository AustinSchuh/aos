// ping, hosted on a libuv loop instead of its own.
//
// The same Ping as ping.cc, but the event loop is a guest on a uv_loop_t that
// this program owns and drives, the way an application that already has a
// libuv loop would run AOS.  Everything else -- the config, the channel, the
// ^C handling, the round-trip statistics -- is unchanged, which is what makes
// the two comparable.
#include <uv.h>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"

#include "aos/configuration.h"
#include "aos/events/aio_uv.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/testing/ping_pong/ping_lib.h"

ABSL_FLAG(std::string, config, "pingpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::EventLoop::SetDefaultVersionString("ping_version");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  uv_loop_t uv_loop;
  ABSL_CHECK_EQ(uv_loop_init(&uv_loop), 0);
  {
    // AOS is the only thing on this loop, so its Exit() (which ^C calls)
    // should end uv_run() too.
    aos::UvAio aio(&uv_loop, aos::UvQuitBehavior::kStopLoop);
    aos::ShmEventLoop event_loop(&config.message(), &aio);

    aos::Ping ping(&event_loop);

    event_loop.Startup();
    {
      // libuv's own bookkeeping allocates; only AOS's callbacks are held to
      // the realtime malloc check.
      aos::UvAio::ScopedLoopTurn turn(&aio);
      uv_run(&uv_loop, UV_RUN_DEFAULT);
    }
    ABSL_CHECK(event_loop.Shutdown().has_value());

    ping.LogRoundTripStatistics();
  }
  // Shutdown only starts closing AOS's handles; the close callbacks run on a
  // later turn, and the loop refuses to close while any are live.
  uv_run(&uv_loop, UV_RUN_DEFAULT);
  ABSL_CHECK_EQ(uv_loop_close(&uv_loop), 0);

  return 0;
}

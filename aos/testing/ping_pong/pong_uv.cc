// pong, hosted on a libuv loop instead of its own.  See ping_uv.cc.
#include <uv.h>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"

#include "aos/configuration.h"
#include "aos/events/aio_uv.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/testing/ping_pong/pong_lib.h"

ABSL_FLAG(std::string, config, "pingpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::EventLoop::SetDefaultVersionString("pong_version");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  uv_loop_t uv_loop;
  ABSL_CHECK_EQ(uv_loop_init(&uv_loop), 0);
  {
    aos::UvAio aio(&uv_loop, aos::UvQuitBehavior::kStopLoop);
    aos::ShmEventLoop event_loop(&config.message(), &aio);

    aos::Pong pong(&event_loop);

    event_loop.Startup();
    {
      aos::UvAio::ScopedLoopTurn turn(&aio);
      uv_run(&uv_loop, UV_RUN_DEFAULT);
    }
    ABSL_CHECK(event_loop.Shutdown().has_value());
  }
  uv_run(&uv_loop, UV_RUN_DEFAULT);
  ABSL_CHECK_EQ(uv_loop_close(&uv_loop), 0);

  return 0;
}

// Replays a log with robot_server and web_proxy in it, so the robot's pages,
// /api and web_proxy's data channels can be used against a log on a laptop:
//
//   bazel run //aos/network/robot_server:robot_server_replay -- \
//       --data_dir=/path/to/www /path/to/log
//
// then open http://localhost:8000/.  Plain HTTP: a laptop has no robot
// certificate, and browsers treat localhost as secure anyway.
//
// C++ assembles the replay, as for any other; robot_server, which is Rust,
// adds itself to a TokioSimulation, which runs the replay with tokio as the
// outer loop.  See documentation/adr/0008-tokio-on-the-event-loop-thread.md.

#include <limits>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/events/tokio_simulation.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"

ABSL_FLAG(std::string, data_dir, "www", "The pages to serve.");
ABSL_FLAG(std::string, node, "",
          "The node to replay robot_server on, in a multi-node log.");
ABSL_FLAG(uint16_t, port, 8000, "Port for robot_server's plain HTTP.");
ABSL_FLAG(double, replay_rate, 1.0,
          "1.0 to replay in real time, 0.5 at half speed, -1 as fast as "
          "possible.");
ABSL_DECLARE_FLAG(int32_t, proxy_port);

extern "C" {
// From lib.rs.
bool robot_server_add_to_simulation(aos_tokio_simulation_t *simulation,
                                    aos_event_loop_t *event_loop,
                                    const uint8_t *config, size_t config_size,
                                    const char *data_dir, uint16_t port,
                                    uint16_t web_proxy_port);
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  reader.Register();
  // Keep serving once the log ends.
  reader.set_exit_on_finish(false);
  // As fast as possible, but not infinitely: the servers need turns.
  reader.SetRealtimeReplayRate(absl::GetFlag(FLAGS_replay_rate) < 0
                                   ? std::numeric_limits<double>::max()
                                   : absl::GetFlag(FLAGS_replay_rate));
  aos::SimulatedEventLoopFactory *factory = reader.event_loop_factory();

  const aos::Node *node = nullptr;
  if (absl::GetFlag(FLAGS_node).empty()) {
    ABSL_CHECK(!aos::configuration::MultiNode(reader.configuration()))
        << ": Pass --node for a multi-node log.";
  } else {
    node = aos::configuration::GetNode(reader.configuration(),
                                       absl::GetFlag(FLAGS_node));
    ABSL_CHECK(node != nullptr)
        << ": No node " << absl::GetFlag(FLAGS_node) << " in the log.";
  }

  std::unique_ptr<aos::EventLoop> web_proxy_loop =
      factory->MakeEventLoop("web_proxy", node);
  web_proxy_loop->SkipTimingReport();
  aos::web_proxy::WebProxy web_proxy(web_proxy_loop.get(),
                                     factory->scheduler_aio(),
                                     aos::web_proxy::StoreHistory::kNo, -1);
  web_proxy.SetDataPath(absl::GetFlag(FLAGS_data_dir).c_str());

  std::unique_ptr<aos::EventLoop> server_loop =
      factory->MakeEventLoop("robot_server", node);
  server_loop->SkipTimingReport();
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::CopyFlatBuffer(server_loop->configuration());

  aos::TokioSimulation simulation(factory);
  ABSL_CHECK(robot_server_add_to_simulation(
      simulation.get(), aos::TokioSimulation::ForRust(server_loop.get()),
      config.span().data(), config.span().size(),
      absl::GetFlag(FLAGS_data_dir).c_str(), absl::GetFlag(FLAGS_port),
      absl::GetFlag(FLAGS_proxy_port)))
      << ": robot_server couldn't start";
  ABSL_LOG(INFO) << "Serving http://localhost:" << absl::GetFlag(FLAGS_port)
                 << "/";
  simulation.Run();
}

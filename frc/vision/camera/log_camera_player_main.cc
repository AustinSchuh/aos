// Plays the camera images in a log as live cameras, for a robot without (all
// of) its cameras.  See log_camera_player.h.
//
//   log_camera_player --cameras=0,1,3 /home/pi/sim_logs/image_log-063/
//
// sends the log's /camera0, /camera1 and /camera3 on this robot's, at the
// log's pace and looping, alongside a real camera on /camera2.  Stop the
// camera_readers for the cameras it plays.

#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/strings/numbers.h"

#include "aos/configuration.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/vision/camera/log_camera_player.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of the aos configuration.");
ABSL_FLAG(std::vector<std::string>, cameras,
          std::vector<std::string>({"0", "1", "2", "3"}),
          "The cameras to play: each N plays the log's /cameraN on /cameraN.");
ABSL_FLAG(std::string, log_node, "orin",
          "The node whose cameras to play, in a multi-node log.");
ABSL_FLAG(bool, loop, true,
          "If true, start the log again at its end; otherwise exit.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  std::vector<int> cameras;
  for (const std::string &camera : absl::GetFlag(FLAGS_cameras)) {
    int number;
    ABSL_CHECK(absl::SimpleAtoi(camera, &number))
        << ": --cameras takes camera numbers, not " << camera;
    cameras.push_back(number);
  }

  const std::vector<aos::logger::LogFile> log_files =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  frc::vision::LogCameraPlayer player(
      &event_loop, log_files, absl::GetFlag(FLAGS_log_node), cameras,
      absl::GetFlag(FLAGS_loop), [&event_loop]() { event_loop.Exit(); });
  event_loop.Run();
  return 0;
}

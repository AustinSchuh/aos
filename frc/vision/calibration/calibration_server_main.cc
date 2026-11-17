// calibration_server: calibrates a camera's intrinsics from the vision page.
// robot_server starts it, after stopping the AprilTag detectors, when the page
// starts a calibration, and stops it when the page finishes.

#include "absl/flags/flag.h"
#include "absl/strings/str_split.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "frc/vision/calibration/calibration_server_lib.h"

ABSL_FLAG(std::string, config, "aos_config.json", "The AOS configuration.");
ABSL_FLAG(std::string, node, "orin",
          "The node whose cameras these are, as calibrations name it.");
ABSL_FLAG(std::string, cameras, "0,1,2,3", "The cameras it may calibrate.");
ABSL_FLAG(std::string, calibration_folder, "/home/pi/calibration",
          "Where saved calibrations go.");
ABSL_FLAG(std::string, www_folder, "www/calibration",
          "Where robot_server serves calibration files from.");
ABSL_FLAG(std::string, www_path, "/calibration",
          "The URL path robot_server serves --www_folder at.");
ABSL_FLAG(double, tag_size, 6.5 * 0.0254,
          "The AprilTags' size for the two-tag check, in meters, edge to edge "
          "of their black squares.");
ABSL_FLAG(std::string, board_store, "/home/pi/calibration/boards.json",
          "Where saved boards and the last choices are kept.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  frc::vision::CalibrationServer::Options options{
      .node_name = absl::GetFlag(FLAGS_node),
      .team_number = aos::network::GetTeamNumber(),
      .cameras = {},
      .calibration_folder = absl::GetFlag(FLAGS_calibration_folder),
      .www_folder = absl::GetFlag(FLAGS_www_folder),
      .www_path = absl::GetFlag(FLAGS_www_path),
      .board_store = absl::GetFlag(FLAGS_board_store),
      .period = std::chrono::milliseconds(100),
      .tag_size = absl::GetFlag(FLAGS_tag_size),
  };
  for (std::string_view camera :
       absl::StrSplit(absl::GetFlag(FLAGS_cameras), ',', absl::SkipEmpty())) {
    options.cameras.push_back(std::stoi(std::string(camera)));
  }
  frc::vision::CalibrationServer server(&event_loop, std::move(options));
  event_loop.Run();
  return 0;
}

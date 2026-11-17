// Keeps every camera's exposure and gain; see ExposureAdjuster.

#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/vision/field_side_exposure_adjuster_lib.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");
ABSL_FLAG(uint32_t, red_exposure, 15,
          "Exposure on red, until one is saved, in 100 us units.");
ABSL_FLAG(uint32_t, blue_exposure, 20,
          "Exposure on blue, until one is saved, in 100 us units.");
ABSL_FLAG(std::string, settings_file, "/home/pi/camera_settings.json",
          "Where the exposure and gain the page sets are saved.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());
  frc::vision::ExposureAdjuster adjuster(
      &event_loop, {.settings_file = absl::GetFlag(FLAGS_settings_file),
                    .red_exposure = absl::GetFlag(FLAGS_red_exposure),
                    .blue_exposure = absl::GetFlag(FLAGS_blue_exposure)});
  event_loop.Run();
}

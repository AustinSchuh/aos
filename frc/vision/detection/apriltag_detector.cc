#include <algorithm>
#include <chrono>
#include <string>
#include <string_view>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/status/status.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "frc/orin/gpu_apriltag.h"
#include "frc/vision/calibration/calibration_validator.h"
#include "frc/vision/camera_constants_generated.h"

ABSL_FLAG(std::string, channel, "/camera", "Channel name");
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

ABSL_FLAG(uint32_t, width, 0,
          "Width of the image in pixels. Set to zero to use values from the "
          "constants channel.");
ABSL_FLAG(uint32_t, height, 0,
          "Height of the image in pixels. Set to zero to use values from the "
          "constants channel.");

namespace frc::vision {

const calibration::CameraCalibration *FindCameraCalibration(
    const CameraConstants &calibration_data, std::string_view node_name,
    int camera_number) {
  ABSL_CHECK(calibration_data.has_calibration());
  for (const calibration::CameraCalibration *candidate :
       *calibration_data.calibration()) {
    if (candidate->node_name()->string_view() != node_name ||
        candidate->camera_number() != camera_number) {
      continue;
    }
    return candidate;
  }
  ABSL_LOG(FATAL) << ": Failed to find camera calibration for " << node_name
                  << " and camera number " << camera_number;
}

// Dies unless the detector can undistort every point of a width x height
// image with calibration, or constants waives the calibration.  The build
// validates calibrations at default_camera_stream_settings' size, but --width
// and --height can change it, and the deployed constants can differ.
void CheckCalibration(const CameraConstants &constants,
                      const calibration::CameraCalibration &calibration,
                      int width, int height) {
  const aos::monotonic_clock::time_point start = aos::monotonic_clock::now();
  CalibrationReport report;
  const absl::Status status =
      ValidateCalibration(calibration, width, height, &report);
  const std::string_view camera_id =
      calibration.has_camera_id() ? calibration.camera_id()->string_view()
                                  : std::string_view();
  ABSL_LOG(INFO) << "Validated camera_id \"" << camera_id
                 << "\" calibration_timestamp "
                 << calibration.calibration_timestamp() << " in "
                 << std::chrono::duration<double, std::milli>(
                        aos::monotonic_clock::now() - start)
                        .count()
                 << " ms: " << report;
  if (status.ok()) {
    return;
  }
  const bool waived =
      constants.has_waived_calibrations() &&
      std::any_of(constants.waived_calibrations()->begin(),
                  constants.waived_calibrations()->end(),
                  [&](const CalibrationId *waiver) {
                    return (waiver->has_camera_id()
                                ? waiver->camera_id()->string_view()
                                : std::string_view()) == camera_id &&
                           waiver->calibration_timestamp() ==
                               calibration.calibration_timestamp();
                  });
  ABSL_CHECK(waived) << ": The calibration of camera_id \"" << camera_id
                     << "\" (calibration_timestamp "
                     << calibration.calibration_timestamp()
                     << ") fails validation at " << width << "x" << height
                     << ", so the detector can't reliably undistort all of "
                        "the image:\n"
                     << status.message() << "\nRecalibrate the camera at "
                     << width << "x" << height
                     << ", run it at the size it was calibrated at, or add "
                        "its camera_id and calibration_timestamp to the "
                        "robot's waived_calibrations to run it anyway.";
  ABSL_LOG(WARNING) << "The calibration of camera_id \"" << camera_id
                    << "\" fails validation at " << width << "x" << height
                    << ", and runs anyway because it is in "
                       "waived_calibrations:\n"
                    << status.message();
}

void GpuApriltagDetector() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc::constants::WaitForConstants<CameraConstants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  const frc::constants::ConstantsFetcher<CameraConstants> calibration_data(
      &event_loop);

  ABSL_CHECK_GE(absl::GetFlag(FLAGS_channel).length(), 8u)
      << ": Channel name must be of the form /cameraX*";
  int camera_id = std::stoi(absl::GetFlag(FLAGS_channel).substr(7, 1));
  const frc::vision::calibration::CameraCalibration *calibration =
      FindCameraCalibration(calibration_data.constants(),
                            event_loop.node()->name()->string_view(),
                            camera_id);

  ABSL_CHECK(calibration_data.constants().has_default_camera_stream_settings())
      << ": Must provide camera stream settings for image width/height.";

  const CameraStreamSettings *const stream_settings =
      calibration_data.constants().default_camera_stream_settings();

  const uint32_t width = absl::GetFlag(FLAGS_width) > 0
                             ? absl::GetFlag(FLAGS_width)
                             : stream_settings->image_width();
  const uint32_t height = absl::GetFlag(FLAGS_height) > 0
                              ? absl::GetFlag(FLAGS_height)
                              : stream_settings->image_height();
  CheckCalibration(calibration_data.constants(), *calibration, width, height);

  frc::apriltag::ApriltagDetector detector(
      &event_loop, absl::GetFlag(FLAGS_channel), calibration, width, height);

  detector.PinMemory(&event_loop);

  // TODO(austin): Figure out our core pinning strategy.
  // event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({5}));

  ABSL_LOG(INFO) << "Setting scheduler priority";
  struct sched_param param;
  param.sched_priority = 21;
  ABSL_PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);

  ABSL_LOG(INFO) << "Running event loop";
  // TODO(austin): Pre-warm it...
  event_loop.Run();
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::GpuApriltagDetector();

  return 0;
}

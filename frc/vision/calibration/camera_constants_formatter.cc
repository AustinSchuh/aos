#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"

#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"
#include "frc/vision/calibration/calibration_validator.h"
#include "frc/vision/camera_constants_list_generated.h"

namespace frc::vision {
namespace {

std::string_view StringOrEmpty(const flatbuffers::String *string) {
  return string == nullptr ? std::string_view() : string->string_view();
}

// A calibration's camera_id and calibration_timestamp, which waivers match.
using CalibrationKey = std::pair<std::string_view, int64_t>;

// Validates the calibrations of one entry at its
// default_camera_stream_settings size, and marks the waivers the failing ones
// need.  Logs each problem and returns false if there were any.
bool ValidateEntry(std::string_view robot, const CameraConstants &data,
                   std::map<CalibrationKey, bool> *waiver_needed) {
  const CameraStreamSettings *settings = data.default_camera_stream_settings();
  if (settings == nullptr || settings->image_width() == 0 ||
      settings->image_height() == 0) {
    ABSL_LOG(ERROR) << robot
                    << " has calibrations but no "
                       "default_camera_stream_settings image size to "
                       "validate them at.";
    return false;
  }
  const int width = settings->image_width();
  const int height = settings->image_height();
  bool valid = true;
  for (const calibration::CameraCalibration *calibration :
       *data.calibration()) {
    const CalibrationKey key(StringOrEmpty(calibration->camera_id()),
                             calibration->calibration_timestamp());
    CalibrationReport report;
    const absl::Status status =
        ValidateCalibration(*calibration, width, height, &report);
    const std::string name = absl::StrCat(
        robot, " node \"", StringOrEmpty(calibration->node_name()),
        "\" camera ", calibration->camera_number(), " (camera_id \"", key.first,
        "\", calibration_timestamp ", key.second, ")");
    if (status.ok()) {
      ABSL_VLOG(1) << name << ": " << report;
      continue;
    }
    auto waiver = waiver_needed->find(key);
    if (waiver != waiver_needed->end()) {
      waiver->second = true;
      ABSL_LOG(WARNING) << name << " fails validation at " << width << "x"
                        << height
                        << ", and is accepted only because it is in "
                           "waived_calibrations:\n"
                        << status.message() << "\n"
                        << report;
      continue;
    }
    ABSL_LOG(ERROR)
        << name << " fails validation at its robot's "
        << "default_camera_stream_settings size, " << width << "x" << height
        << ":\n"
        << status.message() << "\n"
        << report << "\n"
        << "The AprilTag detector can't reliably undistort all of the image "
           "with it.  Recalibrate the camera at "
        << width << "x" << height
        << " with //frc/vision:intrinsics_calibration, or fix "
           "default_camera_stream_settings if the camera captures at another "
           "size.  To deploy it anyway, add {\"camera_id\": \""
        << key.first << "\", \"calibration_timestamp\": " << key.second
        << "} to this robot's waived_calibrations in "
           "frc/vision/templates/constants.jinja2.json, with a comment saying "
           "why.";
    valid = false;
  }
  return valid;
}

// Validates every calibration of every entry at the entry's
// default_camera_stream_settings size, which apriltag_detector runs it at
// unless --width and --height say otherwise.  Logs each problem and returns
// false if there were any.  A waived calibration which fails only logs a
// warning, and a waiver which no failing calibration needs is a problem, so
// the waivers stay the list of calibrations known to be bad.
bool ValidateCalibrations(const CameraConstantsList &constants) {
  bool valid = true;
  for (const TeamAndCameraConstants *entry : *constants.constants()) {
    if (!entry->has_data()) {
      continue;
    }
    const CameraConstants &data = *entry->data();
    const std::string robot =
        absl::StrCat("team ", entry->team(), " robot \"",
                     StringOrEmpty(entry->robot_name()), "\"");

    // Whether a failing calibration needs each waiver.
    std::map<CalibrationKey, bool> waiver_needed;
    if (data.has_waived_calibrations()) {
      for (const CalibrationId *waiver : *data.waived_calibrations()) {
        waiver_needed[{StringOrEmpty(waiver->camera_id()),
                       waiver->calibration_timestamp()}] = false;
      }
    }

    if (data.has_calibration() && !ValidateEntry(robot, data, &waiver_needed)) {
      valid = false;
    }

    for (const auto &[key, needed] : waiver_needed) {
      if (!needed) {
        ABSL_LOG(ERROR) << robot << " waives camera_id \"" << key.first
                        << "\" calibration_timestamp " << key.second
                        << ", but none of its calibrations with those fail "
                           "validation.  Remove it from waived_calibrations.";
        valid = false;
      }
    }
  }
  return valid;
}

}  // namespace
}  // namespace frc::vision

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  ABSL_CHECK(argc == 3)
      << ": Expected input and output json files to be passed in.";

  aos::FlatbufferDetachedBuffer<frc::vision::CameraConstantsList> constants =
      aos::JsonFileToFlatbuffer<frc::vision::CameraConstantsList>(argv[1]);

  // Make sure the file is valid json before we output a formatted version.
  ABSL_CHECK(constants.message().constants() != nullptr)
      << ": Failed to parse " << std::string(argv[2]);

  if (!frc::vision::ValidateCalibrations(constants.message())) {
    return 1;
  }

  aos::util::WriteStringToFileOrDie(
      std::string(argv[2]),
      aos::FlatbufferToJson(constants, {.multi_line = true}));

  return 0;
}

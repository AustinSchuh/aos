#ifndef FRC_VISION_FIELD_SIDE_EXPOSURE_ADJUSTER_LIB_H_
#define FRC_VISION_FIELD_SIDE_EXPOSURE_ADJUSTER_LIB_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "aos/events/event_loop.h"
#include "frc/input/joystick_state_generated.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/camera_settings_generated.h"

namespace frc::vision {

// Keeps every camera's exposure and gain: sends them on each /cameraN
// CameraStreamSettings channel, which camera_reader follows, once a second.
//
// One exposure and gain hold for every camera.  The page sets them with a
// CameraStreamSettings on /camera_settings; they're saved to a file, so they
// hold across restarts and deploys, and sent at once.  Until any are saved,
// the exposure is the alliance's, while the driver station says which, and
// otherwise the constants' (default_camera_stream_settings in /constants), as
// is the gain.
//
// While any are saved, the robot's settings are also written whole, the
// constants' with the saved exposure and gain, to www_file, for robot_server
// to serve: to check in as the robot's
// frc/vision/constants/camera_settings_<team>-<robot>.json.  An empty
// CameraStreamSettings on /camera_settings forgets what's saved, going back to
// the constants'.
class ExposureAdjuster {
 public:
  struct Options {
    // Where the settings are saved, as CameraStreamSettings JSON.
    std::string settings_file;
    uint32_t red_exposure;
    uint32_t blue_exposure;
    // Where the robot's whole settings are written while any are saved, for
    // robot_server to serve; empty for nowhere.
    std::string www_file = "";
  };

  ExposureAdjuster(aos::EventLoop *event_loop, Options options);

  // What's saved; nullopt when nothing is.
  std::optional<uint32_t> exposure_100us() const { return exposure_100us_; }
  std::optional<uint32_t> gain() const { return gain_; }

 private:
  void Load();
  void Save();
  void Send();
  // Writes www_file, or removes it when nothing is saved.
  void WriteWwwFile();
  // The constants' default settings, if they've come.
  const CameraStreamSettings *Defaults();

  aos::EventLoop *const event_loop_;
  const Options options_;
  aos::Fetcher<JoystickState> joystick_state_fetcher_;
  aos::Fetcher<CameraConstants> constants_fetcher_;
  // What www_file holds, so it's only written when that changes.
  std::optional<std::string> www_contents_;
  // Every camera's, as the configuration has them: /camera0, /camera1, ...
  std::vector<aos::Sender<CameraStreamSettings>> senders_;
  std::optional<uint32_t> exposure_100us_;
  std::optional<uint32_t> gain_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_FIELD_SIDE_EXPOSURE_ADJUSTER_LIB_H_

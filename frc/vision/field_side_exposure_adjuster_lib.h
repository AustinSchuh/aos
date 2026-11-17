#ifndef FRC_VISION_FIELD_SIDE_EXPOSURE_ADJUSTER_LIB_H_
#define FRC_VISION_FIELD_SIDE_EXPOSURE_ADJUSTER_LIB_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "aos/events/event_loop.h"
#include "frc/input/joystick_state_generated.h"
#include "frc/vision/camera_settings_generated.h"

namespace frc::vision {

// Keeps every camera's exposure and gain: sends them on each /cameraN
// CameraStreamSettings channel, which camera_reader follows, once a second.
//
// One exposure and gain hold for every camera.  The page sets them with a
// CameraStreamSettings on /camera_settings; they're saved to a file, so they
// hold across restarts and deploys, and sent at once.  Until any are saved,
// the exposure is the alliance's, while the driver station says which, and
// the cameras keep the constants' otherwise.
class ExposureAdjuster {
 public:
  struct Options {
    // Where the settings are saved, as CameraStreamSettings JSON.
    std::string settings_file;
    uint32_t red_exposure;
    uint32_t blue_exposure;
  };

  ExposureAdjuster(aos::EventLoop *event_loop, Options options);

  // What's saved; nullopt when nothing is.
  std::optional<uint32_t> exposure_100us() const { return exposure_100us_; }
  std::optional<uint32_t> gain() const { return gain_; }

 private:
  void Load();
  void Save();
  void Send();

  aos::EventLoop *const event_loop_;
  const Options options_;
  aos::Fetcher<JoystickState> joystick_state_fetcher_;
  // Every camera's, as the configuration has them: /camera0, /camera1, ...
  std::vector<aos::Sender<CameraStreamSettings>> senders_;
  std::optional<uint32_t> exposure_100us_;
  std::optional<uint32_t> gain_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_FIELD_SIDE_EXPOSURE_ADJUSTER_LIB_H_

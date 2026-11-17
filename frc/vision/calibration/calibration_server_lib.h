#ifndef FRC_VISION_CALIBRATION_CALIBRATION_SERVER_LIB_H_
#define FRC_VISION_CALIBRATION_CALIBRATION_SERVER_LIB_H_

#include <filesystem>
#include <future>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "opencv2/core.hpp"

#include "ImageAnnotations_generated.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "frc/vision/calibration/calibration_command_generated.h"
#include "frc/vision/calibration/calibration_session_generated.h"
#include "frc/vision/calibration/calibration_status_generated.h"
#include "frc/vision/calibration/capture_gate.h"
#include "frc/vision/calibration/charuco_detector.h"
#include "frc/vision/calibration/intrinsics_solver.h"
#include "frc/vision/calibration/validation.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/util/charuco_board.h"
#include "frc/vision/vision_generated.h"

namespace frc::vision {

// Runs a calibration session from the vision page: takes commands on
// /calibration Command, finds the board in one camera's /cameraN/gray images,
// captures the frames worth keeping, solves, and saves.  Reports on
// /calibration Status at 5 Hz, and sends the whole session on /calibration
// Session after each change, at most once a second.  Draws the corners it
// finds on /cameraN/calibration, for camera_streamer to draw into the video.
class CalibrationServer {
 public:
  struct Options {
    // The node whose cameras these are, as calibrations name it.
    std::string node_name = "orin";
    uint16_t team_number = 0;
    // The cameras it may calibrate, by number.
    std::vector<int> cameras = {0, 1, 2, 3};
    // Where saved calibrations go.
    std::string calibration_folder = "/home/pi/calibration";
    // Where robot_server serves calibration files from, and the URL path it
    // serves that folder at.
    std::string www_folder = "www/calibration";
    std::string www_path = "/calibration";
    // Where saved boards and the last choices are kept.
    std::string board_store = "/home/pi/calibration/boards.json";
    // How often to look for the board.
    std::chrono::milliseconds period = std::chrono::milliseconds(100);
    // The AprilTags' size for TWO_TAG, edge to edge of their black squares.
    double tag_size = 6.5 * 0.0254;
  };

  CalibrationServer(aos::EventLoop *event_loop, Options options);
  ~CalibrationServer();

  // The default target number of captures.
  static constexpr int kDefaultTargetCaptures = 70;

 private:
  struct StoredCapture {
    int64_t monotonic_timestamp_ns;
    CharucoCapture capture;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    // Its image, as a PNG in the session's folder; empty if it couldn't be
    // written.
    std::string image_file;
  };

  // One calibration's checks, over the frames looked at while validating.
  struct LensChecks {
    std::string name;
    Lens lens;
    ValidationGrid straightness_px;
    ValidationGrid rigidity_mm;
    ValidationGrid rigidity_deg;
    TagPairTracker tags;
  };

  struct Solved {
    IntrinsicsSolution solution;
    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> calibration;
    int captures;
    absl::Status validation;
    std::string saved_path;
    std::string download_path;
  };

  void HandleCommand(const calibration::Command &command);
  absl::Status Execute(const calibration::Command &command);
  absl::Status Start(const calibration::Command &command);
  absl::Status UseBoard(const BoardSpec &board);
  absl::Status Save(bool force);
  absl::Status ExportImages();
  // Where this session keeps its captures' images.
  std::filesystem::path SessionFolder() const;

  // Looks at the newest image, and captures it if it's worth keeping.
  void Tick();
  void LookAtImage(const CameraImage &image);
  // Picks up a finished solve.
  void CollectSolve();
  void CapturesChanged();

  absl::Status Validate(const calibration::Command &command);
  void Measure(const cv::Mat &gray, const CharucoDetection &detection);
  bool Checking(calibration::ValidationCheck check) const;
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<calibration::LensValidation>>>
  PackValidation(flatbuffers::FlatBufferBuilder *fbb) const;

  void SendStatus();
  void MaybeSendSession();
  void SendSession();
  flatbuffers::Offset<calibration::Session> PackSession(
      flatbuffers::FlatBufferBuilder *fbb, bool residuals, bool corners) const;

  void LoadStore();
  void WriteStore();
  std::vector<NamedBoard> Boards() const;

  // The starting guess at the intrinsics: the current calibration's, or a 60
  // degree field of view.
  void UpdateGuess();

  aos::EventLoop *const event_loop_;
  const Options options_;

  aos::Sender<calibration::Status> status_sender_;
  aos::Sender<calibration::Session> session_sender_;
  aos::Fetcher<CameraConstants> constants_fetcher_;
  std::map<int, aos::Fetcher<CameraImage>> image_fetchers_;
  std::map<int, aos::Sender<foxglove::ImageAnnotations>> annotation_senders_;
  aos::TimerHandler *timer_;

  // What's kept between sessions.
  std::vector<NamedBoard> saved_boards_;
  std::map<int, BoardSpec> last_boards_;
  int target_captures_ = kDefaultTargetCaptures;

  // The session.
  std::string session_id_;
  calibration::SessionState state_ = calibration::SessionState::IDLE;
  int camera_ = -1;
  std::string camera_id_;
  BoardSpec board_;
  cv::Ptr<cv::aruco::CharucoBoard> cv_board_;
  std::string board_image_path_;
  std::optional<aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>>
      current_;
  // The starting guess at the lens; no camera matrix until there is one.
  CameraLens guess_;
  cv::Size image_size_;
  std::vector<StoredCapture> captures_;
  // Numbers the session's images.
  int next_image_ = 0;
  std::string images_download_path_;
  int images_exported_ = 0;
  CaptureGate gate_;
  bool capture_next_ = false;
  std::optional<Solved> solved_;
  std::optional<std::future<absl::StatusOr<IntrinsicsSolution>>> solving_;
  int solving_captures_ = 0;
  std::string solve_error_;

  // The last frame looked at.
  calibration::FrameVerdict verdict_ = calibration::FrameVerdict::NO_IMAGE;
  int corners_ = 0;
  double moved_ = 0.0;
  double motion_ = 0.0;
  aos::monotonic_clock::time_point last_image_time_ =
      aos::monotonic_clock::min_time;
  int64_t last_image_timestamp_ns_ = -1;

  // Validation: the checks asked for, and each calibration's measurements.
  std::vector<calibration::ValidationCheck> checks_;
  std::vector<LensChecks> validations_;

  std::string error_;
  int ticks_ = 0;
  bool session_dirty_ = true;
  int session_sequence_ = 0;
  aos::monotonic_clock::time_point last_session_sent_ =
      aos::monotonic_clock::min_time;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_CALIBRATION_SERVER_LIB_H_

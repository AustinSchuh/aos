#ifndef FRC_VISION_CALIBRATION_CAPTURE_GATE_H_
#define FRC_VISION_CALIBRATION_CAPTURE_GATE_H_

#include <numbers>
#include <optional>

#include "Eigen/Geometry"

namespace frc::vision {

// Decides which frames of a board to keep for an intrinsics calibration: a
// pose far enough from the last capture to add something, held still enough
// that motion blur doesn't smear the corners.
class CaptureGate {
 public:
  struct Limits {
    // How far the board must move from the last capture, in rotation or
    // translation, to be worth capturing again.
    double move_rotation = std::numbers::pi / 6.0;
    double move_translation = 0.3;
    // How little it may move between consecutive frames to count as still.
    double still_rotation = std::numbers::pi / 60.0;
    double still_translation = 0.01;
  };

  enum class Verdict {
    // Keep this frame.
    kCapture,
    // Too close to the last capture.
    kMoveToNewPose,
    // Far enough, but moving.
    kHoldStill,
  };

  struct Decision {
    Verdict verdict;
    // How far the board has moved from the last capture, as a fraction of
    // what's needed: 1 or more is enough.  1 for the first capture.
    double moved;
    // How much it moved since the last frame, as a fraction of the still
    // limits: under 1 is still.  0 when it hasn't moved far enough to ask.
    double motion;
  };

  CaptureGate() : CaptureGate(Limits{}) {}
  explicit CaptureGate(const Limits &limits) : limits_(limits) {}

  // Judges a frame whose board is at H_camera_board, and remembers it as the
  // last frame.  The first frame judged is always captured.
  Decision Judge(const Eigen::Affine3d &H_camera_board);

  // Records that the board was captured at H_camera_board.
  void Captured(const Eigen::Affine3d &H_camera_board) {
    last_capture_ = H_camera_board;
  }

  // Sets the last capture's pose, as after an undo; nullopt if none is left.
  void set_last_capture(std::optional<Eigen::Affine3d> H_camera_board) {
    last_capture_ = H_camera_board;
  }

 private:
  const Limits limits_;
  std::optional<Eigen::Affine3d> last_capture_;
  std::optional<Eigen::Affine3d> last_frame_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_CAPTURE_GATE_H_

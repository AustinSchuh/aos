#include "frc/vision/calibration/capture_gate.h"

#include <algorithm>
#include <cmath>

namespace frc::vision {
namespace {

// The rotation angle and translation distance from `from` to `to`.
std::pair<double, double> Distance(const Eigen::Affine3d &from,
                                   const Eigen::Affine3d &to) {
  const Eigen::Affine3d delta = to * from.inverse();
  return {std::abs(Eigen::AngleAxisd(delta.rotation()).angle()),
          delta.translation().norm()};
}

}  // namespace

CaptureGate::Decision CaptureGate::Judge(
    const Eigen::Affine3d &H_camera_board) {
  const std::optional<Eigen::Affine3d> last_frame = last_frame_;
  last_frame_ = H_camera_board;
  if (!last_capture_.has_value()) {
    return {.verdict = Verdict::kCapture, .moved = 1.0, .motion = 0.0};
  }

  const auto [rotation, translation] = Distance(*last_capture_, H_camera_board);
  const double moved = std::max(rotation / limits_.move_rotation,
                                translation / limits_.move_translation);
  if (moved <= 1.0) {
    return {.verdict = Verdict::kMoveToNewPose, .moved = moved, .motion = 0.0};
  }

  if (!last_frame.has_value()) {
    // Nothing to tell how fast it's moving yet.
    return {.verdict = Verdict::kHoldStill, .moved = moved, .motion = 1.0};
  }
  const auto [frame_rotation, frame_translation] =
      Distance(*last_frame, H_camera_board);
  const double motion = std::max(frame_rotation / limits_.still_rotation,
                                 frame_translation / limits_.still_translation);
  return {.verdict = frame_rotation < limits_.still_rotation &&
                             frame_translation < limits_.still_translation
                         ? Verdict::kCapture
                         : Verdict::kHoldStill,
          .moved = moved,
          .motion = motion};
}

}  // namespace frc::vision

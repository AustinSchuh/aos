#ifndef FRC_VISION_CALIBRATION_CHARUCO_DETECTOR_H_
#define FRC_VISION_CALIBRATION_CHARUCO_DETECTOR_H_

#include <vector>

#include "opencv2/aruco/charuco.hpp"
#include "opencv2/core.hpp"

#include "frc/vision/util/vision_util_lib.h"

namespace frc::vision {

// Rule of thumb from https://arxiv.org/pdf/1907.04096: 5 constraints for each
// unknown.  A capture adds 6 (the board's pose), so 30 constraints, and each
// corner gives 2.
inline constexpr int kMinCharucoCorners = 15;

struct CharucoDetection {
  // The ArUco markers found.
  int markers = 0;
  // The board's inner corners found, and where, not undistorted: what an
  // intrinsics calibration is made from.
  std::vector<int> ids;
  std::vector<cv::Point2f> corners;
  // The board's pose, board to camera, from the lens, if there were enough
  // corners to find it.
  bool pose_valid = false;
  cv::Vec3d rvec;
  cv::Vec3d tvec;
};

// Finds `board` in `image` (grayscale or BGR).  Interpolates the corners
// twice: without the calibration, for a calibration to use (refined with
// RefineCharucoCorners), and with it, for a better pose.
CharucoDetection DetectCharuco(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                               const cv::Mat &image, const CameraLens &lens,
                               int min_corners = kMinCharucoCorners);

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_CHARUCO_DETECTOR_H_

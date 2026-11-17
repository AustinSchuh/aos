#ifndef FRC_VISION_CALIBRATION_INTRINSICS_SOLVER_H_
#define FRC_VISION_CALIBRATION_INTRINSICS_SOLVER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/core.hpp"

namespace frc::vision {

// One capture of a ChArUco board: the ids of the inner corners found, and
// where they are in the image, not undistorted.
struct CharucoCapture {
  std::vector<int> ids;
  std::vector<cv::Point2f> corners;
};

struct IntrinsicsSolution {
  // 3x3, CV_64F.
  cv::Mat camera_matrix;
  // 5 coefficients, or for the rational model all 14 of OpenCV's (0 past
  // the eighth), CV_64F.
  cv::Mat dist_coeffs;
  // RMS reprojection error over every corner, in pixels.
  double reprojection_error;
  // Each capture's RMS reprojection error, in pixels.
  std::vector<double> per_view_errors;
  // Each capture's board pose: board to camera.
  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  // Each capture's corners' leftover errors, observed less projected, in the
  // order of the capture's ids.
  std::vector<std::vector<cv::Point2f>> residuals;
};

// The largest RMS reprojection error a solve may have and still succeed.
inline constexpr double kMaxReprojectionError = 5.0;

// Solves for a camera's intrinsics from captures of `board` in images of
// `image_size`, starting from `camera_matrix` and `dist_coeffs` (a guess, even
// a rough one, helps it converge; an empty matrix starts from a 60 degree
// field of view).  The model is OpenCV's, so the result is what
// calibrateCameraCharuco would find, but solved with Ceres: it eliminates the
// boards' poses, so 70 captures take well under a second where OpenCV's dense
// solve takes 10 s.  Fails with fewer than 4 captures, or a reprojection error
// over kMaxReprojectionError.
absl::StatusOr<IntrinsicsSolution> SolveIntrinsics(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures, cv::Size image_size,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    bool rational_model);

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_INTRINSICS_SOLVER_H_

#ifndef FRC_VISION_CALIBRATION_INTRINSICS_SOLVER_H_
#define FRC_VISION_CALIBRATION_INTRINSICS_SOLVER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/core.hpp"

#include "frc/vision/util/vision_util_lib.h"

namespace frc::vision {

// How a lens bends light, as a solve models it.
enum class LensModel {
  // OpenCV's: k1, k2, p1, p2, k3.
  kOpenCv5,
  // OpenCV's rational model: k1, k2, p1, p2, k3, k4, k5, k6, the radial
  // part a ratio of polynomials.
  kOpenCvRational,
  // Kannala-Brandt, as OpenCV's fisheye module has it: the distorted angle a
  // polynomial in the angle from the axis, k1..k4.
  kKannalaBrandt,
};

struct SolveOptions {
  LensModel model = LensModel::kOpenCvRational;
  // For the rational model, a prior pulling the denominator's terms, k4..k6,
  // toward 0 with this standard deviation, so they only grow where the data
  // needs them: unconstrained, numerator and denominator trade off, and the
  // fit wanders to coefficients that fold inside the image.  0 for none.
  double denominator_sigma = 0.0;
};

// One capture of a ChArUco board: the ids of the inner corners found, and
// where they are in the image, not undistorted.
struct CharucoCapture {
  std::vector<int> ids;
  std::vector<cv::Point2f> corners;
};

struct IntrinsicsSolution {
  LensModel model;
  // 3x3, CV_64F.
  cv::Mat camera_matrix;
  // As OpenCV has them for the model: 5; for the rational model all 14 (0
  // past the eighth); for Kannala-Brandt 4.  CV_64F.
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
    const SolveOptions &options);

// As above, starting from `guess`, in either model: its distortion starts the
// solve if it's in the model being solved, and its poses always do.
absl::StatusOr<IntrinsicsSolution> SolveIntrinsics(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures, cv::Size image_size,
    const CameraLens &guess, const SolveOptions &options);

// The lens a solution describes, to undistort and project with.
CameraLens SolutionLens(const IntrinsicsSolution &solution);

// As above, with OpenCV's rational model if `rational_model`, else its 5.
absl::StatusOr<IntrinsicsSolution> SolveIntrinsics(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures, cv::Size image_size,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    bool rational_model);

// How well `solution` fits captures it wasn't solved from: each board's pose
// fitted with the intrinsics held, then the RMS of every corner's error, in
// pixels.  What a fit's own error can't show: a model fitting its captures'
// noise.
// Each corner of captures `solution` wasn't solved from, where it is in the
// image, and its error (observed less projected), each board's pose fitted
// with the intrinsics held.
struct CornerResidual {
  cv::Point2f corner;
  cv::Point2f residual;
};
absl::StatusOr<std::vector<CornerResidual>> HeldOutResiduals(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures,
    const IntrinsicsSolution &solution);

absl::StatusOr<double> HeldOutError(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures,
    const IntrinsicsSolution &solution);

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_INTRINSICS_SOLVER_H_

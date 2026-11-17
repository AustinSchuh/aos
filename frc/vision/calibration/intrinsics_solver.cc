#include "frc/vision/calibration/intrinsics_solver.h"

#include <cmath>

#include "absl/log/absl_log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/calib3d.hpp"

namespace frc::vision {
namespace {

const std::vector<cv::Point3f> &ChessboardCorners(
    const cv::Ptr<cv::aruco::CharucoBoard> &board) {
#if CV_VERSION_MINOR >= 9
  return board->getChessboardCorners();
#else
  return board->chessboardCorners;
#endif
}

// OpenCV's distortion coefficients, in its order: k1, k2, p1, p2, k3, k4, k5,
// k6.
constexpr int kDistortionTerms = 8;

// One board corner's reprojection error, as OpenCV's model projects it.
struct CornerError {
  CornerError(cv::Point3f board, cv::Point2f image)
      : board_(board), image_(image) {}

  // intrinsics: fx, fy, cx, cy.  distortion: OpenCV's 8 coefficients.  pose:
  // board to camera, as an angle-axis rotation then a translation.
  template <typename T>
  bool operator()(const T *intrinsics, const T *distortion, const T *pose,
                  T *residual) const {
    const T board[3] = {T(board_.x), T(board_.y), T(board_.z)};
    T camera[3];
    ceres::AngleAxisRotatePoint(pose, board, camera);
    for (int i = 0; i < 3; ++i) {
      camera[i] += pose[3 + i];
    }
    const T x = camera[0] / camera[2];
    const T y = camera[1] / camera[2];
    const T r2 = x * x + y * y;
    const T r4 = r2 * r2;
    const T r6 = r4 * r2;
    const T &k1 = distortion[0], &k2 = distortion[1], &p1 = distortion[2],
            &p2 = distortion[3], &k3 = distortion[4], &k4 = distortion[5],
            &k5 = distortion[6], &k6 = distortion[7];
    const T radial = (T(1) + k1 * r2 + k2 * r4 + k3 * r6) /
                     (T(1) + k4 * r2 + k5 * r4 + k6 * r6);
    const T xd = x * radial + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
    const T yd = y * radial + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;
    residual[0] = intrinsics[0] * xd + intrinsics[2] - T(image_.x);
    residual[1] = intrinsics[1] * yd + intrinsics[3] - T(image_.y);
    return true;
  }

 private:
  const cv::Point3f board_;
  const cv::Point2f image_;
};

}  // namespace

absl::StatusOr<IntrinsicsSolution> SolveIntrinsics(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures, cv::Size image_size,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    bool rational_model) {
  if (captures.size() < 4) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Need at least 4 captures to calibrate, not ", captures.size()));
  }
  const std::vector<cv::Point3f> &board_corners = ChessboardCorners(board);

  cv::Mat guess_matrix, guess_dist;
  camera_matrix.convertTo(guess_matrix, CV_64F);
  dist_coeffs.convertTo(guess_dist, CV_64F);
  double intrinsics[4] = {
      guess_matrix.at<double>(0, 0), guess_matrix.at<double>(1, 1),
      guess_matrix.at<double>(0, 2), guess_matrix.at<double>(1, 2)};
  if (!(intrinsics[0] > 0 && intrinsics[1] > 0)) {
    // No usable guess: a 60 degree field of view, centred.
    intrinsics[0] = intrinsics[1] = image_size.width * 0.87;
    intrinsics[2] = image_size.width / 2.0;
    intrinsics[3] = image_size.height / 2.0;
  }
  double distortion[kDistortionTerms] = {};
  for (int i = 0; i < std::min<int>(guess_dist.total(), kDistortionTerms);
       ++i) {
    distortion[i] = guess_dist.at<double>(i);
  }
  if (!rational_model) {
    distortion[5] = distortion[6] = distortion[7] = 0.0;
  }
  const cv::Mat start_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
  const cv::Mat start_dist(1, kDistortionTerms, CV_64F, distortion);

  // Each board's pose from the guess, to start from.
  std::vector<std::array<double, 6>> poses(captures.size());
  std::vector<std::vector<cv::Point3f>> object_points(captures.size());
  for (size_t i = 0; i < captures.size(); ++i) {
    const CharucoCapture &capture = captures[i];
    if (capture.ids.size() != capture.corners.size() ||
        capture.ids.size() < 4) {
      return absl::InvalidArgumentError(
          absl::StrCat("Capture ", i, " has ", capture.ids.size(), " ids and ",
                       capture.corners.size(), " corners"));
    }
    for (int id : capture.ids) {
      if (id < 0 || id >= static_cast<int>(board_corners.size())) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Capture ", i, " has corner ", id, ", which the board doesn't"));
      }
      object_points[i].push_back(board_corners[id]);
    }
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(object_points[i], capture.corners, start_matrix,
                      start_dist, rvec, tvec)) {
      return absl::FailedPreconditionError(
          absl::StrCat("Couldn't find the board's pose in capture ", i));
    }
    poses[i] = {rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2]};
  }

  ceres::Problem problem;
  size_t total_corners = 0;
  for (size_t i = 0; i < captures.size(); ++i) {
    for (size_t j = 0; j < captures[i].corners.size(); ++j) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<CornerError, 2, 4, kDistortionTerms,
                                          6>(
              new CornerError(object_points[i][j], captures[i].corners[j])),
          nullptr, intrinsics, distortion, poses[i].data());
    }
    total_corners += captures[i].corners.size();
  }
  if (!rational_model) {
    problem.SetManifold(distortion,
                        new ceres::SubsetManifold(kDistortionTerms, {5, 6, 7}));
  }

  ceres::Solver::Options options;
  // Eliminate the boards' poses, leaving a small dense system in the
  // intrinsics: each iteration is cheap however many captures there are.
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 200;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    return absl::InternalError(
        absl::StrCat("The solve failed: ", summary.message));
  }

  IntrinsicsSolution solution;
  solution.camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
  // As OpenCV returns them: all 14 for the rational model (the thin prism and
  // tilt terms stay 0), 5 otherwise, and shaped as the guess was.
  const int terms = rational_model ? 14 : 5;
  cv::Mat out = cv::Mat::zeros(guess_dist.rows > 1 ? terms : 1,
                               guess_dist.rows > 1 ? 1 : terms, CV_64F);
  for (int i = 0; i < std::min(terms, kDistortionTerms); ++i) {
    out.at<double>(i) = distortion[i];
  }
  solution.dist_coeffs = out;

  double total_squared = 0.0;
  for (size_t i = 0; i < captures.size(); ++i) {
    const cv::Vec3d rvec(poses[i][0], poses[i][1], poses[i][2]);
    const cv::Vec3d tvec(poses[i][3], poses[i][4], poses[i][5]);
    solution.rvecs.push_back(rvec);
    solution.tvecs.push_back(tvec);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points[i], rvec, tvec, solution.camera_matrix,
                      solution.dist_coeffs, projected);
    std::vector<cv::Point2f> residuals;
    double view_squared = 0.0;
    for (size_t j = 0; j < projected.size(); ++j) {
      const cv::Point2f residual = captures[i].corners[j] - projected[j];
      view_squared += residual.dot(residual);
      residuals.push_back(residual);
    }
    total_squared += view_squared;
    solution.per_view_errors.push_back(
        std::sqrt(view_squared / projected.size()));
    solution.residuals.push_back(std::move(residuals));
  }
  // As OpenCV reports it: the RMS of each corner's error distance.
  solution.reprojection_error = std::sqrt(total_squared / total_corners);
  ABSL_LOG(INFO) << "Solved from " << captures.size() << " captures in "
                 << summary.iterations.size() << " iterations, "
                 << summary.total_time_in_seconds << " s: reprojection error "
                 << solution.reprojection_error << " px";
  if (!(solution.reprojection_error <= kMaxReprojectionError)) {
    return absl::FailedPreconditionError(absl::StrCat(
        "The reprojection error is ", solution.reprojection_error, " px, over ",
        kMaxReprojectionError, " px: some captures are probably wrong"));
  }
  return solution;
}

}  // namespace frc::vision

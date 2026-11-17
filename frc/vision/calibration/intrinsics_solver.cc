#include "frc/vision/calibration/intrinsics_solver.h"

#include <array>
#include <cmath>
#include <optional>

#include "absl/log/absl_log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "opencv2/calib3d.hpp"

namespace frc::vision {
namespace {

std::vector<cv::Point3f> ChessboardCorners(
    const cv::Ptr<cv::aruco::CharucoBoard> &board) {
#if CV_VERSION_MINOR >= 9
  return board->getChessboardCorners();
#else
  return board->chessboardCorners;
#endif
}

// The most distortion terms any model has: OpenCV's rational model's k1, k2,
// p1, p2, k3, k4, k5, k6.  Kannala-Brandt uses the first 4.
constexpr int kDistortionTerms = 8;

// Where a point at (x, y) on the normalized image plane lands, distorted by
// `model` with `d`.
template <typename T>
void Distort(LensModel model, const T *d, const T &x, const T &y, T *xd,
             T *yd) {
  const T r2 = x * x + y * y;
  if (model == LensModel::kKannalaBrandt) {
    // Kannala-Brandt: the angle from the axis, polynomially distorted.
    using std::atan;
    using std::sqrt;
    if (r2 < T(1e-16)) {
      *xd = x;
      *yd = y;
      return;
    }
    const T r = sqrt(r2);
    const T theta = atan(r);
    const T theta2 = theta * theta;
    const T theta_d =
        theta *
        (T(1) +
         theta2 * (d[0] + theta2 * (d[1] + theta2 * (d[2] + theta2 * d[3]))));
    const T scale = theta_d / r;
    *xd = x * scale;
    *yd = y * scale;
    return;
  }
  const T r4 = r2 * r2;
  const T r6 = r4 * r2;
  const T &k1 = d[0], &k2 = d[1], &p1 = d[2], &p2 = d[3], &k3 = d[4],
          &k4 = d[5], &k5 = d[6], &k6 = d[7];
  const T radial = (T(1) + k1 * r2 + k2 * r4 + k3 * r6) /
                   (T(1) + k4 * r2 + k5 * r4 + k6 * r6);
  *xd = x * radial + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
  *yd = y * radial + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;
}

// Where a board point lands in the image: board to camera by `pose`
// (angle-axis, then translation), distorted, then through fx, fy, cx, cy.
template <typename T>
void Project(LensModel model, const T *intrinsics, const T *distortion,
             const T *pose, const cv::Point3f &point, T *u, T *v) {
  const T board[3] = {T(point.x), T(point.y), T(point.z)};
  T camera[3];
  ceres::AngleAxisRotatePoint(pose, board, camera);
  for (int i = 0; i < 3; ++i) {
    camera[i] += pose[3 + i];
  }
  T xd, yd;
  Distort(model, distortion, camera[0] / camera[2], camera[1] / camera[2], &xd,
          &yd);
  *u = intrinsics[0] * xd + intrinsics[2];
  *v = intrinsics[1] * yd + intrinsics[3];
}

// One board corner's reprojection error.
struct CornerError {
  CornerError(LensModel model, cv::Point3f board, cv::Point2f image)
      : model_(model), board_(board), image_(image) {}

  template <typename T>
  bool operator()(const T *intrinsics, const T *distortion, const T *pose,
                  T *residual) const {
    T u, v;
    Project(model_, intrinsics, distortion, pose, board_, &u, &v);
    residual[0] = u - T(image_.x);
    residual[1] = v - T(image_.y);
    return true;
  }

 private:
  const LensModel model_;
  const cv::Point3f board_;
  const cv::Point2f image_;
};

// A prior on the rational model's denominator terms, k4..k6.
struct DenominatorPrior {
  explicit DenominatorPrior(double sigma) : sigma_(sigma) {}

  template <typename T>
  bool operator()(const T *distortion, T *residual) const {
    for (int i = 0; i < 3; ++i) {
      residual[i] = distortion[5 + i] / T(sigma_);
    }
    return true;
  }

 private:
  const double sigma_;
};

// The board's pose in one capture, with the intrinsics held.
struct PoseOnly {
  PoseOnly(LensModel model, const double *intrinsics, const double *distortion,
           cv::Point3f board, cv::Point2f image)
      : model_(model),
        intrinsics_(intrinsics),
        distortion_(distortion),
        board_(board),
        image_(image) {}

  template <typename T>
  bool operator()(const T *pose, T *residual) const {
    T intrinsics[4], distortion[kDistortionTerms];
    for (int i = 0; i < 4; ++i) {
      intrinsics[i] = T(intrinsics_[i]);
    }
    for (int i = 0; i < kDistortionTerms; ++i) {
      distortion[i] = T(distortion_[i]);
    }
    T u, v;
    Project(model_, intrinsics, distortion, pose, board_, &u, &v);
    residual[0] = u - T(image_.x);
    residual[1] = v - T(image_.y);
    return true;
  }

 private:
  const LensModel model_;
  const double *intrinsics_;
  const double *distortion_;
  const cv::Point3f board_;
  const cv::Point2f image_;
};

// A capture's board points, by its corners' ids, or an error if they're not
// the board's.
absl::StatusOr<std::vector<cv::Point3f>> ObjectPoints(
    const std::vector<cv::Point3f> &board_corners,
    const CharucoCapture &capture, size_t index) {
  if (capture.ids.size() != capture.corners.size() || capture.ids.size() < 4) {
    return absl::InvalidArgumentError(
        absl::StrCat("Capture ", index, " has ", capture.ids.size(),
                     " ids and ", capture.corners.size(), " corners"));
  }
  std::vector<cv::Point3f> points;
  for (int id : capture.ids) {
    if (id < 0 || id >= static_cast<int>(board_corners.size())) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Capture ", index, " has corner ", id, ", which the board doesn't"));
    }
    points.push_back(board_corners[id]);
  }
  return points;
}

// A rough pose for a capture, from a camera matrix and OpenCV distortion
// coefficients (empty for none).
std::optional<std::array<double, 6>> StartingPose(
    const std::vector<cv::Point3f> &object, const CharucoCapture &capture,
    const cv::Mat &matrix, const cv::Mat &dist_coeffs) {
  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(object, capture.corners, matrix, dist_coeffs, rvec, tvec)) {
    return std::nullopt;
  }
  return std::array<double, 6>{rvec[0], rvec[1], rvec[2],
                               tvec[0], tvec[1], tvec[2]};
}

ceres::Solver::Options SolverOptions() {
  ceres::Solver::Options options;
  // Eliminate the boards' poses, leaving a small dense system in the
  // intrinsics: each iteration is cheap however many captures there are.
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 200;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.num_threads = 4;
  return options;
}

// The distortion terms a model uses.
int Terms(LensModel model) {
  switch (model) {
    case LensModel::kOpenCv5:
      return 5;
    case LensModel::kOpenCvRational:
      return 8;
    case LensModel::kKannalaBrandt:
      return 4;
  }
  return 8;
}

}  // namespace

absl::StatusOr<IntrinsicsSolution> SolveIntrinsics(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures, cv::Size image_size,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    bool rational_model) {
  return SolveIntrinsics(
      board, captures, image_size, camera_matrix, dist_coeffs,
      SolveOptions{.model = rational_model ? LensModel::kOpenCvRational
                                           : LensModel::kOpenCv5,
                   .denominator_sigma = 0.0});
}

absl::StatusOr<IntrinsicsSolution> SolveIntrinsics(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures, cv::Size image_size,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    const SolveOptions &solve_options) {
  const LensModel model = solve_options.model;
  if (captures.size() < 4) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Need at least 4 captures to calibrate, not ", captures.size()));
  }
  const std::vector<cv::Point3f> board_corners = ChessboardCorners(board);

  cv::Mat guess_matrix, guess_dist;
  camera_matrix.convertTo(guess_matrix, CV_64F);
  dist_coeffs.convertTo(guess_dist, CV_64F);
  double intrinsics[4] = {0, 0, 0, 0};
  if (!guess_matrix.empty()) {
    intrinsics[0] = guess_matrix.at<double>(0, 0);
    intrinsics[1] = guess_matrix.at<double>(1, 1);
    intrinsics[2] = guess_matrix.at<double>(0, 2);
    intrinsics[3] = guess_matrix.at<double>(1, 2);
  }
  if (!(intrinsics[0] > 0 && intrinsics[1] > 0)) {
    // No usable guess: a 60 degree field of view, centred.
    intrinsics[0] = intrinsics[1] = image_size.width * 0.87;
    intrinsics[2] = image_size.width / 2.0;
    intrinsics[3] = image_size.height / 2.0;
  }
  // The guess's distortion only means something in the guess's model: start
  // other models from none.
  double distortion[kDistortionTerms] = {};
  if (model != LensModel::kKannalaBrandt) {
    for (int i = 0; i < std::min<int>(guess_dist.total(), Terms(model)); ++i) {
      distortion[i] = guess_dist.at<double>(i);
    }
  }

  // Each board's pose from the guess, distortion and all, to start from:
  // whatever the model being solved, the guess's poses are close.
  const cv::Mat start_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
  const cv::Mat start_dist =
      guess_dist.total() == 4 || guess_dist.total() == 5 ||
              guess_dist.total() == 8 || guess_dist.total() == 12 ||
              guess_dist.total() == 14
          ? guess_dist
          : cv::Mat();
  std::vector<std::array<double, 6>> poses(captures.size());
  std::vector<std::vector<cv::Point3f>> object_points(captures.size());
  for (size_t i = 0; i < captures.size(); ++i) {
    absl::StatusOr<std::vector<cv::Point3f>> points =
        ObjectPoints(board_corners, captures[i], i);
    if (!points.ok()) {
      return points.status();
    }
    object_points[i] = *std::move(points);
    const std::optional<std::array<double, 6>> pose =
        StartingPose(object_points[i], captures[i], start_matrix, start_dist);
    if (!pose.has_value()) {
      return absl::FailedPreconditionError(
          absl::StrCat("Couldn't find the board's pose in capture ", i));
    }
    poses[i] = *pose;
  }

  ceres::Problem problem;
  size_t total_corners = 0;
  for (size_t i = 0; i < captures.size(); ++i) {
    for (size_t j = 0; j < captures[i].corners.size(); ++j) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<CornerError, 2, 4, kDistortionTerms,
                                          6>(new CornerError(
              model, object_points[i][j], captures[i].corners[j])),
          nullptr, intrinsics, distortion, poses[i].data());
    }
    total_corners += captures[i].corners.size();
  }
  // Hold the terms the model doesn't use.
  std::vector<int> unused;
  for (int i = Terms(model); i < kDistortionTerms; ++i) {
    unused.push_back(i);
  }
  if (!unused.empty()) {
    problem.SetManifold(distortion,
                        new ceres::SubsetManifold(kDistortionTerms, unused));
  }
  if (model == LensModel::kOpenCvRational &&
      solve_options.denominator_sigma > 0) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<DenominatorPrior, 3, kDistortionTerms>(
            new DenominatorPrior(solve_options.denominator_sigma)),
        nullptr, distortion);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(SolverOptions(), &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    return absl::InternalError(
        absl::StrCat("The solve failed: ", summary.message));
  }

  IntrinsicsSolution solution;
  solution.model = model;
  solution.camera_matrix =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
  // As OpenCV returns them: all 14 for the rational model (the thin prism and
  // tilt terms stay 0), 5 or 4 otherwise, and shaped as the guess was.
  const int terms = model == LensModel::kOpenCvRational ? 14 : Terms(model);
  cv::Mat out = cv::Mat::zeros(guess_dist.rows > 1 ? terms : 1,
                               guess_dist.rows > 1 ? 1 : terms, CV_64F);
  for (int i = 0; i < std::min(terms, kDistortionTerms); ++i) {
    out.at<double>(i) = distortion[i];
  }
  solution.dist_coeffs = out;

  double total_squared = 0.0;
  for (size_t i = 0; i < captures.size(); ++i) {
    solution.rvecs.emplace_back(poses[i][0], poses[i][1], poses[i][2]);
    solution.tvecs.emplace_back(poses[i][3], poses[i][4], poses[i][5]);
    std::vector<cv::Point2f> residuals;
    double view_squared = 0.0;
    for (size_t j = 0; j < object_points[i].size(); ++j) {
      double u, v;
      Project(model, intrinsics, distortion, poses[i].data(),
              object_points[i][j], &u, &v);
      const cv::Point2f residual = captures[i].corners[j] - cv::Point2f(u, v);
      view_squared += residual.dot(residual);
      residuals.push_back(residual);
    }
    total_squared += view_squared;
    solution.per_view_errors.push_back(
        std::sqrt(view_squared / object_points[i].size()));
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

absl::StatusOr<double> HeldOutError(
    const cv::Ptr<cv::aruco::CharucoBoard> &board,
    const std::vector<CharucoCapture> &captures,
    const IntrinsicsSolution &solution) {
  const std::vector<cv::Point3f> board_corners = ChessboardCorners(board);
  double intrinsics[4] = {solution.camera_matrix.at<double>(0, 0),
                          solution.camera_matrix.at<double>(1, 1),
                          solution.camera_matrix.at<double>(0, 2),
                          solution.camera_matrix.at<double>(1, 2)};
  double distortion[kDistortionTerms] = {};
  for (int i = 0;
       i < std::min<int>(solution.dist_coeffs.total(), kDistortionTerms); ++i) {
    distortion[i] = solution.dist_coeffs.at<double>(i);
  }
  double total_squared = 0.0;
  size_t total_corners = 0;
  for (size_t i = 0; i < captures.size(); ++i) {
    absl::StatusOr<std::vector<cv::Point3f>> points =
        ObjectPoints(board_corners, captures[i], i);
    if (!points.ok()) {
      return points.status();
    }
    // OpenCV can start from the OpenCV models' coefficients; Kannala-Brandt
    // starts from none, and the fit below takes it from there.
    std::optional<std::array<double, 6>> pose = StartingPose(
        *points, captures[i], solution.camera_matrix,
        solution.model == LensModel::kKannalaBrandt ? cv::Mat()
                                                    : solution.dist_coeffs);
    if (!pose.has_value()) {
      return absl::FailedPreconditionError(
          absl::StrCat("Couldn't find the board's pose in capture ", i));
    }
    ceres::Problem problem;
    for (size_t j = 0; j < points->size(); ++j) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<PoseOnly, 2, 6>(
              new PoseOnly(solution.model, intrinsics, distortion, (*points)[j],
                           captures[i].corners[j])),
          nullptr, pose->data());
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    for (size_t j = 0; j < points->size(); ++j) {
      double u, v;
      Project(solution.model, intrinsics, distortion, pose->data(),
              (*points)[j], &u, &v);
      const cv::Point2f residual = captures[i].corners[j] - cv::Point2f(u, v);
      total_squared += residual.dot(residual);
    }
    total_corners += points->size();
  }
  if (total_corners == 0) {
    return absl::InvalidArgumentError("No captures to measure against");
  }
  return std::sqrt(total_squared / total_corners);
}

}  // namespace frc::vision

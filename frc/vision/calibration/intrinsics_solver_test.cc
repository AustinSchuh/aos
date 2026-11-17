#include "frc/vision/calibration/intrinsics_solver.h"

#include <chrono>
#include <random>

#include "absl/log/absl_log.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"

#include "frc/vision/util/charuco_board.h"

namespace frc::vision::testing {

constexpr int kWidth = 1600, kHeight = 1304;

const cv::Mat &TrueCameraMatrix() {
  static const cv::Mat kMatrix =
      (cv::Mat_<double>(3, 3) << 1000, 0, 810, 0, 1001, 640, 0, 0, 1);
  return kMatrix;
}

const cv::Mat &TrueDistCoeffs() {
  static const cv::Mat kCoeffs = (cv::Mat_<double>(1, 8) << 0.12, -0.05, 0.001,
                                  -0.0005, 0.01, 0.1, -0.04, 0.008);
  return kCoeffs;
}

// Captures of the board at `count` random poses, as the true camera sees
// them, with every corner in the image.
std::vector<CharucoCapture> Captures(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, int count) {
  std::mt19937 random(4646);
  std::uniform_real_distribution<double> tilt(-0.6, 0.6), shift(-0.35, 0.35),
      distance(0.8, 1.6);
#if CV_VERSION_MINOR >= 9
  const std::vector<cv::Point3f> corners = board->getChessboardCorners();
#else
  const std::vector<cv::Point3f> &corners = board->chessboardCorners;
#endif
  std::vector<CharucoCapture> captures;
  while (static_cast<int>(captures.size()) < count) {
    const cv::Vec3d rvec(tilt(random), tilt(random), tilt(random) / 2);
    // Centre the board on the optical axis, then shift it.
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    const cv::Vec3d centre(0.26, 0.16, 0.0);
    cv::Mat t = cv::Mat(cv::Vec3d(shift(random), shift(random) * 0.8,
                                  distance(random))) -
                R * cv::Mat(centre);
    const cv::Vec3d tvec(t);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(corners, rvec, tvec, TrueCameraMatrix(), TrueDistCoeffs(),
                      projected);
    CharucoCapture capture;
    for (size_t i = 0; i < projected.size(); ++i) {
      if (projected[i].x >= 0 && projected[i].x < kWidth &&
          projected[i].y >= 0 && projected[i].y < kHeight) {
        capture.ids.push_back(i);
        capture.corners.push_back(projected[i]);
      }
    }
    if (capture.ids.size() >= 40) {
      captures.push_back(std::move(capture));
    }
  }
  return captures;
}

TEST(IntrinsicsSolverTest, RecoversTheCamera) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  const std::vector<CharucoCapture> captures = Captures(board, 70);

  const cv::Mat guess =
      (cv::Mat_<double>(3, 3) << 950, 0, 800, 0, 950, 652, 0, 0, 1);
  const auto start = std::chrono::steady_clock::now();
  absl::StatusOr<IntrinsicsSolution> solution =
      SolveIntrinsics(board, captures, cv::Size(kWidth, kHeight), guess,
                      cv::Mat::zeros(1, 8, CV_64F), true);
  ABSL_LOG(INFO) << "Solving 70 captures took "
                 << std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start)
                        .count()
                 << " s";
  ASSERT_TRUE(solution.ok()) << solution.status();

  EXPECT_LT(solution->reprojection_error, 1e-3);
  EXPECT_LT(cv::norm(solution->camera_matrix - TrueCameraMatrix()), 1e-2);
  ASSERT_EQ(solution->per_view_errors.size(), captures.size());
  ASSERT_EQ(solution->residuals.size(), captures.size());
  for (size_t i = 0; i < captures.size(); ++i) {
    ASSERT_EQ(solution->residuals[i].size(), captures[i].ids.size());
    for (const cv::Point2f &residual : solution->residuals[i]) {
      EXPECT_LT(cv::norm(residual), 1e-2);
    }
  }
}

TEST(IntrinsicsSolverTest, ShowsTheBadCapture) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  std::vector<CharucoCapture> captures = Captures(board, 20);
  // Shift one capture's corners as a bad detection might.
  for (cv::Point2f &corner : captures[7].corners) {
    corner += cv::Point2f(corner.x * 0.004f, 0.0f);
  }
  absl::StatusOr<IntrinsicsSolution> solution =
      SolveIntrinsics(board, captures, cv::Size(kWidth, kHeight),
                      TrueCameraMatrix(), TrueDistCoeffs(), true);
  ASSERT_TRUE(solution.ok()) << solution.status();
  const auto worst = std::max_element(solution->per_view_errors.begin(),
                                      solution->per_view_errors.end());
  EXPECT_EQ(worst - solution->per_view_errors.begin(), 7);
}

TEST(IntrinsicsSolverTest, NeedsCaptures) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  EXPECT_FALSE(SolveIntrinsics(board, Captures(board, 3),
                               cv::Size(kWidth, kHeight), TrueCameraMatrix(),
                               TrueDistCoeffs(), true)
                   .ok());
}

}  // namespace frc::vision::testing

namespace frc::vision::testing {

// With realistic noise, the solve finds as good a fit as OpenCV's
// calibrateCameraCharuco does.
TEST(IntrinsicsSolverTest, MatchesOpenCv) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  std::vector<CharucoCapture> captures = Captures(board, 30);
  std::mt19937 random(1);
  std::normal_distribution<float> noise(0.0, 0.2);
  std::vector<std::vector<int>> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  for (CharucoCapture &capture : captures) {
    for (cv::Point2f &corner : capture.corners) {
      corner += cv::Point2f(noise(random), noise(random));
    }
    ids.push_back(capture.ids);
    corners.push_back(capture.corners);
  }
  const cv::Mat guess =
      (cv::Mat_<double>(3, 3) << 950, 0, 800, 0, 950, 652, 0, 0, 1);

  cv::Mat opencv_matrix = guess.clone(),
          opencv_dist = cv::Mat::zeros(1, 8, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;
  const double opencv_error = cv::aruco::calibrateCameraCharuco(
      corners, ids, board, cv::Size(kWidth, kHeight), opencv_matrix,
      opencv_dist, rvecs, tvecs,
      cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_RATIONAL_MODEL,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10000,
                       1e-12));

  absl::StatusOr<IntrinsicsSolution> solution =
      SolveIntrinsics(board, captures, cv::Size(kWidth, kHeight), guess,
                      cv::Mat::zeros(1, 8, CV_64F), true);
  ASSERT_TRUE(solution.ok()) << solution.status();
  EXPECT_LE(solution->reprojection_error, opencv_error + 1e-4);
  // The rational model's coefficients trade off against each other, but the
  // camera matrix doesn't.
  EXPECT_LT(cv::norm(solution->camera_matrix - opencv_matrix), 1.0);
  EXPECT_LT(cv::norm(solution->camera_matrix - TrueCameraMatrix()), 2.0);
}

TEST(IntrinsicsSolverTest, FiveCoefficientModel) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  absl::StatusOr<IntrinsicsSolution> solution =
      SolveIntrinsics(board, Captures(board, 20), cv::Size(kWidth, kHeight),
                      TrueCameraMatrix(), cv::Mat(), false);
  ASSERT_TRUE(solution.ok()) << solution.status();
  EXPECT_EQ(solution->dist_coeffs.total(), 5u);
}

}  // namespace frc::vision::testing

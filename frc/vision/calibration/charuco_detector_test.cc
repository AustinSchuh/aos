#include "frc/vision/calibration/charuco_detector.h"

#include <cmath>

#include "absl/log/absl_log.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include "frc/vision/util/charuco_board.h"

namespace frc::vision::testing {
namespace {

constexpr int kWidth = 1600, kHeight = 1304;

const cv::Mat &CameraMatrix() {
  static const cv::Mat kMatrix =
      (cv::Mat_<double>(3, 3) << 1000, 0, 805, 0, 1000, 650, 0, 0, 1);
  return kMatrix;
}

std::vector<cv::Point3f> Corners(const cv::Ptr<cv::aruco::CharucoBoard> &b) {
#if CV_VERSION_MINOR >= 9
  return b->getChessboardCorners();
#else
  return b->chessboardCorners;
#endif
}

// The board's picture, `pixels_per_square` a square, and the homography from
// its pixels to the board's plane, in meters.
std::pair<cv::Mat, cv::Mat> Flat(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                                 int pixels_per_square) {
  const cv::Size squares = board->getChessboardSize();
  cv::Mat flat;
  const cv::Size size(squares.width * pixels_per_square,
                      squares.height * pixels_per_square);
#if CV_VERSION_MINOR >= 9
  board->generateImage(size, flat, 0, 1);
#else
  board->draw(size, flat, 0, 1);
#endif
  // Drawn edge to edge, so the board's edges are the picture's: pixel centres
  // are at whole numbers, so its edges are at -0.5 and size - 0.5.  Which way
  // up it's drawn is found from where the corners are detected.
  const double scale = pixels_per_square / board->getSquareLength();
  const CharucoDetection detection =
      DetectCharuco(board, flat,
                    (cv::Mat_<double>(3, 3) << 1000, 0, size.width / 2.0, 0,
                     1000, size.height / 2.0, 0, 0, 1),
                    cv::Mat::zeros(5, 1, CV_64F));
  const std::vector<cv::Point3f> corners = Corners(board);
  cv::Mat best;
  double best_error = INFINITY;
  for (bool flip_x : {false, true}) {
    for (bool flip_y : {false, true}) {
      // Plane to pixels.
      const cv::Mat to_pixels =
          (cv::Mat_<double>(3, 3) << (flip_x ? -scale : scale), 0,
           (flip_x ? size.width : 0) - 0.5, 0, (flip_y ? -scale : scale),
           (flip_y ? size.height : 0) - 0.5, 0, 0, 1);
      double error = 0;
      for (size_t i = 0; i < detection.ids.size(); ++i) {
        const cv::Point3f &c = corners[detection.ids[i]];
        const cv::Mat p = to_pixels * (cv::Mat_<double>(3, 1) << c.x, c.y, 1.0);
        error += cv::norm(cv::Point2d(p.at<double>(0), p.at<double>(1)) -
                          cv::Point2d(detection.corners[i]));
      }
      if (error < best_error) {
        best_error = error;
        best = to_pixels.inv();
      }
    }
  }
  return {flat, best};
}

struct Rendered {
  cv::Mat image;
  // Each corner's true place, by id; negative off the image.
  std::vector<cv::Point2f> truth;
};

// The board at a pose, with no distortion, rendered `supersample` times over
// and averaged down, as a lens and sensor average light over each pixel.
Rendered Render(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                const cv::Vec3d &rvec, const cv::Vec3d &tvec, int supersample) {
  const auto [flat, flat_to_plane] = Flat(board, 200);
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::Mat plane_to_camera(3, 3, CV_64F);
  R.col(0).copyTo(plane_to_camera.col(0));
  R.col(1).copyTo(plane_to_camera.col(1));
  cv::Mat(tvec).copyTo(plane_to_camera.col(2));
  // Pixel centres at whole numbers, at every scale.
  const double s = supersample;
  const cv::Mat scale = (cv::Mat_<double>(3, 3) << s, 0, (s - 1) / 2.0, 0, s,
                         (s - 1) / 2.0, 0, 0, 1);
  const cv::Mat H = scale * CameraMatrix() * plane_to_camera * flat_to_plane;
  cv::Mat big;
  cv::warpPerspective(flat, big, H, cv::Size(kWidth * s, kHeight * s),
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
  Rendered rendered;
  cv::resize(big, rendered.image, cv::Size(kWidth, kHeight), 0, 0,
             cv::INTER_AREA);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(Corners(board), rvec, tvec, CameraMatrix(),
                    cv::Mat::zeros(5, 1, CV_64F), projected);
  rendered.truth = projected;
  return rendered;
}

// The corners found's mean offset from their true places, and their RMS
// distance from them.
std::pair<cv::Point2f, double> DetectionError(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const Rendered &rendered) {
  const CharucoDetection detection = DetectCharuco(
      board, rendered.image, CameraMatrix(), cv::Mat::zeros(5, 1, CV_64F));
  EXPECT_GT(detection.ids.size(), 30u);
  double sum = 0;
  cv::Point2f mean(0, 0);
  for (size_t i = 0; i < detection.ids.size(); ++i) {
    const cv::Point2f d =
        detection.corners[i] - rendered.truth[detection.ids[i]];
    sum += d.dot(d);
    mean += d;
  }
  const float count = std::max<size_t>(1, detection.ids.size());
  return {mean / count, std::sqrt(sum / count)};
}

// OpenCV 4.6's interpolateCornersCharuco puts every corner about half a pixel
// off in x and y, which moves a calibration's principal point by as much;
// RefineCorners takes that out.
TEST(CharucoDetectorTest, FindsCornersWithoutBias) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  const cv::Vec3d rvec(0.3, -0.25, 0.1);
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  const cv::Mat t = cv::Mat(cv::Vec3d(0.05, -0.03, 1.1)) -
                    R * cv::Mat(cv::Vec3d(0.28, 0.18, 0.0));
  for (int supersample : {1, 4}) {
    const auto [mean, rms] =
        DetectionError(board, Render(board, rvec, cv::Vec3d(t), supersample));
    ABSL_LOG(INFO) << "Rendered " << supersample << "x over: corners off by "
                   << mean << " on average, " << rms << " px RMS";
    EXPECT_LT(cv::norm(mean), 0.05);
    // Rendered once a pixel, edges alias; averaged over 16 points a pixel, as
    // a sensor averages light, they don't, and corners are found to about
    // 0.08 px.
    EXPECT_LT(rms, supersample > 1 ? 0.1 : 0.5);
  }
}

}  // namespace
}  // namespace frc::vision::testing

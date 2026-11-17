#include "frc/vision/calibration/charuco_detector.h"

#include <cmath>

#include "absl/log/absl_log.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include "frc/vision/calibration/charuco_rendering.h"
#include "frc/vision/util/charuco_board.h"

namespace frc::vision::testing {
namespace {

constexpr int kWidth = 1600, kHeight = 1304;

const cv::Mat &CameraMatrix() {
  static const cv::Mat kMatrix =
      (cv::Mat_<double>(3, 3) << 1000, 0, 805, 0, 1000, 650, 0, 0, 1);
  return kMatrix;
}

Rendered Render(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                const cv::Vec3d &rvec, const cv::Vec3d &tvec, int supersample) {
  return RenderBoard(board, CameraMatrix(), cv::Size(kWidth, kHeight), rvec,
                     tvec, supersample);
}

// The corners found's mean offset from their true places, and their RMS
// distance from them.
std::pair<cv::Point2f, double> DetectionError(
    const cv::Ptr<cv::aruco::CharucoBoard> &board, const Rendered &rendered) {
  const CharucoDetection detection =
      DetectCharuco(board, rendered.image,
                    CameraLens{.camera_matrix = CameraMatrix(),
                               .dist_coeffs = cv::Mat::zeros(5, 1, CV_64F),
                               .kannala_brandt = false});
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

// The board's corners are found to a few hundredths of a pixel, with no bias:
// OpenCV 4.6's interpolateCornersCharuco puts every corner about half a pixel
// off in x and y, which moves a calibration's principal point by as much, and
// cornerSubPix alone finds them to 0.05-0.15 px.  Rendered 4 times over and
// averaged down, as a sensor averages light over each pixel.
TEST(CharucoDetectorTest, FindsCornersToAFewHundredthsOfAPixel) {
  cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  const cv::Vec3d rvec(0.3, -0.25, 0.1);
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::setRNGSeed(971);
  struct Case {
    double distance, blur, noise, max_rms;
  };
  for (const Case &c : {Case{1.1, 0.0, 0.0, 0.03}, Case{1.1, 0.8, 2.0, 0.03},
                        Case{1.1, 1.2, 4.0, 0.04}, Case{1.8, 0.0, 0.0, 0.05},
                        Case{1.8, 0.8, 2.0, 0.05}}) {
    const cv::Mat t = cv::Mat(cv::Vec3d(0.05, -0.03, c.distance)) -
                      R * cv::Mat(cv::Vec3d(0.28, 0.18, 0.0));
    const auto [mean, rms] = DetectionError(
        board, Degrade(Render(board, rvec, cv::Vec3d(t), 4), c.blur, c.noise));
    ABSL_LOG(INFO) << c.distance << " m away, blur " << c.blur << ", noise "
                   << c.noise << ": corners off by " << mean << " on average, "
                   << rms << " px RMS";
    EXPECT_LT(cv::norm(mean), 0.01);
    EXPECT_LT(rms, c.max_rms);
  }
  // Rendered once a pixel, edges alias, but no bias.
  const cv::Mat t = cv::Mat(cv::Vec3d(0.05, -0.03, 1.1)) -
                    R * cv::Mat(cv::Vec3d(0.28, 0.18, 0.0));
  const auto [mean, rms] =
      DetectionError(board, Render(board, rvec, cv::Vec3d(t), 1));
  EXPECT_LT(cv::norm(mean), 0.05);
  EXPECT_LT(rms, 0.2);
}

}  // namespace
}  // namespace frc::vision::testing

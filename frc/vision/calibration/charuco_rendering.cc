#include "frc/vision/calibration/charuco_rendering.h"

#include <cmath>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include "frc/vision/calibration/charuco_detector.h"

namespace frc::vision::testing {
namespace {

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
                    CameraLens{.camera_matrix = (cv::Mat_<double>(3, 3) << 1000,
                                                 0, size.width / 2.0, 0, 1000,
                                                 size.height / 2.0, 0, 0, 1),
                               .dist_coeffs = cv::Mat::zeros(5, 1, CV_64F),
                               .kannala_brandt = false});
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

}  // namespace

Rendered RenderBoard(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                     const cv::Mat &camera_matrix, cv::Size size,
                     const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                     int supersample) {
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
  const cv::Mat H = scale * camera_matrix * plane_to_camera * flat_to_plane;
  cv::Mat big;
  cv::warpPerspective(flat, big, H, cv::Size(size.width * s, size.height * s),
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
  Rendered rendered;
  cv::resize(big, rendered.image, size, 0, 0, cv::INTER_AREA);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(Corners(board), rvec, tvec, camera_matrix,
                    cv::Mat::zeros(5, 1, CV_64F), projected);
  rendered.truth = projected;
  return rendered;
}

// `rendered`, blurred by a lens and with a sensor's noise.
Rendered Degrade(Rendered rendered, double blur, double noise) {
  cv::Mat image;
  rendered.image.convertTo(image, CV_32F);
  if (blur > 0) {
    cv::GaussianBlur(image, image, cv::Size(0, 0), blur);
  }
  if (noise > 0) {
    cv::Mat n(image.size(), CV_32F);
    cv::randn(n, 0, noise);
    image += n;
  }
  image.convertTo(rendered.image, CV_8U);
  return rendered;
}

}  // namespace frc::vision::testing

#ifndef FRC_VISION_CALIBRATION_CHARUCO_RENDERING_H_
#define FRC_VISION_CALIBRATION_CHARUCO_RENDERING_H_

#include <vector>

#include "opencv2/aruco/charuco.hpp"
#include "opencv2/core.hpp"

namespace frc::vision::testing {

// A ChArUco board as a camera with no distortion sees it, for tests.
struct Rendered {
  cv::Mat image;
  // Each inner corner's true place, by id.
  std::vector<cv::Point2f> truth;
};

// The board at a pose, board to camera, as `camera_matrix` sees it in an image
// of `size`, rendered `supersample` times over and averaged down, as a sensor
// averages light over each pixel.  Pixel centres are at whole numbers, as
// OpenCV has them.
Rendered RenderBoard(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                     const cv::Mat &camera_matrix, cv::Size size,
                     const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                     int supersample = 4);

// `rendered`, blurred by a lens (a standard deviation in pixels) and with a
// sensor's noise (a standard deviation in grey levels).
Rendered Degrade(Rendered rendered, double blur, double noise);

}  // namespace frc::vision::testing

#endif  // FRC_VISION_CALIBRATION_CHARUCO_RENDERING_H_

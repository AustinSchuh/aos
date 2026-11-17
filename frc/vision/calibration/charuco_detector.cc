#include "frc/vision/calibration/charuco_detector.h"

#include <algorithm>
#include <cmath>

#include "opencv2/imgproc.hpp"

namespace frc::vision {

void RefineCorners(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                   const cv::Mat &image, std::vector<cv::Point2f> &corners) {
  if (corners.size() < 2) {
    return;
  }
  cv::Mat gray = image;
  if (image.channels() != 1) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  // A square's size in the image: the median distance to the nearest corner.
  std::vector<double> nearest;
  for (size_t i = 0; i < corners.size(); ++i) {
    double best = INFINITY;
    for (size_t j = 0; j < corners.size(); ++j) {
      if (i != j) {
        best = std::min<double>(best, cv::norm(corners[i] - corners[j]));
      }
    }
    nearest.push_back(best);
  }
  std::nth_element(nearest.begin(), nearest.begin() + nearest.size() / 2,
                   nearest.end());
  const double square_px = nearest[nearest.size() / 2];
  // The window stays inside the white margin around the markers, which is
  // (square - marker) / 2 from the corner, so their edges don't pull on it.
  const double margin =
      (1.0 - board->getMarkerLength() / board->getSquareLength()) / 2.0;
  const int half_window =
      std::clamp(static_cast<int>(square_px * margin * 0.9), 2, 8);
  cv::cornerSubPix(
      gray, corners, cv::Size(half_window, half_window), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100,
                       1e-4));
}

CharucoDetection DetectCharuco(const cv::Ptr<cv::aruco::CharucoBoard> &board,
                               const cv::Mat &image,
                               const cv::Mat &camera_matrix,
                               const cv::Mat &dist_coeffs, int min_corners) {
  CharucoDetection detection;
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::makePtr<cv::aruco::DetectorParameters>();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
#if CV_VERSION_MINOR >= 9
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::makePtr<cv::aruco::Dictionary>(board->getDictionary());
#else
  const cv::Ptr<cv::aruco::Dictionary> dictionary = board->dictionary;
#endif
  cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids,
                           parameters);
  detection.markers = marker_ids.size();
  if (marker_ids.empty()) {
    return detection;
  }

  cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board,
                                       detection.corners, detection.ids);
  if (static_cast<int>(detection.ids.size()) < min_corners) {
    return detection;
  }
  RefineCorners(board, image, detection.corners);

  std::vector<cv::Point2f> calibrated_corners;
  std::vector<int> calibrated_ids;
  cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board,
                                       calibrated_corners, calibrated_ids,
                                       camera_matrix, dist_coeffs);
  if (static_cast<int>(calibrated_ids.size()) < min_corners) {
    return detection;
  }
  detection.pose_valid = cv::aruco::estimatePoseCharucoBoard(
      calibrated_corners, calibrated_ids, board, camera_matrix, dist_coeffs,
      detection.rvec, detection.tvec);
  return detection;
}

}  // namespace frc::vision

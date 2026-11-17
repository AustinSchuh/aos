#include "frc/vision/calibration/charuco_detector.h"

#include "frc/vision/util/corner_refinement.h"

namespace frc::vision {

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
  RefineCharucoCorners(board, image, detection.corners);

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

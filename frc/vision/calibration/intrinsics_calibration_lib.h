#ifndef FRC_VISION_CALIBRATION_LIB_H_
#define FRC_VISION_CALIBRATION_LIB_H_
#include <cmath>
#include <filesystem>
#include <numbers>
#include <regex>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/event_loop.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc/vision/calibration/capture_gate.h"
#include "frc/vision/calibration/intrinsics_solver.h"
#include "frc/vision/util/charuco_lib.h"
#include "frc/vision/util/vision_util_lib.h"

ABSL_DECLARE_FLAG(bool, draw_axes);
ABSL_DECLARE_FLAG(std::string, image_load_path);
ABSL_DECLARE_FLAG(bool, use_rational_model);

namespace frc::vision {

class IntrinsicsCalibration {
 public:
  IntrinsicsCalibration(aos::EventLoop *event_loop, std::string_view hostname,
                        std::string_view camera_channel,
                        std::string_view camera_id,
                        std::string_view base_intrinsics_file,
                        bool display_undistorted,
                        std::string_view calibration_folder,
                        aos::ExitHandle *exit_handle);

  void HandleCharuco(cv::Mat rgb_image,
                     const aos::monotonic_clock::time_point /*eof*/,
                     std::vector<cv::Vec4i> charuco_ids,
                     std::vector<std::vector<cv::Point2f>> charuco_corners,
                     bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                     std::vector<Eigen::Vector3d> tvecs_eigen);

  // Load images from disk, for testing
  void LoadImagesFromPath(const std::filesystem::path &path);
  void LoadImages(std::vector<std::string> file_list);

  void DrawCornersOnImage(cv::Mat image, uint index, std::vector<cv::Mat> tvecs,
                          std::vector<cv::Mat> rvecs, cv::Mat camera_matrix,
                          cv::Mat dist_coeffs);
  // Calibrates from the captures, if there are enough, and writes the
  // calibration file.  Returns an error, and writes nothing, if the
  // calibration fails ValidateCalibration at the captured image size and
  // --write_invalid_calibration is false.
  absl::Status MaybeCalibrate();

  // Expose CharucoExtractor for testing purposes
  CharucoExtractor &GetCharucoExtractor() { return charuco_extractor_; };

  static aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
  BuildCalibration(cv::Mat camera_matrix, cv::Mat dist_coeffs,
                   aos::realtime_clock::time_point realtime_now,
                   std::string_view node_name, std::string_view camera_channel,
                   std::string_view camera_id, uint16_t team_number,
                   double reprojection_error,
                   std::optional<BoardSpec> board = std::nullopt);

  // Writes calibration to filename if it passes ValidateCalibration at
  // image_size, or --write_invalid_calibration says to write it anyway.
  // Returns an error, having written nothing, otherwise.
  static absl::Status WriteValidatedCalibration(
      const calibration::CameraCalibration &calibration, cv::Size image_size,
      const std::string &filename);

  // Return how many captures we've made so far
  int NumCaptures() const { return all_charuco_ids_.size(); };

  // The captures so far, and the images' size.
  std::vector<CharucoCapture> Captures() const {
    std::vector<CharucoCapture> captures;
    for (size_t i = 0; i < all_charuco_ids_.size(); ++i) {
      captures.push_back({all_charuco_ids_[i], all_charuco_corners_[i]});
    }
    return captures;
  }
  cv::Size ImageSize() const { return image_size_; }

  // Allow setting of ids and corners, for testing purposes
  void SetCharucoIds(std::vector<std::vector<int>> &charuco_ids) {
    all_charuco_ids_ = charuco_ids;
  };

  void SetCharucoCorners(
      std::vector<std::vector<cv::Point2f>> &charuco_corners) {
    all_charuco_corners_ = charuco_corners;
  };

  // Retrieve calibration parameters (for testing)
  const cv::Mat GetCameraMatrix() { return camera_mat_; };
  void SetCameraMatrix(cv::Mat camera_mat) { camera_mat_ = camera_mat; };
  const cv::Mat GetDistortionCoefficients() { return dist_coeffs_; };
  void SetDistortionCoefficients(cv::Mat dist_coeffs) {
    dist_coeffs_ = dist_coeffs;
  };
  const std::vector<cv::Mat> GetTVecs() { return tvecs_; };
  const std::vector<cv::Mat> GetRVecs() { return rvecs_; };
  double GetReprojectionError() { return reprojection_error_; };

 private:
  std::string hostname_;
  std::string node_name_;
  const std::string camera_channel_;
  const std::string camera_id_;

  std::vector<std::vector<int>> all_charuco_ids_;
  std::vector<std::vector<cv::Point2f>> all_charuco_corners_;

  // Computed intrinsics calibration data
  cv::Mat camera_mat_;
  cv::Mat dist_coeffs_;
  std::vector<cv::Mat> tvecs_;
  std::vector<cv::Mat> rvecs_;
  double reprojection_error_;

  // Decides which frames to capture.
  CaptureGate capture_gate_;

  // Image used to visualize collected points as they come in
  cv::Mat point_viz_image_;
  cv::Size image_size_;
  // Camera intrinsics that we will use to bootstrap the intrinsics estimation
  // here. We make use of the intrinsics in this calibration to allow us to
  // estimate the relative pose of the charuco board and then identify how
  // much the board is moving.
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      base_intrinsics_;

  CharucoExtractor charuco_extractor_;

  ImageCallback image_callback_;

  const bool display_undistorted_;
  const std::string calibration_folder_;
  aos::ExitHandle *exit_handle_;

  bool exit_collection_;
  std::vector<std::string> file_list_;

  int image_save_count_ = 0;
};

}  // namespace frc::vision
#endif  // FRC_VISION_CALIBRATION_LIB_H_

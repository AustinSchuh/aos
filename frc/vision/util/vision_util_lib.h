#ifndef FRC_VISION_UTIL_VISION_UTIL_LIB_H_
#define FRC_VISION_UTIL_VISION_UTIL_LIB_H_
#include <optional>
#include <string_view>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include "frc/vision/calibration_generated.h"
#include "frc/vision/ceres/types.h"

// Extract the CameraExtrinsics from a CameraCalibration struct
namespace frc::vision {
std::optional<cv::Mat> CameraExtrinsics(
    const frc::vision::calibration::CameraCalibration *camera_calibration);

// Extract the CameraIntrinsics from a CameraCalibration struct
cv::Mat CameraIntrinsics(
    const frc::vision::calibration::CameraCalibration *camera_calibration);

// Extract the CameraDistCoeffs from a CameraCalibration struct
cv::Mat CameraDistCoeffs(
    const frc::vision::calibration::CameraCalibration *camera_calibration);

// A camera's lens, as OpenCV's functions take it: its camera matrix and
// distortion coefficients, which are OpenCV's pinhole model's, or with
// kannala_brandt, Kannala-Brandt's k1..k4, which OpenCV's fisheye functions
// take.  Code that undistorts, projects or fits poses goes through this, so it
// works with either model.
struct CameraLens {
  // 3x3.
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  bool kannala_brandt = false;

  // `calibration`'s, as CV_64F.
  static CameraLens FromCalibration(
      const frc::vision::calibration::CameraCalibration *calibration);

  // Where `points` would be without the lens's distortion, as pixels of a
  // pinhole camera with the same camera matrix.
  std::vector<cv::Point2f> Undistort(
      const std::vector<cv::Point2f> &points) const;

  // Where `object` lands in the image at the pose (rvec, tvec).
  std::vector<cv::Point2f> Project(const std::vector<cv::Point3f> &object,
                                   const cv::Vec3d &rvec,
                                   const cv::Vec3d &tvec) const;

  // The pose of `object` seen at `image`, as cv::solvePnP finds it: for
  // Kannala-Brandt, from the undistorted points.  `refine` polishes it with
  // solvePnPRefineLM, keeping it only if it fits better.
  bool SolvePnP(const std::vector<cv::Point3f> &object,
                const std::vector<cv::Point2f> &image, cv::Vec3d &rvec,
                cv::Vec3d &tvec, int flags = cv::SOLVEPNP_ITERATIVE,
                bool refine = false) const;

  // `image` undistorted, as a pinhole camera with the same camera matrix sees
  // it.
  cv::Mat UndistortImage(const cv::Mat &image) const;

  // Distortion coefficients for OpenCV's pinhole functions: the lens's, or
  // none for Kannala-Brandt, whose points must be undistorted first.
  cv::Mat PinholeDistCoeffs() const {
    return kannala_brandt ? cv::Mat() : dist_coeffs;
  }
};

// Get the camera number from a camera channel name, e.g., return 2 from
// "/camera2".  Returns nullopt if string doesn't start with "/camera" or does
// not have a number
std::optional<uint16_t> CameraNumberFromChannel(std::string camera_channel);

// Return a calibration filename to save to based on the given data
std::string CalibrationFilename(std::string calibration_folder,
                                std::string node_name, int team_number,
                                int camera_number, std::string camera_id,
                                std::string timestamp);

// Utility functions for dealing with ceres::examples::Pose3d structs
class PoseUtils {
 public:
  // Embeds a 3d pose into an affine transformation
  static Eigen::Affine3d Pose3dToAffine3d(
      const ceres::examples::Pose3d &pose3d);
  // Inverse of above function
  static ceres::examples::Pose3d Affine3dToPose3d(const Eigen::Affine3d &H);

  // Computes pose_2 relative to pose_1. This is equivalent to (pose_1^-1 *
  // pose_2)
  static ceres::examples::Pose3d ComputeRelativePose(
      const ceres::examples::Pose3d &pose_1,
      const ceres::examples::Pose3d &pose_2);

  // Computes pose_2 given a pose_1 and pose_2 relative to pose_1. This is
  // equivalent to (pose_1 * pose_2_relative)
  static ceres::examples::Pose3d ComputeOffsetPose(
      const ceres::examples::Pose3d &pose_1,
      const ceres::examples::Pose3d &pose_2_relative);

  // Converts a rotation with roll, pitch, and yaw into a quaternion
  static Eigen::Quaterniond EulerAnglesToQuaternion(const Eigen::Vector3d &rpy);
  // Inverse of above function
  static Eigen::Vector3d QuaternionToEulerAngles(const Eigen::Quaterniond &q);
  // Converts a 3d rotation matrix into a rotation with roll, pitch, and yaw
  static Eigen::Vector3d RotationMatrixToEulerAngles(const Eigen::Matrix3d &R);
};

// Compute the average of a set of quaternions
Eigen::Vector4d QuaternionAverage(std::vector<Eigen::Vector4d> quaternions);

// Compute the average pose given a list of translations (as
// Eigen::Vector3d's) and rotations (as Eigen::Vector4d quaternions)
// Returned as an Eigen::Affine3d pose
// Also, compute the variance of each of these list of vectors
Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Vector3d> &translation_list,
    std::vector<Eigen::Vector4d> &rotation_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr);

// Helper function to compute average pose when supplied with list
// of Eigen::Affine3d's
Eigen::Affine3d ComputeAveragePose(
    std::vector<Eigen::Affine3d> &pose_list,
    Eigen::Vector3d *translation_variance = nullptr,
    Eigen::Vector3d *rotation_variance = nullptr);

}  // namespace frc::vision

#endif  // FRC_VISION_UTIL_VISION_UTIL_LIB_H_

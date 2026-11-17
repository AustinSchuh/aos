#include "frc/vision/util/vision_util_lib.h"

#include <numbers>

#include "absl/strings/str_format.h"
#include "gtest/gtest.h"

#include "aos/util/math.h"

namespace numbers = std::numbers;

namespace frc::vision {
// For now, just testing extracting camera number from channel name
TEST(VisionUtilsTest, CameraNumberFromChannel) {
  ASSERT_EQ(CameraNumberFromChannel("/camera0").value(), 0);
  ASSERT_EQ(CameraNumberFromChannel("/camera1").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/camera"), std::nullopt);
  ASSERT_EQ(CameraNumberFromChannel("/orin1/camera0").value(), 0);
  ASSERT_EQ(CameraNumberFromChannel("/orin1/camera1").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/orin1"), std::nullopt);
  ASSERT_EQ(CameraNumberFromChannel("/camera1/gray").value(), 1);
  ASSERT_EQ(CameraNumberFromChannel("/camera2/gray").value(), 2);
}

namespace {
constexpr double kToleranceRadians = 0.05;
// Conversions between euler angles and quaternion result in slightly-off
// doubles
constexpr double kOrientationEqTolerance = 1e-10;
}  // namespace

// Angles normalized by aos::math::NormalizeAngle()
#define EXPECT_NORMALIZED_ANGLES_NEAR(theta1, theta2, tolerance)           \
  {                                                                        \
    double delta = std::abs(aos::math::DiffAngle(theta1, theta2));         \
    /* Have to check delta - 2pi for the case that one angle is very */    \
    /* close to -pi, and the other is very close to +pi */                 \
    EXPECT_TRUE(delta < tolerance ||                                       \
                std::abs(aos::math::DiffAngle(delta, 2.0 * numbers::pi)) < \
                    tolerance);                                            \
  }

#define EXPECT_POSE_NEAR(pose1, pose2)                                     \
  {                                                                        \
    for (size_t i = 0; i < 3; i++) {                                       \
      EXPECT_NEAR(pose1.p(i), pose2.p(i), kToleranceMeters);               \
    }                                                                      \
    auto rpy_1 = PoseUtils::QuaternionToEulerAngles(pose1.q);              \
    auto rpy_2 = PoseUtils::QuaternionToEulerAngles(pose2.q);              \
    for (size_t i = 0; i < 3; i++) {                                       \
      SCOPED_TRACE(absl::StrFormat("rpy_1(%d) = %f, rpy_2(%d) = %f", i,    \
                                   rpy_1(i), i, rpy_2(i)));                \
      EXPECT_NORMALIZED_ANGLES_NEAR(rpy_1(i), rpy_2(i), kToleranceRadians) \
    }                                                                      \
  }

#define EXPECT_POSE_EQ(pose1, pose2) \
  EXPECT_EQ(pose1.p, pose2.p);       \
  EXPECT_EQ(pose1.q, pose2.q);

#define EXPECT_QUATERNION_NEAR(q1, q2)                                        \
  EXPECT_NEAR(q1.x(), q2.x(), kOrientationEqTolerance) << q1 << " != " << q2; \
  EXPECT_NEAR(q1.y(), q2.y(), kOrientationEqTolerance) << q1 << " != " << q2; \
  EXPECT_NEAR(q1.z(), q2.z(), kOrientationEqTolerance) << q1 << " != " << q2; \
  EXPECT_NEAR(q1.w(), q2.w(), kOrientationEqTolerance) << q1 << " != " << q2;

// Expects same roll, pitch, and yaw values (not equivalent rotation)
#define EXPECT_RPY_EQ(rpy_1, rpy_2)                                     \
  {                                                                     \
    for (size_t i = 0; i < 3; i++) {                                    \
      SCOPED_TRACE(absl::StrFormat("rpy_1(%d) = %f, rpy_2(%d) = %f", i, \
                                   rpy_1(i), i, rpy_2(i)));             \
      EXPECT_NORMALIZED_ANGLES_NEAR(rpy_1(i), rpy_2(i),                 \
                                    kOrientationEqTolerance)            \
    }                                                                   \
  }

#define EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(roll, pitch, yaw) \
  {                                                                        \
    auto rpy = Eigen::Vector3d(roll, pitch, yaw);                          \
    auto converted_rpy = PoseUtils::QuaternionToEulerAngles(               \
        PoseUtils::EulerAnglesToQuaternion(rpy));                          \
    EXPECT_RPY_EQ(converted_rpy, rpy);                                     \
  }

// Both confidence matrixes should have the same dimensions and be square
#define EXPECT_CONFIDENCE_GT(confidence1, confidence2) \
  {                                                    \
    ASSERT_EQ(confidence1.rows(), confidence2.rows()); \
    ASSERT_EQ(confidence1.rows(), confidence1.cols()); \
    ASSERT_EQ(confidence2.rows(), confidence2.cols()); \
    for (size_t i = 0; i < confidence1.rows(); i++) {  \
      EXPECT_GT(confidence1(i, i), confidence2(i, i)); \
    }                                                  \
  }

TEST(PoseUtilsTest, EulerAnglesAndQuaternionConversions) {
  // Make sure that the conversions are consistent back and forth.
  // These angles shouldn't get changed to a different, equivalent roll pitch
  // yaw.
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, numbers::pi);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, -numbers::pi);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, numbers::pi / 2.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0,
                                                   -numbers::pi / 2.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, 0.0, 0.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, numbers::pi / 4.0, 0.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, -numbers::pi / 4.0,
                                                   0.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(0.0, -numbers::pi / 4.0,
                                                   numbers::pi / 4.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(
      numbers::pi / 4.0, -numbers::pi / 4.0, numbers::pi / 4.0);
  EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(
      -numbers::pi / 2.0, -numbers::pi / 4.0, numbers::pi / 4.0);

  // Now, do a sweep of roll, pitch, and yaws in the normalized
  // range.
  // - roll: (-pi/2, pi/2)
  // - pitch: (-pi/2, pi/2)
  // - yaw: [-pi, pi)
  constexpr double kThetaMaxRoll = numbers::pi / 2.0 - kToleranceRadians;
  constexpr double kThetaMaxPitch = numbers::pi / 2.0 - kToleranceRadians;
  constexpr double kThetaMaxYaw = numbers::pi;
  constexpr double kDeltaTheta = numbers::pi / 16;

  for (double roll = -kThetaMaxRoll; roll < kThetaMaxRoll;
       roll += kDeltaTheta) {
    for (double pitch = -kThetaMaxPitch; pitch < kThetaMaxPitch;
         pitch += kDeltaTheta) {
      for (double yaw = -kThetaMaxYaw; yaw < kThetaMaxYaw; yaw += kDeltaTheta) {
        SCOPED_TRACE(
            absl::StrFormat("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw));
        EXPECT_EULER_ANGLES_QUATERNION_BACK_AND_FORTH_EQ(roll, pitch, yaw);
      }
    }
  }
}

// A wide lens, as Kannala-Brandt models it: projecting, undistorting and
// fitting a pose agree with each other.
TEST(CameraLensTest, KannalaBrandtRoundTrips) {
  const CameraLens lens{
      .camera_matrix =
          (cv::Mat_<double>(3, 3) << 648, 0, 766, 0, 648, 557, 0, 0, 1),
      .dist_coeffs = (cv::Mat_<double>(4, 1) << 0.025, 0.001, 0.004, -0.002),
      .kannala_brandt = true};
  // A grid 1.5 m away, out to 60 degrees off the axis.
  std::vector<cv::Point3f> object;
  for (int i = -4; i <= 4; ++i) {
    for (int j = -3; j <= 3; ++j) {
      object.emplace_back(i * 0.3, j * 0.3, 0);
    }
  }
  const cv::Vec3d rvec(0.1, -0.2, 0.05), tvec(0.1, -0.05, 1.5);
  const std::vector<cv::Point2f> image = lens.Project(object, rvec, tvec);

  // Undistorted, the points are where a pinhole camera puts them.
  std::vector<cv::Point2f> pinhole;
  cv::projectPoints(object, rvec, tvec, lens.camera_matrix, cv::Mat(), pinhole);
  const std::vector<cv::Point2f> undistorted = lens.Undistort(image);
  for (size_t i = 0; i < image.size(); ++i) {
    EXPECT_LT(cv::norm(undistorted[i] - pinhole[i]), 1e-3) << i;
  }

  cv::Vec3d found_rvec, found_tvec;
  ASSERT_TRUE(lens.SolvePnP(object, image, found_rvec, found_tvec,
                            cv::SOLVEPNP_ITERATIVE, true));
  EXPECT_LT(cv::norm(found_rvec - rvec), 1e-5);
  EXPECT_LT(cv::norm(found_tvec - tvec), 1e-5);
}

TEST(CameraLensTest, OpenCvModelIsOpenCvs) {
  const CameraLens lens{
      .camera_matrix =
          (cv::Mat_<double>(3, 3) << 1000, 0, 800, 0, 1000, 650, 0, 0, 1),
      .dist_coeffs = (cv::Mat_<double>(1, 5) << -0.2, 0.05, 0, 0, 0),
      .kannala_brandt = false};
  const std::vector<cv::Point3f> object = {{0.2, 0.1, 0}, {-0.3, 0.2, 0}};
  std::vector<cv::Point2f> expected;
  cv::projectPoints(object, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1),
                    lens.camera_matrix, lens.dist_coeffs, expected);
  const std::vector<cv::Point2f> projected =
      lens.Project(object, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 1));
  EXPECT_EQ(projected, expected);
}

}  // namespace frc::vision

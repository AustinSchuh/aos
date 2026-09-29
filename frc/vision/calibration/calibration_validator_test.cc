#include "frc/vision/calibration/calibration_validator.h"

#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "frc/vision/calibration_generated.h"
#include "frc/vision/camera_constants_list_generated.h"

namespace frc::vision::testing {
namespace {

using aos::testing::ArtifactPath;
using calibration::CameraCalibration;

aos::FlatbufferDetachedBuffer<CameraCalibration> MakeCalibration(
    const std::vector<float> &intrinsics,
    const std::vector<float> &dist_coeffs) {
  flatbuffers::FlatBufferBuilder fbb;
  const auto intrinsics_offset = fbb.CreateVector(intrinsics);
  const auto dist_coeffs_offset = fbb.CreateVector(dist_coeffs);
  CameraCalibration::Builder builder(fbb);
  builder.add_intrinsics(intrinsics_offset);
  builder.add_dist_coeffs(dist_coeffs_offset);
  fbb.Finish(builder.Finish());
  return aos::FlatbufferDetachedBuffer<CameraCalibration>(fbb.Release());
}

aos::FlatbufferDetachedBuffer<CameraCalibration> ReadCalibration(
    std::string_view path) {
  return aos::JsonFileToFlatbuffer<CameraCalibration>(ArtifactPath(path));
}

std::vector<float> Intrinsics(const CameraCalibration &calibration) {
  return {calibration.intrinsics()->begin(), calibration.intrinsics()->end()};
}

std::vector<float> DistCoeffs(const CameraCalibration &calibration) {
  return {calibration.dist_coeffs()->begin(), calibration.dist_coeffs()->end()};
}

// The 5 parameter model team 0's test calibrations used to hold, from a
// 1280x720 camera.  apriltag_benchmark still falls back to it.
const std::vector<float> kFallbackIntrinsics = {
    893.759521, 0, 645.470764, 0, 893.222351, 388.150269, 0, 0, 1};
const std::vector<float> kFallbackDistCoeffs = {-0.44902, 0.248409, -0.000537,
                                                -0.000112, -0.076989};

constexpr std::string_view k25_07 =
    "frc/vision/constants/"
    "calibration_orin1-4646-0_cam-25-07_2025-02-18_13-23-39.json";
constexpr std::string_view k25_09 =
    "frc/vision/constants/"
    "calibration_orin-4646-2_cam-25-09_2025-03-06_15-37-04.json";
constexpr std::string_view k25_11 =
    "frc/vision/constants/"
    "calibration_orin1-1868-0_cam-25-11_1970-01-01_03-17-57.json";

// Every calibration in frc/vision/constants, at the size its team's robots
// run at, passes, except those of 25-09, which fail in the right corners.
TEST(CalibrationValidatorTest, RepoCalibrations) {
  const aos::FlatbufferDetachedBuffer<CameraConstantsList> constants =
      aos::JsonFileToFlatbuffer<CameraConstantsList>(
          ArtifactPath("frc/vision/constants.json"));
  std::map<int, std::pair<int, int>> team_sizes;
  for (const TeamAndCameraConstants *entry : *constants.message().constants()) {
    const CameraStreamSettings *settings =
        entry->data()->default_camera_stream_settings();
    const std::pair<int, int> size(settings->image_width(),
                                   settings->image_height());
    auto [it, inserted] = team_sizes.emplace(entry->team(), size);
    ASSERT_EQ(it->second, size) << ": team " << entry->team();
  }

  int validated = 0;
  for (const std::filesystem::directory_entry &file :
       std::filesystem::directory_iterator(
           ArtifactPath("frc/vision/constants"))) {
    const aos::FlatbufferDetachedBuffer<CameraCalibration> calibration =
        aos::JsonFileToFlatbuffer<CameraCalibration>(file.path().string());
    const auto size = team_sizes.find(calibration.message().team_number());
    ASSERT_NE(size, team_sizes.end()) << file.path();
    CalibrationReport report;
    const absl::Status status =
        ValidateCalibration(calibration.message(), size->second.first,
                            size->second.second, &report);
    ABSL_LOG(INFO) << file.path().filename() << ": " << report;
    ++validated;
    if (calibration.message().has_camera_id() &&
        calibration.message().camera_id()->string_view() == "25-09") {
      EXPECT_FALSE(status.ok()) << file.path();
      EXPECT_GT(report.unconverged_pixels, 0) << file.path();
      EXPECT_GT(report.failures_by_quadrant[1], 0) << file.path();
      EXPECT_GT(report.failures_by_quadrant[3], 0) << file.path();
      EXPECT_EQ(report.failures_by_quadrant[2], 0) << file.path();
    } else {
      EXPECT_TRUE(status.ok()) << file.path() << ": " << status;
    }
  }
  EXPECT_GT(validated, 20);
}

// The fallback model folds at 685 px, inside even its own 1280x720 image, so
// UnDistort has nothing to converge to in its corners.
TEST(CalibrationValidatorTest, FallbackFolds) {
  const aos::FlatbufferDetachedBuffer<CameraCalibration> calibration =
      MakeCalibration(kFallbackIntrinsics, kFallbackDistCoeffs);
  for (const auto &[width, height] :
       std::vector<std::pair<int, int>>{{1280, 720}, {1456, 1088}}) {
    CalibrationReport report;
    EXPECT_FALSE(
        ValidateCalibration(calibration.message(), width, height, &report)
            .ok());
    ABSL_LOG(INFO) << report;
    EXPECT_EQ(report.failed_check, CalibrationCheck::kRadialModel);
    EXPECT_STREQ(report.limit_reason, "fold, f' <= 0");
    EXPECT_NEAR(report.limit_distorted_radius * kFallbackIntrinsics[0], 685, 1);
    EXPECT_LT(report.distorted_margin, 0);
    EXPECT_TRUE(std::isnan(report.max_undistorted_radius));
    for (const int64_t failures : report.failures_by_quadrant) {
      EXPECT_GT(failures, 0);
    }
  }

  const apriltag::CameraMatrix camera_matrix =
      apriltag::GetCameraMatrix(&calibration.message());
  const apriltag::DistCoeffs dist_coeffs =
      apriltag::GetDistCoeffs(&calibration.message());
  for (const auto &[u, v] : std::vector<std::pair<double, double>>{
           {0, 0}, {1279, 0}, {0, 719}, {1279, 719}}) {
    double undistorted_u = u, undistorted_v = v;
    EXPECT_FALSE(apriltag::UnDistort(&undistorted_u, &undistorted_v,
                                     camera_matrix, dist_coeffs))
        << u << ", " << v;
  }
}

// 25-09's model nearly cancels just past the image, and UnDistort's
// iteration repels in the right corners.  It starts there within 1% of the
// fold, in undistorted radius, which doesn't matter since N / D is well
// behaved past it: only the pixel check fails.  Only the waiver lets
// blackbeard use it.
TEST(CalibrationValidatorTest, Blackbeard25_09Repels) {
  const aos::FlatbufferDetachedBuffer<CameraCalibration> calibration =
      ReadCalibration(k25_09);
  CalibrationReport report;
  const absl::Status status =
      ValidateCalibration(calibration.message(), 1600, 1304, &report);
  ABSL_LOG(INFO) << status;
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(report.failed_check, CalibrationCheck::kPixels);
  EXPECT_STREQ(report.limit_reason, "fold, f' <= 0");
  EXPECT_LT(report.limit_radius, 1.01 * report.max_distorted_radius);
  EXPECT_GT(report.distorted_margin, kRadialMargin);
  EXPECT_GT(report.undistorted_margin, kRadialMargin);
  EXPECT_GT(report.unconverged_pixels, 2000);
  EXPECT_GT(report.slow_pixels, 0);
  EXPECT_EQ(report.off_branch_pixels, 0);
  EXPECT_GT(report.nearest_failure_pixels, 1000);
  // Every failure lies inside the detector's default --pixel_border, though
  // not the 25 px the robots run with.
  EXPECT_GT(report.deepest_failure_pixels, 25);
  EXPECT_LT(report.deepest_failure_pixels, 50);

  // The top right corner gives up far from any answer.
  const apriltag::CameraMatrix camera_matrix =
      apriltag::GetCameraMatrix(&calibration.message());
  const apriltag::DistCoeffs dist_coeffs =
      apriltag::GetDistCoeffs(&calibration.message());
  double u = 1599, v = 0;
  EXPECT_FALSE(apriltag::UnDistort(&u, &v, camera_matrix, dist_coeffs));
  apriltag::ReDistort(&u, &v, camera_matrix, dist_coeffs);
  EXPECT_GT(std::hypot(u - 1599, v), 1);
}

// 1868's 25-11 has a pole at undistorted radius 0.95.  At 1280x800 the image
// ends well inside it, but at 1600x1304 UnDistort starts past it in the
// bottom right, and converges to a root on the far side.
TEST(CalibrationValidatorTest, Team1868PoleInsideTheBiggerImage) {
  const aos::FlatbufferDetachedBuffer<CameraCalibration> calibration =
      ReadCalibration(k25_11);
  CalibrationReport report;
  EXPECT_TRUE(
      ValidateCalibration(calibration.message(), 1280, 800, &report).ok());
  EXPECT_STREQ(report.limit_reason, "pole, D <= 0");
  EXPECT_GE(report.undistorted_margin, kRadialMargin);

  EXPECT_FALSE(
      ValidateCalibration(calibration.message(), 1600, 1304, &report).ok());
  EXPECT_EQ(report.failed_check, CalibrationCheck::kRadialModel);
  EXPECT_STREQ(report.limit_reason, "pole, D <= 0");
  EXPECT_LT(report.undistorted_margin, 0);
  EXPECT_GT(report.off_branch_pixels, 200000);

  // Grown 18% about its center, the image's corners start 2% short of the
  // pole.  UnDistort still handles every point, but the radial check wants
  // 3%.
  std::vector<float> intrinsics = Intrinsics(calibration.message());
  intrinsics[2] += 115;
  intrinsics[5] += 72;
  EXPECT_FALSE(
      ValidateCalibration(
          MakeCalibration(intrinsics, DistCoeffs(calibration.message()))
              .message(),
          1510, 944, &report)
          .ok());
  EXPECT_EQ(report.failed_check, CalibrationCheck::kRadialModel);
  EXPECT_STREQ(report.singular_reason, "pole, D <= 0");
  EXPECT_GT(report.undistorted_margin, 0.01);
  EXPECT_LT(report.undistorted_margin, kRadialMargin);
  EXPECT_GT(report.distorted_margin, kRadialMargin);
  EXPECT_EQ(report.failing_pixels(), 0);

  // The far corner converges quickly and ReDistorts back exactly, but to a
  // point past the pole: only the branch check catches it.
  const apriltag::CameraMatrix camera_matrix =
      apriltag::GetCameraMatrix(&calibration.message());
  const apriltag::DistCoeffs dist_coeffs =
      apriltag::GetDistCoeffs(&calibration.message());
  double u = 1600, v = 1304;
  int iterations;
  EXPECT_TRUE(
      apriltag::UnDistort(&u, &v, camera_matrix, dist_coeffs, &iterations));
  EXPECT_LE(iterations, kMaxUnDistortIterations);
  EXPECT_GT(std::hypot((u - camera_matrix.cx) / camera_matrix.fx,
                       (v - camera_matrix.cy) / camera_matrix.fy),
            report.limit_radius);
  apriltag::ReDistort(&u, &v, camera_matrix, dist_coeffs);
  EXPECT_LT(std::hypot(u - 1600, v - 1304), report.residual_limit_pixels);
}

TEST(CalibrationValidatorTest, ZeroDistortion) {
  CalibrationReport report;
  EXPECT_TRUE(
      ValidateCalibration(MakeCalibration({1000, 0, 800, 0, 1000, 652, 0, 0, 1},
                                          {0, 0, 0, 0, 0})
                              .message(),
                          1600, 1304, &report)
          .ok());
  EXPECT_EQ(report.max_iterations, 1);
  EXPECT_EQ(report.pixels_checked, 1602 * 1306);
  EXPECT_STREQ(report.limit_reason, "where the scan stopped");
  EXPECT_GE(report.limit_radius, 2 * report.max_distorted_radius);
}

// Each malformed calibration fails the sanity check, before anything
// evaluates the model.
TEST(CalibrationValidatorTest, Sanity) {
  const aos::FlatbufferDetachedBuffer<CameraCalibration> good =
      ReadCalibration(k25_07);
  ASSERT_TRUE(ValidateCalibration(good.message(), 1600, 1304).ok());
  const std::vector<float> intrinsics = Intrinsics(good.message());
  const std::vector<float> dist_coeffs = DistCoeffs(good.message());
  ASSERT_EQ(dist_coeffs.size(), 14u);

  const auto expect_insane = [](const std::vector<float> &intrinsics,
                                const std::vector<float> &dist_coeffs,
                                std::string_view message, int width = 1600,
                                int height = 1304) {
    CalibrationReport report;
    const absl::Status status =
        ValidateCalibration(MakeCalibration(intrinsics, dist_coeffs).message(),
                            width, height, &report);
    EXPECT_FALSE(status.ok()) << message;
    EXPECT_NE(status.message().find(message), std::string_view::npos) << status;
    EXPECT_EQ(report.failed_check, CalibrationCheck::kSanity) << message;
    EXPECT_EQ(report.pixels_checked, 0) << message;
  };

  // The detector would silently ignore a 9th coefficient.
  std::vector<float> ninth = dist_coeffs;
  ninth[8] = 1e-3;
  expect_insane(intrinsics, ninth, "dist_coeffs[8] is 0.001");
  std::vector<float> fourteenth = dist_coeffs;
  fourteenth[13] = -1e-3;
  expect_insane(intrinsics, fourteenth, "dist_coeffs[13]");

  expect_insane(intrinsics, {dist_coeffs.begin(), dist_coeffs.begin() + 4},
                "has 4 coefficients");
  expect_insane(intrinsics, {dist_coeffs.begin(), dist_coeffs.begin() + 12},
                "has 12 coefficients");
  std::vector<float> nan_coefficient = dist_coeffs;
  nan_coefficient[1] = NAN;
  expect_insane(intrinsics, nan_coefficient, "not all finite");

  std::vector<float> bad = intrinsics;
  bad[0] = 0;
  expect_insane(bad, dist_coeffs, "must be positive");
  bad = intrinsics;
  bad[4] = -bad[4];
  expect_insane(bad, dist_coeffs, "must be positive");
  bad = intrinsics;
  bad[2] = INFINITY;
  expect_insane(bad, dist_coeffs, "not all finite");
  bad = intrinsics;
  bad[1] = 0.5;
  expect_insane(bad, dist_coeffs, "not of the form");
  expect_insane({intrinsics.begin(), intrinsics.begin() + 6}, dist_coeffs,
                "has 6 values");

  // 25-07's principal point is (779, 629).
  expect_insane(intrinsics, dist_coeffs, "outside the 700x1304 image", 700,
                1304);
  expect_insane(intrinsics, dist_coeffs, "outside the 1600x600 image", 1600,
                600);
  expect_insane(intrinsics, dist_coeffs, "is empty", 0, 1304);
}

// Barrel models which fold 5% and 2% past the corners.  UnDistort handles
// every point of both, taking 21 and 31 steps at the corners, but the radial
// check wants 3%.
TEST(CalibrationValidatorTest, RadialMargin) {
  const std::vector<float> intrinsics = {1000, 0, 640, 0, 1000, 400, 0, 0, 1};
  CalibrationReport report;
  EXPECT_TRUE(ValidateCalibration(
                  MakeCalibration(intrinsics, {-0.235, 0, 0, 0, 0}).message(),
                  1280, 800, &report)
                  .ok());
  EXPECT_STREQ(report.limit_reason, "fold, f' <= 0");
  EXPECT_NEAR(report.distorted_margin, 0.05, 0.005);

  EXPECT_FALSE(ValidateCalibration(
                   MakeCalibration(intrinsics, {-0.249, 0, 0, 0, 0}).message(),
                   1280, 800, &report)
                   .ok());
  EXPECT_EQ(report.failed_check, CalibrationCheck::kRadialModel);
  EXPECT_STREQ(report.limit_reason, "fold, f' <= 0");
  EXPECT_GT(report.distorted_margin, 0.01);
  EXPECT_LT(report.distorted_margin, kRadialMargin);
  EXPECT_EQ(report.failing_pixels(), 0);
}

// A model which is strongly pincushion near the edge, so UnDistort starts
// 1.3% short of its fold in the corners, as with 25-09.  N / D is well
// behaved past the fold, UnDistort converges everywhere within 17 steps, and
// it passes.
TEST(CalibrationValidatorTest, StartNearAFold) {
  CalibrationReport report;
  const absl::Status status = ValidateCalibration(
      MakeCalibration({1000, 0, 800, 0, 1000, 640, 0, 0, 1},
                      {1.67472, -3.05519, -0.000495814, 0.000485415, 1.31357,
                       0.993774, -2.81003, 1.4695})
          .message(),
      1600, 1280, &report);
  EXPECT_TRUE(status.ok()) << status;
  EXPECT_STREQ(report.limit_reason, "fold, f' <= 0");
  EXPECT_LT(report.limit_radius, 1.02 * report.max_distorted_radius);
  EXPECT_GE(report.undistorted_margin, kRadialMargin);
  EXPECT_LE(report.max_iterations, 20);
}

// UnDistort stops once a step is within a normalized tolerance, so the
// residual it leaves grows with the focal length.  A mild barrel lens on a
// 3200x2608 sensor with fx 4000 leaves over 1e-3 px after 10 steps, and
// passes.
TEST(CalibrationValidatorTest, ResidualScalesWithFocalLength) {
  CalibrationReport report;
  const absl::Status status = ValidateCalibration(
      MakeCalibration({4000, 0, 1600, 0, 4000, 1304, 0, 0, 1},
                      {-0.35, 0, 0, 0, 0})
          .message(),
      3200, 2608, &report);
  EXPECT_TRUE(status.ok()) << status;
  EXPECT_LE(report.max_iterations, 10);
  EXPECT_GT(report.max_residual_pixels, 1e-3);
  EXPECT_NEAR(report.residual_limit_pixels, 0.012, 1e-9);
}

// A model with no fold or pole, which R = (1 + s) / (1 + 6 s) compresses so
// hard that UnDistort takes up to 65 steps, which is too close to giving up.
TEST(CalibrationValidatorTest, IterationLimit) {
  CalibrationReport report;
  EXPECT_FALSE(
      ValidateCalibration(MakeCalibration({1000, 0, 640, 0, 1000, 400, 0, 0, 1},
                                          {1, 0, 0, 0, 0, 6, 0, 0})
                              .message(),
                          1280, 800, &report)
          .ok());
  EXPECT_EQ(report.failed_check, CalibrationCheck::kPixels);
  EXPECT_STREQ(report.limit_reason, "where the scan stopped");
  EXPECT_GT(report.slow_pixels, 0);
  EXPECT_EQ(report.unconverged_pixels, 0);
  EXPECT_EQ(report.off_branch_pixels, 0);
  EXPECT_EQ(report.inexact_pixels, 0);
  EXPECT_GT(report.max_iterations, kMaxUnDistortIterations);
  EXPECT_LE(report.max_iterations, apriltag::kUndistortIterationThreshold);
}

}  // namespace
}  // namespace frc::vision::testing

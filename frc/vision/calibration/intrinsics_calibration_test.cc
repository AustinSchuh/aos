#include <filesystem>
#include <map>
#include <numbers>
#include <random>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/status/status.h"
#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/random_seed.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/math.h"
#include "frc/vision/calibration/calibration_validator.h"
#include "frc/vision/calibration/intrinsics_calibration_lib.h"
#include "tools/cpp/runfiles/runfiles.h"

ABSL_DECLARE_FLAG(bool, large_board);
ABSL_DECLARE_FLAG(bool, coarse_pattern);
ABSL_DECLARE_FLAG(bool, dict5x5_9x14_board);
ABSL_DECLARE_FLAG(bool, write_invalid_calibration);

namespace numbers = std::numbers;

namespace frc::vision {
aos::distributed_clock::time_point TimeInMs(size_t ms) {
  return aos::distributed_clock::time_point(std::chrono::milliseconds(ms));
}
namespace fs = std::filesystem;
using aos::testing::ArtifactPath;

void CheckAgainstKnownCalibration(IntrinsicsCalibration &calibrator) {
  // Retrieve the original camera intrinsics
  cv::Mat camera_mat = calibrator.GetCharucoExtractor().camera_matrix();
  cv::Mat dist_coeffs = calibrator.GetCharucoExtractor().dist_coeffs();

  // Retrieve the computed camera intrinsics
  cv::Mat calculated_camera_mat = calibrator.GetCameraMatrix();
  cv::Mat calculated_dist_coeffs = calibrator.GetDistortionCoefficients();

  // Check that our estimates are close to the correct values
  cv::Mat diff_camera_mat;
  cv::Mat diff_dist_coeffs;
  // Have to force add into CV_32F, since calculated_XXX is CV_64F
  cv::add(camera_mat, -calculated_camera_mat, diff_camera_mat, cv::noArray(),
          CV_32F);
  cv::add(dist_coeffs, -calculated_dist_coeffs, diff_dist_coeffs, cv::noArray(),
          CV_32F);

  ABSL_VLOG(1) << "Norm of diffs is " << cv::norm(diff_camera_mat) << " and "
               << cv::norm(diff_dist_coeffs);

  const double kDiffThreshold = 1e-3;
  EXPECT_NEAR(cv::norm(diff_camera_mat), 0.0, kDiffThreshold);
  EXPECT_NEAR(cv::norm(diff_dist_coeffs), 0.0, kDiffThreshold);
}

// With the specified camera calibration file, project a series of boards
// (using their corner points) into the image plane at different poses, and
// generate and check the intrinsic calibration that is computed based on those
// points.
//
// Allow for different intrinsics (camera matrix and distortion coefficients) to
// be used as initial conditions for the solver.  Returns what MaybeCalibrate
// does.
absl::Status RunIntrinsicFromPoints(std::string calib_filename,
                                    cv::Mat camera_mat_init = cv::Mat(),
                                    cv::Mat dist_coeffs_init = cv::Mat()) {
  absl::SetFlag(&FLAGS_override_hostname, "orin-971-1");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("frc/vision/calibration/intrinsics_test_config.bfbs"));
  aos::ShmEventLoop event_loop(&config.message());
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();
  IntrinsicsCalibration calibrator(&event_loop, "orin-971-1", "/orin1/camera0",
                                   "24-00", calib_filename.c_str(), false,
                                   "/tmp", exit_handle.get());
  // We disable visualize for this test, so we don't try to draw things
  absl::SetFlag(&FLAGS_visualize, false);

  cv::Mat zero_vec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Size image_size = cv::Size(1456, 1088);
  cv::Ptr<cv::aruco::CharucoBoard> board =
      calibrator.GetCharucoExtractor().board();

  // Retrieve the camera parameters from the config
  cv::Mat camera_mat = calibrator.GetCharucoExtractor().camera_matrix();
  cv::Mat dist_coeffs = calibrator.GetCharucoExtractor().dist_coeffs();

  // If we're not given initial camera parameters, use the actual camera
  // parameters as initial guesses
  if (camera_mat_init.empty()) {
    camera_mat_init = camera_mat;
  }
  // And same with distortion coefficients
  if (dist_coeffs_init.empty()) {
    dist_coeffs_init = dist_coeffs;
  }
  calibrator.SetCameraMatrix(camera_mat_init);
  calibrator.SetDistortionCoefficients(dist_coeffs_init);

  if (dist_coeffs.size().height == 5) {
    ABSL_LOG(INFO) << "Using 5 parameter traditional model";
    absl::SetFlag(&FLAGS_use_rational_model, false);
  } else {
    ABSL_LOG(INFO) << "Using 8 parameter rational model";
    absl::SetFlag(&FLAGS_use_rational_model, true);
  }

  // Choose reasonable values for starting point of board
  Eigen::Matrix3f R_c_1;
  R_c_1 = Eigen::AngleAxis<float>(0.0, Eigen::Vector3f::UnitX());
  Eigen::Vector3f T_c_1(-0.3, 0.3, 0.3);
  Eigen::Affine3f H_c_1 = Eigen::Translation3f(T_c_1) * R_c_1;

  // Storage to collect all the projected corners and ids
  std::vector<std::vector<int>> all_corner_ids;
  std::vector<std::vector<cv::Point2f>> all_corners;

  bool should_break = false;
  // Iterate through a bunch of poses so that we present the board at enough
  // views to cover the image plane with corner points
  for (float theta_x = 0.0; theta_x <= numbers::pi / 6 && !should_break;
       theta_x += numbers::pi / 8) {
    for (float theta_y = 0.0; theta_y <= numbers::pi / 6 && !should_break;
         theta_y += numbers::pi / 8) {
      for (float depth = 0.6; depth >= 0.0 && !should_break; depth -= 0.2) {
        for (float x = -0.5; x <= 0.5 && !should_break; x += 0.5) {
          for (float y = -1.0; y <= 0.0 && !should_break; y += 0.5) {
            // Go until we've captured enough samples
            if (all_corner_ids.size() > 75) {
              int total_num_ids = 0;
              for (auto corner_ids : all_corner_ids) {
                total_num_ids += corner_ids.size();
              }
              ABSL_LOG(INFO)
                  << "Got enough board captures: " << all_corner_ids.size()
                  << " with " << total_num_ids << " corners";
              should_break = true;
              break;
            }

            // Create the transform from starting pose (1) to target pose (2)
            Eigen::Matrix3f R_1_2;
            R_1_2 = Eigen::AngleAxis<float>(theta_x, Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxis<float>(theta_y, Eigen::Vector3f::UnitY());
            Eigen::Vector3f T_1_2(x, y, depth);
            Eigen::Affine3f H_1_2 = Eigen::Translation3f(T_1_2) * R_1_2;
            Eigen::Affine3f H_c_2 = H_c_1 * H_1_2;

            // Create set of corners and ids that are projected into this view
            // and are within our image boundaries
            std::vector<int> corner_id_list;
            std::vector<cv::Point2f> corner_list;

            cv::Mat point_viz = cv::Mat::zeros(image_size, CV_8UC3);
            auto chessboard_corners =
#if CV_VERSION_MINOR >= 9
                board->getChessboardCorners();
#else
                board->chessboardCorners;
#endif
            for (size_t i = 0; i < chessboard_corners.size(); i++) {
              // Transform the ith corner into its new point location
              Eigen::Vector3f point_eigen;
              cv::cv2eigen(cv::Mat(chessboard_corners[i]), point_eigen);
              Eigen::Vector3f new_point =
                  (H_c_2 * Eigen::Translation3f(point_eigen)).translation();
              cv::Mat new_point_cv;
              cv::eigen2cv(new_point, new_point_cv);
              // Project the new 3D location of the point to th eimage plane
              cv::Mat new_points_proj(2, 1, CV_32F);
              cv::projectPoints(new_point_cv, zero_vec, zero_vec, camera_mat,
                                dist_coeffs, new_points_proj);
              float u = new_points_proj.at<float>(0, 0);
              float v = new_points_proj.at<float>(0, 1);
              if (u >= 0 && u <= image_size.width && v >= 0 &&
                  v <= image_size.height) {
                int id = static_cast<int>(i);
                cv::Point2f corner_proj(u, v);
                corner_id_list.push_back(id);
                corner_list.push_back(corner_proj);
                cv::circle(point_viz, corner_proj, 3, cv::Scalar(0, 255, 0));
              }
            }
            // For visualizing the corners that are generated
            // cv::imshow("point_viz", point_viz);
            // cv::waitKey(0);

            // Require that we have at least 15 visible corners in order to push
            // them back on our stack of valid projected corners
            if (corner_id_list.size() > 15) {
              all_corner_ids.push_back(corner_id_list);
              all_corners.push_back(corner_list);
            }
          }
        }
      }
    }
  }

  // Set the corners in our intrinsic calibration routine
  calibrator.SetCharucoIds(all_corner_ids);
  calibrator.SetCharucoCorners(all_corners);

  cv::Mat blank_image = cv::Mat::zeros(image_size, CV_8UC3);
  // Push image size by sending a blank image of the target size
  calibrator.GetCharucoExtractor().HandleImage(blank_image,
                                               aos::monotonic_clock::now());
  // Run the calibration
  const absl::Status status = calibrator.MaybeCalibrate();

  {
    // For visualizing all points that have been captured
    cv::Mat all_point_viz = cv::Mat::zeros(image_size, CV_8UC3);
    // Draw a small circle for each corner
    for (uint j = 0; j < all_corners.size(); j++) {
      for (uint i = 0; i < all_corners[j].size(); i++) {
        cv::Point2f point = all_corners[j][i];
        cv::circle(all_point_viz, point, 3, cv::Scalar(255, 255, 255));
      }
    }
    //  cv::imshow("all_point_viz", all_point_viz);
    //  cv::waitKey(0);
  }

  {
    // Iterate through the points and visualize versus computed
    for (uint i = 0; i < all_corners.size(); i++) {
      cv::Mat display_image = cv::Mat::zeros(image_size, CV_8UC3);
      // Draw original points
      for (uint j = 0; j < all_corners[i].size(); j++) {
        cv::Point2f point = all_corners[i][j];
        // point = point;
        cv::circle(display_image, point, 3, cv::Scalar(255, 255, 255));
      }
      // Retrieve the computed camera intrinsics
      cv::Mat calculated_camera_mat = calibrator.GetCameraMatrix();
      cv::Mat calculated_dist_coeffs = calibrator.GetDistortionCoefficients();

      // Draw projected points with new calibration
      calibrator.DrawCornersOnImage(
          display_image, i, calibrator.GetTVecs(), calibrator.GetRVecs(),
          calculated_camera_mat, calculated_dist_coeffs);
      //    cv::imshow("compare points", display_image);
      //    cv::waitKey(30);
    }
  }

  CheckAgainstKnownCalibration(calibrator);
  // Reprojection error should be nearly exact
  EXPECT_NEAR(calibrator.GetReprojectionError(), 0.0, 0.001);
  return status;
}

TEST(IntrinsicCalculationTest, SamplingProjectedPoints) {
  // Run using old 5 parameter model (which requires distortion coefficients
  // to be zero-- otherwise our reprojections don't work near the corners of
  // the image)
  EXPECT_TRUE(RunIntrinsicFromPoints(
                  ArtifactPath("frc/vision/test_calib_files/"
                               "calibration_orin-971-1_cam-24-00.json"))
                  .ok());

  // Run the new 8 parameter rational model
  EXPECT_TRUE(
      RunIntrinsicFromPoints(
          ArtifactPath("frc/vision/test_calib_files/"
                       "calibration_orin-971-1_cam-24-00_8parameter.json"))
          .ok());

  // Run the 8 parameter model using rough values for the camera matrix and
  // distortion coeficients to check it converges properly
  cv::Mat camera_mat_init = cv::Mat::eye(3, 3, CV_32F);
  camera_mat_init.at<float>(0, 0) = 650.0;
  camera_mat_init.at<float>(1, 1) = 650.0;
  camera_mat_init.at<float>(0, 2) = 1456 / 2.0;
  camera_mat_init.at<float>(1, 2) = 1088 / 2.0;
  cv::Mat dist_coeffs_init = cv::Mat::zeros(8, 1, CV_32F);

  EXPECT_TRUE(
      RunIntrinsicFromPoints(
          ArtifactPath("frc/vision/test_calib_files/"
                       "calibration_orin-971-1_cam-24-00_8parameter.json"),
          camera_mat_init, dist_coeffs_init)
          .ok());
}

// The 5 parameter model team 0's test calibrations used to hold folds inside
// its 1280x720 image, so WriteValidatedCalibration refuses to write it unless
// told to.  25-07 is valid, and written.
TEST(IntrinsicCalculationTest, WritesOnlyValidCalibrations) {
  const std::string folder = aos::testing::TestTmpDir() + "/write_validated";
  std::filesystem::remove_all(folder);
  std::filesystem::create_directories(folder);

  flatbuffers::FlatBufferBuilder fbb;
  const auto intrinsics = fbb.CreateVector<float>(
      {893.759521, 0, 645.470764, 0, 893.222351, 388.150269, 0, 0, 1});
  const auto dist_coeffs = fbb.CreateVector<float>(
      {-0.44902, 0.248409, -0.000537, -0.000112, -0.076989});
  calibration::CameraCalibration::Builder builder(fbb);
  builder.add_intrinsics(intrinsics);
  builder.add_dist_coeffs(dist_coeffs);
  fbb.Finish(builder.Finish());
  const aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> fallback(
      fbb.Release());

  const std::string invalid = folder + "/invalid.json";
  EXPECT_EQ(IntrinsicsCalibration::WriteValidatedCalibration(
                fallback.message(), cv::Size(1280, 720), invalid)
                .code(),
            absl::StatusCode::kFailedPrecondition);
  EXPECT_FALSE(std::filesystem::exists(invalid));

  absl::SetFlag(&FLAGS_write_invalid_calibration, true);
  EXPECT_TRUE(IntrinsicsCalibration::WriteValidatedCalibration(
                  fallback.message(), cv::Size(1280, 720), invalid)
                  .ok());
  absl::SetFlag(&FLAGS_write_invalid_calibration, false);
  EXPECT_TRUE(std::filesystem::exists(invalid));

  const std::string valid = folder + "/valid.json";
  EXPECT_TRUE(IntrinsicsCalibration::WriteValidatedCalibration(
                  aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
                      ArtifactPath("frc/vision/constants/"
                                   "calibration_orin1-4646-0_cam-25-07_2025-02-"
                                   "18_13-23-39.json"))
                      .message(),
                  cv::Size(1600, 1304), valid)
                  .ok());
  EXPECT_TRUE(std::filesystem::exists(valid));
}

// Test intrinsic calibration when loading from a series of images from disk
TEST(IntrinsicCalculationTest, ImagePlayback) {
  absl::SetFlag(&FLAGS_coarse_pattern, true);
  absl::SetFlag(&FLAGS_large_board, true);
  absl::SetFlag(&FLAGS_dict5x5_9x14_board, false);
  absl::SetFlag(&FLAGS_override_hostname, "orin-971-1");
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("frc/vision/calibration/intrinsics_test_config.bfbs"));
  aos::ShmEventLoop event_loop(&config.message());
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();

  // Use the 8 parameter intrinsic calibraiotn model
  absl::SetFlag(&FLAGS_use_rational_model, true);

  IntrinsicsCalibration calibrator(
      &event_loop, "orin-971-1", "/orin1/camera0", "24-00",
      ArtifactPath("frc/vision/test_calib_files/"
                   "calibration_orin-971-1_cam-24-00_8parameter.json"),
      false, "/tmp", exit_handle.get());

  // We disable visualize for this test, so we don't try to draw things
  absl::SetFlag(&FLAGS_visualize, false);

  // Run the calibration from files
  using bazel::tools::cpp::runfiles::Runfiles;
  std::string error;
  std::unique_ptr<Runfiles> runfiles(
      Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY, &error));
  ABSL_CHECK(runfiles != nullptr) << error;
  std::filesystem::path test_images_path(
      runfiles->Rlocation("intrinsic_calibration_test_images/img_000001.png"));
  test_images_path = test_images_path.parent_path();
  ABSL_LOG(INFO) << "Running intrinsics from disk from path: "
                 << test_images_path.string();
  absl::SetFlag(&FLAGS_image_load_path, test_images_path.string());
  calibrator.LoadImagesFromPath(test_images_path);
  EXPECT_TRUE(calibrator.MaybeCalibrate().ok());

  CheckAgainstKnownCalibration(calibrator);
  // Validate that reprojection error is < 1
  EXPECT_LT(calibrator.GetReprojectionError(), 1.0);
  EXPECT_GT(calibrator.NumCaptures(), 50);
}

// Each lens model, fitted to half of 99 real captures of a wide (about 97
// degree) lens and measured against the other half: what a fit's own error
// can't show.  OpenCV's 5 terms can't describe the lens.  The rational model
// and Kannala-Brandt fit it equally well, but the rational model's 8 terms
// trade off and fold outside the data (calibration_validator refuses them)
// unless a prior holds the denominator near 0, which costs accuracy;
// Kannala-Brandt's 4 terms stay small.  Logs a line per model.
TEST(IntrinsicCalculationTest, CompareModels) {
  // The images are of the large board, with its coarse pattern.
  absl::SetFlag(&FLAGS_coarse_pattern, true);
  absl::SetFlag(&FLAGS_large_board, true);
  absl::SetFlag(&FLAGS_dict5x5_9x14_board, false);
  absl::SetFlag(&FLAGS_override_hostname, "orin-971-1");
  absl::SetFlag(&FLAGS_use_rational_model, true);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("frc/vision/calibration/intrinsics_test_config.bfbs"));
  aos::ShmEventLoop event_loop(&config.message());
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop.MakeExitHandle();
  IntrinsicsCalibration calibrator(
      &event_loop, "orin-971-1", "/orin1/camera0", "24-00",
      ArtifactPath("frc/vision/test_calib_files/"
                   "calibration_orin-971-1_cam-24-00_8parameter.json"),
      false, "/tmp", exit_handle.get());
  absl::SetFlag(&FLAGS_visualize, false);
  using bazel::tools::cpp::runfiles::Runfiles;
  std::string error;
  std::unique_ptr<Runfiles> runfiles(
      Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY, &error));
  std::filesystem::path path(
      runfiles->Rlocation("intrinsic_calibration_test_images/img_000001.png"));
  absl::SetFlag(&FLAGS_image_load_path, path.parent_path().string());
  calibrator.LoadImagesFromPath(path.parent_path());
  const std::vector<CharucoCapture> captures = calibrator.Captures();
  std::vector<CharucoCapture> train, test;
  for (size_t i = 0; i < captures.size(); ++i) {
    (i % 2 == 0 ? train : test).push_back(captures[i]);
  }
  const cv::Mat guess = calibrator.GetCharucoExtractor().camera_matrix();
  struct Variant {
    std::string name;
    SolveOptions options;
  };
  std::map<std::string, double> held_out;
  for (const Variant &variant : std::vector<Variant>{
           {"5-term", {LensModel::kOpenCv5, 0.0}},
           {"rational", {LensModel::kOpenCvRational, 0.0}},
           {"rational, prior 1", {LensModel::kOpenCvRational, 1.0}},
           {"rational, prior 0.1", {LensModel::kOpenCvRational, 0.1}},
           {"rational, prior 0.01", {LensModel::kOpenCvRational, 0.01}},
           {"kannala-brandt", {LensModel::kKannalaBrandt, 0.0}},
       }) {
    // Half, all, and half again starting from no distortion, as a camera with
    // no calibration would.
    for (const auto [all, from_zero] :
         {std::pair{false, false}, std::pair{true, false},
          std::pair{false, true}}) {
      absl::StatusOr<IntrinsicsSolution> solution = SolveIntrinsics(
          calibrator.GetCharucoExtractor().board(), all ? captures : train,
          calibrator.ImageSize(), guess,
          from_zero ? cv::Mat::zeros(1, 8, CV_64F)
                    : calibrator.GetCharucoExtractor().dist_coeffs(),
          variant.options);
      if (!solution.ok()) {
        ABSL_LOG(INFO) << "MODEL " << variant.name
                       << (all ? " (all)" : " (half)")
                       << (from_zero ? " from no distortion" : "") << ": "
                       << solution.status();
        continue;
      }
      std::string valid = "n/a";
      if (variant.options.model != LensModel::kKannalaBrandt) {
        aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> built =
            IntrinsicsCalibration::BuildCalibration(
                solution->camera_matrix, solution->dist_coeffs,
                aos::realtime_clock::epoch(), "orin1", "/orin1/camera0",
                "24-00", 971, solution->reprojection_error);
        const absl::Status status =
            ValidateCalibration(built.message(), calibrator.ImageSize().width,
                                calibrator.ImageSize().height);
        valid = status.ok() ? "valid" : "INVALID";
      }
      std::stringstream line;
      line << "MODEL " << variant.name << (all ? " (all)" : " (half)")
           << (from_zero ? " from no distortion" : "") << ": fit "
           << solution->reprojection_error;
      if (!all) {
        const absl::StatusOr<double> error = HeldOutError(
            calibrator.GetCharucoExtractor().board(), test, *solution);
        ASSERT_TRUE(error.ok()) << error.status();
        line << ", held out " << *error;
        if (!from_zero) {
          held_out[variant.name] = *error;
        }
      }
      line << ", " << valid << ", K "
           << solution->camera_matrix.at<double>(0, 0) << " "
           << solution->camera_matrix.at<double>(1, 1) << " "
           << solution->camera_matrix.at<double>(0, 2) << " "
           << solution->camera_matrix.at<double>(1, 2) << ", d "
           << solution->dist_coeffs.reshape(1, 1).colRange(
                  0, std::min<int>(8, solution->dist_coeffs.total()));
      ABSL_LOG(INFO) << line.str();
    }
  }
  EXPECT_GT(held_out["5-term"], 1.0);
  EXPECT_LT(held_out["rational"], 0.5);
  EXPECT_LT(held_out["kannala-brandt"], 0.5);
  EXPECT_LT(held_out["kannala-brandt"], held_out["rational"] * 1.05);
}

}  // namespace frc::vision

#include "frc/vision/calibration/intrinsics_calibration_lib.h"

#include <filesystem>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/die_if_null.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "opencv2/core/eigen.hpp"

#include "frc/vision/calibration/calibration_validator.h"

// NOTE: This will flip any annotations / text that has already been drawn on
// the image (e.g., in charuco_lib.cc)
ABSL_FLAG(bool, flip_image, true,
          "If true, flip the display image so visualization looks correct "
          "during calibration");
ABSL_FLAG(std::string, image_save_path, "",
          "If specified, save out images that are captured for calibration");
ABSL_FLAG(std::string, image_load_path, "",
          "If specified, folder to load images from (image files named "
          "img_######.png");
ABSL_FLAG(
    size_t, min_images_to_calibrate, 50,
    "Minimum number of image captures needed to run intrinsics calibration");
ABSL_FLAG(
    bool, review_images, false,
    "Whether to review the calibration result (only when reading from files)");
ABSL_FLAG(std::string, distortion_model, "kannala_brandt",
          "The lens model to calibrate: kannala_brandt (as OpenCV's fisheye "
          "module has it), or opencv (its pinhole model, with "
          "--use_rational_model's 8 terms or 5).");
ABSL_FLAG(bool, use_rational_model, true,
          "With --distortion_model=opencv, whether to use the 8 parameter "
          "rational model");

ABSL_DECLARE_FLAG(bool, visualize);

ABSL_FLAG(bool, grayscale, false,
          "If true, calibrate in grayscale, otherwise, calibrate in color");
ABSL_FLAG(bool, write_invalid_calibration, false,
          "If true, write the calibration file even if it fails "
          "frc::vision::ValidateCalibration at the calibrated image size.");

namespace frc::vision {

// Found that under 50 ms would fail image too often on intrinsics with
// visualize on
constexpr aos::monotonic_clock::duration kMaxImageAge =
    aos::monotonic_clock::duration(std::chrono::milliseconds(50));

IntrinsicsCalibration::IntrinsicsCalibration(
    aos::EventLoop *event_loop, std::string_view hostname,
    std::string_view camera_channel, std::string_view camera_id,
    std::string_view base_intrinsics_file, bool display_undistorted,
    std::string_view calibration_folder, aos::ExitHandle *exit_handle)
    : hostname_(hostname),
      node_name_(ABSL_DIE_IF_NULL(event_loop->node())->name()->string_view()),
      camera_channel_(camera_channel),
      camera_id_(camera_id),
      reprojection_error_(std::numeric_limits<double>::max()),
      base_intrinsics_(
          aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
              base_intrinsics_file)),
      charuco_extractor_(
          event_loop, &base_intrinsics_.message(), TargetType::kCharuco,
          camera_channel_,
          [this](cv::Mat rgb_image, const aos::monotonic_clock::time_point eof,
                 std::vector<cv::Vec4i> charuco_ids,
                 std::vector<std::vector<cv::Point2f>> charuco_corners,
                 bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                 std::vector<Eigen::Vector3d> tvecs_eigen) {
            HandleCharuco(rgb_image, eof, charuco_ids, charuco_corners, valid,
                          rvecs_eigen, tvecs_eigen);
          }),
      image_callback_(
          event_loop, camera_channel_,
          [this](cv::Mat rgb_image,
                 const aos::monotonic_clock::time_point eof) {
            if (exit_collection_) {
              return;
            }
            charuco_extractor_.HandleImage(rgb_image, eof);
          },
          kMaxImageAge),
      display_undistorted_(display_undistorted),
      calibration_folder_(calibration_folder),
      exit_handle_(exit_handle),
      exit_collection_(false) {
  if (absl::GetFlag(FLAGS_grayscale)) {
    image_callback_.set_format(ImageCallback::Format::GRAYSCALE);
  }

  ABSL_CHECK((absl::GetFlag(FLAGS_image_save_path) == "") ||
             !absl::GetFlag(FLAGS_draw_axes))
      << "Only save images if we're not drawing on them";

  if (absl::GetFlag(FLAGS_image_save_path) != "") {
    ABSL_CHECK(
        std::filesystem::is_directory(absl::GetFlag(FLAGS_image_save_path)))
        << absl::GetFlag(FLAGS_image_save_path) << " is not a directory";
  }

  ABSL_LOG(INFO) << "Hostname is: " << hostname_ << " and camera channel is "
                 << camera_channel_
                 << " make sure you are using the right channel.";

  std::regex re{"^[0-9][0-9]-[0-9][0-9]"};
  ABSL_CHECK(std::regex_match(camera_id_, re))
      << ": Invalid camera_id '" << camera_id_ << "', should be of form YY-NN";
}

void IntrinsicsCalibration::HandleCharuco(
    const cv::Mat rgb_image, const aos::monotonic_clock::time_point /*eof*/,
    std::vector<cv::Vec4i> charuco_ids,
    std::vector<std::vector<cv::Point2f>> charuco_corners, bool valid,
    std::vector<Eigen::Vector3d> rvecs_eigen,
    std::vector<Eigen::Vector3d> tvecs_eigen) {
  constexpr float kVisualizationScaleFactor = 2.0;
  cv::Mat mutable_rgb_image = rgb_image.clone();

  // Store the image size, since it gets used in undistort and calibration
  image_size_ = rgb_image.size();
  if (absl::GetFlag(FLAGS_visualize)) {
    // Before resizing/flipping, display undistorted image, if asked to
    if (display_undistorted_) {
      cv::Mat undistorted_rgb_image(image_size_, CV_8UC3);
      cv::undistort(mutable_rgb_image, undistorted_rgb_image,
                    charuco_extractor_.camera_matrix(),
                    charuco_extractor_.dist_coeffs());

      cv::imshow("Display undist", undistorted_rgb_image);
    }

    // Reduce resolution displayed on remote viewer to prevent lag
    cv::resize(mutable_rgb_image, mutable_rgb_image,
               cv::Size(mutable_rgb_image.cols / kVisualizationScaleFactor,
                        mutable_rgb_image.rows / kVisualizationScaleFactor));
    cv::Mat display_image = mutable_rgb_image;
    if (absl::GetFlag(FLAGS_flip_image)) {
      cv::flip(mutable_rgb_image, display_image, 1);
    }
    cv::putText(
        display_image,
        "Captured " + std::to_string(all_charuco_ids_.size()) + " images",
        cv::Point(display_image.size().width / 2, 10), cv::FONT_HERSHEY_PLAIN,
        1.0, cv::Scalar(0, 255, 0));
    if (!valid) {
      cv::putText(display_image, "Invalid Estimated Pose",
                  cv::Point(display_image.size().width / 2, 30),
                  cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
    }
    cv::imshow("Display", display_image);
  }

  int keystroke = cv::waitKey(1);
  if ((keystroke & 0xFF) == static_cast<int>('q') ||
      all_charuco_ids_.size() >= absl::GetFlag(FLAGS_min_images_to_calibrate)) {
    ABSL_LOG(INFO) << "Going to exit";
    exit_collection_ = true;
    exit_handle_->Exit();
  }
  // If we haven't got a valid pose estimate, don't use these points
  if (!valid) {
    // Require valid poses on loading from disk, so we can visually
    // review once done
    if (!absl::GetFlag(FLAGS_image_load_path).empty()) {
      ABSL_LOG(ERROR) << "Shouldn't have invalid images in loading from logs";
    }
    ABSL_VLOG(1) << "Skip because pose is not valid";
    return;
  }

  ABSL_CHECK(tvecs_eigen.size() == 1)
      << "Charuco board should only return one translational pose.  Returned "
      << tvecs_eigen.size();
  ABSL_CHECK(rvecs_eigen.size() == 1)
      << "Charuco board should only return one rotational pose. Returned "
      << rvecs_eigen.size();

  const Eigen::Affine3d H_camera_board =
      Eigen::Translation3d(tvecs_eigen[0]) *
      Eigen::AngleAxisd(rvecs_eigen[0].norm(),
                        rvecs_eigen[0] / rvecs_eigen[0].norm());
  const CaptureGate::Decision decision = capture_gate_.Judge(H_camera_board);
  bool store_image = decision.verdict == CaptureGate::Verdict::kCapture;
  ABSL_LOG(INFO) << "Captured: " << all_charuco_ids_.size()
                 << " samples; moved " << static_cast<int>(decision.moved * 100)
                 << "% of what's needed"
                 << (decision.verdict == CaptureGate::Verdict::kMoveToNewPose
                         ? ""
                         : absl::StrCat("; last motion was ",
                                        static_cast<int>(decision.motion * 100),
                                        "% of the limit"))
                 << (store_image ? ". Capturing image" : "");

  if (absl::GetFlag(FLAGS_image_load_path) != "") {
    ABSL_LOG(INFO) << "Loading from disk, so capturing all images";
    store_image = true;
  }

  if ((keystroke & 0xFF) == static_cast<int>('c')) {
    // Protect from hitting capture key but then the image captured is bad
    if (charuco_ids.size() > 15) {
      ABSL_LOG(INFO) << "Manual capture triggered";
      store_image = true;
    } else {
      ABSL_LOG(INFO) << "Manual attempt rejected due to too few features "
                     << charuco_ids.size();
    }
  }

  if (store_image) {
    if (valid) {
      capture_gate_.Captured(H_camera_board);

      // Unpack the Charuco ids from Vec4i
      std::vector<int> charuco_ids_int;
      for (cv::Vec4i charuco_id : charuco_ids) {
        charuco_ids_int.emplace_back(charuco_id[0]);
      }
      all_charuco_ids_.emplace_back(std::move(charuco_ids_int));
      all_charuco_corners_.emplace_back(std::move(charuco_corners[0]));

      if (absl::GetFlag(FLAGS_visualize)) {
        if (point_viz_image_.empty()) {
          point_viz_image_ = cv::Mat::zeros(mutable_rgb_image.size(), CV_8UC1);
        }
        // Draw a small circle for each corner
        for (uint i = 0; i < all_charuco_corners_.back().size(); i++) {
          cv::Point2f point = all_charuco_corners_.back()[i];
          point = point / kVisualizationScaleFactor;
          cv::circle(point_viz_image_, point, 3, cv::Scalar(255, 255, 255));
        }

        cv::Mat display_image;
        if (absl::GetFlag(FLAGS_flip_image)) {
          cv::flip(point_viz_image_, display_image, 1);
        } else {
          display_image = point_viz_image_;
        }
        cv::imshow("Captured Point Visualization", display_image);
        cv::waitKey(1);
      }
      ABSL_LOG(INFO) << "Storing image #" << all_charuco_corners_.size();

      if (absl::GetFlag(FLAGS_image_save_path) != "") {
        std::string image_name =
            absl::StrFormat("/img_%06d.png", image_save_count_++);
        std::string path = absl::GetFlag(FLAGS_image_save_path) + image_name;
        ABSL_VLOG(2) << "Saving intrinsic calibration image to " << path;
        cv::imwrite(path, rgb_image);
      }
    }
  }
}

void IntrinsicsCalibration::LoadImages(std::vector<std::string> file_list) {
  file_list_ = file_list;
  for (std::string filename : file_list_) {
    ABSL_LOG(INFO) << "Loading image " << filename;
    cv::Mat image = cv::imread(filename);
    charuco_extractor_.HandleImage(image, aos::monotonic_clock::now());
  }
}

void IntrinsicsCalibration::LoadImagesFromPath(
    const std::filesystem::path &path) {
  std::vector<std::string> file_list;
  std::filesystem::directory_iterator dir(path);
  for (const auto &entry : dir) {
    if (entry.is_regular_file()) {  // Skip directories
      if (std::regex_match(entry.path().filename().string(),
                           std::regex("img_[0-9]{6}.png"))) {
        file_list.push_back(entry.path().string());
      }
    }
  }

  std::sort(file_list.begin(), file_list.end());
  ABSL_LOG(INFO) << "Loading " << file_list.size() << " images from "
                 << path.string();
  LoadImages(file_list);
}

aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
IntrinsicsCalibration::BuildCalibration(
    cv::Mat camera_matrix, cv::Mat dist_coeffs,
    aos::realtime_clock::time_point realtime_now, std::string_view node_name,
    std::string_view camera_channel, std::string_view camera_id,
    uint16_t team_number, double reprojection_error,
    std::optional<BoardSpec> board,
    calibration::DistortionModel distortion_model) {
  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<calibration::BoardSpec> board_offset;
  if (board.has_value()) {
    board_offset = PackBoardSpec(&fbb, *board);
  }
  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(node_name);
  flatbuffers::Offset<flatbuffers::String> camera_id_offset =
      fbb.CreateString(camera_id);
  flatbuffers::Offset<flatbuffers::Vector<float>> intrinsics_offset =
      fbb.CreateVector<float>(9u, [&camera_matrix](size_t i) {
        return static_cast<float>(
            reinterpret_cast<double *>(camera_matrix.data)[i]);
      });

  std::optional<uint16_t> camera_number =
      frc::vision::CameraNumberFromChannel(std::string(camera_channel));
  ABSL_CHECK(camera_number.has_value())
      << ": Failed to parse camera number from " << camera_channel;

  flatbuffers::Offset<flatbuffers::Vector<float>>
      distortion_coefficients_offset = fbb.CreateVector<float>(
          dist_coeffs.total(), [&dist_coeffs](size_t i) {
            return static_cast<float>(
                reinterpret_cast<double *>(dist_coeffs.data)[i]);
          });

  calibration::CameraCalibration::Builder camera_calibration_builder(fbb);

  camera_calibration_builder.add_node_name(name_offset);
  camera_calibration_builder.add_team_number(team_number);
  camera_calibration_builder.add_camera_number(camera_number.value());
  camera_calibration_builder.add_camera_id(camera_id_offset);
  camera_calibration_builder.add_reprojection_error(
      static_cast<float>(reprojection_error));
  camera_calibration_builder.add_calibration_timestamp(
      realtime_now.time_since_epoch().count());
  camera_calibration_builder.add_intrinsics(intrinsics_offset);
  camera_calibration_builder.add_dist_coeffs(distortion_coefficients_offset);
  if (!board_offset.IsNull()) {
    camera_calibration_builder.add_board(board_offset);
  }
  camera_calibration_builder.add_distortion_model(distortion_model);
  fbb.Finish(camera_calibration_builder.Finish());

  return fbb.Release();
}

void IntrinsicsCalibration::DrawCornersOnImage(cv::Mat image, uint index,
                                               std::vector<cv::Mat> tvecs,
                                               std::vector<cv::Mat> rvecs,
                                               cv::Mat camera_matrix,
                                               cv::Mat dist_coeffs) {
  std::vector<cv::Point3f> board_corners = charuco_extractor_.board()->
#if CV_VERSION_MINOR >= 9
                                           getChessboardCorners();
#else
                                           chessboardCorners;
#endif
  double square_length = charuco_extractor_.board()->getSquareLength();

  {
    //  Draw my own frame axes, since opencv version has issues
    Eigen::Vector3d tvec, rvec;
    cv::cv2eigen(tvecs[index], tvec);
    cv::cv2eigen(rvecs[index], rvec);
    Eigen::Affine3d T_cam_board =
        Eigen::Translation3d(tvec) *
        Eigen::AngleAxisd(rvec.norm(), rvec / rvec.norm());
    // Project the point T_cam_board to the image using camera_matrix
    // and dist_coeffs
    cv::Mat point_in_cam;
    cv::projectPoints(tvecs[index], cv::Mat::zeros(3, 1, CV_32F),
                      cv::Mat::zeros(3, 1, CV_32F), camera_matrix, dist_coeffs,
                      point_in_cam);
    cv::Point2f point;
    point.x = static_cast<float>(point_in_cam.at<double>(0, 0));
    point.y = static_cast<float>(point_in_cam.at<double>(0, 1));
    // Draw a circle using T_cam_board
    cv::circle(image, point, 10, cv::Scalar(0, 255, 0), 2);
    for (uint i = 0; i < 3; i++) {
      Eigen::Vector3d eye_col = Eigen::Matrix3d::Identity().col(i);
      Eigen::Vector3d ray = T_cam_board * (square_length * eye_col);

      cv::Scalar color =
          cv::Scalar((int)(eye_col(2, 0) * 255), (int)(eye_col(1, 0) * 255),
                     (int)(eye_col(0, 0) * 255));
      cv::Mat ray_cv;
      cv::eigen2cv(ray, ray_cv);
      cv::projectPoints(ray_cv, cv::Mat::zeros(3, 1, CV_32F),
                        cv::Mat::zeros(3, 1, CV_32F), camera_matrix,
                        dist_coeffs, point_in_cam);
      cv::Point2f end_point;
      end_point.x = (float)point_in_cam.at<double>(0, 0);
      end_point.y = (float)point_in_cam.at<double>(0, 1);
      cv::line(image, point, end_point, color, 2);
    }

    // Draw circles around locations of all the corners of the board
    for (cv::Point3f corner : board_corners) {
      Eigen::Vector3d point;
      point << corner.x, corner.y, corner.z;
      // Map charuco corner (3D) point to have camera as origin
      Eigen::Vector3d transformed_point = T_cam_board * point;
      cv::Mat t_point_cv;
      cv::eigen2cv(transformed_point, t_point_cv);
      // Project point into camera view using our camera intrinsics
      cv::projectPoints(t_point_cv, cv::Mat::zeros(3, 1, CV_32F),
                        cv::Mat::zeros(3, 1, CV_32F), camera_matrix,
                        dist_coeffs, point_in_cam);
      cv::Point2f point_2f;
      point_2f.x = (float)point_in_cam.at<double>(0, 0);
      point_2f.y = (float)point_in_cam.at<double>(0, 1);
      cv::circle(image, point_2f, 10, cv::Scalar(0, 0, 255), 2);
    }

    // Then, draw over them with green circles showing our matches
    for (uint j = 0; j < all_charuco_ids_[index].size(); j++) {
      cv::Point3f corner = board_corners[all_charuco_ids_[index][j]];
      Eigen::Vector3d point;
      point << corner.x, corner.y, corner.z;
      Eigen::Vector3d transformed_point = T_cam_board * point;
      cv::Mat t_point_cv;
      cv::eigen2cv(transformed_point, t_point_cv);
      cv::projectPoints(t_point_cv, cv::Mat::zeros(3, 1, CV_32F),
                        cv::Mat::zeros(3, 1, CV_32F), camera_matrix,
                        dist_coeffs, point_in_cam);
      cv::Point2f point_2f;
      point_2f.x = (float)point_in_cam.at<double>(0, 0);
      point_2f.y = (float)point_in_cam.at<double>(0, 1);
      cv::circle(image, point_2f, 10, cv::Scalar(0, 255, 0), 2);
    }
  }
}

absl::Status IntrinsicsCalibration::WriteValidatedCalibration(
    const calibration::CameraCalibration &calibration, cv::Size image_size,
    const std::string &filename) {
  CalibrationReport report;
  const absl::Status validation = ValidateCalibration(
      calibration, image_size.width, image_size.height, &report);
  if (validation.ok()) {
    ABSL_LOG(INFO) << "The calibration is valid: " << report;
  } else {
    ABSL_LOG(ERROR) << "The calibration fails validation at the "
                    << image_size.width << "x" << image_size.height
                    << " it was calibrated at, so the AprilTag detector "
                       "can't reliably undistort all of the image with it:\n"
                    << validation.message() << "\n"
                    << report;
    if (!absl::GetFlag(FLAGS_write_invalid_calibration)) {
      const absl::Status status = absl::FailedPreconditionError(absl::StrCat(
          "Not writing ", filename,
          ", which fails validation.  Recalibrate with more captures of the "
          "board in the image's corners, rerun with "
          "--write_invalid_calibration to write it anyway, or copy it from "
          "the log above."));
      ABSL_LOG(ERROR) << status.message();
      return status;
    }
  }
  aos::util::WriteStringToFileOrDie(
      filename, aos::FlatbufferToJson(&calibration, {.multi_line = true}));
  return absl::OkStatus();
}

absl::Status IntrinsicsCalibration::MaybeCalibrate() {
  // TODO: This number should depend on coarse vs. fine pattern
  // Maybe just on total # of ids found, not just images
  if (all_charuco_ids_.size() >= absl::GetFlag(FLAGS_min_images_to_calibrate)) {
    int total_num_ids = 0;
    for (auto charuco_ids : all_charuco_ids_) {
      total_num_ids += charuco_ids.size();
    }
    ABSL_LOG(INFO) << "Beginning calibration on " << all_charuco_ids_.size()
                   << " images, with " << total_num_ids << " ids total";

    if (camera_mat_.empty()) {
      camera_mat_ = charuco_extractor_.camera_matrix();
    }
    if (dist_coeffs_.empty()) {
      dist_coeffs_ = charuco_extractor_.dist_coeffs();
    }
    std::vector<CharucoCapture> captures;
    for (size_t i = 0; i < all_charuco_ids_.size(); ++i) {
      captures.push_back({all_charuco_ids_[i], all_charuco_corners_[i]});
    }
    const std::string distortion_model = absl::GetFlag(FLAGS_distortion_model);
    if (distortion_model != "kannala_brandt" && distortion_model != "opencv") {
      return absl::InvalidArgumentError(
          absl::StrCat("--distortion_model is kannala_brandt or opencv, not ",
                       distortion_model));
    }
    const LensModel model =
        distortion_model == "kannala_brandt"      ? LensModel::kKannalaBrandt
        : absl::GetFlag(FLAGS_use_rational_model) ? LensModel::kOpenCvRational
                                                  : LensModel::kOpenCv5;
    // From the starting calibration's model, with the camera matrix and
    // coefficients it holds (which a test may have set).
    CameraLens guess = charuco_extractor_.lens();
    camera_mat_.convertTo(guess.camera_matrix, CV_64F);
    dist_coeffs_.convertTo(guess.dist_coeffs, CV_64F);
    absl::StatusOr<IntrinsicsSolution> solution =
        SolveIntrinsics(charuco_extractor_.board(), captures, image_size_,
                        guess, SolveOptions{.model = model});
    camera_model_ = model;
    if (!solution.ok()) {
      ABSL_LOG(ERROR) << solution.status();
      return solution.status();
    }
    camera_mat_ = solution->camera_matrix;
    dist_coeffs_ = solution->dist_coeffs;
    reprojection_error_ = solution->reprojection_error;
    rvecs_.clear();
    tvecs_.clear();
    for (size_t i = 0; i < solution->rvecs.size(); ++i) {
      rvecs_.push_back(cv::Mat(solution->rvecs[i]).clone());
      tvecs_.push_back(cv::Mat(solution->tvecs[i]).clone());
    }
    if (reprojection_error_ < 1.0) {
      ABSL_LOG(INFO) << "Reprojection Error is " << reprojection_error_;
    } else {
      ABSL_LOG(WARNING) << "NOTE: Reprojection Error is > 1.0, at "
                        << reprojection_error_;
    }

    const aos::realtime_clock::time_point realtime_now =
        aos::realtime_clock::now();
    std::optional<uint16_t> team_number =
        aos::network::team_number_internal::ParsePiOrOrinTeamNumber(hostname_);
    ABSL_CHECK(team_number.has_value()) << ": Invalid hostname " << hostname_
                                        << ", failed to parse team number";
    aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
        camera_calibration = BuildCalibration(
            camera_mat_, dist_coeffs_, realtime_now, node_name_,
            camera_channel_, camera_id_, team_number.value(),
            reprojection_error_, BoardSpecFromFlags(),
            camera_model_ == LensModel::kKannalaBrandt
                ? calibration::DistortionModel::KANNALA_BRANDT
                : calibration::DistortionModel::OPENCV);
    std::stringstream time_ss;
    time_ss << realtime_now;

    std::optional<uint16_t> camera_number =
        frc::vision::CameraNumberFromChannel(camera_channel_);
    ABSL_CHECK(camera_number.has_value());
    std::string calibration_filename = CalibrationFilename(
        calibration_folder_, node_name_, team_number.value(),
        camera_number.value(), camera_id_, time_ss.str());

    ABSL_LOG(INFO) << calibration_filename << " -> "
                   << aos::FlatbufferToJson(camera_calibration,
                                            {.multi_line = true});
    const absl::Status status = WriteValidatedCalibration(
        camera_calibration.message(), image_size_, calibration_filename);

    // If desired, review the images (requires we're loading from disk);
    if (absl::GetFlag(FLAGS_review_images) &&
        !absl::GetFlag(FLAGS_image_load_path).empty()) {
      uint index = 0;
      for (std::string filename : file_list_) {
        ABSL_LOG(INFO) << "Loading file " << filename;
        cv::Mat image = cv::imread(filename);
        DrawCornersOnImage(image, index, tvecs_, rvecs_, camera_mat_,
                           dist_coeffs_);
        cv::imshow("Image review", image);
        cv::waitKey(0);
        index++;
      }
    }
    return status;
  }
  ABSL_LOG(INFO) << "Skipping calibration due to not enough images.";
  return absl::OkStatus();
}
}  // namespace frc::vision

#include "frc/vision/calibration/calibration_server_lib.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "opencv2/imgcodecs.hpp"

#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/vision/calibration/calibration_validator.h"
#include "frc/vision/calibration/charuco_detector.h"
#include "frc/vision/calibration/intrinsics_calibration_lib.h"
#include "frc/vision/util/charuco_lib.h"
#include "frc/vision/util/vision_util_lib.h"
#include "frc/vision/util/zip_writer.h"

namespace frc::vision {
namespace {

using calibration::CommandAction;
using calibration::FrameVerdict;
using calibration::SessionState;

// How long without an image before saying so.
constexpr std::chrono::seconds kImageTimeout(1);
// Pixels per meter of the board's printable picture: about 200 dpi.
constexpr double kBoardImagePixelsPerMeter = 8000.0;

bool WriteFile(const std::filesystem::path &path, std::string_view contents) {
  std::error_code error;
  std::filesystem::create_directories(path.parent_path(), error);
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << contents;
  file.close();
  if (!file) {
    ABSL_LOG(ERROR) << "Failed to write " << path;
    return false;
  }
  return true;
}

int16_t Hundredths(float pixels) {
  return static_cast<int16_t>(
      std::clamp(std::lround(pixels * 100.0f), -32767l, 32767l));
}

calibration::DistortionModel DistortionModelOf(LensModel model) {
  return model == LensModel::kKannalaBrandt
             ? calibration::DistortionModel::KANNALA_BRANDT
             : calibration::DistortionModel::OPENCV;
}

Eigen::Affine3d Pose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
  const Eigen::Vector3d r(rvec[0], rvec[1], rvec[2]);
  const double angle = r.norm();
  return Eigen::Translation3d(tvec[0], tvec[1], tvec[2]) *
         (angle > 0 ? Eigen::AngleAxisd(angle, r / angle)
                    : Eigen::AngleAxisd::Identity());
}

}  // namespace

CalibrationServer::CalibrationServer(aos::EventLoop *event_loop,
                                     Options options)
    : event_loop_(event_loop),
      options_(std::move(options)),
      status_sender_(
          event_loop_->MakeSender<calibration::Status>("/calibration")),
      session_sender_(
          event_loop_->MakeSender<calibration::Session>("/calibration")),
      constants_fetcher_(
          event_loop_->MakeFetcher<CameraConstants>("/constants")) {
  for (int camera : options_.cameras) {
    image_fetchers_.emplace(camera,
                            event_loop_->MakeFetcher<CameraImage>(
                                absl::StrCat("/camera", camera, "/gray")));
    annotation_senders_.emplace(
        camera, event_loop_->MakeSender<foxglove::ImageAnnotations>(
                    absl::StrCat("/camera", camera, "/calibration")));
  }
  event_loop_->MakeWatcher(
      "/calibration",
      [this](const calibration::Command &command) { HandleCommand(command); });
  timer_ = event_loop_->AddTimer([this]() { Tick(); });
  event_loop_->OnRun([this]() {
    timer_->Schedule(event_loop_->monotonic_now(), options_.period);
  });
  LoadStore();
}

CalibrationServer::~CalibrationServer() {
  if (solving_.has_value()) {
    solving_->wait();
  }
}

void CalibrationServer::HandleCommand(const calibration::Command &command) {
  const absl::Status status = Execute(command);
  error_ = status.ok() ? "" : std::string(status.message());
  if (!status.ok()) {
    ABSL_LOG(WARNING) << calibration::EnumNameCommandAction(command.action())
                      << ": " << status.message();
  }
  session_dirty_ = true;
  SendStatus();
}

absl::Status CalibrationServer::Execute(const calibration::Command &command) {
  if (command.action() == CommandAction::START) {
    return Start(command);
  }
  if (state_ == SessionState::IDLE) {
    return absl::FailedPreconditionError("No session is running");
  }
  if (command.has_session_id() &&
      command.session_id()->string_view() != session_id_) {
    return absl::FailedPreconditionError(
        absl::StrCat("That's for session ", command.session_id()->string_view(),
                     ", not this one, ", session_id_));
  }
  const bool solving = state_ == SessionState::SOLVING;
  if (state_ == SessionState::VALIDATING) {
    switch (command.action()) {
      case CommandAction::SET_BOARD:
      case CommandAction::CAPTURE:
      case CommandAction::UNDO:
      case CommandAction::DROP:
      case CommandAction::SOLVE:
        return absl::FailedPreconditionError(
            "Validating: stop validating to change the captures");
      default:
        break;
    }
  }
  switch (command.action()) {
    case CommandAction::START:
      break;
    case CommandAction::SET_BOARD:
      if (!command.has_board()) {
        return absl::InvalidArgumentError("No board given");
      }
      if (solving) {
        return absl::FailedPreconditionError("Solving");
      }
      return UseBoard(BoardSpecFromFlatbuffer(*command.board()));
    case CommandAction::SET_TARGET:
      if (command.target_captures() < 10 || command.target_captures() > 500) {
        return absl::InvalidArgumentError(absl::StrCat(
            "Aim for 10 to 500 captures, not ", command.target_captures()));
      }
      target_captures_ = command.target_captures();
      WriteStore();
      return absl::OkStatus();
    case CommandAction::SAVE_BOARD: {
      if (!command.has_board() || !command.has_board_name() ||
          command.board_name()->size() == 0) {
        return absl::InvalidArgumentError("A board to save needs a name");
      }
      const BoardSpec spec = BoardSpecFromFlatbuffer(*command.board());
      if (absl::Status valid = ValidateBoardSpec(spec); !valid.ok()) {
        return valid;
      }
      const std::string name = command.board_name()->str();
      for (const NamedBoard &preset : BoardPresets()) {
        if (preset.name == name) {
          return absl::InvalidArgumentError(
              absl::StrCat("\"", name, "\" is a preset"));
        }
      }
      std::erase_if(saved_boards_, [&](const NamedBoard &board) {
        return board.name == name;
      });
      saved_boards_.push_back({name, spec});
      WriteStore();
      return absl::OkStatus();
    }
    case CommandAction::DELETE_BOARD: {
      const std::string name =
          command.has_board_name() ? command.board_name()->str() : "";
      if (std::erase_if(saved_boards_, [&](const NamedBoard &board) {
            return board.name == name;
          }) == 0) {
        return absl::NotFoundError(
            absl::StrCat("No saved board is named \"", name, "\""));
      }
      WriteStore();
      return absl::OkStatus();
    }
    case CommandAction::CAPTURE:
      if (solving) {
        return absl::FailedPreconditionError("Solving");
      }
      capture_next_ = true;
      return absl::OkStatus();
    case CommandAction::UNDO:
      if (solving) {
        return absl::FailedPreconditionError("Solving");
      }
      if (captures_.empty()) {
        return absl::FailedPreconditionError("No captures to undo");
      }
      captures_.pop_back();
      CapturesChanged();
      return absl::OkStatus();
    case CommandAction::DROP: {
      if (solving) {
        return absl::FailedPreconditionError("Solving");
      }
      if (!command.has_drop()) {
        return absl::InvalidArgumentError("No captures to drop");
      }
      std::vector<int> drop(command.drop()->begin(), command.drop()->end());
      std::sort(drop.begin(), drop.end(), std::greater<int>());
      drop.erase(std::unique(drop.begin(), drop.end()), drop.end());
      for (int index : drop) {
        if (index < 0 || index >= static_cast<int>(captures_.size())) {
          return absl::InvalidArgumentError(absl::StrCat("No capture ", index));
        }
      }
      for (int index : drop) {
        captures_.erase(captures_.begin() + index);
      }
      CapturesChanged();
      return absl::OkStatus();
    }
    case CommandAction::SOLVE: {
      if (solving) {
        return absl::FailedPreconditionError("Already solving");
      }
      if (captures_.size() < 4) {
        return absl::FailedPreconditionError(
            "Need at least 4 captures to solve");
      }
      std::vector<CharucoCapture> captures;
      for (const StoredCapture &capture : captures_) {
        captures.push_back(capture.capture);
      }
      solving_captures_ = captures.size();
      solve_error_.clear();
      state_ = SessionState::SOLVING;
      // A second or so: off the event loop's thread, so it keeps reporting.
      solving_ =
          std::async(std::launch::async,
                     [board = cv_board_, captures = std::move(captures),
                      image_size = image_size_, guess = guess_]() {
                       return SolveIntrinsics(board, captures, image_size,
                                              guess, SolveOptions{});
                     });
      return absl::OkStatus();
    }
    case CommandAction::SAVE:
      return Save(command.force());
    case CommandAction::FINISH:
    case CommandAction::CANCEL:
      // robot_server stops this process; until then, stop collecting.
      state_ = SessionState::IDLE;
      return absl::OkStatus();
    case CommandAction::VALIDATE:
      if (solving) {
        return absl::FailedPreconditionError("Solving");
      }
      return Validate(command);
    case CommandAction::EXPORT_IMAGES:
      return ExportImages();
    case CommandAction::STOP_VALIDATING:
      if (state_ == SessionState::VALIDATING) {
        state_ = solved_.has_value() &&
                         solved_->captures == static_cast<int>(captures_.size())
                     ? SessionState::SOLVED
                     : SessionState::COLLECTING;
      }
      return absl::OkStatus();
    case CommandAction::SET_CAMERA_ID: {
      const std::string camera_id =
          command.has_camera_id() ? command.camera_id()->str() : "";
      if (!std::regex_match(camera_id, std::regex("^[0-9][0-9]-[0-9][0-9]$"))) {
        return absl::InvalidArgumentError(
            absl::StrCat("A camera id is YY-NN, a year and a number, not \"",
                         camera_id, "\""));
      }
      camera_id_ = camera_id;
      return absl::OkStatus();
    }
  }
  return absl::InvalidArgumentError("Unknown command");
}

absl::Status CalibrationServer::Start(const calibration::Command &command) {
  if (state_ == SessionState::SOLVING) {
    return absl::FailedPreconditionError("Solving");
  }
  if (std::find(options_.cameras.begin(), options_.cameras.end(),
                command.camera()) == options_.cameras.end()) {
    return absl::InvalidArgumentError(
        absl::StrCat("There's no camera ", command.camera()));
  }
  session_id_ =
      command.has_session_id() && command.session_id()->size() > 0
          ? command.session_id()->str()
          : absl::StrCat(
                event_loop_->realtime_now().time_since_epoch().count());
  camera_ = command.camera();
  captures_.clear();
  gate_.set_last_capture(std::nullopt);
  capture_next_ = false;
  checks_.clear();
  validations_.clear();
  next_image_ = 0;
  images_download_path_.clear();
  images_exported_ = 0;
  solved_.reset();
  solve_error_.clear();
  image_size_ = cv::Size();
  last_image_timestamp_ns_ = -1;
  last_image_time_ = event_loop_->monotonic_now();
  verdict_ = FrameVerdict::NO_IMAGE;

  current_.reset();
  constants_fetcher_.Fetch();
  if (constants_fetcher_.get() != nullptr &&
      constants_fetcher_->has_calibration()) {
    for (const calibration::CameraCalibration *calibration :
         *constants_fetcher_->calibration()) {
      if (calibration->has_node_name() &&
          calibration->node_name()->string_view() == options_.node_name &&
          calibration->camera_number() == camera_ &&
          calibration->has_intrinsics() &&
          calibration->intrinsics()->size() == 9) {
        flatbuffers::FlatBufferBuilder fbb;
        fbb.Finish(aos::RecursiveCopyFlatBuffer(calibration, &fbb));
        current_.emplace(fbb.Release());
      }
    }
  }
  camera_id_ = command.has_camera_id() && command.camera_id()->size() > 0
                   ? command.camera_id()->str()
               : current_.has_value() && current_->message().has_camera_id()
                   ? current_->message().camera_id()->str()
                   : "";
  if (command.target_captures() > 0) {
    target_captures_ = command.target_captures();
  }

  BoardSpec board = BoardPresets()[0].spec;
  if (command.has_board()) {
    board = BoardSpecFromFlatbuffer(*command.board());
  } else if (auto it = last_boards_.find(camera_); it != last_boards_.end()) {
    board = it->second;
  }
  state_ = SessionState::COLLECTING;
  if (absl::Status used = UseBoard(board); !used.ok()) {
    // Fall back on the default board, and say why.
    UseBoard(BoardPresets()[0].spec).IgnoreError();
    return used;
  }
  UpdateGuess();
  WriteStore();
  ABSL_LOG(INFO) << "Calibrating camera " << camera_ << " ("
                 << (camera_id_.empty() ? "unknown id" : camera_id_)
                 << ") with a " << board_ << ", session " << session_id_;
  return absl::OkStatus();
}

absl::Status CalibrationServer::UseBoard(const BoardSpec &board) {
  if (absl::Status valid = ValidateBoardSpec(board); !valid.ok()) {
    return valid;
  }
  if (!captures_.empty() && board != board_) {
    return absl::FailedPreconditionError(
        "The captures are of another board: undo them to change it");
  }
  board_ = board;
  cv_board_ = MakeCharucoBoard(board_);
  last_boards_[camera_] = board_;

  // A picture of it to print, at its size.
  cv::Mat image;
  const cv::Size size(std::lround(board_.squares_x * board_.square_length *
                                  kBoardImagePixelsPerMeter),
                      std::lround(board_.squares_y * board_.square_length *
                                  kBoardImagePixelsPerMeter));
#if CV_VERSION_MINOR >= 9
  cv_board_->generateImage(size, image, 0, 1);
#else
  cv_board_->draw(size, image, 0, 1);
#endif
  std::vector<uchar> png;
  cv::imencode(".png", image, png);
  const std::filesystem::path path =
      std::filesystem::path(options_.www_folder) / "board.png";
  if (WriteFile(path,
                std::string_view(reinterpret_cast<const char *>(png.data()),
                                 png.size()))) {
    std::stringstream version;
    version << std::hash<std::string>()(absl::StrCat(
        board_.squares_x, "x", board_.squares_y, "/", board_.square_length, "/",
        board_.marker_length, "/", board_.dictionary));
    board_image_path_ =
        absl::StrCat(options_.www_path, "/board.png?v=", version.str());
  } else {
    board_image_path_.clear();
  }
  session_dirty_ = true;
  return absl::OkStatus();
}

void CalibrationServer::UpdateGuess() {
  if (current_.has_value() && current_->message().has_dist_coeffs()) {
    guess_ = CameraLens::FromCalibration(&current_->message());
  } else if (current_.has_value()) {
    guess_ = CameraLens{
        .camera_matrix = CameraIntrinsics(&current_->message()).clone(),
        .dist_coeffs = cv::Mat::zeros(5, 1, CV_64F),
        .kannala_brandt = false};
    guess_.camera_matrix.convertTo(guess_.camera_matrix, CV_64F);
  } else if (image_size_.width > 0) {
    // A 60 degree field of view, centred, and no distortion.
    const double f = image_size_.width * 0.87;
    guess_ = CameraLens{.camera_matrix = (cv::Mat_<double>(3, 3) << f, 0,
                                          image_size_.width / 2.0, 0, f,
                                          image_size_.height / 2.0, 0, 0, 1),
                        .dist_coeffs = cv::Mat::zeros(5, 1, CV_64F),
                        .kannala_brandt = false};
  }
}

void CalibrationServer::Tick() {
  ++ticks_;
  CollectSolve();
  if (state_ != SessionState::IDLE) {
    aos::Fetcher<CameraImage> &fetcher = image_fetchers_.at(camera_);
    if (fetcher.Fetch() &&
        fetcher->monotonic_timestamp_ns() != last_image_timestamp_ns_) {
      last_image_timestamp_ns_ = fetcher->monotonic_timestamp_ns();
      last_image_time_ = event_loop_->monotonic_now();
      LookAtImage(*fetcher);
    } else if (event_loop_->monotonic_now() - last_image_time_ >
               kImageTimeout) {
      verdict_ = FrameVerdict::NO_IMAGE;
      corners_ = 0;
    }
  }
  if (ticks_ % 2 == 0) {
    SendStatus();
  }
  MaybeSendSession();
}

void CalibrationServer::LookAtImage(const CameraImage &image) {
  if (image.format() != ImageFormat::MONO8 || !image.has_data()) {
    verdict_ = FrameVerdict::NO_IMAGE;
    return;
  }
  if (image_size_ != cv::Size(image.cols(), image.rows())) {
    image_size_ = cv::Size(image.cols(), image.rows());
    UpdateGuess();
    session_dirty_ = true;
  }
  if (guess_.camera_matrix.empty()) {
    return;
  }
  const cv::Mat gray(image.rows(), image.cols(), CV_8UC1,
                     const_cast<uint8_t *>(image.data()->data()));
  const CharucoDetection detection = DetectCharuco(cv_board_, gray, guess_);
  corners_ = detection.ids.size();

  {
    auto builder = annotation_senders_.at(camera_).MakeBuilder();
    builder.CheckOk(builder.Send(BuildAnnotations(
        builder.fbb(),
        aos::monotonic_clock::time_point(
            std::chrono::nanoseconds(image.monotonic_timestamp_ns())),
        {detection.corners},
        detection.pose_valid ? std::vector<double>{0.0, 1.0, 0.0, 1.0}
                             : std::vector<double>{1.0, 0.8, 0.0, 1.0},
        6.0)));
  }

  if (state_ == SessionState::VALIDATING) {
    Measure(gray, detection);
  }
  if (state_ == SessionState::SOLVING) {
    verdict_ = FrameVerdict::NOT_COLLECTING;
    return;
  }
  if (corners_ < kMinCharucoCorners) {
    verdict_ = FrameVerdict::TOO_FEW_CORNERS;
    return;
  }
  if (!detection.pose_valid) {
    verdict_ = FrameVerdict::NO_POSE;
    return;
  }
  if (state_ == SessionState::VALIDATING) {
    verdict_ = FrameVerdict::NOT_COLLECTING;
    return;
  }
  const Eigen::Affine3d H_camera_board = Pose(detection.rvec, detection.tvec);
  const CaptureGate::Decision decision = gate_.Judge(H_camera_board);
  moved_ = decision.moved;
  motion_ = decision.motion;
  const bool reached = static_cast<int>(captures_.size()) >= target_captures_;
  const bool capture =
      capture_next_ || (state_ == SessionState::COLLECTING && !reached &&
                        decision.verdict == CaptureGate::Verdict::kCapture);
  if (!capture) {
    verdict_ = reached ? FrameVerdict::TARGET_REACHED
               : decision.verdict == CaptureGate::Verdict::kHoldStill
                   ? FrameVerdict::HOLD_STILL
                   : FrameVerdict::MOVE_TO_NEW_POSE;
    return;
  }
  capture_next_ = false;
  gate_.Captured(H_camera_board);
  // The image, for calibrating again offline: calibrate_images.
  std::string image_file;
  std::vector<uchar> png;
  if (cv::imencode(".png", gray, png)) {
    const std::filesystem::path path =
        SessionFolder() / absl::StrFormat("frame_%04d.png", next_image_++);
    if (WriteFile(path,
                  std::string_view(reinterpret_cast<const char *>(png.data()),
                                   png.size()))) {
      image_file = path;
    }
  }
  captures_.push_back({.monotonic_timestamp_ns = image.monotonic_timestamp_ns(),
                       .capture = {detection.ids, detection.corners},
                       .rvec = detection.rvec,
                       .tvec = detection.tvec,
                       .image_file = image_file});
  verdict_ = FrameVerdict::CAPTURED;
  CapturesChanged();
}

void CalibrationServer::CapturesChanged() {
  if (captures_.empty()) {
    gate_.set_last_capture(std::nullopt);
  } else {
    gate_.set_last_capture(Pose(captures_.back().rvec, captures_.back().tvec));
  }
  if (state_ == SessionState::SOLVED) {
    state_ = SessionState::COLLECTING;
  }
  session_dirty_ = true;
}

void CalibrationServer::CollectSolve() {
  if (!solving_.has_value() || solving_->wait_for(std::chrono::seconds(0)) !=
                                   std::future_status::ready) {
    return;
  }
  absl::StatusOr<IntrinsicsSolution> solution = solving_->get();
  solving_.reset();
  session_dirty_ = true;
  if (state_ == SessionState::IDLE) {
    return;
  }
  if (!solution.ok()) {
    solve_error_ = solution.status().message();
    state_ = SessionState::COLLECTING;
    return;
  }
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> calibration =
      IntrinsicsCalibration::BuildCalibration(
          solution->camera_matrix, solution->dist_coeffs,
          event_loop_->realtime_now(), options_.node_name,
          absl::StrCat("/camera", camera_), camera_id_, options_.team_number,
          solution->reprojection_error, board_,
          DistortionModelOf(solution->model));
  const absl::Status validation = ValidateCalibration(
      calibration.message(), image_size_.width, image_size_.height);
  solved_.emplace(Solved{.solution = *std::move(solution),
                         .calibration = std::move(calibration),
                         .captures = solving_captures_,
                         .validation = validation,
                         .saved_path = "",
                         .download_path = ""});
  state_ = SessionState::SOLVED;
}

absl::Status CalibrationServer::Save(bool force) {
  if (!solved_.has_value()) {
    return absl::FailedPreconditionError("Nothing solved to save");
  }
  if (!std::regex_match(camera_id_, std::regex("^[0-9][0-9]-[0-9][0-9]$"))) {
    return absl::FailedPreconditionError(
        "Set the camera's id (YY-NN) before saving");
  }
  if (!solved_->validation.ok() && !force) {
    return absl::FailedPreconditionError(
        absl::StrCat("It fails validation: ", solved_->validation.message()));
  }
  const aos::realtime_clock::time_point now = event_loop_->realtime_now();
  solved_->calibration = IntrinsicsCalibration::BuildCalibration(
      solved_->solution.camera_matrix, solved_->solution.dist_coeffs, now,
      options_.node_name, absl::StrCat("/camera", camera_), camera_id_,
      options_.team_number, solved_->solution.reprojection_error, board_,
      DistortionModelOf(solved_->solution.model));
  std::stringstream time;
  time << now;
  const std::string path = CalibrationFilename(
      options_.calibration_folder, options_.node_name, options_.team_number,
      camera_, camera_id_, time.str());
  const std::string json =
      aos::FlatbufferToJson(solved_->calibration, {.multi_line = true});
  if (!WriteFile(path, json)) {
    return absl::InternalError(absl::StrCat("Failed to write ", path));
  }
  const std::string name = std::filesystem::path(path).filename();
  solved_->saved_path = path;
  if (WriteFile(std::filesystem::path(options_.www_folder) / name, json)) {
    solved_->download_path = absl::StrCat(options_.www_path, "/", name);
  }
  ABSL_LOG(INFO) << "Saved " << path;
  return absl::OkStatus();
}

absl::Status CalibrationServer::Validate(const calibration::Command &command) {
  if (image_size_.width <= 0) {
    return absl::FailedPreconditionError("No images from the camera yet");
  }
  if (!command.has_checks() || command.checks()->size() == 0) {
    return absl::InvalidArgumentError("Pick a check to make");
  }
  if (command.validate_solved() && !solved_.has_value()) {
    return absl::FailedPreconditionError("Nothing solved to validate");
  }
  constexpr int kGridColumns = 12;
  const auto checks = [&](std::string name, Lens lens) {
    return LensChecks{
        .name = std::move(name),
        .lens = std::move(lens),
        .straightness_px = ValidationGrid(image_size_, kGridColumns),
        .rigidity_mm = ValidationGrid(image_size_, kGridColumns),
        .rigidity_deg = ValidationGrid(image_size_, kGridColumns),
        .tags = TagPairTracker(image_size_, kGridColumns)};
  };
  checks_.assign(command.checks()->begin(), command.checks()->end());
  validations_.clear();
  validations_.push_back(
      checks(current_.has_value() ? "current" : "starting guess", guess_));
  if (command.validate_solved()) {
    validations_.push_back(checks("solved", SolutionLens(solved_->solution)));
  }
  capture_next_ = false;
  state_ = SessionState::VALIDATING;
  return absl::OkStatus();
}

bool CalibrationServer::Checking(calibration::ValidationCheck check) const {
  return std::find(checks_.begin(), checks_.end(), check) != checks_.end();
}

void CalibrationServer::Measure(const cv::Mat &gray,
                                const CharucoDetection &detection) {
  using calibration::ValidationCheck;
  std::vector<Tag> tags;
  if (Checking(ValidationCheck::TWO_TAG)) {
    tags = DetectAprilTags(gray);
  }
  for (LensChecks &checks : validations_) {
    if (Checking(ValidationCheck::STRAIGHTNESS)) {
      if (const auto residuals = StraightnessResiduals(
              cv_board_, detection.ids, detection.corners, checks.lens)) {
        for (size_t i = 0; i < residuals->size(); ++i) {
          checks.straightness_px.Add(cv::norm((*residuals)[i]),
                                     detection.corners[i]);
        }
      }
    }
    if (Checking(ValidationCheck::RIGIDITY)) {
      for (bool across : {true, false}) {
        if (const auto disagreement =
                BoardRigidity(cv_board_, detection.ids, detection.corners,
                              checks.lens, across)) {
          checks.rigidity_mm.Add(disagreement->translation_m * 1000.0,
                                 disagreement->where);
          checks.rigidity_deg.Add(disagreement->rotation_rad * 180.0 / M_PI,
                                  disagreement->where);
        }
      }
    }
    // A rig is two tags; more than a few is the field, not a rig.
    if (tags.size() >= 2 && tags.size() <= 4) {
      for (size_t i = 0; i < tags.size(); ++i) {
        for (size_t j = i + 1; j < tags.size(); ++j) {
          const Tag &a = tags[i].id < tags[j].id ? tags[i] : tags[j];
          const Tag &b = tags[i].id < tags[j].id ? tags[j] : tags[i];
          if (const auto pair = TagPair(a, b, options_.tag_size, checks.lens)) {
            checks.tags.Add({a.id, b.id}, *pair);
          }
        }
      }
    }
  }
  session_dirty_ = true;
}

flatbuffers::Offset<
    flatbuffers::Vector<flatbuffers::Offset<calibration::LensValidation>>>
CalibrationServer::PackValidation(flatbuffers::FlatBufferBuilder *fbb) const {
  using calibration::ValidationCheck;
  const auto pack = [&](ValidationCheck check, std::string_view measure,
                        const ValidationGrid &grid, double scale) {
    std::vector<float> means = grid.Means();
    for (float &mean : means) {
      if (mean >= 0) {
        mean *= scale;
      }
    }
    const auto measure_offset = fbb->CreateString(measure);
    const auto means_offset = fbb->CreateVector(means);
    calibration::CheckResult::Builder result(*fbb);
    result.add_check(check);
    result.add_measure(measure_offset);
    result.add_frames(grid.count());
    result.add_median(grid.Median() * scale);
    result.add_worst(grid.Worst() * scale);
    result.add_columns(grid.columns());
    result.add_rows(grid.rows());
    result.add_means(means_offset);
    return result.Finish();
  };
  std::vector<flatbuffers::Offset<calibration::LensValidation>> lenses;
  for (const LensChecks &checks : validations_) {
    std::vector<flatbuffers::Offset<calibration::CheckResult>> results;
    if (Checking(ValidationCheck::STRAIGHTNESS)) {
      results.push_back(pack(ValidationCheck::STRAIGHTNESS, "corner error, px",
                             checks.straightness_px, 1.0));
    }
    if (Checking(ValidationCheck::RIGIDITY)) {
      results.push_back(pack(ValidationCheck::RIGIDITY, "halves apart, mm",
                             checks.rigidity_mm, 1.0));
      results.push_back(pack(ValidationCheck::RIGIDITY,
                             "halves' angle, degrees", checks.rigidity_deg,
                             1.0));
    }
    std::vector<int> tag_ids;
    if (Checking(ValidationCheck::TWO_TAG)) {
      results.push_back(pack(ValidationCheck::TWO_TAG,
                             "change in tags' distance, mm",
                             checks.tags.DistanceGrid(), 1000.0));
      results.push_back(pack(ValidationCheck::TWO_TAG,
                             "change in tags' angle, degrees",
                             checks.tags.AngleGrid(), 180.0 / M_PI));
      if (const auto pair = checks.tags.pair()) {
        tag_ids = {pair->first, pair->second};
      }
    }
    const auto name = fbb->CreateString(checks.name);
    const auto results_offset = fbb->CreateVector(results);
    const auto tag_ids_offset = fbb->CreateVector(tag_ids);
    calibration::LensValidation::Builder lens(*fbb);
    lens.add_name(name);
    lens.add_results(results_offset);
    lens.add_tag_ids(tag_ids_offset);
    lens.add_tag_distance_m(checks.tags.MedianDistance());
    lenses.push_back(lens.Finish());
  }
  return fbb->CreateVector(lenses);
}

std::filesystem::path CalibrationServer::SessionFolder() const {
  return std::filesystem::path(options_.calibration_folder) / "sessions" /
         session_id_;
}

absl::Status CalibrationServer::ExportImages() {
  ZipWriter zip;
  int count = 0;
  for (const StoredCapture &capture : captures_) {
    if (capture.image_file.empty()) {
      continue;
    }
    std::ifstream file(capture.image_file, std::ios::binary);
    std::stringstream contents;
    contents << file.rdbuf();
    if (!file) {
      return absl::InternalError(
          absl::StrCat("Failed to read ", capture.image_file));
    }
    zip.Add(std::filesystem::path(capture.image_file).filename().string(),
            contents.str());
    ++count;
  }
  if (count == 0) {
    return absl::FailedPreconditionError("No captures' images to export");
  }
  const std::string folder =
      absl::StrFormat("camera%d-%s", camera_, session_id_);
  std::ostringstream board;
  board << board_;
  zip.Add(
      folder + ".txt",
      absl::StrCat(
          count, " captures of camera ", camera_,
          camera_id_.empty() ? "" : absl::StrCat(" (", camera_id_, ")"), ", ",
          image_size_.width, "x", image_size_.height, ", of a ", board.str(),
          ".\n\nTo calibrate from them, unzip them to a folder and run:\n\n"
          "  bazel run -c opt //frc/vision/calibration:calibrate_images -- \\\n"
          "      --images=FOLDER --squares_x=",
          board_.squares_x, " --squares_y=", board_.squares_y,
          " \\\n      --square_length=", board_.square_length,
          " --marker_length=", board_.marker_length, " \\\n      --dictionary=",
          board_.dictionary, " --output=OUTPUT_FOLDER\n"));
  const std::string name = folder + ".zip";
  if (!WriteFile(std::filesystem::path(options_.www_folder) / name,
                 zip.Finish())) {
    return absl::InternalError(absl::StrCat("Failed to write ", name));
  }
  images_download_path_ = absl::StrCat(options_.www_path, "/", name);
  images_exported_ = count;
  return absl::OkStatus();
}

void CalibrationServer::SendStatus() {
  auto builder = status_sender_.MakeBuilder();
  const auto session_id = builder.fbb()->CreateString(session_id_);
  const auto error =
      builder.fbb()->CreateString(!error_.empty() ? error_ : solve_error_);
  calibration::Status::Builder status(*builder.fbb());
  status.add_session_id(session_id);
  status.add_state(state_);
  status.add_camera(camera_);
  status.add_verdict(state_ == SessionState::IDLE ? FrameVerdict::NOT_COLLECTING
                                                  : verdict_);
  status.add_corners(corners_);
  status.add_min_corners(kMinCharucoCorners);
  status.add_moved(moved_);
  status.add_motion(motion_);
  status.add_captures(captures_.size());
  status.add_target_captures(target_captures_);
  status.add_error(error);
  status.add_session_sequence(session_sequence_);
  builder.CheckOk(builder.Send(status.Finish()));
}

void CalibrationServer::MaybeSendSession() {
  if (session_dirty_ && (last_session_sent_ == aos::monotonic_clock::min_time ||
                         event_loop_->monotonic_now() - last_session_sent_ >=
                             std::chrono::seconds(1))) {
    SendSession();
  }
}

void CalibrationServer::SendSession() {
  const size_t max_size = session_sender_.channel()->max_size();
  // Corners are what make it big: leave out the solve's per-corner errors,
  // then the captures' corners, until it fits.
  for (const auto [residuals, corners] :
       {std::pair{true, true}, std::pair{false, true},
        std::pair{false, false}}) {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    fbb.Finish(PackSession(&fbb, residuals, corners));
    if (fbb.GetSize() + 64 > max_size) {
      ABSL_LOG(WARNING) << "The session is " << fbb.GetSize()
                        << " bytes, too big for /calibration's " << max_size
                        << (residuals ? "; leaving out the residuals"
                                      : "; leaving out the corners");
      continue;
    }
    session_sender_.CheckOk(session_sender_.Send(
        aos::FlatbufferDetachedBuffer<calibration::Session>(fbb.Release())));
    ++session_sequence_;
    session_dirty_ = false;
    last_session_sent_ = event_loop_->monotonic_now();
    return;
  }
  ABSL_LOG(ERROR) << "The session doesn't fit even without its corners";
  session_dirty_ = false;
  last_session_sent_ = event_loop_->monotonic_now();
}

flatbuffers::Offset<calibration::Session> CalibrationServer::PackSession(
    flatbuffers::FlatBufferBuilder *fbb, bool residuals, bool corners) const {
  const auto session_id = fbb->CreateString(session_id_);
  const auto camera_id = fbb->CreateString(camera_id_);
  const auto board = PackBoardSpec(fbb, board_);
  const auto board_image_path = fbb->CreateString(board_image_path_);

  std::vector<flatbuffers::Offset<calibration::SavedBoard>> boards;
  for (const NamedBoard &named : Boards()) {
    const auto name = fbb->CreateString(named.name);
    const auto spec = PackBoardSpec(fbb, named.spec);
    const bool preset =
        std::any_of(BoardPresets().begin(), BoardPresets().end(),
                    [&](const NamedBoard &p) { return p.name == named.name; });
    boards.push_back(calibration::CreateSavedBoard(*fbb, name, spec, preset));
  }
  const auto boards_offset = fbb->CreateVector(boards);

  std::vector<flatbuffers::Offset<calibration::Capture>> captures;
  for (const StoredCapture &stored : captures_) {
    std::vector<calibration::Corner> points;
    if (corners) {
      for (size_t i = 0; i < stored.capture.ids.size(); ++i) {
        points.emplace_back(stored.capture.corners[i].x,
                            stored.capture.corners[i].y,
                            static_cast<int16_t>(stored.capture.ids[i]));
      }
    }
    const auto points_offset = fbb->CreateVectorOfStructs(points);
    const auto rvec = fbb->CreateVector(
        std::vector<double>{stored.rvec[0], stored.rvec[1], stored.rvec[2]});
    const auto tvec = fbb->CreateVector(
        std::vector<double>{stored.tvec[0], stored.tvec[1], stored.tvec[2]});
    captures.push_back(calibration::CreateCapture(
        *fbb, stored.monotonic_timestamp_ns, points_offset, rvec, tvec));
  }
  const auto captures_offset = fbb->CreateVector(captures);

  flatbuffers::Offset<calibration::CameraCalibration> current;
  if (current_.has_value()) {
    current = aos::RecursiveCopyFlatBuffer(&current_->message(), fbb);
  }

  flatbuffers::Offset<calibration::Solve> solve;
  if (solved_.has_value() || !solve_error_.empty()) {
    flatbuffers::Offset<calibration::CameraCalibration> calibration;
    flatbuffers::Offset<flatbuffers::Vector<double>> capture_errors;
    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<calibration::CaptureResiduals>>>
        residuals_offset;
    flatbuffers::Offset<flatbuffers::String> validation_error, saved_path,
        download_path;
    if (solved_.has_value()) {
      calibration =
          aos::RecursiveCopyFlatBuffer(&solved_->calibration.message(), fbb);
      capture_errors = fbb->CreateVector(solved_->solution.per_view_errors);
      if (residuals) {
        std::vector<flatbuffers::Offset<calibration::CaptureResiduals>>
            per_capture;
        for (const std::vector<cv::Point2f> &capture :
             solved_->solution.residuals) {
          std::vector<calibration::Residual> values;
          for (const cv::Point2f &residual : capture) {
            values.emplace_back(Hundredths(residual.x), Hundredths(residual.y));
          }
          per_capture.push_back(calibration::CreateCaptureResiduals(
              *fbb, fbb->CreateVectorOfStructs(values)));
        }
        residuals_offset = fbb->CreateVector(per_capture);
      }
      validation_error = fbb->CreateString(solved_->validation.message());
      saved_path = fbb->CreateString(solved_->saved_path);
      download_path = fbb->CreateString(solved_->download_path);
    }
    const auto error = fbb->CreateString(solve_error_);
    calibration::Solve::Builder builder(*fbb);
    if (solved_.has_value()) {
      builder.add_calibration(calibration);
      builder.add_reprojection_error(solved_->solution.reprojection_error);
      builder.add_capture_errors(capture_errors);
      if (!residuals_offset.IsNull()) {
        builder.add_residuals(residuals_offset);
      }
      builder.add_valid(solved_->validation.ok());
      builder.add_validation_error(validation_error);
      builder.add_saved_path(saved_path);
      builder.add_download_path(download_path);
      builder.add_captures(solved_->captures);
    }
    builder.add_error(error);
    solve = builder.Finish();
  }

  const auto validation = PackValidation(fbb);
  const auto images_download_path = fbb->CreateString(images_download_path_);
  const auto checks = fbb->CreateVector(checks_);

  calibration::Session::Builder session(*fbb);
  session.add_session_id(session_id);
  session.add_state(state_);
  session.add_camera(camera_);
  session.add_camera_id(camera_id);
  session.add_image_width(image_size_.width);
  session.add_image_height(image_size_.height);
  session.add_board(board);
  session.add_board_image_path(board_image_path);
  session.add_boards(boards_offset);
  session.add_target_captures(target_captures_);
  session.add_captures(captures_offset);
  if (!current.IsNull()) {
    session.add_current(current);
  }
  if (!solve.IsNull()) {
    session.add_solve(solve);
  }
  session.add_validation(validation);
  session.add_images_download_path(images_download_path);
  session.add_images_exported(images_exported_);
  session.add_checks(checks);
  return session.Finish();
}

std::vector<NamedBoard> CalibrationServer::Boards() const {
  std::vector<NamedBoard> boards = BoardPresets();
  boards.insert(boards.end(), saved_boards_.begin(), saved_boards_.end());
  return boards;
}

void CalibrationServer::LoadStore() {
  if (!std::filesystem::exists(options_.board_store)) {
    return;
  }
  std::ifstream file(options_.board_store);
  std::stringstream contents;
  contents << file.rdbuf();
  const aos::FlatbufferDetachedBuffer<calibration::BoardStore> store =
      aos::JsonToFlatbuffer<calibration::BoardStore>(contents.str());
  if (store.span().size() == 0) {
    ABSL_LOG(ERROR) << "Failed to read " << options_.board_store;
    return;
  }
  if (store.message().has_boards()) {
    for (const calibration::SavedBoard *board : *store.message().boards()) {
      if (board->has_name() && board->has_board()) {
        saved_boards_.push_back(
            {board->name()->str(), BoardSpecFromFlatbuffer(*board->board())});
      }
    }
  }
  if (store.message().has_last_boards()) {
    for (const calibration::CameraBoard *board :
         *store.message().last_boards()) {
      if (board->has_board()) {
        last_boards_[board->camera()] =
            BoardSpecFromFlatbuffer(*board->board());
      }
    }
  }
  if (store.message().target_captures() > 0) {
    target_captures_ = store.message().target_captures();
  }
}

void CalibrationServer::WriteStore() {
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<calibration::SavedBoard>> boards;
  for (const NamedBoard &named : saved_boards_) {
    const auto name = fbb.CreateString(named.name);
    const auto spec = PackBoardSpec(&fbb, named.spec);
    boards.push_back(calibration::CreateSavedBoard(fbb, name, spec, false));
  }
  std::vector<flatbuffers::Offset<calibration::CameraBoard>> last_boards;
  for (const auto &[camera, spec] : last_boards_) {
    if (camera < 0) {
      continue;
    }
    last_boards.push_back(
        calibration::CreateCameraBoard(fbb, camera, PackBoardSpec(&fbb, spec)));
  }
  fbb.Finish(calibration::CreateBoardStore(fbb, fbb.CreateVector(boards),
                                           fbb.CreateVector(last_boards),
                                           target_captures_));
  aos::FlatbufferDetachedBuffer<calibration::BoardStore> store(fbb.Release());
  WriteFile(options_.board_store,
            aos::FlatbufferToJson(store, {.multi_line = true}));
}

}  // namespace frc::vision

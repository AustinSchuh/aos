#include <filesystem>
#include <random>

#include "gtest/gtest.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "frc/vision/calibration/calibration_server_lib.h"
#include "frc/vision/calibration/charuco_detector.h"

namespace frc::vision::testing {
namespace {

using calibration::CommandAction;
using calibration::FrameVerdict;
using calibration::SessionState;

constexpr int kWidth = 1600, kHeight = 1304;

const cv::Mat &TrueCameraMatrix() {
  static const cv::Mat kMatrix =
      (cv::Mat_<double>(3, 3) << 1000, 0, 805, 0, 1000, 650, 0, 0, 1);
  return kMatrix;
}

// A command's arguments.
struct Args {
  int camera = 0;
  int target_captures = 0;
  std::optional<BoardSpec> board;
  std::string board_name;
  std::string camera_id;
  bool force = false;
  std::vector<calibration::ValidationCheck> checks;
  bool validate_solved = false;

  Args &Checks(std::vector<calibration::ValidationCheck> value, bool solved) {
    checks = std::move(value);
    validate_solved = solved;
    return *this;
  }
  Args &Camera(int value) {
    camera = value;
    return *this;
  }
  Args &Target(int value) {
    target_captures = value;
    return *this;
  }
  Args &Board(const BoardSpec &value) {
    board = value;
    return *this;
  }
  Args &BoardName(std::string value) {
    board_name = std::move(value);
    return *this;
  }
  Args &CameraId(std::string value) {
    camera_id = std::move(value);
    return *this;
  }
  Args &Force() {
    force = true;
    return *this;
  }
};

// Draws a board, and maps its picture's pixels to the board's plane.
class BoardRenderer {
 public:
  explicit BoardRenderer(const BoardSpec &spec)
      : board_(MakeCharucoBoard(spec)) {
    const cv::Size size(spec.squares_x * 100, spec.squares_y * 100);
#if CV_VERSION_MINOR >= 9
    board_->generateImage(size, flat_, 0, 1);
#else
    board_->draw(size, flat_, 0, 1);
#endif
    // Find the corners in the flat picture to learn its mapping, whatever
    // OpenCV's conventions.
    const cv::Mat identity =
        (cv::Mat_<double>(3, 3) << 1000, 0, size.width / 2.0, 0, 1000,
         size.height / 2.0, 0, 0, 1);
    const CharucoDetection detection =
        DetectCharuco(board_, flat_, identity, cv::Mat::zeros(5, 1, CV_64F));
#if CV_VERSION_MINOR >= 9
    const std::vector<cv::Point3f> corners = board_->getChessboardCorners();
#else
    const std::vector<cv::Point3f> &corners = board_->chessboardCorners;
#endif
    std::vector<cv::Point2f> plane;
    for (int id : detection.ids) {
      plane.emplace_back(corners[id].x, corners[id].y);
    }
    flat_to_plane_ = cv::findHomography(detection.corners, plane);
  }

  // The board at a pose, board to camera, as the true camera sees it.
  cv::Mat Render(const cv::Vec3d &rvec, const cv::Vec3d &tvec) const {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat plane_to_camera(3, 3, CV_64F);
    R.col(0).copyTo(plane_to_camera.col(0));
    R.col(1).copyTo(plane_to_camera.col(1));
    cv::Mat(tvec).copyTo(plane_to_camera.col(2));
    const cv::Mat H = TrueCameraMatrix() * plane_to_camera * flat_to_plane_;
    cv::Mat image;
    cv::warpPerspective(flat_, image, H, cv::Size(kWidth, kHeight),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
    return image;
  }

 private:
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Mat flat_;
  cv::Mat flat_to_plane_;
};

class CalibrationServerTest : public ::testing::Test {
 protected:
  CalibrationServerTest()
      : config_(aos::configuration::ReadConfig(aos::testing::ArtifactPath(
            "frc/vision/calibration/calibration_server_test_config.json"))),
        factory_(&config_.message()),
        server_loop_(factory_.MakeEventLoop("calibration_server")),
        test_loop_(factory_.MakeEventLoop("test")),
        command_sender_(
            test_loop_->MakeSender<calibration::Command>("/calibration")),
        image_sender_(test_loop_->MakeSender<CameraImage>("/camera0/gray")),
        status_fetcher_(
            test_loop_->MakeFetcher<calibration::Status>("/calibration")),
        session_fetcher_(
            test_loop_->MakeFetcher<calibration::Session>("/calibration")),
        annotations_fetcher_(
            test_loop_->MakeFetcher<foxglove::ImageAnnotations>(
                "/camera0/calibration")),
        renderer_(BoardSpec{}) {
    tmp_ = std::filesystem::path(aos::testing::TestTmpDir()) /
           "calibration_server_test";
    std::filesystem::remove_all(tmp_);
    SendConstants();
    MakeServer();
    image_timer_ = test_loop_->AddTimer([this]() { SendImage(); });
    test_loop_->OnRun([this]() {
      image_timer_->Schedule(test_loop_->monotonic_now(),
                             std::chrono::milliseconds(33));
    });
    // Watchers only see what's sent once they're running.
    factory_.RunFor(std::chrono::milliseconds(100));
  }

  // The camera's calibration, a little off.
  void SendConstants() {
    auto sender = test_loop_->MakeSender<CameraConstants>("/constants");
    auto builder = sender.MakeBuilder();
    const auto node = builder.fbb()->CreateString("orin");
    const auto id = builder.fbb()->CreateString("25-01");
    const auto intrinsics = builder.fbb()->CreateVector(
        std::vector<float>{980, 0, 800, 0, 980, 655, 0, 0, 1});
    const auto dist =
        builder.fbb()->CreateVector(std::vector<float>{0, 0, 0, 0, 0});
    calibration::CameraCalibration::Builder calibration(*builder.fbb());
    calibration.add_node_name(node);
    calibration.add_camera_number(0);
    calibration.add_camera_id(id);
    calibration.add_intrinsics(intrinsics);
    calibration.add_dist_coeffs(dist);
    const auto offset = calibration.Finish();
    const auto list = builder.fbb()->CreateVector(
        std::vector<flatbuffers::Offset<calibration::CameraCalibration>>{
            offset});
    CameraConstants::Builder constants(*builder.fbb());
    constants.add_calibration(list);
    builder.CheckOk(builder.Send(constants.Finish()));
  }

  void SendImage() {
    if (image_.empty()) {
      return;
    }
    auto builder = image_sender_.MakeBuilder();
    const auto data = builder.fbb()->CreateVector(
        image_.data, image_.total() * image_.elemSize());
    CameraImage::Builder image(*builder.fbb());
    image.add_rows(image_.rows);
    image.add_cols(image_.cols);
    image.add_format(ImageFormat::MONO8);
    image.add_data(data);
    image.add_monotonic_timestamp_ns(
        test_loop_->monotonic_now().time_since_epoch().count());
    builder.CheckOk(builder.Send(image.Finish()));
  }

  void Command(CommandAction action, const Args &args = Args()) {
    auto builder = command_sender_.MakeBuilder();
    flatbuffers::FlatBufferBuilder *fbb = builder.fbb();
    flatbuffers::Offset<calibration::BoardSpec> board;
    if (args.board.has_value()) {
      board = PackBoardSpec(fbb, *args.board);
    }
    const auto board_name = fbb->CreateString(args.board_name);
    const auto camera_id = fbb->CreateString(args.camera_id);
    const auto checks = fbb->CreateVector(args.checks);
    calibration::Command::Builder command(*fbb);
    command.add_action(action);
    command.add_camera(args.camera);
    command.add_target_captures(args.target_captures);
    if (!board.IsNull()) {
      command.add_board(board);
    }
    if (!args.board_name.empty()) {
      command.add_board_name(board_name);
    }
    if (!args.camera_id.empty()) {
      command.add_camera_id(camera_id);
    }
    command.add_force(args.force);
    command.add_checks(checks);
    command.add_validate_solved(args.validate_solved);
    builder.CheckOk(builder.Send(command.Finish()));
    factory_.RunFor(std::chrono::milliseconds(10));
  }

  void RestartServer() {
    server_.reset();
    server_loop_.reset();
    server_loop_ = factory_.MakeEventLoop("calibration_server");
    MakeServer();
    factory_.RunFor(std::chrono::milliseconds(100));
  }

  void MakeServer() {
    server_.emplace(server_loop_.get(),
                    CalibrationServer::Options{
                        .node_name = "orin",
                        .team_number = 1868,
                        .cameras = {0},
                        .calibration_folder = tmp_ / "calibration",
                        .www_folder = tmp_ / "www" / "calibration",
                        .www_path = "/calibration",
                        .board_store = tmp_ / "boards.json",
                        .period = std::chrono::milliseconds(100),
                        .tag_size = 0.1651,
                    });
  }

  // The latest status.
  const calibration::Status &Status() {
    status_fetcher_.Fetch();
    EXPECT_NE(status_fetcher_.get(), nullptr);
    return *status_fetcher_;
  }

  const calibration::Session &Session() {
    // Long enough for the next one.
    factory_.RunFor(std::chrono::milliseconds(1100));
    session_fetcher_.Fetch();
    EXPECT_NE(session_fetcher_.get(), nullptr);
    return *session_fetcher_;
  }

  // Holds the board still at each of `count` poses, long enough to capture.
  void ShowPoses(int count) {
    std::mt19937 random(971);
    std::uniform_real_distribution<double> tilt(-0.5, 0.5), shift(-0.25, 0.25),
        distance(0.9, 1.4);
    for (int i = 0; i < count; ++i) {
      const cv::Vec3d rvec(tilt(random), tilt(random), tilt(random) / 3);
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      const cv::Mat t =
          cv::Mat(cv::Vec3d(shift(random), shift(random), distance(random))) -
          R * cv::Mat(cv::Vec3d(0.28, 0.18, 0.0));
      image_ = renderer_.Render(rvec, cv::Vec3d(t));
      factory_.RunFor(std::chrono::milliseconds(600));
    }
  }

  // Runs until the solve finishes: it's on another thread, in real time.
  void WaitForSolve() {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::minutes(2)) {
      factory_.RunFor(std::chrono::milliseconds(200));
      if (status_fetcher_.Fetch() &&
          status_fetcher_->state() != SessionState::SOLVING) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    FAIL() << "The solve didn't finish";
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory factory_;
  std::unique_ptr<aos::EventLoop> server_loop_;
  std::unique_ptr<aos::EventLoop> test_loop_;
  aos::Sender<calibration::Command> command_sender_;
  aos::Sender<CameraImage> image_sender_;
  aos::Fetcher<calibration::Status> status_fetcher_;
  aos::Fetcher<calibration::Session> session_fetcher_;
  aos::Fetcher<foxglove::ImageAnnotations> annotations_fetcher_;
  aos::TimerHandler *image_timer_;
  BoardRenderer renderer_;
  cv::Mat image_;
  std::filesystem::path tmp_;
  std::optional<CalibrationServer> server_;
};

TEST_F(CalibrationServerTest, IdleUntilStarted) {
  factory_.RunFor(std::chrono::seconds(1));
  EXPECT_EQ(Status().state(), SessionState::IDLE);
  EXPECT_EQ(Status().verdict(), FrameVerdict::NOT_COLLECTING);
  Command(CommandAction::CAPTURE);
  EXPECT_NE(Status().error()->string_view(), "");
}

TEST_F(CalibrationServerTest, CalibratesSavesAndRemembers) {
  Command(CommandAction::START, Args().Target(12));
  EXPECT_EQ(Status().state(), SessionState::COLLECTING);
  EXPECT_EQ(Status().error()->string_view(), "");
  {
    const calibration::Session &session = Session();
    EXPECT_EQ(session.camera(), 0);
    EXPECT_EQ(session.camera_id()->string_view(), "25-01");
    EXPECT_EQ(BoardSpecFromFlatbuffer(*session.board()), BoardSpec{});
    EXPECT_GE(session.boards()->size(), BoardPresets().size());
    EXPECT_TRUE(session.has_current());
    EXPECT_EQ(session.target_captures(), 12);
    ASSERT_TRUE(session.has_board_image_path());
    EXPECT_TRUE(std::filesystem::exists(tmp_ / "www/calibration/board.png"));
  }

  // No board: no corners.
  image_ = cv::Mat(kHeight, kWidth, CV_8UC1, cv::Scalar(128));
  factory_.RunFor(std::chrono::milliseconds(300));
  EXPECT_EQ(Status().verdict(), FrameVerdict::TOO_FEW_CORNERS);

  const int sequence = Status().session_sequence();
  ShowPoses(16);
  EXPECT_EQ(Status().captures(), 12);
  EXPECT_GT(Status().session_sequence(), sequence);
  EXPECT_EQ(Status().verdict(), FrameVerdict::TARGET_REACHED);
  ASSERT_TRUE(annotations_fetcher_.Fetch());
  EXPECT_GT(annotations_fetcher_->points()->Get(0)->points()->size(), 15u);
  {
    const calibration::Session &session = Session();
    ASSERT_EQ(session.captures()->size(), 12u);
    EXPECT_GE(session.captures()->Get(0)->corners()->size(), 15u);
  }

  // The board can't change under the captures.
  Command(CommandAction::SET_BOARD, Args().Board(BoardPresets()[1].spec));
  EXPECT_NE(Status().error()->string_view(), "");

  Command(CommandAction::UNDO);
  EXPECT_EQ(Status().captures(), 11);

  Command(CommandAction::SOLVE);
  WaitForSolve();
  EXPECT_EQ(Status().state(), SessionState::SOLVED);
  {
    const calibration::Session &session = Session();
    ASSERT_TRUE(session.has_solve());
    const calibration::Solve &solve = *session.solve();
    EXPECT_EQ(solve.error()->string_view(), "");
    EXPECT_LT(solve.reprojection_error(), 0.5);
    EXPECT_EQ(solve.captures(), 11);
    ASSERT_EQ(solve.capture_errors()->size(), 11u);
    ASSERT_EQ(solve.residuals()->size(), 11u);
    const auto *intrinsics = solve.calibration()->intrinsics();
    EXPECT_NEAR(intrinsics->Get(0), 1000.0, 10.0);
    EXPECT_NEAR(intrinsics->Get(2), 805.0, 10.0);
    EXPECT_NEAR(intrinsics->Get(5), 650.0, 10.0);
    EXPECT_EQ(BoardSpecFromFlatbuffer(*solve.calibration()->board()),
              BoardSpec{});
  }

  // Validate the current and solved calibrations on new frames: the solved
  // one, from these very images, fits; the current one, a little off, less.
  Command(CommandAction::VALIDATE,
          Args().Checks({calibration::ValidationCheck::STRAIGHTNESS,
                         calibration::ValidationCheck::RIGIDITY},
                        true));
  EXPECT_EQ(Status().error()->string_view(), "");
  EXPECT_EQ(Status().state(), SessionState::VALIDATING);
  Command(CommandAction::CAPTURE);
  EXPECT_NE(Status().error()->string_view(), "") << "Not while validating";
  ShowPoses(4);
  {
    const calibration::Session &session = Session();
    ASSERT_EQ(session.validation()->size(), 2u);
    const auto *current = session.validation()->Get(0);
    const auto *solved = session.validation()->Get(1);
    EXPECT_EQ(current->name()->string_view(), "current");
    EXPECT_EQ(solved->name()->string_view(), "solved");
    // Straightness, then rigidity's distance and angle.
    ASSERT_EQ(solved->results()->size(), 3u);
    const auto *straightness = solved->results()->Get(0);
    EXPECT_GT(straightness->frames(), 50);
    EXPECT_LT(straightness->median(), 0.5);
    EXPECT_EQ(
        straightness->means()->size(),
        static_cast<size_t>(straightness->columns() * straightness->rows()));
    EXPECT_GT(solved->results()->Get(1)->frames(), 0);
    EXPECT_GT(current->results()->Get(1)->median(),
              solved->results()->Get(1)->median());
  }
  Command(CommandAction::STOP_VALIDATING);
  EXPECT_EQ(Status().state(), SessionState::SOLVED);

  Command(CommandAction::SAVE, Args().Force());
  EXPECT_EQ(Status().error()->string_view(), "");
  {
    const calibration::Session &session = Session();
    const std::filesystem::path saved =
        session.solve()->saved_path()->string_view();
    EXPECT_TRUE(std::filesystem::exists(saved)) << saved;
    EXPECT_EQ(saved.parent_path(), tmp_ / "calibration");
    EXPECT_NE(
        saved.filename().string().find("calibration_orin-1868-0_cam-25-01"),
        std::string::npos)
        << saved;
    EXPECT_EQ(session.solve()->download_path()->string_view(),
              "/calibration/" + saved.filename().string());
    EXPECT_TRUE(
        std::filesystem::exists(tmp_ / "www/calibration" / saved.filename()));
  }

  // The target is remembered for next time.
  Command(CommandAction::FINISH);
  RestartServer();
  Command(CommandAction::START);
  EXPECT_EQ(Session().target_captures(), 12);
}

TEST_F(CalibrationServerTest, BoardsAndRefusals) {
  Command(CommandAction::START);
  // A board too big for its dictionary.
  Command(CommandAction::SET_BOARD,
          Args().Board(BoardSpec{20, 20, 0.04, 0.03, "DICT_5X5_100"}));
  EXPECT_NE(Status().error()->string_view(), "");

  const BoardSpec mine{10, 7, 0.05, 0.035, "DICT_4X4_50"};
  Command(CommandAction::SET_BOARD, Args().Board(mine));
  EXPECT_EQ(Status().error()->string_view(), "");
  EXPECT_EQ(BoardSpecFromFlatbuffer(*Session().board()), mine);

  Command(CommandAction::SAVE_BOARD, Args().Board(mine));
  EXPECT_NE(Status().error()->string_view(), "") << "A board needs a name";
  Command(CommandAction::SAVE_BOARD, Args().Board(mine).BoardName("Mine"));
  EXPECT_EQ(Status().error()->string_view(), "");
  {
    const calibration::Session &session = Session();
    ASSERT_EQ(session.boards()->size(), BoardPresets().size() + 1);
    EXPECT_EQ(
        session.boards()->Get(BoardPresets().size())->name()->string_view(),
        "Mine");
    EXPECT_FALSE(session.boards()->Get(BoardPresets().size())->preset());
  }
  Command(CommandAction::DELETE_BOARD,
          Args().BoardName(BoardPresets()[0].name));
  EXPECT_NE(Status().error()->string_view(), "") << "Presets stay";

  // Another session on the same camera starts on the last board used, and
  // the saved board survives a restart.
  RestartServer();
  Command(CommandAction::START);
  {
    const calibration::Session &session = Session();
    EXPECT_EQ(BoardSpecFromFlatbuffer(*session.board()), mine);
    EXPECT_EQ(session.boards()->size(), BoardPresets().size() + 1);
  }

  // A camera it doesn't have.
  Command(CommandAction::START, Args().Camera(3));
  EXPECT_NE(Status().error()->string_view(), "");

  // A camera id that isn't YY-NN.
  Command(CommandAction::SET_CAMERA_ID, Args().CameraId("cam 1"));
  EXPECT_NE(Status().error()->string_view(), "");
  Command(CommandAction::SET_CAMERA_ID, Args().CameraId("26-03"));
  EXPECT_EQ(Status().error()->string_view(), "");
  EXPECT_EQ(Session().camera_id()->string_view(), "26-03");
}

}  // namespace
}  // namespace frc::vision::testing

// The vision robot, simulated, for using and testing its pages end to end on a
// laptop or a test machine:
//
//   bazel run //frc/vision/sim:vision_robot_sim -- --data_dir=/tmp/www ...
//
// Runs, on one SimulatedEventLoopFactory in real time, the robot's own
// applications with the robot's configuration:
//   - robot_server, started with its arguments from the configuration (its
//     sendable channels and calibration session) on the ports given here,
//     over HTTPS with --tls_dir's certificate;
//   - web_proxy, which the robot's pages' data channels come through;
//   - calibration_server, started and stopped by robot_server through starter,
//     as on the robot;
//   - field_side_exposure_adjuster, with the exposure and gain the page sets;
//   - the vision constants, for --robot_name, from constants.json;
// and in place of the hardware:
//   - cameras: each of --cameras shows a ChArUco board (the calibration
//     server's default board), moving to a new pose every --pose_seconds, on
//     /cameraN/gray;
//   - starter: answers StarterRpc for calibration_server by running it, and
//     for the other applications by saying so; sends starter's Status.
//
// camera_streamer joins it once the amd64 sysroot can run it (see
// frc/amd64/build_rootfs.py); until then, nothing serves /cameras/ws.
//
// Prints "Serving <url>" once it is.

#include <stdlib.h>

#include <filesystem>
#include <iostream>
#include <map>
#include <random>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "opencv2/calib3d.hpp"

#include "aos/configuration.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/events/tokio_simulation.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/calibration/calibration_server_lib.h"
#include "frc/vision/calibration/charuco_rendering.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/camera_constants_list_generated.h"
#include "frc/vision/field_side_exposure_adjuster_lib.h"
#include "frc/vision/util/charuco_board.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, config, "frc/vision/aos_config.json",
          "The robot's AOS configuration.");
ABSL_FLAG(std::string, constants, "frc/vision/testdata/constants.json",
          "The vision constants.  The test robot's by default; a real "
          "robot's are in bazel-bin/frc/vision/constants.json.");
ABSL_FLAG(std::string, robot_name, "test",
          "The robot whose constants to send.");
ABSL_FLAG(uint16_t, team_number, 0, "Its team number.");
ABSL_FLAG(std::string, node, "orin", "The node.");
ABSL_FLAG(std::string, data_dir, "",
          "The pages, as robot_server and web_proxy serve them; writable: "
          "calibration_server writes into its calibration/.");
ABSL_FLAG(std::string, state_dir, "",
          "Where the robot keeps state: calibrations, boards, camera "
          "settings.");
ABSL_FLAG(std::string, tls_dir, "",
          "robot_server's certificate: server-chain.crt and server.key.");
ABSL_FLAG(uint16_t, https_port, 8443, "robot_server's HTTPS port.");
ABSL_FLAG(uint16_t, http_port, 8080, "robot_server's HTTP port.");
ABSL_FLAG(uint16_t, camera_streamer_port, 1190,
          "Where robot_server proxies /cameras/ws to.");
ABSL_FLAG(std::string, cameras, "0,2", "The cameras that show a board.");
ABSL_FLAG(double, pose_seconds, 1.0, "How long the board holds each pose.");
ABSL_DECLARE_FLAG(int32_t, proxy_port);

extern "C" {
// From aos/network/robot_server/lib.rs.
bool robot_server_add_to_simulation_with_flags(
    aos_tokio_simulation_t *simulation, aos_event_loop_t *event_loop,
    const uint8_t *config, size_t config_size, int argc, const char **argv);
}

namespace frc::vision {
namespace {

constexpr int kWidth = 1600, kHeight = 1304;

// The board, as each camera sees it, at a few poses.
std::vector<cv::Mat> RenderPoses(int count) {
  const cv::Ptr<cv::aruco::CharucoBoard> board = MakeCharucoBoard(BoardSpec{});
  const cv::Mat K =
      (cv::Mat_<double>(3, 3) << 1000, 0, 800, 0, 1000, 652, 0, 0, 1);
  std::mt19937 random(971);
  std::uniform_real_distribution<double> tilt(-0.5, 0.5), shift(-0.25, 0.25),
      distance(0.9, 1.4);
  std::vector<cv::Mat> images;
  for (int i = 0; i < count; ++i) {
    const cv::Vec3d rvec(tilt(random), tilt(random), tilt(random) / 3);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    const cv::Mat t =
        cv::Mat(cv::Vec3d(shift(random), shift(random), distance(random))) -
        R * cv::Mat(cv::Vec3d(0.28, 0.18, 0.0));
    images.push_back(testing::RenderBoard(board, K, cv::Size(kWidth, kHeight),
                                          rvec, cv::Vec3d(t), /*supersample=*/2)
                         .image);
  }
  return images;
}

// The cameras: each sends the board, at the current pose, ten times a second.
class FakeCameras {
 public:
  FakeCameras(aos::EventLoop *event_loop, std::vector<int> cameras,
              std::chrono::milliseconds pose_period)
      : event_loop_(event_loop), poses_(RenderPoses(16)) {
    for (int camera : cameras) {
      senders_.push_back(event_loop->MakeSender<CameraImage>(
          absl::StrCat("/camera", camera, "/gray")));
    }
    aos::TimerHandler *frame = event_loop->AddTimer([this]() { Send(); });
    aos::TimerHandler *pose =
        event_loop->AddTimer([this]() { pose_ = (pose_ + 1) % poses_.size(); });
    event_loop->OnRun([this, frame, pose, pose_period]() {
      frame->Schedule(event_loop_->monotonic_now(),
                      std::chrono::milliseconds(100));
      pose->Schedule(event_loop_->monotonic_now() + pose_period, pose_period);
    });
  }

 private:
  void Send() {
    const cv::Mat &image = poses_[pose_];
    for (aos::Sender<CameraImage> &sender : senders_) {
      auto builder = sender.MakeBuilder();
      const auto data = builder.fbb()->CreateVector(image.data, image.total());
      CameraImage::Builder message(*builder.fbb());
      message.add_rows(image.rows);
      message.add_cols(image.cols);
      message.add_format(ImageFormat::MONO8);
      message.add_data(data);
      message.add_monotonic_timestamp_ns(
          event_loop_->monotonic_now().time_since_epoch().count());
      builder.CheckOk(builder.Send(message.Finish()));
    }
  }

  aos::EventLoop *const event_loop_;
  const std::vector<cv::Mat> poses_;
  std::vector<aos::Sender<CameraImage>> senders_;
  size_t pose_ = 0;
};

// Starter, for the applications this simulation has: runs calibration_server
// when told to, and says every application is in whatever state it was last
// told, starting as the configuration says.
class FakeStarter {
 public:
  FakeStarter(aos::SimulatedEventLoopFactory *factory,
              aos::EventLoop *event_loop, const aos::Node *node,
              CalibrationServer::Options calibration_options)
      : factory_(factory),
        event_loop_(event_loop),
        node_(node),
        calibration_options_(std::move(calibration_options)),
        status_sender_(event_loop->MakeSender<aos::starter::Status>("/aos")) {
    for (const aos::Application *application :
         *event_loop->configuration()->applications()) {
      const std::string name = application->name()->str();
      apps_[name] = {
          .running = !application->has_autostart() || application->autostart(),
          .pid = next_pid_++,
          .start_ns = 0};
    }
    event_loop->MakeWatcher(
        "/aos", [this](const aos::starter::StarterRpc &rpc) { Handle(rpc); });
    aos::TimerHandler *timer = event_loop->AddTimer([this]() { SendStatus(); });
    event_loop->OnRun([this, timer]() {
      timer->Schedule(event_loop_->monotonic_now(), std::chrono::seconds(1));
    });
  }

 private:
  struct App {
    bool running;
    uint32_t pid;
    int64_t start_ns;
  };

  void Handle(const aos::starter::StarterRpc &rpc) {
    const std::string name = rpc.name()->str();
    auto it = apps_.find(name);
    if (it == apps_.end()) {
      ABSL_LOG(WARNING) << "starter: no application " << name;
      return;
    }
    const bool start = rpc.command() != aos::starter::Command::STOP;
    ABSL_LOG(INFO) << "starter: "
                   << aos::starter::EnumNameCommand(rpc.command()) << " "
                   << name;
    if (name == "calibration_server") {
      // As starter would: stop it, and start a new one.  The simulation is
      // running, so either needs its leave: it pauses while they do.
      aos::NodeEventLoopFactory *node_factory =
          factory_->GetNodeEventLoopFactory(node_);
      factory_->AllowApplicationCreationDuring([&]() {
        if (calibration_server_ != nullptr) {
          node_factory->Stop(calibration_server_);
          calibration_server_ = nullptr;
        }
        if (start) {
          calibration_server_ = node_factory->AlwaysStart<CalibrationServer>(
              "calibration_server", calibration_options_);
        }
      });
    }
    App &app = it->second;
    if (start) {
      app.pid = next_pid_++;
      app.start_ns = event_loop_->monotonic_now().time_since_epoch().count();
    }
    app.running = start;
    SendStatus();
  }

  void SendStatus() {
    auto builder = status_sender_.MakeBuilder();
    std::vector<flatbuffers::Offset<aos::starter::ApplicationStatus>> statuses;
    for (const auto &[name, app] : apps_) {
      const auto name_offset = builder.fbb()->CreateString(name);
      aos::starter::ApplicationStatus::Builder status(*builder.fbb());
      status.add_name(name_offset);
      status.add_state(app.running ? aos::starter::State::RUNNING
                                   : aos::starter::State::STOPPED);
      status.add_pid(app.running ? app.pid : 0);
      status.add_last_start_time(app.start_ns);
      status.add_file_state(app.running ? aos::starter::FileState::NO_CHANGE
                                        : aos::starter::FileState::NOT_RUNNING);
      statuses.push_back(status.Finish());
    }
    const auto list = builder.fbb()->CreateVector(statuses);
    aos::starter::Status::Builder status(*builder.fbb());
    status.add_statuses(list);
    builder.CheckOk(builder.Send(status.Finish()));
  }

  aos::SimulatedEventLoopFactory *const factory_;
  aos::EventLoop *const event_loop_;
  const aos::Node *const node_;
  const CalibrationServer::Options calibration_options_;
  aos::Sender<aos::starter::Status> status_sender_;
  std::map<std::string, App> apps_;
  uint32_t next_pid_ = 1000;
  // Running when it's been started, owned by the simulation.
  CalibrationServer *calibration_server_ = nullptr;
};

// robot_server's arguments in the configuration, with this simulation's ports
// and folders.
std::vector<std::string> RobotServerArgs(const aos::Configuration *config) {
  std::vector<std::string> args = {"robot_server"};
  for (const aos::Application *application : *config->applications()) {
    if (application->name()->string_view() == "robot_server" &&
        application->has_args()) {
      for (const flatbuffers::String *arg : *application->args()) {
        args.push_back(arg->str());
      }
    }
  }
  args.push_back(
      absl::StrCat("--https-port=", absl::GetFlag(FLAGS_https_port)));
  args.push_back(absl::StrCat("--http-port=", absl::GetFlag(FLAGS_http_port)));
  args.push_back(absl::StrCat("--data-dir=", absl::GetFlag(FLAGS_data_dir)));
  args.push_back(absl::StrCat("--tls-dir=", absl::GetFlag(FLAGS_tls_dir)));
  args.push_back(
      absl::StrCat("--web-proxy-port=", absl::GetFlag(FLAGS_proxy_port)));
  args.push_back(absl::StrCat("--camera-streamer-port=",
                              absl::GetFlag(FLAGS_camera_streamer_port)));
  return args;
}

int Main() {
  ABSL_CHECK(!absl::GetFlag(FLAGS_data_dir).empty()) << "--data_dir is needed";
  ABSL_CHECK(!absl::GetFlag(FLAGS_state_dir).empty())
      << "--state_dir is needed";
  ABSL_CHECK(!absl::GetFlag(FLAGS_tls_dir).empty()) << "--tls_dir is needed";
  const std::filesystem::path data_dir = absl::GetFlag(FLAGS_data_dir);
  const std::filesystem::path state_dir = absl::GetFlag(FLAGS_state_dir);
  std::filesystem::create_directories(state_dir / "calibration");
  std::filesystem::create_directories(data_dir / "calibration");

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::SimulatedEventLoopFactory factory(&config.message());
  factory.SetRealtimeReplayRate(1.0);
  const aos::Node *node =
      aos::configuration::GetNode(&config.message(), absl::GetFlag(FLAGS_node));

  std::unique_ptr<aos::EventLoop> constants_loop =
      factory.MakeEventLoop("vision_constants_sender", node);
  frc::constants::NameConstantSender<CameraConstants, CameraConstantsList>
      constants(constants_loop.get(), absl::GetFlag(FLAGS_constants),
                absl::GetFlag(FLAGS_team_number),
                absl::GetFlag(FLAGS_robot_name), "/constants");

  std::vector<int> cameras;
  for (std::string_view camera :
       absl::StrSplit(absl::GetFlag(FLAGS_cameras), ',', absl::SkipEmpty())) {
    cameras.push_back(std::stoi(std::string(camera)));
  }
  std::unique_ptr<aos::EventLoop> camera_loop =
      factory.MakeEventLoop("fake_cameras", node);
  FakeCameras fake_cameras(camera_loop.get(), cameras,
                           std::chrono::milliseconds(static_cast<int>(
                               absl::GetFlag(FLAGS_pose_seconds) * 1000)));

  std::unique_ptr<aos::EventLoop> exposure_loop =
      factory.MakeEventLoop("field_side_exposure_adjuster", node);
  ExposureAdjuster exposure(
      exposure_loop.get(),
      {.settings_file = (state_dir / "camera_settings.json").string(),
       .red_exposure = 30,
       .blue_exposure = 30});

  std::unique_ptr<aos::EventLoop> starter_loop =
      factory.MakeEventLoop("starter", node);
  CalibrationServer::Options calibration_options;
  calibration_options.node_name = absl::GetFlag(FLAGS_node);
  calibration_options.team_number = absl::GetFlag(FLAGS_team_number);
  calibration_options.calibration_folder = (state_dir / "calibration").string();
  calibration_options.www_folder = (data_dir / "calibration").string();
  calibration_options.board_store =
      (state_dir / "calibration/boards.json").string();
  FakeStarter starter(&factory, starter_loop.get(), node, calibration_options);

  // Its WebRTC can reach a browser on this machine by loopback, as in a test's
  // sandbox, which has nothing else.
  setenv("RAWRTC_GATHER_LOOPBACK", "1", 1);
  std::unique_ptr<aos::EventLoop> web_proxy_loop =
      factory.MakeEventLoop("web_proxy", node);
  aos::web_proxy::WebProxy web_proxy(web_proxy_loop.get(),
                                     factory.scheduler_aio(),
                                     aos::web_proxy::StoreHistory::kNo, -1);
  web_proxy.SetDataPath(data_dir.c_str());

  std::unique_ptr<aos::EventLoop> server_loop =
      factory.MakeEventLoop("robot_server", node);
  const aos::FlatbufferDetachedBuffer<aos::Configuration> server_config =
      aos::CopyFlatBuffer(server_loop->configuration());
  const std::vector<std::string> args = RobotServerArgs(&config.message());
  std::vector<const char *> argv;
  for (const std::string &arg : args) {
    argv.push_back(arg.c_str());
  }
  ABSL_LOG(INFO) << "robot_server " << absl::StrJoin(args, " ");

  aos::TokioSimulation simulation(&factory);
  ABSL_CHECK(robot_server_add_to_simulation_with_flags(
      simulation.get(), aos::TokioSimulation::ForRust(server_loop.get()),
      server_config.span().data(), server_config.span().size(), argv.size(),
      argv.data()))
      << ": robot_server couldn't start";
  // Unbuffered, for whatever's waiting for it.
  std::cout << "Serving https://localhost:" << absl::GetFlag(FLAGS_https_port)
            << "/vision/" << std::endl;
  simulation.Run();
  return 0;
}

}  // namespace
}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}

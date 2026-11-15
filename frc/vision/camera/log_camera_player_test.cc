#include "frc/vision/camera/log_camera_player.h"

#include <chrono>
#include <set>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/uuid.h"

namespace frc::vision::testing {
namespace {

using std::chrono::milliseconds;

constexpr milliseconds kPeriod{16};
constexpr int kFrames = 30;

// What frame |n| of camera |camera| holds, so a played frame can be told
// apart from every other.
std::vector<uint8_t> FrameData(int camera, int n) {
  std::vector<uint8_t> data(100 + n);
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>(camera * 37 + n * 11 + i);
  }
  return data;
}

class LogCameraPlayerTest : public ::testing::Test {
 protected:
  LogCameraPlayerTest()
      : config_(aos::configuration::ReadConfig(aos::testing::ArtifactPath(
            "frc/vision/camera/log_camera_player_test_config.json"))),
        // A directory per test: they share TestTmpDir().
        log_dir_(absl::StrCat(
            aos::testing::TestTmpDir(), "/",
            ::testing::UnitTest::GetInstance()->current_test_info()->name())),
        log_base_(log_dir_ + "/log") {
    WriteLog();
  }

  // Logs kFrames frames on /camera0 and /camera1, kPeriod apart, each
  // stamped with a capture time and UUID that playing must replace.
  void WriteLog() {
    aos::SimulatedEventLoopFactory factory(&config_.message());
    std::unique_ptr<aos::EventLoop> camera_loop =
        factory.MakeEventLoop("camera_reader");
    std::vector<aos::Sender<CameraImage>> senders;
    senders.push_back(camera_loop->MakeSender<CameraImage>("/camera0"));
    senders.push_back(camera_loop->MakeSender<CameraImage>("/camera1"));
    int n = 0;
    aos::TimerHandler *timer = camera_loop->AddTimer([&]() {
      if (n == kFrames) {
        return;
      }
      for (int camera = 0; camera < 2; ++camera) {
        auto builder = senders[camera].MakeBuilder();
        const auto data = builder.fbb()->CreateVector(FrameData(camera, n));
        const auto uuid =
            builder.fbb()->CreateVector(std::vector<uint8_t>(16, 0xee));
        CameraImage::Builder image(*builder.fbb());
        image.add_rows(10 + camera);
        image.add_cols(20 + camera);
        image.add_data(data);
        image.add_format(ImageFormat::MJPEG);
        image.add_monotonic_timestamp_ns(12345);
        image.add_capture_uuid(uuid);
        builder.CheckOk(builder.Send(image.Finish()));
      }
      ++n;
    });
    camera_loop->OnRun([&]() {
      timer->Schedule(camera_loop->monotonic_now() + milliseconds(100),
                      kPeriod);
    });

    std::unique_ptr<aos::EventLoop> logger_loop =
        factory.MakeEventLoop("logger");
    aos::logger::Logger logger(logger_loop.get());
    logger.StartLoggingOnRun(log_base_);
    factory.RunFor(milliseconds(100) + kPeriod * (kFrames + 5));
  }

  std::vector<aos::logger::LogFile> LogFiles() const {
    return aos::logger::SortParts(aos::logger::FindLogs(log_dir_));
  }

  struct Received {
    int camera;
    aos::monotonic_clock::time_point time;
    std::vector<uint8_t> data;
    int32_t rows;
    int32_t cols;
    ImageFormat format;
    int64_t capture_ns;
    aos::UUID uuid;
  };

  // Plays the log's |cameras| into a fresh simulation for |duration|, and
  // returns everything sent on /camera0 to /camera2.
  std::vector<Received> Play(std::vector<int> cameras, bool loop,
                             aos::monotonic_clock::duration duration,
                             LogCameraPlayer::Statistics *statistics) {
    aos::SimulatedEventLoopFactory factory(&config_.message());
    std::unique_ptr<aos::EventLoop> player_loop =
        factory.MakeEventLoop("log_camera_player");
    LogCameraPlayer player(player_loop.get(), LogFiles(), "", cameras, loop);

    std::unique_ptr<aos::EventLoop> watch_loop = factory.MakeEventLoop("watch");
    std::vector<Received> received;
    for (int camera = 0; camera < 3; ++camera) {
      watch_loop->MakeWatcher(
          absl::StrCat("/camera", camera),
          [&, camera](const CameraImage &image) {
            received.push_back(Received{
                .camera = camera,
                .time = watch_loop->context().monotonic_event_time,
                .data = std::vector<uint8_t>(image.data()->begin(),
                                             image.data()->end()),
                .rows = image.rows(),
                .cols = image.cols(),
                .format = image.format(),
                .capture_ns = image.monotonic_timestamp_ns(),
                .uuid = aos::UUID::FromSpan({image.capture_uuid()->data(),
                                             image.capture_uuid()->size()}),
            });
          });
    }
    factory.RunFor(duration);
    *statistics = player.statistics();
    return received;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  const std::string log_dir_;
  const std::string log_base_;
};

// Every logged frame is played, in order, at the logged pace, with its bytes,
// size and format, as captured when it is sent, and with a new UUID.
TEST_F(LogCameraPlayerTest, PlaysEachFrameAsLive) {
  LogCameraPlayer::Statistics statistics;
  const std::vector<Received> received =
      Play({0, 1}, /*loop=*/false, std::chrono::seconds(2), &statistics);

  ASSERT_EQ(received.size(), 2u * kFrames);
  EXPECT_EQ(statistics.sent, std::vector<int>({kFrames, kFrames}));
  EXPECT_EQ(statistics.late, 0);
  EXPECT_EQ(statistics.refused, 0);
  EXPECT_EQ(statistics.loops, 1);

  std::set<aos::UUID> uuids;
  int next[2] = {0, 0};
  for (const Received &frame : received) {
    ASSERT_LT(frame.camera, 2);
    const int n = next[frame.camera]++;
    EXPECT_EQ(frame.data, FrameData(frame.camera, n));
    EXPECT_EQ(frame.rows, 10 + frame.camera);
    EXPECT_EQ(frame.cols, 20 + frame.camera);
    EXPECT_EQ(frame.format, ImageFormat::MJPEG);
    // Captured when sent, not when it was logged.
    EXPECT_EQ(frame.capture_ns,
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  frame.time.time_since_epoch())
                  .count());
    uuids.insert(frame.uuid);
  }
  // Every frame got a UUID of its own, none the logged one.
  EXPECT_EQ(uuids.size(), received.size());
  EXPECT_FALSE(
      uuids.contains(aos::UUID::FromSpan(std::vector<uint8_t>(16, 0xee))));

  // Each camera's frames are kPeriod apart, as logged.
  for (int camera = 0; camera < 2; ++camera) {
    std::optional<aos::monotonic_clock::time_point> last;
    for (const Received &frame : received) {
      if (frame.camera != camera) {
        continue;
      }
      if (last.has_value()) {
        EXPECT_EQ(frame.time - *last, kPeriod);
      }
      last = frame.time;
    }
  }
}

// Only the cameras asked for are played.
TEST_F(LogCameraPlayerTest, PlaysOnlyTheCamerasAskedFor) {
  LogCameraPlayer::Statistics statistics;
  const std::vector<Received> received =
      Play({1}, /*loop=*/false, std::chrono::seconds(2), &statistics);
  ASSERT_EQ(received.size(), static_cast<size_t>(kFrames));
  for (const Received &frame : received) {
    EXPECT_EQ(frame.camera, 1);
  }
}

// At the end of the log it plays again from the start, with new UUIDs.
TEST_F(LogCameraPlayerTest, Loops) {
  LogCameraPlayer::Statistics statistics;
  const std::vector<Received> received =
      Play({0}, /*loop=*/true, kPeriod * (3 * kFrames), &statistics);
  EXPECT_GE(statistics.loops, 2);
  ASSERT_GE(received.size(), 2u * kFrames);
  // The second pass starts again with frame 0.
  EXPECT_EQ(received[kFrames].data, FrameData(0, 0));
  EXPECT_EQ(received[kFrames + 1].data, FrameData(0, 1));
  EXPECT_NE(received[0].uuid, received[kFrames].uuid);
}

}  // namespace
}  // namespace frc::vision::testing

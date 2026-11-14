// Runs NvjpegDecoder on a real ShmEventLoop, on each NVJPG engine, and checks
// every published frame against turbojpeg's decode of the same JPEG.  Needs
// the Orin's NVJPG engines: copy to the target and run by hand, like
// nvmedia_jpeg_decoder_test.

#include "frc/vision/camera/nvjpeg_decoder_service.h"

#include <fstream>
#include <future>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/absl_check.h"
#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/team_number.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "frc/vision/camera/gray_image_diff.h"
#include "frc/vision/vision_generated.h"
#include "turbojpeg.h"

ABSL_DECLARE_FLAG(std::string, channel);
ABSL_DECLARE_FLAG(uint32_t, max_image_age_ms);
ABSL_DECLARE_FLAG(int32_t, nvjpg_engine);

namespace frc::vision::testing {
namespace {

std::vector<uint8_t> ReadFrame(std::string_view name) {
  const std::string path = aos::testing::ArtifactPath(
      absl::StrCat("frc/vision/camera/testdata/", name));
  std::ifstream file(path, std::ios::binary);
  ABSL_CHECK(file.good()) << ": Failed to open " << path;
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

struct Decoded {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> gray;
};

Decoded DecodeWithTurbojpeg(const std::vector<uint8_t> &jpeg) {
  tjhandle tj = tjInitDecompress();
  ABSL_CHECK(tj != nullptr);
  Decoded decoded;
  int subsampling = 0;
  int colorspace = 0;
  ABSL_CHECK_EQ(
      tjDecompressHeader3(tj, jpeg.data(), jpeg.size(), &decoded.width,
                          &decoded.height, &subsampling, &colorspace),
      0);
  decoded.gray.resize(static_cast<size_t>(decoded.width) * decoded.height);
  ABSL_CHECK_EQ(tjDecompress2(tj, jpeg.data(), jpeg.size(), decoded.gray.data(),
                              decoded.width, 0, decoded.height, TJPF_GRAY, 0),
                0);
  tjDestroy(tj);
  return decoded;
}

// Runs every test on each engine.
class NvjpegDecoderServiceTest : public ::testing::TestWithParam<int> {
 protected:
  NvjpegDecoderServiceTest()
      : flag_saver_(),
        config_(aos::configuration::ReadConfig(
            aos::testing::ArtifactPath("frc/vision/aos_config.json"))) {
    aos::testing::SetTestShmBase();
    absl::SetFlag(&FLAGS_override_hostname, "orin-9999-1");
    absl::SetFlag(&FLAGS_channel, "/camera0");
    // The test publishes as fast as the decoder keeps up; don't let the age
    // gate drop frames on a loaded bench.
    absl::SetFlag(&FLAGS_max_image_age_ms, 1000);
    absl::SetFlag(&FLAGS_nvjpg_engine, GetParam());
  }

  absl::FlagSaver flag_saver_;
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
};

// Every frame comes back on /camera0/gray, matching turbojpeg, in order.
TEST_P(NvjpegDecoderServiceTest, OutputMatchesTurbojpeg) {
  const std::vector<std::string> names = {"seq_040.jpg", "seq_080.jpg",
                                          "seq_120.jpg", "seq_200.jpg"};
  std::vector<std::vector<uint8_t>> frames;
  std::vector<Decoded> expected;
  for (const std::string &name : names) {
    frames.push_back(ReadFrame(name));
    expected.push_back(DecodeWithTurbojpeg(frames.back()));
  }

  // The decoder runs on its own event loop and thread, as it would in its own
  // process: one event loop can't both send /camera0 and watch it.
  std::promise<aos::ShmEventLoop *> decoder_ready;
  std::thread decoder_thread([&]() {
    aos::ShmEventLoop decoder_loop(&config_.message());
    NvjpegDecoder decoder(&decoder_loop);
    decoder_loop.OnRun([&]() { decoder_ready.set_value(&decoder_loop); });
    decoder_loop.Run();
  });
  aos::ShmEventLoop *decoder_loop = decoder_ready.get_future().get();

  aos::ShmEventLoop event_loop(&config_.message());

  aos::Sender<CameraImage> camera_sender =
      event_loop.MakeSender<CameraImage>("/camera0");

  // Order: forward, then backward, so a stale frame would show up as the
  // neighbouring one.
  std::vector<size_t> order;
  constexpr int kLaps = 6;
  for (int lap = 0; lap < kLaps; ++lap) {
    for (size_t i = 0; i < frames.size(); ++i) {
      order.push_back(lap % 2 == 0 ? i : frames.size() - 1 - i);
    }
  }

  size_t sent = 0;
  size_t received = 0;
  std::vector<size_t> in_flight;
  auto send_next = [&]() {
    const size_t index = order[sent];
    const std::vector<uint8_t> &jpeg = frames[index];
    auto builder = camera_sender.MakeBuilder();
    auto data = builder.fbb()->CreateVector(jpeg.data(), jpeg.size());
    CameraImage::Builder image_builder = builder.MakeBuilder<CameraImage>();
    image_builder.add_rows(expected[index].height);
    image_builder.add_cols(expected[index].width);
    image_builder.add_data(data);
    image_builder.add_format(ImageFormat::MJPEG);
    image_builder.add_monotonic_timestamp_ns(static_cast<int64_t>(sent + 1) *
                                             1000);
    builder.CheckOk(builder.Send(image_builder.Finish()));
    in_flight.push_back(index);
    ++sent;
  };

  aos::TimerHandler *pace = event_loop.AddTimer([&]() { send_next(); });

  event_loop.MakeWatcher("/camera0/gray", [&](const CameraImage &image) {
    ASSERT_LT(received, in_flight.size());
    const size_t index = in_flight[received];
    // The frame is matched to what was sent by its timestamp, so a dropped
    // frame fails here instead of shifting every later comparison.
    EXPECT_EQ(image.monotonic_timestamp_ns(),
              static_cast<int64_t>(received + 1) * 1000);
    ASSERT_EQ(image.format(), ImageFormat::MONO8);
    ASSERT_EQ(image.cols(), expected[index].width);
    ASSERT_EQ(image.rows(), expected[index].height);
    const absl::Span<const uint8_t> got(image.data()->data(),
                                        image.data()->size());
    EXPECT_LT(MeanAbsDiff(got, expected[index].gray), kSameImage)
        << ": frame " << received << " (" << names[index] << ")";
    ++received;
    if (sent < order.size()) {
      // Pace under /camera0/gray's 125 Hz limit, or AOS refuses the
      // decoder's sends as too fast.
      pace->Schedule(event_loop.monotonic_now() +
                     std::chrono::milliseconds(10));
    } else {
      event_loop.Exit();
    }
  });

  event_loop.OnRun([&]() { send_next(); });
  aos::TimerHandler *timeout = event_loop.AddTimer([&]() {
    ADD_FAILURE() << "Timed out with " << received << " of " << order.size()
                  << " frames back";
    event_loop.Exit();
  });
  event_loop.OnRun([&]() {
    timeout->Schedule(event_loop.monotonic_now() + std::chrono::seconds(20));
  });

  event_loop.Run();
  decoder_loop->Exit();
  decoder_thread.join();
  EXPECT_EQ(received, order.size());
}

INSTANTIATE_TEST_SUITE_P(Engines, NvjpegDecoderServiceTest,
                         ::testing::Values(0, 1),
                         [](const ::testing::TestParamInfo<int> &info) {
                           return "Nvjpg" + std::to_string(info.param);
                         });

}  // namespace
}  // namespace frc::vision::testing

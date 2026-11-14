// Hardware test for NvMediaJpegDecoder: decodes different JPEGs in order
// through one decoder (like the long-running service does), on each NVJPG
// engine, and checks every output against turbojpeg's decode of the same
// frame.
//
// This needs the real NVJPG engines, so it only runs on an Orin; CI hosts are
// x86 and skip it as platform-incompatible.  To run it:
//
//   bazel build --config=arm64 //frc/vision/camera:nvmedia_jpeg_decoder_test
//   cd bazel-bin/frc/vision/camera
//   rsync -aL nvmedia_jpeg_decoder_test{,.runfiles} orin:
//   ssh orin 'cd nvmedia_jpeg_decoder_test.runfiles/_main && \
//       ../../nvmedia_jpeg_decoder_test'
//
// camera_{a,b,c}.jpg are real 1280x800 4:2:2 frames from the robot's UVC
// cameras.  seq_*.jpg are small 300x200 flat gradients, whose width is not a
// whole number of the engine's 16 pixel blocks.

#include "frc/vision/camera/nvmedia_jpeg_decoder.h"

#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include "absl/log/absl_check.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"
#include "aos/testing/path.h"
#include "frc/vision/camera/gray_image_diff.h"
#include "turbojpeg.h"

namespace frc::vision::testing {
namespace {

// Output buffer size: large enough for every test frame.
constexpr size_t kMaxPixels = 1280 * 800;

std::vector<uint8_t> ReadTestJpeg(const std::string &name) {
  const std::string path =
      aos::testing::ArtifactPath("frc/vision/camera/testdata/" + name);
  std::ifstream file(path, std::ios::binary);
  ABSL_CHECK(file) << ": failed to open " << path;
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

// One test frame: the JPEG and turbojpeg's grayscale decode of it.
struct Frame {
  explicit Frame(const std::string &name_in)
      : name(name_in), jpeg(ReadTestJpeg(name_in)) {
    tjhandle handle = tjInitDecompress();
    ABSL_CHECK(handle != nullptr);
    int subsamp, colorspace;
    ABSL_CHECK_EQ(tjDecompressHeader3(handle, jpeg.data(), jpeg.size(), &width,
                                      &height, &subsamp, &colorspace),
                  0)
        << tjGetErrorStr();
    reference.resize(static_cast<size_t>(width) * height);
    ABSL_CHECK_EQ(
        tjDecompress2(handle, jpeg.data(), jpeg.size(), reference.data(), width,
                      0, height, TJPF_GRAY, 0),
        0)
        << tjGetErrorStr();
    tjDestroy(handle);
  }

  std::string name;
  std::vector<uint8_t> jpeg;
  int width = 0;
  int height = 0;
  std::vector<uint8_t> reference;
};

// Decodes |frame| and returns how far the output is from turbojpeg's, or -1
// if the decode failed.  The buffer is poisoned first, so a decode that
// writes nothing cannot pass with a previous decode's pixels.
double DecodeAndDiff(NvMediaJpegDecoder *decoder, const Frame &frame,
                     std::vector<uint8_t> *gray) {
  memset(gray->data(), 0xAA, gray->size());
  const aos::Result<NvMediaJpegDecoder::ImageSize> size = decoder->DecodeToGray(
      frame.jpeg.data(), frame.jpeg.size(), gray->data(), gray->size());
  if (!size.has_value()) {
    ADD_FAILURE() << frame.name << ": " << size.error();
    return -1;
  }
  if (static_cast<int>(size->width) != frame.width ||
      static_cast<int>(size->height) != frame.height) {
    return -1;
  }
  return MeanAbsDiff(absl::MakeConstSpan(gray->data(), frame.reference.size()),
                     frame.reference);
}

// Runs every test on each engine.
class NvMediaJpegDecoderTest : public ::testing::TestWithParam<int> {
 protected:
  NvMediaJpegDecoderTest()
      : decoder_(GetParam()),
        camera_{Frame("camera_a.jpg"), Frame("camera_b.jpg"),
                Frame("camera_c.jpg")},
        small_{Frame("seq_040.jpg"), Frame("seq_080.jpg"), Frame("seq_120.jpg"),
               Frame("seq_200.jpg")},
        gray_(kMaxPixels) {}

  void ExpectDecodesToItself(const Frame &frame) {
    const double diff = DecodeAndDiff(&decoder_, frame, &gray_);
    EXPECT_GE(diff, 0) << frame.name;
    EXPECT_LT(diff, kSameImage) << frame.name;
  }

  NvMediaJpegDecoder decoder_;
  const Frame camera_[3];
  const Frame small_[4];
  std::vector<uint8_t> gray_;
};

// The camera frames must differ from each other by far more than kSameImage,
// or the other tests could not tell a stale frame from the right one.
TEST_P(NvMediaJpegDecoderTest, TestFramesAreDistinguishable) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_GT(MeanAbsDiff(camera_[i].reference, camera_[(i + 1) % 3].reference),
              5 * kSameImage);
  }
}

// Every decode returns the frame passed in: not the previous one, not a
// stale one, not an unwritten buffer.
TEST_P(NvMediaJpegDecoderTest, EveryDecodeReturnsTheCurrentFrame) {
  for (int i = 0; i < 60; ++i) {
    ExpectDecodesToItself(camera_[i % 3]);
  }
}

TEST_P(NvMediaJpegDecoderTest, SmallFramesReturnTheCurrentFrame) {
  for (int i = 0; i < 12; ++i) {
    ExpectDecodesToItself(small_[i % 4]);
  }
}

// A change of image size mid-stream rebuilds the decoder and keeps going.
TEST_P(NvMediaJpegDecoderTest, SizeChangesDecode) {
  ExpectDecodesToItself(camera_[0]);
  ExpectDecodesToItself(small_[0]);
  ExpectDecodesToItself(camera_[1]);
  ExpectDecodesToItself(small_[1]);
  ExpectDecodesToItself(camera_[2]);
}

// A frame that lost its head off the camera (the production corrupt-frame
// signature: the buffer starts at SOF0) fails with a reason, and the next
// frame decodes.
TEST_P(NvMediaJpegDecoderTest, CorruptFrameReportsReasonAndRecovers) {
  ExpectDecodesToItself(camera_[0]);

  std::vector<uint8_t> corrupt = camera_[1].jpeg;
  for (size_t i = 2; i + 1 < corrupt.size(); ++i) {
    if (corrupt[i] == 0xFF && corrupt[i + 1] == 0xC0) {
      corrupt.erase(corrupt.begin(), corrupt.begin() + i);
      break;
    }
  }
  const aos::Result<NvMediaJpegDecoder::ImageSize> size = decoder_.DecodeToGray(
      corrupt.data(), corrupt.size(), gray_.data(), gray_.size());
  ASSERT_FALSE(size.has_value());
  EXPECT_NE(size.error().message().find("Not a JPEG"), std::string_view::npos)
      << size.error();

  ExpectDecodesToItself(camera_[2]);
  ExpectDecodesToItself(camera_[1]);
}

// An output buffer too small for the image fails instead of overrunning.
TEST_P(NvMediaJpegDecoderTest, SmallOutputBufferFails) {
  const aos::Result<NvMediaJpegDecoder::ImageSize> size =
      decoder_.DecodeToGray(camera_[0].jpeg.data(), camera_[0].jpeg.size(),
                            gray_.data(), gray_.size() - 1);
  ASSERT_FALSE(size.has_value());
  EXPECT_NE(size.error().message().find("does not fit"), std::string_view::npos)
      << size.error();
}

// Decoding is safe on a realtime thread: any malloc under ScopedRealtime
// outside the decoder's own ScopedNotRealtime (around NVIDIA's
// NvMediaIJPDRenderYUV, which allocates every frame) aborts the test.
TEST_P(NvMediaJpegDecoderTest, SteadyStateDecodeIsRealtimeSafe) {
  ExpectDecodesToItself(camera_[0]);
  {
    aos::ScopedRealtime realtime;
    for (int i = 0; i < 60; ++i) {
      const Frame &frame = camera_[i % 3];
      const aos::Result<NvMediaJpegDecoder::ImageSize> size =
          decoder_.DecodeToGray(frame.jpeg.data(), frame.jpeg.size(),
                                gray_.data(), gray_.size());
      if (!size.has_value()) {
        // Reporting the failure allocates; leave realtime first.
        aos::ScopedNotRealtime not_realtime;
        FAIL() << frame.name << ": " << size.error();
      }
    }
  }
  // And the frames were right.
  ExpectDecodesToItself(camera_[1]);
}

// Changing size sets the engine up again, which the decoder does out of
// realtime itself.
TEST_P(NvMediaJpegDecoderTest, SizeChangeIsRealtimeSafe) {
  aos::ScopedRealtime realtime;
  for (int i = 0; i < 6; ++i) {
    const Frame &frame = i % 2 == 0 ? camera_[i % 3] : small_[i % 4];
    const aos::Result<NvMediaJpegDecoder::ImageSize> size =
        decoder_.DecodeToGray(frame.jpeg.data(), frame.jpeg.size(),
                              gray_.data(), gray_.size());
    if (!size.has_value()) {
      aos::ScopedNotRealtime not_realtime;
      FAIL() << frame.name << ": " << size.error();
    }
  }
}

// Failing a frame, message included, allocates nothing.
TEST_P(NvMediaJpegDecoderTest, FailedFrameIsRealtimeSafe) {
  ExpectDecodesToItself(camera_[0]);
  std::vector<uint8_t> corrupt = camera_[1].jpeg;
  corrupt.erase(corrupt.begin(), corrupt.begin() + 2);
  bool failed = false;
  {
    aos::ScopedRealtime realtime;
    failed = !decoder_
                  .DecodeToGray(corrupt.data(), corrupt.size(), gray_.data(),
                                gray_.size())
                  .has_value();
  }
  EXPECT_TRUE(failed);
  ExpectDecodesToItself(camera_[2]);
}

INSTANTIATE_TEST_SUITE_P(Engines, NvMediaJpegDecoderTest,
                         ::testing::Values(0, 1),
                         [](const ::testing::TestParamInfo<int> &info) {
                           return "Nvjpg" + std::to_string(info.param);
                         });

// Both engines decoding at once, one decoder each, as two camera processes
// would.
TEST(NvMediaJpegDecoderBothEnginesTest, DecodeAtTheSameTime) {
  const Frame frames[3] = {Frame("camera_a.jpg"), Frame("camera_b.jpg"),
                           Frame("camera_c.jpg")};
  double worst[2] = {0, 0};
  auto run = [&frames, &worst](int engine) {
    NvMediaJpegDecoder decoder(engine);
    std::vector<uint8_t> gray(kMaxPixels);
    for (int i = 0; i < 300; ++i) {
      const double diff = DecodeAndDiff(&decoder, frames[i % 3], &gray);
      if (diff < 0 || diff > worst[engine]) {
        worst[engine] = diff < 0 ? 1e9 : diff;
      }
    }
  };
  std::thread engine0(run, 0);
  std::thread engine1(run, 1);
  engine0.join();
  engine1.join();
  EXPECT_LT(worst[0], kSameImage);
  EXPECT_LT(worst[1], kSameImage);
}

// SteadyStateDecodeDoesNotAllocate only means something if a malloc under
// ScopedRealtime really does abort in this binary.
TEST(NvMediaJpegDecoderDeathTest, MallocUnderRealtimeAborts) {
  EXPECT_DEATH(
      {
        aos::ScopedRealtime realtime;
        void *volatile p = malloc(64);
        free(p);
      },
      "");
}

// There are only two engines.
TEST(NvMediaJpegDecoderDeathTest, NoThirdEngine) {
  EXPECT_DEATH(NvMediaJpegDecoder(2), "There is no NVJPG engine 2");
}

}  // namespace
}  // namespace frc::vision::testing

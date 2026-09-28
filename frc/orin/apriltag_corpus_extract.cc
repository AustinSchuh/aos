// Pulls camera frames out of a log into a directory of standalone CameraImage
// .bfbs files that apriltag_benchmark can replay, along with the
// CameraConstants the robot was running with.  Replaying from a directory
// rather than the log means the benchmark runs on an Orin without the log
// reader or the full log.
//
// Image logs only hold the MJPEG frames the cameras send, not the decoded
// /cameraN/gray frames apriltag_detector consumes, so MJPEG frames are decoded
// to MONO8 here the same way turbojpeg_decoder does it on the robot.

#include <filesystem>
#include <map>
#include <regex>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_replace.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/util/file.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/vision_generated.h"
#include "turbojpeg.h"

ABSL_FLAG(std::string, node, "orin", "Node to extract images from.");
ABSL_FLAG(std::string, output_dir, "", "Directory to write the frames into.");
ABSL_FLAG(std::string, channel_regex, "/camera[0-9]+",
          "Channels of type frc.vision.CameraImage to extract.  MJPEG frames "
          "are decoded to MONO8; anything else is written as is.");
ABSL_FLAG(std::string, prefix, "frame",
          "Prefix for every file written, to keep frames from several logs "
          "apart in one directory.");
ABSL_FLAG(int32_t, stride, 10, "Keep every Nth frame on each channel.");
ABSL_FLAG(int32_t, skip, 0,
          "Frames to skip on each channel before keeping any, e.g. to get "
          "past the robot sitting still before a match.");
ABSL_FLAG(int32_t, max_frames_per_channel, 100,
          "Stop keeping frames on a channel after this many.");

namespace frc::orin {

class CorpusExtractor {
 public:
  explicit CorpusExtractor(aos::EventLoop *event_loop)
      : event_loop_(event_loop), handle_(tjInitDecompress()) {
    ABSL_CHECK(handle_ != nullptr) << ": " << tjGetErrorStr();
    const std::regex channel_regex(absl::GetFlag(FLAGS_channel_regex));
    for (const aos::Channel *channel :
         *event_loop_->configuration()->channels()) {
      if (channel->type()->string_view() !=
              vision::CameraImage::GetFullyQualifiedName() ||
          !std::regex_match(channel->name()->str(), channel_regex) ||
          !aos::configuration::ChannelIsReadableOnNode(channel,
                                                       event_loop_->node())) {
        continue;
      }
      const std::string name = channel->name()->str();
      ABSL_LOG(INFO) << "Extracting " << name;
      event_loop_->MakeWatcher(name,
                               [this, name](const vision::CameraImage &image) {
                                 HandleImage(name, image);
                               });
    }

    const aos::Channel *constants_channel = aos::configuration::GetChannel(
        event_loop_->configuration(), "/constants",
        vision::CameraConstants::GetFullyQualifiedName(), "",
        event_loop_->node());
    if (constants_channel != nullptr) {
      event_loop_->MakeRawWatcher(
          constants_channel, [this](const aos::Context &context, const void *) {
            if (wrote_constants_) {
              return;
            }
            wrote_constants_ = true;
            Write("constants.bfbs",
                  std::string_view(static_cast<const char *>(context.data),
                                   context.size));
          });
    }
  }

  ~CorpusExtractor() { tjDestroy(handle_); }

 private:
  void HandleImage(const std::string &channel,
                   const vision::CameraImage &image) {
    const int count = counts_[channel]++;
    const int skip = absl::GetFlag(FLAGS_skip);
    if (count < skip || (count - skip) % absl::GetFlag(FLAGS_stride) != 0 ||
        kept_[channel] >= absl::GetFlag(FLAGS_max_frames_per_channel)) {
      return;
    }
    // "/camera0/gray" -> "camera0_gray"
    const std::string filename = absl::StrFormat(
        "%s_%s_%06d.bfbs", absl::GetFlag(FLAGS_prefix),
        absl::StrReplaceAll(channel.substr(1), {{"/", "_"}}), count);

    flatbuffers::FlatBufferBuilder fbb;
    if (image.format() == vision::ImageFormat::MJPEG) {
      int width, height, subsamp, colorspace;
      if (tjDecompressHeader3(handle_, image.data()->data(),
                              image.data()->size(), &width, &height, &subsamp,
                              &colorspace) != 0) {
        ABSL_LOG(WARNING) << "Skipping " << filename << ": " << tjGetErrorStr();
        return;
      }
      uint8_t *data = nullptr;
      const flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
          fbb.CreateUninitializedVector(width * height, 1, &data);
      if (tjDecompress2(handle_, image.data()->data(), image.data()->size(),
                        data, width, 0 /* pitch */, height, TJPF_GRAY,
                        0) != 0) {
        ABSL_LOG(WARNING) << "Skipping " << filename << ": " << tjGetErrorStr();
        return;
      }
      vision::CameraImage::Builder builder(fbb);
      builder.add_rows(height);
      builder.add_cols(width);
      builder.add_data(data_offset);
      builder.add_monotonic_timestamp_ns(image.monotonic_timestamp_ns());
      builder.add_format(vision::ImageFormat::MONO8);
      fbb.Finish(builder.Finish());
    } else {
      fbb.Finish(aos::CopyFlatBuffer(&image, &fbb));
    }
    ++kept_[channel];
    Write(filename, std::string_view(
                        reinterpret_cast<const char *>(fbb.GetBufferPointer()),
                        fbb.GetSize()));
  }

  void Write(std::string_view filename, std::string_view data) {
    aos::util::WriteStringToFileOrDie(
        absl::StrCat(absl::GetFlag(FLAGS_output_dir), "/", filename), data);
  }

  aos::EventLoop *event_loop_;
  tjhandle handle_;
  std::map<std::string, int> counts_;
  std::map<std::string, int> kept_;
  bool wrote_constants_ = false;
};

}  // namespace frc::orin

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  ABSL_CHECK(!absl::GetFlag(FLAGS_output_dir).empty())
      << ": --output_dir is required";
  std::filesystem::create_directories(absl::GetFlag(FLAGS_output_dir));

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));
  aos::logger::LogReader reader(logfiles);
  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());
  reader.RegisterWithoutStarting(&event_loop_factory);

  const aos::Node *node = aos::configuration::GetNode(
      event_loop_factory.configuration(), absl::GetFlag(FLAGS_node));
  reader.OnStart(node, [&event_loop_factory, node]() {
    event_loop_factory.GetNodeEventLoopFactory(node)
        ->AlwaysStart<frc::orin::CorpusExtractor>("corpus_extract");
  });

  event_loop_factory.Run();
  reader.Deregister();
}

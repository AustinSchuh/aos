#include "frc/vision/camera/log_camera_player.h"

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"

#include "aos/configuration.h"
#include "aos/uuid.h"

namespace frc::vision {
namespace {

std::string CameraChannel(int camera) {
  return absl::StrCat("/camera", camera);
}

aos::logger::ReplayChannels CameraReplayChannels(
    const std::vector<int> &cameras) {
  aos::logger::ReplayChannels channels;
  for (int camera : cameras) {
    channels.emplace_back(CameraChannel(camera),
                          CameraImage::GetFullyQualifiedName());
  }
  return channels;
}

}  // namespace

LogCameraPlayer::LogCameraPlayer(aos::EventLoop *event_loop,
                                 std::vector<aos::logger::LogFile> log_files,
                                 std::string log_node, std::vector<int> cameras,
                                 bool loop, std::function<void()> on_end)
    : event_loop_(event_loop),
      log_files_(std::move(log_files)),
      log_node_(std::move(log_node)),
      cameras_(std::move(cameras)),
      loop_(loop),
      on_end_(std::move(on_end)),
      replay_channels_(CameraReplayChannels(cameras_)),
      timer_(event_loop_->AddTimer([this]() { Play(); })) {
  ABSL_CHECK(!cameras_.empty()) << ": No cameras to play";
  ABSL_CHECK(!log_files_.empty()) << ": No log to play";
  statistics_.sent.resize(cameras_.size(), 0);
  for (int camera : cameras_) {
    senders_.emplace_back(
        event_loop_->MakeSender<CameraImage>(CameraChannel(camera)));
  }
  event_loop_->OnRun([this]() {
    StartPass();
    Play();
  });
}

LogCameraPlayer::~LogCameraPlayer() {
  // The simulation's event loop has to go before the reader that made it.
  log_event_loop_.reset();
  reader_.reset();
}

void LogCameraPlayer::StartPass() {
  log_event_loop_.reset();
  reader_ = std::make_unique<aos::logger::LogReader>(log_files_, nullptr,
                                                     &replay_channels_);
  reader_->Register();
  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader_->configuration())) {
    node = aos::configuration::GetNode(reader_->configuration(), log_node_);
    ABSL_CHECK(node != nullptr) << ": The log has no node " << log_node_;
  }
  log_event_loop_ =
      reader_->event_loop_factory()->MakeEventLoop("log_camera_player", node);
  log_done_ = false;
  log_first_.reset();
  reader_->OnEnd(node, [this]() { log_done_ = true; });

  for (size_t i = 0; i < cameras_.size(); ++i) {
    log_event_loop_->MakeWatcher(
        CameraChannel(cameras_[i]), [this, i](const CameraImage &image) {
          const aos::monotonic_clock::time_point sent =
              log_event_loop_->context().monotonic_event_time;
          if (!log_first_.has_value()) {
            log_first_ = sent;
          }
          Frame frame{
              .camera = i,
              .offset = sent - *log_first_,
              .rows = image.rows(),
              .cols = image.cols(),
              .format = image.format(),
              .data = {},
          };
          if (image.has_data()) {
            frame.data.assign(image.data()->begin(), image.data()->end());
          }
          frames_.push_back(std::move(frame));
        });
  }
  pass_start_ = event_loop_->monotonic_now();
}

void LogCameraPlayer::Fill() {
  const aos::monotonic_clock::duration playing =
      event_loop_->monotonic_now() - pass_start_;
  while (!log_done_ &&
         (frames_.empty() || frames_.back().offset - playing < kLookahead)) {
    reader_->event_loop_factory()->RunFor(std::chrono::milliseconds(20));
  }
}

void LogCameraPlayer::Play() {
  while (true) {
    Fill();
    const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
    while (!frames_.empty() && Due(frames_.front()) <= now) {
      if (now - Due(frames_.front()) > kMaxLateness) {
        ++statistics_.late;
      } else {
        Send(frames_.front());
      }
      frames_.pop_front();
    }
    if (!frames_.empty()) {
      timer_->Schedule(Due(frames_.front()));
      return;
    }
    if (!log_done_) {
      // Everything read so far was due; read on.
      continue;
    }
    ABSL_CHECK(log_first_.has_value())
        << ": The log has no images for the cameras asked for";
    ++statistics_.loops;
    if (!loop_) {
      if (on_end_) {
        on_end_();
      }
      return;
    }
    ABSL_LOG(INFO) << "Played the log to its end; starting again";
    StartPass();
  }
}

void LogCameraPlayer::Send(const Frame &frame) {
  aos::Sender<CameraImage>::Builder builder =
      senders_[frame.camera].MakeBuilder();
  const auto data = builder.fbb()->CreateVector(frame.data);
  const aos::UUID capture_uuid = aos::UUID::Random();
  const auto capture_uuid_offset = builder.fbb()->CreateVector(
      capture_uuid.span().data(), capture_uuid.span().size());
  CameraImage::Builder image(*builder.fbb());
  image.add_rows(frame.rows);
  image.add_cols(frame.cols);
  image.add_data(data);
  image.add_format(frame.format);
  // Captured now, as far as everything downstream is concerned.
  image.add_monotonic_timestamp_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          event_loop_->monotonic_now().time_since_epoch())
          .count());
  image.add_capture_uuid(capture_uuid_offset);
  const aos::RawSender::Error error = builder.Send(image.Finish());
  if (error == aos::RawSender::Error::kMessagesSentTooFast) {
    ++statistics_.refused;
    return;
  }
  builder.CheckOk(error);
  ++statistics_.sent[frame.camera];
}

}  // namespace frc::vision

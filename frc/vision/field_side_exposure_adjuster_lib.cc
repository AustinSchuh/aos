#include "frc/vision/field_side_exposure_adjuster_lib.h"

#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>

#include "absl/log/absl_log.h"

#include "aos/configuration.h"
#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"

namespace frc::vision {

ExposureAdjuster::ExposureAdjuster(aos::EventLoop *event_loop, Options options)
    : event_loop_(event_loop),
      options_(std::move(options)),
      joystick_state_fetcher_(event_loop->MakeFetcher<JoystickState>("/frc")),
      constants_fetcher_(
          event_loop->MakeFetcher<CameraConstants>("/constants")) {
  const std::regex camera("/camera[0-9]+");
  for (const aos::Channel *channel : *event_loop->configuration()->channels()) {
    if (channel->type()->string_view() ==
            CameraStreamSettings::GetFullyQualifiedName() &&
        std::regex_match(channel->name()->str(), camera) &&
        aos::configuration::ChannelIsSendableOnNode(channel,
                                                    event_loop->node())) {
      senders_.push_back(event_loop->MakeSender<CameraStreamSettings>(
          channel->name()->string_view()));
    }
  }
  Load();

  event_loop->MakeWatcher(
      "/camera_settings", [this](const CameraStreamSettings &settings) {
        if (!settings.has_exposure_100us() && !settings.has_gain()) {
          // Back to the constants'.
          exposure_100us_.reset();
          gain_.reset();
        }
        if (settings.has_exposure_100us()) {
          exposure_100us_ = settings.exposure_100us();
        }
        if (settings.has_gain()) {
          gain_ = settings.gain();
        }
        Save();
        Send();
      });

  aos::TimerHandler *timer = event_loop->AddTimer([this]() { Send(); });
  event_loop->OnRun([this, timer]() {
    timer->Schedule(event_loop_->monotonic_now(), std::chrono::seconds(1));
  });
}

void ExposureAdjuster::Load() {
  if (options_.settings_file.empty() ||
      !std::filesystem::exists(options_.settings_file)) {
    return;
  }
  std::ifstream file(options_.settings_file);
  std::stringstream json;
  json << file.rdbuf();
  const aos::FlatbufferDetachedBuffer<CameraStreamSettings> settings =
      aos::JsonToFlatbuffer<CameraStreamSettings>(json.str());
  if (settings.span().empty()) {
    ABSL_LOG(ERROR) << "Ignoring " << options_.settings_file
                    << ", which isn't CameraStreamSettings JSON";
    return;
  }
  if (settings.message().has_exposure_100us()) {
    exposure_100us_ = settings.message().exposure_100us();
  }
  if (settings.message().has_gain()) {
    gain_ = settings.message().gain();
  }
  ABSL_LOG(INFO) << "Loaded " << options_.settings_file << ": "
                 << aos::FlatbufferToJson(settings);
}

void ExposureAdjuster::Save() {
  if (options_.settings_file.empty()) {
    return;
  }
  if (!exposure_100us_.has_value() && !gain_.has_value()) {
    std::error_code error;
    std::filesystem::remove(options_.settings_file, error);
    return;
  }
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  CameraStreamSettings::Builder builder(fbb);
  if (exposure_100us_.has_value()) {
    builder.add_exposure_100us(*exposure_100us_);
  }
  if (gain_.has_value()) {
    builder.add_gain(*gain_);
  }
  fbb.Finish(builder.Finish());
  const aos::FlatbufferDetachedBuffer<CameraStreamSettings> settings(
      fbb.Release());
  // Written beside it and renamed over it, so a crash never leaves half.
  const std::string temporary = options_.settings_file + ".tmp";
  {
    std::ofstream file(temporary, std::ios::trunc);
    file << aos::FlatbufferToJson(settings, {.multi_line = true}) << "\n";
    if (!file) {
      ABSL_LOG(ERROR) << "Failed to write " << temporary;
      return;
    }
  }
  std::error_code error;
  std::filesystem::rename(temporary, options_.settings_file, error);
  if (error) {
    ABSL_LOG(ERROR) << "Failed to save " << options_.settings_file << ": "
                    << error.message();
  }
}

const CameraStreamSettings *ExposureAdjuster::Defaults() {
  constants_fetcher_.Fetch();
  if (constants_fetcher_.get() == nullptr) {
    return nullptr;
  }
  return constants_fetcher_->default_camera_stream_settings();
}

void ExposureAdjuster::WriteWwwFile() {
  if (options_.www_file.empty()) {
    return;
  }
  std::string contents;
  if (exposure_100us_.has_value() || gain_.has_value()) {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    CameraStreamSettings::Builder builder(fbb);
    if (exposure_100us_.has_value()) {
      builder.add_exposure_100us(*exposure_100us_);
    }
    if (gain_.has_value()) {
      builder.add_gain(*gain_);
    }
    fbb.Finish(builder.Finish());
    const aos::FlatbufferDetachedBuffer<CameraStreamSettings> saved(
        fbb.Release());
    const CameraStreamSettings *defaults = Defaults();
    contents = defaults == nullptr
                   ? aos::FlatbufferToJson(saved, {.multi_line = true})
                   : aos::FlatbufferToJson(
                         aos::MergeFlatBuffers(defaults, &saved.message()),
                         {.multi_line = true});
    contents += "\n";
  }
  if (www_contents_ == contents) {
    return;
  }
  www_contents_ = contents;
  std::error_code error;
  if (contents.empty()) {
    std::filesystem::remove(options_.www_file, error);
    return;
  }
  std::filesystem::create_directories(
      std::filesystem::path(options_.www_file).parent_path(), error);
  std::ofstream file(options_.www_file, std::ios::trunc);
  file << contents;
  if (!file) {
    ABSL_LOG(ERROR) << "Failed to write " << options_.www_file;
    www_contents_.reset();
  }
}

void ExposureAdjuster::Send() {
  const CameraStreamSettings *defaults = Defaults();
  std::optional<uint32_t> exposure = exposure_100us_;
  if (!exposure.has_value() && joystick_state_fetcher_.Fetch()) {
    switch (joystick_state_fetcher_->alliance()) {
      case Alliance::kRed:
        exposure = options_.red_exposure;
        break;
      case Alliance::kBlue:
        exposure = options_.blue_exposure;
        break;
      case Alliance::kInvalid:
      default:
        break;
    }
  }
  std::optional<uint32_t> gain = gain_;
  if (defaults != nullptr) {
    // The constants' otherwise, auto exposure if they have none, as
    // camera_reader starts with.
    if (!exposure.has_value()) {
      exposure = defaults->exposure_100us();
    }
    if (!gain.has_value() && defaults->has_gain()) {
      gain = defaults->gain();
    }
  }
  WriteWwwFile();
  if (!exposure.has_value() && !gain.has_value()) {
    return;
  }
  for (aos::Sender<CameraStreamSettings> &sender : senders_) {
    aos::Sender<CameraStreamSettings>::Builder builder = sender.MakeBuilder();
    // Every field set is sent, 0 included: an exposure of 0 is auto.
    builder.fbb()->ForceDefaults(true);
    CameraStreamSettings::Builder settings =
        builder.MakeBuilder<CameraStreamSettings>();
    if (exposure.has_value()) {
      settings.add_exposure_100us(*exposure);
    }
    if (gain.has_value()) {
      settings.add_gain(*gain);
    }
    builder.CheckOk(builder.Send(settings.Finish()));
  }
}

}  // namespace frc::vision

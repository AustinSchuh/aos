#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "frc/vision/field_side_exposure_adjuster_lib.h"

namespace frc::vision::testing {
namespace {

class ExposureAdjusterTest : public ::testing::Test {
 protected:
  ExposureAdjusterTest()
      : config_(aos::configuration::ReadConfig(aos::testing::ArtifactPath(
            "frc/vision/field_side_exposure_adjuster_test_config.json"))),
        factory_(&config_.message()),
        settings_file_(aos::testing::TestTmpDir() + "/camera_settings.json"),
        test_loop_(factory_.MakeEventLoop("test")),
        page_sender_(
            test_loop_->MakeSender<CameraStreamSettings>("/camera_settings")),
        joystick_sender_(test_loop_->MakeSender<JoystickState>("/frc")),
        camera0_(test_loop_->MakeFetcher<CameraStreamSettings>("/camera0")),
        camera1_(test_loop_->MakeFetcher<CameraStreamSettings>("/camera1")) {
    std::filesystem::remove(settings_file_);
  }

  // A new adjuster, as when it's restarted.
  void Start() {
    adjuster_.reset();
    adjuster_loop_ = factory_.MakeEventLoop("field_side_exposure_adjuster");
    adjuster_ = std::make_unique<ExposureAdjuster>(
        adjuster_loop_.get(),
        ExposureAdjuster::Options{.settings_file = settings_file_,
                                  .red_exposure = 30,
                                  .blue_exposure = 40});
  }

  void SetFromPage(std::optional<uint32_t> exposure,
                   std::optional<uint32_t> gain) {
    auto builder = page_sender_.MakeBuilder();
    builder.fbb()->ForceDefaults(true);
    CameraStreamSettings::Builder settings =
        builder.MakeBuilder<CameraStreamSettings>();
    if (exposure) settings.add_exposure_100us(*exposure);
    if (gain) settings.add_gain(*gain);
    builder.CheckOk(builder.Send(settings.Finish()));
  }

  void SetAlliance(Alliance alliance) {
    auto builder = joystick_sender_.MakeBuilder();
    JoystickState::Builder state = builder.MakeBuilder<JoystickState>();
    state.add_alliance(alliance);
    builder.CheckOk(builder.Send(state.Finish()));
  }

  void RunFor(std::chrono::milliseconds duration) { factory_.RunFor(duration); }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory factory_;
  const std::string settings_file_;
  std::unique_ptr<aos::EventLoop> test_loop_;
  aos::Sender<CameraStreamSettings> page_sender_;
  aos::Sender<JoystickState> joystick_sender_;
  aos::Fetcher<CameraStreamSettings> camera0_;
  aos::Fetcher<CameraStreamSettings> camera1_;
  std::unique_ptr<aos::EventLoop> adjuster_loop_;
  std::unique_ptr<ExposureAdjuster> adjuster_;
};

// With nothing saved and no alliance, the cameras keep the constants'.
TEST_F(ExposureAdjusterTest, SendsNothingUntilTold) {
  Start();
  RunFor(std::chrono::milliseconds(2500));
  EXPECT_FALSE(camera0_.Fetch());
}

// Until one is saved, the exposure is the alliance's, as it always was.
TEST_F(ExposureAdjusterTest, AllianceExposure) {
  Start();
  SetAlliance(Alliance::kRed);
  RunFor(std::chrono::milliseconds(1500));
  ASSERT_TRUE(camera0_.Fetch());
  EXPECT_EQ(camera0_->exposure_100us(), 30u);
  EXPECT_FALSE(camera0_->has_gain());
}

// What the page sets goes to every camera at once, is saved, and holds after
// a restart, over the alliance's.
TEST_F(ExposureAdjusterTest, SavesWhatThePageSets) {
  Start();
  RunFor(std::chrono::milliseconds(100));
  SetFromPage(55, 3);
  RunFor(std::chrono::milliseconds(10));
  for (auto *camera : {&camera0_, &camera1_}) {
    ASSERT_TRUE(camera->Fetch());
    EXPECT_EQ((*camera)->exposure_100us(), 55u);
    EXPECT_EQ((*camera)->gain(), 3u);
  }
  ASSERT_TRUE(std::filesystem::exists(settings_file_));

  // Gain alone keeps the exposure.
  SetFromPage(std::nullopt, 7);
  RunFor(std::chrono::milliseconds(10));
  ASSERT_TRUE(camera1_.Fetch());
  EXPECT_EQ(camera1_->exposure_100us(), 55u);
  EXPECT_EQ(camera1_->gain(), 7u);

  Start();
  EXPECT_EQ(adjuster_->exposure_100us(), 55u);
  EXPECT_EQ(adjuster_->gain(), 7u);
  SetAlliance(Alliance::kBlue);
  RunFor(std::chrono::milliseconds(1500));
  ASSERT_TRUE(camera0_.Fetch());
  EXPECT_EQ(camera0_->exposure_100us(), 55u);
}

// Auto exposure, 0, is sent and saved as a 0, not left out.
TEST_F(ExposureAdjusterTest, SavesAuto) {
  Start();
  RunFor(std::chrono::milliseconds(100));
  SetFromPage(0, std::nullopt);
  RunFor(std::chrono::milliseconds(10));
  ASSERT_TRUE(camera0_.Fetch());
  EXPECT_TRUE(camera0_->has_exposure_100us());
  EXPECT_EQ(camera0_->exposure_100us(), 0u);
  EXPECT_FALSE(camera0_->has_gain());

  Start();
  EXPECT_EQ(adjuster_->exposure_100us(), 0u);
  EXPECT_EQ(adjuster_->gain(), std::nullopt);
}

}  // namespace
}  // namespace frc::vision::testing

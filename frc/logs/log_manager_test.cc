#include "frc/logs/log_manager.h"

#include <filesystem>
#include <fstream>

#include "flatbuffers/flatbuffers.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/logging/log_message_generated.h"
#include "aos/testing/tmpdir.h"
#include "frc/control_loops/can_device_error_generated.h"
#include "frc/control_loops/can_talonfx_generated.h"
#include "frc/input/joystick_state_generated.h"
#include "frc/input/robot_state_generated.h"
#include "frc/logs/test_position_generated.h"
#include "frc/logs/test_status_generated.h"

namespace frc::logs::testing {

namespace fs = std::filesystem;
using std::chrono::milliseconds;
using std::chrono::seconds;

struct JoystickArgs {
  bool enabled = false;
  bool autonomous = false;
  frc::MatchType match_type = frc::MatchType::kNone;
  int match_number = 0;
  std::string event_name;
  frc::Alliance alliance = frc::Alliance::kInvalid;
};

JoystickArgs Enabled() {
  JoystickArgs args;
  args.enabled = true;
  return args;
}

JoystickArgs OnField(frc::MatchType match_type, int match_number,
                     std::string event_name = "",
                     frc::Alliance alliance = frc::Alliance::kInvalid) {
  JoystickArgs args;
  args.match_type = match_type;
  args.match_number = match_number;
  args.event_name = std::move(event_name);
  args.alliance = alliance;
  return args;
}

flatbuffers::DetachedBuffer MakeJoystickState(const JoystickArgs &args) {
  flatbuffers::FlatBufferBuilder fbb;
  const auto event_name = fbb.CreateString(args.event_name);
  frc::JoystickState::Builder builder(fbb);
  builder.add_enabled(args.enabled);
  builder.add_autonomous(args.autonomous);
  builder.add_match_type(args.match_type);
  builder.add_match_number(args.match_number);
  builder.add_event_name(event_name);
  builder.add_alliance(args.alliance);
  builder.add_fms_attached(args.match_type != frc::MatchType::kNone);
  fbb.Finish(builder.Finish());
  return fbb.Release();
}

LogTime AtSeconds(double t) {
  const auto d = std::chrono::duration_cast<aos::monotonic_clock::duration>(
      std::chrono::duration<double>(t));
  return LogTime{.monotonic = aos::monotonic_clock::epoch() + d,
                 .realtime = aos::realtime_clock::epoch() + d};
}

void AddState(MatchTracker *tracker, double t, const JoystickArgs &args) {
  const flatbuffers::DetachedBuffer buffer = MakeJoystickState(args);
  tracker->Add(AtSeconds(t),
               *flatbuffers::GetRoot<frc::JoystickState>(buffer.data()));
}

TEST(MatchTrackerTest, FindsMatchAndPhases) {
  MatchTracker tracker;
  const JoystickArgs qm12 = OnField(frc::MatchType::kQualification, 12,
                                    "2026 CA-BL", frc::Alliance::kRed);
  // Pit time with no field.
  AddState(&tracker, 1.0, {});
  AddState(&tracker, 2.0, Enabled());
  AddState(&tracker, 3.0, {});
  // On the field, disabled.
  AddState(&tracker, 10.0, qm12);
  // Auto.
  JoystickArgs auto_state = qm12;
  auto_state.enabled = true;
  auto_state.autonomous = true;
  AddState(&tracker, 11.0, auto_state);
  AddState(&tracker, 26.0, auto_state);
  // Teleop.
  JoystickArgs teleop_state = qm12;
  teleop_state.enabled = true;
  AddState(&tracker, 29.0, teleop_state);
  AddState(&tracker, 160.0, teleop_state);
  // Disabled after the match, still on the field.
  AddState(&tracker, 161.0, qm12);
  tracker.Finish();

  ASSERT_EQ(tracker.matches().size(), 1u);
  const MatchTracker::Match &match = tracker.matches()[0];
  EXPECT_EQ(match.Label(), "2026cabl_qm12_red");
  EXPECT_TRUE(match.fms_attached);
  EXPECT_EQ(match.start.monotonic, AtSeconds(10.0).monotonic);
  EXPECT_EQ(match.end.monotonic, AtSeconds(161.0).monotonic);
  ASSERT_TRUE(match.auto_start && match.auto_end);
  EXPECT_EQ(match.auto_start->monotonic, AtSeconds(11.0).monotonic);
  EXPECT_EQ(match.auto_end->monotonic, AtSeconds(26.0).monotonic);
  ASSERT_TRUE(match.teleop_start && match.teleop_end);
  EXPECT_EQ(match.teleop_start->monotonic, AtSeconds(29.0).monotonic);
  EXPECT_EQ(match.teleop_end->monotonic, AtSeconds(160.0).monotonic);

  // Pit enable, auto, teleop.
  ASSERT_EQ(tracker.enabled_periods().size(), 3u);
  EXPECT_FALSE(tracker.enabled_periods()[0].autonomous);
  EXPECT_TRUE(tracker.enabled_periods()[1].autonomous);
  EXPECT_FALSE(tracker.enabled_periods()[2].autonomous);
}

TEST(MatchTrackerTest, SplitsBackToBackMatches) {
  MatchTracker tracker;
  AddState(&tracker, 1.0, OnField(frc::MatchType::kPractice, 1));
  AddState(&tracker, 2.0, OnField(frc::MatchType::kPractice, 2));
  AddState(&tracker, 3.0, {});
  tracker.Finish();
  ASSERT_EQ(tracker.matches().size(), 2u);
  EXPECT_EQ(tracker.matches()[0].Label(), "pm1");
  EXPECT_EQ(tracker.matches()[1].Label(), "pm2");
}

TEST(LogTimeTest, FoxgloveTimeUsesMonotonicClock) {
  const nlohmann::json json = ToJson(AtSeconds(123.5));
  EXPECT_EQ(json["foxglove_time"], "1970-01-01T00:02:03.5Z");
  EXPECT_DOUBLE_EQ(json["monotonic_sec"].get<double>(), 123.5);
}

TEST(ChannelSelectionTest, EscapesAndDeduplicates) {
  EXPECT_EQ(ChannelRegex("/superstructure", "y2026.controls.Output"),
            "/superstructure y2026\\.controls\\.Output");
  const std::vector<std::string> regexes =
      IncludeRegexes({"/frc frc\\.RobotState", "/drivetrain .*"});
  EXPECT_EQ(regexes.size(), AlwaysIncludedRegexes().size() + 1);
  EXPECT_EQ(regexes.back(), "/drivetrain .*");
  EXPECT_NE(FindPreset("all"), nullptr);
  EXPECT_EQ(FindPreset("nope"), nullptr);
}

TEST(ListLogsTest, FindsLogsAndCurrent) {
  const fs::path dir = fs::path(aos::testing::TestTmpDir()) / "list_logs";
  fs::remove_all(dir);
  fs::create_directories(dir / "fbs_log-001_2026-11-10_10-00-00");
  fs::create_directories(dir / "fbs_log-002_2026-11-10_11-00-00");
  fs::create_directories(dir / "not_a_log");
  {
    std::ofstream(dir / "fbs_log-001_2026-11-10_10-00-00" / "a.bfbs")
        << "12345";
  }
  fs::create_directory_symlink("fbs_log-002_2026-11-10_11-00-00",
                               dir / "fbs_log-current");

  const std::vector<LogDirInfo> logs = ListLogs(dir.string());
  ASSERT_EQ(logs.size(), 2u);
  EXPECT_EQ(logs[0].index, 2);
  EXPECT_TRUE(logs[0].current);
  EXPECT_EQ(logs[1].index, 1);
  EXPECT_FALSE(logs[1].current);
  EXPECT_EQ(logs[1].size_bytes, 5u);
  EXPECT_FALSE(IsLogName("fbs_log-current"));
}

// Writes a real log with the robot's config, then analyzes and converts it.
class LogManagerLogTest : public ::testing::Test {
 protected:
  LogManagerLogTest()
      : config_(aos::configuration::ReadConfig("frc/logs/test_config.bfbs")),
        factory_(&config_.message()),
        node_(nullptr),
        log_dir_(fs::path(aos::testing::TestTmpDir()) / "log_manager_log") {
    fs::remove_all(log_dir_);
  }

  void WriteLog() {
    std::unique_ptr<aos::EventLoop> sender_loop =
        factory_.MakeEventLoop("sender", node_);
    aos::Sender<frc::JoystickState> joystick_sender =
        sender_loop->MakeSender<frc::JoystickState>("/frc");
    aos::Sender<frc::RobotState> robot_state_sender =
        sender_loop->MakeSender<frc::RobotState>("/frc");
    aos::Sender<aos::logging::LogMessageFbs> log_sender =
        sender_loop->MakeSender<aos::logging::LogMessageFbs>("/aos");
    aos::Sender<Position> position_sender =
        sender_loop->MakeSender<Position>("/test");
    aos::Sender<Status> status_sender =
        sender_loop->MakeSender<Status>("/test");

    int tick = 0;
    sender_loop->AddPhasedLoop(
        [&](int) {
          ++tick;
          // 0-2 s pit, 2-4 s auto in qm7, 4-6 s teleop, then disabled.
          {
            auto builder = joystick_sender.MakeBuilder();
            const auto event_name = builder.fbb()->CreateString("test");
            frc::JoystickState::Builder state(*builder.fbb());
            const bool on_field = tick > 20;
            state.add_match_type(on_field ? frc::MatchType::kQualification
                                          : frc::MatchType::kNone);
            state.add_match_number(on_field ? 7 : 0);
            state.add_event_name(event_name);
            state.add_alliance(frc::Alliance::kBlue);
            state.add_enabled(tick > 20 && tick <= 60);
            state.add_autonomous(tick > 20 && tick <= 40);
            builder.CheckOk(builder.Send(state.Finish()));
          }
          {
            auto builder = robot_state_sender.MakeBuilder();
            frc::RobotState::Builder state(*builder.fbb());
            state.add_voltage_battery(tick == 30 ? 7.5 : 12.4);
            state.add_browned_out(tick == 30);
            builder.CheckOk(builder.Send(state.Finish()));
          }
          {
            auto builder = position_sender.MakeBuilder();
            std::vector<frc::control_loops::CANDeviceError> errors;
            if (tick >= 50) {
              errors.push_back(frc::control_loops::CANDeviceError::READ_FAILED);
            }
            const auto errors_offset = builder.fbb()->CreateVector(errors);
            frc::control_loops::CANTalonFX::Builder talon(*builder.fbb());
            talon.add_id(8);
            talon.add_errors(errors_offset);
            const auto talon_offset = talon.Finish();
            Position::Builder position(*builder.fbb());
            position.add_motor(talon_offset);
            builder.CheckOk(builder.Send(position.Finish()));
          }
          {
            auto builder = status_sender.MakeBuilder();
            Status::Builder status(*builder.fbb());
            status.add_estopped(tick >= 70);
            builder.CheckOk(builder.Send(status.Finish()));
          }
          if (tick == 45) {
            auto builder = log_sender.MakeBuilder();
            const auto message = builder.fbb()->CreateString("CAN went away");
            const auto name = builder.fbb()->CreateString("superstructure");
            aos::logging::LogMessageFbs::Builder log(*builder.fbb());
            log.add_message(message);
            log.add_name(name);
            log.add_level(aos::logging::Level::kERROR);
            builder.CheckOk(builder.Send(log.Finish()));
          }
        },
        milliseconds(100));

    std::unique_ptr<aos::EventLoop> logger_loop =
        factory_.MakeEventLoop("logger", node_);
    {
      aos::logger::Logger logger(logger_loop.get());
      logger.StartLoggingOnRun(log_dir_.string() + "/");
      factory_.RunFor(seconds(8));
    }
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::SimulatedEventLoopFactory factory_;
  const aos::Node *node_;
  fs::path log_dir_;
};

TEST_F(LogManagerLogTest, AnalyzeFindsMatchFaultsAndErrors) {
  WriteLog();
  const nlohmann::json summary = AnalyzeLog({log_dir_.string()}, "");

  EXPECT_EQ(summary["node"], "");

  ASSERT_EQ(summary["matches"].size(), 1u);
  const nlohmann::json &match = summary["matches"][0];
  EXPECT_EQ(match["label"], "test_qm7_blue");
  EXPECT_FALSE(match["auto_start"].is_null());
  EXPECT_FALSE(match["teleop_start"].is_null());
  EXPECT_LT(match["auto_start"]["monotonic_sec"].get<double>(),
            match["teleop_start"]["monotonic_sec"].get<double>());

  EXPECT_EQ(summary["enabled_periods"].size(), 2u);

  ASSERT_FALSE(summary["min_battery_voltage"].is_null());
  EXPECT_DOUBLE_EQ(summary["min_battery_voltage"]["volts"].get<double>(), 7.5);

  bool saw_brownout = false;
  bool saw_read_failed = false;
  bool saw_estop = false;
  for (const nlohmann::json &fault : summary["faults"]) {
    if (fault["what"] == "browned_out") {
      saw_brownout = true;
      EXPECT_EQ(fault["count"], 1);
    }
    if (fault["what"] == "READ_FAILED") {
      saw_read_failed = true;
      EXPECT_EQ(fault["path"], "motor.errors");
      EXPECT_EQ(fault["can_id"], 8);
      EXPECT_EQ(fault["channel"], "/test frc.logs.testing.Position");
    }
    if (fault["what"] == "estopped") {
      saw_estop = true;
      EXPECT_EQ(fault["channel"], "/test frc.logs.testing.Status");
    }
  }
  EXPECT_TRUE(saw_estop);
  EXPECT_TRUE(saw_brownout);
  EXPECT_TRUE(saw_read_failed);

  ASSERT_EQ(summary["log_messages"].size(), 1u);
  EXPECT_EQ(summary["log_messages"][0]["level"], "ERROR");
  EXPECT_EQ(summary["log_messages"][0]["message"], "CAN went away");

  bool saw_position = false;
  for (const nlohmann::json &channel : summary["channels"]) {
    if (channel["type"] == "frc.logs.testing.Position") {
      saw_position = true;
      EXPECT_GT(channel["count"].get<int>(), 50);
    }
  }
  EXPECT_TRUE(saw_position);
}

TEST_F(LogManagerLogTest, ConvertWritesMcap) {
  WriteLog();
  const fs::path output = log_dir_ / "out.mcap";
  const ChannelPreset *preset = FindPreset("power");
  ASSERT_NE(preset, nullptr);
  EXPECT_EQ(ConvertToMcap({log_dir_.string()}, output.string(),
                          IncludeRegexes(preset->regexes), ""),
            0);
  ASSERT_TRUE(fs::exists(output));
  EXPECT_GT(fs::file_size(output), 0u);
  EXPECT_FALSE(fs::exists(output.string() + ".tmp"));

  // MCAP files start with the magic bytes 0x89 "MCAP0\r\n".
  std::ifstream file(output, std::ios::binary);
  char magic[8];
  file.read(magic, sizeof(magic));
  EXPECT_EQ(std::string(magic + 1, 4), "MCAP");
}

TEST(PresetsTest, LoadsARobotsPresets) {
  const fs::path path = fs::path(aos::testing::TestTmpDir()) / "presets.json";
  {
    std::ofstream(path) << R"({"presets": [{"name": "superstructure",
        "description": "Everything on /superstructure.",
        "regexes": ["/superstructure .*"]}]})";
  }
  std::string error;
  ASSERT_TRUE(LoadPresets(path.string(), &error)) << error;
  ASSERT_NE(FindPreset("superstructure"), nullptr);
  EXPECT_EQ(FindPreset("superstructure")->regexes,
            std::vector<std::string>{"/superstructure .*"});
  // "all" is always there, for downloads.
  EXPECT_NE(FindPreset("all"), nullptr);
  EXPECT_EQ(FindPreset("power"), nullptr);

  {
    std::ofstream(path) << "not json";
  }
  EXPECT_FALSE(LoadPresets(path.string(), &error));
  EXPECT_NE(error.find("presets"), std::string::npos) << error;
}

}  // namespace frc::logs::testing

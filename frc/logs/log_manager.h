#ifndef FRC_LOGS_LOG_MANAGER_H_
#define FRC_LOGS_LOG_MANAGER_H_

// Library behind the robot log tooling.  See frc/logs/README.md.
//
// Everything that reads an AOS log here can ABSL_CHECK-fail on a corrupt log,
// so servers run these through the log_manager binary in a child process
// rather than calling them in-process.

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "nlohmann/json.hpp"

#include "aos/configuration_generated.h"
#include "aos/time/time.h"
#include "frc/input/joystick_state_generated.h"

namespace frc::logs {

// ---------------------------------------------------------------------------
// Log folders on disk.

struct LogDirInfo {
  // Folder name, e.g. "fbs_log-012_2026-11-10_18-04-33".
  std::string name;
  std::string path;
  // The NNN in fbs_log-NNN_..., used for ordering.
  int index = -1;
  uint64_t size_bytes = 0;
  // True if fbs_log-current points at this log, i.e. it is still being
  // written.
  bool current = false;
};

// Returns true if `name` looks like an AOS log folder name.
bool IsLogName(std::string_view name);

// Lists the AOS log folders directly inside `dir`, newest (highest index)
// first.  Does not read the logs themselves.
std::vector<LogDirInfo> ListLogs(const std::string &dir);

nlohmann::json ToJson(const LogDirInfo &info);

// ---------------------------------------------------------------------------
// Channel selection for MCAP conversion.

struct ChannelPreset {
  std::string name;
  std::string description;
  // Regexes matched (std::regex_match) against "<channel name> <type>".
  std::vector<std::string> regexes;
};

// Presets offered by the UI.  "all" is what downloads are converted with.  A
// robot's code can replace the generic defaults with its own (LoadPresets).
const std::vector<ChannelPreset> &ChannelPresets();
// Replaces the presets with those in a JSON file:
//   {"presets": [{"name": ..., "description": ..., "regexes": [...]}, ...]}
// Returns false and sets *error if the file can't be read or parsed.
bool LoadPresets(const std::string &path, std::string *error);
const ChannelPreset *FindPreset(std::string_view name);

// Channels always added to any selection so the Foxglove layout's power,
// match, and error panels have data.
const std::vector<std::string> &AlwaysIncludedRegexes();

// Escapes a literal channel name or type for use in a regex.
std::string EscapeRegex(std::string_view s);

// Regex matching exactly one channel.
std::string ChannelRegex(std::string_view name, std::string_view type);

// Combines a selection with the always-included channels, deduplicated.
std::vector<std::string> IncludeRegexes(
    const std::vector<std::string> &selection);

nlohmann::json PresetsJson();

// Converts a log to MCAP with canonical "<name> <type>" topic names.  Only
// channels matching one of `include_regexes` are written.  Writes to a
// temporary file next to `output_path` and renames it into place on success.
// Returns 0 on success.
int ConvertToMcap(const std::vector<std::string> &log_paths,
                  const std::string &output_path,
                  const std::vector<std::string> &include_regexes,
                  const std::string &node);

// ---------------------------------------------------------------------------
// Log analysis: matches, errors, battery.

// A point in time in a log, on both clocks.
struct LogTime {
  aos::monotonic_clock::time_point monotonic = aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point realtime = aos::realtime_clock::min_time;
};

// Serializes a LogTime.  "foxglove_time" is the RFC3339 timestamp to pass as
// Foxglove's time= parameter.  log_to_mcap stamps messages with the monotonic
// clock, so that is the monotonic time treated as time since the epoch.
nlohmann::json ToJson(const LogTime &time);

std::string MatchTypeName(frc::MatchType type);
std::string AllianceName(frc::Alliance alliance);

// Turns the stream of JoystickState messages into matches and enabled
// periods.  Feed every message in order, then call Finish().
class MatchTracker {
 public:
  struct Match {
    std::string event_name;
    frc::MatchType match_type = frc::MatchType::kNone;
    int match_number = 0;
    int replay_number = 0;
    frc::Alliance alliance = frc::Alliance::kInvalid;
    bool fms_attached = false;
    // First and last JoystickState with this match's info.
    LogTime start;
    LogTime end;
    std::optional<LogTime> auto_start;
    std::optional<LogTime> auto_end;
    std::optional<LogTime> teleop_start;
    std::optional<LogTime> teleop_end;

    // A short label like "2026cabl_qm12_red".
    std::string Label() const;
  };

  struct EnabledPeriod {
    LogTime start;
    LogTime end;
    bool autonomous = false;
  };

  void Add(const LogTime &time, const frc::JoystickState &state);
  void Finish();

  const std::vector<Match> &matches() const { return matches_; }
  const std::vector<EnabledPeriod> &enabled_periods() const {
    return enabled_periods_;
  }

 private:
  std::optional<Match> current_match_;
  std::optional<EnabledPeriod> current_enabled_;
  std::vector<Match> matches_;
  std::vector<EnabledPeriod> enabled_periods_;
};

nlohmann::json ToJson(const MatchTracker::Match &match);
nlohmann::json ToJson(const MatchTracker::EnabledPeriod &period);

// Collects everything that went wrong in a log.
class FaultCollector {
 public:
  struct Fault {
    // e.g. "/superstructure offseason2026.controls.Position".
    std::string channel;
    // Path to the field inside the message, e.g. "intake_pivot.errors".
    std::string path;
    // e.g. "READ_FAILED", "estopped", "browned_out".
    std::string what;
    // CAN id when the faulted table has an "id" field.
    std::optional<int64_t> can_id;
    LogTime first;
    LogTime last;
    uint64_t count = 0;
  };

  struct LogMessage {
    LogTime time;
    std::string level;
    std::string name;
    std::string message;
  };

  explicit FaultCollector(size_t max_log_messages = 500)
      : max_log_messages_(max_log_messages) {}

  // Walks a message with reflection looking for the generic fault fields:
  //  * "errors" vectors of enums (CANDeviceError, ZeroingError, ...).
  //  * "estopped" bools that are true.
  void ScanMessage(const aos::Channel *channel, const LogTime &time,
                   const void *data);

  void AddLogMessage(const LogTime &time, int level, std::string_view name,
                     std::string_view message);
  void AddRobotState(const LogTime &time, bool browned_out,
                     double voltage_battery);

  std::vector<Fault> faults() const;
  const std::vector<LogMessage> &log_messages() const { return log_messages_; }
  uint64_t dropped_log_messages() const { return dropped_log_messages_; }
  const std::optional<std::pair<double, LogTime>> &min_battery_voltage() const {
    return min_battery_voltage_;
  }

 private:
  void AddFault(std::string channel, std::string path, std::string what,
                std::optional<int64_t> can_id, const LogTime &time);

  size_t max_log_messages_;
  // Keyed by (channel, path, what) so iteration order is stable.
  std::map<std::tuple<std::string, std::string, std::string>, Fault> faults_;
  std::vector<LogMessage> log_messages_;
  uint64_t dropped_log_messages_ = 0;
  std::optional<std::pair<double, LogTime>> min_battery_voltage_;
};

nlohmann::json ToJson(const FaultCollector::Fault &fault);
nlohmann::json ToJson(const FaultCollector::LogMessage &message);

// Replays a log once and returns its summary: time range, channels with
// message counts, matches, enabled periods, faults, error log messages, and
// the minimum battery voltage.  `node` may be empty for single-node logs.
nlohmann::json AnalyzeLog(const std::vector<std::string> &log_paths,
                          const std::string &node);

}  // namespace frc::logs

#endif  // FRC_LOGS_LOG_MANAGER_H_

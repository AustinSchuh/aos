#include "frc/logs/log_manager.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <regex>
#include <set>
#include <unordered_map>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/ascii.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "flatbuffers/reflection.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/logging/log_message_generated.h"
#include "aos/util/log_to_mcap_lib.h"
#include "frc/input/robot_state_generated.h"

ABSL_DECLARE_FLAG(std::vector<std::string>, include_channels);
ABSL_DECLARE_FLAG(std::vector<std::string>, drop_channels);
ABSL_DECLARE_FLAG(bool, canonical_channel_names);
ABSL_DECLARE_FLAG(std::string, node);

namespace frc::logs {

namespace fs = std::filesystem;

namespace {

const std::regex &LogNameRegex() {
  static const std::regex kRegex("fbs_log-(\\d+)_[A-Za-z0-9_.:-]+");
  return kRegex;
}

std::string FormatRfc3339(int64_t unix_nanos) {
  return absl::FormatTime("%Y-%m-%dT%H:%M:%E*SZ",
                          absl::FromUnixNanos(unix_nanos), absl::UTCTimeZone());
}

double MonotonicSeconds(aos::monotonic_clock::time_point t) {
  return std::chrono::duration<double>(t.time_since_epoch()).count();
}

std::string ChannelKey(const aos::Channel *channel) {
  return absl::StrCat(channel->name()->string_view(), " ",
                      channel->type()->string_view());
}

bool EndsWith(std::string_view s, std::string_view suffix) {
  return s.size() >= suffix.size() &&
         s.substr(s.size() - suffix.size()) == suffix;
}

}  // namespace

// ---------------------------------------------------------------------------
// Log folders.

bool IsLogName(std::string_view name) {
  return std::regex_match(name.begin(), name.end(), LogNameRegex());
}

std::vector<LogDirInfo> ListLogs(const std::string &dir) {
  std::vector<LogDirInfo> logs;
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) {
    return logs;
  }

  // fbs_log-current is a symlink to the log being written.
  std::string current;
  const fs::path current_link = fs::path(dir) / "fbs_log-current";
  if (fs::is_symlink(current_link, ec)) {
    current = fs::read_symlink(current_link, ec).filename().string();
    if (current.empty()) {
      // Handle links with a trailing slash.
      current = fs::read_symlink(current_link, ec).parent_path().filename();
    }
  }

  for (const fs::directory_entry &entry : fs::directory_iterator(dir, ec)) {
    const std::string name = entry.path().filename().string();
    if (entry.is_symlink(ec) || !entry.is_directory(ec)) {
      continue;
    }
    std::smatch match;
    if (!std::regex_match(name, match, LogNameRegex())) {
      continue;
    }
    LogDirInfo info;
    info.name = name;
    info.path = entry.path().string();
    info.index = std::stoi(match[1].str());
    info.current = (name == current);
    for (const fs::directory_entry &file :
         fs::recursive_directory_iterator(entry.path(), ec)) {
      if (file.is_regular_file(ec)) {
        info.size_bytes += file.file_size(ec);
      }
    }
    logs.push_back(std::move(info));
  }
  std::sort(logs.begin(), logs.end(),
            [](const LogDirInfo &a, const LogDirInfo &b) {
              if (a.index != b.index) return a.index > b.index;
              return a.name > b.name;
            });
  return logs;
}

nlohmann::json ToJson(const LogDirInfo &info) {
  return {
      {"name", info.name},       {"path", info.path},
      {"index", info.index},     {"size_bytes", info.size_bytes},
      {"current", info.current},
  };
}

// ---------------------------------------------------------------------------
// Channel selection.

namespace {

std::vector<ChannelPreset> DefaultPresets() {
  return {
      {"all", "Every logged channel.", {".*"}},
      {"power",
       "Battery, brownouts and motor currents: RobotState and every Position.",
       {"/frc frc\\.RobotState", "/.* .*\\.Position"}},
      {"errors",
       "Text log messages, starter status and every Status.",
       {"/aos aos\\.starter\\.Status", "/.* .*\\.Status"}},
  };
}

std::vector<ChannelPreset> &MutablePresets() {
  static std::vector<ChannelPreset> presets = DefaultPresets();
  return presets;
}

}  // namespace

const std::vector<ChannelPreset> &ChannelPresets() { return MutablePresets(); }

bool LoadPresets(const std::string &path, std::string *error) {
  std::ifstream file(path);
  if (!file) {
    *error = absl::StrCat("Can't read ", path);
    return false;
  }
  const nlohmann::json json = nlohmann::json::parse(file, nullptr, false);
  if (json.is_discarded() || !json.contains("presets") ||
      !json["presets"].is_array()) {
    *error = absl::StrCat(path, " isn't {\"presets\": [...]}");
    return false;
  }
  std::vector<ChannelPreset> presets;
  for (const nlohmann::json &preset : json["presets"]) {
    if (!preset.contains("name") || !preset["name"].is_string() ||
        !preset.contains("regexes") || !preset["regexes"].is_array()) {
      *error = absl::StrCat(path,
                            ": each preset needs a name and a list of regexes");
      return false;
    }
    ChannelPreset parsed;
    parsed.name = preset["name"].get<std::string>();
    parsed.description = preset.value("description", "");
    for (const nlohmann::json &regex : preset["regexes"]) {
      if (!regex.is_string()) {
        *error = absl::StrCat(path, ": regexes must be strings");
        return false;
      }
      parsed.regexes.push_back(regex.get<std::string>());
    }
    presets.push_back(std::move(parsed));
  }
  // "all" is what downloads are converted with, so there always is one.
  if (std::none_of(presets.begin(), presets.end(),
                   [](const ChannelPreset &p) { return p.name == "all"; })) {
    presets.insert(presets.begin(), DefaultPresets()[0]);
  }
  MutablePresets() = std::move(presets);
  return true;
}

const ChannelPreset *FindPreset(std::string_view name) {
  for (const ChannelPreset &preset : ChannelPresets()) {
    if (preset.name == name) {
      return &preset;
    }
  }
  return nullptr;
}

const std::vector<std::string> &AlwaysIncludedRegexes() {
  static const std::vector<std::string> kAlways = {
      "/aos aos\\.logging\\.LogMessageFbs",
      "/frc frc\\.RobotState",
      "/frc frc\\.JoystickState",
  };
  return kAlways;
}

std::string EscapeRegex(std::string_view s) {
  std::string result;
  result.reserve(s.size() * 2);
  for (const char c : s) {
    switch (c) {
      case '\\':
      case '^':
      case '$':
      case '.':
      case '*':
      case '+':
      case '?':
      case '(':
      case ')':
      case '[':
      case ']':
      case '{':
      case '}':
      case '|':
        result.push_back('\\');
        [[fallthrough]];
      default:
        result.push_back(c);
    }
  }
  return result;
}

std::string ChannelRegex(std::string_view name, std::string_view type) {
  return absl::StrCat(EscapeRegex(name), " ", EscapeRegex(type));
}

std::vector<std::string> IncludeRegexes(
    const std::vector<std::string> &selection) {
  std::vector<std::string> result;
  std::set<std::string> seen;
  for (const std::vector<std::string> *list :
       {&AlwaysIncludedRegexes(), &selection}) {
    for (const std::string &regex : *list) {
      // absl's vector flag splits on commas, so a comma would corrupt the
      // selection.
      ABSL_CHECK(regex.find(',') == std::string::npos)
          << ": Channel regexes can't contain commas: " << regex;
      if (seen.insert(regex).second) {
        result.push_back(regex);
      }
    }
  }
  return result;
}

nlohmann::json PresetsJson() {
  nlohmann::json presets = nlohmann::json::array();
  for (const ChannelPreset &preset : ChannelPresets()) {
    presets.push_back({{"name", preset.name},
                       {"description", preset.description},
                       {"regexes", preset.regexes}});
  }
  return {{"presets", presets}, {"always_included", AlwaysIncludedRegexes()}};
}

int ConvertToMcap(const std::vector<std::string> &log_paths,
                  const std::string &output_path,
                  const std::vector<std::string> &include_regexes,
                  const std::string &node) {
  absl::SetFlag(&FLAGS_include_channels, include_regexes);
  absl::SetFlag(&FLAGS_drop_channels, std::vector<std::string>{});
  absl::SetFlag(&FLAGS_canonical_channel_names, true);
  absl::SetFlag(&FLAGS_node, node);

  const std::string tmp_path = output_path + ".tmp";
  std::error_code ec;
  fs::remove(tmp_path, ec);
  const int result = aos::util::ConvertLogToMcap(log_paths, tmp_path);
  if (result != 0) {
    fs::remove(tmp_path, ec);
    return result;
  }
  fs::rename(tmp_path, output_path, ec);
  if (ec) {
    ABSL_LOG(ERROR) << "Failed to rename " << tmp_path << " to " << output_path
                    << ": " << ec.message();
    return 1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Times and enums.

nlohmann::json ToJson(const LogTime &time) {
  const int64_t monotonic_ns = time.monotonic.time_since_epoch().count();
  const int64_t realtime_ns = time.realtime.time_since_epoch().count();
  return {
      {"monotonic_sec", MonotonicSeconds(time.monotonic)},
      {"realtime", FormatRfc3339(realtime_ns)},
      {"foxglove_time", FormatRfc3339(monotonic_ns)},
  };
}

std::string MatchTypeName(frc::MatchType type) {
  switch (type) {
    case frc::MatchType::kNone:
      return "none";
    case frc::MatchType::kPractice:
      return "practice";
    case frc::MatchType::kQualification:
      return "qualification";
    case frc::MatchType::kElimination:
      return "elimination";
  }
  return "unknown";
}

std::string AllianceName(frc::Alliance alliance) {
  switch (alliance) {
    case frc::Alliance::kRed:
      return "red";
    case frc::Alliance::kBlue:
      return "blue";
    case frc::Alliance::kInvalid:
      return "unknown";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Matches.

std::string MatchTracker::Match::Label() const {
  std::string prefix;
  switch (match_type) {
    case frc::MatchType::kPractice:
      prefix = "pm";
      break;
    case frc::MatchType::kQualification:
      prefix = "qm";
      break;
    case frc::MatchType::kElimination:
      prefix = "em";
      break;
    case frc::MatchType::kNone:
      prefix = "m";
      break;
  }
  std::string event;
  for (const char c : event_name) {
    if (absl::ascii_isalnum(c)) {
      event.push_back(absl::ascii_tolower(c));
    }
  }
  std::string label = event.empty() ? "" : event + "_";
  absl::StrAppend(&label, prefix, match_number);
  if (replay_number > 1) {
    absl::StrAppend(&label, "r", replay_number);
  }
  if (alliance != frc::Alliance::kInvalid) {
    absl::StrAppend(&label, "_", AllianceName(alliance));
  }
  return label;
}

void MatchTracker::Add(const LogTime &time, const frc::JoystickState &state) {
  // Enabled periods are tracked for every log, with or without a field.
  if (state.enabled()) {
    // Auto running straight into teleop is two periods.
    if (current_enabled_ &&
        current_enabled_->autonomous != state.autonomous()) {
      enabled_periods_.push_back(*current_enabled_);
      current_enabled_.reset();
    }
    if (!current_enabled_) {
      current_enabled_ = EnabledPeriod{
          .start = time, .end = time, .autonomous = state.autonomous()};
    }
    current_enabled_->end = time;
  } else if (current_enabled_) {
    enabled_periods_.push_back(*current_enabled_);
    current_enabled_.reset();
  }

  if (state.match_type() == frc::MatchType::kNone) {
    // The field info goes away when the DS disconnects from the FMS.  Close
    // out the current match.
    if (current_match_) {
      matches_.push_back(*current_match_);
      current_match_.reset();
    }
    return;
  }

  const std::string event_name =
      state.has_event_name() ? state.event_name()->str() : "";
  const bool same_match =
      current_match_ && current_match_->match_type == state.match_type() &&
      current_match_->match_number == state.match_number() &&
      current_match_->replay_number == state.replay_number() &&
      current_match_->event_name == event_name;
  if (!same_match) {
    if (current_match_) {
      matches_.push_back(*current_match_);
    }
    Match match;
    match.event_name = event_name;
    match.match_type = state.match_type();
    match.match_number = state.match_number();
    match.replay_number = state.replay_number();
    match.start = time;
    match.end = time;
    current_match_ = std::move(match);
  }

  Match &match = *current_match_;
  match.end = time;
  if (state.alliance() != frc::Alliance::kInvalid) {
    match.alliance = state.alliance();
  }
  match.fms_attached |= state.fms_attached();
  if (state.enabled() && !state.test_mode()) {
    if (state.autonomous()) {
      if (!match.auto_start) match.auto_start = time;
      match.auto_end = time;
    } else {
      if (!match.teleop_start) match.teleop_start = time;
      match.teleop_end = time;
    }
  }
}

void MatchTracker::Finish() {
  if (current_match_) {
    matches_.push_back(*current_match_);
    current_match_.reset();
  }
  if (current_enabled_) {
    enabled_periods_.push_back(*current_enabled_);
    current_enabled_.reset();
  }
}

namespace {
nlohmann::json OptionalTime(const std::optional<LogTime> &time) {
  return time ? ToJson(*time) : nlohmann::json(nullptr);
}
}  // namespace

nlohmann::json ToJson(const MatchTracker::Match &match) {
  return {
      {"label", match.Label()},
      {"event_name", match.event_name},
      {"match_type", MatchTypeName(match.match_type)},
      {"match_number", match.match_number},
      {"replay_number", match.replay_number},
      {"alliance", AllianceName(match.alliance)},
      {"fms_attached", match.fms_attached},
      {"start", ToJson(match.start)},
      {"end", ToJson(match.end)},
      {"auto_start", OptionalTime(match.auto_start)},
      {"auto_end", OptionalTime(match.auto_end)},
      {"teleop_start", OptionalTime(match.teleop_start)},
      {"teleop_end", OptionalTime(match.teleop_end)},
  };
}

nlohmann::json ToJson(const MatchTracker::EnabledPeriod &period) {
  return {
      {"start", ToJson(period.start)},
      {"end", ToJson(period.end)},
      {"autonomous", period.autonomous},
  };
}

// ---------------------------------------------------------------------------
// Faults.

namespace {

bool IsIntegerType(reflection::BaseType type) {
  return type >= reflection::BaseType::Bool &&
         type <= reflection::BaseType::ULong;
}

std::string EnumValueName(const reflection::Schema *schema, int32_t enum_index,
                          int64_t value) {
  if (enum_index >= 0 && schema->enums() != nullptr &&
      static_cast<uint32_t>(enum_index) < schema->enums()->size()) {
    const reflection::Enum *enum_def = schema->enums()->Get(enum_index);
    for (const reflection::EnumVal *enum_val : *enum_def->values()) {
      if (enum_val->value() == value) {
        return enum_val->name()->str();
      }
    }
  }
  return absl::StrCat(value);
}

std::string JoinPath(const std::string &parent, std::string_view child) {
  return parent.empty() ? std::string(child) : absl::StrCat(parent, ".", child);
}

}  // namespace

void FaultCollector::AddFault(std::string channel, std::string path,
                              std::string what, std::optional<int64_t> can_id,
                              const LogTime &time) {
  auto key = std::make_tuple(channel, path, what);
  auto it = faults_.find(key);
  if (it == faults_.end()) {
    it = faults_
             .emplace(std::move(key), Fault{.channel = std::move(channel),
                                            .path = std::move(path),
                                            .what = std::move(what),
                                            .can_id = can_id,
                                            .first = time,
                                            .last = time,
                                            .count = 0})
             .first;
  }
  it->second.last = time;
  ++it->second.count;
}

void FaultCollector::ScanMessage(const aos::Channel *channel,
                                 const LogTime &time, const void *data) {
  const reflection::Schema *schema = channel->schema();
  if (schema == nullptr || schema->root_table() == nullptr || data == nullptr) {
    return;
  }
  const std::string channel_key = ChannelKey(channel);

  // Iterative walk to keep stack use bounded on deep messages.
  struct Item {
    const reflection::Object *object;
    const flatbuffers::Table *table;
    std::string path;
  };
  std::vector<Item> stack;
  stack.push_back({schema->root_table(),
                   flatbuffers::GetAnyRoot(static_cast<const uint8_t *>(data)),
                   ""});

  while (!stack.empty()) {
    Item item = std::move(stack.back());
    stack.pop_back();
    if (item.table == nullptr || item.object->fields() == nullptr) {
      continue;
    }

    // Label faults with the CAN id when this table has one.
    std::optional<int64_t> can_id;
    for (const reflection::Field *field : *item.object->fields()) {
      if (field->name()->string_view() == "id" &&
          IsIntegerType(field->type()->base_type()) &&
          item.table->CheckField(field->offset())) {
        can_id = flatbuffers::GetAnyFieldI(*item.table, *field);
      }
    }

    for (const reflection::Field *field : *item.object->fields()) {
      const std::string_view name = field->name()->string_view();
      const reflection::Type *type = field->type();
      switch (type->base_type()) {
        case reflection::BaseType::Bool:
          if (name == "estopped" &&
              flatbuffers::GetAnyFieldI(*item.table, *field) != 0) {
            AddFault(channel_key, JoinPath(item.path, name), "estopped", can_id,
                     time);
          }
          break;
        case reflection::BaseType::Obj: {
          const reflection::Object *child =
              schema->objects()->Get(type->index());
          if (child->is_struct()) {
            break;
          }
          stack.push_back({child, flatbuffers::GetFieldT(*item.table, *field),
                           JoinPath(item.path, name)});
          break;
        }
        case reflection::BaseType::Vector: {
          const reflection::BaseType element = type->element();
          if (element == reflection::BaseType::Obj) {
            const reflection::Object *child =
                schema->objects()->Get(type->index());
            if (child->is_struct()) {
              break;
            }
            const auto *vector =
                item.table->GetPointer<const flatbuffers::Vector<
                    flatbuffers::Offset<flatbuffers::Table>> *>(
                    field->offset());
            if (vector == nullptr) {
              break;
            }
            for (flatbuffers::uoffset_t i = 0; i < vector->size(); ++i) {
              stack.push_back(
                  {child, vector->Get(i),
                   absl::StrCat(JoinPath(item.path, name), "[", i, "]")});
            }
          } else if (name == "errors" && IsIntegerType(element)) {
            const flatbuffers::VectorOfAny *vector =
                flatbuffers::GetFieldAnyV(*item.table, *field);
            if (vector == nullptr) {
              break;
            }
            for (flatbuffers::uoffset_t i = 0; i < vector->size(); ++i) {
              const int64_t value =
                  flatbuffers::GetAnyVectorElemI(vector, element, i);
              AddFault(channel_key, JoinPath(item.path, name),
                       EnumValueName(schema, type->index(), value), can_id,
                       time);
            }
          }
          break;
        }
        default:
          break;
      }
    }
  }
}

void FaultCollector::AddLogMessage(const LogTime &time, int level,
                                   std::string_view name,
                                   std::string_view message) {
  const auto level_enum = static_cast<aos::logging::Level>(level);
  if (level_enum != aos::logging::Level::kERROR &&
      level_enum != aos::logging::Level::kFATAL) {
    return;
  }
  if (log_messages_.size() >= max_log_messages_) {
    ++dropped_log_messages_;
    return;
  }
  log_messages_.push_back(LogMessage{
      .time = time,
      .level = level_enum == aos::logging::Level::kERROR ? "ERROR" : "FATAL",
      .name = std::string(name),
      .message = std::string(message),
  });
}

void FaultCollector::AddRobotState(const LogTime &time, bool browned_out,
                                   double voltage_battery) {
  if (browned_out) {
    AddFault("/frc frc.RobotState", "browned_out", "browned_out", std::nullopt,
             time);
  }
  // 0 means the voltage wasn't filled in.
  if (voltage_battery > 0.0 &&
      (!min_battery_voltage_ ||
       voltage_battery < min_battery_voltage_->first)) {
    min_battery_voltage_ = std::make_pair(voltage_battery, time);
  }
}

std::vector<FaultCollector::Fault> FaultCollector::faults() const {
  std::vector<Fault> result;
  for (const auto &[key, fault] : faults_) {
    result.push_back(fault);
  }
  std::stable_sort(result.begin(), result.end(),
                   [](const Fault &a, const Fault &b) {
                     return a.first.monotonic < b.first.monotonic;
                   });
  return result;
}

nlohmann::json ToJson(const FaultCollector::Fault &fault) {
  nlohmann::json json = {
      {"channel", fault.channel},   {"path", fault.path},
      {"what", fault.what},         {"first", ToJson(fault.first)},
      {"last", ToJson(fault.last)}, {"count", fault.count},
  };
  json["can_id"] =
      fault.can_id ? nlohmann::json(*fault.can_id) : nlohmann::json(nullptr);
  return json;
}

nlohmann::json ToJson(const FaultCollector::LogMessage &message) {
  return {
      {"time", ToJson(message.time)},
      {"level", message.level},
      {"name", message.name},
      {"message", message.message},
  };
}

// ---------------------------------------------------------------------------
// Analysis.

nlohmann::json AnalyzeLog(const std::vector<std::string> &log_paths,
                          const std::string &node_name) {
  const std::vector<aos::logger::LogFile> log_files =
      aos::logger::SortParts(aos::logger::FindLogs(log_paths));
  ABSL_CHECK(!log_files.empty()) << ": No log files found.";

  aos::logger::LogReader reader(log_files);
  reader.Register();
  const aos::Configuration *config = reader.configuration();

  std::string replay_node = node_name;
  if (replay_node.empty() && aos::configuration::MultiNode(config)) {
    const std::set<std::string> logger_nodes =
        aos::logger::LoggerNodes(log_files);
    ABSL_CHECK_EQ(logger_nodes.size(), 1u)
        << ": Log has several logger nodes, pass a node.";
    replay_node = *logger_nodes.begin();
  }
  const aos::Node *node = aos::configuration::MultiNode(config)
                              ? aos::configuration::GetNode(config, replay_node)
                              : nullptr;
  ABSL_CHECK(!aos::configuration::MultiNode(config) || node != nullptr)
      << ": Unknown node " << replay_node;

  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("log_manager", node);
  event_loop->SkipTimingReport();
  event_loop->SkipAosLog();

  MatchTracker matches;
  FaultCollector faults;
  std::unordered_map<const aos::Channel *, uint64_t> counts;
  std::optional<LogTime> first_time;
  LogTime last_time;

  for (const aos::Channel *channel : *config->channels()) {
    if (!aos::configuration::ChannelIsReadableOnNode(channel, node)) {
      continue;
    }
    const std::string_view type = channel->type()->string_view();
    const bool is_joystick = type == "frc.JoystickState";
    const bool is_robot_state = type == "frc.RobotState";
    const bool is_log_message = type == "aos.logging.LogMessageFbs";
    const bool scan_faults =
        EndsWith(type, ".Position") || EndsWith(type, ".Status");
    counts[channel] = 0;

    event_loop->MakeRawWatcher(
        channel, [&, channel, is_joystick, is_robot_state, is_log_message,
                  scan_faults](const aos::Context &context, const void *data) {
          const LogTime time{.monotonic = context.monotonic_event_time,
                             .realtime = context.realtime_event_time};
          if (!first_time) first_time = time;
          last_time = time;
          ++counts[channel];
          if (data == nullptr) {
            return;
          }
          if (is_joystick) {
            matches.Add(time, *flatbuffers::GetRoot<frc::JoystickState>(data));
          } else if (is_robot_state) {
            const auto *state = flatbuffers::GetRoot<frc::RobotState>(data);
            faults.AddRobotState(time, state->browned_out(),
                                 state->voltage_battery());
          } else if (is_log_message) {
            const auto *message =
                flatbuffers::GetRoot<aos::logging::LogMessageFbs>(data);
            faults.AddLogMessage(
                time, static_cast<int>(message->level()),
                message->has_name() ? message->name()->string_view() : "",
                message->has_message() ? message->message()->string_view()
                                       : "");
          } else if (scan_faults) {
            faults.ScanMessage(channel, time, data);
          }
        });
  }

  reader.event_loop_factory()->Run();
  matches.Finish();
  // The event loop has to go before the factory does in Deregister().
  event_loop.reset();

  nlohmann::json channels = nlohmann::json::array();
  for (const aos::Channel *channel : *config->channels()) {
    auto it = counts.find(channel);
    if (it == counts.end() || it->second == 0) {
      continue;
    }
    channels.push_back({{"name", channel->name()->str()},
                        {"type", channel->type()->str()},
                        {"count", it->second}});
  }

  nlohmann::json summary;
  summary["node"] = replay_node;
  summary["start"] = first_time ? ToJson(*first_time) : nlohmann::json(nullptr);
  summary["end"] = first_time ? ToJson(last_time) : nlohmann::json(nullptr);
  summary["channels"] = channels;
  summary["matches"] = nlohmann::json::array();
  for (const MatchTracker::Match &match : matches.matches()) {
    summary["matches"].push_back(ToJson(match));
  }
  summary["enabled_periods"] = nlohmann::json::array();
  for (const MatchTracker::EnabledPeriod &period : matches.enabled_periods()) {
    summary["enabled_periods"].push_back(ToJson(period));
  }
  summary["faults"] = nlohmann::json::array();
  for (const FaultCollector::Fault &fault : faults.faults()) {
    summary["faults"].push_back(ToJson(fault));
  }
  summary["log_messages"] = nlohmann::json::array();
  for (const FaultCollector::LogMessage &message : faults.log_messages()) {
    summary["log_messages"].push_back(ToJson(message));
  }
  summary["dropped_log_messages"] = faults.dropped_log_messages();
  if (faults.min_battery_voltage()) {
    summary["min_battery_voltage"] = {
        {"volts", faults.min_battery_voltage()->first},
        {"time", ToJson(faults.min_battery_voltage()->second)}};
  } else {
    summary["min_battery_voltage"] = nullptr;
  }

  reader.Deregister();
  return summary;
}

}  // namespace frc::logs

// Command line front end for frc/logs/log_manager.h.  robot_server's /logs
// (aos/network/robot_server/logs.rs) runs this as a child process so a crash on
// a corrupt log can't take the server down, and so jobs can be cancelled and
// run at low priority.
//
//   log_manager list <dir>
//   log_manager presets
//   log_manager analyze [--output=summary.json] <log_dir>
//   log_manager convert [--preset=all] [--channels='<name> <type>;...']
//       <log_dir> <output.mcap>
//
// Everything prints JSON on stdout.

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_split.h"

#include "aos/init.h"
#include "frc/logs/log_manager.h"

ABSL_FLAG(std::string, output, "",
          "analyze: also write the summary JSON to this file.");
ABSL_FLAG(std::string, preset, "all",
          "convert: channel preset to convert. See `log_manager presets`.");
ABSL_FLAG(std::string, channels, "",
          "convert: ';'-separated list of exact '<name> <type>' channels to "
          "convert instead of --preset.");
ABSL_FLAG(std::string, presets, "",
          "A JSON file of channel presets to use instead of the generic ones: "
          "{\"presets\": [{\"name\", \"description\", \"regexes\"}]}.");
ABSL_FLAG(std::string, replay_node, "",
          "Node to replay from. Defaults to the node that wrote the log.");

namespace {

constexpr char kUsage[] = R"(Usage:
  log_manager list <dir>
  log_manager presets
  log_manager analyze [--output=summary.json] <log_dir>
  log_manager convert [--preset=all | --channels='<name> <type>;...'] <log_dir> <output.mcap>)";

int Fail(const std::string &message) {
  std::cerr << message << "\n" << kUsage << std::endl;
  return 2;
}

int List(const std::vector<std::string> &args) {
  if (args.size() != 1) return Fail("list takes one directory.");
  nlohmann::json logs = nlohmann::json::array();
  for (const frc::logs::LogDirInfo &info : frc::logs::ListLogs(args[0])) {
    logs.push_back(frc::logs::ToJson(info));
  }
  std::cout << nlohmann::json{{"logs", logs}}.dump(2) << std::endl;
  return 0;
}

int Analyze(const std::vector<std::string> &args) {
  if (args.empty()) return Fail("analyze takes a log directory.");
  const nlohmann::json summary =
      frc::logs::AnalyzeLog(args, absl::GetFlag(FLAGS_replay_node));
  const std::string output = absl::GetFlag(FLAGS_output);
  if (!output.empty()) {
    const std::string tmp = output + ".tmp";
    {
      std::ofstream file(tmp);
      file << summary.dump(2) << std::endl;
      if (!file) {
        ABSL_LOG(ERROR) << "Failed to write " << tmp;
        return 1;
      }
    }
    if (std::rename(tmp.c_str(), output.c_str()) != 0) {
      ABSL_LOG(ERROR) << "Failed to rename " << tmp << " to " << output;
      return 1;
    }
  }
  std::cout << summary.dump(2) << std::endl;
  return 0;
}

int Convert(const std::vector<std::string> &args) {
  if (args.size() < 2) return Fail("convert takes a log directory and output.");
  const std::vector<std::string> log_paths(args.begin(), args.end() - 1);
  const std::string output = args.back();

  std::vector<std::string> selection;
  const std::string channels = absl::GetFlag(FLAGS_channels);
  if (!channels.empty()) {
    for (std::string_view channel :
         absl::StrSplit(channels, ';', absl::SkipWhitespace())) {
      const size_t space = channel.find(' ');
      if (space == std::string_view::npos) {
        return Fail("--channels entries must look like '<name> <type>'.");
      }
      selection.push_back(frc::logs::ChannelRegex(channel.substr(0, space),
                                                  channel.substr(space + 1)));
    }
  } else {
    const frc::logs::ChannelPreset *preset =
        frc::logs::FindPreset(absl::GetFlag(FLAGS_preset));
    if (preset == nullptr) {
      return Fail("Unknown --preset " + absl::GetFlag(FLAGS_preset));
    }
    selection = preset->regexes;
  }

  const int result = frc::logs::ConvertToMcap(
      log_paths, output, frc::logs::IncludeRegexes(selection),
      absl::GetFlag(FLAGS_replay_node));
  if (result == 0) {
    std::cout << nlohmann::json{{"output", output}}.dump(2) << std::endl;
  }
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(kUsage);
  aos::InitGoogle(&argc, &argv);

  if (const std::string presets = absl::GetFlag(FLAGS_presets);
      !presets.empty()) {
    std::string error;
    if (!frc::logs::LoadPresets(presets, &error)) return Fail(error);
  }
  if (argc < 2) return Fail("Missing command.");
  const std::string command = argv[1];
  const std::vector<std::string> args(argv + 2, argv + argc);

  if (command == "list") return List(args);
  if (command == "presets") {
    std::cout << frc::logs::PresetsJson().dump(2) << std::endl;
    return 0;
  }
  if (command == "analyze") return Analyze(args);
  if (command == "convert") return Convert(args);
  return Fail("Unknown command " + command);
}

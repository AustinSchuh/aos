#include "aos/util/proc_stat.h"

#include <dirent.h>

#include <array>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/strip.h"

namespace aos::util {
namespace {
std::optional<std::string> ReadShortFile(std::string_view file_name) {
  // Open as input and seek to end immediately.
  std::ifstream file(std::string(file_name), std::ios_base::in);
  if (!file.good()) {
    ABSL_VLOG(1) << "Can't read " << file_name;
    return std::nullopt;
  }
  const size_t kMaxLineLength = 4096;
  char buffer[kMaxLineLength];
  file.read(buffer, kMaxLineLength);
  if (!file.eof()) {
    return std::nullopt;
  }
  return std::string(buffer, file.gcount());
}
}  // namespace

std::optional<ProcStat> ReadProcStat(const pid_t pid,
                                     const std::optional<pid_t> tid) {
  const std::string path =
      tid.has_value() ? absl::StrFormat("/proc/%d/task/%d/stat", pid, *tid)
                      : absl::StrFormat("/proc/%d/stat", pid);
  const std::optional<std::string> contents = ReadShortFile(path);
  if (!contents.has_value()) {
    return std::nullopt;
  }
  const size_t start_name = contents->find_first_of('(');
  const size_t end_name = contents->find_last_of(')');
  if (start_name == std::string::npos || end_name == std::string::npos ||
      end_name < start_name) {
    ABSL_VLOG(1) << "No name found in stat line " << contents.value();
    return std::nullopt;
  }
  std::string_view name(contents->c_str() + start_name + 1,
                        end_name - start_name - 1);

  std::vector<std::string_view> fields =
      absl::StrSplit(std::string_view(contents->c_str() + end_name + 1,
                                      contents->size() - end_name - 1),
                     ' ', absl::SkipWhitespace());
  constexpr int kNumFieldsAfterName = 50;
  if (fields.size() != kNumFieldsAfterName) {
    ABSL_VLOG(1) << "Incorrect number of fields " << fields.size();
    return std::nullopt;
  }
  // The first field is a character for the current process state; every single
  // field after that should be an integer.
  if (fields[0].size() != 1) {
    ABSL_VLOG(1) << "State field is too long: " << fields[0];
    return std::nullopt;
  }
  std::array<absl::int128, kNumFieldsAfterName - 1> numbers;
  for (int ii = 1; ii < kNumFieldsAfterName; ++ii) {
    if (!absl::SimpleAtoi(fields[ii], &numbers[ii - 1])) {
      ABSL_VLOG(1) << "Failed to parse field " << ii
                   << " as number: " << fields[ii];
      return std::nullopt;
    }
  }
  return ProcStat{
      .pid = pid,
      .name = std::string(name),
      .state = fields.at(0).at(0),
      .parent_pid = static_cast<int64_t>(numbers.at(0)),
      .group_id = static_cast<int64_t>(numbers.at(1)),
      .session_id = static_cast<int64_t>(numbers.at(2)),
      .tty = static_cast<int64_t>(numbers.at(3)),
      .tpgid = static_cast<int64_t>(numbers.at(4)),
      .kernel_flags = static_cast<uint64_t>(numbers.at(5)),
      .minor_faults = static_cast<uint64_t>(numbers.at(6)),
      .children_minor_faults = static_cast<uint64_t>(numbers.at(7)),
      .major_faults = static_cast<uint64_t>(numbers.at(8)),
      .children_major_faults = static_cast<uint64_t>(numbers.at(9)),
      .user_mode_ticks = static_cast<uint64_t>(numbers.at(10)),
      .kernel_mode_ticks = static_cast<uint64_t>(numbers.at(11)),
      .children_user_mode_ticks = static_cast<int64_t>(numbers.at(12)),
      .children_kernel_mode_ticks = static_cast<int64_t>(numbers.at(13)),
      .priority = static_cast<int64_t>(numbers.at(14)),
      .nice = static_cast<int64_t>(numbers.at(15)),
      .num_threads = static_cast<int64_t>(numbers.at(16)),
      .itrealvalue = static_cast<int64_t>(numbers.at(17)),
      .start_time_ticks = static_cast<uint64_t>(numbers.at(18)),
      .virtual_memory_size = static_cast<uint64_t>(numbers.at(19)),
      .resident_set_size = static_cast<int64_t>(numbers.at(20)),
      .rss_soft_limit = static_cast<uint64_t>(numbers.at(21)),
      .start_code_address = static_cast<uint64_t>(numbers.at(22)),
      .end_code_address = static_cast<uint64_t>(numbers.at(23)),
      .start_stack_address = static_cast<uint64_t>(numbers.at(24)),
      .stack_pointer = static_cast<uint64_t>(numbers.at(25)),
      .instruction_pointer = static_cast<uint64_t>(numbers.at(26)),
      .signal_bitmask = static_cast<uint64_t>(numbers.at(27)),
      .blocked_signals = static_cast<uint64_t>(numbers.at(28)),
      .ignored_signals = static_cast<uint64_t>(numbers.at(29)),
      .caught_signals = static_cast<uint64_t>(numbers.at(30)),
      .wchan = static_cast<uint64_t>(numbers.at(31)),
      .swap_pages = static_cast<uint64_t>(numbers.at(32)),
      .children_swap_pages = static_cast<uint64_t>(numbers.at(33)),
      .exit_signal = static_cast<int64_t>(numbers.at(34)),
      .processor = static_cast<int64_t>(numbers.at(35)),
      .rt_priority = static_cast<uint64_t>(numbers.at(36)),
      .scheduling_policy = static_cast<uint64_t>(numbers.at(37)),
      .block_io_delay_ticks = static_cast<uint64_t>(numbers.at(38)),
      .guest_ticks = static_cast<uint64_t>(numbers.at(39)),
      .children_guest_ticks = static_cast<uint64_t>(numbers.at(40)),
      .start_data_address = static_cast<uint64_t>(numbers.at(41)),
      .end_data_address = static_cast<uint64_t>(numbers.at(42)),
      .start_brk_address = static_cast<uint64_t>(numbers.at(43)),
      .start_arg_address = static_cast<uint64_t>(numbers.at(44)),
      .end_arg_address = static_cast<uint64_t>(numbers.at(45)),
      .start_env_address = static_cast<uint64_t>(numbers.at(46)),
      .end_env_address = static_cast<uint64_t>(numbers.at(47)),
      .exit_code = static_cast<int64_t>(numbers.at(48))};
}

std::optional<std::vector<pid_t>> ListThreads(pid_t pid) {
  const std::string task_dir = absl::StrFormat("/proc/%d/task", pid);
  DIR *dir = opendir(task_dir.c_str());
  if (dir == nullptr) {
    return std::nullopt;
  }
  std::vector<pid_t> tids;
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    pid_t tid;
    if (entry->d_type == DT_DIR && absl::SimpleAtoi(entry->d_name, &tid)) {
      tids.push_back(tid);
    }
  }
  closedir(dir);
  return tids;
}

std::vector<pid_t> ListProcesses() {
  DIR *const dir = opendir("/proc");
  if (dir == nullptr) {
    ABSL_PLOG(FATAL) << "Failed to open /proc";
  }
  std::vector<pid_t> pids;
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    pid_t pid;
    if (entry->d_type == DT_DIR && absl::SimpleAtoi(entry->d_name, &pid)) {
      pids.push_back(pid);
    }
  }
  closedir(dir);
  return pids;
}

}  // namespace aos::util

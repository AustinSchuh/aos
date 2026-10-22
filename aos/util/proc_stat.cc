#include "aos/util/proc_stat.h"

#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include "absl/strings/str_split.h"
#include "absl/strings/strip.h"

namespace aos::util {

bool MountinfoHidesPids(std::string_view mountinfo) {
  // proc(5): each line is
  //   ID parent major:minor root mount_point mount_opts [optional...] - fstype
  //   source super_opts
  // hidepid is a super option of the proc filesystem, so it is after the " - "
  // separator, and the mount point is the fifth field.
  for (std::string_view line : absl::StrSplit(mountinfo, '\n')) {
    const std::vector<std::string_view> halves =
        absl::StrSplit(line, absl::MaxSplits(" - ", 1));
    if (halves.size() != 2) {
      continue;
    }
    const std::vector<std::string_view> fields =
        absl::StrSplit(halves[0], ' ', absl::SkipEmpty());
    if (fields.size() < 5 || fields[4] != "/proc") {
      continue;
    }
    const std::vector<std::string_view> after =
        absl::StrSplit(halves[1], ' ', absl::SkipEmpty());
    if (after.size() < 3 || after[0] != "proc") {
      continue;
    }
    for (std::string_view option : absl::StrSplit(after[2], ',')) {
      if (!absl::ConsumePrefix(&option, "hidepid=")) {
        continue;
      }
      // hidepid=0 and hidepid=off are the default: nothing hidden.
      if (option != "0" && option != "off") {
        return true;
      }
    }
  }
  return false;
}

bool ProcHidesPids() {
  std::ifstream file("/proc/self/mountinfo", std::ios_base::in);
  if (!file.good()) {
    return false;
  }
  std::string contents((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
  return MountinfoHidesPids(contents);
}

}  // namespace aos::util

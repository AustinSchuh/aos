#include "aos/util/proc_stat.h"

#include <libproc.h>
#include <mach/mach_time.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"

namespace aos::util {
namespace {

// Linux reports times in clock ticks, so convert to the same.
int64_t ClockTicksPerSecond() {
  static const int64_t ticks = sysconf(_SC_CLK_TCK);
  return ticks;
}

// libproc reports task CPU times in mach_absolute_time() units, which aren't
// nanoseconds on Apple silicon.
uint64_t MachTimeToClockTicks(uint64_t mach_time) {
  static const mach_timebase_info_data_t timebase = []() {
    mach_timebase_info_data_t result;
    ABSL_CHECK_EQ(mach_timebase_info(&result), KERN_SUCCESS);
    return result;
  }();
  return static_cast<uint64_t>(absl::uint128(mach_time) * timebase.numer *
                               ClockTicksPerSecond() /
                               (absl::uint128(timebase.denom) * 1000000000));
}

// Microseconds since the epoch at which the system booted.
int64_t BootTimeMicroseconds() {
  static const int64_t boot_time = []() {
    struct timeval boot_time;
    size_t size = sizeof(boot_time);
    ABSL_PCHECK(sysctlbyname("kern.boottime", &boot_time, &size, nullptr, 0) ==
                0);
    return static_cast<int64_t>(boot_time.tv_sec) * 1000000 + boot_time.tv_usec;
  }();
  return boot_time;
}

char StatusToState(uint32_t status) {
  switch (status) {
    case SSLEEP:
      return 'S';
    case SSTOP:
      return 'T';
    case SZOMB:
      return 'Z';
    case SIDL:
    case SRUN:
    default:
      return 'R';
  }
}

}  // namespace

std::optional<ProcStat> ReadProcStat(const pid_t pid,
                                     const std::optional<pid_t> tid) {
  if (tid.has_value()) {
    return std::nullopt;
  }

  struct proc_taskallinfo info;
  const int size =
      proc_pidinfo(pid, PROC_PIDTASKALLINFO, 0, &info, sizeof(info));
  if (size != static_cast<int>(sizeof(info))) {
    ABSL_VLOG(1) << "Can't read process info for " << pid;
    return std::nullopt;
  }

  // pbi_name is the longer name, but isn't always filled in.
  const char *name =
      info.pbsd.pbi_name[0] != '\0' ? info.pbsd.pbi_name : info.pbsd.pbi_comm;

  const int64_t start_time_us =
      static_cast<int64_t>(info.pbsd.pbi_start_tvsec) * 1000000 +
      info.pbsd.pbi_start_tvusec;

  ProcStat result{};
  result.pid = pid;
  result.name = std::string(name, strnlen(name, sizeof(info.pbsd.pbi_name)));
  result.state = StatusToState(info.pbsd.pbi_status);
  result.parent_pid = info.pbsd.pbi_ppid;
  result.user_mode_ticks = MachTimeToClockTicks(info.ptinfo.pti_total_user);
  result.kernel_mode_ticks = MachTimeToClockTicks(info.ptinfo.pti_total_system);
  result.num_threads = info.ptinfo.pti_threadnum;
  result.start_time_ticks = (start_time_us - BootTimeMicroseconds()) *
                            ClockTicksPerSecond() / 1000000;
  result.virtual_memory_size = info.ptinfo.pti_virtual_size;
  result.resident_set_size =
      info.ptinfo.pti_resident_size / sysconf(_SC_PAGESIZE);
  return result;
}

std::optional<std::vector<pid_t>> ListThreads(pid_t /*pid*/) {
  return std::vector<pid_t>();
}

std::vector<pid_t> ListProcesses() {
  std::vector<pid_t> pids;
  while (true) {
    const int count = proc_listallpids(nullptr, 0);
    ABSL_PCHECK(count >= 0) << ": Failed to count processes";
    // Leave room for processes started in between.
    pids.resize(count + 64);
    const int listed = proc_listallpids(
        pids.data(), static_cast<int>(pids.size() * sizeof(pid_t)));
    ABSL_PCHECK(listed >= 0) << ": Failed to list processes";
    if (static_cast<size_t>(listed) < pids.size()) {
      pids.resize(listed);
      return pids;
    }
  }
}

}  // namespace aos::util

#ifndef AOS_UTIL_PROC_STAT_H_
#define AOS_UTIL_PROC_STAT_H_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace aos::util {
// ProcStat is a struct to hold all the fields available in /proc/[pid]/stat.
// Currently we only use a small subset of the fields. See man 5 proc for
// details on what the fields are--these are in the same order as they appear in
// the stat file.
//
// Things are signed or unsigned based on whether they are listed
// as signed/unsigned in man 5 proc. We just make everything 64 bits wide
// because otherwise we have to write out way too many casts everywhere.
struct ProcStat {
  int pid;
  std::string name;
  char state;
  int64_t parent_pid;
  int64_t group_id;
  int64_t session_id;
  int64_t tty;
  int64_t tpgid;
  uint64_t kernel_flags;
  uint64_t minor_faults;
  uint64_t children_minor_faults;
  uint64_t major_faults;
  uint64_t children_major_faults;
  uint64_t user_mode_ticks;
  uint64_t kernel_mode_ticks;
  int64_t children_user_mode_ticks;
  int64_t children_kernel_mode_ticks;
  int64_t priority;
  int64_t nice;
  int64_t num_threads;
  int64_t itrealvalue;  // always zero.
  uint64_t start_time_ticks;
  uint64_t virtual_memory_size;
  // Number of pages in real memory.
  int64_t resident_set_size;
  uint64_t rss_soft_limit;
  uint64_t start_code_address;
  uint64_t end_code_address;
  uint64_t start_stack_address;
  uint64_t stack_pointer;
  uint64_t instruction_pointer;
  uint64_t signal_bitmask;
  uint64_t blocked_signals;
  uint64_t ignored_signals;
  uint64_t caught_signals;
  uint64_t wchan;
  // swap_pages fields are not maintained.
  uint64_t swap_pages;
  uint64_t children_swap_pages;
  int64_t exit_signal;
  // CPU number last exitted on.
  int64_t processor;
  // Zero for non-realtime processes.
  uint64_t rt_priority;
  uint64_t scheduling_policy;
  // Aggregated block I/O delay.
  uint64_t block_io_delay_ticks;
  uint64_t guest_ticks;
  uint64_t children_guest_ticks;
  uint64_t start_data_address;
  uint64_t end_data_address;
  uint64_t start_brk_address;
  uint64_t start_arg_address;
  uint64_t end_arg_address;
  uint64_t start_env_address;
  uint64_t end_env_address;
  int64_t exit_code;
};

// Retrieves the statistics for a particular process or thread. If only a pid is
// provided, it reads the process's stat file at /proc/[pid]/stat. If both pid
// and tid are provided, it reads the thread's stat file at
// /proc/[pid]/task/[tid]/stat. Returns nullopt if unable to read or parse the
// file.
std::optional<ProcStat> ReadProcStat(pid_t pid,
                                     std::optional<pid_t> tid = std::nullopt);

// Whether the /proc mount described by the given /proc/self/mountinfo contents
// hides other users' processes (mounted with hidepid=1, 2, invisible, or
// ptraceable).  Under such a mount, another user's /proc/<pid> entry is
// indistinguishable from a dead process's.
bool MountinfoHidesPids(std::string_view mountinfo);

// MountinfoHidesPids() for this process's /proc.  False if /proc/self/mountinfo
// cannot be read, since nothing would be hidden from a process that cannot
// read its own mount table either.
bool ProcHidesPids();
}  // namespace aos::util
#endif  // AOS_UTIL_PROC_STAT_H_

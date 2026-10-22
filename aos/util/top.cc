#include "aos/util/top.h"

#include <errno.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <istream>
#include <queue>
#include <ratio>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/absl_log.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"

#define PF_KTHREAD 0x00200000

namespace aos::util {

Top::Top(aos::EventLoop *event_loop, TrackThreadsMode track_threads,
         TrackPerThreadInfoMode track_per_thread_info)
    : event_loop_(event_loop),
      clock_tick_(std::chrono::nanoseconds(1000000000 / sysconf(_SC_CLK_TCK))),
      page_size_(sysconf(_SC_PAGESIZE)),
      track_threads_(track_threads),
      track_per_thread_info_(track_per_thread_info) {
  TimerHandler *timer = event_loop_->AddTimer([this]() { UpdateReadings(); });
  event_loop_->OnRun([timer, this]() {
    timer->Schedule(event_loop_->monotonic_now(), kSamplePeriod);
  });
}

std::chrono::nanoseconds Top::TotalProcessTime(const ProcStat &proc_stat) {
  return (proc_stat.user_mode_ticks + proc_stat.kernel_mode_ticks) *
         clock_tick_;
}

aos::monotonic_clock::time_point Top::ProcessStartTime(
    const ProcStat &proc_stat) {
  return aos::monotonic_clock::time_point(proc_stat.start_time_ticks *
                                          clock_tick_);
}

uint64_t Top::RealMemoryUsage(const ProcStat &proc_stat) {
  return proc_stat.resident_set_size * page_size_;
}

void Top::MaybeAddThreadIds(pid_t pid, std::set<pid_t> *pids) {
  if (track_threads_ == TrackThreadsMode::kDisabled) {
    return;
  }

  // Add all the threads of the process.
  const std::optional<std::vector<pid_t>> tids = ListThreads(pid);
  if (!tids.has_value()) {
    ABSL_LOG(WARNING) << "Unable to list the threads of " << pid;
    return;
  }
  pids->insert(tids->begin(), tids->end());
}

ThreadState CharToThreadState(const char state) {
  switch (state) {
    case 'R':
      return ThreadState::RUNNING;
    case 'S':
      return ThreadState::SLEEPING_INTERRUPTIBLE;
    case 'D':
      return ThreadState::SLEEPING_UNINTERRUPTIBLE;
    case 'T':
      return ThreadState::STOPPED;
    case 'Z':
      return ThreadState::ZOMBIE;
    case 'I':
      return ThreadState::IDLE;
    case 'X':
      return ThreadState::DEAD;
    case 't':
      return ThreadState::TRACING_STOP;
    default:
      ABSL_LOG(FATAL) << "Invalid thread state character: " << state;
  }
}

void Top::UpdateThreadReadings(pid_t pid, ProcessReadings &process) {
  const std::optional<std::vector<pid_t>> tids = ListThreads(pid);
  if (!tids.has_value()) {
    ABSL_LOG_EVERY_N_SEC(WARNING, 10)
        << "Unable to list the threads of " << pid;
    return;
  }

  // Use a set to track all the threads that we process.
  std::set<pid_t> updated_threads;

  for (const pid_t tid : *tids) {
    // Read the stats for the thread.
    const std::optional<ProcStat> thread_stats = ReadProcStat(pid, tid);

    // If no stats could be read (thread may have exited), remove it.
    if (!thread_stats.has_value()) {
      ABSL_VLOG(2) << "Removing thread " << tid << " from process " << pid;
      process.thread_readings.erase(tid);
      continue;
    }

    const ThreadState thread_state = CharToThreadState(thread_stats->state);

    // Find or create new thread reading entry.
    ThreadReadings &thread_reading = process.thread_readings[tid];

    // Update thread name.
    thread_reading.name = thread_stats.value().name;
    thread_reading.start_time = ProcessStartTime(thread_stats.value());

    // Update ThreadReadings with the latest cpu usage.
    aos::RingBuffer<ThreadReading, kRingBufferSize> &readings =
        thread_reading.readings;
    const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
    const std::chrono::nanoseconds run_time =
        TotalProcessTime(thread_stats.value());
    // The ring buffer will push out the oldest entry if it is full.
    readings.Push({now, run_time});

    // If the buffer is full, update the CPU usage percentage.
    if (readings.full()) {
      const ThreadReading &previous = readings[0];
      const ThreadReading &current = readings[1];
      const std::chrono::nanoseconds run_time =
          current.total_run_time - previous.total_run_time;
      const std::chrono::nanoseconds reading_time =
          current.reading_time - previous.reading_time;
      thread_reading.cpu_percent = aos::time::DurationInSeconds(run_time) /
                                   aos::time::DurationInSeconds(reading_time);
      thread_reading.state = thread_state;
    }
    updated_threads.insert(tid);
  }

  // Remove all threads from process.thread_readings that didn't get updated.
  std::vector<pid_t> threads_to_remove;
  for (const auto &[tid, thread_reading] : process.thread_readings) {
    if (!updated_threads.contains(tid)) {
      threads_to_remove.push_back(tid);
    }
  }
  for (const pid_t tid : threads_to_remove) {
    process.thread_readings.erase(tid);
  }
}

void Top::UpdateReadings() {
  aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
  // Get all the processes that we *might* care about.
  std::set<pid_t> pids = pids_to_track_;
  // Ensure that we check on the status of every process that we are already
  // tracking.
  for (const auto &reading : readings_) {
    pids.insert(reading.first);
    MaybeAddThreadIds(reading.first, &pids);
  }
  if (track_all_) {
    for (const pid_t pid : ListProcesses()) {
      pids.insert(pid);
      MaybeAddThreadIds(pid, &pids);
    }
  }

  for (const pid_t pid : pids) {
    std::optional<ProcStat> proc_stat = ReadProcStat(pid);
    // Stop tracking processes that have died.
    if (!proc_stat.has_value()) {
      readings_.erase(pid);
      continue;
    }
    const aos::monotonic_clock::time_point start_time =
        ProcessStartTime(*proc_stat);
    auto reading_iter = readings_.find(pid);
    if (reading_iter == readings_.end()) {
      reading_iter =
          readings_
              .insert(std::make_pair(
                  pid,
                  ProcessReadings{
                      .name = proc_stat->name,
                      .start_time = start_time,
                      .cpu_percent = 0.0,
                      .kthread = !!(proc_stat->kernel_flags & PF_KTHREAD),
                      .readings = {},
                      .thread_readings = {},
                  }))
              .first;
    }
    ProcessReadings &process = reading_iter->second;
    // The process associated with the PID has changed; reset the state.
    if (process.start_time != start_time) {
      process.name = proc_stat->name;
      process.start_time = start_time;
      process.readings.Reset();
    }
    // If the process name has changed (e.g., if our first reading for a process
    // name occurred before execvp was called), then update it.
    if (process.name != proc_stat->name) {
      process.name = proc_stat->name;
    }

    process.readings.Push(Reading{now, TotalProcessTime(*proc_stat),
                                  RealMemoryUsage(*proc_stat)});
    if (process.readings.full()) {
      process.cpu_percent =
          aos::time::DurationInSeconds(process.readings[1].total_run_time -
                                       process.readings[0].total_run_time) /
          aos::time::DurationInSeconds(process.readings[1].reading_time -
                                       process.readings[0].reading_time);
    } else {
      process.cpu_percent = 0.0;
    }

    // Update thread readings for this process
    if (track_per_thread_info_ == TrackPerThreadInfoMode::kEnabled) {
      UpdateThreadReadings(pid, process);
    }
  }

  if (on_reading_update_) {
    on_reading_update_();
  }
}

flatbuffers::Offset<ProcessInfo> Top::InfoForProcess(
    flatbuffers::FlatBufferBuilder *fbb, pid_t pid) {
  auto reading_iter = readings_.find(pid);
  if (reading_iter == readings_.end()) {
    return {};
  }
  const ProcessReadings &reading = reading_iter->second;

  if (reading.readings.empty()) {
    return {};  // Return an empty offset if readings is empty.
  }

  std::vector<flatbuffers::Offset<ThreadInfo>> thread_infos_offsets;
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ThreadInfo>>>
      threads_vector_offset;

  if (track_per_thread_info_ == TrackPerThreadInfoMode::kEnabled &&
      !reading.thread_readings.empty()) {
    thread_infos_offsets.reserve(reading.thread_readings.size());
    for (const auto &[tid, thread_reading] : reading.thread_readings) {
      // Calculate how long the thread has been alive by comparing the thread
      // start time to the current time.
      const aos::monotonic_clock::time_point start_time =
          thread_reading.start_time;
      // convert start_time to int64
      const int64_t start_time_ns = start_time.time_since_epoch().count();

      const flatbuffers::Offset<flatbuffers::String> threadName =
          fbb->CreateString(thread_reading.name);
      ThreadInfo::Builder thread_info_builder(*fbb);
      thread_info_builder.add_tid(tid);
      thread_info_builder.add_name(threadName);
      thread_info_builder.add_cpu_usage(thread_reading.cpu_percent);
      thread_info_builder.add_start_time(start_time_ns);
      thread_info_builder.add_state(thread_reading.state);
      const flatbuffers::Offset<ThreadInfo> threadInfo =
          thread_info_builder.Finish();
      thread_infos_offsets.push_back(threadInfo);
    }
    threads_vector_offset = fbb->CreateVector(thread_infos_offsets);
  } else {
    threads_vector_offset = 0;
  }

  // Create name string offset
  const flatbuffers::Offset<flatbuffers::String> name =
      fbb->CreateString(reading.name);
  ProcessInfo::Builder builder(*fbb);
  builder.add_pid(pid);
  builder.add_name(name);
  builder.add_cpu_usage(reading.cpu_percent);
  builder.add_physical_memory(
      reading.readings[reading.readings.size() - 1].memory_usage);
  if (!threads_vector_offset.IsNull()) {
    builder.add_threads(threads_vector_offset);
  }

  return builder.Finish();
}

flatbuffers::Offset<TopProcessesFbs> Top::TopProcesses(
    flatbuffers::FlatBufferBuilder *fbb, int n) {
  // Pair is {cpu_usage, pid}.
  std::priority_queue<std::pair<double, pid_t>> cpu_usages;
  for (const auto &pair : readings_) {
    // Deliberately include 0.0 percent CPU things in the usage list so that if
    // the user asks for an arbitrarily large number of processes they'll get
    // everything.
    cpu_usages.push(std::make_pair(pair.second.cpu_percent, pair.first));
  }
  std::vector<flatbuffers::Offset<ProcessInfo>> offsets;
  for (int ii = 0; ii < n && !cpu_usages.empty(); ++ii) {
    offsets.push_back(InfoForProcess(fbb, cpu_usages.top().second));
    cpu_usages.pop();
  }
  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<ProcessInfo>>>
      vector_offset = fbb->CreateVector(offsets);
  TopProcessesFbs::Builder builder(*fbb);
  builder.add_processes(vector_offset);
  return builder.Finish();
}

}  // namespace aos::util

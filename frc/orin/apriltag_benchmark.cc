// Replays a directory of CameraImage frames (see apriltag_corpus_extract)
// through GpuDetector the way apriltag_detector runs it on the robot, and
// reports how long detection takes.  It can also write out what it detected
// and compare that against an earlier run, so a change can be checked for both
// speed and for whether it changed any detections.

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/tag36h11.h"

#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc/orin/apriltag.h"
#include "frc/orin/cuda.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, corpus, "",
          "Directory of CameraImage .bfbs frames to replay.  A constants.bfbs "
          "or constants.json in it supplies the camera calibrations.");
ABSL_FLAG(std::string, calibration, "",
          "CameraConstants .json or .bfbs file to take the camera "
          "calibrations from, or \"fallback\" for the built in one.  Empty "
          "means constants.bfbs or constants.json in --corpus, then the "
          "built in one.");
ABSL_FLAG(int32_t, skip_frames, 0,
          "Skip this many frames at the start of the sorted corpus.");
ABSL_FLAG(int32_t, max_frames, 0, "If > 0, only replay this many frames.");
ABSL_FLAG(int32_t, iterations, 1, "Number of passes over the corpus.");
ABSL_FLAG(int32_t, measured_frames, 0,
          "If > 0, each detector measures this many frames, continuing "
          "around the corpus from where it starts, instead of --iterations "
          "passes over it.");
ABSL_FLAG(int32_t, warmup, 10, "Untimed detections to run before measuring.");
ABSL_FLAG(int32_t, parallel, 1,
          "Processes to run detectors in at once, the way the robot runs one "
          "detector process per camera.  Each process has its own CUDA "
          "context, and the GPU time slices between them.");
ABSL_FLAG(int32_t, detectors_per_process, 1,
          "Detectors each --parallel process runs at once, each on a thread "
          "of its own with its own streams.  They share the process's CUDA "
          "context, so the GPU can run their kernels side by side instead of "
          "switching contexts.  --parallel=1 --detectors_per_process=4 runs "
          "four cameras the way one process for all of them would.");
ABSL_FLAG(double, pace_hz, 0,
          "If > 0, feed each detector frames at this rate the way a camera "
          "does, instead of as fast as it can take them.  Each detector gets "
          "a random phase, and a frame it gets to too late is dropped.");
ABSL_FLAG(double, pace_max_age_ms, 0,
          "With --pace_hz, drop frames older than this when the detector "
          "gets to them, like gpu_apriltag's --max_image_age_ms.  0 drops "
          "every frame a newer one has already arrived behind.");
ABSL_FLAG(std::string, pace_phases, "seed",
          "How --pace_hz phases the detectors: \"seed\" gives each one a "
          "random phase from --pace_seed, and \"aligned\" gives them all the "
          "same phase, the way cameras triggered together would.");
ABSL_FLAG(uint32_t, pace_seed, 0, "Seed for the --pace_hz phases.");
ABSL_FLAG(int32_t, pace_seeds, 1,
          "Measure this many times, with seeds --pace_seed, --pace_seed + 1, "
          "and so on, and report all of them together.  One seed's phases "
          "can happen to keep the detectors apart or pile them up; a range "
          "of them shows what a robot typically sees.");
ABSL_FLAG(bool, cold_input, false,
          "Have a helper thread write each image a few frames ahead into a "
          "ring of buffers, the way the camera pipeline writes images on "
          "another core, instead of copying it in right before Detect().  "
          "The image then isn't in the detecting core's cache.");
ABSL_FLAG(uint32_t, min_white_black_diff, 20,
          "Matches --min_white_black_diff on the robot.");
ABSL_FLAG(uint32_t, min_cluster_pixels, 25,
          "Matches --min_cluster_pixels on the robot.");
ABSL_FLAG(int32_t, nthreads, 6,
          "Decode threads per detector, counting the one which runs it.");
ABSL_FLAG(bool, stage_timing, false,
          "Report the time each GPU stage takes.  The stages then run one "
          "after another on one stream instead of as the frame's CUDA graph, "
          "and recording their events costs CPU and latency, so leave this "
          "off when measuring GPU time, CPU or latency.");
ABSL_FLAG(bool, gpu_time, true,
          "Report each frame's GPU time.  Recording it costs an event record "
          "and read on every frame, which the robot skips.");
ABSL_FLAG(int32_t, realtime_priority, 0,
          "If > 0, run each detector's main thread at this SCHED_FIFO "
          "priority, the way apriltag_detector runs at 21 on the robot.  The "
          "decode threads stay SCHED_OTHER there and here.");
ABSL_FLAG(int32_t, gpu_keepalive_us, 0,
          "If > 0, launch an empty kernel on a stream of its own this often, "
          "in each detector's process, so the GPU doesn't power gate between "
          "frames.  Waking up adds about 0.3 ms to a frame.");
ABSL_FLAG(int32_t, host_wait_spin_us, 0,
          "Microseconds the detector polls each GPU event before it sleeps "
          "on it.");
ABSL_FLAG(std::string, detections_out, "",
          "If set, write every detection from the first pass to this file.");
ABSL_FLAG(std::string, compare_to, "",
          "If set, compare this run's detections against a file written by "
          "--detections_out and fail if they differ.");
ABSL_FLAG(double, compare_tolerance, 0.0,
          "Largest difference in any corner coordinate or decision margin "
          "--compare_to accepts.");
ABSL_FLAG(std::string, timing_csv, "",
          "If set, write per-frame timings of the first detector here.");

namespace frc::orin {
namespace {

using apriltag::CameraMatrix;
using apriltag::DistCoeffs;
using apriltag::GpuDetector;

struct Frame {
  std::string name;
  int camera;
  aos::FlatbufferVector<vision::CameraImage> image;
};

struct Calibration {
  CameraMatrix camera_matrix;
  DistCoeffs dist_coeffs;
};

// One frame's worth of measurements.  Times are aos::monotonic_clock
// nanoseconds, which every process shares.
struct FrameTiming {
  // When the frame arrived with --pace_hz, 0 otherwise.
  int64_t tick_ns;
  int64_t start_ns;
  int64_t end_ns;
  float wall_ms;
  float gpu_ms;
  float host_decode_ms;
  std::vector<std::pair<std::string_view, float>> stages;
};

struct DetectorResults {
  std::vector<FrameTiming> timings;
  // Arrival times of the frames --pace_hz dropped.
  std::vector<int64_t> dropped_ticks_ns;
  // CPU time of the whole process while measuring.  With several detectors in
  // a process, the first one's results carry it and the rest have 0, so the
  // sum counts each process once.
  double cpu_s = 0;
  // The part of cpu_s spent copying images in, which the robot doesn't do.
  double input_cpu_s = 0;
  // Frame name -> formatted detections.
  std::map<std::string, std::vector<std::string>> detections;
  size_t num_detections = 0;
};

int64_t Nanoseconds(aos::monotonic_clock::time_point time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             time.time_since_epoch())
      .count();
}

aos::monotonic_clock::time_point FromNanoseconds(int64_t ns) {
  return aos::monotonic_clock::time_point(std::chrono::nanoseconds(ns));
}

void SleepUntil(aos::monotonic_clock::time_point time) {
  const struct timespec deadline = aos::time::to_timespec(time);
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr) ==
         EINTR) {
  }
}

// CPU time of all threads in this process.
double CpuSeconds() {
  struct rusage usage;
  ABSL_PCHECK(getrusage(RUSAGE_SELF, &usage) == 0);
  return usage.ru_utime.tv_sec + usage.ru_stime.tv_sec +
         1e-6 * (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec);
}

// CPU time of the thread `clock` belongs to, from pthread_getcpuclockid or
// CLOCK_THREAD_CPUTIME_ID.
double ThreadCpuSeconds(clockid_t clock) {
  struct timespec time;
  ABSL_PCHECK(clock_gettime(clock, &time) == 0);
  return time.tv_sec + 1e-9 * time.tv_nsec;
}

std::vector<Frame> LoadCorpus() {
  const std::string corpus = absl::GetFlag(FLAGS_corpus);
  ABSL_CHECK(!corpus.empty()) << ": --corpus is required";
  std::vector<std::string> paths;
  for (const auto &entry : std::filesystem::directory_iterator(corpus)) {
    if (entry.path().extension() == ".bfbs" &&
        entry.path().filename() != "constants.bfbs") {
      paths.push_back(entry.path().string());
    }
  }
  std::sort(paths.begin(), paths.end());
  paths.erase(
      paths.begin(),
      paths.begin() +
          std::min<size_t>(paths.size(),
                           std::max(0, absl::GetFlag(FLAGS_skip_frames))));
  if (absl::GetFlag(FLAGS_max_frames) > 0 &&
      paths.size() > static_cast<size_t>(absl::GetFlag(FLAGS_max_frames))) {
    paths.resize(absl::GetFlag(FLAGS_max_frames));
  }
  ABSL_CHECK(!paths.empty()) << ": No frames in " << corpus;

  const std::regex camera_regex("camera([0-9]+)");
  std::vector<Frame> frames;
  frames.reserve(paths.size());
  for (const std::string &path : paths) {
    const std::string name = std::filesystem::path(path).filename().string();
    std::smatch match;
    const int camera = std::regex_search(name, match, camera_regex)
                           ? std::stoi(match[1].str())
                           : 0;
    frames.push_back(Frame{
        .name = name,
        .camera = camera,
        .image = aos::FileToFlatbuffer<vision::CameraImage>(path),
    });
    ABSL_CHECK_EQ(frames.back().image.message().rows(),
                  frames.front().image.message().rows());
    ABSL_CHECK_EQ(frames.back().image.message().cols(),
                  frames.front().image.message().cols());
    ABSL_CHECK(frames.back().image.message().format() ==
               frames.front().image.message().format());
  }
  return frames;
}

// Returns the calibration for each camera number in `constants`, read the way
// gpu_apriltag reads it.
std::map<int, Calibration> CalibrationsFrom(
    const vision::CameraConstants &constants) {
  std::map<int, Calibration> result;
  if (!constants.has_calibration()) {
    return result;
  }
  for (const vision::calibration::CameraCalibration *calibration :
       *constants.calibration()) {
    result[calibration->camera_number()] = Calibration{
        .camera_matrix = apriltag::GetCameraMatrix(calibration),
        .dist_coeffs = apriltag::GetDistCoeffs(calibration),
    };
  }
  return result;
}

// Returns the calibration for each camera number per --calibration, and sets
// `source` to where it came from.
std::map<int, Calibration> LoadCalibrations(std::string *source) {
  std::string path = absl::GetFlag(FLAGS_calibration);
  if (path.empty()) {
    for (const char *name : {"constants.bfbs", "constants.json"}) {
      const std::string candidate =
          absl::StrCat(absl::GetFlag(FLAGS_corpus), "/", name);
      if (std::filesystem::exists(candidate)) {
        path = candidate;
        break;
      }
    }
  }

  std::map<int, Calibration> result;
  if (!path.empty() && path != "fallback") {
    result = path.ends_with(".json")
                 ? CalibrationsFrom(
                       aos::JsonFileToFlatbuffer<vision::CameraConstants>(path)
                           .message())
                 : CalibrationsFrom(
                       aos::FileToFlatbuffer<vision::CameraConstants>(path)
                           .message());
    *source = path;
  }
  if (result.empty()) {
    if (path != "fallback") {
      ABSL_LOG(WARNING)
          << "No calibration in "
          << (path.empty() ? absl::GetFlag(FLAGS_corpus) : path)
          << ", using a 1280x720, 5-parameter calibration from another "
             "camera.  It doesn't fit these images, so RefineEdges' "
             "UnDistort takes far more iterations than on the robot and "
             "fewer tags are detected.";
    }
    *source = "fallback (1280x720, 5 parameters)";
    result[0] = Calibration{
        .camera_matrix = {.fx = 893.7595,
                          .cx = 645.47076,
                          .fy = 893.22235,
                          .cy = 388.15027},
        .dist_coeffs = {.k1 = -0.44902,
                        .k2 = 0.248409,
                        .p1 = -0.000537,
                        .p2 = -0.000112,
                        .k3 = -0.076989,
                        .k4 = 0,
                        .k5 = 0,
                        .k6 = 0,
                        .num_params = 5},
    };
  }
  return result;
}

const Calibration &CalibrationFor(const std::map<int, Calibration> &calibs,
                                  int camera) {
  auto it = calibs.find(camera);
  return it == calibs.end() ? calibs.begin()->second : it->second;
}

// When in each --pace_hz period detector `index` gets its frames.
std::chrono::nanoseconds PacePhase(uint32_t pace_seed, int index) {
  if (absl::GetFlag(FLAGS_pace_phases) == "aligned") {
    return std::chrono::nanoseconds(0);
  }
  std::seed_seq seed{pace_seed, static_cast<uint32_t>(index)};
  std::mt19937 rng(seed);
  return std::chrono::nanoseconds(
      static_cast<int64_t>(std::uniform_real_distribution<double>(
          0, 1e9 / absl::GetFlag(FLAGS_pace_hz))(rng)));
}

std::string FormatDetection(const apriltag_detection_t *det) {
  // %.9g round-trips the float decision margin and %.17g the double center
  // and corners, so equal lines mean bit-identical detections.
  std::string result =
      absl::StrFormat("%d %d %.9g %.17g %.17g", det->id, det->hamming,
                      det->decision_margin, det->c[0], det->c[1]);
  for (int i = 0; i < 4; ++i) {
    absl::StrAppendFormat(&result, " %.17g %.17g", det->p[i][0], det->p[i][1]);
  }
  return result;
}

// Puts each image into mapped pinned memory for Detect().  On the robot the
// image sits in shared memory which is registered with CUDA, so the GPU reads
// it in place; mapped pinned memory is the same thing without the shared
// memory.
class ImageSource {
 public:
  // `image_for` returns the image for a sequence number.  Sequence numbers
  // passed to Get() only go up.
  ImageSource(size_t image_bytes, bool cold,
              std::function<const uint8_t *(size_t)> image_for)
      : image_bytes_(image_bytes),
        image_for_(std::move(image_for)),
        slots_(cold ? kSlots : 1) {
    for (Slot &slot : slots_) {
      CHECK_CUDA(cudaHostAlloc(reinterpret_cast<void **>(&slot.host),
                               image_bytes_, cudaHostAllocMapped));
      CHECK_CUDA(cudaHostGetDevicePointer(
          reinterpret_cast<void **>(&slot.device), slot.host, 0));
    }
    if (cold) {
      filler_ = std::thread([this]() { Fill(); });
    }
  }
  ~ImageSource() {
    if (filler_.joinable()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        done_ = true;
      }
      changed_.notify_all();
      filler_.join();
    }
    for (Slot &slot : slots_) {
      CHECK_CUDA(cudaFreeHost(slot.host));
    }
  }

  // Returns the host and device pointers to image `sequence`, which stay
  // valid until the next call.
  std::pair<const uint8_t *, const uint8_t *> Get(size_t sequence) {
    if (!filler_.joinable()) {
      const double cpu_start = ThreadCpuSeconds(CLOCK_THREAD_CPUTIME_ID);
      memcpy(slots_[0].host, image_for_(sequence), image_bytes_);
      copy_cpu_s_ += ThreadCpuSeconds(CLOCK_THREAD_CPUTIME_ID) - cpu_start;
      return {slots_[0].host, slots_[0].device};
    }
    std::unique_lock<std::mutex> lock(mutex_);
    ABSL_CHECK_GE(sequence, current_);
    current_ = sequence;
    changed_.notify_all();
    Slot &slot = slots_[sequence % kSlots];
    changed_.wait(lock, [&]() { return slot.sequence == sequence; });
    return {slot.host, slot.device};
  }

  // CPU time spent copying images in so far.  With a filler thread, that is
  // all of its CPU time.
  double CopyCpuSeconds() {
    if (!filler_.joinable()) {
      return copy_cpu_s_;
    }
    clockid_t clock;
    ABSL_CHECK_EQ(pthread_getcpuclockid(filler_.native_handle(), &clock), 0);
    return ThreadCpuSeconds(clock);
  }

 private:
  // Enough slots that the filler never catches up with the one in use.
  static constexpr size_t kSlots = 16;
  // How many images ahead of the one in use the filler writes.
  static constexpr size_t kLookahead = 4;
  static constexpr size_t kEmpty = std::numeric_limits<size_t>::max();

  struct Slot {
    uint8_t *host = nullptr;
    uint8_t *device = nullptr;
    size_t sequence = kEmpty;
  };

  void Fill() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      changed_.wait(lock,
                    [&]() { return done_ || next_ < current_ + kLookahead; });
      if (done_) {
        return;
      }
      // Skip images the consumer has already moved past.
      const size_t sequence = std::max(next_, current_);
      next_ = sequence + 1;
      Slot &slot = slots_[sequence % kSlots];
      slot.sequence = kEmpty;
      lock.unlock();
      memcpy(slot.host, image_for_(sequence), image_bytes_);
      lock.lock();
      slot.sequence = sequence;
      changed_.notify_all();
    }
  }

  const size_t image_bytes_;
  const std::function<const uint8_t *(size_t)> image_for_;
  std::vector<Slot> slots_;
  std::thread filler_;
  // CPU time Get() has spent copying, without a filler thread.
  double copy_cpu_s_ = 0;

  std::mutex mutex_;
  std::condition_variable changed_;
  // The sequence number the consumer wants or is using.
  size_t current_ = 0;
  // The next sequence number the filler writes.
  size_t next_ = 0;
  bool done_ = false;
};

__global__ void KeepaliveKernel() {}

// Launches an empty kernel every --gpu_keepalive_us so the GPU stays powered
// up between frames.  The kernel runs on a non-blocking stream of its own and
// touches no memory, so it doesn't change which managed memory the CPU may
// access while the detector's stream is idle.
class GpuKeepalive {
 public:
  explicit GpuKeepalive(std::chrono::microseconds period) {
    if (period > std::chrono::microseconds(0)) {
      thread_ = std::thread([this, period]() { Run(period); });
    }
  }
  ~GpuKeepalive() {
    if (thread_.joinable()) {
      done_ = true;
      thread_.join();
    }
  }

 private:
  void Run(std::chrono::microseconds period) {
    cudaStream_t stream;
    CHECK_CUDA(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
    while (!done_) {
      KeepaliveKernel<<<1, 1, 0, stream>>>();
      CHECK_CUDA(cudaGetLastError());
      SleepUntil(aos::monotonic_clock::now() + period);
    }
    CHECK_CUDA(cudaStreamSynchronize(stream));
    CHECK_CUDA(cudaStreamDestroy(stream));
  }

  std::atomic<bool> done_ = false;
  std::thread thread_;
};

// Runs detector `index` of `detectors` over the corpus.  Each one starts at a
// different place in the corpus so parallel detectors aren't all looking at
// the same frame.  After warming up, it calls `ready`, which returns when to
// start measuring, and it calls `finished` once it has measured every frame.
DetectorResults RunDetector(
    const std::vector<Frame> &frames, const std::map<int, Calibration> &calibs,
    int index, int detectors, uint32_t pace_seed, bool record_detections,
    const std::function<aos::monotonic_clock::time_point()> &ready,
    const std::function<void()> &finished) {
  const vision::CameraImage &first = frames.front().image.message();
  const size_t width = first.cols();
  const size_t height = first.rows();
  const size_t image_bytes = first.data()->size();
  const size_t offset = index * frames.size() / detectors;
  const size_t warmup = absl::GetFlag(FLAGS_warmup);
  const size_t total = absl::GetFlag(FLAGS_measured_frames) > 0
                           ? absl::GetFlag(FLAGS_measured_frames)
                           : frames.size() * absl::GetFlag(FLAGS_iterations);
  const double pace_hz = absl::GetFlag(FLAGS_pace_hz);
  const bool stage_timing = absl::GetFlag(FLAGS_stage_timing);

  // Warmup runs sequence numbers [0, warmup), and pass index i of the
  // measured frames is sequence number warmup + i.
  auto frame_for = [&](size_t sequence) -> const Frame & {
    const size_t pass_index = sequence < warmup ? sequence : sequence - warmup;
    return frames[(pass_index + offset) % frames.size()];
  };

  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_t *tag_detector = apriltag_detector_create();
  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);
  tag_detector->nthreads = absl::GetFlag(FLAGS_nthreads);
  tag_detector->qtp.min_white_black_diff =
      absl::GetFlag(FLAGS_min_white_black_diff);
  tag_detector->qtp.min_cluster_pixels =
      absl::GetFlag(FLAGS_min_cluster_pixels);

  DetectorResults results;
  {
    const Calibration &calib = CalibrationFor(calibs, frames.front().camera);
    GpuDetector detector(width, height, tag_detector, calib.camera_matrix,
                         calib.dist_coeffs, first.format());
    detector.set_stage_timing(stage_timing);
    detector.set_measure_gpu_time(absl::GetFlag(FLAGS_gpu_time));
    detector.set_host_wait_spin(
        std::chrono::microseconds(absl::GetFlag(FLAGS_host_wait_spin_us)));
    ImageSource source(
        image_bytes, absl::GetFlag(FLAGS_cold_input), [&](size_t sequence) {
          return frame_for(sequence).image.message().data()->data();
        });

    // Makes image `sequence` and its calibration ready for Detect().
    auto prepare = [&](size_t sequence) {
      const Frame &frame = frame_for(sequence);
      const auto [image_host, image_device] = source.Get(sequence);
      const Calibration &calib = CalibrationFor(calibs, frame.camera);
      detector.SetCameraMatrix(calib.camera_matrix);
      detector.SetDistortionCoefficients(calib.dist_coeffs);
      return std::make_pair(image_host, image_device);
    };

    // The decode and input threads already exist, so they keep SCHED_OTHER.
    // The old policy comes back at the end, or threads this thread starts
    // later would inherit SCHED_FIFO.
    struct RestoreScheduler {
      RestoreScheduler() : policy(sched_getscheduler(0)) {
        ABSL_PCHECK(policy >= 0);
        ABSL_PCHECK(sched_getparam(0, &param) == 0);
      }
      ~RestoreScheduler() {
        ABSL_PCHECK(sched_setscheduler(0, policy, &param) == 0);
      }
      const int policy;
      struct sched_param param;
    } restore_scheduler;
    if (const int priority = absl::GetFlag(FLAGS_realtime_priority);
        priority > 0) {
      struct sched_param param;
      param.sched_priority = priority;
      ABSL_PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);
    }

    for (size_t i = 0; i < warmup; ++i) {
      const auto [image_host, image_device] = prepare(i);
      detector.Detect(image_host, image_device);
    }

    const aos::monotonic_clock::time_point epoch = ready();
    SleepUntil(epoch);
    const std::chrono::nanoseconds period =
        pace_hz > 0
            ? std::chrono::nanoseconds(static_cast<int64_t>(1e9 / pace_hz))
            : std::chrono::nanoseconds(0);
    const aos::monotonic_clock::time_point first_tick =
        epoch + (pace_hz > 0 ? PacePhase(pace_seed, index)
                             : std::chrono::nanoseconds(0));
    const aos::monotonic_clock::duration max_age =
        std::chrono::duration_cast<aos::monotonic_clock::duration>(
            std::chrono::duration<double, std::milli>(
                absl::GetFlag(FLAGS_pace_max_age_ms)));
    auto tick = [&](size_t pass_index) {
      return first_tick + static_cast<int64_t>(pass_index) * period;
    };
    // Whether the detector drops frame `pass_index` when it gets to it at
    // `now`.
    auto stale = [&](size_t pass_index, aos::monotonic_clock::time_point now) {
      return max_age > aos::monotonic_clock::duration(0)
                 ? now - tick(pass_index) > max_age
                 : now >= tick(pass_index + 1);
    };

    const double cpu_start = CpuSeconds();
    const double input_cpu_start = source.CopyCpuSeconds();
    results.timings.reserve(total);
    for (size_t pass_index = 0; pass_index < total; ++pass_index) {
      if (pace_hz > 0) {
        const aos::monotonic_clock::time_point now =
            aos::monotonic_clock::now();
        while (pass_index + 1 < total && stale(pass_index, now)) {
          results.dropped_ticks_ns.push_back(Nanoseconds(tick(pass_index)));
          ++pass_index;
        }
      }
      const Frame &frame = frame_for(warmup + pass_index);
      const auto [image_host, image_device] = prepare(warmup + pass_index);
      if (pace_hz > 0) {
        SleepUntil(tick(pass_index));
      }

      const aos::monotonic_clock::time_point start =
          aos::monotonic_clock::now();
      detector.Detect(image_host, image_device);
      const aos::monotonic_clock::time_point end = aos::monotonic_clock::now();

      results.timings.push_back(FrameTiming{
          .tick_ns = pace_hz > 0 ? Nanoseconds(tick(pass_index)) : 0,
          .start_ns = Nanoseconds(start),
          .end_ns = Nanoseconds(end),
          .wall_ms =
              std::chrono::duration<float, std::milli>(end - start).count(),
          .gpu_ms = detector.GpuTimeMs(),
          .host_decode_ms = detector.HostDecodeTimeMs(),
          .stages = detector.StageTimesMs(),
      });

      const zarray_t *detections = detector.Detections();
      results.num_detections += zarray_size(detections);
      // Only the first pass; later passes see the same frames again.
      if (record_detections && pass_index < frames.size()) {
        std::vector<std::string> formatted;
        for (int j = 0; j < zarray_size(detections); ++j) {
          apriltag_detection_t *det;
          zarray_get(detections, j, &det);
          formatted.push_back(FormatDetection(det));
        }
        // Detections() comes out in a fixed order, but the --compare_to
        // baselines come from the original detector, whose decode threads
        // finished in any order, so the detections are compared sorted.
        std::sort(formatted.begin(), formatted.end());
        results.detections[frame.name] = std::move(formatted);
      }
    }
    finished();
    results.cpu_s = CpuSeconds() - cpu_start;
    results.input_cpu_s = source.CopyCpuSeconds() - input_cpu_start;
  }

  apriltag_detector_destroy(tag_detector);
  tag36h11_destroy(tag_family);
  return results;
}

// Runs detectors [first_index, first_index + count) of `detectors` in this
// process, each on a thread of its own.  Once all of them have warmed up, one
// calls `ready`, which returns when to start measuring.
std::vector<DetectorResults> RunDetectorsInProcess(
    const std::vector<Frame> &frames, const std::map<int, Calibration> &calibs,
    int first_index, int count, int detectors, uint32_t pace_seed,
    bool record_detections,
    const std::function<aos::monotonic_clock::time_point()> &ready) {
  // Started before any detector's realtime priority, which it doesn't need.
  GpuKeepalive keepalive(
      std::chrono::microseconds(absl::GetFlag(FLAGS_gpu_keepalive_us)));
  if (count == 1) {
    return {RunDetector(frames, calibs, first_index, detectors, pace_seed,
                        record_detections, ready, []() {})};
  }

  std::mutex mutex;
  std::condition_variable changed;
  int warm = 0;
  int done = 0;
  std::optional<aos::monotonic_clock::time_point> epoch;
  double cpu_start = 0;
  double cpu_end = 0;
  std::vector<DetectorResults> results(count);
  std::vector<std::thread> threads;
  for (int i = 0; i < count; ++i) {
    threads.emplace_back([&, i]() {
      results[i] = RunDetector(
          frames, calibs, first_index + i, detectors, pace_seed,
          record_detections,
          [&]() {
            std::unique_lock<std::mutex> lock(mutex);
            if (++warm == count) {
              epoch = ready();
              cpu_start = CpuSeconds();
              changed.notify_all();
            }
            changed.wait(lock, [&]() { return epoch.has_value(); });
            return *epoch;
          },
          // Every detector keeps its GPU memory until all of them are done,
          // because freeing it synchronizes the whole device.
          [&]() {
            std::unique_lock<std::mutex> lock(mutex);
            if (++done == count) {
              cpu_end = CpuSeconds();
              changed.notify_all();
            }
            changed.wait(lock, [&]() { return done == count; });
          });
    });
  }
  for (std::thread &thread : threads) {
    thread.join();
  }
  // Each detector measured the whole process's CPU time over its own frames,
  // which overlap.  Count the process once, from when they all started until
  // the last one finished.
  for (DetectorResults &result : results) {
    result.cpu_s = 0;
  }
  results.front().cpu_s = cpu_end - cpu_start;
  return results;
}

struct Stats {
  double mean, p50, p90, p99, max;
};

Stats ComputeStats(std::vector<float> values) {
  ABSL_CHECK(!values.empty());
  std::sort(values.begin(), values.end());
  auto percentile = [&](double p) {
    return values[std::min(values.size() - 1,
                           static_cast<size_t>(p * values.size()))];
  };
  return Stats{
      .mean =
          std::accumulate(values.begin(), values.end(), 0.0) / values.size(),
      .p50 = percentile(0.5),
      .p90 = percentile(0.9),
      .p99 = percentile(0.99),
      .max = values.back(),
  };
}

void PrintStats(std::string_view label, const std::vector<float> &values) {
  const Stats stats = ComputeStats(values);
  std::cout << absl::StrFormat(
      "  %-22s mean %7.3f  p50 %7.3f  p90 %7.3f  p99 %7.3f  max %7.3f ms\n",
      label, stats.mean, stats.p50, stats.p90, stats.p99, stats.max);
}

// Reports on the frames inside the window where every detector of a run was
// measuring, so setup, warmup, and whichever detectors finish first don't
// dilute the contended numbers.  The runs' windows are reported together.
// `processes` is how many processes the detectors ran in.
void Report(const std::vector<std::vector<DetectorResults>> &runs,
            int processes) {
  size_t frames = 0;
  size_t detections = 0;
  double cpu_s = 0;
  double input_cpu_s = 0;
  std::vector<float> wall, gpu, host_decode, residual, latency;
  // Stage name -> times, kept in first-seen order.
  std::vector<std::string_view> stage_order;
  std::map<std::string_view, std::vector<float>> stages;
  size_t window_frames = 0;
  size_t window_dropped = 0;
  double window_s = 0;
  for (const std::vector<DetectorResults> &all : runs) {
    int64_t window_start = std::numeric_limits<int64_t>::min();
    int64_t window_end = std::numeric_limits<int64_t>::max();
    for (const DetectorResults &results : all) {
      ABSL_CHECK(!results.timings.empty());
      window_start = std::max(window_start, results.timings.front().start_ns);
      window_end = std::min(window_end, results.timings.back().end_ns);
      frames += results.timings.size();
      detections += results.num_detections;
      cpu_s += results.cpu_s;
      input_cpu_s += results.input_cpu_s;
    }
    ABSL_CHECK_LT(window_start, window_end)
        << ": The detectors never ran at the same time";
    window_s += 1e-9 * (window_end - window_start);
    auto in_window = [&](int64_t ns) {
      return ns >= window_start && ns <= window_end;
    };

    for (const DetectorResults &results : all) {
      for (const FrameTiming &timing : results.timings) {
        if (!in_window(timing.start_ns) || !in_window(timing.end_ns)) {
          continue;
        }
        ++window_frames;
        wall.push_back(timing.wall_ms);
        if (!std::isnan(timing.gpu_ms)) {
          gpu.push_back(timing.gpu_ms);
          residual.push_back(timing.wall_ms - timing.gpu_ms -
                             timing.host_decode_ms);
        }
        host_decode.push_back(timing.host_decode_ms);
        if (timing.tick_ns != 0) {
          latency.push_back(1e-6 * (timing.end_ns - timing.tick_ns));
        }
        for (const auto &[name, ms] : timing.stages) {
          auto [it, inserted] = stages.try_emplace(name);
          if (inserted) {
            stage_order.push_back(name);
          }
          it->second.push_back(ms);
        }
      }
      window_dropped +=
          std::count_if(results.dropped_ticks_ns.begin(),
                        results.dropped_ticks_ns.end(), in_window);
    }
  }

  std::cout << absl::StrFormat(
      "%zu frames across %zu detector(s) in %d process(es), %zu detections, "
      "%.1f frames/s, %.3f ms CPU/frame, %.3f without copying the input in\n",
      frames, runs.front().size(), processes, detections,
      window_frames / window_s, 1000.0 * cpu_s / frames,
      1000.0 * (cpu_s - input_cpu_s) / frames);
  std::cout << absl::StrFormat(
      "  Steady state: %zu frames in the %.3f s all detectors ran\n",
      window_frames, window_s);
  PrintStats("Detect() wall", wall);
  if (!gpu.empty()) {
    PrintStats("GPU (events)", gpu);
  }
  PrintStats("Host decode", host_decode);
  if (!residual.empty()) {
    // What neither the GPU nor the decode accounts for.  When the GPU has
    // powered down between frames, waking it adds about 0.3 ms before the
    // first kernel starts, which shows up here on some share of frames.  That
    // share can be over half, so it is measured from the fastest frames, not
    // from the median.  When every frame pays it, as a lone detector paced at
    // 60 Hz does, the share is 0 and only the residual itself shows it.  With
    // --use_neon, the host threshold runs before the GPU time starts, so the
    // residual includes it too and can't be compared with the GPU threshold's.
    PrintStats("Wall - GPU - decode", residual);
    std::vector<float> sorted = residual;
    std::sort(sorted.begin(), sorted.end());
    const float slow = sorted[sorted.size() / 100] + 0.2;
    std::cout << absl::StrFormat(
        "  Residual over p1 + 0.2 ms on %.1f%% of frames\n",
        100.0 *
            std::count_if(residual.begin(), residual.end(),
                          [&](float value) { return value > slow; }) /
            residual.size());
  }
  if (!latency.empty()) {
    PrintStats("Latency", latency);
    std::cout << absl::StrFormat(
        "  Dropped %zu of %zu frames (%.3f%%), %.2f/s\n", window_dropped,
        window_dropped + window_frames,
        100.0 * window_dropped / (window_dropped + window_frames),
        window_dropped / window_s);
  }
  if (!stage_order.empty()) {
    std::cout << "  GPU stages (mean ms):\n";
  }
  for (std::string_view name : stage_order) {
    const std::vector<float> &values = stages[name];
    std::cout << absl::StrFormat(
        "    %-22s %7.4f\n", name,
        std::accumulate(values.begin(), values.end(), 0.0) / values.size());
  }
}

void WriteDetections(const DetectorResults &results, const std::string &path) {
  std::ofstream out(path);
  ABSL_CHECK(out) << ": Failed to open " << path;
  for (const auto &[name, detections] : results.detections) {
    out << "frame " << name << " " << detections.size() << "\n";
    for (const std::string &detection : detections) {
      out << detection << "\n";
    }
  }
}

std::map<std::string, std::vector<std::string>> ReadDetections(
    const std::string &path) {
  std::ifstream in(path);
  ABSL_CHECK(in) << ": Failed to open " << path;
  std::map<std::string, std::vector<std::string>> result;
  std::string line;
  std::vector<std::string> *current = nullptr;
  while (std::getline(in, line)) {
    if (line.starts_with("frame ")) {
      std::vector<std::string> parts = absl::StrSplit(line, ' ');
      current = &result[parts[1]];
    } else {
      ABSL_CHECK(current != nullptr);
      current->push_back(line);
    }
  }
  return result;
}

// Returns true if the detections match within --compare_tolerance.
bool Compare(const DetectorResults &results, const std::string &path) {
  const auto expected = ReadDetections(path);
  const double tolerance = absl::GetFlag(FLAGS_compare_tolerance);
  size_t frames_compared = 0, frames_identical = 0, count_mismatches = 0;
  size_t missing_frames = 0;
  double max_diff = 0;
  std::string worst;
  for (const auto &[name, actual] : results.detections) {
    auto it = expected.find(name);
    if (it == expected.end()) {
      ++missing_frames;
      continue;
    }
    ++frames_compared;
    if (it->second == actual) {
      ++frames_identical;
      continue;
    }
    if (it->second.size() != actual.size()) {
      ++count_mismatches;
      std::cout << "  " << name << ": expected " << it->second.size()
                << " detections, got " << actual.size() << "\n";
      continue;
    }
    for (size_t i = 0; i < actual.size(); ++i) {
      std::vector<std::string> a = absl::StrSplit(it->second[i], ' ');
      std::vector<std::string> b = absl::StrSplit(actual[i], ' ');
      ABSL_CHECK_EQ(a.size(), b.size());
      if (a[0] != b[0] || a[1] != b[1]) {
        ++count_mismatches;
        std::cout << "  " << name << ": expected tag " << a[0] << " got "
                  << b[0] << "\n";
        break;
      }
      for (size_t j = 2; j < a.size(); ++j) {
        const double diff = std::abs(std::stod(a[j]) - std::stod(b[j]));
        if (diff > max_diff) {
          max_diff = diff;
          worst = absl::StrCat(name, " tag ", a[0], " field ", j);
        }
      }
    }
  }
  std::cout << absl::StrFormat(
      "Compared %zu frames against %s: %zu identical, %zu with a different "
      "set of tags, max difference %g (%s), %zu frames missing\n",
      frames_compared, path, frames_identical, count_mismatches, max_diff,
      worst, missing_frames);
  return count_mismatches == 0 && missing_frames == 0 && max_diff <= tolerance;
}

// Timings go from a child process back to the parent as text.  Per detector,
// an "R" line starts its results, followed by a "D <count>" line with the
// number of detections, a "C <seconds>" line with its CPU time, an
// "I <seconds>" line with the part of that spent copying images in, an
// "X <tick>" line per dropped frame, then per frame a "F <tick> <start>
// <end> <wall> <gpu> <host> <stages>" line followed by that many
// "<name>\t<ms>" lines.
std::string SerializeTimings(const std::vector<DetectorResults> &all) {
  std::string out;
  for (const DetectorResults &results : all) {
    absl::StrAppend(&out, "R\n", "D ", results.num_detections, "\n", "C ",
                    absl::StrFormat("%.9g", results.cpu_s), "\n", "I ",
                    absl::StrFormat("%.9g", results.input_cpu_s), "\n");
    for (const int64_t tick_ns : results.dropped_ticks_ns) {
      absl::StrAppend(&out, "X ", tick_ns, "\n");
    }
    for (const FrameTiming &timing : results.timings) {
      absl::StrAppend(&out, "F ", timing.tick_ns, " ", timing.start_ns, " ",
                      timing.end_ns, " ", timing.wall_ms, " ", timing.gpu_ms,
                      " ", timing.host_decode_ms, " ", timing.stages.size(),
                      "\n");
      for (const auto &[name, ms] : timing.stages) {
        absl::StrAppend(&out, name, "\t", ms, "\n");
      }
    }
  }
  return out;
}

// `names` owns the stage names the returned string_views point at.
std::vector<DetectorResults> DeserializeTimings(
    std::string_view data, std::set<std::string, std::less<>> *names) {
  std::vector<DetectorResults> all;
  std::istringstream in{std::string(data)};
  std::string tag;
  while (in >> tag) {
    if (tag == "R") {
      all.emplace_back();
      continue;
    }
    ABSL_CHECK(!all.empty()) << ": Detector results without an R line";
    DetectorResults &results = all.back();
    if (tag == "D") {
      in >> results.num_detections;
      continue;
    }
    if (tag == "C") {
      in >> results.cpu_s;
      continue;
    }
    if (tag == "I") {
      in >> results.input_cpu_s;
      continue;
    }
    if (tag == "X") {
      in >> results.dropped_ticks_ns.emplace_back();
      continue;
    }
    ABSL_CHECK_EQ(tag, "F");
    FrameTiming timing;
    // gpu_ms is "nan" without --gpu_time, which operator>> doesn't parse.
    std::string gpu_ms;
    size_t num_stages;
    in >> timing.tick_ns >> timing.start_ns >> timing.end_ns >>
        timing.wall_ms >> gpu_ms >> timing.host_decode_ms >> num_stages;
    timing.gpu_ms = std::stof(gpu_ms);
    in.ignore();
    for (size_t i = 0; i < num_stages; ++i) {
      std::string line;
      std::getline(in, line);
      const size_t tab = line.find('\t');
      ABSL_CHECK_NE(tab, std::string::npos);
      const std::string_view name = *names->emplace(line.substr(0, tab)).first;
      timing.stages.emplace_back(name, std::stof(line.substr(tab + 1)));
    }
    results.timings.push_back(std::move(timing));
  }
  ABSL_CHECK(in.eof()) << ": Failed to parse detector results";
  return all;
}

void WriteAll(int fd, const void *data, size_t size) {
  size_t written = 0;
  while (written < size) {
    const ssize_t result =
        write(fd, static_cast<const char *>(data) + written, size - written);
    ABSL_PCHECK(result > 0);
    written += result;
  }
}

void ReadAll(int fd, void *data, size_t size) {
  size_t read_bytes = 0;
  while (read_bytes < size) {
    const ssize_t result =
        read(fd, static_cast<char *>(data) + read_bytes, size - read_bytes);
    ABSL_PCHECK(result >= 0);
    ABSL_CHECK_GT(result, 0) << ": Pipe closed early";
    read_bytes += result;
  }
}

// Runs `parallel` child processes of `per_process` detectors each and
// collects their timings.  All of them warm up before any starts measuring, so
// they measure together.  CUDA must not have been touched yet in this process.
std::vector<DetectorResults> RunInChildren(
    const std::vector<Frame> &frames, const std::map<int, Calibration> &calibs,
    int parallel, int per_process, uint32_t pace_seed,
    std::set<std::string, std::less<>> *names) {
  struct Child {
    pid_t pid;
    // Results come back on this.
    int from_child;
    // The start time goes out on this.
    int to_child;
  };
  std::vector<Child> children;
  for (int i = 0; i < parallel; ++i) {
    int from_child[2], to_child[2];
    ABSL_PCHECK(pipe(from_child) == 0);
    ABSL_PCHECK(pipe(to_child) == 0);
    const pid_t pid = fork();
    ABSL_PCHECK(pid >= 0);
    if (pid == 0) {
      close(from_child[0]);
      close(to_child[1]);
      for (const Child &child : children) {
        close(child.from_child);
        close(child.to_child);
      }
      const std::string data = SerializeTimings(RunDetectorsInProcess(
          frames, calibs, i * per_process, per_process, parallel * per_process,
          pace_seed, /*record_detections=*/false, [&]() {
            WriteAll(from_child[1], "W", 1);
            int64_t epoch_ns;
            ReadAll(to_child[0], &epoch_ns, sizeof(epoch_ns));
            return FromNanoseconds(epoch_ns);
          }));
      WriteAll(from_child[1], data.data(), data.size());
      close(from_child[1]);
      _exit(0);
    }
    close(from_child[1]);
    close(to_child[0]);
    children.push_back(Child{
        .pid = pid, .from_child = from_child[0], .to_child = to_child[1]});
  }

  for (const Child &child : children) {
    char warm;
    ReadAll(child.from_child, &warm, 1);
    ABSL_CHECK_EQ(warm, 'W');
  }
  // Leave the children time to wake up before they start.
  const int64_t epoch_ns =
      Nanoseconds(aos::monotonic_clock::now() + std::chrono::milliseconds(20));
  for (const Child &child : children) {
    WriteAll(child.to_child, &epoch_ns, sizeof(epoch_ns));
    close(child.to_child);
  }

  std::vector<DetectorResults> results;
  for (const Child &child : children) {
    std::string data;
    char buffer[65536];
    while (true) {
      const ssize_t result = read(child.from_child, buffer, sizeof(buffer));
      ABSL_PCHECK(result >= 0);
      if (result == 0) {
        break;
      }
      data.append(buffer, result);
    }
    close(child.from_child);
    int status;
    ABSL_PCHECK(waitpid(child.pid, &status, 0) == child.pid);
    ABSL_CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << ": Detector process " << child.pid << " failed";
    for (DetectorResults &detector : DeserializeTimings(data, names)) {
      results.push_back(std::move(detector));
    }
  }
  return results;
}

int Main() {
  const std::vector<Frame> frames = LoadCorpus();
  std::string calibration_source;
  const std::map<int, Calibration> calibs =
      LoadCalibrations(&calibration_source);
  const vision::CameraImage &first = frames.front().image.message();
  std::cout << absl::StrFormat("Loaded %zu %dx%d %s frames\n", frames.size(),
                               first.cols(), first.rows(),
                               vision::EnumNameImageFormat(first.format()));
  std::cout << "Calibration: " << calibration_source << "\n";

  const int parallel = absl::GetFlag(FLAGS_parallel);
  const int per_process = absl::GetFlag(FLAGS_detectors_per_process);
  ABSL_CHECK_GE(parallel, 1);
  ABSL_CHECK_GE(per_process, 1);
  const int detectors = parallel * per_process;
  const bool record = !absl::GetFlag(FLAGS_detections_out).empty() ||
                      !absl::GetFlag(FLAGS_compare_to).empty();
  ABSL_CHECK(parallel == 1 || !record)
      << ": --detections_out and --compare_to need --parallel=1";
  ABSL_CHECK(absl::GetFlag(FLAGS_pace_hz) <= 0 || !record)
      << ": --detections_out and --compare_to can't use --pace_hz, which "
         "drops frames";
  const std::string pace_phases = absl::GetFlag(FLAGS_pace_phases);
  ABSL_CHECK(pace_phases == "seed" || pace_phases == "aligned")
      << ": Unknown --pace_phases " << pace_phases;
  const int pace_seeds = absl::GetFlag(FLAGS_pace_seeds);
  ABSL_CHECK_GE(pace_seeds, 1);
  ABSL_CHECK(pace_seeds == 1 ||
             (absl::GetFlag(FLAGS_pace_hz) > 0 && pace_phases == "seed"))
      << ": --pace_seeds needs --pace_hz and --pace_phases=seed";

  std::vector<std::vector<DetectorResults>> runs;
  std::set<std::string, std::less<>> stage_names;
  for (int run = 0; run < pace_seeds; ++run) {
    const uint32_t pace_seed = absl::GetFlag(FLAGS_pace_seed) + run;
    if (absl::GetFlag(FLAGS_pace_hz) > 0) {
      std::cout << absl::StrFormat("Paced at %g Hz, phases",
                                   absl::GetFlag(FLAGS_pace_hz));
      for (int i = 0; i < detectors; ++i) {
        std::cout << absl::StrFormat(" %.2f",
                                     1e-6 * PacePhase(pace_seed, i).count());
      }
      std::cout << " ms\n";
    }
    if (parallel == 1) {
      runs.push_back(RunDetectorsInProcess(
          frames, calibs, 0, per_process, detectors, pace_seed, record,
          []() { return aos::monotonic_clock::now(); }));
    } else {
      runs.push_back(RunInChildren(frames, calibs, parallel, per_process,
                                   pace_seed, &stage_names));
    }
  }
  Report(runs, parallel);
  const std::vector<DetectorResults> &results = runs.front();

  if (!absl::GetFlag(FLAGS_timing_csv).empty()) {
    std::ofstream out(absl::GetFlag(FLAGS_timing_csv));
    out << "wall_ms,gpu_ms,host_decode_ms\n";
    for (const FrameTiming &timing : results[0].timings) {
      out << timing.wall_ms << "," << timing.gpu_ms << ","
          << timing.host_decode_ms << "\n";
    }
  }
  if (!absl::GetFlag(FLAGS_detections_out).empty()) {
    WriteDetections(results[0], absl::GetFlag(FLAGS_detections_out));
  }
  if (!absl::GetFlag(FLAGS_compare_to).empty()) {
    // Detectors sharing a process must not change each other's detections,
    // so every one of them has to match.
    bool matched = true;
    for (const DetectorResults &detector : results) {
      matched &= Compare(detector, absl::GetFlag(FLAGS_compare_to));
    }
    return matched ? 0 : 1;
  }
  return 0;
}

}  // namespace
}  // namespace frc::orin

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::orin::Main();
}

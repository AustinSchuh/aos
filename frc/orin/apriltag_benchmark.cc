// Replays a directory of CameraImage frames (see apriltag_corpus_extract)
// through GpuDetector the way apriltag_detector runs it on the robot, and
// reports how long detection takes.  It can also write out what it detected
// and compare that against an earlier run, so a change can be checked for both
// speed and for whether it changed any detections.

#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>

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
          "in it supplies the camera calibrations.");
ABSL_FLAG(int32_t, max_frames, 0, "If > 0, only replay this many frames.");
ABSL_FLAG(int32_t, iterations, 1, "Number of passes over the corpus.");
ABSL_FLAG(int32_t, warmup, 10, "Untimed detections to run before measuring.");
ABSL_FLAG(int32_t, parallel, 1,
          "Detectors to run at once, each in its own process the way the "
          "robot runs one detector per camera.  They have to be processes: "
          "the Orin can't do concurrent managed memory access, so the CPU "
          "touching managed memory while another detector's kernel runs in "
          "the same process faults.");
ABSL_FLAG(uint32_t, min_white_black_diff, 20,
          "Matches --min_white_black_diff on the robot.");
ABSL_FLAG(uint32_t, min_cluster_pixels, 25,
          "Matches --min_cluster_pixels on the robot.");
ABSL_FLAG(int32_t, nthreads, 6, "Decode threads per detector.");
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

// One frame's worth of measurements.
struct FrameTiming {
  float wall_ms;
  float gpu_ms;
  float host_decode_ms;
  std::vector<std::pair<std::string_view, float>> stages;
};

struct DetectorResults {
  std::vector<FrameTiming> timings;
  // Frame name -> formatted detections.
  std::map<std::string, std::vector<std::string>> detections;
  size_t num_detections = 0;
};

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

// Returns the calibration for each camera number in constants.bfbs, or the
// calibration the robot ships as a default when there is none.
std::map<int, Calibration> LoadCalibrations() {
  std::map<int, Calibration> result;
  const std::string path = absl::GetFlag(FLAGS_corpus) + "/constants.bfbs";
  if (std::filesystem::exists(path)) {
    aos::FlatbufferVector<vision::CameraConstants> constants =
        aos::FileToFlatbuffer<vision::CameraConstants>(path);
    if (constants.message().has_calibration()) {
      for (const vision::calibration::CameraCalibration *calibration :
           *constants.message().calibration()) {
        const auto *intrinsics = calibration->intrinsics();
        const auto *dist = calibration->dist_coeffs();
        const int num_params = dist->size();
        result[calibration->camera_number()] = Calibration{
            .camera_matrix = {.fx = intrinsics->Get(0),
                              .cx = intrinsics->Get(2),
                              .fy = intrinsics->Get(4),
                              .cy = intrinsics->Get(5)},
            .dist_coeffs = {.k1 = dist->Get(0),
                            .k2 = dist->Get(1),
                            .p1 = dist->Get(2),
                            .p2 = dist->Get(3),
                            .k3 = dist->Get(4),
                            .k4 = num_params >= 8 ? dist->Get(5) : 0.0,
                            .k5 = num_params >= 8 ? dist->Get(6) : 0.0,
                            .k6 = num_params >= 8 ? dist->Get(7) : 0.0,
                            .num_params = num_params},
        };
      }
    }
  }
  if (result.empty()) {
    ABSL_LOG(WARNING) << "No calibration in " << path
                      << ", using a made up one";
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

// Runs one detector over the corpus.  `offset` rotates where in the corpus it
// starts so parallel detectors aren't all looking at the same frame.
DetectorResults RunDetector(const std::vector<Frame> &frames,
                            const std::map<int, Calibration> &calibs,
                            size_t offset, bool record_detections) {
  const vision::CameraImage &first = frames.front().image.message();
  const size_t width = first.cols();
  const size_t height = first.rows();
  const size_t image_bytes = first.data()->size();

  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_t *tag_detector = apriltag_detector_create();
  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);
  tag_detector->nthreads = absl::GetFlag(FLAGS_nthreads);
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff =
      absl::GetFlag(FLAGS_min_white_black_diff);
  tag_detector->qtp.min_cluster_pixels =
      absl::GetFlag(FLAGS_min_cluster_pixels);

  DetectorResults results;
  {
    const Calibration &calib = CalibrationFor(calibs, frames.front().camera);
    GpuDetector detector(width, height, tag_detector, calib.camera_matrix,
                         calib.dist_coeffs, first.format());

    // On the robot the image sits in shared memory which is registered with
    // CUDA, so the GPU reads it in place.  Mapped pinned memory is the same
    // thing without the shared memory.
    uint8_t *image_host = nullptr;
    CHECK_CUDA(cudaHostAlloc(reinterpret_cast<void **>(&image_host),
                             image_bytes, cudaHostAllocMapped));
    uint8_t *image_device = nullptr;
    CHECK_CUDA(cudaHostGetDevicePointer(
        reinterpret_cast<void **>(&image_device), image_host, 0));

    const size_t total = frames.size() * absl::GetFlag(FLAGS_iterations);
    const size_t warmup = absl::GetFlag(FLAGS_warmup);
    results.timings.reserve(total);
    for (size_t i = 0; i < warmup + total; ++i) {
      const bool measuring = i >= warmup;
      const size_t pass_index = measuring ? i - warmup : i;
      const Frame &frame = frames[(pass_index + offset) % frames.size()];
      const vision::CameraImage &image = frame.image.message();
      memcpy(image_host, image.data()->data(), image_bytes);
      const Calibration &calib = CalibrationFor(calibs, frame.camera);
      detector.SetCameraMatrix(calib.camera_matrix);
      detector.SetDistortionCoefficients(calib.dist_coeffs);

      const aos::monotonic_clock::time_point start =
          aos::monotonic_clock::now();
      detector.Detect(image_host, image_device);
      const aos::monotonic_clock::time_point end = aos::monotonic_clock::now();

      if (!measuring) {
        continue;
      }
      results.timings.push_back(FrameTiming{
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
        // Decode threads finish in any order, so sort for a stable result.
        std::sort(formatted.begin(), formatted.end());
        results.detections[frame.name] = std::move(formatted);
      }
    }
    CHECK_CUDA(cudaFreeHost(image_host));
  }

  apriltag_detector_destroy(tag_detector);
  tag36h11_destroy(tag_family);
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

void Report(const std::vector<DetectorResults> &all, double elapsed_s,
            double cpu_s) {
  std::vector<float> wall, gpu, host_decode;
  // Stage name -> times, kept in first-seen order.
  std::vector<std::string_view> stage_order;
  std::map<std::string_view, std::vector<float>> stages;
  size_t frames = 0;
  size_t detections = 0;
  for (const DetectorResults &results : all) {
    detections += results.num_detections;
    for (const FrameTiming &timing : results.timings) {
      ++frames;
      wall.push_back(timing.wall_ms);
      gpu.push_back(timing.gpu_ms);
      host_decode.push_back(timing.host_decode_ms);
      for (const auto &[name, ms] : timing.stages) {
        auto [it, inserted] = stages.try_emplace(name);
        if (inserted) {
          stage_order.push_back(name);
        }
        it->second.push_back(ms);
      }
    }
  }

  std::cout << absl::StrFormat(
      "%zu frames across %zu detector(s), %zu detections, %.1f frames/s, "
      "%.3f ms CPU/frame\n",
      frames, all.size(), detections, frames / elapsed_s,
      1000.0 * cpu_s / frames);
  PrintStats("Detect() wall", wall);
  PrintStats("GPU (events)", gpu);
  PrintStats("Host decode", host_decode);
  std::cout << "  GPU stages (mean ms):\n";
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

double CpuSeconds(int who) {
  struct rusage usage;
  ABSL_PCHECK(getrusage(who, &usage) == 0);
  return usage.ru_utime.tv_sec + usage.ru_stime.tv_sec +
         1e-6 * (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec);
}

// Timings go from a child detector back to the parent as text: a "D <count>"
// line with the number of detections, then per frame a "F <wall> <gpu> <host>
// <stages>" line followed by that many "<name>\t<ms>" lines.
std::string SerializeTimings(const DetectorResults &results) {
  std::string out = absl::StrCat("D ", results.num_detections, "\n");
  for (const FrameTiming &timing : results.timings) {
    absl::StrAppend(&out, "F ", timing.wall_ms, " ", timing.gpu_ms, " ",
                    timing.host_decode_ms, " ", timing.stages.size(), "\n");
    for (const auto &[name, ms] : timing.stages) {
      absl::StrAppend(&out, name, "\t", ms, "\n");
    }
  }
  return out;
}

// `names` owns the stage names the returned string_views point at.
DetectorResults DeserializeTimings(std::string_view data,
                                   std::set<std::string, std::less<>> *names) {
  DetectorResults results;
  std::istringstream in{std::string(data)};
  std::string tag;
  while (in >> tag) {
    if (tag == "D") {
      in >> results.num_detections;
      continue;
    }
    ABSL_CHECK_EQ(tag, "F");
    FrameTiming timing;
    size_t num_stages;
    in >> timing.wall_ms >> timing.gpu_ms >> timing.host_decode_ms >>
        num_stages;
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
  return results;
}

// Runs `parallel` detectors in child processes and collects their timings.
// CUDA must not have been touched yet in this process.
std::vector<DetectorResults> RunInChildren(
    const std::vector<Frame> &frames, const std::map<int, Calibration> &calibs,
    int parallel, std::set<std::string, std::less<>> *names) {
  std::vector<std::pair<pid_t, int>> children;
  for (int i = 0; i < parallel; ++i) {
    int fds[2];
    ABSL_PCHECK(pipe(fds) == 0);
    const pid_t pid = fork();
    ABSL_PCHECK(pid >= 0);
    if (pid == 0) {
      close(fds[0]);
      const std::string data = SerializeTimings(RunDetector(
          frames, calibs, i * frames.size() / parallel, /*record=*/false));
      size_t written = 0;
      while (written < data.size()) {
        const ssize_t result =
            write(fds[1], data.data() + written, data.size() - written);
        ABSL_PCHECK(result > 0);
        written += result;
      }
      close(fds[1]);
      _exit(0);
    }
    close(fds[1]);
    children.emplace_back(pid, fds[0]);
  }

  std::vector<DetectorResults> results;
  for (const auto &[pid, fd] : children) {
    std::string data;
    char buffer[65536];
    while (true) {
      const ssize_t result = read(fd, buffer, sizeof(buffer));
      ABSL_PCHECK(result >= 0);
      if (result == 0) {
        break;
      }
      data.append(buffer, result);
    }
    close(fd);
    int status;
    ABSL_PCHECK(waitpid(pid, &status, 0) == pid);
    ABSL_CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << ": Detector process " << pid << " failed";
    results.push_back(DeserializeTimings(data, names));
  }
  return results;
}

int Main() {
  const std::vector<Frame> frames = LoadCorpus();
  const std::map<int, Calibration> calibs = LoadCalibrations();
  const vision::CameraImage &first = frames.front().image.message();
  std::cout << absl::StrFormat("Loaded %zu %dx%d %s frames\n", frames.size(),
                               first.cols(), first.rows(),
                               vision::EnumNameImageFormat(first.format()));

  const int parallel = absl::GetFlag(FLAGS_parallel);
  const bool record = !absl::GetFlag(FLAGS_detections_out).empty() ||
                      !absl::GetFlag(FLAGS_compare_to).empty();
  ABSL_CHECK(parallel == 1 || !record)
      << ": --detections_out and --compare_to need --parallel=1";
  std::vector<DetectorResults> results;
  std::set<std::string, std::less<>> stage_names;

  const double cpu_start = CpuSeconds(RUSAGE_SELF);
  const aos::monotonic_clock::time_point start = aos::monotonic_clock::now();
  if (parallel == 1) {
    results.push_back(RunDetector(frames, calibs, 0, record));
  } else {
    results = RunInChildren(frames, calibs, parallel, &stage_names);
  }
  // This includes warmup and setup, so frames/s is a lower bound.
  const double elapsed_s =
      std::chrono::duration<double>(aos::monotonic_clock::now() - start)
          .count();
  const double cpu_s = parallel == 1 ? CpuSeconds(RUSAGE_SELF) - cpu_start
                                     : CpuSeconds(RUSAGE_CHILDREN);
  Report(results, elapsed_s, cpu_s);

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
    return Compare(results[0], absl::GetFlag(FLAGS_compare_to)) ? 0 : 1;
  }
  return 0;
}

}  // namespace
}  // namespace frc::orin

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::orin::Main();
}

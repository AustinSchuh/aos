#ifndef FRC_ORIN_APRILTAG_H_
#define FRC_ORIN_APRILTAG_H_

#include <array>
#include <chrono>
#include <cub/iterator/transform_input_iterator.cuh>

#include "third_party/apriltag/apriltag.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "frc/orin/cuda.h"
#include "frc/orin/gpu_image.h"
#include "frc/orin/line_fit_filter.h"
#include "frc/orin/points.h"
#include "frc/orin/threshold.h"
#include "frc/vision/calibration_generated.h"
#include "frc/vision/vision_generated.h"

namespace frc::apriltag {

// Class to find the blob index of a point in a point vector.
class BlobExtentsIndexFinder {
 public:
  // num_extents is read when searching, so it may be a device pointer the
  // GPU fills in after this is constructed.
  BlobExtentsIndexFinder(const MinMaxExtents *extents_device,
                         const size_t *num_extents)
      : extents_device_(extents_device), num_extents_(num_extents) {}

  __host__ __device__ apriltag_size_t
  FindBlobIndex(apriltag_size_t point_index) const {
    const apriltag_size_t num_extents = *num_extents_;
    // Do a binary search for the blob which has the point in it's
    // starting_offset range.
    apriltag_size_t min = 0;
    apriltag_size_t max = num_extents;
    while (true) {
      if (min + 1 == max) {
        return min;
      }

      apriltag_size_t average = min + (max - min) / 2;
      if (average < num_extents &&
          extents_device_[average].starting_offset <= point_index) {
        min = average;
      } else {
        max = average;
      }
    }
  }

  // Returns the extents for a blob index.
  __host__ __device__ MinMaxExtents Get(apriltag_size_t index) const {
    return extents_device_[index];
  }

 private:
  const MinMaxExtents *extents_device_;
  const size_t *num_extents_;

  // TODO(austin): Cache the last one?
};

struct QuadCorners {
  float corners[4][2];
  bool reversed_border;
  uint32_t blob_index;
};

// The selected blobs with points, split by which kernel sorts them, each list
// in blob order.
struct BlobWorkList {
  uint32_t num_small_blobs;
  uint32_t num_big_blobs;
  uint16_t small_blobs[IndexPoint::kMaxBlobs];
  uint16_t big_blobs[IndexPoint::kMaxBlobs];
};

struct CameraMatrix {
  double fx;
  double cx;
  double fy;
  double cy;
};

struct DistCoeffs {
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
  double k4;
  double k5;
  double k6;
  // Are we using 5 or 8 parameter model?
  int num_params;
};

// Returns calibration's intrinsics in the forms the detector takes.
// gpu_apriltag and apriltag_benchmark both use these, so the benchmark
// undistorts the way the robot does.
inline CameraMatrix GetCameraMatrix(
    const frc::vision::calibration::CameraCalibration *calibration) {
  auto intrinsics = calibration->intrinsics();
  return CameraMatrix{
      .fx = intrinsics->Get(0),
      .cx = intrinsics->Get(2),
      .fy = intrinsics->Get(4),
      .cy = intrinsics->Get(5),
  };
}

inline DistCoeffs GetDistCoeffs(
    const frc::vision::calibration::CameraCalibration *calibration) {
  auto dist_coeffs = calibration->dist_coeffs();
  int num_params = dist_coeffs->size();
  return DistCoeffs{.k1 = dist_coeffs->Get(0),
                    .k2 = dist_coeffs->Get(1),
                    .p1 = dist_coeffs->Get(2),
                    .p2 = dist_coeffs->Get(3),
                    .k3 = dist_coeffs->Get(4),
                    .k4 = (num_params >= 8 ? dist_coeffs->Get(5) : 0.0),
                    .k5 = (num_params >= 8 ? dist_coeffs->Get(6) : 0.0),
                    .k6 = (num_params >= 8 ? dist_coeffs->Get(7) : 0.0),
                    .num_params = num_params};
}

// GPU based april tag detector.
class GpuDetector {
 public:
  // The number of blobs we will consider when counting april tags.
  static constexpr size_t kMaxBlobs = IndexPoint::kMaxBlobs;

  // Constructs a detector, reserving space for detecting tags of the provided
  // with and height, using the provided detector options.
  GpuDetector(size_t width, size_t height, apriltag_detector_t *tag_detector,
              CameraMatrix camera_matrix, DistCoeffs distortion_coefficients,
              vision::ImageFormat image_format);
  virtual ~GpuDetector();

  size_t width() const { return width_; }
  size_t height() const { return height_; }

  // Detects april tags in the provided image.
  void Detect(const uint8_t *image, const uint8_t *image_device);

  const std::vector<QuadCorners> &FitQuads() const;

  const zarray_t *Detections() const { return detections_; }

  // Records a CUDA event between every GPU stage of Detect() so StageTimesMs()
  // can report them.  Off by default because each record costs launch-path CPU
  // time and the report adds to Detect()'s latency.
  void set_stage_timing(bool stage_timing) {
    stage_timing_ = stage_timing;
    stage_times_ms_.clear();
  }

  // Time each stage of the last Detect() took, in pipeline order, if stage
  // timing is on.  For benchmarking; the names may change as the pipeline
  // does.  "NEON threshold (host)" is host time, "Memset" runs on another
  // stream, and the "UF" stages split "Unionfinding"; the rest sum to
  // GpuTimeMs().
  const std::vector<std::pair<std::string_view, float>> &StageTimesMs() const {
    return stage_times_ms_;
  }
  // GPU time of the last Detect(), from the first event on the main stream to
  // the end of the last kernel.
  float GpuTimeMs() const { return gpu_time_ms_; }
  // Host time spent after the GPU finished (quad filtering and decoding) in
  // the last Detect().
  float HostDecodeTimeMs() const { return host_decode_time_ms_; }

  // How long Detect() polls each GPU event before it sleeps on it.  Polling
  // saves the tens of microseconds a blocking wait takes to wake up, and burns
  // a core for as long as it polls.  Defaults to 0, sleeping right away.
  void set_host_wait_spin(std::chrono::nanoseconds host_wait_spin) {
    host_wait_spin_ = host_wait_spin;
  }

  // True if the last Detect() found more than kMaxBlobs blob pairs and dropped
  // the frame without detecting anything.
  bool last_frame_overflowed() const { return last_frame_overflowed_; }

  // Debug methods to expose internal state for testing.
  void CopyGrayTo(uint8_t *output) const {
    gray_image_device_.MemcpyTo(output);
  }
  void CopyDecimatedTo(uint8_t *output) const {
    decimated_image_device_.MemcpyTo(output);
  }
  void CopyThresholdedTo(uint8_t *output) const {
    thresholded_image_device_.MemcpyTo(output);
  }
  void CopyUnionMarkersTo(uint32_t *output) const {
    union_markers_device_.MemcpyTo(output);
  }

  void CopyUnionMarkerPairTo(QuadBoundaryPoint *output) const {
    union_marker_pair_device_.MemcpyTo(output);
  }

  void CopyCompressedUnionMarkerPairTo(QuadBoundaryPoint *output) const {
    compressed_union_marker_pair_device_.MemcpyTo(output);
  }

  std::vector<QuadBoundaryPoint> CopySortedUnionMarkerPair() const {
    std::vector<QuadBoundaryPoint> result;
    int size = NumCompressedUnionMarkerPairs();
    result.resize(size);
    sorted_union_marker_pair_device_.MemcpyTo(result.data(), size);
    return result;
  }

  int NumCompressedUnionMarkerPairs() const {
    return *num_compressed_union_marker_pair_.host_ptr();
  }

  void CopyUnionMarkersSizeTo(uint32_t *output) const {
    union_markers_size_device_.MemcpyTo(output);
  }

  int NumQuads() const { return num_quads_device_.Copy()[0]; }

  std::vector<MinMaxExtents> CopyExtents() const {
    return extents_device_.Copy(NumQuads());
  }

  std::vector<cub::KeyValuePair<long, MinMaxExtents>> CopySelectedExtents()
      const {
    return selected_extents_device_.Copy(NumQuads());
  }

  int NumSelectedPairs() const { return num_selected_blobs_device_.Copy()[0]; }

  std::vector<IndexPoint> CopySelectedBlobs() const {
    return selected_blobs_device_.Copy(NumSelectedPairs());
  }

  std::vector<IndexPoint> CopySortedSelectedBlobs() const {
    return sorted_selected_blobs_device_.Copy(NumSelectedPairs());
  }

  std::vector<LineFitPoint> CopyLineFitPoints() const {
    return line_fit_points_device_.Copy(NumSelectedPairs());
  }

  std::vector<double> CopyErrors() const {
    return errs_device_.Copy(NumSelectedPairs());
  }

  std::vector<double> CopyFilteredErrors() const {
    return filtered_errs_device_.Copy(NumSelectedPairs());
  }
  std::vector<Peak> CopyPeaks() const {
    return filtered_is_local_peak_device_.Copy(NumSelectedPairs());
  }

  // The number of valid quads the GPU fit.
  int NumFitQuads() const { return CopyFitQuads().size(); }

  // The valid quads the GPU fit, in blob order.
  std::vector<FitQuad> CopyFitQuads() const;

  void AdjustCenter(float corners[4][2]) const;

  // TODO(max): We probably don't want to use these after our test images are
  // just orin images
  void SetCameraMatrix(CameraMatrix camera_matrix) {
    camera_matrix_ = camera_matrix;
  }

  void SetDistortionCoefficients(DistCoeffs distortion_coefficients) {
    distortion_coefficients_ = distortion_coefficients;
  }

  // (Re-)distort pixels based on our camera model, using pinhole camera model
  static void ReDistort(double *u, double *v, const CameraMatrix *camera_matrix,
                        const DistCoeffs *distortion_coefficients);

  // Undistort pixels based on our camera model, using iterative algorithm
  // Returns false if we fail to converge
  static bool UnDistort(double *u, double *v, const CameraMatrix *camera_matrix,
                        const DistCoeffs *distortion_coefficients);

 private:
  void UpdateFitQuads();

  void AdjustPixelCenters();

  void DecodeTags();

  static void QuadDecodeTask(void *_u);

  // Creates a GPU image wrapped around the provided memory.
  template <typename T>
  GpuImage<T> ToGpuImage(GpuMemory<T> &memory) {
    if (memory.size() == width_ * height_) {
      return GpuImage<T>{
          .data = memory.get(),
          .rows = height_,
          .cols = width_,
          .step = width_,
      };
    } else if (memory.size() == width_ * height_ / 4) {
      return GpuImage<T>{
          .data = memory.get(),
          .rows = height_ / 2,
          .cols = width_ / 2,
          .step = width_ / 2,
      };
    } else {
      ABSL_LOG(FATAL) << "Unknown image shape";
    }
  }

  template <typename T>
  GpuImage<T> ToGpuImage(UnifiedMemory<T> &memory) {
    if (memory.size() == width_ * height_) {
      return GpuImage<T>{
          .data = memory.get(),
          .rows = height_,
          .cols = width_,
          .step = width_,
      };
    } else if (memory.size() == width_ * height_ / 4) {
      return GpuImage<T>{
          .data = memory.get(),
          .rows = height_ / 2,
          .cols = width_ / 2,
          .step = width_ / 2,
      };
    } else {
      ABSL_LOG(FATAL) << "Unknown image shape";
    }
  }

  // Size of the image.
  const size_t width_;
  const size_t height_;

  // Detector parameters.
  apriltag_detector_t *tag_detector_;

  // Stream to operate on.
  CudaStream stream_;
  CudaStream memset_stream_;

  // Events between the steps.  Detect() always records start_ and the events
  // it waits on, and records the rest only for stage timing.
  //
  // Every event the host waits on is a cudaEventBlockingSync event.  Sleeping
  // there instead of spinning frees the core while the GPU is busy, which
  // matters most when other processes hold the GPU.
  CudaEvent start_;
  CudaEvent after_image_memcpy_to_device_;
  CudaEvent after_threshold_;
  CudaEvent after_memcpy_gray_;
  CudaEvent memset_start_;
  CudaEvent after_memset_{cudaEventBlockingSync};
  CudaEvent before_unionfinding_;
  // After each of LabelImage's kernels.
  std::array<CudaEvent, 5> after_labeling_kernel_;
  CudaEvent after_unionfinding_;
  CudaEvent after_diff_;
  CudaEvent after_compact_{cudaEventBlockingSync};
  CudaEvent after_sort_;
  CudaEvent after_bounds_;
  CudaEvent after_transform_extents_;
  CudaEvent after_filter_;
  CudaEvent after_small_blob_sort_;
  CudaEvent after_big_blob_sort_;
  CudaEvent after_line_filter_;
  CudaEvent after_quad_fit_{cudaEventBlockingSync};

  // A stage's time is from begin to end.
  struct Stage {
    std::string_view name;
    CudaEvent *begin;
    CudaEvent *end;
  };
  // The stages StageTimesMs() reports, in order.
  const std::array<Stage, 21> stages_{{
      {"Memset", &memset_start_, &after_memset_},
      {"Memcpy", &start_, &after_image_memcpy_to_device_},
      {"Threshold", &after_image_memcpy_to_device_, &after_threshold_},
      {"Memcpy Gray", &after_threshold_, &after_memcpy_gray_},
      {"Wait memset", &after_memcpy_gray_, &before_unionfinding_},
      {"Unionfinding", &before_unionfinding_, &after_unionfinding_},
      {"UF Init", &before_unionfinding_, &after_labeling_kernel_[0]},
      {"UF C1", &after_labeling_kernel_[0], &after_labeling_kernel_[1]},
      {"UF Merge", &after_labeling_kernel_[1], &after_labeling_kernel_[2]},
      {"UF C2", &after_labeling_kernel_[2], &after_labeling_kernel_[3]},
      {"UF Final", &after_labeling_kernel_[3], &after_labeling_kernel_[4]},
      {"Diff", &after_unionfinding_, &after_diff_},
      {"Compact", &after_diff_, &after_compact_},
      {"Sort", &after_compact_, &after_sort_},
      {"Bounds", &after_sort_, &after_bounds_},
      {"Transform Extents", &after_bounds_, &after_transform_extents_},
      {"Filter by dot product", &after_transform_extents_, &after_filter_},
      {"Small blob sort", &after_filter_, &after_small_blob_sort_},
      {"Big blob sort", &after_small_blob_sort_, &after_big_blob_sort_},
      {"Error Filter", &after_big_blob_sort_, &after_line_filter_},
      {"FitQuads", &after_line_filter_, &after_quad_fit_},
  }};

  // TODO(austin): Remove this...
  HostMemory<uint8_t> gray_image_host_;
  const uint8_t *gray_image_host_ptr_;

  // Starting color image.
  GpuMemory<uint8_t> color_image_device_;
  // Full size gray scale image.
  GpuMemory<uint8_t> gray_image_device_;
  // decimated_image_device_ and thresholded_image_device_ are attached to
  // stream_.  Every kernel that touches them runs on stream_, and the CPU
  // touches them only while stream_ is idle: the NEON threshold at the start of
  // Detect() and the Copy*To methods after it returns.
  //
  // Half resolution, gray, decimated image.
  UnifiedMemory<uint8_t> decimated_image_device_;
  // Intermediates for thresholding.
  UnifiedMemory<uint8_t> thresholded_image_device_;

  // The union markers for each pixel.
  GpuMemory<uint32_t> union_markers_device_;
  // The size of each blob.  The blob size is stored at the index of the stored
  // union marker id in union_markers_device_ aboe.
  GpuMemory<uint32_t> union_markers_size_device_;

  // Full list of boundary points, densly stored but mostly zero.
  GpuMemory<QuadBoundaryPoint> union_marker_pair_device_;
  // Unsorted list of points with 0's removed.
  GpuMemory<QuadBoundaryPoint> compressed_union_marker_pair_device_;
  // Blob representation sorted list of points.
  GpuMemory<QuadBoundaryPoint> sorted_union_marker_pair_device_;
  // Number of compressed points.  Mapped so the host reads the count straight
  // from where the compaction writes it.
  MappedHostMemory<int> num_compressed_union_marker_pair_{1};

  // Number of unique blob IDs.  Only the GPU reads it during Detect().
  GpuMemory<size_t> num_quads_device_{/* allocate 1 integer...*/ 1};
  // Bounds per blob, one blob per ID.
  GpuMemory<MinMaxExtents> extents_device_;
  // Extents of all the blobs under consideration.
  GpuMemory<cub::KeyValuePair<long, MinMaxExtents>> selected_extents_device_;

  // Number of keys in selected_blobs_device_.  Only the GPU reads it during
  // Detect().
  GpuMemory<int> num_selected_blobs_device_{/* allocate 1 integer...*/ 1};

  // Compacted blobs which pass our threshold.
  GpuMemory<IndexPoint> selected_blobs_device_;
  // Which selected blobs each sorting kernel handles.
  GpuMemory<BlobWorkList> blob_work_list_device_{1};
  // Sorted list of those points.
  GpuMemory<IndexPoint> sorted_selected_blobs_device_;

  // TODO(austin): Can we bound this better?  This is a lot of memory.
  GpuMemory<LineFitPoint> line_fit_points_device_;

  GpuMemory<double> errs_device_;
  GpuMemory<double> filtered_errs_device_;
  GpuMemory<Peak> filtered_is_local_peak_device_;

  CameraMatrix camera_matrix_;
  DistCoeffs distortion_coefficients_;

  // The quad fit for each blob, and whether it is valid.  Mapped so the host
  // reads them where the GPU writes them.
  MappedHostMemory<FitQuad> fit_quads_host_{kMaxBlobs};
  MappedHostMemory<bool> fit_quad_valid_host_{kMaxBlobs};
  std::vector<QuadCorners> quad_corners_host_;

  // Temporary storage for each of the steps.
  // TODO(austin): Can we combine these and just use the max?
  GpuMemory<uint32_t> radix_sort_tmpstorage_device_;
  GpuMemory<uint8_t> temp_storage_compressed_union_marker_pair_device_;
  GpuMemory<uint8_t> temp_storage_bounds_reduce_by_key_device_;
  GpuMemory<uint8_t> temp_storage_compressed_filtered_blobs_device_;
  GpuMemory<uint8_t> temp_storage_selected_extents_scan_device_;

  std::unique_ptr<Threshold> threshold_;

  // Cumulative duration of april tag detection.
  std::chrono::nanoseconds execution_duration_{0};
  // Number of detections.
  size_t execution_count_ = 0;
  // True if this is the first detection.
  bool first_ = true;

  bool stage_timing_ = false;
  std::chrono::nanoseconds host_wait_spin_{0};
  std::vector<std::pair<std::string_view, float>> stage_times_ms_;
  float gpu_time_ms_ = 0;
  float host_decode_time_ms_ = 0;
  bool last_frame_overflowed_ = false;

  // What the GPU reports about the frame, for the host to read after the last
  // wait in Detect().
  struct FrameStatus {
    // Nonzero if there were more than kMaxBlobs blob pairs.
    uint32_t overflow;
    // The number of blob pairs.
    uint32_t num_blob_pairs;
  };
  MappedHostMemory<FrameStatus> frame_status_{1};

  // Cached quantities used for tag filtering.
  bool normal_border_ = false;
  bool reversed_border_ = false;
  int min_tag_width_ = 1000000;

  zarray_t *poly0_;
  zarray_t *poly1_;

  zarray_t *detections_ = nullptr;

  const vision::ImageFormat image_format_;
};

}  // namespace frc::apriltag

#endif  // FRC_ORIN_APRILTAG_H_

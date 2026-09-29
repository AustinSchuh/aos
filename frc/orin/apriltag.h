#ifndef FRC_ORIN_APRILTAG_H_
#define FRC_ORIN_APRILTAG_H_

#include <array>
#include <chrono>
#include <cub/iterator/transform_input_iterator.cuh>
#include <optional>

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

// The pairs of blobs with points between them, in an open addressing hash
// table keyed by rep01(), and the order the kernels after BinBlobPairs put
// them in.  Between frames, every slot is empty and every count is zero.
struct BlobPairTable {
  // Twice as many as fit downstream, so a frame which fits always has room.
  static constexpr uint32_t kSlotBits = 12;
  static constexpr uint32_t kSlots = 1 << kSlotBits;
  static_assert(kSlots == 2 * IndexPoint::kMaxBlobs);

  // Each slot's rep01(), or 0 when it is empty.  No pair has a rep01() of 0,
  // since rep1 is bigger than rep0.
  unsigned long long keys[kSlots];
  // The same sums of each pair's points as MinMaxExtents holds.
  unsigned long long pxgx_plus_pygy_sums[kSlots];
  uint32_t counts[kSlots];
  uint32_t min_xs[kSlots];
  uint32_t max_xs[kSlots];
  uint32_t min_ys[kSlots];
  uint32_t max_ys[kSlots];
  int32_t gx_sums[kSlots];
  int32_t gy_sums[kSlots];

  // The first kMaxBlobs pairs, in the order BinBlobPairs claimed their slots,
  // as rep01() << kSlotBits | slot.
  uint64_t pairs[IndexPoint::kMaxBlobs];
  // Each pair's rank by rep01(), which is its blob index, by slot.
  uint16_t slot_ranks[kSlots];
  // How many of each blob's selected points have been placed.
  uint32_t cursors[IndexPoint::kMaxBlobs];

  // The number of pairs BinBlobPairs found, and of points BlobDiff found.
  uint32_t num_pairs;
  uint32_t num_points;
  // Nonzero if BinBlobPairs could not fit every pair.
  uint32_t overflow;
  // The frame's point count and whether it overflowed, for the kernels after
  // BlobPairExtents, which zeros the counts above for the next frame.
  uint32_t frame_num_points;
  uint32_t frame_overflowed;
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
  // time and the report adds to Detect()'s latency, and because it launches
  // each stage on its own instead of as one CUDA graph.
  void set_stage_timing(bool stage_timing) {
    stage_timing_ = stage_timing;
    stage_times_ms_.clear();
  }

  // Time each stage of the last Detect() took, in pipeline order, if stage
  // timing is on.  For benchmarking; the names may change as the pipeline
  // does.  "NEON threshold (host)" is host time, and the "UF" stages split
  // "Unionfinding"; the rest sum to GpuTimeMs().
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

  // The points between blobs, in no particular order.  Only valid for frames
  // which did not overflow.
  std::vector<QuadBoundaryPoint> CopyBoundaryPoints() const;

  int NumBoundaryPoints() const {
    return frame_status_.host_ptr()->num_boundary_points;
  }

  void CopyUnionMarkersSizeTo(uint32_t *output) const {
    union_markers_size_device_.MemcpyTo(output);
  }

  // The number of blob pairs.  Past kMaxBlobs, the GPU stops counting some of
  // them.
  int NumQuads() const { return frame_status_.host_ptr()->num_blob_pairs; }

  std::vector<MinMaxExtents> CopyExtents() const {
    return extents_device_.Copy(std::min<size_t>(NumQuads(), kMaxBlobs));
  }

  std::vector<cub::KeyValuePair<long, MinMaxExtents>> CopySelectedExtents()
      const {
    return selected_extents_device_.Copy(
        std::min<size_t>(NumQuads(), kMaxBlobs));
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

  // Moves quad's corners onto the strongest gradients in image near its
  // edges, fitting each edge's line in undistorted pixels.  td->quad_decimate
  // must be 2.
  static void RefineEdges(const apriltag_detector_t *td,
                          const image_u8_t *image, struct quad *quad,
                          const CameraMatrix *camera_matrix,
                          const DistCoeffs *distortion_coefficients);

 private:
  // Enqueues everything from labeling the thresholded image to fitting quads
  // on stream_, recording the stage events if record_stages.
  void EnqueueQuadFitting(bool record_stages);

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
  // After each of LabelImage's kernels.
  std::array<CudaEvent, 3> after_labeling_kernel_;
  CudaEvent after_unionfinding_;
  CudaEvent after_diff_;
  CudaEvent after_bin_pairs_;
  CudaEvent after_rank_pairs_;
  CudaEvent after_pair_extents_;
  CudaEvent after_scatter_;
  CudaEvent after_small_blob_sort_;
  CudaEvent after_big_blob_sort_;
  CudaEvent after_line_filter_;
  CudaEvent after_quad_fit_{cudaEventBlockingSync};

  // EnqueueQuadFitting(false), captured, unless --sync is set.  Launching it
  // submits the frame's GPU work after thresholding in one call.
  std::optional<CudaGraph> quad_fitting_graph_;

  // A stage's time is from begin to end.
  struct Stage {
    std::string_view name;
    CudaEvent *begin;
    CudaEvent *end;
  };
  // The stages StageTimesMs() reports, in order.
  const std::array<Stage, 16> stages_{{
      {"Memcpy", &start_, &after_image_memcpy_to_device_},
      {"Threshold", &after_image_memcpy_to_device_, &after_threshold_},
      {"Memcpy Gray", &after_threshold_, &after_memcpy_gray_},
      {"Unionfinding", &after_memcpy_gray_, &after_unionfinding_},
      {"UF Tile", &after_memcpy_gray_, &after_labeling_kernel_[0]},
      {"UF Border", &after_labeling_kernel_[0], &after_labeling_kernel_[1]},
      {"UF Final", &after_labeling_kernel_[1], &after_labeling_kernel_[2]},
      {"Diff", &after_unionfinding_, &after_diff_},
      {"Bin pairs", &after_diff_, &after_bin_pairs_},
      {"Rank pairs", &after_bin_pairs_, &after_rank_pairs_},
      {"Pair extents", &after_rank_pairs_, &after_pair_extents_},
      {"Scatter selected", &after_pair_extents_, &after_scatter_},
      {"Small blob sort", &after_scatter_, &after_small_blob_sort_},
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
  // stream_.  Every kernel that touches them runs on stream_, including those
  // in quad_fitting_graph_, which launches there, and the CPU touches them only
  // while stream_ is idle: the NEON threshold at the start of Detect() and the
  // Copy*To methods after it returns.
  //
  // Half resolution, gray, decimated image.
  UnifiedMemory<uint8_t> decimated_image_device_;
  // Intermediates for thresholding.
  UnifiedMemory<uint8_t> thresholded_image_device_;

  // The union markers for each pixel.
  GpuMemory<uint32_t> union_markers_device_;
  // The size of each blob.  The blob size is stored at the index of the stored
  // union marker id in union_markers_device_ aboe.  Entries no union marker
  // points to hold whatever an earlier frame left there.
  GpuMemory<uint32_t> union_markers_size_device_;
  // LabelImage's scratch: each 2x2 block's roots within its tile, and the
  // union-find forest joining the tiles.
  GpuMemory<uint32_t> label_tile_roots_device_;
  GpuMemory<uint32_t> label_forest_device_;

  // The points between blobs, in the order BlobDiff found them.  BlobDiff
  // writes each one's key, and BinBlobPairs replaces it with its blob pair's
  // slot in blob_pairs_device_ << 32 | its point_bits().
  GpuMemory<uint64_t> boundary_points_device_;
  GpuMemory<BlobPairTable> blob_pairs_device_{1};

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
    // The number of points between blobs.
    uint32_t num_boundary_points;
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

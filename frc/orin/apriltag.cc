#include "frc/orin/apriltag.h"

#include <cub/block/block_scan.cuh>
#include <cuda/atomic>
#include <numbers>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "third_party/apriltag/common/g2d.h"

#include "aos/time/time.h"
#include "frc/orin/labeling_allegretti_2019_BKE.h"
#include "frc/orin/threshold.h"

namespace numbers = std::numbers;

ABSL_FLAG(bool, use_neon, false, "Use NEON optimized threshold");
ABSL_DECLARE_FLAG(bool, sync);

namespace frc::apriltag {
namespace {

// Returns BytesPerPixel(image_format), after checking for the formats it traps
// on, since a trap on the host doesn't say which format it was.
apriltag_size_t HostBytesPerPixel(vision::ImageFormat image_format) {
  ABSL_CHECK(image_format == vision::ImageFormat::MONO8 ||
             image_format == vision::ImageFormat::MONO16 ||
             image_format == vision::ImageFormat::YUYV422 ||
             image_format == vision::ImageFormat::BGR8 ||
             image_format == vision::ImageFormat::BGRA8)
      << ": Unknown image format: " << vision::EnumNameImageFormat(image_format)
      << " (" << static_cast<int>(image_format) << ")";
  return BytesPerPixel(image_format);
}

typedef std::chrono::duration<float, std::milli> float_milli;

}  // namespace

GpuDetector::GpuDetector(size_t width, size_t height,
                         apriltag_detector_t *tag_detector,
                         CameraMatrix camera_matrix,
                         DistCoeffs distortion_coefficients,
                         vision::ImageFormat image_format,
                         bool keep_intermediates)
    : width_(width),
      height_(height),
      tag_detector_(tag_detector),
      use_neon_(absl::GetFlag(FLAGS_use_neon)),
      keep_intermediates_(keep_intermediates),
      gray_image_host_(width * height),
      color_image_device_(HostBytesPerPixel(image_format) * width * height),
      gray_image_device_(width * height),
      union_markers_device_(width / 2 * height / 2),
      union_markers_size_device_(width / 2 * height / 2),
      label_tile_roots_device_(width / 4 * height / 4),
      label_forest_device_(width / 2 * height / 2),
      label_roots_device_(3 * label_tile_roots_device_.size()),
      // Up to 4 points for each pixel off the image's edge.
      boundary_points_device_((width / 2 - 2) * (height / 2 - 2) * 4),
      extents_device_(kMaxBlobs),
      selected_extents_device_(kMaxBlobs),
      selected_blobs_device_(boundary_points_device_.size()),
      sorted_selected_blobs_device_(selected_blobs_device_.size()),
      line_fit_points_device_(selected_blobs_device_.size()),
      errs_device_(line_fit_points_device_.size()),
      filtered_errs_device_(line_fit_points_device_.size()),
      filtered_is_local_peak_device_(line_fit_points_device_.size()),
      camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients),
      threshold_(use_neon_ ? MakeNeonThreshold(image_format, width, height)
                           : MakeThreshold(image_format, width, height)),
      decode_pool_(tag_detector->nthreads),
      image_format_(image_format) {
  quad_corners_host_.reserve(kMaxBlobs);

  // The image size limits in apriltag.h.
  ABSL_CHECK(width % 8 == 0 && height % 8 == 0)
      << ": " << width << "x" << height
      << " image: width and height must be multiples of 8 for the threshold's "
         "4x4 tiles of the decimated image";
  ABSL_CHECK(width <= 2048 && height <= 2048)
      << ": " << width << "x" << height
      << " image: width and height must be at most 2048, since boundary points "
         "hold decimated coordinates in 10 bits";
  ABSL_CHECK_LT(width * height, size_t{1} << 22)
      << ": " << width << "x" << height
      << " image: width * height must be under 2^22, since boundary points "
         "hold blob labels, which are decimated pixel indices, in 20 bits";
  ABSL_CHECK_EQ(tag_detector_->quad_decimate, 2);
  ABSL_CHECK(!tag_detector_->qtp.deglitch);
  // The NEON threshold reads one byte per pixel from the host image.
  ABSL_CHECK(!use_neon_ || image_format == vision::ImageFormat::MONO8)
      << ": --use_neon only supports MONO8 images";

  const size_t decimated_size = width / 2 * height / 2;
  if (use_neon_) {
    decimated_image_unified_.emplace(decimated_size);
    thresholded_image_unified_.emplace(decimated_size);
    decimated_image_device_ = decimated_image_unified_->get();
    thresholded_image_device_ = thresholded_image_unified_->get();
    // The Orin has no concurrent managed access, so the CPU may only touch
    // globally attached managed memory while no stream in the process has
    // work in flight.  Attached to stream_, these only need stream_ to be
    // idle.
    CHECK_CUDA(cudaStreamAttachMemAsync(stream_.get(), decimated_image_device_,
                                        0, cudaMemAttachSingle));
    CHECK_CUDA(cudaStreamAttachMemAsync(
        stream_.get(), thresholded_image_device_, 0, cudaMemAttachSingle));
  } else {
    decimated_image_gpu_.emplace(decimated_size);
    thresholded_image_gpu_.emplace(decimated_size);
    decimated_image_device_ = decimated_image_gpu_->get();
    thresholded_image_device_ = thresholded_image_gpu_->get();
  }
  // Every frame leaves the blob pair table empty for the next, with each
  // slot's minimums at their largest.
  char *blob_pairs = reinterpret_cast<char *>(blob_pairs_device_.get());
  CHECK_CUDA(
      cudaMemsetAsync(blob_pairs, 0, sizeof(BlobPairTable), stream_.get()));
  for (size_t offset :
       {offsetof(BlobPairTable, min_xs), offsetof(BlobPairTable, min_ys)}) {
    CHECK_CUDA(cudaMemsetAsync(blob_pairs + offset, 0xff,
                               sizeof(BlobPairTable::min_xs), stream_.get()));
  }
  // Every frame leaves LabelImage's list of roots empty for the next.
  CHECK_CUDA(cudaMemsetAsync(label_num_roots_device_.get(), 0, sizeof(uint32_t),
                             stream_.get()));
  // selected_extents_device_ starts out uninitialized, so the first frame
  // writes all of it.
  const uint32_t num_nonempty_selected_extents = kMaxBlobs;
  CHECK_CUDA(cudaMemcpyAsync(
      blob_pairs + offsetof(BlobPairTable, num_nonempty_selected_extents),
      &num_nonempty_selected_extents, sizeof(num_nonempty_selected_extents),
      cudaMemcpyHostToDevice, stream_.get()));
  CHECK_CUDA(cudaStreamSynchronize(stream_.get()));

  for (int i = 0; i < zarray_size(tag_detector_->tag_families); i++) {
    apriltag_family_t *family;
    zarray_get(tag_detector_->tag_families, i, &family);
    if (family->width_at_border < min_tag_width_) {
      min_tag_width_ = family->width_at_border;
    }
    normal_border_ |= !family->reversed_border;
    reversed_border_ |= family->reversed_border;
  }
  min_tag_width_ /= tag_detector_->quad_decimate;
  if (min_tag_width_ < 3) {
    min_tag_width_ = 3;
  }

  poly0_ = g2d_polygon_create_zeros(4);
  poly1_ = g2d_polygon_create_zeros(4);

  detections_ = zarray_create(sizeof(apriltag_detection_t *));
  zarray_ensure_capacity(detections_, kMaxBlobs);
  const int num_families = zarray_size(tag_detector_->tag_families);
  for (int i = 0; i < decode_pool_.threads(); ++i) {
    zarray_t *detections = zarray_create(sizeof(apriltag_detection_t *));
    zarray_ensure_capacity(detections, num_families);
    decode_thread_detections_.push_back(detections);
  }
  quad_detections_.reserve(kMaxBlobs * num_families);

  // Everything from labeling to the quad fit has the same shape every frame,
  // so capture it once, along with the GPU MONO8 threshold, which reads the
  // image address when it runs.  That fixes tag_detector_'s threshold and quad
  // parameters at their values now.  --sync synchronizes between kernels,
  // which a graph can't do.
  if (!absl::GetFlag(FLAGS_sync)) {
    frame_graph_thresholds_ =
        !use_neon_ && image_format_ == vision::ImageFormat::MONO8;
    frame_graph_.emplace(
        &stream_,
        [this]() {
          if (frame_graph_thresholds_) {
            threshold_->ThresholdAndDecimateIndirect(
                image_address_.device_ptr(), decimated_image_device_,
                thresholded_image_device_,
                tag_detector_->qtp.min_white_black_diff, &stream_);
          }
          EnqueueQuadFitting(/*record_stages=*/false, /*fork_big_blobs=*/true);
        },
        /*use_node_priority=*/true);
    // The big-blob quad fit's overlap with the small-blob quad fits rests on
    // its priority, and nothing else would notice it going missing.
    const std::vector<int> big_blob_priorities =
        BigBlobQuadFitPriorities(*frame_graph_);
    ABSL_CHECK_EQ(big_blob_priorities.size(), 1u);
    ABSL_CHECK_EQ(big_blob_priorities[0], big_blob_priority_);
  }
}

GpuDetector::~GpuDetector() {
  for (int i = 0; i < zarray_size(detections_); ++i) {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);
    apriltag_detection_destroy(det);
  }

  zarray_destroy(detections_);
  for (zarray_t *detections : decode_thread_detections_) {
    zarray_destroy(detections);
  }
  zarray_destroy(poly1_);
  zarray_destroy(poly0_);
}

std::vector<QuadBoundaryPoint> GpuDetector::CopyBoundaryPoints() const {
  // The pairs' keys are gone from the table, but not from their list.
  const std::vector<BlobPairTable> table = blob_pairs_device_.Copy();
  const uint64_t *pairs = table[0].pairs;
  std::vector<uint64_t> rep01s(BlobPairTable::kSlots, 0);
  for (size_t i = 0; i < std::min<size_t>(NumQuads(), kMaxBlobs); ++i) {
    rep01s[pairs[i] & (BlobPairTable::kSlots - 1)] =
        pairs[i] >> BlobPairTable::kSlotBits;
  }
  std::vector<QuadBoundaryPoint> result;
  for (const uint64_t point :
       boundary_points_device_.Copy(NumBoundaryPoints())) {
    QuadBoundaryPoint boundary_point;
    boundary_point.key =
        (rep01s[point >> 32] << QuadBoundaryPoint::kRepEndBit) |
        (point & 0xffffff);
    result.push_back(boundary_point);
  }
  return result;
}

namespace {

// Returns the slot a blob pair's search starts at in a BlobPairTable.
__device__ __forceinline__ uint32_t BlobPairHash(uint64_t rep01) {
  return (rep01 * 0x9e3779b97f4a7c15ull) >> (64 - BlobPairTable::kSlotBits);
}

// Swaps key from 0 to rep01 if it is 0, and returns what it was.  atomicCAS is
// sequentially consistent at system scope under clang, which fences all of
// memory every time, and nothing here needs more than a relaxed swap.
__device__ __forceinline__ unsigned long long SwapIfEmpty(
    unsigned long long &key, unsigned long long rep01) {
  unsigned long long old_key = 0;
  cuda::atomic_ref<unsigned long long, cuda::thread_scope_device>(key)
      .compare_exchange_strong(old_key, rep01, cuda::memory_order_relaxed);
  return old_key;
}

// Returns the slot holding rep01 in pairs, claiming an empty one if none does,
// or -1 if the frame overflows first.
__device__ int FindBlobPair(BlobPairTable *pairs, uint64_t rep01) {
  uint32_t slot = BlobPairHash(rep01);
  for (uint32_t probe = 0; probe < BlobPairTable::kSlots; ++probe) {
    const unsigned long long key = SwapIfEmpty(pairs->keys[slot], rep01);
    if (key == rep01) {
      return slot;
    }
    if (key == 0) {
      const uint32_t index = atomicAdd(&pairs->num_pairs, 1);
      if (index < GpuDetector::kMaxBlobs) {
        pairs->pairs[index] = (rep01 << BlobPairTable::kSlotBits) | slot;
      } else {
        atomicOr(&pairs->overflow, 1);
      }
      return slot;
    }
    // Nothing reads the table of a frame which overflows, so stop rather than
    // search through a full table.
    if (probe % 8 == 7 &&
        *static_cast<volatile uint32_t *>(&pairs->overflow) != 0) {
      return -1;
    }
    slot = (slot + 1) % BlobPairTable::kSlots;
  }
  atomicOr(&pairs->overflow, 1);
  return -1;
}

// Adds some of a blob pair's points to its slot in pairs.
__device__ __forceinline__ void AddToBlobPair(BlobPairTable *pairs,
                                              uint32_t slot, uint32_t count,
                                              uint32_t min_x, uint32_t max_x,
                                              uint32_t min_y, uint32_t max_y,
                                              int32_t gx_sum, int32_t gy_sum,
                                              int32_t pxgx_plus_pygy_sum) {
  atomicAdd(&pairs->counts[slot], count);
  atomicMin(&pairs->min_xs[slot], min_x);
  atomicMax(&pairs->max_xs[slot], max_x);
  atomicMin(&pairs->min_ys[slot], min_y);
  atomicMax(&pairs->max_ys[slot], max_y);
  if (gx_sum != 0) {
    atomicAdd(&pairs->gx_sums[slot], gx_sum);
  }
  if (gy_sum != 0) {
    atomicAdd(&pairs->gy_sums[slot], gy_sum);
  }
  if (pxgx_plus_pygy_sum != 0) {
    // Adding the sign extended sum unsigned is the same as adding it signed.
    atomicAdd(&pairs->pxgx_plus_pygy_sums[slot],
              static_cast<unsigned long long>(
                  static_cast<int64_t>(pxgx_plus_pygy_sum)));
  }
}

// Returns the sum, minimum, and maximum of value over the warp.
__device__ __forceinline__ uint32_t WarpSum(uint32_t value) {
#if __CUDA_ARCH__ >= 800
  return __reduce_add_sync(0xffffffffu, value);
#else
  for (int i = 16; i > 0; i /= 2) {
    value += __shfl_xor_sync(0xffffffffu, value, i);
  }
  return value;
#endif
}
__device__ __forceinline__ uint32_t WarpMin(uint32_t value) {
#if __CUDA_ARCH__ >= 800
  return __reduce_min_sync(0xffffffffu, value);
#else
  for (int i = 16; i > 0; i /= 2) {
    value = min(value, __shfl_xor_sync(0xffffffffu, value, i));
  }
  return value;
#endif
}
__device__ __forceinline__ uint32_t WarpMax(uint32_t value) {
#if __CUDA_ARCH__ >= 800
  return __reduce_max_sync(0xffffffffu, value);
#else
  for (int i = 16; i > 0; i /= 2) {
    value = max(value, __shfl_xor_sync(0xffffffffu, value, i));
  }
  return value;
#endif
}

// The sums of some of a blob pair's points, as in MinMaxExtents.
struct BlobPairSums {
  uint32_t count;
  uint32_t min_x;
  uint32_t max_x;
  uint32_t min_y;
  uint32_t max_y;
  int32_t gx_sum;
  int32_t gy_sum;
  // A warp's points sum to well under 2^31.
  int32_t pxgx_plus_pygy_sum;
};

constexpr int kBinThreads = 256;

// Sums up the points between blobs by blob pair in pairs, and replaces each
// point's key with its pair's slot << 32 | its point_bits().  Each warp takes
// 32 points at a time, and the first lane with points from each pair sums them
// up and adds them to the pair's slot.
//
// This is its own kernel so LabelAndDiff keeps few enough registers to hide
// its loads behind other blocks.
__global__ void __launch_bounds__(kBinThreads)
    BinBlobPairs(BlobPairTable *pairs, uint64_t *points) {
  const uint32_t num_points = pairs->num_points;
  const uint32_t lane = threadIdx.x % 32;
  // Whole warps go around together.
  for (uint32_t i = blockIdx.x * kBinThreads + threadIdx.x;
       i - lane < num_points; i += gridDim.x * kBinThreads) {
    QuadBoundaryPoint point;
    if (i < num_points) {
      point.key = points[i];
    }
    // No point has a rep01() of 0.
    const uint64_t key = point.rep01();
    const uint32_t x = point.x();
    const uint32_t y = point.y();
    const int32_t gx = point.gx();
    const int32_t gy = point.gy();

    BlobPairSums first_sums{};
    bool first = false;
    uint32_t first_lane = lane;
    uint32_t pending = __ballot_sync(0xffffffffu, key != 0);
    while (pending != 0) {
      const uint32_t leader = __ffs(pending) - 1;
      const bool member = key == __shfl_sync(0xffffffffu, key, leader);
      const uint32_t members = __ballot_sync(0xffffffffu, member);
      pending &= ~members;
      const BlobPairSums sums{
          .count = static_cast<uint32_t>(__popc(members)),
          .min_x = WarpMin(member ? x : 0xffffffffu),
          .max_x = WarpMax(member ? x : 0),
          .min_y = WarpMin(member ? y : 0xffffffffu),
          .max_y = WarpMax(member ? y : 0),
          .gx_sum = static_cast<int32_t>(
              WarpSum(member ? static_cast<uint32_t>(gx) : 0)),
          .gy_sum = static_cast<int32_t>(
              WarpSum(member ? static_cast<uint32_t>(gy) : 0)),
          .pxgx_plus_pygy_sum = static_cast<int32_t>(WarpSum(
              member ? static_cast<uint32_t>(static_cast<int32_t>(x) * gx +
                                             static_cast<int32_t>(y) * gy)
                     : 0)),
      };
      if (lane == leader) {
        first_sums = sums;
        first = true;
      }
      if (member) {
        first_lane = leader;
      }
    }
    // Every pair's first lane adds its sums at once, so the warp waits on the
    // table once.
    int slot = 0;
    if (first) {
      slot = FindBlobPair(pairs, key);
      if (slot >= 0) {
        AddToBlobPair(pairs, slot, first_sums.count, first_sums.min_x,
                      first_sums.max_x, first_sums.min_y, first_sums.max_y,
                      first_sums.gx_sum, first_sums.gy_sum,
                      first_sums.pxgx_plus_pygy_sum);
      }
    }
    slot = __shfl_sync(0xffffffffu, max(slot, 0), first_lane);
    if (key != 0) {
      points[i] = (static_cast<uint64_t>(slot) << 32) | point.point_bits();
    }
  }
}

// Selects blobs which are big enough, not too big, and have the right color in
// the middle.
class SelectBlobs {
 public:
  SelectBlobs(const MinMaxExtents *extents_device, apriltag_size_t tag_width,
              bool reversed_border, bool normal_border,
              apriltag_size_t min_cluster_pixels,
              apriltag_size_t max_cluster_pixels)
      : extents_device_(extents_device),
        tag_width_(tag_width),
        reversed_border_(reversed_border),
        normal_border_(normal_border),
        min_cluster_pixels_(std::max<apriltag_size_t>(24u, min_cluster_pixels)),
        max_cluster_pixels_(max_cluster_pixels) {}

  // Returns true if the blob passes the size and dot product checks and is
  // worth further consideration.
  __host__ __device__ __forceinline__ bool operator()(
      MinMaxExtents extents) const {
    if (extents.count < min_cluster_pixels_) {
      return false;
    }
    if (extents.count > max_cluster_pixels_) {
      return false;
    }

    // Area must also be reasonable.
    if (static_cast<apriltag_size_t>(extents.max_x - extents.min_x) *
            static_cast<apriltag_size_t>(extents.max_y - extents.min_y) <
        tag_width_) {
      return false;
    }

    // And the right side must be inside.
    const bool quad_reversed_border = extents.dot() < 0.0f;
    if (!reversed_border_ && quad_reversed_border) {
      return false;
    }
    if (!normal_border_ && !quad_reversed_border) {
      return false;
    }

    return true;
  }

  const MinMaxExtents *extents_device_;
  apriltag_size_t tag_width_;

  bool reversed_border_;
  bool normal_border_;
  apriltag_size_t min_cluster_pixels_;
  apriltag_size_t max_cluster_pixels_;
};

// Returns a point with its angle around its blob's center.
__device__ __forceinline__ IndexPoint AddTheta(IndexPoint a,
                                               const MinMaxExtents &extents) {
  float theta =
      (atan2f(a.y() - extents.cy(), a.x() - extents.cx()) + numbers::pi) * 8e6;
  long long int theta_int = llrintf(theta);

  a.set_theta(std::max<long long int>(0, theta_int));
  return a;
}

// Empties a slot of pairs.
__device__ __forceinline__ void EmptyBlobPair(BlobPairTable *pairs,
                                              uint32_t slot) {
  pairs->keys[slot] = 0;
  pairs->pxgx_plus_pygy_sums[slot] = 0;
  pairs->counts[slot] = 0;
  pairs->min_xs[slot] = 0xffffffffu;
  pairs->max_xs[slot] = 0;
  pairs->min_ys[slot] = 0xffffffffu;
  pairs->max_ys[slot] = 0;
  pairs->gx_sums[slot] = 0;
  pairs->gy_sums[slot] = 0;
}

constexpr int kRankThreads = 256;
// The blob pairs each RankBlobPairs block ranks, one per lane.
constexpr int kRankPairsPerBlock = 32;

// Ranks the blob pairs by rep01(), which puts them in the order sorting their
// points by it would, records each pair's rank, and moves its sums from the
// table to its extents, leaving the table empty for the next frame.  Each warp
// counts the smaller pairs among its share of them.
__global__ void __launch_bounds__(kRankThreads)
    RankBlobPairs(BlobPairTable *pairs, MinMaxExtents *extents) {
  __shared__ uint64_t keys[GpuDetector::kMaxBlobs];
  __shared__ uint32_t ranks[kRankPairsPerBlock];
  const uint32_t num_pairs = pairs->num_pairs;
  // Nothing reads the ranks of a frame which overflows.
  if (num_pairs > GpuDetector::kMaxBlobs || pairs->overflow != 0 ||
      blockIdx.x * kRankPairsPerBlock >= num_pairs) {
    return;
  }
  for (uint32_t i = threadIdx.x; i < num_pairs; i += kRankThreads) {
    keys[i] = pairs->pairs[i];
  }
  if (threadIdx.x < kRankPairsPerBlock) {
    ranks[threadIdx.x] = 0;
  }
  __syncthreads();

  // The rep01()s are in the high bits and all differ, so the keys sort by them.
  const uint32_t lane = threadIdx.x % kRankPairsPerBlock;
  const uint32_t pair = blockIdx.x * kRankPairsPerBlock + lane;
  const uint64_t key = pair < num_pairs ? keys[pair] : 0;
  uint32_t rank = 0;
  for (uint32_t i = threadIdx.x / kRankPairsPerBlock; i < num_pairs;
       i += kRankThreads / kRankPairsPerBlock) {
    rank += keys[i] < key;
  }
  atomicAdd(&ranks[lane], rank);
  __syncthreads();

  if (threadIdx.x < kRankPairsPerBlock && pair < num_pairs) {
    const uint32_t slot = key & (BlobPairTable::kSlots - 1);
    const uint32_t rank = ranks[lane];
    pairs->slot_ranks[slot] = rank;
    // BlobPairExtents fills in the starting offset.
    extents[rank] = MinMaxExtents{
        .min_x = static_cast<uint16_t>(pairs->min_xs[slot]),
        .min_y = static_cast<uint16_t>(pairs->min_ys[slot]),
        .max_x = static_cast<uint16_t>(pairs->max_xs[slot]),
        .max_y = static_cast<uint16_t>(pairs->max_ys[slot]),
        .starting_offset = 0,
        .count = pairs->counts[slot],
        .gx_sum = pairs->gx_sums[slot],
        .gy_sum = pairs->gy_sums[slot],
        .pxgx_plus_pygy_sum =
            static_cast<int64_t>(pairs->pxgx_plus_pygy_sums[slot]),
    };
    EmptyBlobPair(pairs, slot);
  }
}

constexpr int kExtentsThreads = 1024;
constexpr int kExtentsPerThread = GpuDetector::kMaxBlobs / kExtentsThreads;
static_assert(kExtentsPerThread * kExtentsThreads == GpuDetector::kMaxBlobs);
// Each thread zeros its cursors with one uint2 store.
static_assert(kExtentsPerThread == 2);
static_assert(offsetof(BlobPairTable, cursors) % sizeof(uint2) == 0);

using SelectedExtents = cub::KeyValuePair<long, MinMaxExtents>;
// BlobPairExtents builds selected_extents in shared memory a part at a time,
// since all 80 KB of it is more than a block gets without opting in.
constexpr int kSelectedExtentsParts = 2;
constexpr int kSelectedExtentsPerPart =
    GpuDetector::kMaxBlobs / kSelectedExtentsParts;
constexpr int kThreadsPerSelectedExtentsPart =
    kExtentsThreads / kSelectedExtentsParts;
// Each thread's entries, and each part, are whole uint4s.
static_assert(kExtentsPerThread * sizeof(SelectedExtents) % sizeof(uint4) == 0);
static_assert(sizeof(MinMaxExtents) % sizeof(uint4) == 0);

// Stores value with one 16 byte store.  Clang copies a uint4 as two 8 byte
// halves, so this builds a new one from its fields.
__device__ __forceinline__ void StoreUint4(uint4 value, uint4 *destination) {
  *destination = make_uint4(value.x, value.y, value.z, value.w);
}

// Stores kBytes of source to a 16 byte aligned destination as uint4s.  The
// structs here are only 8 byte aligned, so the compiler stores them 8 bytes at
// a time.
template <size_t kBytes>
__device__ __forceinline__ void StoreAsUint4s(const void *source,
                                              void *destination) {
  static_assert(kBytes % sizeof(uint4) == 0);
  uint4 words[kBytes / sizeof(uint4)];
  memcpy(words, source, kBytes);
#pragma unroll
  for (size_t i = 0; i < kBytes / sizeof(uint4); ++i) {
    StoreUint4(words[i], reinterpret_cast<uint4 *>(destination) + i);
  }
}

// Fills in each blob pair's starting offset in extents, where sorting the
// points by pair would put it, and writes the extents of the selected blobs,
// with their offsets once the rest are left out.  selected_extents covers all
// kMaxBlobs blobs, and those past the last pair are zero but for their keys.
// Then reports on the frame and zeros the table's counts for the next one.
__global__ void __launch_bounds__(kExtentsThreads)
    BlobPairExtents(BlobPairTable *pairs, SelectBlobs select_blobs,
                    MinMaxExtents *extents, SelectedExtents *selected_extents,
                    int *num_selected_points, uint32_t *overflow,
                    uint32_t *num_blob_pairs, uint32_t *num_boundary_points) {
  typedef cub::BlockScan<unsigned long long, kExtentsThreads> BlockScan;
  // Written from each thread, selected_extents' 40 byte entries leave a warp's
  // stores 80 bytes apart, so they go through shared memory and out as
  // consecutive uint4s.  KeyValuePair's constructor keeps it out of the union.
  union Storage {
    typename BlockScan::TempStorage scan;
    uint4 selected_extents[kSelectedExtentsPerPart * sizeof(SelectedExtents) /
                           sizeof(uint4)];
  };
  __shared__ Storage storage;

  const uint32_t num_pairs = pairs->num_pairs;
  const uint32_t num_points = pairs->num_points;
  // selected_extents and the other per-blob buffers only hold kMaxBlobs blobs,
  // and there is no good way to pick which to keep, so a frame with more has
  // every blob filtered out.
  const bool overflowed =
      num_pairs > GpuDetector::kMaxBlobs || pairs->overflow != 0;

  MinMaxExtents blob[kExtentsPerThread] = {};
  // Each blob's number of points in the low 32 bits, and its number of
  // selected points in the high 32.
  unsigned long long counts[kExtentsPerThread];
#pragma unroll
  for (int i = 0; i < kExtentsPerThread; ++i) {
    const uint32_t rank = threadIdx.x * kExtentsPerThread + i;
    counts[i] = 0;
    if (!overflowed && rank < num_pairs) {
      blob[i] = extents[rank];
      counts[i] =
          blob[i].count | (static_cast<unsigned long long>(
                               select_blobs(blob[i]) ? blob[i].count : 0)
                           << 32);
    }
  }
  unsigned long long offsets[kExtentsPerThread];
  unsigned long long total;
  BlockScan(storage.scan).ExclusiveSum(counts, offsets, total);

  SelectedExtents selected[kExtentsPerThread];
#pragma unroll
  for (int i = 0; i < kExtentsPerThread; ++i) {
    const uint32_t rank = threadIdx.x * kExtentsPerThread + i;
    if (!overflowed && rank < num_pairs) {
      blob[i].starting_offset = offsets[i] & 0xffffffffu;
      StoreAsUint4s<sizeof(MinMaxExtents)>(&blob[i], &extents[rank]);
    }
    selected[i].key = rank;
    selected[i].value = MinMaxExtents{};
    selected[i].value.min_x = blob[i].min_x;
    selected[i].value.min_y = blob[i].min_y;
    selected[i].value.max_x = blob[i].max_x;
    selected[i].value.max_y = blob[i].max_y;
    selected[i].value.count = counts[i] >> 32;
    // Past the pairs, every field but the key is zero, the same every frame.
    selected[i].value.starting_offset = rank < num_pairs ? offsets[i] >> 32 : 0;
  }
  reinterpret_cast<uint2 *>(pairs->cursors)[threadIdx.x] = make_uint2(0, 0);

  // Entries past both this frame's pairs and the last frame's are still
  // empty.
  const uint32_t num_selected_extents = std::min<uint32_t>(
      GpuDetector::kMaxBlobs,
      std::max(num_pairs, pairs->num_nonempty_selected_extents));
#pragma unroll
  for (int part = 0; part < kSelectedExtentsParts; ++part) {
    const uint32_t first = part * kSelectedExtentsPerPart;
    if (num_selected_extents <= first) {
      break;
    }
    // The first part reuses the scan's storage, and the rest the last part's.
    __syncthreads();
    const int part_thread =
        static_cast<int>(threadIdx.x) - part * kThreadsPerSelectedExtentsPart;
    if (part_thread >= 0 && part_thread < kThreadsPerSelectedExtentsPart) {
      StoreAsUint4s<sizeof(selected)>(
          selected, &storage.selected_extents[part_thread * sizeof(selected) /
                                              sizeof(uint4)]);
    }
    __syncthreads();
    uint4 *destination = reinterpret_cast<uint4 *>(selected_extents + first);
    // Rounding up rewrites the start of the next entry with its own value.
    const uint32_t num_words = std::min<uint32_t>(
        sizeof(storage.selected_extents) / sizeof(uint4),
        ((num_selected_extents - first) * sizeof(SelectedExtents) +
         sizeof(uint4) - 1) /
            sizeof(uint4));
    for (uint32_t word = threadIdx.x; word < num_words;
         word += kExtentsThreads) {
      StoreUint4(storage.selected_extents[word], destination + word);
    }
  }

  // RankBlobPairs emptied every slot of a frame which fits.  One which
  // overflows may have taken slots it could not list.
  if (overflowed) {
    for (uint32_t slot = threadIdx.x; slot < BlobPairTable::kSlots;
         slot += kExtentsThreads) {
      EmptyBlobPair(pairs, slot);
    }
  }
  // Every thread has read the counts.
  __syncthreads();
  if (threadIdx.x == 0) {
    *num_selected_points = overflowed ? 0 : total >> 32;
    *overflow = overflowed;
    *num_blob_pairs = num_pairs;
    *num_boundary_points = num_points;
    pairs->frame_num_points = num_points;
    pairs->frame_overflowed = overflowed;
    pairs->num_nonempty_selected_extents = overflowed ? 0 : num_pairs;
    pairs->num_pairs = 0;
    pairs->num_points = 0;
    pairs->overflow = 0;
  }
}

constexpr int kScatterThreads = 256;

// Writes each selected blob's points, with their angles, to its range of
// selected_points, in no particular order within the blob.
__global__ void __launch_bounds__(kScatterThreads) ScatterSelectedPoints(
    const uint64_t *points, BlobPairTable *pairs, const MinMaxExtents *extents,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents,
    IndexPoint *selected_points) {
  if (pairs->frame_overflowed) {
    return;
  }
  const uint32_t num_points = pairs->frame_num_points;
  const uint32_t lane = threadIdx.x % 32;
  const uint32_t lanes_below = (1u << lane) - 1;
  constexpr uint32_t kNoBlob = 0xffffffffu;
  // Whole warps go around together, for __match_any_sync.
  for (uint32_t i = blockIdx.x * kScatterThreads + threadIdx.x;
       i - lane < num_points; i += gridDim.x * kScatterThreads) {
    uint32_t blob_index = kNoBlob;
    uint32_t starting_offset = 0;
    IndexPoint point;
    if (i < num_points) {
      const uint64_t boundary_point = points[i];
      const uint32_t index = pairs->slot_ranks[boundary_point >> 32];
      const MinMaxExtents selected = selected_extents[index].value;
      if (selected.count > 0) {
        blob_index = index;
        starting_offset = selected.starting_offset;
        point = AddTheta(IndexPoint(index, boundary_point & 0xffffff),
                         extents[index]);
      }
    }
    // The first lane with points from a blob claims space for all of them.
    const uint32_t peers = __match_any_sync(0xffffffffu, blob_index);
    uint32_t position = 0;
    if (blob_index != kNoBlob && (peers & lanes_below) == 0) {
      position = atomicAdd(&pairs->cursors[blob_index], __popc(peers));
    }
    position = __shfl_sync(0xffffffffu, position, __ffs(peers) - 1) +
               __popc(peers & lanes_below);
    if (blob_index != kNoBlob) {
      selected_points[starting_offset + position] = point;
    }
  }
}

}  // namespace

void GpuDetector::Detect(const uint8_t *image, const uint8_t *image_device) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  // Records event on stream if timing every stage.
  auto record_stage = [this](CudaEvent &event, CudaStream *stream) {
    if (stage_timing_) {
      event.Record(stream);
    }
  };

  // start_ begins the GPU time and the "Memcpy" stage.
  const bool record_start = measure_gpu_time_ || stage_timing_;
  const bool use_graph = frame_graph_.has_value() && !stage_timing_;

  // Threshold the image.
  float neon_threshold_ms = 0;
  if (use_neon_) {
    // We are being asked to do this on the CPU.  Use the CPU source image, and
    // put it into unified memory so it gets moved over to the GPU in the
    // background.
    const aos::monotonic_clock::time_point before_threshold =
        aos::monotonic_clock::now();
    threshold_->ThresholdAndDecimate(
        image, decimated_image_device_, thresholded_image_device_,
        tag_detector_->qtp.min_white_black_diff, &stream_);
    neon_threshold_ms =
        float_milli(aos::monotonic_clock::now() - before_threshold).count();

    if (record_start) {
      start_.Record(&stream_);
    }
    record_stage(after_image_memcpy_to_device_, &stream_);
  } else {
    if (record_start) {
      start_.Record(&stream_);
    }
    // If the image isn't already in device memory, copy it there and update the
    // pointer.
    if (image_device == nullptr) {
      color_image_device_.MemcpyAsyncFrom(image, &stream_);
      image_device = color_image_device_.get();
    }
    record_stage(after_image_memcpy_to_device_, &stream_);

    // Now, threshold on the GPU fully.
    if (image_format_ == vision::ImageFormat::MONO8) {
      // CameraImage.data is a flatbuffer [ubyte] vector, which flatbuffers
      // only aligns to 4 bytes.
      ABSL_CHECK_EQ(reinterpret_cast<uintptr_t>(image_device) % 4, 0u)
          << ": the MONO8 threshold loads the image 4 bytes at a time";
      *image_address_.mutable_host_ptr() = image_device;
      if (!(use_graph && frame_graph_thresholds_)) {
        threshold_->ThresholdAndDecimateIndirect(
            image_address_.device_ptr(), decimated_image_device_,
            thresholded_image_device_, tag_detector_->qtp.min_white_black_diff,
            &stream_);
      }
    } else {
      threshold_->ThresholdAndDecimate(
          image_device, decimated_image_device_, thresholded_image_device_,
          tag_detector_->qtp.min_white_black_diff, &stream_);
    }
  }

  record_stage(after_threshold_, &stream_);

  if (image_format_ != vision::ImageFormat::MONO8) {
    threshold_->ToGreyscale(image_device, gray_image_device_.get(), &stream_);
    gray_image_device_.MemcpyAsyncTo(&gray_image_host_, &stream_);
    gray_image_host_ptr_ = gray_image_host_.get();
  } else {
    gray_image_host_ptr_ = image;
  }

  record_stage(after_memcpy_gray_, &stream_);

  if (use_graph) {
    frame_graph_->Launch(&stream_);
  } else {
    EnqueueQuadFitting(stage_timing_, /*fork_big_blobs=*/false);
  }
  // The host reads the quads where the quad fits write them, so this is
  // Detect()'s only wait on the GPU.
  after_quad_fit_.Record(&stream_);
  after_quad_fit_.Synchronize(host_wait_spin_);

  // selected_extents_device_ and the other per-blob buffers only hold kMaxBlobs
  // blobs, so the GPU filtered every blob out of an overflowing frame.
  last_frame_overflowed_ = frame_status_.host_ptr()->overflow != 0;
  const uint32_t num_blob_pairs = frame_status_.host_ptr()->num_blob_pairs;
  if (last_frame_overflowed_) {
    ABSL_LOG_EVERY_N_SEC(ERROR, 1)
        << "Dropping a frame with " << num_blob_pairs
        << " or more blob pairs, more than " << kMaxBlobs;
  }

  const aos::monotonic_clock::time_point before_fit_quads =
      aos::monotonic_clock::now();
  UpdateFitQuads();
  AdjustPixelCenters();

  DecodeTags();

  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  // TODO(austin): Bring it back to the CPU and see how good we did.

  // Report out how long things took.

  ABSL_VLOG(1) << "Found " << frame_status_.host_ptr()->num_boundary_points
               << " items";
  ABSL_VLOG(1) << "Found compressed runs: " << num_blob_pairs;
  ABSL_VLOG(1) << "Fit " << quad_corners_host_.size() << " quads";
  host_decode_time_ms_ = float_milli(end_time - before_fit_quads).count();
  if (stage_timing_) {
    // Every event completed before after_quad_fit_, which the host already
    // waited on.
    stage_times_ms_.clear();
    if (use_neon_) {
      stage_times_ms_.emplace_back("NEON threshold (host)", neon_threshold_ms);
    }
    for (const Stage &stage : stages_) {
      stage_times_ms_.emplace_back(
          stage.name,
          float_milli(stage.end->ElapsedTime(*stage.begin)).count());
      ABSL_VLOG(1) << "    " << stage.name << " "
                   << stage_times_ms_.back().second << "ms";
    }
  }
  ABSL_VLOG(1) << "  FitQuads "
               << float_milli(end_time - before_fit_quads).count()
               << "ms on host";

  if (measure_gpu_time_) {
    const std::chrono::nanoseconds gpu_time =
        after_quad_fit_.ElapsedTime(start_);
    gpu_time_ms_ = float_milli(gpu_time).count();
    ABSL_VLOG(1) << "Overall " << gpu_time_ms_ << "ms, "
                 << float_milli(end_time - start_time).count() << "ms on host, "
                 << (1000.0 / gpu_time_ms_) << "hz";
    // Average.  Skip the first one as the kernel is warming up and is slower.
    if (!first_) {
      ++execution_count_;
      execution_duration_ += gpu_time;
      ABSL_VLOG(1)
          << "Average overall "
          << float_milli(execution_duration_ / execution_count_).count()
          << "ms, "
          << (1000.0 /
              float_milli(execution_duration_ / execution_count_).count())
          << "hz";
    }
    first_ = false;
  } else {
    gpu_time_ms_ = std::numeric_limits<float>::quiet_NaN();
  }
}

void GpuDetector::EnqueueQuadFitting(bool record_stages, bool fork_big_blobs) {
  // Records event on stream if timing every stage.
  auto record_stage = [record_stages](CudaEvent &event, CudaStream *stream) {
    if (record_stages) {
      event.Record(stream);
    }
  };

  // Unionfind the image, and find the points between blobs.
  LabelImage(ToDecimatedGpuImage(thresholded_image_device_),
             ToGpuImage(union_markers_size_device_),
             keep_intermediates_ ? union_markers_device_.get() : nullptr,
             LabelScratch{
                 .tile_roots = label_tile_roots_device_.get(),
                 .forest = label_forest_device_.get(),
                 .roots = label_roots_device_.get(),
                 .num_roots = label_num_roots_device_.get(),
             },
             boundary_points_device_.get(),
             &blob_pairs_device_.get()->num_points, stream_.get(),
             record_stages ? std::span<CudaEvent>(after_labeling_kernel_)
                           : std::span<CudaEvent>());
  MaybeCheckAndSynchronize("LabelImage");

  // Longest april tag will be the full perimeter of the image.  Each point
  // results in 2 neighbor points, 1 straight, and one at 45 degrees.  But, we
  // are in decimated space here, and width_ and height_ are in full image
  // space.  And there are 2 sides to each rectangle...
  //
  // Aprilrobotics has a *3 instead of a *2 here since they have duplicated
  // points in their list at this stage.
  //
  // FitBigBlobQuads() holds at most kMaxBlobPoints points, so bigger blobs are
  // rejected as too big as well.  A point is about a pixel of the quad's
  // perimeter, so that is a quad over 6144 pixels around, like a 1536 pixel
  // square.  It changes nothing while width_ + height_ <= 3072, and past that
  // it only drops quads spanning most of the frame: 98% of its perimeter at
  // 1920x1200, and 75% at 2048x2040, the longest the constructor accepts.
  const apriltag_size_t max_april_tag_perimeter =
      std::min<apriltag_size_t>(2 * (width_ + height_), kMaxBlobPoints);

  // Sum them up by blob pair.
  {
    // Only the GPU knows the number of points, so the grid fills the GPU once
    // and loops over them.
    static const int kBinBlocks = ResidentBlocks(BinBlobPairs, kBinThreads);
    BinBlobPairs<<<kBinBlocks, kBinThreads, 0, stream_.get()>>>(
        blob_pairs_device_.get(), boundary_points_device_.get());
    MaybeCheckAndSynchronize("BinBlobPairs");
  }
  record_stage(after_bin_pairs_, &stream_);

  // Number the blob pairs in the order sorting their points by pair would put
  // them in, and write their extents in that order.
  RankBlobPairs<<<kMaxBlobs / kRankPairsPerBlock, kRankThreads, 0,
                  stream_.get()>>>(blob_pairs_device_.get(),
                                   extents_device_.get());
  MaybeCheckAndSynchronize("RankBlobPairs");
  record_stage(after_rank_pairs_, &stream_);

  // Pick the blobs worth fitting quads to by their extents and dot products,
  // and work out where their points go.
  BlobPairExtents<<<1, kExtentsThreads, 0, stream_.get()>>>(
      blob_pairs_device_.get(),
      SelectBlobs(nullptr, min_tag_width_, reversed_border_, normal_border_,
                  tag_detector_->qtp.min_cluster_pixels,
                  max_april_tag_perimeter),
      extents_device_.get(), selected_extents_device_.get(),
      num_selected_blobs_device_.get(), &frame_status_.device_ptr()->overflow,
      &frame_status_.device_ptr()->num_blob_pairs,
      &frame_status_.device_ptr()->num_boundary_points);
  MaybeCheckAndSynchronize("BlobPairExtents");
  record_stage(after_pair_extents_, &stream_);

  // Copy the selected blobs' points there.
  {
    // Only the GPU knows the number of points, so the grid fills the GPU once
    // and loops over them.
    static const int kScatterBlocks =
        ResidentBlocks(ScatterSelectedPoints, kScatterThreads);
    ScatterSelectedPoints<<<kScatterBlocks, kScatterThreads, 0,
                            stream_.get()>>>(
        boundary_points_device_.get(), blob_pairs_device_.get(),
        extents_device_.get(), selected_extents_device_.get(),
        selected_blobs_device_.get());
    MaybeCheckAndSynchronize("ScatterSelectedPoints");
  }
  record_stage(after_scatter_, &stream_);

  // Fit each selected blob's quad, from sorting its points by angle around it
  // on, in a block per blob.  Blobs are contiguous and in blob order in
  // selected_blobs_device_, so sorting each blob's points matches a global
  // sort by (blob, angle, AngleTieKey()), and the host never needs the number
  // of selected points.
  const BlobQuadFitArgs blob_quad_fit_args{
      .work_list = blob_work_list_device_.get(),
      .selected_blobs = selected_blobs_device_.get(),
      .selected_extents = selected_extents_device_.get(),
      .decimated_image = decimated_image_device_,
      .decimated_width = static_cast<int>(width_ / 2),
      .decimated_height = static_cast<int>(height_ / 2),
      .sorted_points = sorted_selected_blobs_device_.get(),
      .line_fit_points = line_fit_points_device_.get(),
      .errs = errs_device_.get(),
      .filtered_errs = filtered_errs_device_.get(),
      .peaks = filtered_is_local_peak_device_.get(),
      .keep_intermediates = keep_intermediates_,
      .max_blob_points = max_april_tag_perimeter,
      .nmaxima = tag_detector_->qtp.max_nmaxima,
      .max_line_fit_mse = tag_detector_->qtp.max_line_fit_mse,
      .cos_critical_rad = tag_detector_->qtp.cos_critical_rad,
      .fit_quads = fit_quads_host_.device_ptr(),
      .fit_quad_valid = fit_quad_valid_host_.device_ptr(),
  };
  ListBlobs(blob_quad_fit_args, &stream_);
  MaybeCheckAndSynchronize("ListBlobs");
  if (fork_big_blobs) {
    // Each big-blob block needs a whole SM's registers, so after the
    // small-blob quad fits, the few of them would leave most SMs idle.  On a
    // branch of its own, the big-blob quad fit overlaps the small-blob ones
    // instead, as long as its blocks are dispatched before the small-blob
    // blocks fill the SMs.  Its stream's priority sees to that, whatever order
    // the graph's ready kernels would otherwise go in.  The two write disjoint
    // blobs, and the host reads the quads after the join.
    fork_big_blobs_.Record(&stream_);
    big_blob_stream_.Wait(&fork_big_blobs_);
    FitBigBlobQuads(blob_quad_fit_args, &big_blob_stream_);
    FitSmallBlobQuads(blob_quad_fit_args, &stream_);
    join_big_blobs_.Record(&big_blob_stream_);
    stream_.Wait(&join_big_blobs_);
  } else {
    FitSmallBlobQuads(blob_quad_fit_args, &stream_);
    MaybeCheckAndSynchronize("FitSmallBlobQuads");
    record_stage(after_small_blob_quads_, &stream_);
    FitBigBlobQuads(blob_quad_fit_args, &stream_);
    MaybeCheckAndSynchronize("FitBigBlobQuads");
  }
}

}  // namespace frc::apriltag

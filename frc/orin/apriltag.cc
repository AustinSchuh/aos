#include "frc/orin/apriltag.h"

#include <cub/block/block_radix_sort.cuh>
#include <cub/block/block_scan.cuh>
#include <cub/warp/warp_scan.cuh>
#include <cuda/atomic>
#include <numbers>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "third_party/apriltag/common/g2d.h"

#include "aos/time/time.h"
#include "frc/orin/labeling_allegretti_2019_BKE.h"
#include "frc/orin/threshold.h"

namespace numbers = std::numbers;

ABSL_FLAG(bool, use_neon, false, "Use NEON optimized threshold");

namespace frc::apriltag {
namespace {

typedef std::chrono::duration<float, std::milli> float_milli;

}  // namespace

GpuDetector::GpuDetector(size_t width, size_t height,
                         apriltag_detector_t *tag_detector,
                         CameraMatrix camera_matrix,
                         DistCoeffs distortion_coefficients,
                         vision::ImageFormat image_format)
    : width_(width),
      height_(height),
      tag_detector_(tag_detector),
      gray_image_host_(width * height),
      color_image_device_(BytesPerPixel(image_format) * width * height),
      gray_image_device_(width * height),
      decimated_image_device_(width / 2 * height / 2),
      thresholded_image_device_(width / 2 * height / 2),
      union_markers_device_(width / 2 * height / 2),
      union_markers_size_device_(width / 2 * height / 2),
      label_tile_roots_device_(width / 4 * height / 4),
      label_forest_device_(width / 2 * height / 2),
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
      threshold_(absl::GetFlag(FLAGS_use_neon)
                     ? MakeNeonThreshold(image_format, width, height)
                     : MakeThreshold(image_format, width, height)),
      image_format_(image_format) {
  quad_corners_host_.reserve(kMaxBlobs);

  ABSL_CHECK_EQ(tag_detector_->quad_decimate, 2);
  ABSL_CHECK(!tag_detector_->qtp.deglitch);
  // The NEON threshold reads one byte per pixel from the host image.
  ABSL_CHECK(!absl::GetFlag(FLAGS_use_neon) ||
             image_format == vision::ImageFormat::MONO8)
      << ": --use_neon only supports MONO8 images";

  // The Orin has no concurrent managed access, so the CPU may only touch
  // globally attached managed memory while no stream in the process has work
  // in flight.  Attached to stream_, these only need stream_ to be idle.
  CHECK_CUDA(cudaStreamAttachMemAsync(
      stream_.get(), decimated_image_device_.get(), 0, cudaMemAttachSingle));
  CHECK_CUDA(cudaStreamAttachMemAsync(
      stream_.get(), thresholded_image_device_.get(), 0, cudaMemAttachSingle));
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
}

GpuDetector::~GpuDetector() {
  for (int i = 0; i < zarray_size(detections_); ++i) {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);
    apriltag_detection_destroy(det);
  }

  zarray_destroy(detections_);
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

// Finds a QuadBoundaryPoint for each pixel pair which crosses the boundary
// between two blobs of kMinBlobPixels or more, and appends their keys to
// points in no particular order, counting them in pairs->num_points.
//
// Each block row is one warp.  Blocks overlap by 1 column on each side and 1
// row below, so each pixel's neighbors are in shared memory.
template <int kBlockWidth, int kBlockHeight>
__global__ void __launch_bounds__(kBlockWidth *kBlockHeight)
    BlobDiff(const uint8_t *thresholded_image, const uint32_t *blobs,
             const uint32_t *union_markers_size, BlobPairTable *pairs,
             uint64_t *points, apriltag_size_t width, apriltag_size_t height) {
  static_assert(kBlockWidth == 32);
  constexpr uint32_t kMinBlobPixels = 25;
  __shared__ uint8_t thresholded[kBlockHeight][kBlockWidth];
  typedef cub::WarpScan<uint32_t> WarpScan;
  __shared__ typename WarpScan::TempStorage scan_storage[kBlockHeight];

  const uint x = blockIdx.x * (kBlockWidth - 2) + threadIdx.x;
  const uint y = blockIdx.y * (kBlockHeight - 1) + threadIdx.y;
  const bool in_image = x < width && y < height;
  thresholded[threadIdx.y][threadIdx.x] =
      in_image ? thresholded_image[x + y * width] : 127;
  __syncthreads();

  // We search the following 4 neighbors.
  //      ________
  //      | x | 0 |
  //  -------------
  //  | 3 | 2 | 1 |
  //  -------------
  //
  // The pixels along the image's edges and the block's overlap have no point
  // of their own.
  QuadBoundaryPoint point[4];
  const uint8_t v0 = thresholded[threadIdx.y][threadIdx.x];
  if (in_image && threadIdx.x != 0 && threadIdx.x != kBlockWidth - 1 &&
      threadIdx.y != kBlockHeight - 1 && y != 0 && x != width - 1 &&
      y != height - 1 && v0 != 127) {
    const uint8_t v1[4] = {
        thresholded[threadIdx.y][threadIdx.x + 1],
        thresholded[threadIdx.y + 1][threadIdx.x + 1],
        thresholded[threadIdx.y + 1][threadIdx.x],
        thresholded[threadIdx.y + 1][threadIdx.x - 1],
    };
    const uint8_t v_left = thresholded[threadIdx.y][threadIdx.x - 1];
    bool crosses[4];
#pragma unroll
    for (int i = 0; i < 4; ++i) {
      crosses[i] = v0 + v1[i] == 255;
    }

    // Most pixels are inside a blob, and need nothing more.
    if (crosses[0] || crosses[1] || crosses[2] || crosses[3]) {
      // When the left pixel and 2 have different colors, the left pixel's
      // point across 1 is at the same place as ours across 3, between the
      // same two blobs, so ours is left out.  Pixels on the image's edge have
      // no points.
      const bool maybe_duplicate = crosses[3] && v_left != 127 &&
                                   v1[2] != 127 && v_left != v1[2] && x != 1;
      const uint index0 = x + y * width;
      const uint index1[4] = {index0 + 1, index0 + width + 1, index0 + width,
                              index0 + width - 1};
      // Issue every load before using any, since each is a round trip to
      // memory.
      const uint32_t rep0 = blobs[index0];
      uint32_t rep1[4];
#pragma unroll
      for (int i = 0; i < 4; ++i) {
        rep1[i] =
            crosses[i] || (i == 2 && maybe_duplicate) ? blobs[index1[i]] : 0;
      }
      const uint32_t rep_left = maybe_duplicate ? blobs[index0 - 1] : 0;
      const uint32_t size0 = union_markers_size[rep0];
      uint32_t size1[4];
#pragma unroll
      for (int i = 0; i < 4; ++i) {
        size1[i] = crosses[i] || (i == 2 && maybe_duplicate)
                       ? union_markers_size[rep1[i]]
                       : 0;
      }
      const uint32_t size_left =
          maybe_duplicate ? union_markers_size[rep_left] : 0;

      if (size0 >= kMinBlobPixels) {
        const bool duplicate = maybe_duplicate && size_left >= kMinBlobPixels &&
                               size1[2] >= kMinBlobPixels;
#pragma unroll
        for (int i = 0; i < 4; ++i) {
          if (crosses[i] && size1[i] >= kMinBlobPixels &&
              !(i == 3 && duplicate)) {
            point[i].set_rep0(min(rep0, rep1[i]));
            point[i].set_rep1(max(rep0, rep1[i]));
            point[i].set_base_xy(x, y);
            point[i].set_dxy(i);
            point[i].set_black_to_white(v1[i] > v0);
          }
        }
      }
    }
  }

  // Each warp claims space for its points with one atomic.
  const uint32_t num_thread_points = point[0].nonzero() + point[1].nonzero() +
                                     point[2].nonzero() + point[3].nonzero();
  uint32_t offset;
  uint32_t num_warp_points;
  WarpScan(scan_storage[threadIdx.y])
      .ExclusiveSum(num_thread_points, offset, num_warp_points);
  uint32_t warp_start = 0;
  if (threadIdx.x == 0 && num_warp_points > 0) {
    warp_start = atomicAdd(&pairs->num_points, num_warp_points);
  }
  offset += __shfl_sync(0xffffffffu, warp_start, 0);
#pragma unroll
  for (int i = 0; i < 4; ++i) {
    if (point[i].nonzero()) {
      points[offset++] = point[i].key;
    }
  }
}

constexpr int kBinThreads = 256;

// Sums up the points between blobs by blob pair in pairs, and replaces each
// point's key with its pair's slot << 32 | its point_bits().  Each warp takes
// 32 points at a time, and the first lane with points from each pair sums them
// up and adds them to the pair's slot.
//
// This is its own kernel so BlobDiff keeps few enough registers to hide its
// loads behind other blocks.
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

// Fills in each blob pair's starting offset in extents, where sorting the
// points by pair would put it, and writes the extents of the selected blobs,
// with their offsets once the rest are left out.  selected_extents covers all
// kMaxBlobs blobs, and those past the last pair have no points.  Then reports
// on the frame and zeros the table's counts for the next one.
__global__ void __launch_bounds__(kExtentsThreads)
    BlobPairExtents(BlobPairTable *pairs, SelectBlobs select_blobs,
                    MinMaxExtents *extents,
                    cub::KeyValuePair<long, MinMaxExtents> *selected_extents,
                    int *num_selected_points, uint32_t *overflow,
                    uint32_t *num_blob_pairs, uint32_t *num_boundary_points) {
  typedef cub::BlockScan<unsigned long long, kExtentsThreads> BlockScan;
  __shared__ typename BlockScan::TempStorage scan_storage;

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
  BlockScan(scan_storage).ExclusiveSum(counts, offsets, total);

#pragma unroll
  for (int i = 0; i < kExtentsPerThread; ++i) {
    const uint32_t rank = threadIdx.x * kExtentsPerThread + i;
    if (!overflowed && rank < num_pairs) {
      blob[i].starting_offset = offsets[i] & 0xffffffffu;
      extents[rank] = blob[i];
    }
    cub::KeyValuePair<long, MinMaxExtents> selected;
    selected.key = rank;
    selected.value = MinMaxExtents{};
    selected.value.min_x = blob[i].min_x;
    selected.value.min_y = blob[i].min_y;
    selected.value.max_x = blob[i].max_x;
    selected.value.max_y = blob[i].max_y;
    selected.value.count = counts[i] >> 32;
    selected.value.starting_offset = offsets[i] >> 32;
    selected_extents[rank] = selected;
    pairs->cursors[rank] = 0;
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

struct TransformLineFitPoint {
  // Returns the weight of a point, from the image gradient at it.
  __device__ __forceinline__ int32_t Weight(IndexPoint p) const {
    // we now undo our fixed-point arithmetic.
    // adjust for pixel center bias
    constexpr int delta = 1;
    int32_t ix2 = p.x() + delta;
    int32_t iy2 = p.y() + delta;
    int32_t ix = ix2 / 2;
    int32_t iy = iy2 / 2;

    int32_t W = 1;

    if (ix > 0 && ix + 1 < decimated_width && iy > 0 &&
        iy + 1 < decimated_height) {
      int32_t grad_x = decimated_image_device_[iy * decimated_width + ix + 1] -
                       decimated_image_device_[iy * decimated_width + ix - 1];

      int32_t grad_y =
          decimated_image_device_[(iy + 1) * decimated_width + ix] -
          decimated_image_device_[(iy - 1) * decimated_width + ix];

      // XXX Tunable. How to shape the gradient magnitude?
      W = hypotf(grad_x, grad_y) + 1;
    }
    return W;
  }

  // Returns the line fit moments of a point with weight W.
  __device__ __forceinline__ static LineFitPoint Moments(IndexPoint p,
                                                         int32_t W) {
    LineFitPoint result;

    constexpr int delta = 1;
    int32_t ix2 = p.x() + delta;
    int32_t iy2 = p.y() + delta;

    result.Mx = W * ix2;
    result.My = W * iy2;
    result.Mxx = W * ix2 * ix2;
    result.Mxy = W * ix2 * iy2;
    result.Myy = W * iy2 * iy2;
    result.W = W;
    result.blob_index = p.blob_index();
    return result;
  }

  __device__ __forceinline__ LineFitPoint operator()(IndexPoint p) const {
    return Moments(p, Weight(p));
  }

  TransformLineFitPoint(const uint8_t *decimated_image_device,
                        int decimated_width_param, int decimated_height_param)
      : decimated_image_device_(decimated_image_device),
        decimated_width(decimated_width_param),
        decimated_height(decimated_height_param) {}
  const uint8_t *decimated_image_device_;
  int decimated_width;
  int decimated_height;
};

struct SumLineFitPoints {
  __host__ __device__ __forceinline__ LineFitPoint
  operator()(const LineFitPoint &a, const LineFitPoint &b) const {
    LineFitPoint result;
    result.Mx = a.Mx + b.Mx;
    result.My = a.My + b.My;
    result.Mxx = a.Mxx + b.Mxx;
    result.Mxy = a.Mxy + b.Mxy;
    result.Myy = a.Myy + b.Myy;
    result.W = a.W + b.W;
    result.blob_index = a.blob_index;
    return result;
  }
};

// One block sorts each selected blob's points by angle and prefix sums their
// line fit points.  Blobs with up to kMaxSmallBlobPoints points get a
// SortAndScanSmallBlobs block each, and the few bigger ones share the
// SortAndScanBigBlobs blocks.
constexpr uint32_t kMaxSmallBlobPoints = 2048;
// Blobs with up to this many points are sorted by ranking every point against
// the others, which takes fewer steps than a radix sort's passes.
constexpr uint32_t kMaxRankSortPoints = 256;
constexpr int kSmallBlobThreads = 256;
constexpr int kBigBlobThreads = 512;
constexpr int kBigBlobPointsPerThread = 12;
// A frame has about 1.5 big blobs.
constexpr int kBigBlobBlocks = 8;
constexpr int kWorkListThreads = 1024;
// The width of IndexPoint::theta().
constexpr int kThetaBits = 28;

static_assert(GpuDetector::kMaxBlobs % kWorkListThreads == 0);
// BuildBlobWorkList counts both kinds of blob in one 32 bit sum.
static_assert(GpuDetector::kMaxBlobs < (1 << 16));

// Lists the selected blobs with points, split by size, in blob order.
__global__ void __launch_bounds__(kWorkListThreads) BuildBlobWorkList(
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    BlobWorkList *work_list) {
  constexpr int kBlobsPerThread = GpuDetector::kMaxBlobs / kWorkListThreads;
  typedef cub::BlockScan<uint32_t, kWorkListThreads> BlockScan;
  __shared__ typename BlockScan::TempStorage scan_storage;

  // Small blobs count in the low 16 bits, and big ones in the high 16.
  constexpr uint32_t kSmallBlob = 1;
  constexpr uint32_t kBigBlob = 1 << 16;
  uint32_t kind[kBlobsPerThread];
#pragma unroll
  for (int i = 0; i < kBlobsPerThread; ++i) {
    const uint32_t count =
        selected_extents_device[threadIdx.x * kBlobsPerThread + i].value.count;
    if (count == 0) {
      kind[i] = 0;
    } else if (count <= kMaxSmallBlobPoints) {
      kind[i] = kSmallBlob;
    } else {
      kind[i] = kBigBlob;
    }
  }
  uint32_t position[kBlobsPerThread];
  uint32_t total;
  BlockScan(scan_storage).ExclusiveSum(kind, position, total);
#pragma unroll
  for (int i = 0; i < kBlobsPerThread; ++i) {
    const uint16_t blob = threadIdx.x * kBlobsPerThread + i;
    if (kind[i] == kSmallBlob) {
      work_list->small_blobs[position[i] & 0xffff] = blob;
    } else if (kind[i] == kBigBlob) {
      work_list->big_blobs[position[i] >> 16] = blob;
    }
  }
  if (threadIdx.x == 0) {
    work_list->num_small_blobs = total & 0xffff;
    work_list->num_big_blobs = total >> 16;
  }
}

template <int kThreads>
using LineFitPointScan =
    cub::BlockScan<LineFitPoint, kThreads, cub::BLOCK_SCAN_WARP_SCANS>;

__device__ __forceinline__ LineFitPoint ZeroLineFitPoint(uint32_t blob_index) {
  LineFitPoint result;
  result.Mxx = 0;
  result.Myy = 0;
  result.Mxy = 0;
  result.Mx = 0;
  result.My = 0;
  result.W = 0;
  result.blob_index = blob_index;
  return result;
}

// Carries the sum of a blob's line fit points from one chunk of them to the
// next.
struct LineFitPointPrefix {
  __device__ __forceinline__ LineFitPoint
  operator()(const LineFitPoint &chunk_sum) {
    const LineFitPoint prefix = sum;
    sum = SumLineFitPoints()(sum, chunk_sum);
    return prefix;
  }

  LineFitPoint sum;
};

// Writes the next kThreads of a blob's sorted points, where this thread's
// point goes at position, and the prefix sums of their line fit points.  Every
// thread in the block calls this for each chunk.
template <int kThreads>
__device__ __forceinline__ void WriteSortedChunk(
    typename LineFitPointScan<kThreads>::TempStorage &scan_storage,
    LineFitPointPrefix &prefix, IndexPoint point, uint32_t position,
    uint32_t count, const TransformLineFitPoint &transform,
    IndexPoint *sorted_points, LineFitPoint *line_fit_points) {
  LineFitPoint line_fit_point = ZeroLineFitPoint(prefix.sum.blob_index);
  if (position < count) {
    sorted_points[position] = point;
    line_fit_point = transform(point);
  }
  LineFitPointScan<kThreads>(scan_storage)
      .InclusiveScan(line_fit_point, line_fit_point, SumLineFitPoints(),
                     prefix);
  if (position < count) {
    line_fit_points[position] = line_fit_point;
  }
  // The next chunk reuses scan_storage.
  __syncthreads();
}

// Returns the key that orders a blob's points with the same angle: dxy, then
// base_y, then base_x.  No two points share it, so sorting by angle and then
// this does not depend on the order the blob has its points in.
__device__ __forceinline__ uint32_t AngleTieKey(IndexPoint point) {
  return (point.dxy() << 20) | (point.base_y() << 10) | point.base_x();
}

// Puts each run of a blob's sorted points with the same angle in AngleTieKey()
// order, and redoes the line fit prefix sums inside the run.  Every thread in
// the block calls this after writing the blob's points and passing a barrier.
template <int kThreads, int kPointsPerThread>
__device__ __forceinline__ void FixAngleTies(
    uint32_t blob_index, uint32_t count, const TransformLineFitPoint &transform,
    IndexPoint *sorted_points, LineFitPoint *line_fit_points) {
  // Reading the angles back keeps the sort's registers free for the scans.
  // Each lane reads one, and gets the next from the next lane.
  const uint32_t lane = threadIdx.x % 32;
  bool has_tie = false;
#pragma unroll
  for (int i = 0; i < kPointsPerThread; ++i) {
    const uint32_t position = i * kThreads + threadIdx.x;
    const uint32_t theta =
        position < count ? sorted_points[position].theta() : 0;
    uint32_t next_theta = __shfl_down_sync(0xffffffffu, theta, 1);
    if (lane == 31 && position + 1 < count) {
      next_theta = sorted_points[position + 1].theta();
    }
    has_tie |= position + 1 < count && next_theta == theta;
  }
  // Ties are rare, and most blobs have none.
  if (!__syncthreads_or(has_tie)) {
    return;
  }

  // Every run is found before any moves, since finding one reads the points
  // on both sides of it.  run_end is 0 where no run starts.
  uint32_t run_end[kPointsPerThread];
#pragma unroll
  for (int i = 0; i < kPointsPerThread; ++i) {
    const uint32_t start = i * kThreads + threadIdx.x;
    run_end[i] = 0;
    if (start + 1 < count) {
      const uint32_t theta = sorted_points[start].theta();
      if (sorted_points[start + 1].theta() == theta &&
          (start == 0 || sorted_points[start - 1].theta() != theta)) {
        uint32_t end = start + 2;
        while (end < count && sorted_points[end].theta() == theta) {
          ++end;
        }
        run_end[i] = end;
      }
    }
  }
  __syncthreads();

#pragma unroll
  for (int i = 0; i < kPointsPerThread; ++i) {
    const uint32_t start = i * kThreads + threadIdx.x;
    const uint32_t end = run_end[i];
    if (end == 0) {
      continue;
    }
    for (uint32_t j = start + 1; j < end; ++j) {
      const IndexPoint point = sorted_points[j];
      const uint32_t key = AngleTieKey(point);
      uint32_t k = j;
      for (; k > start && AngleTieKey(sorted_points[k - 1]) > key; --k) {
        sorted_points[k] = sorted_points[k - 1];
      }
      sorted_points[k] = point;
    }
    // The run's last prefix sum covers all of it in any order, so it stays,
    // and the run after this one may read it.
    LineFitPoint sum =
        start == 0 ? ZeroLineFitPoint(blob_index) : line_fit_points[start - 1];
    for (uint32_t j = start; j + 1 < end; ++j) {
      sum = SumLineFitPoints()(sum, transform(sorted_points[j]));
      line_fit_points[j] = sum;
    }
  }
}

// Sorts a blob's points by angle with a radix sort, then writes them and their
// line fit points.  Points with the same angle end up in AngleTieKey() order.
//
// With kScanInChunks, the block scans kThreads sorted points at a time, which
// keeps the writes coalesced.  Otherwise each thread sums its own
// kPointsPerThread consecutive points and the block scans once, which is
// faster for many points per thread.
template <int kThreads, int kPointsPerThread, bool kScanInChunks>
__device__ __forceinline__ void RadixSortAndScanBlob(
    typename cub::BlockRadixSort<uint32_t, kThreads, kPointsPerThread,
                                 uint32_t>::TempStorage &sort_storage,
    typename LineFitPointScan<kThreads>::TempStorage &scan_storage,
    uint32_t blob_index, const IndexPoint *points, uint32_t count,
    const TransformLineFitPoint &transform, IndexPoint *sorted_points,
    LineFitPoint *line_fit_points) {
  typedef cub::BlockRadixSort<uint32_t, kThreads, kPointsPerThread, uint32_t>
      BlockRadixSort;
  uint32_t theta[kPointsPerThread];
  uint32_t point_bits[kPointsPerThread];
#pragma unroll
  for (int i = 0; i < kPointsPerThread; ++i) {
    const uint32_t index = threadIdx.x * kPointsPerThread + i;
    if (index < count) {
      const IndexPoint point = points[index];
      theta[i] = point.theta();
      point_bits[i] = point.point_bits();
    } else {
      // Sorts after every point, since no angle is bigger and the padding
      // comes last.
      theta[i] = 0xffffffffu;
      point_bits[i] = 0;
    }
  }
  if constexpr (kScanInChunks) {
    BlockRadixSort(sort_storage)
        .SortBlockedToStriped(theta, point_bits, 0, kThetaBits);
  } else {
    BlockRadixSort(sort_storage).Sort(theta, point_bits, 0, kThetaBits);
  }
  // scan_storage may alias sort_storage.
  __syncthreads();

  if constexpr (kScanInChunks) {
    LineFitPointPrefix prefix{ZeroLineFitPoint(blob_index)};
#pragma unroll
    for (int i = 0; i < kPointsPerThread; ++i) {
      if (i * kThreads >= count) {
        break;
      }
      IndexPoint point(blob_index, point_bits[i]);
      point.set_theta(theta[i]);
      WriteSortedChunk<kThreads>(scan_storage, prefix, point,
                                 i * kThreads + threadIdx.x, count, transform,
                                 sorted_points, line_fit_points);
    }
  } else {
    // Only the weights are kept between the two passes over the points, since
    // the moments take too many registers.
    int32_t weight[kPointsPerThread];
    LineFitPoint sum = ZeroLineFitPoint(blob_index);
#pragma unroll
    for (int i = 0; i < kPointsPerThread; ++i) {
      if (threadIdx.x * kPointsPerThread + i < count) {
        IndexPoint point(blob_index, point_bits[i]);
        weight[i] = transform.Weight(point);
        sum = SumLineFitPoints()(
            sum, TransformLineFitPoint::Moments(point, weight[i]));
      }
    }
    LineFitPointScan<kThreads>(scan_storage)
        .ExclusiveScan(sum, sum, ZeroLineFitPoint(blob_index),
                       SumLineFitPoints());
#pragma unroll
    for (int i = 0; i < kPointsPerThread; ++i) {
      const uint32_t position = threadIdx.x * kPointsPerThread + i;
      if (position < count) {
        IndexPoint point(blob_index, point_bits[i]);
        point.set_theta(theta[i]);
        sum = SumLineFitPoints()(
            sum, TransformLineFitPoint::Moments(point, weight[i]));
        sorted_points[position] = point;
        line_fit_points[position] = sum;
      }
    }
    // The next blob reuses scan_storage.
    __syncthreads();
  }
  // The sort is stable, so it left points with the same angle in the blob's
  // order.
  FixAngleTies<kThreads, kPointsPerThread>(blob_index, count, transform,
                                           sorted_points, line_fit_points);
}

struct SmallBlobStorage {
  union {
    typename cub::BlockRadixSort<uint32_t, kSmallBlobThreads, 2,
                                 uint32_t>::TempStorage radix_sort2;
    typename cub::BlockRadixSort<uint32_t, kSmallBlobThreads, 4,
                                 uint32_t>::TempStorage radix_sort4;
    typename cub::BlockRadixSort<uint32_t, kSmallBlobThreads, 8,
                                 uint32_t>::TempStorage radix_sort8;
    uint64_t rank_keys[kMaxRankSortPoints];
    typename LineFitPointScan<kSmallBlobThreads>::TempStorage scan;
  } temp;
  // The ranked points' keys, in sorted order.
  uint64_t ranked_points[kMaxRankSortPoints];
};

// Sorts a blob's points by angle by ranking each against the others, then
// writes them and their line fit points.  Points with the same angle rank by
// AngleTieKey().
__device__ __forceinline__ void RankSortAndScanBlob(
    SmallBlobStorage &storage, uint32_t blob_index, const IndexPoint *points,
    uint32_t count, const TransformLineFitPoint &transform,
    IndexPoint *sorted_points, LineFitPoint *line_fit_points) {
  static_assert(kSmallBlobThreads == kMaxRankSortPoints);
  const bool has_point = threadIdx.x < count;
  // Every key is unique, so the ranks are too.
  uint64_t key = 0;
  IndexPoint point;
  if (has_point) {
    point = points[threadIdx.x];
    key = (static_cast<uint64_t>(point.theta()) << 32) | AngleTieKey(point);
    storage.temp.rank_keys[threadIdx.x] = key;
  }
  __syncthreads();

  if (has_point) {
    uint32_t rank = 0;
    for (uint32_t j = 0; j < count; ++j) {
      rank += storage.temp.rank_keys[j] < key;
    }
    storage.ranked_points[rank] = point.key;
  }
  // The scan reuses rank_keys' storage.
  __syncthreads();

  LineFitPointPrefix prefix{ZeroLineFitPoint(blob_index)};
  IndexPoint sorted_point;
  if (has_point) {
    sorted_point.key = storage.ranked_points[threadIdx.x];
  }
  WriteSortedChunk<kSmallBlobThreads>(storage.temp.scan, prefix, sorted_point,
                                      threadIdx.x, count, transform,
                                      sorted_points, line_fit_points);
}

// Sorts the points of each small blob in the work list by angle, and prefix
// sums their line fit points, one blob per block.
__global__ void __launch_bounds__(kSmallBlobThreads, 4) SortAndScanSmallBlobs(
    const BlobWorkList *work_list, const IndexPoint *selected_blobs,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    TransformLineFitPoint transform, IndexPoint *sorted_selected_blobs,
    LineFitPoint *line_fit_points) {
  __shared__ SmallBlobStorage storage;
  if (blockIdx.x >= work_list->num_small_blobs) {
    return;
  }
  const uint32_t blob_index = work_list->small_blobs[blockIdx.x];
  const MinMaxExtents extents = selected_extents_device[blob_index].value;
  const uint32_t count = extents.count;
  const IndexPoint *points = selected_blobs + extents.starting_offset;
  IndexPoint *sorted_points = sorted_selected_blobs + extents.starting_offset;
  LineFitPoint *blob_line_fit_points =
      line_fit_points + extents.starting_offset;
  if (count <= kMaxRankSortPoints) {
    RankSortAndScanBlob(storage, blob_index, points, count, transform,
                        sorted_points, blob_line_fit_points);
  } else if (count <= 2 * kSmallBlobThreads) {
    RadixSortAndScanBlob<kSmallBlobThreads, 2, true>(
        storage.temp.radix_sort2, storage.temp.scan, blob_index, points, count,
        transform, sorted_points, blob_line_fit_points);
  } else if (count <= 4 * kSmallBlobThreads) {
    RadixSortAndScanBlob<kSmallBlobThreads, 4, true>(
        storage.temp.radix_sort4, storage.temp.scan, blob_index, points, count,
        transform, sorted_points, blob_line_fit_points);
  } else {
    static_assert(8 * kSmallBlobThreads == kMaxSmallBlobPoints);
    RadixSortAndScanBlob<kSmallBlobThreads, 8, true>(
        storage.temp.radix_sort8, storage.temp.scan, blob_index, points, count,
        transform, sorted_points, blob_line_fit_points);
  }
}

union BigBlobStorage {
  typename cub::BlockRadixSort<uint32_t, kBigBlobThreads,
                               kBigBlobPointsPerThread, uint32_t>::TempStorage
      radix_sort;
  typename LineFitPointScan<kBigBlobThreads>::TempStorage scan;
};
// Kernels get at most 48 KiB of static shared memory.
static_assert(sizeof(BigBlobStorage) <= 48 * 1024);
static_assert(sizeof(SmallBlobStorage) <= 48 * 1024);

// Sorts the points of each big blob in the work list by angle, and prefix sums
// their line fit points, looping over the blobs.
__global__ void __launch_bounds__(kBigBlobThreads) SortAndScanBigBlobs(
    const BlobWorkList *work_list, const IndexPoint *selected_blobs,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    TransformLineFitPoint transform, IndexPoint *sorted_selected_blobs,
    LineFitPoint *line_fit_points) {
  __shared__ BigBlobStorage storage;
  const uint32_t num_big_blobs = work_list->num_big_blobs;
  for (uint32_t i = blockIdx.x; i < num_big_blobs; i += gridDim.x) {
    const uint32_t blob_index = work_list->big_blobs[i];
    const MinMaxExtents extents = selected_extents_device[blob_index].value;
    RadixSortAndScanBlob<kBigBlobThreads, kBigBlobPointsPerThread, false>(
        storage.radix_sort, storage.scan, blob_index,
        selected_blobs + extents.starting_offset, extents.count, transform,
        sorted_selected_blobs + extents.starting_offset,
        line_fit_points + extents.starting_offset);
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

  // Threshold the image.
  float neon_threshold_ms = 0;
  if (absl::GetFlag(FLAGS_use_neon)) {
    // We are being asked to do this on the CPU.  Use the CPU source image, and
    // put it into unified memory so it gets moved over to the GPU in the
    // background.
    const aos::monotonic_clock::time_point before_threshold =
        aos::monotonic_clock::now();
    threshold_->ThresholdAndDecimate(
        image, decimated_image_device_.get(), thresholded_image_device_.get(),
        tag_detector_->qtp.min_white_black_diff, &stream_);
    neon_threshold_ms =
        float_milli(aos::monotonic_clock::now() - before_threshold).count();

    start_.Record(&stream_);
    record_stage(after_image_memcpy_to_device_, &stream_);
  } else {
    start_.Record(&stream_);
    // If the image isn't already in device memory, copy it there and update the
    // pointer.
    if (image_device == nullptr) {
      color_image_device_.MemcpyAsyncFrom(image, &stream_);
      image_device = color_image_device_.get();
    }
    record_stage(after_image_memcpy_to_device_, &stream_);

    // Now, threshold on the GPU fully.
    threshold_->ThresholdAndDecimate(
        image_device, decimated_image_device_.get(),
        thresholded_image_device_.get(),
        tag_detector_->qtp.min_white_black_diff, &stream_);
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

  // Unionfind the image.
  LabelImage(
      ToGpuImage(thresholded_image_device_), ToGpuImage(union_markers_device_),
      ToGpuImage(union_markers_size_device_), label_tile_roots_device_.get(),
      label_forest_device_.get(), stream_.get(),
      stage_timing_ ? std::span<CudaEvent>(after_labeling_kernel_)
                    : std::span<CudaEvent>());

  record_stage(after_unionfinding_, &stream_);

  ABSL_CHECK((width_ % 8) == 0);
  ABSL_CHECK((height_ % 8) == 0);

  size_t decimated_width = width_ / 2;
  size_t decimated_height = height_ / 2;

  // TODO(austin): Tune for the global shutter camera.
  // 1280 -> 2 * 128 * 5
  // 720 -> 2 * 8 * 5 * 9

  // Longest april tag will be the full perimeter of the image.  Each point
  // results in 2 neighbor points, 1 straight, and one at 45 degrees.  But, we
  // are in decimated space here, and width_ and height_ are in full image
  // space.  And there are 2 sides to each rectangle...
  //
  // Aprilrobotics has a *3 instead of a *2 here since they have duplicated
  // points in their list at this stage.
  const apriltag_size_t max_april_tag_perimeter = 2 * (width_ + height_);
  // SortAndScanBigBlobs holds a whole blob in one block.
  ABSL_CHECK_LE(
      max_april_tag_perimeter,
      static_cast<apriltag_size_t>(kBigBlobThreads * kBigBlobPointsPerThread));

  // Find the points between blobs.
  {
    constexpr int kBlockWidth = 32;
    constexpr int kBlockHeight = 8;
    dim3 threads(kBlockWidth, kBlockHeight, 1);
    // Overlap 1 on each side in x, and 1 in y.
    dim3 blocks((decimated_width + threads.x - 3) / (threads.x - 2),
                (decimated_height + threads.y - 2) / (threads.y - 1), 1);

    //  Make sure we fit in our mask.
    ABSL_CHECK_LT(width_ * height_, static_cast<size_t>(1 << 22));

    BlobDiff<kBlockWidth, kBlockHeight><<<blocks, threads, 0, stream_.get()>>>(
        thresholded_image_device_.get(), union_markers_device_.get(),
        union_markers_size_device_.get(), blob_pairs_device_.get(),
        boundary_points_device_.get(), decimated_width, decimated_height);
    MaybeCheckAndSynchronize("BlobDiff");
  }
  record_stage(after_diff_, &stream_);

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

  {
    // Sort each selected blob's points by angle around the blob, and prefix
    // sum their line fit points, in a block per blob.  Blobs are contiguous
    // and in blob order in selected_blobs_device_, so this matches a global
    // sort by (blob, angle, AngleTieKey()), and the host never needs the
    // number of selected points.
    BuildBlobWorkList<<<1, kWorkListThreads, 0, stream_.get()>>>(
        selected_extents_device_.get(), blob_work_list_device_.get());
    MaybeCheckAndSynchronize("BuildBlobWorkList");
    TransformLineFitPoint transform(decimated_image_device_.get(), width_ / 2,
                                    height_ / 2);
    SortAndScanSmallBlobs<<<kMaxBlobs, kSmallBlobThreads, 0, stream_.get()>>>(
        blob_work_list_device_.get(), selected_blobs_device_.get(),
        selected_extents_device_.get(), transform,
        sorted_selected_blobs_device_.get(), line_fit_points_device_.get());
    MaybeCheckAndSynchronize("SortAndScanSmallBlobs");
    record_stage(after_small_blob_sort_, &stream_);
    SortAndScanBigBlobs<<<kBigBlobBlocks, kBigBlobThreads, 0, stream_.get()>>>(
        blob_work_list_device_.get(), selected_blobs_device_.get(),
        selected_extents_device_.get(), transform,
        sorted_selected_blobs_device_.get(), line_fit_points_device_.get());
    MaybeCheckAndSynchronize("SortAndScanBigBlobs");
  }
  record_stage(after_big_blob_sort_, &stream_);

  FitLines(line_fit_points_device_.get(), num_selected_blobs_device_.get(),
           selected_extents_device_.get(), errs_device_.get(),
           filtered_errs_device_.get(), filtered_is_local_peak_device_.get(),
           &stream_);
  record_stage(after_line_filter_, &stream_);

  // The host reads the quads where this writes them, so this is Detect()'s
  // only wait on the GPU.
  apriltag::FitQuads(
      filtered_is_local_peak_device_.get(), line_fit_points_device_.get(),
      selected_extents_device_.get(), kMaxBlobs, max_april_tag_perimeter,
      tag_detector_->qtp.max_nmaxima, tag_detector_->qtp.max_line_fit_mse,
      tag_detector_->qtp.cos_critical_rad, fit_quads_host_.device_ptr(),
      fit_quad_valid_host_.device_ptr(), &stream_);
  MaybeCheckAndSynchronize("FitQuads");
  after_quad_fit_.Record(&stream_);
  after_quad_fit_.Synchronize(host_wait_spin_);

  // selected_extents_device_ and the 12 bit blob index in IndexPoint only hold
  // kMaxBlobs blobs, so the GPU filtered every blob out of an overflowing
  // frame.
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
  const std::chrono::nanoseconds gpu_time = after_quad_fit_.ElapsedTime(start_);
  gpu_time_ms_ = float_milli(gpu_time).count();
  host_decode_time_ms_ = float_milli(end_time - before_fit_quads).count();
  if (stage_timing_) {
    // Every event completed before after_quad_fit_, which the host already
    // waited on.
    stage_times_ms_.clear();
    if (absl::GetFlag(FLAGS_use_neon)) {
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

  ABSL_VLOG(1) << "Overall " << gpu_time_ms_ << "ms, "
               << float_milli(end_time - start_time).count() << "ms on host, "
               << (1000.0 / gpu_time_ms_) << "hz";
  // Average.  Skip the first one as the kernel is warming up and is slower.
  if (!first_) {
    ++execution_count_;
    execution_duration_ += gpu_time;
    ABSL_VLOG(1)
        << "Average overall "
        << float_milli(execution_duration_ / execution_count_).count() << "ms, "
        << (1000.0 /
            float_milli(execution_duration_ / execution_count_).count())
        << "hz";
  }

  first_ = false;
}

}  // namespace frc::apriltag

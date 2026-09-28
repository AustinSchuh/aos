#include "frc/orin/apriltag.h"

#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/transform_iterator.h>

#include <cub/block/block_radix_sort.cuh>
#include <cub/block/block_scan.cuh>
#include <cub/device/device_copy.cuh>
#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_reduce.cuh>
#include <cub/device/device_run_length_encode.cuh>
#include <cub/device/device_scan.cuh>
#include <cub/device/device_segmented_sort.cuh>
#include <cub/device/device_select.cuh>
#include <cub/iterator/discard_output_iterator.cuh>
#include <cub/iterator/transform_input_iterator.cuh>
#include <numbers>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "third_party/apriltag/common/g2d.h"

#include "aos/time/time.h"
#include "frc/orin/labeling_allegretti_2019_BKE.h"
#include "frc/orin/threshold.h"
#include "frc/orin/transform_output_iterator.h"

namespace numbers = std::numbers;

ABSL_FLAG(bool, use_neon, false, "Use NEON optimized threshold");

namespace frc::apriltag {
namespace {

typedef std::chrono::duration<float, std::milli> float_milli;

// Returns true if the QuadBoundaryPoint is nonzero.
struct NonZero {
  __host__ __device__ __forceinline__ bool operator()(
      const QuadBoundaryPoint &a) const {
    return a.nonzero();
  }
};

// Always returns true (only used for scratch space calcs).
template <typename T>
struct True {
  __host__ __device__ __forceinline__ bool operator()(const T &) const {
    return true;
  }
};

// Computes and returns the scratch space needed for DeviceRadixSort::SortKeys
// of the provided key with the provided number of elements.
template <typename T>
static size_t RadixSortScratchSpace(size_t elements) {
  size_t temp_storage_bytes = 0;
  QuadBoundaryPointDecomposer decomposer;
  cub::DeviceRadixSort::SortKeys(nullptr, temp_storage_bytes, (T *)(nullptr),
                                 (T *)(nullptr), elements, decomposer);
  return temp_storage_bytes;
}

// Computes and returns the scratch space needed for DeviceSelect::If of the
// provided type Tin, to be written to the provided type Tout, for the provided
// number of elements.  num_markers is the device pointer used to hold the
// selected number of elements.
template <typename Tin, typename Tout>
static size_t DeviceSelectIfScratchSpace(size_t elements, int *num_markers) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceSelect::If(nullptr, temp_storage_bytes,
                                   (Tin *)(nullptr), (Tout *)(nullptr),
                                   num_markers, elements, True<Tin>()));
  return temp_storage_bytes;
}

// Always returns the first element (only used for scratch space calcs).
template <typename T>
struct CustomFirst {
  __host__ __device__ __forceinline__ T operator()(const T &a,
                                                   const T & /*b*/) const {
    return a;
  }
};

// Computes and returns the scratch space needed for DeviceReduce::ReduceByKey
// of the provided key K and value V with the provided number of elements.
// NumRunsT and NumItemsT must match the call being sized since they pick the
// offset type cub uses.
template <typename K, typename V, typename NumRunsT = size_t,
          typename NumItemsT = size_t>
static size_t DeviceReduceByKeyScratchSpace(NumItemsT elements) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceReduce::ReduceByKey(
      nullptr, temp_storage_bytes, (K *)(nullptr), (K *)(nullptr),
      (V *)(nullptr), (V *)(nullptr), (NumRunsT *)(nullptr), CustomFirst<V>(),
      elements));
  return temp_storage_bytes;
}

// Computes and returns the scratch space needed for DeviceScan::InclusiveScan
// of the provided value V with the provided number of elements.
template <typename V>
static size_t DeviceScanInclusiveScanScratchSpace(size_t elements) {
  size_t temp_storage_bytes = 0;
  CHECK_CUDA(cub::DeviceScan::InclusiveScan(nullptr, temp_storage_bytes,
                                            (V *)(nullptr), (V *)(nullptr),
                                            CustomFirst<V>(), elements));
  return temp_storage_bytes;
}

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
      union_marker_pair_device_((width / 2 - 2) * (height / 2 - 2) * 4),
      compressed_union_marker_pair_device_(union_marker_pair_device_.size()),
      sorted_union_marker_pair_device_(union_marker_pair_device_.size()),
      extents_device_(union_marker_pair_device_.size()),
      selected_extents_device_(kMaxBlobs),
      selected_blobs_device_(union_marker_pair_device_.size()),
      sorted_selected_blobs_device_(selected_blobs_device_.size()),
      line_fit_points_device_(selected_blobs_device_.size()),
      errs_device_(line_fit_points_device_.size()),
      filtered_errs_device_(line_fit_points_device_.size()),
      filtered_is_local_peak_device_(line_fit_points_device_.size()),
      camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients),
      radix_sort_tmpstorage_device_(RadixSortScratchSpace<QuadBoundaryPoint>(
          sorted_union_marker_pair_device_.size())),
      temp_storage_compressed_union_marker_pair_device_(
          DeviceSelectIfScratchSpace<QuadBoundaryPoint, QuadBoundaryPoint>(
              union_marker_pair_device_.size(),
              num_compressed_union_marker_pair_.device_ptr())),
      temp_storage_bounds_reduce_by_key_device_(
          DeviceReduceByKeyScratchSpace<uint64_t, MinMaxExtents>(
              union_marker_pair_device_.size())),
      temp_storage_compressed_filtered_blobs_device_(
          DeviceSelectIfScratchSpace<IndexPoint, IndexPoint>(
              union_marker_pair_device_.size(),
              num_selected_blobs_device_.get())),
      temp_storage_selected_extents_scan_device_(
          DeviceScanInclusiveScanScratchSpace<
              cub::KeyValuePair<long, MinMaxExtents>>(kMaxBlobs)),
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
  // in flight, and memset_stream_ can while NEON writes these.  Attached to
  // stream_, they only need stream_ to be idle.
  CHECK_CUDA(cudaStreamAttachMemAsync(
      stream_.get(), decimated_image_device_.get(), 0, cudaMemAttachSingle));
  CHECK_CUDA(cudaStreamAttachMemAsync(
      stream_.get(), thresholded_image_device_.get(), 0, cudaMemAttachSingle));
  // The extents scan reads all kMaxBlobs entries each frame and ignores those
  // past the frame's blob pair count, which start out uninitialized.
  CHECK_CUDA(cudaMemsetAsync(extents_device_.get(), 0,
                             kMaxBlobs * sizeof(MinMaxExtents), stream_.get()));
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

namespace {

// Computes a massive image of 4x QuadBoundaryPoint per pixel with a
// QuadBoundaryPoint for each pixel pair which crosses a blob boundary.
template <size_t kBlockWidth, size_t kBlockHeight>
__global__ void BlobDiff(const uint8_t *thresholded_image,
                         const uint32_t *blobs,
                         const uint32_t *union_markers_size,
                         QuadBoundaryPoint *result, apriltag_size_t width,
                         apriltag_size_t height) {
  __shared__ uint32_t temp_blob_storage[kBlockWidth * kBlockHeight];
  __shared__ uint8_t temp_image_storage[kBlockWidth * kBlockHeight];

  // We overlap both directions in X, and only one direction in Y.
  const uint x = blockIdx.x * (blockDim.x - 2) + threadIdx.x;
  const uint y = blockIdx.y * (blockDim.y - 1) + threadIdx.y;

  // Ignore anything outside the image.
  if (x >= width || y >= height || y == 0) {
    return;
  }

  // Compute the location in the image this threads is responsible for.
  const uint global_input_index = x + y * width;
  // And the destination in the temporary storage to save it.
  const uint thread_linear_index = threadIdx.x + blockDim.x * threadIdx.y;

  // Now, load all the data.
  const uint32_t rep0 = temp_blob_storage[thread_linear_index] =
      blobs[global_input_index];
  const uint8_t v0 = temp_image_storage[thread_linear_index] =
      thresholded_image[global_input_index];

  // All threads in the block have now gotten this far so the shared memory is
  // consistent.
  __syncthreads();

  // We've done our job loading things into memory, this pixel is a boundary
  // with the upper and left sides, or covered by the next block.
  if (threadIdx.x == 0) {
    return;
  }

  // We need to search both ways in x, this thread doesn't participate further
  // :(
  if (threadIdx.x == blockDim.x - 1) {
    return;
  }

  if (threadIdx.y == blockDim.y - 1) {
    return;
  }

  // This is the last pixel along the lower and right sides, we don't need to
  // compute it.
  if (x == width - 1 || y == height - 1) {
    return;
  }

  // Place in memory to write the result.
  const uint global_output_index = (x - 1) + (y - 1) * (width - 2);

  // Short circuit 127's and write an empty point out.
  if (v0 == 127 || union_markers_size[rep0] < 25) {
#pragma unroll
    for (size_t point_offset = 0; point_offset < 4; ++point_offset) {
      const apriltag_size_t write_address =
          (width - 2) * (height - 2) * point_offset + global_output_index;
      result[write_address] = QuadBoundaryPoint();
    }
    return;
  }

  uint32_t rep1;
  uint8_t v1;

#define DO_CONN(dx, dy, point_offset)                                    \
  {                                                                      \
    QuadBoundaryPoint cluster_id;                                        \
    const uint x1 = dx + threadIdx.x;                                    \
    const uint y1 = dy + threadIdx.y;                                    \
    const uint thread_linear_index1 = x1 + blockDim.x * y1;              \
    v1 = temp_image_storage[thread_linear_index1];                       \
    rep1 = temp_blob_storage[thread_linear_index1];                      \
    if (v0 + v1 == 255) {                                                \
      if (union_markers_size[rep1] >= 25) {                              \
        if (rep0 < rep1) {                                               \
          cluster_id.set_rep1(rep1);                                     \
          cluster_id.set_rep0(rep0);                                     \
        } else {                                                         \
          cluster_id.set_rep1(rep0);                                     \
          cluster_id.set_rep0(rep1);                                     \
        }                                                                \
        cluster_id.set_base_xy(x, y);                                    \
        cluster_id.set_dxy(point_offset);                                \
        cluster_id.set_black_to_white(v1 > v0);                          \
      }                                                                  \
    }                                                                    \
    const apriltag_size_t write_address =                                \
        (width - 2) * (height - 2) * point_offset + global_output_index; \
    result[write_address] = cluster_id;                                  \
  }

  // We search the following 4 neighbors.
  //      ________
  //      | x | 0 |
  //  -------------
  //  | 3 | 2 | 1 |
  //  -------------
  //
  //  If connection 3 has the same IDs as the connection between blocks 0 and 2,
  //  we will have a duplicate entry.  Detect and don't add it.  This will only
  //  happen if id(x) == id(0) and id(2) == id(3),
  //         or id(x) == id(2) and id(0) ==id(3).

  DO_CONN(1, 0, 0);
  DO_CONN(1, 1, 1);
  DO_CONN(0, 1, 2);
  const uint64_t rep_block_2 = rep1;
  const uint8_t v1_block_2 = v1;

  const uint left_thread_linear_index1 =
      threadIdx.x - 1 + blockDim.x * threadIdx.y;
  const uint8_t v1_block_left = temp_image_storage[left_thread_linear_index1];
  const uint32_t rep_block_left = temp_blob_storage[left_thread_linear_index1];

  // Do the dedup calculation now.
  if (v1_block_left != 127 && v1_block_2 != 127 &&
      v1_block_2 != v1_block_left) {
    if (x != 1 && union_markers_size[rep_block_left] >= 25 &&
        union_markers_size[rep_block_2] >= 25) {
      const uint32_t write_address =
          (width - 2) * (height - 2) * 3 + global_output_index;
      result[write_address] = QuadBoundaryPoint();
      return;
    }
  }

  DO_CONN(-1, 1, 3);
}

// Masks out just the blob ID pair, rep01.
struct MaskRep01 {
  __host__ __device__ __forceinline__ uint64_t
  operator()(const QuadBoundaryPoint &a) const {
    return a.rep01();
  }
};

// Rewrites a QuadBoundaryPoint to an IndexPoint, adding the angle to the
// center.
class RewriteToIndexPoint {
 public:
  RewriteToIndexPoint(MinMaxExtents *extents_device, const size_t *num_extents)
      : blob_finder_(extents_device, num_extents) {}

  __host__ __device__ __forceinline__ IndexPoint
  operator()(cub::KeyValuePair<long, QuadBoundaryPoint> pt) const {
    const apriltag_size_t index = blob_finder_.FindBlobIndex(pt.key);
    IndexPoint result(index, pt.value.point_bits());
    return result;
  }

  BlobExtentsIndexFinder blob_finder_;
};

// Calculates Theta for a given IndexPoint
class AddThetaToIndexPoint {
 public:
  AddThetaToIndexPoint(MinMaxExtents *extents_device, const size_t *num_extents)
      : blob_finder_(extents_device, num_extents) {}
  __host__ __device__ __forceinline__ IndexPoint operator()(IndexPoint a) {
    MinMaxExtents extents = blob_finder_.Get(a.blob_index());
    float theta =
        (atan2f(a.y() - extents.cy(), a.x() - extents.cx()) + numbers::pi) *
        8e6;
    long long int theta_int = llrintf(theta);

    a.set_theta(std::max<long long int>(0, theta_int));
    return a;
  }

 private:
  BlobExtentsIndexFinder blob_finder_;
};

// TODO(austin): Make something which rewrites points on the way back out to
// memory and adds the slope.

// Transforms a QuadBoundaryPoint into a single point extent for Reduce.
struct TransformQuadBoundaryPointToMinMaxExtents {
  __host__ __device__ __forceinline__ MinMaxExtents
  operator()(cub::KeyValuePair<long, QuadBoundaryPoint> pt) const {
    MinMaxExtents result;
    result.min_y = result.max_y = pt.value.y();
    result.min_x = result.max_x = pt.value.x();
    result.starting_offset = pt.key;
    result.count = 1;
    result.pxgx_plus_pygy_sum =
        static_cast<int64_t>(pt.value.x()) * pt.value.gx() +
        static_cast<int64_t>(pt.value.y()) * pt.value.gy();
    result.gx_sum = pt.value.gx();
    result.gy_sum = pt.value.gy();
    return result;
  }
};

// Reduces 2 extents by tracking the range and updating the offset and count
// accordingly.
struct QuadBoundaryPointExtents {
  __host__ __device__ __forceinline__ MinMaxExtents
  operator()(const MinMaxExtents &a, const MinMaxExtents &b) const {
    MinMaxExtents result;
    result.min_x = std::min(a.min_x, b.min_x);
    result.max_x = std::max(a.max_x, b.max_x);
    result.min_y = std::min(a.min_y, b.min_y);
    result.max_y = std::max(a.max_y, b.max_y);
    // And the actual start is the first of the points.
    result.starting_offset = std::min(a.starting_offset, b.starting_offset);
    // We want to count everything.
    result.count = a.count + b.count;
    result.pxgx_plus_pygy_sum = a.pxgx_plus_pygy_sum + b.pxgx_plus_pygy_sum;
    result.gx_sum = a.gx_sum + b.gx_sum;
    result.gy_sum = a.gy_sum + b.gy_sum;
    return result;
  }
};

class NonzeroBlobs {
 public:
  __host__ __device__
  NonzeroBlobs(const cub::KeyValuePair<long, MinMaxExtents> *extents_device)
      : extents_device_(extents_device) {}

  __host__ __device__ __forceinline__ bool operator()(
      const IndexPoint &a) const {
    return a.blob_index() < GpuDetector::kMaxBlobs &&
           extents_device_[a.blob_index()].value.count > 0;
  }

 private:
  const cub::KeyValuePair<long, MinMaxExtents> *extents_device_;
};

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

// Class to zero out the count (and clear the starting offset) for each extents
// which is going to be filtered out.  Used in conjunction with SumPoints to
// update zero sized blobs.
//
// The scan covers all kMaxBlobs entries so the host never needs the blob pair
// count, and this zeros the ones past *num_quads_device.  Every blob is
// filtered out if there are more than kMaxBlobs, since there is no good way to
// pick which to keep.
struct TransformZeroFilteredBlobSizes {
 public:
  TransformZeroFilteredBlobSizes(const size_t *num_quads_device,
                                 apriltag_size_t tag_width,
                                 bool reversed_border, bool normal_border,
                                 apriltag_size_t min_cluster_pixels,
                                 apriltag_size_t max_cluster_pixels)
      : num_quads_device_(num_quads_device),
        select_blobs_(nullptr, tag_width, reversed_border, normal_border,
                      min_cluster_pixels, max_cluster_pixels) {}

  __host__ __device__ __forceinline__ cub::KeyValuePair<long, MinMaxExtents>
  operator()(cub::KeyValuePair<long, MinMaxExtents> pt) const {
    const size_t num_quads = *num_quads_device_;
    pt.value.count *= num_quads <= GpuDetector::kMaxBlobs &&
                      static_cast<size_t>(pt.key) < num_quads &&
                      select_blobs_(pt.value);
    pt.value.starting_offset = 0;
    return pt;
  }

 private:
  const size_t *num_quads_device_;
  SelectBlobs select_blobs_;
};

// Reports the frame's blob pair count, and whether it overflowed, to the host.
__global__ void ReportBlobPairs(const size_t *num_quads_device,
                                uint32_t *overflow, uint32_t *num_blob_pairs) {
  const size_t num_quads = *num_quads_device;
  *overflow = num_quads > GpuDetector::kMaxBlobs;
  *num_blob_pairs = num_quads;
}

// Class to implement a custom Scan operator which passes through the previous
// min/max/etc, but re-sums count into starting_offset.  This lets us collapse
// out regions which don't pass the minimum filters.
struct SumPoints {
  __host__ __device__ __forceinline__ cub::KeyValuePair<long, MinMaxExtents>
  operator()(const cub::KeyValuePair<long, MinMaxExtents> &a,
             const cub::KeyValuePair<long, MinMaxExtents> &b) const {
    cub::KeyValuePair<long, MinMaxExtents> result;
    // KeyValuePair's constructor leaves these uninitialized, and nothing reads
    // them after the scan.
    result.value.gx_sum = 0;
    result.value.gy_sum = 0;
    result.value.pxgx_plus_pygy_sum = 0;
    if (a.key < b.key) {
      result.value.min_x = b.value.min_x;
      result.value.min_y = b.value.min_y;
      result.value.max_x = b.value.max_x;
      result.value.max_y = b.value.max_y;
      result.value.count = b.value.count;
      result.key = b.key;
    } else {
      result.value.min_x = a.value.min_x;
      result.value.min_y = a.value.min_y;
      result.value.max_x = a.value.max_x;
      result.value.max_y = a.value.max_y;
      result.value.count = a.value.count;
      result.key = a.key;
    }

    result.value.starting_offset = a.value.starting_offset +
                                   b.value.starting_offset + a.value.count +
                                   b.value.count - result.value.count;

    return result;
  }
};

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

// Sorts a blob's points by angle with a radix sort, then writes them and their
// line fit points.  The sort is stable, so points with the same angle stay in
// the order the blob had them, the way the global (blob, angle) sort left them.
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
  // The sort breaks ties by position in this blocked arrangement, which is the
  // blob's order.
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
// their position in the blob, keeping the order the blob had them.
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
    key = (static_cast<uint64_t>(point.theta()) << 32) | threadIdx.x;
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

  record_stage(memset_start_, &memset_stream_);
  union_markers_size_device_.MemsetAsync(0u, &memset_stream_);
  after_memset_.Record(&memset_stream_);

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

  after_memset_.Synchronize(host_wait_spin_);

  // Unionfind the image.
  record_stage(before_unionfinding_, &stream_);
  LabelImage(ToGpuImage(thresholded_image_device_),
             ToGpuImage(union_markers_device_),
             ToGpuImage(union_markers_size_device_), stream_.get(),
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

  // Compute the unfiltered list of blob pairs and points.
  {
    constexpr size_t kBlockWidth = 32;
    constexpr size_t kBlockHeight = 16;
    dim3 threads(kBlockWidth, kBlockHeight, 1);
    // Overlap 1 on each side in x, and 1 in y.
    dim3 blocks((decimated_width + threads.x - 3) / (threads.x - 2),
                (decimated_height + threads.y - 2) / (threads.y - 1), 1);

    //  Make sure we fit in our mask.
    ABSL_CHECK_LT(width_ * height_, static_cast<size_t>(1 << 22));

    BlobDiff<kBlockWidth, kBlockHeight><<<blocks, threads, 0, stream_.get()>>>(
        thresholded_image_device_.get(), union_markers_device_.get(),
        union_markers_size_device_.get(), union_marker_pair_device_.get(),
        decimated_width, decimated_height);
    MaybeCheckAndSynchronize("BlobDiff");
  }

  // TODO(austin): Can I do the first step of the zero removal in the BlobDiff
  // kernel?

  record_stage(after_diff_, &stream_);

  {
    // Remove empty points which aren't to be considered before sorting to speed
    // things up.
    size_t temp_storage_bytes =
        temp_storage_compressed_union_marker_pair_device_.size();
    NonZero nz;
    CHECK_CUDA(cub::DeviceSelect::If(
        temp_storage_compressed_union_marker_pair_device_.get(),
        temp_storage_bytes, union_marker_pair_device_.get(),
        compressed_union_marker_pair_device_.get(),
        num_compressed_union_marker_pair_.device_ptr(),
        union_marker_pair_device_.size(), nz, stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceSelect::If");
  }

  after_compact_.Record(&stream_);

  int num_compressed_union_marker_pairs;
  {
    // Now, sort just the keys to group like points.
    size_t temp_storage_bytes = radix_sort_tmpstorage_device_.size();
    QuadBoundaryPointDecomposer decomposer;
    after_compact_.Synchronize(host_wait_spin_);
    num_compressed_union_marker_pairs =
        *num_compressed_union_marker_pair_.host_ptr();
    ABSL_CHECK_LE(static_cast<size_t>(num_compressed_union_marker_pairs),
                  union_marker_pair_device_.size());
    CHECK_CUDA(cub::DeviceRadixSort::SortKeys(
        radix_sort_tmpstorage_device_.get(), temp_storage_bytes,
        compressed_union_marker_pair_device_.get(),
        sorted_union_marker_pair_device_.get(),
        num_compressed_union_marker_pairs, decomposer,
        QuadBoundaryPoint::kRepEndBit, QuadBoundaryPoint::kBitsInKey,
        stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceRadixSort::SortKeys");
  }

  record_stage(after_sort_, &stream_);

  {
    // Our next step is to compute the extents and dot product so we can filter
    // blobs.
    cub::ArgIndexInputIterator<QuadBoundaryPoint *> value_index_input_iterator(
        sorted_union_marker_pair_device_.get());
    TransformQuadBoundaryPointToMinMaxExtents min_max;
    cub::TransformInputIterator<MinMaxExtents,
                                TransformQuadBoundaryPointToMinMaxExtents,
                                cub::ArgIndexInputIterator<QuadBoundaryPoint *>>
        value_input_iterator(value_index_input_iterator, min_max);

    // Don't care about the output keys...
    cub::DiscardOutputIterator<uint64_t> key_discard_iterator;

    // Provide a mask to detect keys by rep01()
    MaskRep01 mask;
    cub::TransformInputIterator<uint64_t, MaskRep01, QuadBoundaryPoint *>
        key_input_iterator(sorted_union_marker_pair_device_.get(), mask);

    // Reduction operator.
    QuadBoundaryPointExtents reduce;

    size_t temp_storage_bytes =
        temp_storage_bounds_reduce_by_key_device_.size();
    CHECK_CUDA(cub::DeviceReduce::ReduceByKey(
        temp_storage_bounds_reduce_by_key_device_.get(), temp_storage_bytes,
        key_input_iterator, key_discard_iterator, value_input_iterator,
        extents_device_.get(), num_quads_device_.get(), reduce,
        num_compressed_union_marker_pairs, stream_.get()));
    ReportBlobPairs<<<1, 1, 0, stream_.get()>>>(
        num_quads_device_.get(), &frame_status_.device_ptr()->overflow,
        &frame_status_.device_ptr()->num_blob_pairs);
    record_stage(after_bounds_, &stream_);
  }

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

  {
    // Now that we have the dot products, we need to rewrite the extents for the
    // post-thresholded world so we can find the start and end address of blobs
    // for fitting lines.
    //
    // Clear the size of non-passing extents and the starting offset of all
    // extents.
    cub::ArgIndexInputIterator<MinMaxExtents *> value_index_input_iterator(
        extents_device_.get());
    TransformZeroFilteredBlobSizes rewrite(
        num_quads_device_.get(), min_tag_width_, reversed_border_,
        normal_border_, tag_detector_->qtp.min_cluster_pixels,
        max_april_tag_perimeter);
    cub::TransformInputIterator<cub::KeyValuePair<long, MinMaxExtents>,
                                TransformZeroFilteredBlobSizes,
                                cub::ArgIndexInputIterator<MinMaxExtents *>>
        input_iterator(value_index_input_iterator, rewrite);

    // Sum the counts of everything before us, and update the offset.
    SumPoints sum_points;

    // TODO(justin): Rip the key off when writing.

    // Rewrite the extents to have the starting offset and count match the
    // post-selected values.
    size_t temp_storage_bytes =
        temp_storage_selected_extents_scan_device_.size();

    CHECK_CUDA(cub::DeviceScan::InclusiveScan(
        temp_storage_selected_extents_scan_device_.get(), temp_storage_bytes,
        input_iterator, selected_extents_device_.get(), sum_points, kMaxBlobs,
        stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceScan::InclusiveScan");
  }

  record_stage(after_transform_extents_, &stream_);

  {
    // Now, copy over all points which pass our thresholds.
    cub::ArgIndexInputIterator<QuadBoundaryPoint *> value_index_input_iterator(
        sorted_union_marker_pair_device_.get());
    RewriteToIndexPoint rewrite(extents_device_.get(), num_quads_device_.get());

    cub::TransformInputIterator<IndexPoint, RewriteToIndexPoint,
                                cub::ArgIndexInputIterator<QuadBoundaryPoint *>>
        input_iterator(value_index_input_iterator, rewrite);

    AddThetaToIndexPoint add_theta(extents_device_.get(),
                                   num_quads_device_.get());

    TransformOutputIterator<IndexPoint, IndexPoint, AddThetaToIndexPoint>
        output_iterator(selected_blobs_device_.get(), add_theta);

    NonzeroBlobs select_blobs(selected_extents_device_.get());

    size_t temp_storage_bytes =
        temp_storage_compressed_filtered_blobs_device_.size();

    CHECK_CUDA(cub::DeviceSelect::If(
        temp_storage_compressed_filtered_blobs_device_.get(),
        temp_storage_bytes, input_iterator, output_iterator,
        num_selected_blobs_device_.get(), num_compressed_union_marker_pairs,
        select_blobs, stream_.get()));

    MaybeCheckAndSynchronize("cub::DeviceSelect::If");
  }

  record_stage(after_filter_, &stream_);

  {
    // Sort each selected blob's points by angle around the blob, and prefix
    // sum their line fit points, in a block per blob.  Blobs are contiguous
    // and in blob order in selected_blobs_device_, so this matches a global
    // stable sort by (blob, angle), and the host never needs the number of
    // selected points.
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

  {
    // Every selected point is one of the compacted points, so their number
    // bounds the launch.
    FitLines(line_fit_points_device_.get(), num_selected_blobs_device_.get(),
             num_compressed_union_marker_pairs, selected_extents_device_.get(),
             errs_device_.get(), filtered_errs_device_.get(),
             filtered_is_local_peak_device_.get(), &stream_);
  }
  record_stage(after_line_filter_, &stream_);

  // The host reads the quads where this writes them, so this is the only wait
  // after the filtered points' count.
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
    ABSL_LOG_EVERY_N_SEC(ERROR, 1) << "Dropping a frame with " << num_blob_pairs
                                   << " blob pairs, more than " << kMaxBlobs;
  }

  const aos::monotonic_clock::time_point before_fit_quads =
      aos::monotonic_clock::now();
  UpdateFitQuads();
  AdjustPixelCenters();

  DecodeTags();

  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  // TODO(austin): Bring it back to the CPU and see how good we did.

  // Report out how long things took.

  ABSL_VLOG(1) << "Found " << num_compressed_union_marker_pairs << " items";
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

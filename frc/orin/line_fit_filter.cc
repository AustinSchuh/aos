#include "frc/orin/line_fit_filter.h"

#include <cub/block/block_radix_sort.cuh>
#include <cub/block/block_scan.cuh>
#include <iomanip>
#include <iterator>

#include "absl/log/absl_check.h"

#include "frc/orin/cuda.h"

namespace frc::apriltag {

// Half width of the window FitLineError fits a line to around each point.
constexpr uint32_t kMaxKsz = 20;
// A point's W is at most hypot(255, 255) + 1 (see LineFitPoint).
constexpr uint64_t kMaxPointW = 361;
// FitLineError computes 8 W^2 in uint32.  Blobs have at least 24 points
// (min_cluster_pixels), so ksz is at least 2 and a window holds 2 * ksz + 1
// distinct points.
static_assert(8 * ((2 * kMaxKsz + 1) * kMaxPointW) *
                  ((2 * kMaxKsz + 1) * kMaxPointW) <
              (uint64_t{1} << 32));

__device__ double FitLineError(size_t N, int64_t Mx, int64_t My, int64_t Mxx,
                               int64_t Myy, int64_t Mxy, int64_t W) {
  int64_t Cxx = Mxx * W - Mx * Mx;
  int64_t Cxy = Mxy * W - Mx * My;
  int64_t Cyy = Myy * W - My * My;
  // 8 W^2 is an exact uint32 (see kMaxKsz).  Converting it to float rounds it
  // once, as converting the exact double W * W * 8.0 to float does, without
  // the FP64 pipe.
  const uint32_t w = static_cast<uint32_t>(W);
  const float eight_w_squared = static_cast<float>(w * w * 8u);

  // Pose it as an eigenvalue problem.
  //
  // TODO(austin): Are floats good enough?  Hard to tell what the "right answer"
  // actually is...
  float eig_small =
      ((Cxx + Cyy) - std::hypotf((Cxx - Cyy), 2 * Cxy)) / eight_w_squared;
  return N * eig_small;
}

namespace {

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

// Blobs with up to this many points are sorted by ranking every point against
// the others, which takes fewer steps than a radix sort's passes.
constexpr uint32_t kMaxRankSortPoints = 256;
constexpr int kTinyBlobThreads = kMaxTinyBlobPoints;
constexpr int kSmallBlobThreads = 256;
constexpr int kBigBlobThreads = 512;
constexpr int kBigBlobPointsPerThread = 12;
// A frame has about 1.5 big blobs.
constexpr int kBigBlobBlocks = 8;
constexpr int kWorkListThreads = 1024;
// The width of IndexPoint::theta().
constexpr int kThetaBits = 28;

static_assert(IndexPoint::kMaxBlobs % kWorkListThreads == 0);
// BuildBlobWorkList counts the three kinds of blob in 16 bit fields of one sum.
static_assert(IndexPoint::kMaxBlobs < (1 << 16));

// Lists the selected blobs with points, split by size, in blob order, and
// marks the rest invalid.
__global__ void __launch_bounds__(kWorkListThreads)
    BuildBlobWorkList(BlobQuadFitArgs args) {
  constexpr int kBlobsPerThread = IndexPoint::kMaxBlobs / kWorkListThreads;
  typedef cub::BlockScan<uint64_t, kWorkListThreads> BlockScan;
  __shared__ typename BlockScan::TempStorage scan_storage;

  // Tiny blobs count in bits 0-15, small ones in bits 16-31, and big ones in
  // bits 32-47.
  constexpr uint64_t kTinyBlob = 1;
  constexpr uint64_t kSmallBlob = uint64_t{1} << 16;
  constexpr uint64_t kBigBlob = uint64_t{1} << 32;
  uint64_t kind[kBlobsPerThread];
#pragma unroll
  for (int i = 0; i < kBlobsPerThread; ++i) {
    const uint32_t blob = threadIdx.x * kBlobsPerThread + i;
    const uint32_t count = args.selected_extents[blob].value.count;
    if (count == 0) {
      kind[i] = 0;
      // No quad fit block sees it.
      args.fit_quad_valid[blob] = false;
    } else if (count <= kMaxTinyBlobPoints) {
      kind[i] = kTinyBlob;
    } else if (count <= kMaxSmallBlobPoints) {
      kind[i] = kSmallBlob;
    } else {
      kind[i] = kBigBlob;
    }
  }
  uint64_t position[kBlobsPerThread];
  uint64_t total;
  BlockScan(scan_storage).ExclusiveSum(kind, position, total);
#pragma unroll
  for (int i = 0; i < kBlobsPerThread; ++i) {
    const uint16_t blob = threadIdx.x * kBlobsPerThread + i;
    if (kind[i] == kTinyBlob) {
      args.work_list->tiny_blobs[position[i] & 0xffff] = blob;
    } else if (kind[i] == kSmallBlob) {
      args.work_list->small_blobs[(position[i] >> 16) & 0xffff] = blob;
    } else if (kind[i] == kBigBlob) {
      args.work_list->big_blobs[(position[i] >> 32) & 0xffff] = blob;
    }
  }
  if (threadIdx.x == 0) {
    args.work_list->num_tiny_blobs = total & 0xffff;
    args.work_list->num_small_blobs = (total >> 16) & 0xffff;
    args.work_list->num_big_blobs = (total >> 32) & 0xffff;
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

// Shared memory for RankSortAndScanBlob.
template <int kThreads>
struct RankSortStorage {
  union {
    uint64_t rank_keys[kThreads];
    typename LineFitPointScan<kThreads>::TempStorage scan;
  } temp;
  // The ranked points' keys, in sorted order.
  uint64_t ranked_points[kThreads];
};

struct SmallBlobStorage {
  union {
    typename cub::BlockRadixSort<uint32_t, kSmallBlobThreads, 2,
                                 uint32_t>::TempStorage radix_sort2;
    typename cub::BlockRadixSort<uint32_t, kSmallBlobThreads, 4,
                                 uint32_t>::TempStorage radix_sort4;
    typename cub::BlockRadixSort<uint32_t, kSmallBlobThreads, 8,
                                 uint32_t>::TempStorage radix_sort8;
    typename LineFitPointScan<kSmallBlobThreads>::TempStorage scan;
    RankSortStorage<kSmallBlobThreads> rank_sort;
  } temp;
};

// Sorts a blob's points by angle by ranking each against the others, then
// writes them and their line fit points.  Points with the same angle rank by
// AngleTieKey().  The blob has at most kThreads points.
template <int kThreads>
__device__ __forceinline__ void RankSortAndScanBlob(
    RankSortStorage<kThreads> &storage, uint32_t blob_index,
    const IndexPoint *points, uint32_t count,
    const TransformLineFitPoint &transform, IndexPoint *sorted_points,
    LineFitPoint *line_fit_points) {
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
  WriteSortedChunk<kThreads>(storage.temp.scan, prefix, sorted_point,
                             threadIdx.x, count, transform, sorted_points,
                             line_fit_points);
}

union BigBlobStorage {
  typename cub::BlockRadixSort<uint32_t, kBigBlobThreads,
                               kBigBlobPointsPerThread, uint32_t>::TempStorage
      radix_sort;
  typename LineFitPointScan<kBigBlobThreads>::TempStorage scan;
};

// Returns n choose k as a constexpr.
constexpr uint Binomial(uint n, uint k) {
  if (n < k) {
    return 0;
  }

  uint m = std::min(k, n - k);
  uint result = 1;

  for (uint i = 0; i < m; ++i) {
    result *= (n - i);
    result /= (i + 1);
  }

  return result;
}

constexpr int kNMaxima = 10;

// Returns the sum of binomials for CumulativeSumsM0.
template <size_t nmaxima, size_t N>
constexpr uint SumBinomial(uint i, uint start = 0) {
  size_t result = 0;
  for (uint m0 = start; m0 <= i; ++m0) {
    result += Binomial(nmaxima - m0 - 1, N);
  }
  return result;
}

// Returns the cumulative number of indices consumed by incrementing each m0
// index.
constexpr std::array<uint, 7> CumulativeSumsM0() {
  constexpr size_t nmaxima = kNMaxima;
  return {
      SumBinomial<nmaxima, 3>(0, 0), SumBinomial<nmaxima, 3>(1, 0),
      SumBinomial<nmaxima, 3>(2, 0), SumBinomial<nmaxima, 3>(3, 0),
      SumBinomial<nmaxima, 3>(4, 0), SumBinomial<nmaxima, 3>(5, 0),
      SumBinomial<nmaxima, 3>(6, 0),
  };
}

// Returns the number of indices consumed by each m1 index.  [0] is how many
// you'd consume if m0 and m1 were both as small as they could be.  To handle m0
// > 0, increment your index.
constexpr std::array<uint, 7> BinomialM1() {
  constexpr size_t nmaxima = kNMaxima;
  return {
      Binomial(nmaxima - 2, 2), Binomial(nmaxima - 3, 2),
      Binomial(nmaxima - 4, 2), Binomial(nmaxima - 5, 2),
      Binomial(nmaxima - 6, 2), Binomial(nmaxima - 7, 2),
      Binomial(nmaxima - 8, 2),
  };
}

// Returns the m0 for the provided global index.
constexpr __host__ __device__ const cuda::std::pair<uint, uint> FindM0(uint i) {
  uint m0 = 0;
  uint last = 0;
  while (true) {
    uint next = CumulativeSumsM0()[m0];
    if (i < next) {
      return cuda::std::make_pair(m0, i - last);
    }
    last = next;
    ++m0;
  }
}

// Returns the m1 for the provided m0 and global index remainder.
constexpr __host__ __device__ const cuda::std::pair<uint, uint> FindM1(uint m0,
                                                                       uint i) {
  uint m1 = m0;
  while (true) {
    uint next = BinomialM1()[m1];
    if (i < next || m1 == kNMaxima - 4) {
      return cuda::std::make_pair(m1, i);
    }
    i -= next;
    ++m1;
  }
}

// Returns the m2 for the provided m1 and global index remainder.
constexpr __host__ __device__ const cuda::std::pair<uint, uint> FindM2(uint m1,
                                                                       uint i) {
  uint m2 = m1;
  while (true) {
    uint next = kNMaxima - m2 - 1 - 1;
    if (i < next || m2 == kNMaxima - 3) {
      return cuda::std::make_pair(m2, i);
    }
    i -= next;
    ++m2;
  }
}

// Unrank(), for use in constant expressions.
constexpr std::tuple<uint, uint, uint, uint> ConstexprUnrank(uint i) {
  uint m0 = 0;
  uint m1;
  uint m2;
  uint m3;

  auto result0 = FindM0(i);
  m0 = result0.first;
  i = result0.second;

  auto result1 = FindM1(m0, result0.second);
  m1 = result1.first + 1;

  i = result1.second;
  auto result2 = FindM2(m1, result1.second);
  m2 = result2.first + 1;

  m3 = m2 + result2.second + 1;
  return std::make_tuple(m0, m1, m2, m3);
}

constexpr std::array<uint32_t, MaxRankedIndex()> MakePackedUnrank() {
  std::array<uint32_t, MaxRankedIndex()> result{};
  for (uint i = 0; i < MaxRankedIndex(); ++i) {
    const std::tuple<uint, uint, uint, uint> ms = ConstexprUnrank(i);
    result[i] = std::get<0>(ms) | std::get<1>(ms) << 8 | std::get<2>(ms) << 16 |
                std::get<3>(ms) << 24;
  }
  return result;
}

}  // namespace

__host__ __device__ std::tuple<uint, uint, uint, uint> Unrank(uint i) {
  return ConstexprUnrank(i);
}

std::array<uint32_t, MaxRankedIndex()> PackedUnrank() {
  return MakePackedUnrank();
}

__device__ __forceinline__ LineFitMoments
ReadMoments(const LineFitPoint *line_fit_points_device, size_t blob_point_count,
            size_t index0, size_t index1) {
  LineFitMoments result;

  if (index0 < index1) {
    result.N = index1 - index0 + 1;

    LineFitPoint lf1 = line_fit_points_device[index1];

    result.Mx = lf1.Mx;
    result.My = lf1.My;
    result.Mxx = lf1.Mxx;
    result.Mxy = lf1.Mxy;
    result.Myy = lf1.Myy;
    result.W = lf1.W;

    if (index0 > 0) {
      LineFitPoint lf0 = line_fit_points_device[index0 - 1];

      result.Mx -= lf0.Mx;
      result.My -= lf0.My;
      result.Mxx -= lf0.Mxx;
      result.Mxy -= lf0.Mxy;
      result.Myy -= lf0.Myy;
      result.W -= lf0.W;
    }
  } else {
    // index0 > index1, e.g. [15, 2]. Wrap around.
    LineFitPoint lf0 = line_fit_points_device[index0 - 1];
    LineFitPoint lfsz = line_fit_points_device[blob_point_count - 1];

    result.Mx = lfsz.Mx - lf0.Mx;
    result.My = lfsz.My - lf0.My;
    result.Mxx = lfsz.Mxx - lf0.Mxx;
    result.Mxy = lfsz.Mxy - lf0.Mxy;
    result.Myy = lfsz.Myy - lf0.Myy;
    result.W = lfsz.W - lf0.W;

    LineFitPoint lf1 = line_fit_points_device[index1];

    result.Mx += lf1.Mx;
    result.My += lf1.My;
    result.Mxx += lf1.Mxx;
    result.Mxy += lf1.Mxy;
    result.Myy += lf1.Myy;
    result.W += lf1.W;

    result.N = blob_point_count - index0 + index1 + 1;
  }
  return result;
}

__device__ void FitLine(LineFitMoments moments, double *lineparam01,
                        double *lineparam23, double *err, double *mse) {
  int64_t Cxx = moments.Mxx * moments.W - static_cast<int64_t>(moments.Mx) *
                                              static_cast<int64_t>(moments.Mx);
  int64_t Cxy = moments.Mxy * moments.W - static_cast<int64_t>(moments.Mx) *
                                              static_cast<int64_t>(moments.My);
  int64_t Cyy = moments.Myy * moments.W - static_cast<int64_t>(moments.My) *
                                              static_cast<int64_t>(moments.My);

  // Pose it as an eigenvalue problem.
  //
  // TODO(austin): Are floats good enough?  Hard to tell what the "right answer"
  // actually is...
  const float hypot_cached = std::hypotf((Cxx - Cyy), 2 * Cxy);
  // A blob's W is at most kMaxBlobPoints * 361, so 8 W^2 is an integer below
  // 2^53.  Converting it straight to float rounds it once, exactly as
  // converting it to double (exact) and then to float would, without the FP64
  // pipe.
  const float eight_w_squared = static_cast<float>(
      static_cast<int64_t>(moments.W) * static_cast<int64_t>(moments.W) * 8);
  const float eig_small = (Cxx + Cyy - hypot_cached) / eight_w_squared;

  if (lineparam01) {
    lineparam01[0] =
        static_cast<float>(moments.Mx) / static_cast<float>(moments.W * 2);
    lineparam01[1] =
        static_cast<float>(moments.My) / static_cast<float>(moments.W * 2);
  }
  if (lineparam23) {
    // These don't match the originals at all, but the math should come out
    // right.  n{xy}{12} end up being multiplied by 8 W^2, and we compare the
    // square but use hypot on nx, ny directly.  (and let the W^2 term come out
    // as common to both the hypot and nxy terms.)
    const float nx1 = static_cast<float>(Cxx - Cyy) - hypot_cached;
    const float ny1 = 2 * Cxy;
    const float M1 = nx1 * nx1 + ny1 * ny1;
    const float nx2 = 2 * Cxy;
    const float ny2 = static_cast<float>(Cyy - Cxx) - hypot_cached;
    const float M2 = nx2 * nx2 + ny2 * ny2;

    float nx, ny;
    if (M1 > M2) {
      nx = nx1;
      ny = ny1;
    } else {
      nx = nx2;
      ny = ny2;
    }

    float length = std::hypotf(nx, ny);
    lineparam23[0] = nx / length;
    lineparam23[1] = ny / length;
  }

  // sum of squared errors
  *err = moments.N * eig_small;

  // mean squared error
  *mse = eig_small;
}

__device__ __forceinline__ void FitLine(
    const LineFitPoint *line_fit_points_device, size_t blob_point_count,
    size_t index0, size_t index1, double *lineparam01, double *lineparam23,
    double *err, double *mse) {
  LineFitMoments moments =
      ReadMoments(line_fit_points_device, blob_point_count, index0, index1);
  FitLine(moments, lineparam01, lineparam23, err, mse);
}

namespace {

// Unrank() for each combination of 4 peaks, so the quad fit's threads look
// their combinations up instead of computing them.  Every thread reads a
// different entry, which __constant__ memory would serialize.
__device__ const std::array<uint32_t, MaxRankedIndex()> kPackedUnrank =
    MakePackedUnrank();

// The peaks at a quad's corners, as indices into the best peaks.
struct QuadCorners {
  uint32_t m0;
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
};

__device__ __forceinline__ QuadCorners UnrankCorners(uint32_t combination) {
  const uint32_t packed = __ldg(&kPackedUnrank[combination]);
  return QuadCorners{packed & 0xff, (packed >> 8) & 0xff, (packed >> 16) & 0xff,
                     packed >> 24};
}

// A line fit to the points between two peaks.  FitLine computes all of these
// in float, so storing them as floats loses nothing.
struct SegmentFit {
  float err;
  float mse;
  // Only set for segments which don't wrap around the end of the blob.
  float lineparam23[2];
};

// A combination of 4 peaks and its quad fit error.
struct QuadCandidate {
  // DoubleOrderKey(error).
  uint64_t key;
  double error;
  uint32_t combination;
};

// Returns whether a has a smaller error than b, or the same error and an
// earlier combination.
__device__ __forceinline__ bool BetterQuad(const QuadCandidate &a,
                                           const QuadCandidate &b) {
  return a.key < b.key || (a.key == b.key && a.combination < b.combination);
}

// Returns the best of the candidates in the warp's first kLanes lanes to lane
// 0.
template <int kLanes>
__device__ __forceinline__ QuadCandidate WarpBestQuad(QuadCandidate candidate) {
  static_assert(kLanes <= 32 && (kLanes & (kLanes - 1)) == 0);
#pragma unroll
  for (int offset = kLanes / 2; offset > 0; offset /= 2) {
    QuadCandidate other;
    other.key = __shfl_down_sync(0xffffffffu, candidate.key, offset);
    other.error = __shfl_down_sync(0xffffffffu, candidate.error, offset);
    other.combination =
        __shfl_down_sync(0xffffffffu, candidate.combination, offset);
    if (BetterQuad(other, candidate)) {
      candidate = other;
    }
  }
  return candidate;
}

// Shared memory for QuadFitTail.
template <int kThreads>
struct QuadFitTailStorage {
  // The best kNMaxima peaks' point indices, by rank.
  uint16_t best_peaks[kNMaxima];
  // The same point indices, in point order.
  uint16_t point_indices[kNMaxima];

  // segments[a][b] fits the points from point_indices[a] to point_indices[b],
  // wrapping around the end of the blob when a > b.
  SegmentFit segments[kNMaxima][kNMaxima];

  // Each warp's best quad.
  QuadCandidate warp_best[kThreads / 32];
};

// Returns the error of the quad with corners at the peaks quad.m0-m3, or the
// max double if they don't make a quad, checking the segments in the same
// order aprilrobotics does.  The mse limit is a float too, so comparing it in
// float decides the same way as comparing in double.
__device__ double QuadFitError(const SegmentFit (&segments)[kNMaxima][kNMaxima],
                               const QuadCorners &quad, uint32_t num_maxima,
                               float max_line_fit_mse, double max_dot) {
  constexpr double kMaxError = std::numeric_limits<double>::max();
  if (quad.m3 >= num_maxima) {
    return kMaxError;
  }

  const SegmentFit &segment01 = segments[quad.m0][quad.m1];
  if (segment01.mse > max_line_fit_mse) {
    return kMaxError;
  }
  const SegmentFit &segment12 = segments[quad.m1][quad.m2];
  if (segment12.mse > max_line_fit_mse) {
    return kMaxError;
  }

  const double dot =
      static_cast<double>(segment01.lineparam23[0]) * segment12.lineparam23[0] +
      static_cast<double>(segment01.lineparam23[1]) * segment12.lineparam23[1];
  if (fabs(dot) > max_dot) {
    return kMaxError;
  }

  const SegmentFit &segment23 = segments[quad.m2][quad.m3];
  if (segment23.mse > max_line_fit_mse) {
    return kMaxError;
  }
  const SegmentFit &segment30 = segments[quad.m3][quad.m0];
  if (segment30.mse > max_line_fit_mse) {
    return kMaxError;
  }

  return static_cast<double>(segment01.err) + segment12.err + segment23.err +
         segment30.err;
}

// Fits a quad to blob blob_index, which has count points, from its best
// kNMaxima peaks, the way aprilrobotics' fit_quad does, and writes it to the
// blob's slots in fit_quads_device and fit_quad_valid_device.  peak_keys holds
// the PeakSortKey()s of the blob's first min(num_peaks, max_peaks) peaks.
// Every thread of a kThreads thread block calls this.
template <int kThreads>
__device__ __forceinline__ void QuadFitTail(
    QuadFitTailStorage<kThreads> &storage, uint32_t blob_index, uint32_t count,
    const LineFitPoint *line_fit_points, const uint64_t *peak_keys,
    uint32_t num_peaks, uint32_t max_peaks, float max_line_fit_mse,
    double max_dot, FitQuad *fit_quads_device, bool *fit_quad_valid_device) {
  static_assert(kThreads % 32 == 0);
  constexpr int kWarps = kThreads / 32;

  // It takes 4 peaks to make a quad.
  if (num_peaks < 4) {
    if (threadIdx.x == 0) {
      fit_quad_valid_device[blob_index] = false;
    }
    return;
  }
  num_peaks = min(num_peaks, max_peaks);
  const uint32_t num_maxima = min(num_peaks, static_cast<uint32_t>(kNMaxima));

  // Rank each peak against the others to pick the best num_maxima.  Keys are
  // unique, so the ranks are too.
  for (uint32_t i = threadIdx.x; i < num_peaks; i += kThreads) {
    const uint64_t key = peak_keys[i];
    uint32_t rank = 0;
    for (uint32_t j = 0; j < num_peaks && rank < kNMaxima; ++j) {
      rank += peak_keys[j] < key;
    }
    if (rank < kNMaxima) {
      storage.best_peaks[rank] = static_cast<uint16_t>(key);
    }
  }
  __syncthreads();

  // And put them back in point order.
  if (threadIdx.x < num_maxima) {
    const uint16_t index = storage.best_peaks[threadIdx.x];
    uint32_t position = 0;
    for (uint32_t j = 0; j < num_maxima; ++j) {
      position += storage.best_peaks[j] < index;
    }
    storage.point_indices[position] = index;
  }
  __syncthreads();

  // Fit each segment between two of the peaks once, for all the combinations
  // to share.  Only the forward segments' line parameters get used.
  for (uint32_t i = threadIdx.x; i < kNMaxima * kNMaxima; i += kThreads) {
    const uint32_t a = i / kNMaxima;
    const uint32_t b = i % kNMaxima;
    if (a != b && a < num_maxima && b < num_maxima) {
      double lineparam23[2];
      double err;
      double mse;
      FitLine(line_fit_points, count, storage.point_indices[a],
              storage.point_indices[b], nullptr, a < b ? lineparam23 : nullptr,
              &err, &mse);
      SegmentFit &segment = storage.segments[a][b];
      segment.err = err;
      segment.mse = mse;
      if (a < b) {
        segment.lineparam23[0] = lineparam23[0];
        segment.lineparam23[1] = lineparam23[1];
      }
    }
  }
  __syncthreads();

  // aprilrobotics keeps the first of equally good quads.  The errors are
  // finite (W >= 1), so the smallest (DoubleOrderKey(error), combination) is
  // that quad, and comparing the keys keeps the search off the FP64 pipe.  Each
  // thread's combinations ascend, so it keeps the first of its best.
  QuadCandidate best{.key = std::numeric_limits<uint64_t>::max(),
                     .error = std::numeric_limits<double>::max(),
                     .combination = static_cast<uint32_t>(MaxRankedIndex())};
  for (uint32_t combination = threadIdx.x; combination < MaxRankedIndex();
       combination += kThreads) {
    const double error =
        QuadFitError(storage.segments, UnrankCorners(combination), num_maxima,
                     max_line_fit_mse, max_dot);
    const uint64_t key = DoubleOrderKey(error);
    if (key < best.key) {
      best =
          QuadCandidate{.key = key, .error = error, .combination = combination};
    }
  }
  best = WarpBestQuad<32>(best);
  if (threadIdx.x % 32 == 0) {
    storage.warp_best[threadIdx.x / 32] = best;
  }
  __syncthreads();

  if (threadIdx.x >= 32) {
    return;
  }
  if (threadIdx.x < kWarps) {
    best = storage.warp_best[threadIdx.x];
  }
  best = WarpBestQuad<kWarps>(best);

  if (threadIdx.x == 0) {
    const bool valid = best.error < max_line_fit_mse * count;
    fit_quad_valid_device[blob_index] = valid;
    if (valid) {
      const QuadCorners corners = UnrankCorners(best.combination);
      FitQuad &fit_quad = fit_quads_device[blob_index];
      fit_quad.blob_index = blob_index;
      const uint32_t i0 = storage.point_indices[corners.m0];
      const uint32_t i1 = storage.point_indices[corners.m1];
      const uint32_t i2 = storage.point_indices[corners.m2];
      const uint32_t i3 = storage.point_indices[corners.m3];
      fit_quad.indices[0] = i0;
      fit_quad.indices[1] = i1;
      fit_quad.indices[2] = i2;
      fit_quad.indices[3] = i3;
      fit_quad.moments[0] = ReadMoments(line_fit_points, count, i0, i1);
      fit_quad.moments[1] = ReadMoments(line_fit_points, count, i1, i2);
      fit_quad.moments[2] = ReadMoments(line_fit_points, count, i2, i3);
      fit_quad.moments[3] = ReadMoments(line_fit_points, count, i3, i0);
    }
  }
}

// Returns the error of the line fit to the points within ksz of point i of a
// blob of count points, wrapping around the blob.  line_fit_points holds the
// prefix sums of the blob's points' line fit points.
__device__ __forceinline__ double PointError(
    const LineFitPoint *line_fit_points, uint32_t count, uint32_t ksz,
    uint32_t i) {
  // Blobs have at most 2 * (width + height) points, so 32 bit division is
  // exact, and much cheaper than 64 bit division on the GPU.
  const uint32_t i0 = (i + 2 * count - ksz) % count;
  const uint32_t i1 = (i + count + ksz) % count;
  const LineFitMoments moments = ReadMoments(line_fit_points, count, i0, i1);
  return FitLineError(moments.N, moments.Mx, moments.My, moments.Mxx,
                      moments.Myy, moments.Mxy, moments.W);
}

// Shared memory for FitBlobQuad.
template <int kThreads>
struct FitBlobQuadStorage {
  typename cub::BlockScan<int, kThreads>::TempStorage scan;
  QuadFitTailStorage<kThreads> tail;
};

// Fits blob blob_index's quad from its count sorted points' line fit points,
// the way aprilrobotics' fit_quad does: fits a line around each point, filters
// the fits' errors, and fits the quad from the best kNMaxima peaks of the
// filtered errors.  errs and filtered_errs have room for count errors, and
// peak_keys for max_peaks keys, which is more than the blob has peaks.  All of
// them may be in shared or global memory.  The blob's points start at offset
// in args' buffers.  Every thread of a kThreads thread block calls this.
template <int kThreads>
__device__ __forceinline__ void FitBlobQuad(
    FitBlobQuadStorage<kThreads> &storage, const BlobQuadFitArgs &args,
    uint32_t blob_index, uint32_t count, uint32_t offset,
    const LineFitPoint *line_fit_points, double *errs, double *filtered_errs,
    uint64_t *peak_keys, uint32_t max_peaks) {
  const uint32_t ksz = min(kMaxKsz, count / 12);
  for (uint32_t i = threadIdx.x; i < count; i += kThreads) {
    const double error = PointError(line_fit_points, count, ksz, i);
    errs[i] = error;
    if (args.keep_intermediates) {
      args.errs[offset + i] = error;
    }
  }
  __syncthreads();

  for (uint32_t i = threadIdx.x; i < count; i += kThreads) {
    double filtered_error = 0.0;
#pragma unroll
    for (size_t j = 0; j < FilterCoefficients().size(); ++j) {
      // Wraps around the blob, which has more points than the filter.
      uint32_t k = i + j + count - FilterCoefficients().size() / 2;
      k = k >= count ? k - count : k;
      k = k >= count ? k - count : k;
      filtered_error += errs[k] * FilterCoefficients()[j];
    }
    filtered_errs[i] = filtered_error;
    if (args.keep_intermediates) {
      args.filtered_errs[offset + i] = filtered_error;
    }
  }
  __syncthreads();

  // Collect the peaks' keys, in point order.
  uint32_t num_peaks = 0;
  for (uint32_t chunk = 0; chunk < count; chunk += kThreads) {
    const uint32_t i = chunk + threadIdx.x;
    int is_peak = 0;
    float error = 0;
    if (i < count) {
      // Every window has W >= 1, so the filtered errors are finite, and
      // comparing their DoubleOrderKey()s decides the same as comparing them,
      // without the FP64 pipe.
      const double filtered_error = filtered_errs[i];
      const uint64_t key = DoubleOrderKey(filtered_error);
      is_peak =
          key > DoubleOrderKey(filtered_errs[i == 0 ? count - 1 : i - 1]) &&
          key > DoubleOrderKey(filtered_errs[i + 1 == count ? 0 : i + 1]);
      error = -filtered_error;
      if (args.keep_intermediates) {
        Peak peak;
        peak.error = error;
        peak.filtered_point_index = offset + i;
        peak.blob_index = is_peak ? blob_index : Peak::kNoPeak();
        args.peaks[offset + i] = peak;
      }
    }
    int position;
    int chunk_peaks;
    cub::BlockScan<int, kThreads>(storage.scan)
        .ExclusiveSum(is_peak, position, chunk_peaks);
    if (is_peak && num_peaks + position < max_peaks) {
      peak_keys[num_peaks + position] = PeakSortKey(error, i);
    }
    num_peaks += chunk_peaks;
    // The next chunk reuses storage.scan, and QuadFitTail reads peak_keys.
    __syncthreads();
  }

  QuadFitTail<kThreads>(storage.tail, blob_index, count, line_fit_points,
                        peak_keys, num_peaks, max_peaks, args.max_line_fit_mse,
                        args.cos_critical_rad, args.fit_quads,
                        args.fit_quad_valid);
}

struct SmallBlobQuadStorage {
  union {
    SmallBlobStorage sort;
    // Two neighboring points are never both peaks.
    uint64_t peak_keys[kMaxSmallBlobPoints / 2 + 1];
  } temp;
  // IndexPoint's constructor keeps it out of a __shared__ variable, so this
  // holds the points uninitialized.
  cub::Uninitialized<IndexPoint[kMaxSharedBlobPoints]> sorted_points;
  LineFitPoint line_fit_points[kMaxSharedBlobPoints];
  double errs[kMaxSharedBlobPoints];
  double filtered_errs[kMaxSharedBlobPoints];
  FitBlobQuadStorage<kSmallBlobThreads> fit;
};

// Fits the quad of each small blob in the work list, one blob per block.
__global__ void __launch_bounds__(kSmallBlobThreads, 4)
    FitSmallBlobQuadsKernel(BlobQuadFitArgs args,
                            TransformLineFitPoint transform) {
  __shared__ SmallBlobQuadStorage storage;
  if (blockIdx.x >= args.work_list->num_small_blobs) {
    return;
  }
  const uint32_t blob_index = args.work_list->small_blobs[blockIdx.x];
  const MinMaxExtents extents = args.selected_extents[blob_index].value;
  const uint32_t count = extents.count;
  const uint32_t offset = extents.starting_offset;
  const IndexPoint *points = args.selected_blobs + offset;
  const bool in_shared = count <= kMaxSharedBlobPoints;
  IndexPoint *sorted_points =
      in_shared ? storage.sorted_points.Alias() : args.sorted_points + offset;
  LineFitPoint *line_fit_points =
      in_shared ? storage.line_fit_points : args.line_fit_points + offset;
  if (count <= kMaxRankSortPoints) {
    // Each thread ranks one point.
    static_assert(kSmallBlobThreads == kMaxRankSortPoints);
    RankSortAndScanBlob<kSmallBlobThreads>(storage.temp.sort.temp.rank_sort,
                                           blob_index, points, count, transform,
                                           sorted_points, line_fit_points);
  } else if (count <= 2 * kSmallBlobThreads) {
    RadixSortAndScanBlob<kSmallBlobThreads, 2, true>(
        storage.temp.sort.temp.radix_sort2, storage.temp.sort.temp.scan,
        blob_index, points, count, transform, sorted_points, line_fit_points);
  } else if (count <= 4 * kSmallBlobThreads) {
    RadixSortAndScanBlob<kSmallBlobThreads, 4, true>(
        storage.temp.sort.temp.radix_sort4, storage.temp.sort.temp.scan,
        blob_index, points, count, transform, sorted_points, line_fit_points);
  } else {
    static_assert(8 * kSmallBlobThreads == kMaxSmallBlobPoints);
    RadixSortAndScanBlob<kSmallBlobThreads, 8, true>(
        storage.temp.sort.temp.radix_sort8, storage.temp.sort.temp.scan,
        blob_index, points, count, transform, sorted_points, line_fit_points);
  }
  // FixAngleTies may still be writing the sorted points and line fit points
  // the rest reads.
  __syncthreads();

  if (args.keep_intermediates && in_shared) {
    for (uint32_t i = threadIdx.x; i < count; i += kSmallBlobThreads) {
      args.sorted_points[offset + i] = sorted_points[i];
      args.line_fit_points[offset + i] = line_fit_points[i];
    }
  }
  FitBlobQuad<kSmallBlobThreads>(
      storage.fit, args, blob_index, count, offset, line_fit_points,
      in_shared ? storage.errs : args.errs + offset,
      in_shared ? storage.filtered_errs : args.filtered_errs + offset,
      storage.temp.peak_keys, std::size(storage.temp.peak_keys));
}

struct TinyBlobQuadStorage {
  union {
    RankSortStorage<kTinyBlobThreads> sort;
    // Two neighboring points are never both peaks.
    uint64_t peak_keys[kMaxTinyBlobPoints / 2 + 1];
  } temp;
  cub::Uninitialized<IndexPoint[kMaxTinyBlobPoints]> sorted_points;
  LineFitPoint line_fit_points[kMaxTinyBlobPoints];
  double errs[kMaxTinyBlobPoints];
  double filtered_errs[kMaxTinyBlobPoints];
  FitBlobQuadStorage<kTinyBlobThreads> fit;
};

// Fits the quad of each tiny blob in the work list, one blob per block, with a
// thread for each point.  Everything but the outputs stays in shared memory.
__global__ void __launch_bounds__(kTinyBlobThreads, 8)
    FitTinyBlobQuadsKernel(BlobQuadFitArgs args,
                           TransformLineFitPoint transform) {
  __shared__ TinyBlobQuadStorage storage;
  if (blockIdx.x >= args.work_list->num_tiny_blobs) {
    return;
  }
  const uint32_t blob_index = args.work_list->tiny_blobs[blockIdx.x];
  const MinMaxExtents extents = args.selected_extents[blob_index].value;
  const uint32_t count = extents.count;
  const uint32_t offset = extents.starting_offset;
  IndexPoint *sorted_points = storage.sorted_points.Alias();
  RankSortAndScanBlob<kTinyBlobThreads>(
      storage.temp.sort, blob_index, args.selected_blobs + offset, count,
      transform, sorted_points, storage.line_fit_points);
  // WriteSortedChunk ends with a barrier, so every sorted point and line fit
  // point is written.
  if (args.keep_intermediates) {
    for (uint32_t i = threadIdx.x; i < count; i += kTinyBlobThreads) {
      args.sorted_points[offset + i] = sorted_points[i];
      args.line_fit_points[offset + i] = storage.line_fit_points[i];
    }
  }
  FitBlobQuad<kTinyBlobThreads>(storage.fit, args, blob_index, count, offset,
                                storage.line_fit_points, storage.errs,
                                storage.filtered_errs, storage.temp.peak_keys,
                                std::size(storage.temp.peak_keys));
}

struct BigBlobQuadStorage {
  union {
    BigBlobStorage sort;
    // Two neighboring points are never both peaks.
    uint64_t peak_keys[kBigBlobThreads * kBigBlobPointsPerThread / 2 + 1];
  } temp;
  FitBlobQuadStorage<kBigBlobThreads> fit;
};
// Kernels get at most 48 KiB of static shared memory.
static_assert(sizeof(TinyBlobQuadStorage) <= 48 * 1024);
static_assert(sizeof(SmallBlobQuadStorage) <= 48 * 1024);
static_assert(sizeof(BigBlobQuadStorage) <= 48 * 1024);
// An Orin SM has 164 KiB of shared memory, and each block also takes 1 KiB for
// the system.
static_assert(8 * (sizeof(TinyBlobQuadStorage) + 1024) <= 164 * 1024);
static_assert(4 * (sizeof(SmallBlobQuadStorage) + 1024) <= 164 * 1024);

// Fits the quad of each big blob in the work list, looping over the blobs.
// They keep everything but their peaks in args' buffers.
__global__ void __launch_bounds__(kBigBlobThreads)
    FitBigBlobQuadsKernel(BlobQuadFitArgs args,
                          TransformLineFitPoint transform) {
  __shared__ BigBlobQuadStorage storage;
  const uint32_t num_big_blobs = args.work_list->num_big_blobs;
  for (uint32_t i = blockIdx.x; i < num_big_blobs; i += gridDim.x) {
    const uint32_t blob_index = args.work_list->big_blobs[i];
    const MinMaxExtents extents = args.selected_extents[blob_index].value;
    const uint32_t offset = extents.starting_offset;
    RadixSortAndScanBlob<kBigBlobThreads, kBigBlobPointsPerThread, false>(
        storage.temp.sort.radix_sort, storage.temp.sort.scan, blob_index,
        args.selected_blobs + offset, extents.count, transform,
        args.sorted_points + offset, args.line_fit_points + offset);
    // FixAngleTies may still be writing the line fit points FitBlobQuad
    // reads.
    __syncthreads();
    FitBlobQuad<kBigBlobThreads>(
        storage.fit, args, blob_index, extents.count, offset,
        args.line_fit_points + offset, args.errs + offset,
        args.filtered_errs + offset, storage.temp.peak_keys,
        std::size(storage.temp.peak_keys));
    // The next blob reuses storage.
    __syncthreads();
  }
}

}  // namespace

void ListBlobs(const BlobQuadFitArgs &args, CudaStream *stream) {
  BuildBlobWorkList<<<1, kWorkListThreads, 0, stream->get()>>>(args);
}

void FitTinyBlobQuads(const BlobQuadFitArgs &args, CudaStream *stream) {
  ABSL_CHECK_EQ(args.nmaxima, kNMaxima)
      << ": Kernel is compiled and optimized for a fixed nmaxima, please "
         "recompile if you want to change it.";
  FitTinyBlobQuadsKernel<<<IndexPoint::kMaxBlobs, kTinyBlobThreads, 0,
                           stream->get()>>>(
      args, TransformLineFitPoint(args.decimated_image, args.decimated_width,
                                  args.decimated_height));
}

void FitSmallBlobQuads(const BlobQuadFitArgs &args, CudaStream *stream) {
  ABSL_CHECK_EQ(args.nmaxima, kNMaxima)
      << ": Kernel is compiled and optimized for a fixed nmaxima, please "
         "recompile if you want to change it.";
  FitSmallBlobQuadsKernel<<<IndexPoint::kMaxBlobs, kSmallBlobThreads, 0,
                            stream->get()>>>(
      args, TransformLineFitPoint(args.decimated_image, args.decimated_width,
                                  args.decimated_height));
}

void FitBigBlobQuads(const BlobQuadFitArgs &args, CudaStream *stream) {
  ABSL_CHECK_EQ(args.nmaxima, kNMaxima)
      << ": Kernel is compiled and optimized for a fixed nmaxima, please "
         "recompile if you want to change it.";
  // Each block holds a whole blob.
  static_assert(kBigBlobThreads * kBigBlobPointsPerThread == kMaxBlobPoints);
  ABSL_CHECK_LE(args.max_blob_points, kMaxBlobPoints);
  FitBigBlobQuadsKernel<<<kBigBlobBlocks, kBigBlobThreads, 0, stream->get()>>>(
      args, TransformLineFitPoint(args.decimated_image, args.decimated_width,
                                  args.decimated_height));
}

std::vector<int> BigBlobQuadFitPriorities(const CudaGraph &graph) {
  return graph.KernelPriorities(FitBigBlobQuadsKernel);
}

std::ostream &operator<<(std::ostream &os,
                         const frc::apriltag::LineFitMoments &moments) {
  os << "{Mx:" << std::setprecision(20) << moments.Mx / 2.
     << ", My:" << std::setprecision(20) << moments.My / 2.
     << ", Mxx:" << std::setprecision(20) << moments.Mxx / 4.
     << ", Mxy:" << std::setprecision(20) << moments.Mxy / 4.
     << ", Myy:" << std::setprecision(20) << moments.Myy / 4.
     << ", W:" << std::setprecision(20) << moments.W << ", N:" << moments.N
     << "}";
  return os;
}

}  // namespace frc::apriltag

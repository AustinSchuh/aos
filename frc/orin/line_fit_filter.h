#ifndef FRC_ORIN_LINE_FIT_FILTER_H_
#define FRC_ORIN_LINE_FIT_FILTER_H_

#include <array>
#include <bit>
#include <cub/iterator/transform_input_iterator.cuh>
#include <cuda/std/tuple>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "frc/orin/cuda.h"
#include "frc/orin/points.h"

namespace frc::apriltag {

// Class to hold the extents of a blob of points.
struct MinMaxExtents {
  // Min and max coordinates (in non-decimated coordinates)
  uint16_t min_x;
  uint16_t min_y;
  uint16_t max_x;
  uint16_t max_y;

  // The starting offset of this blob of points in the vector holding all the
  // points.
  uint32_t starting_offset;

  // The number of points in the blob.
  uint32_t count;

  // The dot product is:
  //  dot = sum( (px - cx) * gx) + (py - cy) * gy )
  //
  // We can split this up into:
  //  dot = sum(px * gx + py * gy) - cx * sum(gx) - cy * sum(gy)
  //
  // Which can be calculated without knowing the center.
  //
  // Since p and g are all integers, we can sum them into integers too so we
  // don't lose precision.
  int32_t gx_sum;
  int32_t gy_sum;

  int64_t pxgx_plus_pygy_sum;

  // Center location of the blob using the aprilrobotics algorithm.
  __host__ __device__ float cx() const {
    return (min_x + max_x) * 0.5f + 0.05118f;
  }
  __host__ __device__ float cy() const {
    return (min_y + max_y) * 0.5f + -0.028581f;
  }

  __host__ __device__ float dot() const {
    return static_cast<float>(pxgx_plus_pygy_sum * 2 -
                              (min_x + max_x) * gx_sum -
                              (min_y + max_y) * gy_sum) *
               0.5f -
           0.05118f * static_cast<float>(gx_sum) +
           0.028581f * static_cast<float>(gy_sum);
  }
};

struct LineFitPoint {
  // TODO(austin): How much precision do we actually need?  The less, the
  // faster...  The less memory too, the faster.
  //
  // Is it the double's or the memory which makes us slow?  Could probably bit
  // pack ints in here...
  int64_t Mxx;
  int64_t Myy;
  int64_t Mxy;
  // Mx and My are prefix sums of W * (x + 1) and W * (y + 1) over a blob.
  // SelectBlobs admits blobs of up to 2 * (width + height) points, and W is at
  // most hypot(255, 255) + 1 = 361, so at 1600x1304 the bound is
  // 5808 * 361 * 1600 ~= 3.4e9, which doesn't fit in an int32.  FitLine's
  // int64 Mxx * W similarly passes 2^63 for a segment of about 5200 or more
  // maximum weight points.  Both need thousands of near maximum gradient points
  // near the far edge of the image, which real frames don't have.  These limits
  // are this port's own: aprilrobotics keeps the moments in doubles, which
  // only lose precision.
  int32_t Mx;
  int32_t My;
  int32_t W;
  uint32_t blob_index;
};

// Aligning the struct to 16 bytes would pad it to 48 bytes and add 20% to
// every buffer and read.
static_assert(sizeof(LineFitPoint) == 40);
static_assert(alignof(LineFitPoint) == 8);

struct LineFitMoments {
  // See LineFitPoint for more info.
  int32_t Mx;
  int32_t My;
  int32_t W;
  int64_t Mxx;
  int64_t Myy;
  int64_t Mxy;
  int N;  // how many points are included in the set?
};

std::ostream &operator<<(std::ostream &os,
                         const frc::apriltag::LineFitMoments &moments);

struct Peak {
  static constexpr uint16_t kNoPeak() { return 0xffff; }
  float error;
  // Point index.
  uint32_t filtered_point_index;
  // 0xffff if this isn't a peak, otherwise the blob index.
  uint16_t blob_index;
};

constexpr std::array<float, 7> FilterCoefficients() {
  return std::array<float, 7>{
      0.01110899634659290314, 0.13533528149127960205, 0.60653066635131835938,
      1.00000000000000000000, 0.60653066635131835938, 0.13533528149127960205,
      0.01110899634659290314,
  };
}

struct FitQuad {
  uint16_t blob_index;
  bool valid;
  uint16_t indices[4];
  LineFitMoments moments[4];
};

// The selected blobs with points, split by which kernel fits their quads, each
// list in blob order.
struct BlobWorkList {
  uint32_t num_small_blobs;
  uint32_t num_big_blobs;
  uint16_t small_blobs[IndexPoint::kMaxBlobs];
  uint16_t big_blobs[IndexPoint::kMaxBlobs];
};

// The most points a blob can have, since FitBigBlobQuads() sorts each blob in
// one block's shared memory.
constexpr uint32_t kMaxBlobPoints = 6144;

// What the blob quad fits read, and where they write.
struct BlobQuadFitArgs {
  // ListBlobs() writes it, and the quad fits read it.
  BlobWorkList *work_list;
  // The selected blobs' points, each blob's in the range its entry of
  // selected_extents gives, and the extents, one per blob.
  const IndexPoint *selected_blobs;
  const cub::KeyValuePair<long, MinMaxExtents> *selected_extents;
  // The decimated image, for each point's weight.
  const uint8_t *decimated_image;
  int decimated_width;
  int decimated_height;

  // Each point's sorted point, line fit point, error, filtered error, and
  // peak, in selected_blobs' order.  Blobs with too many points to keep in
  // shared memory use the first four as scratch.  With keep_intermediates,
  // every blob writes all five, for the debug methods.
  IndexPoint *sorted_points;
  LineFitPoint *line_fit_points;
  double *errs;
  double *filtered_errs;
  Peak *peaks;
  bool keep_intermediates;

  // No blob has more points than this, which is at most kMaxBlobPoints.
  uint32_t max_blob_points;
  // The quad fit's parameters, as in apriltag_quad_thresh.  nmaxima must be
  // 10, which the kernels are compiled for.
  int nmaxima;
  float max_line_fit_mse;
  double cos_critical_rad;

  // Each blob's quad fit, in its own slot, and whether it is valid.
  FitQuad *fit_quads;
  bool *fit_quad_valid;
};

// Lists the selected blobs with points in work_list, and marks every other
// blob's quad fit invalid.  FitSmallBlobQuads() and FitBigBlobQuads() read the
// list, so this goes first.
void ListBlobs(const BlobQuadFitArgs &args, CudaStream *stream);

// Fits a quad to each blob on the work list, the way aprilrobotics' fit_quad
// does: sorts its points by angle around it, fits a line around each point,
// filters the fits' errors, and fits the quad from the best nmaxima peaks of
// the filtered errors.  FitSmallBlobQuads() covers the small blobs, with a
// block for each, and FitBigBlobQuads() the few big ones.  The two write
// different blobs, so they can run in parallel.
void FitSmallBlobQuads(const BlobQuadFitArgs &args, CudaStream *stream);
void FitBigBlobQuads(const BlobQuadFitArgs &args, CudaStream *stream);

// Returns the priority of each FitBigBlobQuads() launch in graph.
std::vector<int> BigBlobQuadFitPriorities(const CudaGraph &graph);

// Returns a key which orders peaks the way a stable cub radix sort by error
// does: by error, then by index.  cub compares floats by their bits, with -0.0
// equal to +0.0.
__host__ __device__ inline uint64_t PeakSortKey(float error, uint32_t index) {
  uint32_t bits = std::bit_cast<uint32_t>(error);
  if (bits == 0x80000000u) {
    bits = 0;
  }
  // Negative floats order backwards by their bits, and below positive ones.
  bits ^= (bits & 0x80000000u) ? 0xffffffffu : 0x80000000u;
  return (static_cast<uint64_t>(bits) << 32) | index;
}

// Returns a key which orders doubles the way < does, with -0.0 equal to +0.0,
// so comparing keys as integers picks the same minimum as comparing the
// doubles.  NaNs get keys too, but < doesn't order them.
__host__ __device__ inline uint64_t DoubleOrderKey(double d) {
  uint64_t bits = std::bit_cast<uint64_t>(d);
  if (bits == 0x8000000000000000ull) {
    bits = 0;
  }
  // Negative doubles order backwards by their bits, and below positive ones.
  bits ^= (bits & 0x8000000000000000ull) ? 0xffffffffffffffffull
                                         : 0x8000000000000000ull;
  return bits;
}

// Returns the m0, m1, m2, m3 indices for the provided index.  The index is the
// inner loop number when you process the 4 for loops in order and count.  See
// FilterTest.Unrank for an example.
//
// This lets us distribute work amoung the cuda threads and get back the index.
__host__ __device__ std::tuple<uint, uint, uint, uint> Unrank(uint i);
// The max number of work elements for a max maxes of 10.
constexpr size_t MaxRankedIndex() { return 210; }

// Returns Unrank(i) for every i, packed as m0 | m1 << 8 | m2 << 16 | m3 << 24,
// the table the quad fit's threads look their combination up in.
std::array<uint32_t, MaxRankedIndex()> PackedUnrank();

}  // namespace frc::apriltag

#endif  // FRC_ORIN_LINE_FIT_FILTER_H_

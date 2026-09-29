#ifndef FRC_ORIN_LINE_FIT_FILTER_H_
#define FRC_ORIN_LINE_FIT_FILTER_H_

#include <array>
#include <bit>
#include <cub/iterator/transform_input_iterator.cuh>
#include <cuda/std/tuple>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "frc/orin/cuda.h"

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

// DoFitLines copies LineFitPoints into shared memory with 16 byte loads, and
// its offsets rely on this layout.  Aligning the struct to 16 bytes would pad
// it to 48 bytes and add 20% to every buffer and read.
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

__host__ __device__ void FitLine(LineFitMoments moments, double *lineparam01,
                                 double *lineparam23, double *err, double *mse);

// Computes the line fit error, filtered error, and peak at every point.  The
// GPU reads the number of points from num_points_device, so it only has to be
// written by the time the kernel runs.
void FitLines(
    const LineFitPoint *line_fit_points_device, const int *num_points_device,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    double *errs_device, double *filtered_errs_device, Peak *peaks_device,
    CudaStream *stream);

// Fits a quad to each of the first num_blobs blobs in selected_extents_device
// from its best nmaxima peaks, the way aprilrobotics' fit_quad does.  A blob's
// quad goes in its own slot of fit_quads_device, and fit_quad_valid_device says
// which slots hold a valid quad.  No blob has more than max_blob_points points.
void FitQuads(
    const Peak *peaks_device, const LineFitPoint *line_fit_points_device,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    size_t num_blobs, size_t max_blob_points, int nmaxima,
    float max_line_fit_mse, double cos_critical_rad, FitQuad *fit_quads_device,
    bool *fit_quad_valid_device, CudaStream *stream);

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

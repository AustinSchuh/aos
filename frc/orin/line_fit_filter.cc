#include "frc/orin/line_fit_filter.h"

#include <cub/block/block_reduce.cuh>
#include <cub/block/block_scan.cuh>
#include <iomanip>

#include "absl/log/absl_check.h"

#include "frc/orin/cuda.h"

// #define DEBUG_BLOB_NUMBER 401

namespace frc::apriltag {

static_assert(sizeof(LineFitPoint) == 40, "Size of LineFitPoint changed");
static_assert(sizeof(int4) == 16, "Size of int4 changed");

constexpr size_t kPointsPerBlock = 256;
// Number of errors to calculate before and after for the err filter and peak
// finder.
constexpr size_t kBeforeBuffer = 4;
constexpr size_t kAfterBuffer = 4;
constexpr size_t kErrorsBuffer = kBeforeBuffer + kAfterBuffer;

__device__ double FitLineError(size_t N, int64_t Mx, int64_t My, int64_t Mxx,
                               int64_t Myy, int64_t Mxy, int64_t W) {
  int64_t Cxx = Mxx * W - Mx * Mx;
  int64_t Cxy = Mxy * W - Mx * My;
  int64_t Cyy = Myy * W - My * My;

  // Pose it as an eigenvalue problem.
  //
  // TODO(austin): Are floats good enough?  Hard to tell what the "right answer"
  // actually is...
  float eig_small = ((Cxx + Cyy) - std::hypotf((Cxx - Cyy), 2 * Cxy)) /
                    static_cast<float>(W * W * 8.0);
  return N * eig_small;
}

struct TempStorage {
  // Block of points that we pre-load.
  LineFitPoint tmp_storage[kPointsPerBlock];

  // The errors loaded.  The first kAfterBuffer errors are the indices (could be
  // duplicated) immediately before the first blob.  The last kAfterBuffer
  // errors are the indices immediately following the last blob.
  double tmp_errs[kPointsPerBlock - 2 * kAfterBuffer];

  // The errors which are from the start of the first blob before the beginning
  // of the buffer.
  double tmp_errs_before[kAfterBuffer];
  // The errors which are from the end of the last blob after the end of the
  // buffer.
  double tmp_errs_after[kAfterBuffer];

  // The filtered errors in the primary block.
  double tmp_filtered_errs[kPointsPerBlock - 4 * kAfterBuffer + 2];

  // The filtered error corresponding to the first error in tmp_errs_before.
  double tmp_filtered_err_before;
  // The filtered error corresponding to the last error in tmp_errs_after.
  double tmp_filtered_err_after;

  static_assert(sizeof(tmp_storage) == sizeof(LineFitPoint) * kPointsPerBlock,
                "Storage changed size.");
};

template <size_t kThreads>
class ErrorCalculator {
 public:
  __host__ __device__ ErrorCalculator(
      const LineFitPoint *line_fit_points_device, size_t points,
      const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
      TempStorage *storage)
      : line_fit_points_device_(line_fit_points_device),
        points_(points),
        selected_extents_device_(selected_extents_device),
        storage_(storage) {}

  __host__ __device__ void Load() {
    constexpr size_t kLoadSize = sizeof(int4);
    static_assert(kLoadSize == 16, "kload");

    // Global index in 128 bit loads.
    const uint32_t global_index =
        global_block_index_cache_start_ * sizeof(LineFitPoint) / kLoadSize +
        threadIdx.x;

    for (uint32_t i = 0; i < kPointsPerBlock * sizeof(LineFitPoint) / kLoadSize;
         i += kThreads) {
      // Load things in 128 bit increments strided across our threads in such a
      // way our loads will coalesce nicely.
      //
      // Maybe overload a little bit past the end, but we over-allocate
      // things on the host too.  Otherwise the end of the message can be
      // corrupted.  For arrays which are multiples of kPointsPerBlock long,
      // this will have no issues.  LineFitPoint is 40 bytes, not a multiple of
      // the load size, so round up to load the last point's tail but nothing
      // entirely past it.
      if (i + global_index <
          (points_ * sizeof(LineFitPoint) + kLoadSize - 1) / kLoadSize) {
        *(reinterpret_cast<int4 *>(storage_->tmp_storage) + i + threadIdx.x) =
            *(reinterpret_cast<const int4 *>(line_fit_points_device_) +
              global_index + i);
      }
    }
  }

  // Loads the blob extents for the provided buffer index. This index is from
  // the front of the range we are responsible for computing.
  __host__ __device__ void LoadExtents(size_t index) {
    const size_t current_blob_index =
        storage_->tmp_storage[index + (blockIdx.x == 0 ? 0 : kErrorsBuffer)]
            .blob_index;
    if (current_blob_index != blob_index_) {
      starting_offset_ =
          selected_extents_device_[current_blob_index].value.starting_offset;
      count_ = selected_extents_device_[current_blob_index].value.count;
      blob_index_ = current_blob_index;
      ksz_ = std::min<int>(20, count_ / 12);
    }
  }

  // Loads the error from our shared memory.  This follows the same model as
  // BlobIndex and CalculateError below.  Call LoadExtents first to get the
  // right index, and proceed from there.
  __host__ __device__ double LoadError(ssize_t index) const {
    // Index into the current blob.  Should be between 0 and count_.
    const uint32_t positive_blob_index = (index + count_) % count_;

    // This one is a normal one, it's global index is inside our range or
    // contiguously just before our range.  Just do it.
    //
    // (note: kBeforeBuffer should be - on the right side of the >=, but due to
    // unsigned arithmatic, it needs to be on the left.)
    if (positive_blob_index + starting_offset_ + kBeforeBuffer >=
        global_block_index_start()) {
      // It is either inside our block, or just after our block.  That is all
      // contiguous in memory and easy to calculate, so just return it.
      if (positive_blob_index + starting_offset_ <
          global_block_index_end() + kAfterBuffer) {
        return storage_->tmp_errs[positive_blob_index + starting_offset_ +
                                  kBeforeBuffer - global_block_index_start()];
      }

      // OK, this is past the back, but at the back end of the blob at the back
      // end. These bytes are packed onto the back 4 objects of the buffer.
      const size_t buffer_end = starting_offset_ + count_;
      const size_t distance_from_end =
          buffer_end - (positive_blob_index + starting_offset_);
      return storage_->tmp_errs_after[4 - distance_from_end];
    } else {
      // We must be inside the before bits.
      return storage_->tmp_errs_before[positive_blob_index];
    }
  }

  // Loads the filtered error from the shared memory.  This follows the same
  // model as BlobIndex and CalculateError below.  Call LoadExtents first to get
  // the right index, and proceed from there.
  __host__ __device__ double LoadFilteredError(ssize_t index) const {
    const uint32_t positive_blob_index =
        (index + global_block_index_start_ + count_ - starting_offset_) %
        count_;
    // This one is a normal one, it's global index is inside our range or
    // contiguously just before our range.  Just do it.
    if (positive_blob_index + starting_offset_ + 1 >=
        global_block_index_start()) {
      // It is either inside our block, or just after our block.  That is all
      // contiguous in memory and easy to calculate, so just return it.
      if (positive_blob_index + starting_offset_ <
          global_block_index_end() + 1) {
        return storage_
            ->tmp_filtered_errs[positive_blob_index + starting_offset_ + 1 -
                                global_block_index_start()];
      }

      // OK, this is past the back, but at the back end of the blob at the back
      // end.  Return the on error we've got there.
      return storage_->tmp_filtered_err_after;
    } else {
      // We must be inside the before bits.
      return storage_->tmp_filtered_err_before;
    }
  }

  // Gets the line fit point from either our local cache, or from main memory if
  // it is too far away.
  __host__ __device__ LineFitPoint GetPoint(size_t global_index,
                                            bool print = false) const {
    if (global_index >= global_block_index_cache_end_ ||
        global_index < global_block_index_cache_start_) {
      if (print) {
#ifdef DEBUG_BLOB_NUMBER
        printf(
            "Block %d Thread %d   Loading global %d, relative %d from global\n",
            blockIdx.x, threadIdx.x, (int)global_index,
            (int)(global_index - starting_offset_));
#endif
      }
      return line_fit_points_device_[global_index];
    } else {
      if (print) {
#ifdef DEBUG_BLOB_NUMBER
        printf(
            "Block %d Thread %d   Loading global %d, relative %d from cache\n",
            blockIdx.x, threadIdx.x, (int)global_index,
            (int)(global_index - starting_offset_));
#endif
      }
      return storage_
          ->tmp_storage[global_index - global_block_index_cache_start_];
    }
  }

  // Returns the blob index for the provided buffer index.  This is relative to
  // the start of the range we are responsible for.
  __forceinline__ __host__ __device__ size_t
  BlobIndex(size_t buffer_index) const {
    return global_block_index_start_ + buffer_index - starting_offset_;
  }

  // Calculates the line fit error centered on the provided blob index in the
  // current extent.
  __device__ double CalculateError(ssize_t blob_index,
                                   bool print = false) const {
    // Index into the blob list for the current key.
    const size_t i0 = (blob_index + 2 * count_ - ksz_) % count_;
    const size_t i1 = (blob_index + count_ + ksz_) % count_;

    const size_t global_i0 = i0 + starting_offset_;
    const size_t global_i1 = i1 + starting_offset_;

    int32_t Mx, My, W;
    int64_t Mxx, Myy, Mxy;
    int N;  // how many points are included in the set?

    if (i0 < i1) {
      N = i1 - i0 + 1;

      LineFitPoint lf1 = GetPoint(global_i1);

      Mx = lf1.Mx;
      My = lf1.My;
      Mxx = lf1.Mxx;
      Mxy = lf1.Mxy;
      Myy = lf1.Myy;
      W = lf1.W;

      if (i0 > 0) {
        LineFitPoint lf0 = GetPoint(global_i0 - 1);

        Mx -= lf0.Mx;
        My -= lf0.My;
        Mxx -= lf0.Mxx;
        Mxy -= lf0.Mxy;
        Myy -= lf0.Myy;
        W -= lf0.W;
      }
    } else {
      // i0 > i1, e.g. [15, 2]. Wrap around.
      LineFitPoint lf0 = GetPoint(global_i0 - 1, print);
      LineFitPoint lfsz = GetPoint(starting_offset_ + count_ - 1, print);

      Mx = lfsz.Mx - lf0.Mx;
      My = lfsz.My - lf0.My;
      Mxx = lfsz.Mxx - lf0.Mxx;
      Mxy = lfsz.Mxy - lf0.Mxy;
      Myy = lfsz.Myy - lf0.Myy;
      W = lfsz.W - lf0.W;

      LineFitPoint lf1 = GetPoint(global_i1, print);

      Mx += lf1.Mx;
      My += lf1.My;
      Mxx += lf1.Mxx;
      Mxy += lf1.Mxy;
      Myy += lf1.Myy;
      W += lf1.W;

      N = count_ - i0 + i1 + 1;
    }

    // And now fit it.
    return FitLineError(N, Mx, My, Mxx, Myy, Mxy, W);
  }

  // Returns the starting global index of the region we are responsible for
  // computing.
  __host__ __device__ __forceinline__ uint32_t
  global_block_index_start() const {
    return global_block_index_start_;
  }
  // Returns the ending global index of the region we are responsible for
  // computing.
  __host__ __device__ __forceinline__ uint32_t global_block_index_end() const {
    return global_block_index_end_;
  }
  // Returns the size of the region we are responsible for computing.
  __host__ __device__ __forceinline__ uint32_t global_block_index_size() const {
    return global_block_index_end_ - global_block_index_start_;
  }

  // Returns the starting index in global indices of the current blob ID.
  __host__ __device__ __forceinline__ uint32_t
  global_current_blob_starting_index() const {
    return starting_offset_;
  }

  // Returns the number of points in the current blob ID.
  __host__ __device__ __forceinline__ uint32_t
  global_current_blob_count() const {
    return count_;
  }

  // Returns the current blob ID.
  __host__ __device__ __forceinline__ uint32_t blob_index() const {
    return blob_index_;
  }

 private:
  const LineFitPoint *line_fit_points_device_;
  size_t points_;
  const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device_;

  TempStorage *storage_;

  // Start and end location of the LineFitPoint cache in tmp_storage_.
  const uint32_t global_block_index_cache_start_ =
      blockIdx.x == 0
          ? 0
          : blockIdx.x * (kPointsPerBlock - 2 * kErrorsBuffer) - kErrorsBuffer;
  const uint32_t global_block_index_cache_end_ = std::min<uint32_t>(
      global_block_index_cache_start_ + kPointsPerBlock, points_);

  const uint32_t global_block_index_start_ =
      blockIdx.x * (kPointsPerBlock - 2 * kErrorsBuffer);
  const uint32_t global_block_index_end_ = std::min<uint32_t>(
      global_block_index_start_ + kPointsPerBlock - 2 * kErrorsBuffer, points_);

  size_t blob_index_ = 0xffffffff;
  size_t starting_offset_ = 0;
  size_t count_ = 0;
  size_t ksz_ = 0;
};

template <size_t kThreads>
__global__ void DoFitLines(
    const LineFitPoint *line_fit_points_device, size_t points,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    double *errs_device, double *filtered_errs_device, Peak *peaks_device) {
  __shared__ TempStorage storage;

  ErrorCalculator<kThreads> calculator(line_fit_points_device, points,
                                       selected_extents_device, &storage);

  calculator.Load();

  // TODO(austin): Whichever warp loads the first ksz blobs should load the
  // prior ksz blobs.  Figure out if things are too slow first.  __syncwarp()
  // and make the first warp do it?

  __syncthreads();

  // We need to compute a couple of errs_device past each end in shared memory.
  // This needs to be (FilterCoefficients().size() - 1) + 2.  The first term is
  // the number of extra errors we need for the filter.  The second is the extra
  // number of errors needed to implement the peak finder (1 on each side).

  // OK, the way this all works now, error calculation work [0, kErrorsBuffer)
  // is done in the first kErrorsBuffer threads, and written to
  // tmp_errs [0, kErrorsBuffer), even if this wraps back into the buffer.

  // Now that everything is loaded, have each thread process its points by
  // strides.

  // TODO(austin): We need 4 reserved ranges.  Proposal is:
  //   [0, kErrorsBuffer) -> beginning points of the extent of blob 0, ie
  //     starting at starting_offset_ globally.
  //
  //   [kErrorsBuffer, 2 * kErrorsBuffer) -> 4 points immediately before the
  //   starting point.
  //
  // And then symetrically off the back end too.  This will mean we process 16
  // less points per thread.  Whatever.

  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    // OK, now we need to do the actual line fit.
    if (i < kErrorsBuffer) {
      // Start by extending the errors before the blob in our buffer space.  All
      // of this will be done in blob0's address space.
      //
      // In the first block, this will load the 8th element.  But, since blobs
      // are 24 or longer, this will be the 0th blob.  For following blocks,
      // this is actually the right index.
      calculator.LoadExtents(0);

      // First 4 threads do the first 4 elements in the first blob.
      if (i < kBeforeBuffer) {
        storage.tmp_errs_before[i] = calculator.CalculateError(i);
      } else {
        // Then, compute the 4 errors right before the buffer.
        storage.tmp_errs[i - kBeforeBuffer] = calculator.CalculateError(
            calculator.BlobIndex(i - kBeforeBuffer) - kBeforeBuffer);
      }
    } else if (i - kErrorsBuffer + calculator.global_block_index_start() <
               calculator.global_block_index_end() + kBeforeBuffer) {
      // Now we are solidly in the middle of the block.  Make sure anything
      // computed after the end of the buffer uses the extents of the end.
      calculator.LoadExtents(std::min<uint32_t>(
          calculator.global_block_index_size() - 1, i - kErrorsBuffer));
      size_t target_index = calculator.BlobIndex(i - kErrorsBuffer);

#ifdef DEBUG_BLOB_NUMBER
      const bool print =
          (calculator.blob_index() == DEBUG_BLOB_NUMBER &&
           ((484 - 4 <= target_index && target_index <= 484 + 4)));
#endif

      storage.tmp_errs[i - kBeforeBuffer] =
          calculator.CalculateError(target_index);
#ifdef DEBUG_BLOB_NUMBER
      if (print) {
        printf("Block %d Thread %d (idx %d)   Err: %f\n", blockIdx.x,
               threadIdx.x, (int)target_index,
               calculator.CalculateError(target_index, true));
      }
#endif
    } else {
      // Past the end of the normal calcs, do the end buffering.
      if (i < kPointsPerBlock - kBeforeBuffer) {
        continue;
      }

      // We are just supposed to continue with the extents of the last index
      // and keep going.
      calculator.LoadExtents(calculator.global_block_index_size() - 1);

      // Wrap before the beginning now of the last blob.
      storage.tmp_errs_after[i - (kPointsPerBlock - kBeforeBuffer)] =
          calculator.CalculateError(calculator.global_current_blob_count() -
                                    kBeforeBuffer +
                                    (i - (kPointsPerBlock - kBeforeBuffer)));
    }
  }

  __syncthreads();

#ifdef DEBUG_BLOB_NUMBER
  /*
  if (threadIdx.x == 0) {
    for (int i = (int)calculator.global_block_index_cache_start_;
         i < (int)calculator.global_block_index_cache_end_; ++ i) {
      auto x = calculator.GetPoint(i);
      if (x.blob_index == DEBUG_BLOB_NUMBER) {
        printf("Block %d Thread %d   Loading global %d, relative %d, Mx: %f\n",
               blockIdx.x, threadIdx.x, i,
               (int)(i - calculator.selected_extents_device_[x.blob_index]
                             .value.starting_offset),
               x.Mx / 2.0);
      }
    }
  }

  __syncthreads();
  */
#endif

  // We now have all the errors loaded! Box filter them.
  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    // The peak finder needs 1 more filtered error in each direction.
    ssize_t target_index;
    double *destination;
    if (i < kErrorsBuffer) {
      if (i < kErrorsBuffer - 2) {
        continue;
      } else {
        calculator.LoadExtents(0);
        if (i < kErrorsBuffer - 1) {
          // Target index is now 0.
          target_index = 0;
          destination = &storage.tmp_filtered_err_before;
        } else {
          target_index = calculator.BlobIndex(0) - 1;
          destination = &storage.tmp_filtered_errs[i + 1 - kErrorsBuffer];
        }
      }
    } else if (i - kErrorsBuffer + calculator.global_block_index_start() >=
               calculator.global_block_index_end()) {
      if (i - kErrorsBuffer + calculator.global_block_index_start() >=
          calculator.global_block_index_end() + 2) {
        // Past 1 past the end for the peak finder.
        break;
      }
      calculator.LoadExtents(calculator.global_block_index_end() -
                             calculator.global_block_index_start() - 1);

      if (i - kErrorsBuffer + calculator.global_block_index_start() >=
          calculator.global_block_index_end() + 1) {
        destination = &storage.tmp_filtered_err_after;
        target_index = calculator.global_current_blob_count() - 1;
      } else {
        destination = &storage.tmp_filtered_errs[i + 1 - kErrorsBuffer];
        target_index = calculator.BlobIndex(i - kErrorsBuffer);
      }
    } else {
      calculator.LoadExtents(i - kErrorsBuffer);
      target_index = calculator.BlobIndex(i - kErrorsBuffer);
      destination = &storage.tmp_filtered_errs[i + 1 - kErrorsBuffer];
    }

    double accumulated_error = 0.0;

#ifdef DEBUG_BLOB_NUMBER
    const bool print = (calculator.blob_index() == DEBUG_BLOB_NUMBER &&
                        ((484 - 4 <= target_index && target_index <= 484 + 4)));
#endif

#pragma unroll
    for (size_t j = 0; j < FilterCoefficients().size(); ++j) {
      const double e = calculator.LoadError(target_index + j -
                                            FilterCoefficients().size() / 2);
      accumulated_error += e * FilterCoefficients()[j];
#ifdef DEBUG_BLOB_NUMBER
      if (print) {
        printf("Block %d Thread %d (idx %d)    + %f * %f (%f) -> %f\n",
               blockIdx.x, threadIdx.x, (int)target_index, e,
               FilterCoefficients()[j], e * FilterCoefficients()[j],
               accumulated_error);
      }
#endif
    }
    *destination = accumulated_error;

#ifdef DEBUG_BLOB_NUMBER
    if (print) {
      printf(
          "Block %d Thread %d (idx %d)  Blob %d of size %d, index %d "
          "filtered_error = "
          "%f, error = %f\n",
          blockIdx.x, threadIdx.x, (int)target_index,
          (int)calculator.blob_index(),
          (int)calculator.global_current_blob_count(), (int)target_index,
          accumulated_error, calculator.LoadError(target_index));
    }
#endif
  }

  __syncthreads();
  // OK, errors are now saved to shared memory.  Compute peaks and push out to
  // disk.

  // TODO(austin): Don't save out the unfiltered errors if we aren't
  // testing/debugging...
  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    if (i < kErrorsBuffer) {
      continue;
    }

    if (i - kErrorsBuffer + calculator.global_block_index_start() >=
        calculator.global_block_index_end()) {
      break;
    }

    errs_device[i - kErrorsBuffer + calculator.global_block_index_start()] =
        storage.tmp_errs[i - kBeforeBuffer];
  }

  for (size_t i = threadIdx.x; i < kPointsPerBlock; i += blockDim.x) {
    if (i < kErrorsBuffer) {
      continue;
    }

    if (i - kErrorsBuffer + calculator.global_block_index_start() >=
        calculator.global_block_index_end()) {
      break;
    }

    calculator.LoadExtents(i - kErrorsBuffer);

    const double before_error =
        calculator.LoadFilteredError(i - kErrorsBuffer - 1);
    const double my_error = calculator.LoadFilteredError(i - kErrorsBuffer);
    const double after_error =
        calculator.LoadFilteredError(i - kErrorsBuffer + 1);

    // This gets us ready to sort.
    filtered_errs_device[i - kErrorsBuffer +
                         calculator.global_block_index_start()] = my_error;

    const bool is_peak = my_error > before_error && my_error > after_error;

    Peak peak;
    peak.error = -my_error;
    peak.blob_index = is_peak ? calculator.blob_index() : Peak::kNoPeak();
    peak.filtered_point_index =
        i - kErrorsBuffer + calculator.global_block_index_start();

    peaks_device[peak.filtered_point_index] = peak;
  }
}

void FitLines(
    const LineFitPoint *line_fit_points_device, size_t points,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    double *errs_device, double *filtered_errs_device, Peak *peaks_device,
    CudaStream *stream) {
  constexpr size_t kThreads = 128;
  const size_t kBlocks = (points + kPointsPerBlock - 2 * kErrorsBuffer - 1) /
                         (kPointsPerBlock - 2 * kErrorsBuffer);

  ABSL_VLOG(1) << "Spawning with " << kThreads << " threads, and " << kBlocks
               << " blocks for " << points << " points";
  // A launch with no blocks fails and leaves the error pending.
  if (kBlocks == 0) {
    return;
  }
  DoFitLines<kThreads><<<kBlocks, kThreads, 0, stream->get()>>>(
      line_fit_points_device, points, selected_extents_device, errs_device,
      filtered_errs_device, peaks_device);
}

namespace {

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
                        double *lineparam23, double *err, double *mse,
                        bool print = false) {
  if (print) {
    printf(
        "Block %d Thread %d   Mx: %.4f, My: %.4f, Mxx: %.4f, Mxy: %.4f, Myy: "
        "%.4f, W: %d, N: %d\n",
        blockIdx.x, threadIdx.x, moments.Mx / 2.0, moments.My / 2.0,
        moments.Mxx / 4.0, moments.Mxy / 4.0, moments.Myy / 4.0, moments.W,
        moments.N);
  }
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
  const float eight_w_squared = static_cast<float>(
      static_cast<int64_t>(moments.W) * static_cast<int64_t>(moments.W) * 8.0);
  const float eig_small = (Cxx + Cyy - hypot_cached) / eight_w_squared;
  if (print) {
    printf("Block %d Thread %d   eig_small: (%ld + %ld - %f) / %f -> %f\n",
           blockIdx.x, threadIdx.x, Cxx, Cyy, hypot_cached, eight_w_squared,
           eig_small);
  }

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

  if (print) {
    printf("Block %d Thread %d   err: %f\n", blockIdx.x, threadIdx.x,
           moments.N * eig_small);
  }

  // mean squared error
  *mse = eig_small;
}

__device__ __forceinline__ void FitLine(
    const LineFitPoint *line_fit_points_device, size_t blob_point_count,
    size_t index0, size_t index1, double *lineparam01, double *lineparam23,
    double *err, double *mse, bool print = false) {
  LineFitMoments moments =
      ReadMoments(line_fit_points_device, blob_point_count, index0, index1);
  FitLine(moments, lineparam01, lineparam23, err, mse, print);
}

struct QuadError {
  uint8_t m0;
  uint8_t m1;
  uint8_t m2;
  uint8_t m3;
  double error;
};

struct MinQuadError {
  __host__ __device__ QuadError operator()(const QuadError &a,
                                           const QuadError &b) {
    if (a.error <= b.error) {
      return a;
    } else {
      return b;
    }
  }
};

namespace {

// Threads per DoFitQuads block.  Each combination of 4 peaks gets its own
// thread.
constexpr uint32_t kFitQuadsThreads = 256;
static_assert(kFitQuadsThreads >= MaxRankedIndex());

// Unrank() for each DoFitQuads thread, so the threads look their combination up
// instead of computing it.
__constant__ std::array<uint32_t, MaxRankedIndex()> kPackedUnrank =
    MakePackedUnrank();

// A line fit to the points between two peaks.
struct SegmentFit {
  double err;
  double mse;
  // Only set for segments which don't wrap around the end of the blob.
  double lineparam23[2];
};

struct FitQuadsStorage {
  typedef cub::BlockScan<int, kFitQuadsThreads> BlockScan;
  typedef cub::BlockReduce<QuadError, kFitQuadsThreads> BlockReduce;

  union {
    typename BlockScan::TempStorage scan;
    typename BlockReduce::TempStorage reduce;
  } cub;

  // The best kNMaxima peaks' point indices, by rank.
  uint16_t best_peaks[kNMaxima];
  // The same point indices, in point order.
  uint16_t point_indices[kNMaxima];

  // segments[a][b] fits the points from point_indices[a] to point_indices[b],
  // wrapping around the end of the blob when a > b.
  SegmentFit segments[kNMaxima][kNMaxima];
};

// Returns the error of the quad with corners at the peaks quad.m0-m3, or the
// max double if they don't make a quad, checking the segments in the same
// order aprilrobotics does.
__device__ double QuadFitError(const FitQuadsStorage &storage,
                               const QuadError &quad, uint32_t num_maxima,
                               float max_line_fit_mse, double max_dot) {
  constexpr double kMaxError = std::numeric_limits<double>::max();
  if (quad.m3 >= num_maxima) {
    return kMaxError;
  }

  const SegmentFit &segment01 = storage.segments[quad.m0][quad.m1];
  if (segment01.mse > max_line_fit_mse) {
    return kMaxError;
  }
  const SegmentFit &segment12 = storage.segments[quad.m1][quad.m2];
  if (segment12.mse > max_line_fit_mse) {
    return kMaxError;
  }

  const double dot = segment01.lineparam23[0] * segment12.lineparam23[0] +
                     segment01.lineparam23[1] * segment12.lineparam23[1];
  if (fabs(dot) > max_dot) {
    return kMaxError;
  }

  const SegmentFit &segment23 = storage.segments[quad.m2][quad.m3];
  if (segment23.mse > max_line_fit_mse) {
    return kMaxError;
  }
  const SegmentFit &segment30 = storage.segments[quad.m3][quad.m0];
  if (segment30.mse > max_line_fit_mse) {
    return kMaxError;
  }

  return segment01.err + segment12.err + segment23.err + segment30.err;
}

// Fits a quad to blob blockIdx.x from its best kNMaxima peaks.  peak_keys has
// room for max_peaks keys.
__global__ void __launch_bounds__(kFitQuadsThreads) DoFitQuads(
    const Peak *peaks_device, const LineFitPoint *line_fit_points_device,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    uint32_t max_peaks, float max_line_fit_mse, double max_dot,
    FitQuad *fit_quads_device, bool *fit_quad_valid_device) {
  extern __shared__ uint64_t peak_keys[];
  __shared__ FitQuadsStorage storage;

  const uint32_t blob_index = blockIdx.x;
  const MinMaxExtents extents = selected_extents_device[blob_index].value;
  const uint32_t count = extents.count;
  // Filtered out blobs, and blobs past the end, have no points.
  if (count == 0) {
    if (threadIdx.x == 0) {
      fit_quad_valid_device[blob_index] = false;
    }
    return;
  }
  const Peak *peaks = peaks_device + extents.starting_offset;
  const LineFitPoint *line_fit_points =
      line_fit_points_device + extents.starting_offset;

  // Collect the peaks' keys, in point order.
  uint32_t num_peaks = 0;
  for (uint32_t chunk = 0; chunk < count; chunk += kFitQuadsThreads) {
    const uint32_t index = chunk + threadIdx.x;
    int is_peak = 0;
    float error = 0;
    if (index < count) {
      is_peak = peaks[index].blob_index != Peak::kNoPeak();
      error = peaks[index].error;
    }
    int offset;
    int chunk_peaks;
    FitQuadsStorage::BlockScan(storage.cub.scan)
        .ExclusiveSum(is_peak, offset, chunk_peaks);
    // Two neighboring points can't both be peaks, so this always holds.
    if (is_peak && num_peaks + offset < max_peaks) {
      peak_keys[num_peaks + offset] = PeakSortKey(error, index);
    }
    num_peaks += chunk_peaks;
    __syncthreads();
  }

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
  for (uint32_t i = threadIdx.x; i < num_peaks; i += kFitQuadsThreads) {
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
  if (threadIdx.x < kNMaxima * kNMaxima) {
    const uint32_t a = threadIdx.x / kNMaxima;
    const uint32_t b = threadIdx.x % kNMaxima;
    if (a != b && a < num_maxima && b < num_maxima) {
      SegmentFit &segment = storage.segments[a][b];
      FitLine(line_fit_points, count, storage.point_indices[a],
              storage.point_indices[b], nullptr,
              a < b ? segment.lineparam23 : nullptr, &segment.err,
              &segment.mse);
    }
  }
  __syncthreads();

  // Threads past the combinations contribute the max error, and come after
  // them, so the reduction still picks the first of equally good quads.
  QuadError quad;
  quad.m0 = quad.m1 = quad.m2 = quad.m3 = 0;
  quad.error = std::numeric_limits<double>::max();
  if (threadIdx.x < MaxRankedIndex()) {
    const uint32_t packed = kPackedUnrank[threadIdx.x];
    quad.m0 = packed;
    quad.m1 = packed >> 8;
    quad.m2 = packed >> 16;
    quad.m3 = packed >> 24;
    quad.error =
        QuadFitError(storage, quad, num_maxima, max_line_fit_mse, max_dot);
  }
  const QuadError best = FitQuadsStorage::BlockReduce(storage.cub.reduce)
                             .Reduce(quad, MinQuadError());

  if (threadIdx.x == 0) {
    const bool valid = best.error < max_line_fit_mse * count;
    fit_quad_valid_device[blob_index] = valid;
    if (valid) {
      FitQuad &fit_quad = fit_quads_device[blob_index];
      fit_quad.blob_index = blob_index;
      fit_quad.valid = true;
      const uint32_t i0 = storage.point_indices[best.m0];
      const uint32_t i1 = storage.point_indices[best.m1];
      const uint32_t i2 = storage.point_indices[best.m2];
      const uint32_t i3 = storage.point_indices[best.m3];
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

}  // namespace

void FitQuads(
    const Peak *peaks_device, const LineFitPoint *line_fit_points_device,
    const cub::KeyValuePair<long, MinMaxExtents> *selected_extents_device,
    size_t num_blobs, size_t max_blob_points, int nmaxima,
    float max_line_fit_mse, double cos_critical_rad, FitQuad *fit_quads_device,
    bool *fit_quad_valid_device, CudaStream *stream) {
  ABSL_CHECK_EQ(nmaxima, kNMaxima)
      << ": Kernel is compiled and optimized for a fixed nmaxima, please "
         "recompile if you want to change it.";
  // No two neighboring points are both peaks.
  const uint32_t max_peaks = max_blob_points / 2 + 1;
  const size_t peak_keys_bytes = max_peaks * sizeof(uint64_t);
  // Launches get 48 KiB of shared memory unless the kernel asks for more.
  if (peak_keys_bytes + sizeof(FitQuadsStorage) > 48 * 1024) {
    CHECK_CUDA(cudaFuncSetAttribute(DoFitQuads,
                                    cudaFuncAttributeMaxDynamicSharedMemorySize,
                                    peak_keys_bytes));
  }
  DoFitQuads<<<num_blobs, kFitQuadsThreads, peak_keys_bytes, stream->get()>>>(
      peaks_device, line_fit_points_device, selected_extents_device, max_peaks,
      max_line_fit_mse, cos_critical_rad, fit_quads_device,
      fit_quad_valid_device);
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

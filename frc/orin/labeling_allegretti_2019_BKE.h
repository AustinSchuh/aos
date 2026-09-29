#ifndef FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_
#define FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

#include <cuda_runtime.h>

#include <cstdint>
#include <span>

#include "frc/orin/cuda.h"
#include "frc/orin/gpu_image.h"

// The number of pixels a blob needs for LabelImage to find points on its
// boundary.
constexpr uint32_t kMinBlobPixels = 25;

// LabelImage's scratch memory, which carries over from one call to the next.
// Nothing in it needs initializing but num_roots.
struct LabelScratch {
  // One entry per 2x2 block.
  uint32_t *tile_roots;
  // The union-find forest joining the tiles, one entry per pixel.
  uint32_t *forest;
  // The tile-local roots, listed for joining them up.  Three entries per 2x2
  // block, since each of a block's three nodes is listed at most once.
  uint32_t *roots;
  // The length of roots, which must be 0 before the first call.  Every call
  // leaves it 0.
  uint32_t *num_roots;
};

// Labels the connected components of input, and finds the points between the
// ones of kMinBlobPixels or more.
//
// Writes the number of pixels in each component to sizes at its label, and if
// labels is not null, each pixel's label to labels, which is laid out like
// sizes.  Only the entries of sizes labels point to hold sizes; the rest are
// left stale.
//
// Appends each point's QuadBoundaryPoint key to points, in no particular
// order, and adds their number to *num_points.  points needs room for 4 per
// pixel off the image's edge.
//
// If kernel_events is not empty, it holds 4 events to record on stream after
// each of the TileLabel, BorderMerge, Resolve, and LabelAndDiff kernels, for
// timing them.
void LabelImage(const GpuImage<uint8_t> input, GpuImage<uint32_t> sizes,
                uint32_t *labels, const LabelScratch &scratch, uint64_t *points,
                uint32_t *num_points, cudaStream_t stream,
                std::span<frc::apriltag::CudaEvent> kernel_events = {});

#endif  // FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

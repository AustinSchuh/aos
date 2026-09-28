#ifndef FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_
#define FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

#include <cuda_runtime.h>

#include <cstdint>
#include <span>

#include "frc/orin/cuda.h"
#include "frc/orin/gpu_image.h"

// Labels the connected components of input into output, and writes the number
// of pixels in each component to union_markers_size_device at its label.  Only
// the entries labels point to hold sizes; the rest are left stale.  tile_roots
// is scratch for one entry per 2x2 block, and forest for one entry per output
// pixel.  If kernel_events is not empty, it holds 3 events to record on stream
// after each of the TileLabel, BorderMerge, and FinalLabel kernels, for timing
// them.
void LabelImage(const GpuImage<uint8_t> input, GpuImage<uint32_t> output,
                GpuImage<uint32_t> union_markers_size_device,
                uint32_t *tile_roots, uint32_t *forest, cudaStream_t stream,
                std::span<frc::apriltag::CudaEvent> kernel_events = {});

#endif  // FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

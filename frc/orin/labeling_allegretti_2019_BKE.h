#ifndef FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_
#define FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

#include <cuda_runtime.h>

#include <cstdint>
#include <span>

#include "frc/orin/cuda.h"
#include "frc/orin/gpu_image.h"

// Labels the connected components of input into output.  If kernel_events is
// not empty, it holds 5 events to record on stream after each of the
// InitLabeling, Compression, Merge, Compression, and FinalLabeling kernels, for
// timing them.
void LabelImage(const GpuImage<uint8_t> input, GpuImage<uint32_t> output,
                GpuImage<uint32_t> union_markers_size_device,
                cudaStream_t stream,
                std::span<frc::apriltag::CudaEvent> kernel_events = {});

#endif  // FRC_ORIN_LABELING_ALLEGRETTI_2019_BKE_H_

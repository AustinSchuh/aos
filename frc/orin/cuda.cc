#include "frc/orin/cuda.h"

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"

ABSL_FLAG(
    bool, sync, false,
    "If true, force synchronization after each step to isolate errors better.");

namespace frc::apriltag {

size_t overall_memory = 0;

void CudaStream::Wait(CudaEvent *event) {
  CHECK_CUDA(cudaStreamWaitEvent(stream_, event->get(), 0));
}

std::vector<int> CudaGraph::KernelPriorities(const void *kernel) const {
  size_t num_nodes = 0;
  CHECK_CUDA(cudaGraphGetNodes(graph_, nullptr, &num_nodes));
  std::vector<cudaGraphNode_t> nodes(num_nodes);
  CHECK_CUDA(cudaGraphGetNodes(graph_, nodes.data(), &num_nodes));
  std::vector<int> priorities;
  for (cudaGraphNode_t node : nodes) {
    cudaGraphNodeType type;
    CHECK_CUDA(cudaGraphNodeGetType(node, &type));
    if (type != cudaGraphNodeTypeKernel) {
      continue;
    }
    cudaKernelNodeParams params;
    CHECK_CUDA(cudaGraphKernelNodeGetParams(node, &params));
    if (params.func != kernel) {
      continue;
    }
    cudaKernelNodeAttrValue value;
    CHECK_CUDA(cudaGraphKernelNodeGetAttribute(
        node, cudaKernelNodeAttributePriority, &value));
    priorities.push_back(value.priority);
  }
  return priorities;
}

int GreatestStreamPriority() {
  int least_priority;
  int greatest_priority;
  CHECK_CUDA(
      cudaDeviceGetStreamPriorityRange(&least_priority, &greatest_priority));
  return greatest_priority;
}

void CheckAndSynchronize(std::string_view message) {
  CHECK_CUDA(cudaDeviceSynchronize()) << message;
  CHECK_CUDA(cudaGetLastError()) << message;
}

void MaybeCheckAndSynchronize() {
  if (absl::GetFlag(FLAGS_sync)) CheckAndSynchronize();
}

void MaybeCheckAndSynchronize(std::string_view message) {
  if (absl::GetFlag(FLAGS_sync)) CheckAndSynchronize(message);
}

}  // namespace frc::apriltag

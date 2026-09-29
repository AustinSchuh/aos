#ifndef FRC_ORIN_CUDA_H_
#define FRC_ORIN_CUDA_H_

#include <chrono>
#include <span>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

// CHECKs that a cuda method returned success.
// TODO(austin): This will not handle if and else statements quite right, fix if
// we care.
#define CHECK_CUDA(condition)                                                  \
  if (auto c = condition)                                                      \
  ABSL_LOG(FATAL) << "Check failed: " #condition " (" << cudaGetErrorString(c) \
                  << ") "

namespace frc::apriltag {

class CudaEvent;

// Class to manage the lifetime of a Cuda stream.  This is used to provide
// relative ordering between kernels on the same stream.
class CudaStream {
 public:
  CudaStream() { CHECK_CUDA(cudaStreamCreate(&stream_)); }

  CudaStream(const CudaStream &) = delete;
  CudaStream &operator=(const CudaStream &) = delete;
  CudaStream(const CudaStream &&) noexcept = delete;
  CudaStream &operator=(const CudaStream &&) noexcept = delete;

  virtual ~CudaStream() { CHECK_CUDA(cudaStreamDestroy(stream_)); }

  // Returns the stream.
  cudaStream_t get() { return stream_; }

  // Waits until the selected event has been triggered.
  void Wait(CudaEvent *event);

 private:
  cudaStream_t stream_;
};

// Class to manage the lifetime of a Cuda Event.  Cuda events are used for
// timing events on a stream.
class CudaEvent {
 public:
  // flags are the cudaEventCreateWithFlags() flags.  Events which are only
  // waited on, never timed, should pass cudaEventDisableTiming.
  explicit CudaEvent(unsigned int flags = cudaEventDefault) {
    CHECK_CUDA(cudaEventCreateWithFlags(&event_, flags));
  }

  CudaEvent(const CudaEvent &) = delete;
  CudaEvent &operator=(const CudaEvent &) = delete;
  CudaEvent(const CudaEvent &&) noexcept = delete;
  CudaEvent &operator=(const CudaEvent &&) noexcept = delete;

  virtual ~CudaEvent() { CHECK_CUDA(cudaEventDestroy(event_)); }

  // Returns the event.
  cudaEvent_t get() { return event_; }

  // Queues up an event to be timestamped on the stream when it is executed.
  void Record(CudaStream *stream) {
    CHECK_CUDA(cudaEventRecord(event_, stream->get()));
  }

  // Returns the time elapsed between start and this event if it has been
  // triggered.
  std::chrono::nanoseconds ElapsedTime(const CudaEvent &start) {
    float ms;
    CHECK_CUDA(cudaEventElapsedTime(&ms, start.event_, event_));
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<float, std::milli>(ms));
  }

  // Waits until the event has been triggered.
  void Synchronize() { CHECK_CUDA(cudaEventSynchronize(event_)); }

  // Polls the event for up to spin, then waits for it with Synchronize().
  // With cudaEventBlockingSync, polling first trades CPU time for the wake-up
  // latency of the blocking wait on events which trigger soon.
  void Synchronize(std::chrono::nanoseconds spin) {
    if (spin > std::chrono::nanoseconds(0)) {
      const std::chrono::steady_clock::time_point end =
          std::chrono::steady_clock::now() + spin;
      do {
        const cudaError_t result = cudaEventQuery(event_);
        if (result == cudaSuccess) {
          return;
        }
        if (result != cudaErrorNotReady) {
          CHECK_CUDA(result);
        }
      } while (std::chrono::steady_clock::now() < end);
    }
    Synchronize();
  }

 private:
  cudaEvent_t event_;
};

// Class to manage the lifetime of an executable CUDA graph, captured from the
// work a function enqueues on a stream.  Launching it submits all that work in
// one call, which takes less CPU time than launching each piece, and the GPU
// can't run out of queued work while the launching thread is preempted.
class CudaGraph {
 public:
  // Captures what enqueue() enqueues on stream.  enqueue() must enqueue work on
  // stream only, and must not wait for the GPU.
  template <typename Enqueue>
  CudaGraph(CudaStream *stream, Enqueue &&enqueue) {
    CHECK_CUDA(cudaStreamBeginCapture(stream->get(),
                                      cudaStreamCaptureModeThreadLocal));
    enqueue();
    cudaGraph_t graph;
    CHECK_CUDA(cudaStreamEndCapture(stream->get(), &graph));
    CHECK_CUDA(cudaGetLastError());
    // cudaGraphInstantiate's signature differs between CUDA 11 and 12.
    CHECK_CUDA(cudaGraphInstantiateWithFlags(&exec_, graph, 0));
    CHECK_CUDA(cudaGraphDestroy(graph));
  }

  CudaGraph(const CudaGraph &) = delete;
  CudaGraph &operator=(const CudaGraph &) = delete;
  CudaGraph(const CudaGraph &&) noexcept = delete;
  CudaGraph &operator=(const CudaGraph &&) noexcept = delete;

  ~CudaGraph() { CHECK_CUDA(cudaGraphExecDestroy(exec_)); }

  // Enqueues the captured work on stream.
  void Launch(CudaStream *stream) {
    CHECK_CUDA(cudaGraphLaunch(exec_, stream->get()));
  }

 private:
  cudaGraphExec_t exec_;
};

// Class to manage the lifetime of page locked host memory for fast copies back
// to host memory.
template <typename T>
class HostMemory {
 public:
  // Allocates a block of memory for holding up to size objects of type T.
  HostMemory(size_t size) {
    T *memory;
    CHECK_CUDA(cudaMallocHost((void **)(&memory), size * sizeof(T)));
    span_ = std::span<T>(memory, size);
  }
  HostMemory(const HostMemory &) = delete;
  HostMemory &operator=(const HostMemory &) = delete;
  HostMemory(const HostMemory &&) noexcept = delete;
  HostMemory &operator=(const HostMemory &&) noexcept = delete;

  virtual ~HostMemory() { CHECK_CUDA(cudaFreeHost(span_.data())); }

  // Returns a pointer to the memory.
  T *get() { return span_.data(); }
  const T *get() const { return span_.data(); }

  // Returns the number of objects the memory can hold.
  size_t size() const { return span_.size(); }

  // Copies data from other (host memory) to this's memory.
  void MemcpyFrom(const T *other) {
    memcpy(span_.data(), other, sizeof(T) * size());
  }
  void MemcpyFrom(const T *other, const size_t size) {
    memcpy(span_.data(), other, sizeof(T) * size);
  }

  // Copies data to other (host memory) from this's memory.
  void MemcpyTo(T *other) { memcpy(other, span_.data(), sizeof(T) * size()); }

 private:
  std::span<T> span_;
};

// Class to manage the lifetime of page locked host memory which kernels write
// directly through device_ptr(), with no copy on the stream.  The host may only
// read it once an event recorded after the writing kernel has completed.
template <typename T>
class MappedHostMemory {
 public:
  // Allocates a block of memory for holding up to size objects of type T.
  MappedHostMemory(size_t size) {
    T *memory;
    CHECK_CUDA(cudaHostAlloc((void **)(&memory), size * sizeof(T),
                             cudaHostAllocMapped));
    span_ = std::span<T>(memory, size);
    CHECK_CUDA(cudaHostGetDevicePointer((void **)(&device_ptr_), memory, 0));
  }
  MappedHostMemory(const MappedHostMemory &) = delete;
  MappedHostMemory &operator=(const MappedHostMemory &) = delete;
  MappedHostMemory(const MappedHostMemory &&) noexcept = delete;
  MappedHostMemory &operator=(const MappedHostMemory &&) noexcept = delete;

  virtual ~MappedHostMemory() { CHECK_CUDA(cudaFreeHost(span_.data())); }

  // Returns the pointer for the host to read through.  Volatile because the
  // GPU writes it behind the compiler's back.
  const volatile T *host_ptr() const { return span_.data(); }

  // Returns the memory for the host to read as ordinary memory, which structs
  // need since they can't be copied out through host_ptr().  Only read it
  // after the same wait as host_ptr(): the wait is a call the compiler can't
  // see into, so it doesn't reuse anything read before it.
  std::span<const T> host_span() const { return span_; }

  // Returns the pointer for the host to write through.  A kernel launched
  // after the write sees it; only write while no GPU work reading it is in
  // flight.
  volatile T *mutable_host_ptr() { return span_.data(); }

  // Returns the pointer for kernels to use.
  T *device_ptr() { return device_ptr_; }

  // Returns the number of objects the memory can hold.
  size_t size() const { return span_.size(); }

 private:
  std::span<T> span_;
  T *device_ptr_;
};

// Class to manage the lifetime of device memory.
template <typename T>
class GpuMemory {
 public:
  // Allocates a block of memory for holding up to size objects of type T in
  // device memory.
  GpuMemory(size_t size) : size_(size) {
    CHECK_CUDA(cudaMalloc((void **)(&memory_), size * sizeof(T)));
  }
  GpuMemory(const GpuMemory &) = delete;
  GpuMemory &operator=(const GpuMemory &) = delete;
  GpuMemory(const GpuMemory &&) noexcept = delete;
  GpuMemory &operator=(const GpuMemory &&) noexcept = delete;

  virtual ~GpuMemory() { CHECK_CUDA(cudaFree(memory_)); }

  // Returns the device pointer to the memory.
  T *get() { return memory_; }
  const T *get() const { return memory_; }

  // Returns the number of objects this memory can hold.
  size_t size() const { return size_; }

  // Copies data from host memory to this memory asynchronously on the provided
  // stream.
  void MemcpyAsyncFrom(const T *host_memory, CudaStream *stream) {
    CHECK_CUDA(cudaMemcpyAsync(memory_, host_memory, sizeof(T) * size_,
                               cudaMemcpyHostToDevice, stream->get()));
  }
  void MemcpyAsyncFrom(const T *host_memory, const size_t size,
                       CudaStream *stream) {
    ABSL_CHECK_LE(size, size_);
    CHECK_CUDA(cudaMemcpyAsync(memory_, host_memory, sizeof(T) * size,
                               cudaMemcpyHostToDevice, stream->get()));
  }
  void MemcpyAsyncFrom(const HostMemory<T> *host_memory, CudaStream *stream) {
    CHECK_CUDA(cudaMemcpyAsync(memory_, host_memory->get(),
                               sizeof(T) * host_memory->size(),
                               cudaMemcpyHostToDevice, stream->get()));
  }
  void MemcpyAsyncFrom(const HostMemory<T> *host_memory, const size_t size,
                       CudaStream *stream) {
    ABSL_CHECK_LE(size, size_);
    CHECK_CUDA(cudaMemcpyAsync(memory_, host_memory->get(), sizeof(T) * size,
                               cudaMemcpyHostToDevice, stream->get()));
  }

  // Copies data to host memory from this memory asynchronously on the provided
  // stream.
  void MemcpyAsyncTo(T *host_memory, size_t size, CudaStream *stream) const {
    CHECK_CUDA(cudaMemcpyAsync(reinterpret_cast<void *>(host_memory),
                               reinterpret_cast<void *>(memory_),
                               sizeof(T) * size, cudaMemcpyDeviceToHost,
                               stream->get()));
  }
  void MemcpyAsyncTo(T *host_memory, CudaStream *stream) const {
    MemcpyAsyncTo(host_memory, size_, stream);
  }
  void MemcpyAsyncTo(HostMemory<T> *host_memory, CudaStream *stream) const {
    MemcpyAsyncTo(host_memory->get(), stream);
  }

  // Copies data from host_memory to this memory blocking.
  void MemcpyFrom(const T *host_memory) {
    CHECK_CUDA(cudaMemcpy(reinterpret_cast<void *>(memory_),
                          reinterpret_cast<const void *>(host_memory),
                          sizeof(T) * size_, cudaMemcpyHostToDevice));
  }
  void MemcpyFrom(const HostMemory<T> *host_memory) {
    MemcpyFrom(host_memory->get());
  }

  // Copies data to host_memory from this memory.  Only copies size objects.
  void MemcpyTo(T *host_memory, size_t size) const {
    CHECK_CUDA(cudaMemcpy(reinterpret_cast<void *>(host_memory), memory_,
                          sizeof(T) * size, cudaMemcpyDeviceToHost));
  }
  // Copies data to host_memory from this memory.
  void MemcpyTo(T *host_memory) const { MemcpyTo(host_memory, size_); }
  void MemcpyTo(HostMemory<T> *host_memory) const {
    MemcpyTo(host_memory->get());
  }

  // Sets the memory asynchronously to contain data of type 'val' on the provide
  // stream.
  void MemsetAsync(const uint8_t val, CudaStream *stream) const {
    CHECK_CUDA(cudaMemsetAsync(memory_, val, sizeof(T) * size_, stream->get()));
  }

  // Allocates a vector on the host, copies size objects into it, and returns
  // it.
  std::vector<T> Copy(size_t s) const {
    ABSL_CHECK_LE(s, size_);
    std::vector<T> result(s);
    MemcpyTo(result.data(), s);
    return result;
  }

  // Copies all the objects in this memory to a vector on the host and returns
  // it.
  std::vector<T> Copy() const { return Copy(size_); }

 private:
  T *memory_;
  const size_t size_;
};

// Class to manage the lifetime of page locked host memory for fast copies back
// to host memory.
template <typename T>
class UnifiedMemory {
 public:
  // Allocates a block of memory for holding up to size objects of type T.
  UnifiedMemory(size_t size) {
    T *memory;
    CHECK_CUDA(cudaMallocManaged((void **)(&memory), size * sizeof(T)));
    span_ = std::span<T>(memory, size);
  }
  UnifiedMemory(const UnifiedMemory &) = delete;
  UnifiedMemory &operator=(const UnifiedMemory &) = delete;
  UnifiedMemory(const UnifiedMemory &&) noexcept = delete;
  UnifiedMemory &operator=(const UnifiedMemory &&) noexcept = delete;

  virtual ~UnifiedMemory() { CHECK_CUDA(cudaFree(span_.data())); }

  // Returns a pointer to the memory.
  T *get() { return span_.data(); }
  const T *get() const { return span_.data(); }

  // Returns the number of objects the memory can hold.
  size_t size() const { return span_.size(); }

  // Copies data from other (host memory) to this's memory.
  void MemcpyFrom(const T *other) {
    memcpy(span_.data(), other, sizeof(T) * size());
  }
  void MemcpyFrom(const T *other, const size_t size) {
    memcpy(span_.data(), other, sizeof(T) * size);
  }

  // Copies data to other (host memory) from this's memory.
  void MemcpyTo(T *other) const {
    memcpy(other, span_.data(), sizeof(T) * size());
  }

 private:
  std::span<T> span_;
};

// Synchronizes and CHECKs for success the last CUDA operation.
void CheckAndSynchronize(std::string_view message = "");

// Synchronizes and CHECKS iff --sync is passed on the command line.  Makes it
// so we can leave debugging in the code.
void MaybeCheckAndSynchronize();
void MaybeCheckAndSynchronize(std::string_view message);

// Returns how many blocks of kernel, with threads threads each, fit on the GPU
// at once.
template <typename Kernel>
int ResidentBlocks(Kernel kernel, int threads) {
  int device;
  CHECK_CUDA(cudaGetDevice(&device));
  int sms;
  CHECK_CUDA(
      cudaDeviceGetAttribute(&sms, cudaDevAttrMultiProcessorCount, device));
  int blocks_per_sm;
  CHECK_CUDA(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm,
                                                           kernel, threads, 0));
  return sms * blocks_per_sm;
}

}  // namespace frc::apriltag

#endif  // FRC_ORIN_CUDA_H_

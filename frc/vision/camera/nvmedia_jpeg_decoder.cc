#include "frc/vision/camera/nvmedia_jpeg_decoder.h"

#include <string.h>

#include <chrono>
#include <source_location>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_format.h"

#include "aos/realtime.h"
#include "nvmedia_ijpd.h"
#include "nvscibuf.h"
#include "nvscisync.h"

namespace frc::vision {
namespace {

// How long to wait for the engine to finish one frame before giving up on
// it.  A frame takes about 2 ms of engine time.
constexpr std::chrono::microseconds kFenceTimeout = std::chrono::seconds(1);

// A formatted error message held in a fixed buffer, so failing a frame
// allocates nothing: aos::ErrorType keeps a message this short inline.  Use
// like aos::ErrFormat: return Format("%d bytes", n).MakeError();
struct InlineErrorMaker {
  char message[aos::ErrorType::kStaticMessageLength];

  [[nodiscard]] aos::Error MakeError(
      std::source_location location = std::source_location::current()) const {
    return aos::MakeError(std::string_view(message), location);
  }
};

template <typename... Args>
[[nodiscard]] InlineErrorMaker Format(const absl::FormatSpec<Args...> &format,
                                      const Args &...args) {
  InlineErrorMaker maker;
  absl::SNPrintF(maker.message, sizeof(maker.message), format, args...);
  return maker;
}

// Turns an NvMedia call's status into an aos::Status naming the call.
aos::Status NvMediaOk(
    NvMediaStatus status, const char *what,
    std::source_location location = std::source_location::current()) {
  if (status == NVMEDIA_STATUS_OK) {
    return aos::Ok();
  }
  return Format("%s failed with NvMediaStatus %d", what,
                static_cast<int>(status))
      .MakeError(location);
}

// Turns an NvSci call's result into an aos::Status naming the call.
aos::Status NvSciOk(
    NvSciError error, const char *what,
    std::source_location location = std::source_location::current()) {
  if (error == NvSciError_Success) {
    return aos::Ok();
  }
  return Format("%s failed with NvSciError %#x", what,
                static_cast<unsigned>(error))
      .MakeError(location);
}

// Reads a JPEG's dimensions from its frame header (SOFn).  Used instead of
// NvMediaIJPDGetInfo on every frame because that allocates (it creates and
// destroys an NvVideo device context per call).  Returns false if the data
// isn't a JPEG with a frame header.
bool ReadJpegSize(const uint8_t *data, size_t size, uint32_t *width,
                  uint32_t *height) {
  if (size < 4 || data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }
  size_t i = 2;
  while (i + 4 <= size) {
    if (data[i] != 0xFF) {
      return false;
    }
    const uint8_t marker = data[i + 1];
    if (marker == 0xFF) {
      // Fill byte.
      ++i;
      continue;
    }
    if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {
      // Markers without a length.
      i += 2;
      continue;
    }
    if (marker == 0xD9 || marker == 0xDA) {
      // End of image or start of scan before any frame header.
      return false;
    }
    const size_t length = (static_cast<size_t>(data[i + 2]) << 8) | data[i + 3];
    if (length < 2) {
      return false;
    }
    // SOF0 through SOF15, except DHT (C4), JPG (C8) and DAC (CC).
    if (marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 &&
        marker != 0xCC) {
      // Length, precision, height, width.
      if (length < 7 || i + 9 > size) {
        return false;
      }
      *height = (static_cast<uint32_t>(data[i + 5]) << 8) | data[i + 6];
      *width = (static_cast<uint32_t>(data[i + 7]) << 8) | data[i + 8];
      return *width > 0 && *height > 0;
    }
    i += 2 + length;
  }
  return false;
}

NvMediaJPEGInstanceId InstanceId(int engine) {
  switch (engine) {
    case 0:
      return NVMEDIA_JPEG_INSTANCE_0;
    case 1:
      return NVMEDIA_JPEG_INSTANCE_1;
  }
  ABSL_LOG(FATAL) << "There is no NVJPG engine " << engine
                  << "; the Orin has engines 0 and 1";
}

}  // namespace

struct NvMediaJpegDecoder::Modules {
  NvSciBufModule buf = nullptr;
  NvSciSyncModule sync = nullptr;
  NvSciSyncCpuWaitContext wait_context = nullptr;

  Modules() {
    aos::CheckExpected(NvSciOk(NvSciBufModuleOpen(&buf), "NvSciBufModuleOpen"));
    aos::CheckExpected(
        NvSciOk(NvSciSyncModuleOpen(&sync), "NvSciSyncModuleOpen"));
    aos::CheckExpected(
        NvSciOk(NvSciSyncCpuWaitContextAlloc(sync, &wait_context),
                "NvSciSyncCpuWaitContextAlloc"));
  }
  ~Modules() {
    NvSciSyncCpuWaitContextFree(wait_context);
    NvSciSyncModuleClose(sync);
    NvSciBufModuleClose(buf);
  }
};

class NvMediaJpegDecoder::Stream {
 public:
  // Sets up |instance| to decode |width| x |height| images.
  static aos::Result<std::unique_ptr<Stream>> Make(
      const Modules &modules, NvMediaJPEGInstanceId instance, uint32_t width,
      uint32_t height) {
    // Not make_unique: the constructor is private.
    std::unique_ptr<Stream> stream(new Stream(width, height));
    // The decoded 4:2:2 image is 2 bytes per pixel; a JPEG of a camera frame
    // is far smaller than that.
    stream->max_bitstream_bytes_ = width * height * 2;
    stream->decoder_ =
        NvMediaIJPDCreate(width, height, stream->max_bitstream_bytes_,
                          /*supportPartialAccel=*/false, instance);
    if (stream->decoder_ == nullptr) {
      return Format("NvMediaIJPDCreate(%ux%u) failed", width, height)
          .MakeError();
    }
    AOS_RETURN_IF_ERROR(stream->AllocateSurface(modules, instance));
    AOS_RETURN_IF_ERROR(stream->AllocateFence(modules));
    return stream;
  }

  ~Stream() {
    if (decoder_ != nullptr) {
      if (eof_registered_) NvMediaIJPDUnregisterNvSciSyncObj(decoder_, eof_);
      if (surface_registered_) {
        NvMediaIJPDUnregisterNvSciBufObj(decoder_, surface_);
      }
      NvMediaIJPDDestroy(decoder_);
    }
    if (eof_ != nullptr) NvSciSyncObjFree(eof_);
    if (surface_ != nullptr) NvSciBufObjFree(surface_);
  }

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  uint32_t max_bitstream_bytes() const { return max_bitstream_bytes_; }

  // Decodes |bitstream| into the surface and waits for the engine to finish.
  aos::Status Decode(const Modules &modules, NvMediaJPEGInstanceId instance,
                     const NvMediaBitstreamBuffer &bitstream) {
    NvMediaStatus render_status;
    {
      // NVIDIA's submit path allocates on every frame: a 5312 byte malloc in
      // libnvmedia_ijpd_sci, and 24 and 120 byte callocs in
      // NvRmHost1xStreamFlush.  Nothing else in a steady-state decode does.
      aos::ScopedNotRealtime not_realtime;
      render_status =
          NvMediaIJPDRenderYUV(decoder_, surface_, /*downscaleLog2=*/0, 1,
                               &bitstream, /*flags=*/0, instance);
    }
    AOS_RETURN_IF_ERROR(NvMediaOk(render_status, "NvMediaIJPDRenderYUV"));
    NvSciSyncFence fence = NvSciSyncFenceInitializer;
    AOS_RETURN_IF_ERROR(
        NvMediaOk(NvMediaIJPDGetEOFNvSciSyncFence(decoder_, eof_, &fence),
                  "NvMediaIJPDGetEOFNvSciSyncFence"));
    // Sleeps in the kernel until the engine signals the fence.
    const NvSciError wait_error =
        NvSciSyncFenceWait(&fence, modules.wait_context, kFenceTimeout.count());
    NvSciSyncFenceClear(&fence);
    return NvSciOk(wait_error, "Waiting for the NVJPG engine");
  }

  // Copies the decoded Y plane into |gray_out|, packed.
  aos::Status CopyLuma(uint8_t *gray_out) const {
    // The engine wrote behind the CPU cache; drop any stale lines before
    // reading.
    AOS_RETURN_IF_ERROR(
        NvSciOk(NvSciBufObjFlushCpuCacheRange(
                    surface_, luma_offset_,
                    static_cast<uint64_t>(luma_pitch_) * height_),
                "NvSciBufObjFlushCpuCacheRange"));
    if (luma_pitch_ == width_) {
      memcpy(gray_out, luma_, static_cast<size_t>(width_) * height_);
    } else {
      for (uint32_t row = 0; row < height_; ++row) {
        memcpy(gray_out + static_cast<size_t>(row) * width_,
               luma_ + static_cast<size_t>(row) * luma_pitch_, width_);
      }
    }
    return aos::Ok();
  }

 private:
  Stream(uint32_t width, uint32_t height) : width_(width), height_(height) {}

  aos::Status AllocateSurface(const Modules &modules,
                              NvMediaJPEGInstanceId instance) {
    NvSciBufAttrList attributes = nullptr;
    AOS_RETURN_IF_ERROR(
        NvSciOk(NvSciBufAttrListCreate(modules.buf, &attributes),
                "NvSciBufAttrListCreate"));
    const aos::Status filled = FillSurfaceAttributes(instance, attributes);
    NvSciBufAttrList reconciled = nullptr;
    NvSciBufAttrList conflicts = nullptr;
    aos::Status reconciled_status = filled;
    if (aos::IsOk(filled)) {
      reconciled_status = NvSciOk(
          NvSciBufAttrListReconcile(&attributes, 1, &reconciled, &conflicts),
          "NvSciBufAttrListReconcile");
    }
    NvSciBufAttrListFree(attributes);
    if (conflicts != nullptr) NvSciBufAttrListFree(conflicts);
    AOS_RETURN_IF_ERROR(reconciled_status);

    NvSciBufAttrKeyValuePair layout_query[] = {
        {NvSciBufImageAttrKey_PlanePitch, nullptr, 0},
        {NvSciBufImageAttrKey_PlaneOffset, nullptr, 0},
    };
    aos::Status allocated =
        NvSciOk(NvSciBufAttrListGetAttrs(reconciled, layout_query, 2),
                "NvSciBufAttrListGetAttrs");
    if (aos::IsOk(allocated)) {
      // Plane 0 is Y.
      luma_pitch_ = static_cast<const uint32_t *>(layout_query[0].value)[0];
      luma_offset_ = static_cast<const uint64_t *>(layout_query[1].value)[0];
      allocated =
          NvSciOk(NvSciBufObjAlloc(reconciled, &surface_), "NvSciBufObjAlloc");
    }
    NvSciBufAttrListFree(reconciled);
    AOS_RETURN_IF_ERROR(allocated);
    ABSL_CHECK_GE(luma_pitch_, width_);

    const void *base = nullptr;
    AOS_RETURN_IF_ERROR(NvSciOk(NvSciBufObjGetConstCpuPtr(surface_, &base),
                                "NvSciBufObjGetConstCpuPtr"));
    luma_ = static_cast<const uint8_t *>(base) + luma_offset_;
    AOS_RETURN_IF_ERROR(
        NvMediaOk(NvMediaIJPDRegisterNvSciBufObj(decoder_, surface_),
                  "NvMediaIJPDRegisterNvSciBufObj"));
    surface_registered_ = true;
    return aos::Ok();
  }

  aos::Status FillSurfaceAttributes(NvMediaJPEGInstanceId instance,
                                    NvSciBufAttrList attributes) const {
    // IJPD names itself as the engine that will write the buffer.
    AOS_RETURN_IF_ERROR(
        NvMediaOk(NvMediaIJPDFillNvSciBufAttrList(instance, attributes),
                  "NvMediaIJPDFillNvSciBufAttrList"));
    // Pitch-linear YUV 4:2:0, which IJPD documents as an output it writes.
    // (It has no luma-only output.)  The CPU mapping is cached: reading the
    // 1 MB luma plane uncached takes ten times as long.
    const NvSciBufType type = NvSciBufType_Image;
    const bool need_cpu_access = true;
    const bool enable_cpu_cache = true;
    const NvSciBufAttrValAccessPerm permission = NvSciBufAccessPerm_ReadWrite;
    const NvSciBufAttrValImageLayoutType layout = NvSciBufImage_PitchLinearType;
    const NvSciBufAttrValImageScanType scan = NvSciBufScan_ProgressiveType;
    const NvSciBufSurfType surface_type = NvSciSurfType_YUV;
    const NvSciBufSurfMemLayout memory_layout = NvSciSurfMemLayout_SemiPlanar;
    const NvSciBufSurfSampleType sampling = NvSciSurfSampleType_420;
    const NvSciBufSurfBPC bits = NvSciSurfBPC_8;
    const NvSciBufSurfComponentOrder order = NvSciSurfComponentOrder_YUV;
    const NvSciBufAttrValColorStd color = NvSciColorStd_REC601_ER;
    // The engine writes whole 16x16 blocks, and IJPD refuses a surface that
    // doesn't cover them ("Surface resolution is small" for a 300 wide
    // image).  Only width x height of it is copied out.
    const uint32_t surface_width = (width_ + 15) & ~15u;
    const uint32_t surface_height = (height_ + 15) & ~15u;
    NvSciBufAttrKeyValuePair pairs[] = {
        {NvSciBufGeneralAttrKey_Types, &type, sizeof(type)},
        {NvSciBufGeneralAttrKey_NeedCpuAccess, &need_cpu_access,
         sizeof(need_cpu_access)},
        {NvSciBufGeneralAttrKey_EnableCpuCache, &enable_cpu_cache,
         sizeof(enable_cpu_cache)},
        {NvSciBufGeneralAttrKey_RequiredPerm, &permission, sizeof(permission)},
        {NvSciBufImageAttrKey_Layout, &layout, sizeof(layout)},
        {NvSciBufImageAttrKey_ScanType, &scan, sizeof(scan)},
        {NvSciBufImageAttrKey_SurfType, &surface_type, sizeof(surface_type)},
        {NvSciBufImageAttrKey_SurfMemLayout, &memory_layout,
         sizeof(memory_layout)},
        {NvSciBufImageAttrKey_SurfSampleType, &sampling, sizeof(sampling)},
        {NvSciBufImageAttrKey_SurfBPC, &bits, sizeof(bits)},
        {NvSciBufImageAttrKey_SurfComponentOrder, &order, sizeof(order)},
        {NvSciBufImageAttrKey_SurfColorStd, &color, sizeof(color)},
        {NvSciBufImageAttrKey_SurfWidthBase, &surface_width,
         sizeof(surface_width)},
        {NvSciBufImageAttrKey_SurfHeightBase, &surface_height,
         sizeof(surface_height)},
    };
    return NvSciOk(NvSciBufAttrListSetAttrs(attributes, pairs,
                                            sizeof(pairs) / sizeof(pairs[0])),
                   "NvSciBufAttrListSetAttrs");
  }

  // The engine signals |eof_| when a frame is done; the CPU waits on it.
  aos::Status AllocateFence(const Modules &modules) {
    NvSciSyncAttrList lists[2] = {nullptr, nullptr};
    AOS_RETURN_IF_ERROR(
        NvSciOk(NvSciSyncAttrListCreate(modules.sync, &lists[0]),
                "NvSciSyncAttrListCreate"));
    aos::Status status =
        NvSciOk(NvSciSyncAttrListCreate(modules.sync, &lists[1]),
                "NvSciSyncAttrListCreate");
    if (aos::IsOk(status)) {
      status = NvMediaOk(NvMediaIJPDFillNvSciSyncAttrList(decoder_, lists[0],
                                                          NVMEDIA_SIGNALER),
                         "NvMediaIJPDFillNvSciSyncAttrList");
    }
    if (aos::IsOk(status)) {
      const bool need_cpu_access = true;
      const NvSciSyncAccessPerm permission = NvSciSyncAccessPerm_WaitOnly;
      NvSciSyncAttrKeyValuePair pairs[] = {
          {NvSciSyncAttrKey_NeedCpuAccess, &need_cpu_access,
           sizeof(need_cpu_access)},
          {NvSciSyncAttrKey_RequiredPerm, &permission, sizeof(permission)},
      };
      status = NvSciOk(NvSciSyncAttrListSetAttrs(lists[1], pairs, 2),
                       "NvSciSyncAttrListSetAttrs");
    }
    NvSciSyncAttrList reconciled = nullptr;
    NvSciSyncAttrList conflicts = nullptr;
    if (aos::IsOk(status)) {
      status =
          NvSciOk(NvSciSyncAttrListReconcile(lists, 2, &reconciled, &conflicts),
                  "NvSciSyncAttrListReconcile");
    }
    for (NvSciSyncAttrList list : lists) {
      if (list != nullptr) NvSciSyncAttrListFree(list);
    }
    if (conflicts != nullptr) NvSciSyncAttrListFree(conflicts);
    AOS_RETURN_IF_ERROR(status);

    status = NvSciOk(NvSciSyncObjAlloc(reconciled, &eof_), "NvSciSyncObjAlloc");
    NvSciSyncAttrListFree(reconciled);
    AOS_RETURN_IF_ERROR(status);
    AOS_RETURN_IF_ERROR(NvMediaOk(
        NvMediaIJPDRegisterNvSciSyncObj(decoder_, NVMEDIA_EOFSYNCOBJ, eof_),
        "NvMediaIJPDRegisterNvSciSyncObj"));
    eof_registered_ = true;
    return NvMediaOk(NvMediaIJPDSetNvSciSyncObjforEOF(decoder_, eof_),
                     "NvMediaIJPDSetNvSciSyncObjforEOF");
  }

  const uint32_t width_;
  const uint32_t height_;
  uint32_t max_bitstream_bytes_ = 0;

  NvMediaIJPD *decoder_ = nullptr;
  NvSciBufObj surface_ = nullptr;
  NvSciSyncObj eof_ = nullptr;
  bool surface_registered_ = false;
  bool eof_registered_ = false;

  const uint8_t *luma_ = nullptr;
  uint32_t luma_pitch_ = 0;
  uint64_t luma_offset_ = 0;
};

NvMediaJpegDecoder::NvMediaJpegDecoder(int engine)
    : engine_(engine), modules_(std::make_unique<Modules>()) {
  const NvMediaJPEGInstanceId instance = InstanceId(engine_);
  NvMediaVersion version;
  aos::CheckExpected(
      NvMediaOk(NvMediaIJPDGetVersion(&version), "NvMediaIJPDGetVersion"));
  ABSL_CHECK_EQ(version.major, NVMEDIA_IJPD_VERSION_MAJOR)
      << ": libnvmedia_ijpd_sci is IJPD " << static_cast<int>(version.major)
      << "." << static_cast<int>(version.minor)
      << ", which these declarations do not describe";
  // Fail now, not on the first frame, if the engine is unusable.
  NvMediaIJPD *probe =
      NvMediaIJPDCreate(16, 16, 1024,
                        /*supportPartialAccel=*/false, instance);
  ABSL_CHECK(probe != nullptr)
      << ": NvMediaIJPDCreate failed on NVJPG engine " << engine_
      << "; is /dev/nvmap readable and the tegra-nvjpg driver bound?";
  NvMediaIJPDDestroy(probe);
  ABSL_LOG(INFO) << "Decoding on NVJPG engine " << engine_ << " (IJPD "
                 << static_cast<int>(version.major) << "."
                 << static_cast<int>(version.minor) << "."
                 << static_cast<int>(version.patch) << ")";
}

NvMediaJpegDecoder::~NvMediaJpegDecoder() {
  // The stream's NvSci objects must go before the modules.
  stream_.reset();
}

aos::Result<NvMediaJpegDecoder::ImageSize> NvMediaJpegDecoder::DecodeToGray(
    const uint8_t *jpeg_data, size_t jpeg_size, uint8_t *gray_out,
    size_t max_out_size) {
  // IJPD reads the bitstream through a non-const pointer but does not write
  // it.
  const NvMediaBitstreamBuffer bitstream = {const_cast<uint8_t *>(jpeg_data),
                                            static_cast<uint32_t>(jpeg_size),
                                            static_cast<uint32_t>(jpeg_size)};
  ImageSize size;
  if (!ReadJpegSize(jpeg_data, jpeg_size, &size.width, &size.height)) {
    return Format("Not a JPEG: no frame header in %d bytes",
                  static_cast<int>(jpeg_size))
        .MakeError();
  }
  if (static_cast<size_t>(size.width) * size.height > max_out_size) {
    return Format("%ux%u image does not fit in %d bytes", size.width,
                  size.height, static_cast<int>(max_out_size))
        .MakeError();
  }

  const NvMediaJPEGInstanceId instance = InstanceId(engine_);
  if (stream_ == nullptr || stream_->width() != size.width ||
      stream_->height() != size.height) {
    // Setting the engine up for a new size allocates.
    aos::ScopedNotRealtime not_realtime;
    stream_.reset();
    NVMEDIAJPEGDecInfo info;
    if (NvMediaIJPDGetInfo(&info, 1, &bitstream) != NVMEDIA_STATUS_OK) {
      return Format("Not a JPEG the NVJPG engine can parse (%d bytes)",
                    static_cast<int>(jpeg_size))
          .MakeError();
    }
    if (info.partialAccel) {
      // The engine would need the CPU's help with this JPEG.  Later frames
      // like it fail in NvMediaIJPDRenderYUV instead, since the decoder is
      // made without partial acceleration.
      return Format("%dx%d JPEG is not fully hardware-decodable", info.width,
                    info.height)
          .MakeError();
    }
    if (info.width != size.width || info.height != size.height) {
      return Format("JPEG header says %ux%u, NvMedia says %dx%d", size.width,
                    size.height, info.width, info.height)
          .MakeError();
    }
    AOS_ASSIGN_OR_RETURN_ERROR(
        stream_, Stream::Make(*modules_, instance, size.width, size.height));
  }
  if (jpeg_size > stream_->max_bitstream_bytes()) {
    return Format("%d byte JPEG is larger than the %u bytes allowed",
                  static_cast<int>(jpeg_size), stream_->max_bitstream_bytes())
        .MakeError();
  }

  const aos::Status decoded = stream_->Decode(*modules_, instance, bitstream);
  if (!aos::IsOk(decoded)) {
    // The engine may still own the surface; start over with a new one.
    // Tearing the stream down frees.
    aos::ScopedNotRealtime not_realtime;
    stream_.reset();
    return aos::MakeError(decoded.error());
  }
  AOS_RETURN_IF_ERROR(stream_->CopyLuma(gray_out));
  return size;
}

}  // namespace frc::vision

// Declarations for NVIDIA's NvMedia image JPEG decoder (IJPD), the API of
// libnvmedia_ijpd_sci.so.  Jetson Linux R39 ships the library but not its
// header, so these are transcribed from NVIDIA's public DRIVE OS 6.0 API
// reference: nvmedia_ijpd.h, nvmedia_common_encode_decode.h and
// nvmedia_core.h, IJPD version 1.
//   https://developer.nvidia.com/docs/drive/drive-os/6.0.6/public/drive-os-linux-sdk/api_reference/nvmedia__ijpd_8h_source.html
//   https://developer.nvidia.com/docs/drive/drive-os/6.0.10/public/drive-os-linux-sdk/api_reference/group__x__image__jpeg__decode__api.html
// Only what the decoder uses is declared.  The core types (NvMediaStatus,
// NvMediaVersion and the NvSciSync enums) match the
// nvmedia_core.h in NVIDIA's Jetson_SIPL_API_R39.2.1 tarball, which is the
// only NvMedia header R39 ships; the R39.2.1 library reports IJPD 1.1.0.
#ifndef THIRD_PARTY_NVMEDIA_NVMEDIA_IJPD_H_
#define THIRD_PARTY_NVMEDIA_NVMEDIA_IJPD_H_
#include <stdbool.h>
#include <stdint.h>
#include "nvscibuf.h"
#include "nvscisync.h"
#ifdef __cplusplus
extern "C" {
#endif  // THIRD_PARTY_NVMEDIA_NVMEDIA_IJPD_H_
typedef enum {
  NVMEDIA_STATUS_OK = 0, NVMEDIA_STATUS_BAD_PARAMETER = 1, NVMEDIA_STATUS_PENDING = 2,
  NVMEDIA_STATUS_TIMED_OUT = 3, NVMEDIA_STATUS_OUT_OF_MEMORY = 4,
  NVMEDIA_STATUS_NOT_INITIALIZED = 5, NVMEDIA_STATUS_NOT_SUPPORTED = 6,
  NVMEDIA_STATUS_ERROR = 7, NVMEDIA_STATUS_NONE_PENDING = 8,
  NVMEDIA_STATUS_INSUFFICIENT_BUFFERING = 9, NVMEDIA_STATUS_INVALID_SIZE = 10,
  NVMEDIA_STATUS_INCOMPATIBLE_VERSION = 11, NVMEDIA_STATUS_UNDEFINED_STATE = 13,
  NVMEDIA_STATUS_PFSD_ERROR = 14, NVMEDIA_STATUS_INVALID_STATE = 15
} NvMediaStatus;
typedef struct { uint8_t major, minor, patch; } NvMediaVersion;
typedef enum { NVMEDIA_SIGNALER, NVMEDIA_WAITER, NVMEDIA_SIGNALER_WAITER } NvMediaNvSciSyncClientType;
typedef enum { NVMEDIA_PRESYNCOBJ, NVMEDIA_EOFSYNCOBJ, NVMEDIA_SOFSYNCOBJ, NVMEDIA_EOF_PRESYNCOBJ,
               NVMEDIA_SOF_PRESYNCOBJ } NvMediaNvSciSyncObjType;
typedef enum { NVMEDIA_JPEG_INSTANCE_0 = 0, NVMEDIA_JPEG_INSTANCE_1, NVMEDIA_JPEG_INSTANCE_AUTO } NvMediaJPEGInstanceId;
typedef struct { uint8_t *bitstream; uint32_t bitstreamBytes; uint32_t bitstreamSize; } NvMediaBitstreamBuffer;
#define NVMEDIA_IJPD_VERSION_MAJOR 1
#define NVMEDIA_IJPD_VERSION_MINOR 0
#define NVMEDIA_MAX_JPEG_APP_MARKERS 16
typedef struct { uint16_t marker; uint16_t len; void *pMarker; } NvMediaJPEGAppMarkerInfo;
typedef struct {
  uint16_t width, height;
  uint8_t partialAccel;
  uint8_t num_app_markers;
  NvMediaJPEGAppMarkerInfo appMarkerInfo[NVMEDIA_MAX_JPEG_APP_MARKERS];
} NVMEDIAJPEGDecInfo;
typedef struct NvMediaIJPD NvMediaIJPD;
NvMediaStatus NvMediaIJPDGetVersion(NvMediaVersion *version);
NvMediaIJPD *NvMediaIJPDCreate(uint16_t maxWidth, uint16_t maxHeight, uint32_t maxBitstreamBytes,
                               bool supportPartialAccel, NvMediaJPEGInstanceId instanceId);
void NvMediaIJPDDestroy(NvMediaIJPD *decoder);
NvMediaStatus NvMediaIJPDGetInfo(NVMEDIAJPEGDecInfo *info, uint32_t numBitstreamBuffers,
                                 const NvMediaBitstreamBuffer *bitstreams);
NvMediaStatus NvMediaIJPDRenderYUV(const NvMediaIJPD *decoder, NvSciBufObj target, uint8_t downscaleLog2,
                                   uint32_t numBitstreamBuffers, const NvMediaBitstreamBuffer *bitstreams,
                                   uint32_t flags, NvMediaJPEGInstanceId instanceId);
NvMediaStatus NvMediaIJPDRegisterNvSciBufObj(const NvMediaIJPD *decoder, NvSciBufObj bufObj);
NvMediaStatus NvMediaIJPDUnregisterNvSciBufObj(const NvMediaIJPD *decoder, NvSciBufObj bufObj);
NvMediaStatus NvMediaIJPDFillNvSciBufAttrList(NvMediaJPEGInstanceId instanceId, NvSciBufAttrList attrlist);
NvMediaStatus NvMediaIJPDFillNvSciSyncAttrList(const NvMediaIJPD *decoder, NvSciSyncAttrList attrlist,
                                               NvMediaNvSciSyncClientType clienttype);
NvMediaStatus NvMediaIJPDRegisterNvSciSyncObj(const NvMediaIJPD *decoder, NvMediaNvSciSyncObjType syncobjtype,
                                              NvSciSyncObj nvscisync);
NvMediaStatus NvMediaIJPDUnregisterNvSciSyncObj(const NvMediaIJPD *decoder, NvSciSyncObj nvscisync);
NvMediaStatus NvMediaIJPDSetNvSciSyncObjforEOF(const NvMediaIJPD *decoder, NvSciSyncObj nvscisyncEOF);
NvMediaStatus NvMediaIJPDGetEOFNvSciSyncFence(const NvMediaIJPD *decoder, NvSciSyncObj eofnvscisyncobj,
                                              NvSciSyncFence *eofnvscisyncfence);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // THIRD_PARTY_NVMEDIA_NVMEDIA_IJPD_H_

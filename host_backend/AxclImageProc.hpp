#pragma once

// AXCL IVPS image-processing helpers. Each moves a host frame into device CMM,
// runs the IVPS op on the card, and copies the result back; returns false on
// failure so the caller can fall back to the host CPU path.

#include "ax_global_type.h"

#include <string>

namespace stereo_depth::host_backend {

// Bring up / tear down AXCL SYS + IVPS (shares the AXCL runtime).
bool axclImageProcInit();
void axclImageProcShutdown();

// NV12 crop/resize (stretch) on the card.
bool axclCropResizeNv12(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst);

// NV12 -> BGR888 on the card.
bool axclCscNv12ToBgr(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst);

// GDC dewarp on the card via AXCL_IVPS_Dewarp (per-eye user mesh).
bool axclDewarpInit(const std::string& leftMeshPath, const std::string& rightMeshPath, int srcWidth,
                    int srcHeight, int dstWidth, int dstHeight);
bool axclDewarpReady();

// Dewarp one eye (0 = left, 1 = right) from a host NV12 src to a host NV12 dst.
bool axclDewarpEye(int eye, const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst);

void axclDewarpShutdown();

}  // namespace stereo_depth::host_backend

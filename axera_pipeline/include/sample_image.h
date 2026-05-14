#ifndef SAMPLE_IMAGE_H
#define SAMPLE_IMAGE_H

#include "ax_dsp_api.h"
#include "sample_resource.h"

#ifdef __cplusplus
extern "C" {
#endif

AX_S32 Get_FileSize(const AX_CHAR* filename);
AX_S32 sample_load_bin(const AX_CHAR* pszBinFileName, AX_VOID* vaddr);
AX_S32 sample_save_bin(const char* pszBinFileName, AX_VOID* vaddr, AX_U32 size);
AX_S32 sample_release_image_frame(SAMPLE_RESOURCE_T* pResource);
AX_S32 sample_create_image_frame(SAMPLE_RESOURCE_T* pResource, AX_U32 inputWidth,
                                 AX_U32 inputHeight);
AX_S32 sample_create_processing_image_frame(SAMPLE_RESOURCE_T* pResource, AX_U32 inputWidth,
                                            AX_U32 inputHeight);

AX_S32 sample_csc_yuyv_nv12(AX_VIDEO_FRAME_T* pSrcFrame, AX_VIDEO_FRAME_T* pCscFrame);
AX_S32 sample_crop_resize(const AX_VIDEO_FRAME_T* pInFrame, AX_VIDEO_FRAME_T* pOutFrame);
AX_S32 sample_csc_nv12_bgr(const AX_VIDEO_FRAME_T* pInFrame, AX_VIDEO_FRAME_T* pOutFrame);

#ifdef __cplusplus
}
#endif

#endif

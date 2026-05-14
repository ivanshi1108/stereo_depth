#ifndef SAMPLE_GDC_H
#define SAMPLE_GDC_H

#include "ax_ivps_api.h"
#include "ax_sys_api.h"
#include "sample_resource.h"

#ifdef __cplusplus
extern "C" {
#endif

AX_S32 sample_load_mesh_file_to_cmm(const AX_CHAR* pFile, AX_VOID** ppVirAddr, AX_U64* pU64PhyAddr,
                                    AX_U32* pMeshTableSize);
AX_S32 sample_gdc_init(AX_CHAR* pMeshFile, const AX_VIDEO_FRAME_T* pSrcFrame,
                       AX_VIDEO_FRAME_T* pDstFrame, GDC_HANDLE* pGdcHandle, AX_U64* pMeshPhyAddr,
                       AX_VOID** pMeshVirAddr);
AX_S32 sample_gdc_deinit(SAMPLE_RESOURCE_T* pResource, AX_S32 idx);

#ifdef __cplusplus
}
#endif

#endif

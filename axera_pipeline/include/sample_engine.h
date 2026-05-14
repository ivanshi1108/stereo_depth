#ifndef _SAMPLE_NPU_ENGINE_H_
#define _SAMPLE_NPU_ENGINE_H_

#include <ax_base_type.h>
#include <ax_global_type.h>
#include "ax_engine_api.h"
#include "ax_ivps_api.h"
#include "ax_sys_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SAMPLE_MODEL_OUTPUT_S {
    AX_U32 width;
    AX_U32 height;
    AX_U32 size;
    AX_U64 phyAddr;
    AX_VOID* virAddr;
    AX_U32 confidenceSize;
    AX_U64 confidencePhyAddr;
    AX_VOID* confidenceVirAddr;
    AX_BLK blockId;
} SAMPLE_MODEL_OUTPUT_T;

AX_S32 sample_npu_init(const AX_CHAR* modelPath);

AX_S32 sample_run_axmodel(const AX_VIDEO_FRAME_T* pstFrame1, const AX_VIDEO_FRAME_T* pstFrame2,
                          SAMPLE_MODEL_OUTPUT_T* pstOutput);

AX_S32 sample_npu_deinit(AX_VOID);

#ifdef __cplusplus
}
#endif

#endif

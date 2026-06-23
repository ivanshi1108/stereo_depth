#ifndef _SAMPLE_NPU_ENGINE_H_
#define _SAMPLE_NPU_ENGINE_H_

// Host/AXCL port: the NPU is reached over PCIe through libaxcl_rt, so this
// header only needs the AX data types (frame description, scalar typedefs).
// The implementation lives in host_backend/sample_engine_axcl.cpp.
#include <ax_base_type.h>
#include <ax_global_type.h>
#include <ax_pool_type.h>

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

/* Release the per-frame host output buffer produced by sample_run_axmodel. */
void sample_engine_release_output(SAMPLE_MODEL_OUTPUT_T* pstOutput);

#ifdef __cplusplus
}
#endif

#endif

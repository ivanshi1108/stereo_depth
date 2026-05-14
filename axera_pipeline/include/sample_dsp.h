#ifndef SAMPLE_DSP_H
#define SAMPLE_DSP_H

#include "ax_dsp_api.h"
#include "ax_dsp_slam_api.h"
#include "ax_ivps_api.h"

#ifdef __cplusplus
extern "C" {
#endif

AX_S32 SAMPLE_DSP_Init(AX_DSP_ID_E dspid, char* itcm, char* sram, char* dtcm, char* dtcm2);
AX_S32 SAMPLE_DSP_DeInit(AX_DSP_ID_E dspid);

typedef struct SAMPLE_DSP_SGBM_CTX_S {
    AX_DSP_SLAM_SGBM_T sgbm;
    AX_U64 memPhyAddr;
    AX_VOID* memVirAddr;
    AX_U32 memSize;
} SAMPLE_DSP_SGBM_CTX_T;

typedef struct SAMPLE_DSP_SGBM_RUN_PARAM_S {
    AX_DSP_ID_E primaryDspId;
    AX_DSP_ID_E secondaryDspId;
    AX_BOOL dualCore;
    AX_S32 overlapRows;
    AX_DSP_SLAM_SGBM_T* pSgbm;
    AX_U64 memPhyAddr;
    AX_VOID* memVirAddr;
    AX_U32 memSize;
} SAMPLE_DSP_SGBM_RUN_PARAM_T;

AX_S32 sample_dsp_sgbm_prepare(const AX_VIDEO_FRAME_T* pLeftFrame,
                               const AX_VIDEO_FRAME_T* pRightFrame, SAMPLE_DSP_SGBM_CTX_T* pCtx);
AX_VOID sample_dsp_sgbm_release(SAMPLE_DSP_SGBM_CTX_T* pCtx);
AX_S32 sample_run_sgbm(SAMPLE_DSP_SGBM_RUN_PARAM_T* pParam);
AX_VOID sample_dsp_sgbm_deinit(AX_VOID);

#ifdef __cplusplus
}
#endif

#endif

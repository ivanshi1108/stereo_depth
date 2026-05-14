#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include "ax_dmadim_api.h"
#include "ax_sys_api.h"
#include "sample_dsp.h"

static AX_U64 g_sgbmTmpDispConfPhy = 0;
static AX_VOID* g_sgbmTmpDispConfVir = NULL;
static AX_U32 g_sgbmTmpDispConfSize = 0;
static AX_U64 g_sgbmTmpDispPostPhy = 0;
static AX_VOID* g_sgbmTmpDispPostVir = NULL;
static AX_U32 g_sgbmTmpDispPostSize = 0;
static const AX_U32 g_sgbmPostSysBuffers = 7;

typedef struct SAMPLE_DSP_SGBM_THREAD_JOB_S {
    AX_DSP_ID_E dspId;
    AX_DSP_SLAM_SGBM_T* pSgbm;
    AX_S32 ret;
} SAMPLE_DSP_SGBM_THREAD_JOB_T;

static void* sample_run_sgbm_thread(void* arg) {
    SAMPLE_DSP_SGBM_THREAD_JOB_T* job = (SAMPLE_DSP_SGBM_THREAD_JOB_T*)arg;
    if (job == NULL || job->pSgbm == NULL) {
        return NULL;
    }

    job->ret = AX_DSP_SLAM_SgbmDisparityCompute(job->dspId, job->pSgbm);
    return NULL;
}

static AX_U64 align_up64(AX_U64 value, AX_U64 align) { return (value + align - 1) / align * align; }

static AX_S32 ensure_sgbm_tmp_buffer(AX_U64* pPhyAddr, AX_VOID** pVirAddr, AX_U32* pCurSize,
                                     AX_U32 requiredSize, const AX_S8* token) {
    if (requiredSize <= *pCurSize && *pVirAddr != NULL) {
        return 0;
    }

    if (*pVirAddr != NULL) {
        AX_SYS_MemFree(*pPhyAddr, *pVirAddr);
        *pPhyAddr = 0;
        *pVirAddr = NULL;
        *pCurSize = 0;
    }

    AX_U64 phyAddr = 0;
    AX_VOID* virAddr = NULL;
    AX_S32 ret = AX_SYS_MemAlloc(&phyAddr, &virAddr, requiredSize, 128, token);
    if (ret != 0) {
        printf("AX_SYS_MemAlloc %s fail, ret=0x%x\n", token, ret);
        return ret;
    }

    *pPhyAddr = phyAddr;
    *pVirAddr = virAddr;
    *pCurSize = requiredSize;
    return 0;
}

AX_S32 SAMPLE_DSP_Init(AX_DSP_ID_E dspid, char* itcm, char* sram, char* dtcm, char* dtcm2) {
    AX_S32 ret;
    ret = AX_DSP_PowerOn(dspid);
    if (ret != AX_DSP_SUCCESS) {
        printf("AX DSP Poweron error %x\n", ret);
        return -1;
    }
    ret = AX_DSP_LoadBin(dspid, sram, AX_DSP_MEM_TYPE_SRAM);
    if (ret != AX_DSP_SUCCESS) {
        printf("AX DSP LoadBin error %x\n", ret);
        return -1;
    }
    ret = AX_DSP_LoadBin(dspid, itcm, AX_DSP_MEM_TYPE_ITCM);
    if (ret != AX_DSP_SUCCESS) {
        printf("AX DSP LoadBin error %x\n", ret);
        return -1;
    }
    if (dtcm) {
        ret = AX_DSP_LoadBin(dspid, dtcm, AX_DSP_MEM_TYPE_DTCM);
        if (ret != AX_DSP_SUCCESS) {
            printf("AX DSP LoadBin error %x\n", ret);
            return -1;
        }
    }
    if (dtcm2) {
        ret = AX_DSP_LoadBin(dspid, dtcm2, AX_DSP_MEM_TYPE_DTCM_2);
        if (ret != AX_DSP_SUCCESS) {
            printf("AX DSP LoadBin error %x\n", ret);
            return -1;
        }
    }
    ret = AX_DSP_EnableCore(dspid);
    if (ret != AX_DSP_SUCCESS) {
        printf("AX DSP Enable Core error %x\n", ret);
        return -1;
    }
    return 0;
}

AX_S32 SAMPLE_DSP_DeInit(AX_DSP_ID_E dspid) {
    AX_DSP_DisableCore(dspid);
    AX_DSP_PowerOff(dspid);

    return 0;
}

AX_S32 sample_dsp_sgbm_prepare(const AX_VIDEO_FRAME_T* pLeftFrame,
                               const AX_VIDEO_FRAME_T* pRightFrame, SAMPLE_DSP_SGBM_CTX_T* pCtx) {
    if (pLeftFrame == NULL || pRightFrame == NULL || pCtx == NULL) {
        return -1;
    }

    const AX_U32 width = pLeftFrame->u32Width;
    const AX_U32 height = pLeftFrame->u32Height;
    const AX_U32 pitch = pLeftFrame->u32PicStride[0];
    if (width == 0 || height == 0 || pitch < width) {
        return -1;
    }

    AX_U64 offset = 0;
    const AX_U64 pitchHeight = (AX_U64)pitch * (AX_U64)height;
    offset = align_up64(offset, 64);
    const AX_U64 dispOffset = offset;
    offset += pitchHeight * sizeof(AX_S16);

    offset = align_up64(offset, 64);
    const AX_U64 dispConfOffset = offset;
    offset += pitchHeight * sizeof(AX_U8);

    offset = align_up64(offset, 64);
    const AX_U64 dispPostOffset = offset;
    offset += pitchHeight * sizeof(AX_U16);

    offset = align_up64(offset, 64);
    const AX_U64 postSysOffset = offset;
    offset += pitchHeight * sizeof(AX_F32) * g_sgbmPostSysBuffers;

    AX_U64 phyAddr = 0;
    AX_VOID* virAddr = NULL;
    AX_U32 totalSize = (AX_U32)offset;
    AX_S32 ret = AX_SYS_MemAlloc(&phyAddr, &virAddr, totalSize, 128, (AX_S8*)"sgbmFrame");
    if (ret != 0) {
        return ret;
    }
    memset(virAddr, 0, totalSize);

    memset(pCtx, 0, sizeof(*pCtx));
    pCtx->memPhyAddr = phyAddr;
    pCtx->memVirAddr = virAddr;
    pCtx->memSize = totalSize;

    AX_U8* base = (AX_U8*)virAddr;
    pCtx->sgbm.imageLeft.y = (AX_U8*)pLeftFrame->u64VirAddr[0];
    pCtx->sgbm.imageLeft.width = width;
    pCtx->sgbm.imageLeft.pitch = pitch;
    pCtx->sgbm.imageLeft.height = height;
    pCtx->sgbm.imageRight.y = (AX_U8*)pRightFrame->u64VirAddr[0];
    pCtx->sgbm.imageRight.width = pRightFrame->u32Width;
    pCtx->sgbm.imageRight.pitch = pRightFrame->u32PicStride[0];
    pCtx->sgbm.imageRight.height = pRightFrame->u32Height;

    pCtx->sgbm.disp = (AX_S16*)(base + dispOffset);
    pCtx->sgbm.dispConf = (AX_U8*)(base + dispConfOffset);
    pCtx->sgbm.dispPostOut = (AX_U16*)(base + dispPostOffset);
    pCtx->sgbm.postSysMem = (AX_F32*)(base + postSysOffset);
    pCtx->sgbm.postControl = AX_TRUE;
    pCtx->sgbm.sgbm_ctl.SADWindowSize = AX_DSP_SLAM_SGBM_WIN_SIZE;
    pCtx->sgbm.sgbm_ctl.P1 =
        8 * pCtx->sgbm.sgbm_ctl.SADWindowSize * pCtx->sgbm.sgbm_ctl.SADWindowSize;
    pCtx->sgbm.sgbm_ctl.P2 =
        32 * pCtx->sgbm.sgbm_ctl.SADWindowSize * pCtx->sgbm.sgbm_ctl.SADWindowSize;
    pCtx->sgbm.sgbm_ctl.minDisparity = 0;
    pCtx->sgbm.sgbm_ctl.numberOfDisparities = AX_DSP_SLAM_SGBM_MAX_DISPARITY;
    pCtx->sgbm.sgbm_ctl.uniquenessRatio = AX_DSP_SLAM_SGBM_UNIQUENESS_RATIO;
    pCtx->sgbm.sgbm_ctl.disp12MaxDiff = AX_DSP_SLAM_SGBM_DISP_LEFT_RIGHT_MAX_DIFF;
    pCtx->sgbm.sgbm_ctl.preFilterCap = AX_DSP_SLAM_SGBM_PRE_FILT_CAP;

    return 0;
}

AX_VOID sample_dsp_sgbm_release(SAMPLE_DSP_SGBM_CTX_T* pCtx) {
    if (pCtx == NULL) {
        return;
    }
    if (pCtx->memVirAddr != NULL) {
        AX_SYS_MemFree(pCtx->memPhyAddr, pCtx->memVirAddr);
    }
    memset(pCtx, 0, sizeof(*pCtx));
}

AX_S32 sample_run_sgbm(SAMPLE_DSP_SGBM_RUN_PARAM_T* pParam) {
    if (pParam == NULL || pParam->pSgbm == NULL) {
        return -1;
    }

    AX_DSP_SLAM_SGBM_T* pSgbm = pParam->pSgbm;
    AX_S32 ret = 0;

    if (!pParam->dualCore) {
        ret = AX_DSP_SLAM_SgbmDisparityCompute(pParam->primaryDspId, pSgbm);
        if (ret != 0) {
            return ret;
        }
        if (pParam->memVirAddr != NULL && pParam->memSize > 0) {
            AX_SYS_MinvalidateCache(pParam->memPhyAddr, pParam->memVirAddr, pParam->memSize);
        }
        return 0;
    }

    AX_DSP_SLAM_SGBM_T sgbmTop = *pSgbm;
    const AX_S32 imageHeight = (AX_S32)sgbmTop.imageLeft.height;
    const AX_S32 halfHeight = imageHeight / 2;
    AX_S32 overlapRows = pParam->overlapRows;
    if (overlapRows < 1) {
        overlapRows = 1;
    }
    if (overlapRows > (halfHeight - 1)) {
        overlapRows = halfHeight - 1;
    }
    sgbmTop.imageLeft.height = (AX_U32)(halfHeight + overlapRows);
    sgbmTop.imageRight.height = (AX_U32)(halfHeight + overlapRows);

    const AX_U32 pitch = sgbmTop.imageLeft.pitch;
    const AX_U32 partHeight = sgbmTop.imageLeft.height;
    const AX_U32 dispConfSize = pitch * partHeight * sizeof(AX_U8);
    const AX_U32 dispPostSize = pitch * partHeight * sizeof(AX_U16);

    ret = ensure_sgbm_tmp_buffer(&g_sgbmTmpDispConfPhy, &g_sgbmTmpDispConfVir,
                                 &g_sgbmTmpDispConfSize, dispConfSize, (AX_S8*)"sgbmDispConf");
    if (ret != 0) {
        return ret;
    }
    ret = ensure_sgbm_tmp_buffer(&g_sgbmTmpDispPostPhy, &g_sgbmTmpDispPostVir,
                                 &g_sgbmTmpDispPostSize, dispPostSize, (AX_S8*)"sgbmDispPost");
    if (ret != 0) {
        return ret;
    }

    AX_DSP_SLAM_SGBM_T sgbmBottom = sgbmTop;
    const AX_S32 startRow = (AX_S32)partHeight - overlapRows * 2;
    const size_t rowOffsetLeft = (size_t)sgbmBottom.imageLeft.pitch * (size_t)startRow;
    const size_t rowOffsetRight = (size_t)sgbmBottom.imageRight.pitch * (size_t)startRow;
    sgbmBottom.imageLeft.y += rowOffsetLeft;
    sgbmBottom.imageRight.y += rowOffsetRight;
    sgbmBottom.disp += (size_t)sgbmBottom.imageLeft.pitch * (size_t)startRow;
    sgbmBottom.dispConf = (AX_U8*)g_sgbmTmpDispConfVir;
    sgbmBottom.dispPostOut = (AX_U16*)g_sgbmTmpDispPostVir;

    SAMPLE_DSP_SGBM_THREAD_JOB_T bottomJob;
    memset(&bottomJob, 0, sizeof(bottomJob));
    bottomJob.dspId = pParam->secondaryDspId;
    bottomJob.pSgbm = &sgbmBottom;

    pthread_t bottomThread;
    int threadCreateRet = pthread_create(&bottomThread, NULL, sample_run_sgbm_thread, &bottomJob);
    if (threadCreateRet != 0) {
        printf("pthread_create for dual-core SGBM failed, ret=%d, fallback to serial mode\n",
               threadCreateRet);

        ret = AX_DSP_SLAM_SgbmDisparityCompute(pParam->primaryDspId, &sgbmTop);
        if (ret != 0) {
            return ret;
        }
        ret = AX_DSP_SLAM_SgbmDisparityCompute(pParam->secondaryDspId, &sgbmBottom);
        if (ret != 0) {
            return ret;
        }
    } else {
        ret = AX_DSP_SLAM_SgbmDisparityCompute(pParam->primaryDspId, &sgbmTop);

        AX_S32 joinRet = pthread_join(bottomThread, NULL);
        if (joinRet != 0) {
            printf("pthread_join for dual-core SGBM failed, ret=%d\n", joinRet);
            return -1;
        }

        if (ret != 0) {
            return ret;
        }
        if (bottomJob.ret != 0) {
            return bottomJob.ret;
        }
    }

    AX_U64 dstPhyAddr = 0;
    AX_S32 cacheType = 0;

    /* DMA copy dispConf (AX_U8) */
    AX_U64 dstSize = (AX_U64)pitch * (AX_U64)(imageHeight / 2) * sizeof(AX_U8);
    AX_U64 offset = (AX_U64)pitch * (AX_U64)overlapRows * sizeof(AX_U8);
    AX_SYS_MemGetBlockInfoByVirt((AX_VOID*)pSgbm->dispConf, &dstPhyAddr, &cacheType);
    AX_U64 srcDmaAddr = g_sgbmTmpDispConfPhy + offset;
    AX_U64 dstDmaAddr = dstPhyAddr + dstSize;
    ret = AX_DMA_MemCopy(dstDmaAddr, srcDmaAddr, dstSize);
    if (ret != 0) {
        return ret;
    }

    /* DMA copy dispPostOut (AX_U16) */
    dstSize = (AX_U64)pitch * (AX_U64)(imageHeight / 2) * sizeof(AX_U16);
    offset = (AX_U64)pitch * (AX_U64)overlapRows * sizeof(AX_U16);
    AX_SYS_MemGetBlockInfoByVirt((AX_VOID*)pSgbm->dispPostOut, &dstPhyAddr, &cacheType);
    srcDmaAddr = g_sgbmTmpDispPostPhy + offset;
    dstDmaAddr = dstPhyAddr + dstSize;
    ret = AX_DMA_MemCopy(dstDmaAddr, srcDmaAddr, dstSize);
    if (ret != 0) {
        return ret;
    }

    /* Full disparity FGS smoothing (overwrites dispPostOut with better quality) */
    ret = AX_DSP_SLAM_SgbmFastGlobalSmoother(pParam->primaryDspId, pSgbm);
    if (ret != 0) {
        printf("AX_DSP_SLAM_SgbmFastGlobalSmoother fail ret=0x%x, using per-half post\n", ret);
    }

    if (pParam->memVirAddr != NULL && pParam->memSize > 0) {
        AX_SYS_MinvalidateCache(pParam->memPhyAddr, pParam->memVirAddr, pParam->memSize);
    }

    return 0;
}

AX_VOID sample_dsp_sgbm_deinit(AX_VOID) {
    if (g_sgbmTmpDispConfVir != NULL) {
        AX_SYS_MemFree(g_sgbmTmpDispConfPhy, g_sgbmTmpDispConfVir);
        g_sgbmTmpDispConfPhy = 0;
        g_sgbmTmpDispConfVir = NULL;
        g_sgbmTmpDispConfSize = 0;
    }
    if (g_sgbmTmpDispPostVir != NULL) {
        AX_SYS_MemFree(g_sgbmTmpDispPostPhy, g_sgbmTmpDispPostVir);
        g_sgbmTmpDispPostPhy = 0;
        g_sgbmTmpDispPostVir = NULL;
        g_sgbmTmpDispPostSize = 0;
    }
}

#include "sample_engine.h"
#include "file.hpp"

#define SAMPLE_LOG_TAG "ENGINE"
#include "sample_log.h"
#include "string.h"
#include "timer.hpp"

typedef struct SAMPLE_ENGINE_S {
    AX_ENGINE_HANDLE engineHandle;
    AX_ENGINE_IO_INFO_T* ioInfo;
    std::vector<AX_CHAR> modelBuffer;
    AX_POOL poolId;
    AX_BLK blockId;
    AX_ENGINE_IO_T ioData;
    std::vector<AX_ENGINE_IO_BUFFER_T> inputs;
    std::vector<AX_ENGINE_IO_BUFFER_T> outputs;
} SAMPLE_ENGINE_T;

static SAMPLE_ENGINE_T g_engineMsg;

// extern "C" {

/* Run model on frame data */
AX_S32 sample_run_axmodel(const AX_VIDEO_FRAME_T* pstFrame1, const AX_VIDEO_FRAME_T* pstFrame2,
                          SAMPLE_MODEL_OUTPUT_T* pstOutput) {
    AX_S32 ret;
    SAMPLE_ENGINE_T* pEngineMsg = &g_engineMsg;

    /* Add parameter validation */
    if (!pstFrame1 || !pstFrame2 || !pstOutput) {
        ALOGE("Invalid parameters passed to sample_run_axmodel");
        return AX_ERR_NULL_PTR;
    }

    auto& inputBuffer = pEngineMsg->inputs[0];
    inputBuffer.pVirAddr = (AX_VOID*)pstFrame1->u64VirAddr[0];
    inputBuffer.phyAddr = pstFrame1->u64PhyAddr[0];
    inputBuffer.nSize = pEngineMsg->ioInfo->pInputs[0].nSize;
    inputBuffer.pStride = AX_NULL;
    inputBuffer.nStrideSize = 0;

    auto& inputBuffer1 = pEngineMsg->inputs[1];
    inputBuffer1.pVirAddr = (AX_VOID*)pstFrame2->u64VirAddr[0];
    inputBuffer1.phyAddr = pstFrame2->u64PhyAddr[0];
    inputBuffer1.nSize = pEngineMsg->ioInfo->pInputs[1].nSize;
    inputBuffer1.pStride = AX_NULL;
    inputBuffer1.nStrideSize = 0;

    pEngineMsg->blockId = AX_INVALID_BLOCKID;

    /* Calculate total memory required for output tensors only */
    AX_SIZE_T totalOutputSize = 0;
    AX_SIZE_T alignedOutputSizes[3] = {0}; /* Aligned sizes for output tensors */
    AX_SIZE_T outputOffsets[3] = {0};      /* Offsets within the output memory block */

    /* Calculate output sizes and align them */
    for (AX_U32 i = 0; i < pEngineMsg->ioData.nOutputSize && i < 2; i++) {
        /* Align to 128 bytes */
        alignedOutputSizes[i] = (pEngineMsg->ioInfo->pOutputs[i].nSize + 127) & ~127;
        outputOffsets[i] = totalOutputSize;
        totalOutputSize += alignedOutputSizes[i];
    }

    /* Allocate memory block for outputs only */
    AX_BLK blkId = AX_POOL_GetBlock(pEngineMsg->poolId, totalOutputSize, AX_NULL);
    if (blkId == AX_INVALID_BLOCKID) {
        ALOGE("Failed to allocate output memory block of size %u", totalOutputSize);
        return AX_FALSE;
    }

    /* Store the block ID in the outputs structure */
    pEngineMsg->blockId = blkId;

    /* Get base virtual address of the output block */
    AX_VOID* outputBaseVirAddr = AX_POOL_GetBlockVirAddr(blkId);
    if (!outputBaseVirAddr) {
        ALOGE("Failed to get virtual address for output block");
        AX_POOL_ReleaseBlock(blkId);
        pEngineMsg->blockId = AX_INVALID_BLOCKID;
        return AX_FALSE;
    }

    /* Get physical address of the output block */
    AX_U64 outputBasePhyAddr = AX_POOL_Handle2PhysAddr(blkId);

    /* Invalidate cache for the output memory block before setup */
    AX_SYS_MinvalidateCache(outputBasePhyAddr, outputBaseVirAddr, totalOutputSize);

    /* Set up output buffers with offsets in the allocated output block */
    for (AX_U32 i = 0; i < pEngineMsg->ioData.nOutputSize && i < 2; i++) {
        auto& outputBuffer = pEngineMsg->outputs[i];

        /* Calculate pointer to this output tensor */
        outputBuffer.pVirAddr = (AX_U8*)outputBaseVirAddr + outputOffsets[i];
        outputBuffer.phyAddr = outputBasePhyAddr + outputOffsets[i];
        outputBuffer.nSize = pEngineMsg->ioInfo->pOutputs[i].nSize;
        outputBuffer.pStride = AX_NULL;
        outputBuffer.nStrideSize = 0;
    }

    /* Run model */
    // timer tick;
    // AX_F32 inferenceTime = 0.0f;
    // tick.start();
    ret = AX_ENGINE_RunSync(pEngineMsg->engineHandle, &pEngineMsg->ioData);
    // inferenceTime = tick.cost();
    // ALOGI("AX_ENGINE_RunSync Inference time: %.2f ms\n", inferenceTime);
    if (0 != ret) {
        ALOGE("AX_ENGINE_RunSync Error code: %X", ret);
        AX_POOL_ReleaseBlock(pEngineMsg->blockId);
        pEngineMsg->blockId = AX_INVALID_BLOCKID;
        return ret;
    }

    /* Invalidate cache for outputs before reading results */
    for (AX_U32 i = 0; i < pEngineMsg->ioData.nOutputSize; i++) {
        AX_SYS_MinvalidateCache(pEngineMsg->outputs[i].phyAddr, pEngineMsg->outputs[i].pVirAddr,
                                pEngineMsg->outputs[i].nSize);
    }

    pstOutput->width = pstFrame1->u32Width;
    pstOutput->height = pstFrame1->u32Height;
    pstOutput->size = pEngineMsg->outputs[0].nSize;
    pstOutput->phyAddr = pEngineMsg->outputs[0].phyAddr;
    pstOutput->virAddr = pEngineMsg->outputs[0].pVirAddr;
    pstOutput->confidenceSize = 0;
    pstOutput->confidencePhyAddr = 0;
    pstOutput->confidenceVirAddr = AX_NULL;
    if (pEngineMsg->ioData.nOutputSize > 1) {
        pstOutput->confidenceSize = pEngineMsg->outputs[1].nSize;
        pstOutput->confidencePhyAddr = pEngineMsg->outputs[1].phyAddr;
        pstOutput->confidenceVirAddr = pEngineMsg->outputs[1].pVirAddr;
    }
    pstOutput->blockId = pEngineMsg->blockId;

    return 0;
}

AX_S32 sample_npu_init(const AX_CHAR* modelPath) {
    AX_S32 ret = 0;
    SAMPLE_ENGINE_T* pEngineMsg = &g_engineMsg;

    /* 1. Initialize engine Should be initialized by the caller */
    // AX_ENGINE_NPU_ATTR_T attr;
    // memset(&attr, 0, sizeof(AX_ENGINE_NPU_ATTR_T));
    // attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    // ret = AX_ENGINE_Init(&attr);
    // if (0 != ret) {
    //     ALOGE("AX_ENGINE_Init failed with code %d\n", ret);
    //     return -1;
    // }

    /* 2. Load model */
    if (!utilities::read_file(modelPath, pEngineMsg->modelBuffer)) {
        ALOGE("Read model file(%s) failed.", modelPath);
        return AX_ERR_ILLEGAL_PARAM;
    }

    /* 3. Create engine handle */
    ret = AX_ENGINE_CreateHandle(&pEngineMsg->engineHandle, pEngineMsg->modelBuffer.data(),
                                 pEngineMsg->modelBuffer.size());
    if (0 != ret) {
        ALOGE("AX_ENGINE_CreateHandle failed: %X", ret);
        return ret;
    }
    ALOGI("Engine creating handle is done.");

    /* 4. Create context */
    ret = AX_ENGINE_CreateContext(pEngineMsg->engineHandle);
    if (0 != ret) {
        ALOGE("AX_ENGINE_CreateContext failed: %X", ret);
        AX_ENGINE_DestroyHandle(pEngineMsg->engineHandle);
        return ret;
    }
    ALOGI("Engine creating context is done.");

    /* 5. Set IO info */
    ret = AX_ENGINE_GetIOInfo(pEngineMsg->engineHandle, &pEngineMsg->ioInfo);
    if (0 != ret) {
        ALOGE("AX_ENGINE_GetIOInfo failed: %X", ret);
        AX_ENGINE_DestroyHandle(pEngineMsg->engineHandle);
        return ret;
    }
    ALOGI("Engine get io info is done.");

    /* 6. Set Affinity */
    // AX_ENGINE_NPU_SET_T nNpuSet = 0b10;
    // ret = AX_ENGINE_SetAffinity(pEngineMsg->engineHandle, nNpuSet);
    // if (0 != ret) {
    //     ALOGE("AX_ENGINE_SetAffinity failed: %X\n", ret);
    //     AX_ENGINE_DestroyHandle(pEngineMsg->engineHandle);
    //     return ret;
    // }
    // ALOGI("Engine set affinity is done.\n");

    /* Initialize ioData structure and resize vectors for inputs and outputs */
    memset(&pEngineMsg->ioData, 0, sizeof(AX_ENGINE_IO_T));
    pEngineMsg->ioData.nInputSize = pEngineMsg->ioInfo->nInputSize;
    pEngineMsg->ioData.nOutputSize = pEngineMsg->ioInfo->nOutputSize;
    ALOGI("ioData.nInputSize:%d, nOutputSize:%d", pEngineMsg->ioData.nInputSize,
          pEngineMsg->ioData.nOutputSize);

    /* Resize input and output buffer vectors */
    pEngineMsg->inputs.resize(pEngineMsg->ioInfo->nInputSize);
    pEngineMsg->outputs.resize(pEngineMsg->ioInfo->nOutputSize);

    /* Set pointers in ioData to point to our vectors */
    pEngineMsg->ioData.pInputs = pEngineMsg->inputs.data();
    pEngineMsg->ioData.pOutputs = pEngineMsg->outputs.data();

    /* 6. Create memory pool for I/O operations */
    /* Calculate aligned sizes for output tensors */
    AX_SIZE_T alignedOutputSize = 0;
    for (AX_U32 i = 0; i < pEngineMsg->ioInfo->nOutputSize; i++) {
        alignedOutputSize += (pEngineMsg->ioInfo->pOutputs[i].nSize + 127) & ~127;
    }

    /* Add margin for other allocations */
    AX_SIZE_T blockSize = (alignedOutputSize * 11 / 10 + 127) & ~127;

    /* 7. Create pool configuration */
    AX_POOL_CONFIG_T poolConfig;
    memset(&poolConfig, 0, sizeof(AX_POOL_CONFIG_T));
    poolConfig.MetaSize = 0x1000;
    poolConfig.BlkSize = blockSize;
    poolConfig.BlkCnt = 16;
    poolConfig.CacheMode = POOL_CACHE_MODE_CACHED;
    strcpy((AX_CHAR*)poolConfig.PartitionName, "anonymous");

    /* Create the pool */
    pEngineMsg->poolId = AX_POOL_CreatePool(&poolConfig);
    if (pEngineMsg->poolId == AX_INVALID_POOLID) {
        ALOGE("AX_POOL_CreatePool error!!");
        AX_ENGINE_DestroyHandle(pEngineMsg->engineHandle);
        return -1;
    }

    /* Map the pool */
    ret = AX_POOL_MmapPool(pEngineMsg->poolId);
    if (ret != 0) {
        ALOGE("AX_POOL_MmapPool fail!!");
        AX_POOL_DestroyPool(pEngineMsg->poolId);
        AX_ENGINE_DestroyHandle(pEngineMsg->engineHandle);
        return -1;
    }

    return 0;
}

AX_S32 sample_npu_deinit(AX_VOID) {
    SAMPLE_ENGINE_T* pEngineMsg = &g_engineMsg;

    // Destroy engine handle if it exists
    if (pEngineMsg->engineHandle != AX_NULL) {
        AX_ENGINE_DestroyHandle(pEngineMsg->engineHandle);
        pEngineMsg->engineHandle = AX_NULL;
    }

    // Clear model buffer
    pEngineMsg->modelBuffer.clear();

    // Reset other fields
    pEngineMsg->ioInfo = AX_NULL;
    pEngineMsg->inputs.clear();
    pEngineMsg->outputs.clear();

    if (pEngineMsg->blockId != AX_INVALID_BLOCKID) {
        AX_POOL_ReleaseBlock(pEngineMsg->blockId);
    }
    AX_POOL_MunmapPool(pEngineMsg->poolId);
    AX_POOL_DestroyPool(pEngineMsg->poolId);

    return 0;
}
// } /* end of extern "C" */
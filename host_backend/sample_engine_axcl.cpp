// Host/AXCL implementation of the stereo NPU engine: runs the .axmodel on the
// Axera card via the AXCL runtime engine API (axclrt*).

#include "sample_engine.h"

#include <axcl.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

#include "AxclRuntime.hpp"

#define SAMPLE_LOG_TAG "ENGINE"
#include "sample_log.h"

using stereo_depth::host_backend::AxclRuntime;

namespace {

struct AxclEngine {
    bool loaded = false;
    bool runtimeAcquired = false;

    uint64_t modelId = 0;
    uint64_t contextId = 0;
    axclrtEngineIOInfo ioInfo = nullptr;
    axclrtEngineIO io = nullptr;
    void* modelDevMem = nullptr;

    uint32_t numInputs = 0;
    uint32_t numOutputs = 0;
    std::vector<void*> inputDev;
    std::vector<uint64_t> inputSize;
    std::vector<void*> outputDev;
    std::vector<uint64_t> outputSize;

    std::mutex runMutex;
};

AxclEngine g_engine;

void destroyEngineLocked() {
    if (g_engine.io != nullptr) {
        axclrtEngineDestroyIO(g_engine.io);
        g_engine.io = nullptr;
    }
    for (void* p : g_engine.inputDev) {
        if (p != nullptr) {
            axclrtFree(p);
        }
    }
    for (void* p : g_engine.outputDev) {
        if (p != nullptr) {
            axclrtFree(p);
        }
    }
    g_engine.inputDev.clear();
    g_engine.inputSize.clear();
    g_engine.outputDev.clear();
    g_engine.outputSize.clear();

    if (g_engine.ioInfo != nullptr) {
        axclrtEngineDestroyIOInfo(g_engine.ioInfo);
        g_engine.ioInfo = nullptr;
    }
    if (g_engine.loaded) {
        axclrtEngineUnload(g_engine.modelId);
        g_engine.loaded = false;
    }
    if (g_engine.modelDevMem != nullptr) {
        axclrtFree(g_engine.modelDevMem);
        g_engine.modelDevMem = nullptr;
    }
    g_engine.numInputs = 0;
    g_engine.numOutputs = 0;
}

bool readFile(const char* path, std::vector<uint8_t>& out) {
    FILE* fp = fopen(path, "rb");
    if (fp == nullptr) {
        return false;
    }
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (size <= 0) {
        fclose(fp);
        return false;
    }
    out.resize(static_cast<size_t>(size));
    const size_t got = fread(out.data(), 1, out.size(), fp);
    fclose(fp);
    return got == out.size();
}

}  // namespace

AX_S32 sample_npu_init(const AX_CHAR* modelPath) {
    if (g_engine.loaded) {
        return 0;
    }

    AxclRuntime& runtime = AxclRuntime::instance();
    if (!runtime.acquire()) {
        ALOGE("failed to acquire AXCL runtime");
        return -1;
    }
    g_engine.runtimeAcquired = true;
    runtime.ensureThreadContext();

    std::vector<uint8_t> modelBuffer;
    if (!readFile(modelPath, modelBuffer)) {
        ALOGE("read model file(%s) failed", modelPath);
        runtime.release();
        g_engine.runtimeAcquired = false;
        return -1;
    }

    axclError ret =
        axclrtMalloc(&g_engine.modelDevMem, modelBuffer.size(), AXCL_MEM_MALLOC_HUGE_FIRST);
    if (ret != 0 || g_engine.modelDevMem == nullptr) {
        ALOGE("axclrtMalloc(model %zu) failed: 0x%x", modelBuffer.size(), ret);
        goto fail;
    }
    ret = axclrtMemcpy(g_engine.modelDevMem, modelBuffer.data(), modelBuffer.size(),
                       AXCL_MEMCPY_HOST_TO_DEVICE);
    if (ret != 0) {
        ALOGE("axclrtMemcpy(model) failed: 0x%x", ret);
        goto fail;
    }

    ret = axclrtEngineLoadFromMem(g_engine.modelDevMem, modelBuffer.size(), &g_engine.modelId);
    if (ret != 0) {
        ALOGE("axclrtEngineLoadFromMem failed: 0x%x", ret);
        goto fail;
    }
    g_engine.loaded = true;
    ALOGI("axclrtEngineLoadFromMem done, modelId=%llu",
          static_cast<unsigned long long>(g_engine.modelId));

    ret = axclrtEngineCreateContext(g_engine.modelId, &g_engine.contextId);
    if (ret != 0) {
        ALOGE("axclrtEngineCreateContext failed: 0x%x", ret);
        goto fail;
    }

    ret = axclrtEngineGetIOInfo(g_engine.modelId, &g_engine.ioInfo);
    if (ret != 0) {
        ALOGE("axclrtEngineGetIOInfo failed: 0x%x", ret);
        goto fail;
    }

    g_engine.numInputs = axclrtEngineGetNumInputs(g_engine.ioInfo);
    g_engine.numOutputs = axclrtEngineGetNumOutputs(g_engine.ioInfo);
    ALOGI("model io: inputs=%u outputs=%u", g_engine.numInputs, g_engine.numOutputs);
    if (g_engine.numInputs < 2 || g_engine.numOutputs < 1) {
        ALOGE("unexpected stereo model io layout (inputs=%u outputs=%u)", g_engine.numInputs,
              g_engine.numOutputs);
        goto fail;
    }

    ret = axclrtEngineCreateIO(g_engine.ioInfo, &g_engine.io);
    if (ret != 0) {
        ALOGE("axclrtEngineCreateIO failed: 0x%x", ret);
        goto fail;
    }

    g_engine.inputDev.assign(g_engine.numInputs, nullptr);
    g_engine.inputSize.assign(g_engine.numInputs, 0);
    g_engine.outputDev.assign(g_engine.numOutputs, nullptr);
    g_engine.outputSize.assign(g_engine.numOutputs, 0);

    for (uint32_t i = 0; i < g_engine.numInputs; ++i) {
        const uint64_t size = axclrtEngineGetInputSizeByIndex(g_engine.ioInfo, 0, i);
        void* dev = nullptr;
        ret = axclrtMalloc(&dev, size, AXCL_MEM_MALLOC_HUGE_FIRST);
        if (ret != 0 || dev == nullptr) {
            ALOGE("axclrtMalloc(input %u, %llu) failed: 0x%x", i,
                  static_cast<unsigned long long>(size), ret);
            goto fail;
        }
        axclrtMemset(dev, 0, size);
        g_engine.inputDev[i] = dev;
        g_engine.inputSize[i] = size;
        ret = axclrtEngineSetInputBufferByIndex(g_engine.io, i, dev, size);
        if (ret != 0) {
            ALOGE("axclrtEngineSetInputBufferByIndex(%u) failed: 0x%x", i, ret);
            goto fail;
        }
    }

    for (uint32_t i = 0; i < g_engine.numOutputs; ++i) {
        const uint64_t size = axclrtEngineGetOutputSizeByIndex(g_engine.ioInfo, 0, i);
        void* dev = nullptr;
        ret = axclrtMalloc(&dev, size, AXCL_MEM_MALLOC_HUGE_FIRST);
        if (ret != 0 || dev == nullptr) {
            ALOGE("axclrtMalloc(output %u, %llu) failed: 0x%x", i,
                  static_cast<unsigned long long>(size), ret);
            goto fail;
        }
        g_engine.outputDev[i] = dev;
        g_engine.outputSize[i] = size;
        ret = axclrtEngineSetOutputBufferByIndex(g_engine.io, i, dev, size);
        if (ret != 0) {
            ALOGE("axclrtEngineSetOutputBufferByIndex(%u) failed: 0x%x", i, ret);
            goto fail;
        }
    }

    ALOGN("NPU engine ready (AXCL): inputs=%u outputs=%u", g_engine.numInputs, g_engine.numOutputs);
    return 0;

fail:
    destroyEngineLocked();
    if (g_engine.runtimeAcquired) {
        runtime.release();
        g_engine.runtimeAcquired = false;
    }
    return -1;
}

AX_S32 sample_run_axmodel(const AX_VIDEO_FRAME_T* pstFrame1, const AX_VIDEO_FRAME_T* pstFrame2,
                          SAMPLE_MODEL_OUTPUT_T* pstOutput) {
    if (pstFrame1 == nullptr || pstFrame2 == nullptr || pstOutput == nullptr) {
        ALOGE("invalid parameters passed to sample_run_axmodel");
        return AX_ERR_NULL_PTR;
    }
    if (!g_engine.loaded) {
        ALOGE("engine not initialized");
        return -1;
    }

    AxclRuntime::instance().ensureThreadContext();

    std::lock_guard<std::mutex> lock(g_engine.runMutex);

    const AX_VIDEO_FRAME_T* frames[2] = {pstFrame1, pstFrame2};
    for (uint32_t i = 0; i < g_engine.numInputs && i < 2; ++i) {
        const AX_VIDEO_FRAME_T* frame = frames[i];
        const void* hostSrc = reinterpret_cast<const void*>(frame->u64VirAddr[0]);
        uint64_t copyBytes = g_engine.inputSize[i];
        if (frame->u32FrameSize != 0 && copyBytes > frame->u32FrameSize) {
            copyBytes = frame->u32FrameSize;
        }
        axclError ret =
            axclrtMemcpy(g_engine.inputDev[i], hostSrc, copyBytes, AXCL_MEMCPY_HOST_TO_DEVICE);
        if (ret != 0) {
            ALOGE("axclrtMemcpy input %u H2D failed: 0x%x", i, ret);
            return ret;
        }
    }

    axclError ret = axclrtEngineExecute(g_engine.modelId, g_engine.contextId, 0, g_engine.io);
    if (ret != 0) {
        ALOGE("axclrtEngineExecute failed: 0x%x", ret);
        return ret;
    }

    // Pack all outputs into a single host buffer owned by the frame context.
    // virAddr points at output 0 (disparity), confidenceVirAddr at output 1.
    uint64_t totalOut = 0;
    for (uint32_t i = 0; i < g_engine.numOutputs; ++i) {
        totalOut += g_engine.outputSize[i];
    }
    auto* hostOut = static_cast<uint8_t*>(std::malloc(static_cast<size_t>(totalOut)));
    if (hostOut == nullptr) {
        ALOGE("alloc host output (%llu) failed", static_cast<unsigned long long>(totalOut));
        return -1;
    }

    uint64_t offset = 0;
    uint64_t outputOffsets[2] = {0, 0};
    for (uint32_t i = 0; i < g_engine.numOutputs; ++i) {
        if (i < 2) {
            outputOffsets[i] = offset;
        }
        ret = axclrtMemcpy(hostOut + offset, g_engine.outputDev[i], g_engine.outputSize[i],
                           AXCL_MEMCPY_DEVICE_TO_HOST);
        if (ret != 0) {
            ALOGE("axclrtMemcpy output %u D2H failed: 0x%x", i, ret);
            std::free(hostOut);
            return ret;
        }
        offset += g_engine.outputSize[i];
    }

    pstOutput->width = pstFrame1->u32Width;
    pstOutput->height = pstFrame1->u32Height;
    pstOutput->size = static_cast<AX_U32>(g_engine.outputSize[0]);
    pstOutput->phyAddr = 0;
    pstOutput->virAddr = hostOut + outputOffsets[0];
    pstOutput->confidenceSize = 0;
    pstOutput->confidencePhyAddr = 0;
    pstOutput->confidenceVirAddr = AX_NULL;
    if (g_engine.numOutputs > 1) {
        pstOutput->confidenceSize = static_cast<AX_U32>(g_engine.outputSize[1]);
        pstOutput->confidenceVirAddr = hostOut + outputOffsets[1];
    }
    // blockId is repurposed: 0 (AX_INVALID_BLOCKID). The host output buffer is
    // released by the pipeline via sample_engine_release_output().
    pstOutput->blockId = AX_INVALID_BLOCKID;

    return 0;
}

AX_S32 sample_npu_deinit(AX_VOID) {
    std::lock_guard<std::mutex> lock(g_engine.runMutex);
    destroyEngineLocked();
    if (g_engine.runtimeAcquired) {
        AxclRuntime::instance().release();
        g_engine.runtimeAcquired = false;
    }
    return 0;
}

void sample_engine_release_output(SAMPLE_MODEL_OUTPUT_T* pstOutput) {
    if (pstOutput == nullptr) {
        return;
    }
    // virAddr is the base of the packed host output allocation (output 0 sits
    // at offset 0), so freeing it releases the whole block.
    if (pstOutput->virAddr != AX_NULL) {
        std::free(pstOutput->virAddr);
    }
    pstOutput->virAddr = AX_NULL;
    pstOutput->confidenceVirAddr = AX_NULL;
    pstOutput->size = 0;
    pstOutput->confidenceSize = 0;
    pstOutput->blockId = AX_INVALID_BLOCKID;
}

#include "ax_stereo_depth_api.h"
#include "StereoDepthPipeline.hpp"

#include <cstring>
#include <memory>

using stereo_depth::GdcMeshMode;
using stereo_depth::InferenceEngine;
using stereo_depth::InputFrameFormat;
using stereo_depth::PipelineOutput;
using stereo_depth::StereoDepthPipeline;

struct AX_STEREO_PIPELINE_CTX {
    StereoDepthPipeline pipeline;
    AX_STEREO_ATTR_T attr;
};

struct AX_STEREO_FRAME_CTX_INTERNAL {
    StereoDepthPipeline::FrameContextPtr frameContext;
};

struct AX_STEREO_OUTPUT_INTERNAL {
    PipelineOutput pipelineOutput;
};

static InferenceEngine toInternalEngine(AX_STEREO_ENGINE_E e) {
    switch (e) {
        case AX_STEREO_ENGINE_NPU:
        default:
            return InferenceEngine::NPU;
    }
}

static AX_STEREO_ENGINE_E toApiEngine(InferenceEngine e) {
    switch (e) {
        case InferenceEngine::NPU:
        default:
            return AX_STEREO_ENGINE_NPU;
    }
}

static GdcMeshMode toInternalGdcMeshMode(AX_STEREO_GDC_MESH_MODE_E e) {
    switch (e) {
        case AX_STEREO_GDC_MESH_DYNAMIC_REUSE:
            return GdcMeshMode::DynamicReuse;
        case AX_STEREO_GDC_MESH_DYNAMIC_FORCE:
            return GdcMeshMode::DynamicForce;
        case AX_STEREO_GDC_MESH_DEFAULT:
        default:
            return GdcMeshMode::Default;
    }
}

static InputFrameFormat toInternalInputFormat(AX_STEREO_INPUT_FORMAT_E e) {
    switch (e) {
        case AX_STEREO_INPUT_FORMAT_NV12:
            return InputFrameFormat::Nv12;
        case AX_STEREO_INPUT_FORMAT_YUYV:
        default:
            return InputFrameFormat::Yuyv;
    }
}

AX_S32 AX_STEREO_Create(AX_STEREO_HANDLE* phPipeline, const AX_STEREO_ATTR_T* pstAttr) {
    if (phPipeline == AX_NULL) {
        return -1;
    }

    auto* ctx = new (std::nothrow) AX_STEREO_PIPELINE_CTX();
    if (ctx == AX_NULL) {
        return -1;
    }

    AX_STEREO_ATTR_T attr;
    if (pstAttr != AX_NULL) {
        attr = *pstAttr;
    } else {
        std::memset(&attr, 0, sizeof(attr));
        attr.eEngine = AX_STEREO_ENGINE_NPU;
        attr.bEnableGdc = AX_TRUE;
        attr.eGdcMeshMode = AX_STEREO_GDC_MESH_DYNAMIC_REUSE;
        attr.bExportVoFrames = AX_FALSE;
        attr.s32InputWidth = AX_STEREO_DEFAULT_INPUT_WIDTH;
        attr.s32InputHeight = AX_STEREO_DEFAULT_INPUT_HEIGHT;
    }
    ctx->attr = attr;

    std::string npuModelPath;
    if (attr.szNpuModelPath[0] != '\0') {
        npuModelPath = attr.szNpuModelPath;
    }

    std::string cameraSerialNumber;
    if (attr.szCameraSerialNumber[0] != '\0') {
        cameraSerialNumber = attr.szCameraSerialNumber;
    }

    AX_S32 ret = ctx->pipeline.initialize(
        toInternalEngine(attr.eEngine), attr.bEnableGdc == AX_TRUE,
        toInternalGdcMeshMode(attr.eGdcMeshMode), attr.bExportVoFrames == AX_TRUE, npuModelPath,
        cameraSerialNumber, attr.s32InputWidth, attr.s32InputHeight);

    if (ret != 0) {
        delete ctx;
        return ret;
    }

    *phPipeline = static_cast<AX_STEREO_HANDLE>(ctx);
    return 0;
}

AX_S32 AX_STEREO_Destroy(AX_STEREO_HANDLE hPipeline) {
    if (hPipeline == AX_NULL) {
        return -1;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    ctx->pipeline.shutdown();
    delete ctx;
    return 0;
}

AX_S32 AX_STEREO_GetAttr(AX_STEREO_HANDLE hPipeline, AX_STEREO_ATTR_T* pstAttr) {
    if (hPipeline == AX_NULL || pstAttr == AX_NULL) {
        return -1;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    *pstAttr = ctx->attr;
    return 0;
}

AX_S32 AX_STEREO_SendFrame(AX_STEREO_HANDLE hPipeline, const AX_VOID* pFrameData,
                           AX_U32 u32FrameSize, AX_U64 u64TimestampNs,
                           AX_STEREO_FRAME_CTX* pContext) {
    return AX_STEREO_SendFrameEx(hPipeline, pFrameData, u32FrameSize, AX_STEREO_INPUT_FORMAT_YUYV,
                                 u64TimestampNs, pContext);
}

AX_S32 AX_STEREO_SendFrameEx(AX_STEREO_HANDLE hPipeline, const AX_VOID* pFrameData,
                             AX_U32 u32FrameSize, AX_STEREO_INPUT_FORMAT_E eInputFormat,
                             AX_U64 u64TimestampNs, AX_STEREO_FRAME_CTX* pContext) {
    if (hPipeline == AX_NULL || pContext == AX_NULL) {
        return -1;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    auto* frameCtx = new (std::nothrow) AX_STEREO_FRAME_CTX_INTERNAL();
    if (frameCtx == AX_NULL) {
        return -1;
    }

    AX_S32 ret = ctx->pipeline.preprocessFrame(frameCtx->frameContext, pFrameData,
                                               static_cast<size_t>(u32FrameSize), u64TimestampNs,
                                               toInternalInputFormat(eInputFormat));
    if (ret != 0) {
        delete frameCtx;
        return ret;
    }

    *pContext = static_cast<AX_STEREO_FRAME_CTX>(frameCtx);
    return 0;
}

AX_S32 AX_STEREO_SendVideoFrame(AX_STEREO_HANDLE hPipeline, const AX_VIDEO_FRAME_T* pFrame,
                                AX_STEREO_INPUT_FORMAT_E eInputFormat, AX_U64 u64TimestampNs,
                                AX_STEREO_FRAME_CTX* pContext) {
    return AX_STEREO_SendVideoFrameWithOwner(hPipeline, pFrame, eInputFormat, u64TimestampNs, {},
                                             pContext);
}

AX_S32 AX_STEREO_SendVideoFrameWithOwner(AX_STEREO_HANDLE hPipeline, const AX_VIDEO_FRAME_T* pFrame,
                                         AX_STEREO_INPUT_FORMAT_E eInputFormat,
                                         AX_U64 u64TimestampNs, std::shared_ptr<void> frameOwner,
                                         AX_STEREO_FRAME_CTX* pContext) {
    if (hPipeline == AX_NULL || pFrame == AX_NULL || pContext == AX_NULL) {
        return -1;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    auto* frameCtx = new (std::nothrow) AX_STEREO_FRAME_CTX_INTERNAL();
    if (frameCtx == AX_NULL) {
        return -1;
    }

    AX_S32 ret =
        ctx->pipeline.preprocessFrame(frameCtx->frameContext, pFrame, u64TimestampNs,
                                      toInternalInputFormat(eInputFormat), std::move(frameOwner));
    if (ret != 0) {
        delete frameCtx;
        return ret;
    }

    *pContext = static_cast<AX_STEREO_FRAME_CTX>(frameCtx);
    return 0;
}

AX_S32 AX_STEREO_RunInference(AX_STEREO_HANDLE hPipeline, AX_STEREO_FRAME_CTX context) {
    if (hPipeline == AX_NULL || context == AX_NULL) {
        return -1;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    auto* frameCtx = static_cast<AX_STEREO_FRAME_CTX_INTERNAL*>(context);

    return ctx->pipeline.inferFrame(frameCtx->frameContext);
}

AX_S32 AX_STEREO_GetResult(AX_STEREO_HANDLE hPipeline, AX_STEREO_FRAME_CTX context,
                           AX_STEREO_OUTPUT_T* pOutput) {
    if (hPipeline == AX_NULL || context == AX_NULL || pOutput == AX_NULL) {
        return -1;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    auto* frameCtx = static_cast<AX_STEREO_FRAME_CTX_INTERNAL*>(context);

    auto* outputInternal = new (std::nothrow) AX_STEREO_OUTPUT_INTERNAL();
    if (outputInternal == AX_NULL) {
        return -1;
    }

    AX_S32 ret =
        ctx->pipeline.postprocessFrame(frameCtx->frameContext, outputInternal->pipelineOutput);
    if (ret != 0) {
        delete outputInternal;
        return ret;
    }

    pOutput->u64FrameTimestampNs = outputInternal->pipelineOutput.frameTimestampNs;
    pOutput->u32RawWidth = outputInternal->pipelineOutput.rawWidth;
    pOutput->u32RawHeight = outputInternal->pipelineOutput.rawHeight;
    pOutput->pPrivateData = static_cast<AX_VOID*>(outputInternal);

    return 0;
}

AX_S32 AX_STEREO_ReleaseFrame(AX_STEREO_HANDLE hPipeline, AX_STEREO_FRAME_CTX context) {
    if (context == AX_NULL) {
        return 0;
    }

    auto* frameCtx = static_cast<AX_STEREO_FRAME_CTX_INTERNAL*>(context);

    if (hPipeline != AX_NULL) {
        auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
        ctx->pipeline.releaseFrameContext(frameCtx->frameContext);
    }

    delete frameCtx;
    return 0;
}

AX_S32 AX_STEREO_ReleaseOutput(AX_STEREO_OUTPUT_T* pOutput) {
    if (pOutput == AX_NULL || pOutput->pPrivateData == AX_NULL) {
        return 0;
    }

    auto* outputInternal = static_cast<AX_STEREO_OUTPUT_INTERNAL*>(pOutput->pPrivateData);
    delete outputInternal;
    pOutput->pPrivateData = AX_NULL;
    return 0;
}

AX_F32 AX_STEREO_GetBaselineMeters(AX_STEREO_HANDLE hPipeline) {
    if (hPipeline == AX_NULL) {
        return 0.0f;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    return ctx->pipeline.cameraBaselineMeters();
}

AX_S32 AX_STEREO_SetComputePointCloud(AX_STEREO_HANDLE hPipeline, AX_BOOL bEnable) {
    if (hPipeline == AX_NULL) {
        return -1;
    }
    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    ctx->pipeline.setComputePointCloud(bEnable == AX_TRUE);
    return 0;
}

AX_STEREO_ENGINE_E AX_STEREO_GetEngine(AX_STEREO_HANDLE hPipeline) {
    if (hPipeline == AX_NULL) {
        return AX_STEREO_ENGINE_NPU;
    }

    auto* ctx = static_cast<AX_STEREO_PIPELINE_CTX*>(hPipeline);
    return toApiEngine(ctx->pipeline.inferenceEngine());
}

const AX_CHAR* AX_STEREO_GetDefaultModelPath(AX_VOID) {
    return stereo_depth::defaultNpuModelPath();
}

const AX_CHAR* AX_STEREO_GetEngineName(AX_STEREO_ENGINE_E eEngine) {
    return stereo_depth::inferenceEngineName(toInternalEngine(eEngine));
}

stereo_depth::PipelineOutput* AX_STEREO_OutputGetData(AX_STEREO_OUTPUT_T* pOutput) {
    if (pOutput == nullptr || pOutput->pPrivateData == nullptr) {
        return nullptr;
    }

    auto* outputInternal = static_cast<AX_STEREO_OUTPUT_INTERNAL*>(pOutput->pPrivateData);
    return &outputInternal->pipelineOutput;
}

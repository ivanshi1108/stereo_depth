#include "AxVdecH264Decoder.hpp"

#include <ax_buffer_tool.h>

#define SAMPLE_LOG_TAG "MCAPVDEC"
#include "sample_log.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace stereo_depth::axera_pipeline {
namespace {

constexpr AX_U32 kMaxFrameBuffers = 6;

uint32_t alignUp(uint32_t value, uint32_t alignment) {
    return ((value + alignment - 1) / alignment) * alignment;
}

std::string makeVdecError(const char* what, AX_S32 ret) {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "%s failed: ret=0x%x", what,
                  static_cast<unsigned int>(ret));
    return std::string(buffer);
}

}  // namespace

AxVdecH264Decoder::AxVdecH264Decoder() : m_releaseCtx(std::make_shared<ReleaseContext>()) {}

AxVdecH264Decoder::~AxVdecH264Decoder() { stop(); }

bool AxVdecH264Decoder::beginSequence(int width, int height, std::string& error) {
    if (width <= 0 || height <= 0) {
        error = "invalid H.264 decode geometry";
        return false;
    }

    if (!initModule(error)) {
        return false;
    }

    if (m_group < 0 || m_width != width || m_height != height) {
        if (!destroyGroup(&error)) {
            return false;
        }
        m_width = width;
        m_height = height;
        if (!createGroup(width, height, error) || !startGroup(error)) {
            destroyGroup();
            return false;
        }
        return true;
    }

    return resetGroup(error);
}

bool AxVdecH264Decoder::decodeAccessUnitToFrame(const std::vector<std::byte>& payload, uint64_t pts,
                                                DecodedFrameHandle& decodedFrame,
                                                std::string& error) {
    if (m_group < 0 || !m_groupStarted) {
        error = "H.264 decoder is not ready";
        return false;
    }
    if (payload.empty()) {
        error = "H.264 access unit is empty";
        return false;
    }

    AX_VDEC_STREAM_T stream;
    std::memset(&stream, 0, sizeof(stream));
    stream.u64PTS = pts;
    stream.bEndOfFrame = AX_TRUE;
    stream.pu8Addr = reinterpret_cast<AX_U8*>(const_cast<std::byte*>(payload.data()));
    stream.u32StreamPackLen = static_cast<AX_U32>(payload.size());

    const AX_S32 sendRet = AX_VDEC_SendStream(m_group, &stream, -1);
    if (sendRet != AX_SUCCESS) {
        error = makeVdecError("AX_VDEC_SendStream", sendRet);
        return false;
    }

    return waitDecodedFrame(decodedFrame, error);
}

void AxVdecH264Decoder::stop() {
    std::string error;
    if (destroyGroup(&error)) {
        deinitModule();
    } else if (!error.empty()) {
        ALOGW("%s", error.c_str());
    }
}

bool AxVdecH264Decoder::initModule(std::string& error) {
    if (m_moduleInitialized) {
        return true;
    }

    AX_VDEC_MOD_ATTR_T modAttr;
    std::memset(&modAttr, 0, sizeof(modAttr));
    modAttr.enDecModule = AX_ENABLE_ONLY_VDEC;
    modAttr.u32MaxGroupCount = AX_VDEC_MAX_GRP_NUM;

    const AX_S32 ret = AX_VDEC_Init(&modAttr);
    if (ret != AX_SUCCESS) {
        error = makeVdecError("AX_VDEC_Init", ret);
        return false;
    }

    m_moduleInitialized = true;
    return true;
}

bool AxVdecH264Decoder::createGroup(int width, int height, std::string& error) {
    AX_VDEC_GRP_ATTR_T groupAttr;
    std::memset(&groupAttr, 0, sizeof(groupAttr));
    groupAttr.enCodecType = PT_H264;
    groupAttr.enInputMode = AX_VDEC_INPUT_MODE_FRAME;
    groupAttr.u32MaxPicWidth = alignUp(static_cast<uint32_t>(width), 16);
    groupAttr.u32MaxPicHeight = alignUp(static_cast<uint32_t>(height), 16);
    groupAttr.u32StreamBufSize = groupAttr.u32MaxPicWidth * groupAttr.u32MaxPicHeight * 2;
    groupAttr.bSdkAutoFramePool = AX_TRUE;

    AX_S32 ret = AX_SUCCESS;
    for (AX_VDEC_GRP candidate = 0; candidate < AX_VDEC_MAX_GRP_NUM; ++candidate) {
        ret = AX_VDEC_CreateGrp(candidate, &groupAttr);
        if (ret == AX_SUCCESS) {
            m_group = candidate;
            break;
        }
    }
    if (m_group < 0) {
        error = makeVdecError("AX_VDEC_CreateGrp", ret);
        return false;
    }

    AX_VDEC_GRP_PARAM_T groupParam;
    std::memset(&groupParam, 0, sizeof(groupParam));
    groupParam.stVdecVideoParam.enVdecMode = VIDEO_DEC_MODE_IPB;
    groupParam.stVdecVideoParam.enOutputOrder = AX_VDEC_OUTPUT_ORDER_DEC;
    ret = AX_VDEC_SetGrpParam(m_group, &groupParam);
    if (ret != AX_SUCCESS) {
        error = makeVdecError("AX_VDEC_SetGrpParam", ret);
        return false;
    }

    ret = AX_VDEC_SetDisplayMode(m_group, AX_VDEC_DISPLAY_MODE_PLAYBACK);
    if (ret != AX_SUCCESS) {
        error = makeVdecError("AX_VDEC_SetDisplayMode", ret);
        return false;
    }

    for (AX_VDEC_CHN channel = 0; channel < AX_VDEC_MAX_CHN_NUM; ++channel) {
        if (channel != kOutputChannel) {
            AX_VDEC_DisableChn(m_group, channel);
            continue;
        }

        AX_VDEC_CHN_ATTR_T channelAttr;
        std::memset(&channelAttr, 0, sizeof(channelAttr));
        channelAttr.u32PicWidth = static_cast<AX_U32>(width);
        channelAttr.u32PicHeight = static_cast<AX_U32>(height);
        channelAttr.u32FrameStride = alignUp(static_cast<AX_U32>(width), 256);
        channelAttr.u32OutputFifoDepth = 1;
        channelAttr.enOutputMode = AX_VDEC_OUTPUT_SCALE;
        channelAttr.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        channelAttr.stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        channelAttr.stCompressInfo.u32CompressLevel = 0;
        channelAttr.u32FrameBufCnt = kMaxFrameBuffers;
        channelAttr.u32FrameBufSize = AX_VDEC_GetPicBufferSize(
            channelAttr.u32PicWidth, channelAttr.u32PicHeight, channelAttr.enImgFormat,
            &channelAttr.stCompressInfo, groupAttr.enCodecType);

        ret = AX_VDEC_SetChnAttr(m_group, channel, &channelAttr);
        if (ret != AX_SUCCESS) {
            error = makeVdecError("AX_VDEC_SetChnAttr", ret);
            return false;
        }

        ret = AX_VDEC_EnableChn(m_group, channel);
        if (ret != AX_SUCCESS) {
            error = makeVdecError("AX_VDEC_EnableChn", ret);
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lk(m_releaseCtx->mtx);
        m_releaseCtx->group = m_group;
        m_releaseCtx->channel = kOutputChannel;
        m_releaseCtx->alive = true;
    }

    return true;
}

bool AxVdecH264Decoder::startGroup(std::string& error) {
    AX_VDEC_RECV_PIC_PARAM_T recvParam;
    std::memset(&recvParam, 0, sizeof(recvParam));
    recvParam.s32RecvPicNum = -1;

    const AX_S32 ret = AX_VDEC_StartRecvStream(m_group, &recvParam);
    if (ret != AX_SUCCESS) {
        error = makeVdecError("AX_VDEC_StartRecvStream", ret);
        return false;
    }

    m_groupStarted = true;
    return true;
}

bool AxVdecH264Decoder::resetGroup(std::string& error) {
    if (!waitForOutstandingFrames(error)) {
        return false;
    }

    if (m_groupStarted) {
        AX_VDEC_StopRecvStream(m_group);
        AX_S32 ret = AX_ERR_VDEC_BUSY;
        for (int retry = 0; retry < 5; ++retry) {
            ret = AX_VDEC_ResetGrp(m_group);
            if (ret == AX_SUCCESS) {
                break;
            }
        }
        if (ret != AX_SUCCESS) {
            error = makeVdecError("AX_VDEC_ResetGrp", ret);
            return false;
        }
        m_groupStarted = false;
    }

    return startGroup(error);
}

bool AxVdecH264Decoder::waitForOutstandingFrames(std::string& error) {
    for (int retry = 0; retry < 500; ++retry) {
        if (m_releaseCtx->outstanding.load(std::memory_order_acquire) == 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    error = "timed out waiting for outstanding decoded H.264 frames to be released";
    return false;
}

bool AxVdecH264Decoder::waitDecodedFrame(DecodedFrameHandle& decodedFrame, std::string& error) {
    AX_VIDEO_FRAME_INFO_T frameInfo;
    std::memset(&frameInfo, 0, sizeof(frameInfo));

    AX_S32 ret = AX_ERR_VDEC_QUEUE_EMPTY;
    for (int retry = 0; retry < 200; ++retry) {
        ret = AX_VDEC_GetChnFrame(m_group, kOutputChannel, &frameInfo, 10);
        if (ret == AX_SUCCESS) {
            break;
        }
        if (ret == AX_ERR_VDEC_QUEUE_EMPTY || ret == AX_ERR_VDEC_BUF_EMPTY ||
            ret == AX_ERR_VDEC_TIMED_OUT) {
            continue;
        }
        if (ret == AX_ERR_VDEC_FLOW_END) {
            error = "vdec flow ended while waiting decoded frame";
            return false;
        }
        error = makeVdecError("AX_VDEC_GetChnFrame", ret);
        return false;
    }
    if (ret != AX_SUCCESS) {
        error = "timed out waiting for decoded H.264 frame";
        return false;
    }

    return retainDecodedFrame(frameInfo, decodedFrame, error);
}

bool AxVdecH264Decoder::retainDecodedFrame(const AX_VIDEO_FRAME_INFO_T& frameInfo,
                                           DecodedFrameHandle& decodedFrame, std::string& error) {
    const AX_VIDEO_FRAME_T& frame = frameInfo.stVFrame;
    if (frame.enImgFormat != AX_FORMAT_YUV420_SEMIPLANAR) {
        releaseDecodedFrame(frameInfo);
        error = "decoded H.264 frame is not NV12";
        return false;
    }

    m_releaseCtx->outstanding.fetch_add(1, std::memory_order_acq_rel);

    auto* frameHolder = new (std::nothrow) DecodedFrame();
    if (frameHolder == nullptr) {
        m_releaseCtx->outstanding.fetch_sub(1, std::memory_order_acq_rel);
        releaseDecodedFrame(frameInfo);
        error = "failed to allocate decoded H.264 frame holder";
        return false;
    }

    frameHolder->frameInfo = frameInfo;

    auto ctx = m_releaseCtx;
    decodedFrame = DecodedFrameHandle(frameHolder, [ctx](DecodedFrame* frame) {
        if (frame == nullptr) {
            return;
        }
        {
            std::lock_guard<std::mutex> lk(ctx->mtx);
            if (ctx->alive && ctx->group >= 0) {
                const AX_S32 releaseRet =
                    AX_VDEC_ReleaseChnFrame(ctx->group, ctx->channel, &frame->frameInfo);
                if (releaseRet != AX_SUCCESS) {
                    ALOGW(
                        "AX_VDEC_ReleaseChnFrame failed while releasing retained frame: "
                        "ret=0x%x",
                        static_cast<unsigned int>(releaseRet));
                }
            }
        }
        delete frame;
        ctx->outstanding.fetch_sub(1, std::memory_order_acq_rel);
    });
    return true;
}

void AxVdecH264Decoder::releaseDecodedFrame(const AX_VIDEO_FRAME_INFO_T& frameInfo) const {
    const AX_S32 ret = AX_VDEC_ReleaseChnFrame(m_group, kOutputChannel, &frameInfo);
    if (ret != AX_SUCCESS) {
        ALOGW("AX_VDEC_ReleaseChnFrame failed while releasing retained frame: ret=0x%x",
              static_cast<unsigned int>(ret));
    }
}

bool AxVdecH264Decoder::destroyGroup(std::string* error) {
    if (m_group < 0) {
        return true;
    }

    std::string waitError;
    const bool drained = waitForOutstandingFrames(waitError);

    // Detach any still-outstanding DecodedFrame deleters from this group before we
    // destroy it. After this point, late deleter callbacks will observe alive=false
    // and skip the AX_VDEC_ReleaseChnFrame call entirely.
    {
        std::lock_guard<std::mutex> lk(m_releaseCtx->mtx);
        m_releaseCtx->alive = false;
        m_releaseCtx->group = -1;
    }

    if (m_groupStarted) {
        AX_VDEC_StopRecvStream(m_group);
        AX_VDEC_ResetGrp(m_group);
        m_groupStarted = false;
    }

    for (AX_VDEC_CHN channel = 0; channel < AX_VDEC_MAX_CHN_NUM; ++channel) {
        AX_VDEC_DisableChn(m_group, channel);
    }
    AX_VDEC_DestroyGrp(m_group);
    m_group = -1;
    m_width = 0;
    m_height = 0;

    if (!drained) {
        if (error != nullptr) {
            *error = waitError;
        }
        return false;
    }
    return true;
}

void AxVdecH264Decoder::deinitModule() {
    if (!m_moduleInitialized) {
        return;
    }
    AX_VDEC_Deinit();
    m_moduleInitialized = false;
}

}  // namespace stereo_depth::axera_pipeline

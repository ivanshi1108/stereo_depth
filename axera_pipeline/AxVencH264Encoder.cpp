#include "AxVencH264Encoder.hpp"

#include "ax_sys_api.h"
#include "ax_venc_api.h"
#include "ax_venc_comm.h"
#include "ax_venc_rc.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <sstream>

namespace stereo_depth::axera_pipeline {
namespace {

constexpr VENC_CHN kStereoDepthH264VencChannel = 0;

void appendPacketRange(std::vector<std::byte>& dst, const AX_U8* src, AX_U32 totalLen,
                       AX_U32 offset, AX_U32 length) {
    if (src == nullptr || offset >= totalLen || length == 0) {
        return;
    }

    const AX_U32 available = totalLen - offset;
    const AX_U32 copyLen = std::min(length, available);
    const auto* begin = reinterpret_cast<const std::byte*>(src + offset);
    dst.insert(dst.end(), begin, begin + copyLen);
}

}  // namespace

AxVencH264Encoder::~AxVencH264Encoder() { stop(); }

bool AxVencH264Encoder::start(uint32_t width, uint32_t height, uint32_t fps,
                              const H264EncoderLogFn& logFn) {
    if (m_active) {
        return true;
    }

    if (width == 0 || height == 0) {
        if (logFn) {
            logFn("h264 recording failed: invalid input geometry");
        }
        return false;
    }

    m_width = width;
    m_height = height;
    m_fps = std::max<uint32_t>(1, fps);
    m_frameSize = static_cast<size_t>(m_width) * static_cast<size_t>(m_height) * 2;

    AX_VENC_MOD_ATTR_T stModAttr = {};
    stModAttr.enVencType = AX_VENC_VIDEO_ENCODER;
    stModAttr.stModThdAttr.u32TotalThreadNum = 1;
    stModAttr.stModThdAttr.bExplicitSched = AX_FALSE;

    AX_S32 ret = AX_VENC_Init(&stModAttr);
    if (ret != AX_SUCCESS) {
        logApiFailure(logFn, "AX_VENC_Init", ret);
        stop(logFn);
        return false;
    }
    m_vencInitialized = true;

    AX_POOL_CONFIG_T stPoolConfig = {};
    stPoolConfig.MetaSize = 512;
    stPoolConfig.BlkSize = static_cast<AX_U64>(m_frameSize);
    stPoolConfig.BlkCnt = 1;
    stPoolConfig.CacheMode = POOL_CACHE_MODE_CACHED;
    std::strncpy(reinterpret_cast<char*>(stPoolConfig.PartitionName), "anonymous",
                 sizeof(stPoolConfig.PartitionName) - 1);
    std::strncpy(reinterpret_cast<char*>(stPoolConfig.PoolName), "stereo_h264",
                 sizeof(stPoolConfig.PoolName) - 1);

    m_framePoolId = AX_POOL_CreatePool(&stPoolConfig);
    if (m_framePoolId == AX_INVALID_POOLID) {
        if (logFn) {
            logFn("h264 recording failed: AX_POOL_CreatePool returned AX_INVALID_POOLID");
        }
        stop(logFn);
        return false;
    }

    m_frameBlkId = AX_POOL_GetBlock(m_framePoolId, stPoolConfig.BlkSize, nullptr);
    if (m_frameBlkId == AX_INVALID_BLOCKID) {
        if (logFn) {
            logFn("h264 recording failed: AX_POOL_GetBlock returned AX_INVALID_BLOCKID");
        }
        stop(logFn);
        return false;
    }

    m_framePhyAddr = AX_POOL_Handle2PhysAddr(m_frameBlkId);
    m_frameVirAddr = AX_POOL_GetBlockVirAddr(m_frameBlkId);
    if (m_framePhyAddr == 0 || m_frameVirAddr == nullptr) {
        if (logFn) {
            logFn("h264 recording failed: AX_POOL block address lookup failed");
        }
        stop(logFn);
        return false;
    }

    AX_VENC_CHN_ATTR_T stChnAttr = {};
    stChnAttr.stVencAttr.enType = PT_H264;
    stChnAttr.stVencAttr.u32MaxPicWidth = m_width;
    stChnAttr.stVencAttr.u32MaxPicHeight = m_height;
    stChnAttr.stVencAttr.enMemSource = AX_MEMORY_SOURCE_POOL;
    stChnAttr.stVencAttr.u32BufSize = static_cast<AX_U32>(m_frameSize);
    stChnAttr.stVencAttr.enProfile = AX_VENC_H264_MAIN_PROFILE;
    stChnAttr.stVencAttr.enLevel = AX_VENC_H264_LEVEL_5_1;
    stChnAttr.stVencAttr.u32PicWidthSrc = m_width;
    stChnAttr.stVencAttr.u32PicHeightSrc = m_height;
    stChnAttr.stVencAttr.enLinkMode = AX_VENC_UNLINK_MODE;
    stChnAttr.stVencAttr.u8InFifoDepth = 1;
    stChnAttr.stVencAttr.u8OutFifoDepth = 1;

    stChnAttr.stRcAttr.enRcMode = AX_VENC_RC_MODE_H264CBR;
    stChnAttr.stRcAttr.s32FirstFrameStartQp = -1;
    stChnAttr.stRcAttr.stFrameRate.fSrcFrameRate = static_cast<AX_F32>(m_fps);
    stChnAttr.stRcAttr.stFrameRate.fDstFrameRate = static_cast<AX_F32>(m_fps);

    AX_VENC_H264_CBR_T stH264Cbr = {};
    stH264Cbr.u32Gop = m_fps;
    stH264Cbr.u32StatTime = 1;
    stH264Cbr.u32BitRate = estimateBitrateKbps(m_width, m_height, m_fps);
    stH264Cbr.u32MinQp = 22;
    stH264Cbr.u32MaxQp = 51;
    stH264Cbr.u32MinIQp = 20;
    stH264Cbr.u32MaxIQp = 51;
    stH264Cbr.u32MaxIprop = 60;
    stH264Cbr.u32MinIprop = 10;
    stH264Cbr.s32IntraQpDelta = -2;
    stH264Cbr.u32IdrQpDeltaRange = 5;
    stChnAttr.stRcAttr.stH264Cbr = stH264Cbr;
    stChnAttr.stGopAttr.enGopMode = AX_VENC_GOPMODE_NORMALP;

    ret = AX_VENC_CreateChn(kStereoDepthH264VencChannel, &stChnAttr);
    if (ret != AX_SUCCESS) {
        logApiFailure(logFn, "AX_VENC_CreateChn", ret);
        stop(logFn);
        return false;
    }
    m_channelCreated = true;

    AX_VENC_RECV_PIC_PARAM_T stRecvParam = {};
    stRecvParam.s32RecvPicNum = -1;
    ret = AX_VENC_StartRecvFrame(kStereoDepthH264VencChannel, &stRecvParam);
    if (ret != AX_SUCCESS) {
        logApiFailure(logFn, "AX_VENC_StartRecvFrame", ret);
        stop(logFn);
        return false;
    }
    m_recvStarted = true;

    ret = AX_VENC_RequestIDR(kStereoDepthH264VencChannel, AX_TRUE);
    if (ret != AX_SUCCESS && logFn) {
        std::ostringstream oss;
        oss << "h264 recording warning: AX_VENC_RequestIDR ret=0x" << std::hex << ret;
        logFn(oss.str());
    }

    m_framesEncoded = 0;
    m_cachedParameterSets.clear();
    m_active = true;
    return true;
}

bool AxVencH264Encoder::encodeFrame(const std::vector<std::byte>& rawYuyvData, uint32_t rawWidth,
                                    uint32_t rawHeight, uint64_t frameTimestampNs,
                                    EncodedH264Frame& encodedFrame, const H264EncoderLogFn& logFn) {
    encodedFrame = {};
    if (!m_active) {
        return false;
    }
    if (rawWidth != m_width || rawHeight != m_height) {
        if (logFn) {
            std::ostringstream oss;
            oss << "h264 recording skipped: geometry changed to " << rawWidth << 'x' << rawHeight
                << ", expected " << m_width << 'x' << m_height;
            logFn(oss.str());
        }
        return false;
    }
    if (rawYuyvData.size() < m_frameSize || m_frameVirAddr == nullptr) {
        if (logFn) {
            logFn("h264 recording skipped: invalid raw YUYV buffer");
        }
        return false;
    }

    std::memcpy(m_frameVirAddr, rawYuyvData.data(), m_frameSize);
    AX_S32 ret = AX_SYS_MflushCache(static_cast<AX_U64>(m_framePhyAddr), m_frameVirAddr,
                                    static_cast<AX_U32>(m_frameSize));
    if (ret != AX_SUCCESS) {
        logApiFailure(logFn, "AX_SYS_MflushCache", ret);
        return false;
    }

    AX_VIDEO_FRAME_INFO_T stFrame = {};
    stFrame.stVFrame.u32FrameSize = static_cast<AX_U32>(m_frameSize);
    stFrame.stVFrame.u64PhyAddr[0] = static_cast<AX_U64>(m_framePhyAddr);
    stFrame.stVFrame.u64VirAddr[0] = reinterpret_cast<AX_U64>(m_frameVirAddr);
    stFrame.stVFrame.u32BlkId[0] = m_frameBlkId;
    stFrame.stVFrame.u32BlkId[1] = 0;
    stFrame.stVFrame.u32BlkId[2] = 0;
    stFrame.stVFrame.u64PhyAddr[1] = 0;
    stFrame.stVFrame.u64PhyAddr[2] = 0;
    stFrame.stVFrame.u64VirAddr[1] = 0;
    stFrame.stVFrame.u64VirAddr[2] = 0;
    stFrame.stVFrame.enImgFormat = AX_FORMAT_YUV422_INTERLEAVED_YUYV;
    stFrame.stVFrame.u32Width = m_width;
    stFrame.stVFrame.u32Height = m_height;
    stFrame.stVFrame.u32PicStride[0] = m_width * 2;
    stFrame.stVFrame.u64PTS =
        frameTimestampNs != 0 ? frameTimestampNs / 1000ULL : (m_framesEncoded + 1);
    stFrame.stVFrame.u64SeqNum = m_framesEncoded + 1;

    ret = AX_VENC_SendFrame(kStereoDepthH264VencChannel, &stFrame, 2000);
    if (ret != AX_SUCCESS) {
        logApiFailure(logFn, "AX_VENC_SendFrame", ret);
        return false;
    }

    AX_VENC_STREAM_T stStream = {};
    ret = AX_VENC_GetStream(kStereoDepthH264VencChannel, &stStream, 2000);
    if (ret != AX_SUCCESS) {
        logApiFailure(logFn, "AX_VENC_GetStream", ret);
        return false;
    }

    const bool buildOk =
        buildCompressedVideoFrame(&stStream, frameTimestampNs, encodedFrame, logFn);
    const AX_S32 releaseRet = AX_VENC_ReleaseStream(kStereoDepthH264VencChannel, &stStream);
    if (releaseRet != AX_SUCCESS) {
        logApiFailure(logFn, "AX_VENC_ReleaseStream", releaseRet);
        return false;
    }
    if (!buildOk) {
        return false;
    }

    ++m_framesEncoded;
    return true;
}

void AxVencH264Encoder::stop(const H264EncoderLogFn& logFn) {
    if (m_recvStarted) {
        const AX_S32 ret = AX_VENC_StopRecvFrame(kStereoDepthH264VencChannel);
        if (ret != AX_SUCCESS && logFn) {
            logApiFailure(logFn, "AX_VENC_StopRecvFrame", ret);
        }
        m_recvStarted = false;
    }

    if (m_channelCreated) {
        const AX_S32 ret = AX_VENC_DestroyChn(kStereoDepthH264VencChannel);
        if (ret != AX_SUCCESS && logFn) {
            logApiFailure(logFn, "AX_VENC_DestroyChn", ret);
        }
        m_channelCreated = false;
    }

    if (m_frameBlkId != AX_INVALID_BLOCKID) {
        const AX_S32 ret = AX_POOL_ReleaseBlock(m_frameBlkId);
        if (ret != AX_SUCCESS && logFn) {
            logApiFailure(logFn, "AX_POOL_ReleaseBlock", ret);
        }
        m_frameBlkId = AX_INVALID_BLOCKID;
    }

    if (m_framePoolId != AX_INVALID_POOLID) {
        const AX_S32 ret = AX_POOL_DestroyPool(m_framePoolId);
        if (ret != AX_SUCCESS && logFn) {
            logApiFailure(logFn, "AX_POOL_DestroyPool", ret);
        }
        m_framePoolId = AX_INVALID_POOLID;
    }

    if (m_frameVirAddr != nullptr || m_framePhyAddr != 0) {
        m_framePhyAddr = 0;
        m_frameVirAddr = nullptr;
    }

    if (m_vencInitialized) {
        const AX_S32 ret = AX_VENC_Deinit();
        if (ret != AX_SUCCESS && logFn) {
            logApiFailure(logFn, "AX_VENC_Deinit", ret);
        }
        m_vencInitialized = false;
    }

    m_active = false;
    m_width = 0;
    m_height = 0;
    m_fps = 0;
    m_frameSize = 0;
    m_framesEncoded = 0;
    m_cachedParameterSets.clear();
}

uint32_t AxVencH264Encoder::estimateBitrateKbps(uint32_t width, uint32_t height, uint32_t fps) {
    const uint64_t pixelsPerSecond = static_cast<uint64_t>(width) * static_cast<uint64_t>(height) *
                                     static_cast<uint64_t>(std::max<uint32_t>(1, fps));
    const uint32_t estimate = static_cast<uint32_t>(pixelsPerSecond / 3000ULL);
    return std::clamp<uint32_t>(estimate, 4000, 40000);
}

void AxVencH264Encoder::logApiFailure(const H264EncoderLogFn& logFn, const char* api, int ret) {
    if (!logFn) {
        return;
    }

    std::ostringstream oss;
    oss << "h264 recording failed: " << api << " ret=0x" << std::hex << ret;
    logFn(oss.str());
}

uint64_t AxVencH264Encoder::currentWallClockNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

size_t AxVencH264Encoder::findStartCode(const std::vector<std::byte>& data, size_t pos,
                                        size_t* codeSize) {
    for (size_t i = pos; i + 3 < data.size(); ++i) {
        const auto b0 = std::to_integer<unsigned char>(data[i]);
        const auto b1 = std::to_integer<unsigned char>(data[i + 1]);
        const auto b2 = std::to_integer<unsigned char>(data[i + 2]);
        if (b0 == 0 && b1 == 0 && b2 == 1) {
            if (codeSize != nullptr) {
                *codeSize = 3;
            }
            return i;
        }
        if (i + 4 < data.size()) {
            const auto b3 = std::to_integer<unsigned char>(data[i + 3]);
            if (b0 == 0 && b1 == 0 && b2 == 0 && b3 == 1) {
                if (codeSize != nullptr) {
                    *codeSize = 4;
                }
                return i;
            }
        }
    }
    return data.size();
}

AxVencH264Encoder::NaluSummary AxVencH264Encoder::summarizePacketByScan(
    const std::vector<std::byte>& packet) {
    NaluSummary summary;
    size_t searchPos = 0;
    while (searchPos + 4 <= packet.size()) {
        size_t startCodeSize = 0;
        const size_t start = findStartCode(packet, searchPos, &startCodeSize);
        if (start == packet.size()) {
            break;
        }

        const size_t nalHeaderPos = start + startCodeSize;
        if (nalHeaderPos >= packet.size()) {
            break;
        }

        size_t nextCodeSize = 0;
        const size_t nextStart = findStartCode(packet, nalHeaderPos + 1, &nextCodeSize);
        const size_t nalEnd = nextStart == packet.size() ? packet.size() : nextStart;
        const unsigned char nalType = std::to_integer<unsigned char>(packet[nalHeaderPos]) & 0x1fU;

        if (nalType == AX_H264E_NALU_SPS || nalType == AX_H264E_NALU_PPS) {
            if (nalType == AX_H264E_NALU_SPS) {
                summary.hasSps = true;
            }
            if (nalType == AX_H264E_NALU_PPS) {
                summary.hasPps = true;
            }
            summary.parameterSets.insert(summary.parameterSets.end(), packet.begin() + start,
                                         packet.begin() + nalEnd);
        }
        if (nalType == AX_H264E_NALU_IDRSLICE) {
            summary.hasIdr = true;
        }

        searchPos = nalEnd;
    }
    return summary;
}

AxVencH264Encoder::NaluSummary AxVencH264Encoder::summarizePacketByNaluInfo(
    const void* packAddr, uint32_t packLen, const void* naluInfo, uint32_t naluCount) const {
    NaluSummary summary;
    if (packAddr == nullptr || packLen == 0 || naluInfo == nullptr || naluCount == 0) {
        return summary;
    }

    const auto* addr = reinterpret_cast<const AX_U8*>(packAddr);
    const auto* info = reinterpret_cast<const AX_VENC_NALU_INFO_T*>(naluInfo);
    for (uint32_t i = 0; i < naluCount; ++i) {
        const AX_H264E_NALU_TYPE_E nalType = info[i].unNaluType.enH264EType;
        if (nalType == AX_H264E_NALU_SPS || nalType == AX_H264E_NALU_PPS) {
            if (nalType == AX_H264E_NALU_SPS) {
                summary.hasSps = true;
            }
            if (nalType == AX_H264E_NALU_PPS) {
                summary.hasPps = true;
            }
            appendPacketRange(summary.parameterSets, addr, packLen, info[i].u32NaluOffset,
                              info[i].u32NaluLength);
        }
        if (nalType == AX_H264E_NALU_IDRSLICE) {
            summary.hasIdr = true;
        }
    }

    return summary;
}

bool AxVencH264Encoder::buildCompressedVideoFrame(const void* streamPtr, uint64_t frameTimestampNs,
                                                  EncodedH264Frame& encodedFrame,
                                                  const H264EncoderLogFn& logFn) {
    const auto& stream = *reinterpret_cast<const AX_VENC_STREAM_T*>(streamPtr);
    if (stream.stPack.pu8Addr == nullptr || stream.stPack.u32Len == 0) {
        if (logFn) {
            logFn("h264 recording failed: empty encoded stream");
        }
        return false;
    }
    if (stream.stPack.enCodingType == AX_VENC_BIDIR_PREDICTED_FRAME) {
        if (logFn) {
            logFn("h264 recording skipped: B-frames are not supported by Foxglove CompressedVideo");
        }
        return false;
    }

    std::vector<std::byte> packet(stream.stPack.u32Len);
    std::memcpy(packet.data(), stream.stPack.pu8Addr, stream.stPack.u32Len);

    NaluSummary summary =
        stream.stPack.u32NaluNum > 0
            ? summarizePacketByNaluInfo(stream.stPack.pu8Addr, stream.stPack.u32Len,
                                        stream.stPack.stNaluInfo, stream.stPack.u32NaluNum)
            : summarizePacketByScan(packet);
    if (!summary.parameterSets.empty()) {
        m_cachedParameterSets = summary.parameterSets;
    }

    const bool isKeyFrame = stream.stPack.enCodingType == AX_VENC_INTRA_FRAME || summary.hasIdr;
    if (isKeyFrame && !summary.hasSps && !m_cachedParameterSets.empty()) {
        encodedFrame.payload.reserve(m_cachedParameterSets.size() + packet.size());
        encodedFrame.payload.insert(encodedFrame.payload.end(), m_cachedParameterSets.begin(),
                                    m_cachedParameterSets.end());
        encodedFrame.payload.insert(encodedFrame.payload.end(), packet.begin(), packet.end());
    } else {
        encodedFrame.payload = std::move(packet);
    }

    encodedFrame.timestampNs = frameTimestampNs != 0 ? frameTimestampNs : currentWallClockNs();
    encodedFrame.keyFrame = isKeyFrame;
    return !encodedFrame.payload.empty();
}

}  // namespace stereo_depth::axera_pipeline
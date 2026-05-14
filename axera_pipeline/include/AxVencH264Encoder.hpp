#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace stereo_depth::axera_pipeline {

using H264EncoderLogFn = std::function<void(const std::string&)>;

struct EncodedH264Frame {
    uint64_t timestampNs = 0;
    bool keyFrame = false;
    std::vector<std::byte> payload;
};

class AxVencH264Encoder {
public:
    AxVencH264Encoder() = default;
    ~AxVencH264Encoder();

    bool start(uint32_t width, uint32_t height, uint32_t fps, const H264EncoderLogFn& logFn);
    bool encodeFrame(const std::vector<std::byte>& rawYuyvData, uint32_t rawWidth,
                     uint32_t rawHeight, uint64_t frameTimestampNs, EncodedH264Frame& encodedFrame,
                     const H264EncoderLogFn& logFn);
    void stop(const H264EncoderLogFn& logFn = {});

    bool isActive() const { return m_active; }

private:
    struct NaluSummary {
        bool hasSps = false;
        bool hasPps = false;
        bool hasIdr = false;
        std::vector<std::byte> parameterSets;
    };

    static uint32_t estimateBitrateKbps(uint32_t width, uint32_t height, uint32_t fps);
    static void logApiFailure(const H264EncoderLogFn& logFn, const char* api, int ret);
    static uint64_t currentWallClockNs();
    static size_t findStartCode(const std::vector<std::byte>& data, size_t pos, size_t* codeSize);
    static NaluSummary summarizePacketByScan(const std::vector<std::byte>& packet);
    NaluSummary summarizePacketByNaluInfo(const void* packAddr, uint32_t packLen,
                                          const void* naluInfo, uint32_t naluCount) const;
    bool buildCompressedVideoFrame(const void* streamPtr, uint64_t frameTimestampNs,
                                   EncodedH264Frame& encodedFrame, const H264EncoderLogFn& logFn);

    bool m_active = false;
    bool m_vencInitialized = false;
    bool m_channelCreated = false;
    bool m_recvStarted = false;
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint32_t m_fps = 0;
    size_t m_frameSize = 0;
    uint64_t m_framesEncoded = 0;
    uint32_t m_framePoolId = 0xffffffffU;
    uint32_t m_frameBlkId = 0;
    unsigned long long m_framePhyAddr = 0;
    void* m_frameVirAddr = nullptr;
    std::vector<std::byte> m_cachedParameterSets;
};

}  // namespace stereo_depth::axera_pipeline
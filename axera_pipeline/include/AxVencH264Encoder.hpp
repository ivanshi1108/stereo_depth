#pragma once

// Host/AXCL port: the public encoder interface is unchanged, but the
// implementation now delegates to a pluggable H.264 backend (host FFmpeg or the
// AXCL card). See host_backend/H264Codec.hpp.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace stereo_depth::host_backend {
class IH264Encoder;
}

namespace stereo_depth::axera_pipeline {

using H264EncoderLogFn = std::function<void(const std::string&)>;

struct EncodedH264Frame {
    uint64_t timestampNs = 0;
    bool keyFrame = false;
    std::vector<std::byte> payload;
};

class AxVencH264Encoder {
public:
    AxVencH264Encoder();
    ~AxVencH264Encoder();

    bool start(uint32_t width, uint32_t height, uint32_t fps, const H264EncoderLogFn& logFn);
    bool encodeFrame(const std::vector<std::byte>& rawYuyvData, uint32_t rawWidth,
                     uint32_t rawHeight, uint64_t frameTimestampNs, EncodedH264Frame& encodedFrame,
                     const H264EncoderLogFn& logFn);
    void stop(const H264EncoderLogFn& logFn = {});

    bool isActive() const { return m_active; }

private:
    static uint64_t currentWallClockNs();

    bool m_active = false;
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint32_t m_fps = 0;
    uint64_t m_framesEncoded = 0;
    std::unique_ptr<stereo_depth::host_backend::IH264Encoder> m_backend;
};

}  // namespace stereo_depth::axera_pipeline
// Host H.264 encoder facade. Uses the host FFmpeg (libavcodec) backend.

#include "AxVencH264Encoder.hpp"

#include <chrono>
#include <cstring>

#include "H264Codec.hpp"

namespace stereo_depth::axera_pipeline {

using stereo_depth::host_backend::IH264Encoder;

uint64_t AxVencH264Encoder::currentWallClockNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

AxVencH264Encoder::AxVencH264Encoder() = default;

AxVencH264Encoder::~AxVencH264Encoder() { stop(); }

bool AxVencH264Encoder::start(uint32_t width, uint32_t height, uint32_t fps,
                              const H264EncoderLogFn& logFn) {
    if (m_active) {
        return true;
    }

    std::unique_ptr<IH264Encoder> backend = host_backend::createHostH264Encoder();
    if (!backend || !backend->start(width, height, fps)) {
        if (logFn) {
            logFn("H.264 encoder unavailable; continuing without H.264 recording");
        }
        return false;
    }
    m_backend = std::move(backend);

    m_active = true;
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_framesEncoded = 0;
    if (logFn) {
        logFn(std::string("H.264 encoder backend: ") + m_backend->backendName());
    }
    return true;
}

bool AxVencH264Encoder::encodeFrame(const std::vector<std::byte>& rawYuyvData, uint32_t rawWidth,
                                    uint32_t rawHeight, uint64_t frameTimestampNs,
                                    EncodedH264Frame& encodedFrame, const H264EncoderLogFn& logFn) {
    encodedFrame.payload.clear();
    encodedFrame.keyFrame = false;
    encodedFrame.timestampNs = frameTimestampNs != 0 ? frameTimestampNs : currentWallClockNs();

    if (!m_active || !m_backend) {
        return false;
    }

    std::vector<uint8_t> annexB;
    bool keyFrame = false;
    if (!m_backend->encode(reinterpret_cast<const uint8_t*>(rawYuyvData.data()), rawYuyvData.size(),
                           rawWidth, rawHeight, encodedFrame.timestampNs, annexB, keyFrame)) {
        if (logFn) {
            logFn("H.264 encode failed for one frame");
        }
        return false;
    }

    encodedFrame.keyFrame = keyFrame;
    encodedFrame.payload.resize(annexB.size());
    if (!annexB.empty()) {
        std::memcpy(encodedFrame.payload.data(), annexB.data(), annexB.size());
    }
    ++m_framesEncoded;
    return true;
}

void AxVencH264Encoder::stop(const H264EncoderLogFn& logFn) {
    if (m_backend) {
        m_backend->stop();
        m_backend.reset();
    }
    if (m_active && logFn) {
        logFn("H.264 encoder stopped");
    }
    m_active = false;
}

}  // namespace stereo_depth::axera_pipeline

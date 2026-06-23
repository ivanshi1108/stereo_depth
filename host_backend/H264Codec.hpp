#pragma once

// Host H.264 codec abstraction (FFmpeg / libavcodec). Works on amd64 and on
// Raspberry Pi / Orange Pi (V4L2 M2M when available, else software).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace stereo_depth::host_backend {

struct DecodedNv12 {
    int width = 0;
    int height = 0;
    int stride = 0;  // equals width (contiguous NV12)
    uint64_t pts = 0;
    std::vector<uint8_t> data;  // Y plane (stride*h) followed by interleaved UV
};

class IH264Encoder {
public:
    virtual ~IH264Encoder() = default;
    virtual bool start(uint32_t width, uint32_t height, uint32_t fps) = 0;
    // Encode one packed YUYV (YUYV422) frame. Produces an Annex-B H.264 access
    // unit (may be empty if the encoder is buffering).
    virtual bool encode(const uint8_t* yuyv, size_t size, uint32_t width, uint32_t height,
                        uint64_t ptsNs, std::vector<uint8_t>& outAnnexB, bool& keyFrame) = 0;
    virtual void stop() = 0;
    virtual const char* backendName() const = 0;
};

class IH264Decoder {
public:
    virtual ~IH264Decoder() = default;
    virtual bool begin(int width, int height) = 0;
    // Decode one Annex-B H.264 access unit to an NV12 frame. Returns true and
    // fills `out` when a frame is produced.
    virtual bool decode(const uint8_t* annexB, size_t size, uint64_t pts, DecodedNv12& out) = 0;
    virtual void stop() = 0;
    virtual const char* backendName() const = 0;
};

// Host H.264 codec backend constructors (return nullptr if unavailable).
std::unique_ptr<IH264Encoder> createHostH264Encoder();
std::unique_ptr<IH264Decoder> createHostH264Decoder();

}  // namespace stereo_depth::host_backend

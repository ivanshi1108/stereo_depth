#pragma once

// Host/AXCL port: the public decoder interface is unchanged; the implementation
// delegates to a pluggable H.264 backend (host FFmpeg or the AXCL card). The
// decoded NV12 frame is described by an AX_VIDEO_FRAME_INFO_T whose planes point
// into the host-owned `nv12` buffer kept alive by the DecodedFrame.

#include <ax_global_type.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace stereo_depth::host_backend {
class IH264Decoder;
}

namespace stereo_depth::axera_pipeline {

class AxVdecH264Decoder {
public:
    struct DecodedFrame {
        AX_VIDEO_FRAME_INFO_T frameInfo = {};
        std::vector<uint8_t> nv12;  // host-owned backing store for frameInfo planes
    };
    using DecodedFrameHandle = std::shared_ptr<DecodedFrame>;

    AxVdecH264Decoder();
    ~AxVdecH264Decoder();

    bool beginSequence(int width, int height, std::string& error);
    bool decodeAccessUnitToFrame(const std::vector<std::byte>& payload, uint64_t pts,
                                 DecodedFrameHandle& decodedFrame, std::string& error);
    void stop();

    // True once a decoder backend has been created and started. Used by the
    // replay loop to avoid recreating the decoder while skipping leading
    // non-IDR access units.
    bool isStarted() const { return m_started; }

private:
    int m_width = 0;
    int m_height = 0;
    bool m_started = false;
    std::unique_ptr<stereo_depth::host_backend::IH264Decoder> m_backend;
};

}  // namespace stereo_depth::axera_pipeline

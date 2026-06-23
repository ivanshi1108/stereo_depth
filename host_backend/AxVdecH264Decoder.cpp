// Host H.264 decoder facade. Decodes H.264 access units to NV12 frames backed
// by host memory using the host FFmpeg (libavcodec) backend.

#include "AxVdecH264Decoder.hpp"

#include <cstring>

#include "H264Codec.hpp"

#define SAMPLE_LOG_TAG "VDEC"
#include "sample_log.h"

namespace stereo_depth::axera_pipeline {

using stereo_depth::host_backend::DecodedNv12;
using stereo_depth::host_backend::IH264Decoder;

AxVdecH264Decoder::AxVdecH264Decoder() = default;

AxVdecH264Decoder::~AxVdecH264Decoder() { stop(); }

bool AxVdecH264Decoder::beginSequence(int width, int height, std::string& error) {
    m_width = width;
    m_height = height;

    std::unique_ptr<IH264Decoder> backend = host_backend::createHostH264Decoder();
    if (!backend || !backend->begin(width, height)) {
        error = "no H.264 decoder backend available";
        return false;
    }
    m_backend = std::move(backend);

    m_started = true;
    ALOGN("H.264 decoder backend: %s", m_backend->backendName());
    return true;
}

bool AxVdecH264Decoder::decodeAccessUnitToFrame(const std::vector<std::byte>& payload, uint64_t pts,
                                                DecodedFrameHandle& decodedFrame,
                                                std::string& error) {
    if (!m_started || !m_backend) {
        error = "decoder not started";
        return false;
    }

    DecodedNv12 nv12;
    if (!m_backend->decode(reinterpret_cast<const uint8_t*>(payload.data()), payload.size(), pts,
                           nv12)) {
        error = "decode failed or needs more data";
        return false;
    }

    auto frame = std::make_shared<DecodedFrame>();
    frame->nv12 = std::move(nv12.data);

    AX_VIDEO_FRAME_T& vf = frame->frameInfo.stVFrame;
    const uint64_t base = reinterpret_cast<uint64_t>(frame->nv12.data());
    const uint64_t uvOffset = static_cast<uint64_t>(nv12.stride) * nv12.height;
    vf.u32Width = static_cast<AX_U32>(nv12.width);
    vf.u32Height = static_cast<AX_U32>(nv12.height);
    vf.u32PicStride[0] = static_cast<AX_U32>(nv12.stride);
    vf.u32PicStride[1] = static_cast<AX_U32>(nv12.stride);
    vf.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    vf.u64VirAddr[0] = base;
    vf.u64VirAddr[1] = base + uvOffset;
    vf.u64PhyAddr[0] = vf.u64VirAddr[0];
    vf.u64PhyAddr[1] = vf.u64VirAddr[1];
    vf.u64PTS = pts;

    decodedFrame = std::move(frame);
    return true;
}

void AxVdecH264Decoder::stop() {
    if (m_backend) {
        m_backend->stop();
        m_backend.reset();
    }
    m_started = false;
}

}  // namespace stereo_depth::axera_pipeline

// Host H.264 codec backend built on FFmpeg / libavcodec.
//
// Codec runs on the host (amd64 desktop FFmpeg, and equally Raspberry Pi /
// Orange Pi). It decodes with the always-available software H.264 decoder and
// encodes with the first available H.264 encoder (libx264 / libopenh264, or the
// V4L2 M2M hardware encoder on Raspberry Pi / Orange Pi).

#include "H264Codec.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include <linux/videodev2.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#define SAMPLE_LOG_TAG "CODEC"
#include "sample_log.h"

namespace stereo_depth::host_backend {

namespace {

// Scan /dev/video* for a V4L2 M2M decoder that advertises H.264 as a coded input
// format. Used so Raspberry Pi 5 (HEVC-only hardware decode) falls back to the
// software H.264 decoder instead of a non-functional hardware path.
bool v4l2HasH264Decoder() {
    for (int i = 0; i < 64; ++i) {
        char path[32];
        std::snprintf(path, sizeof(path), "/dev/video%d", i);
        const int fd = ::open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            continue;
        }
        v4l2_capability cap = {};
        if (::ioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) {
            ::close(fd);
            continue;
        }
        const bool isM2m =
            (cap.capabilities & (V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE)) != 0;
        if (!isM2m) {
            ::close(fd);
            continue;
        }
        for (auto type : {V4L2_BUF_TYPE_VIDEO_OUTPUT, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE}) {
            v4l2_fmtdesc desc = {};
            desc.type = type;
            for (desc.index = 0; ::ioctl(fd, VIDIOC_ENUM_FMT, &desc) == 0; ++desc.index) {
                if (desc.pixelformat == V4L2_PIX_FMT_H264) {
                    ::close(fd);
                    return true;
                }
            }
        }
        ::close(fd);
    }
    return false;
}

// libavcodec / libswscale emit AV_LOG_WARNING noise (e.g. "deprecated pixel
// format used, make sure you did set range correctly") during decode/scale.
// Quiet everything below errors once so the app log stays clean.
void quietLibavLogsOnce() {
    static bool done = false;
    if (!done) {
        av_log_set_level(AV_LOG_ERROR);
        done = true;
    }
}

// Map deprecated full-range "JPEG" YUV formats to their canonical equivalents so
// libswscale does not warn, and report whether the source is full range.
AVPixelFormat normalizeDeprecatedPixfmt(AVPixelFormat fmt, bool& fullRange) {
    fullRange = false;
    switch (fmt) {
        case AV_PIX_FMT_YUVJ420P:
            fullRange = true;
            return AV_PIX_FMT_YUV420P;
        case AV_PIX_FMT_YUVJ422P:
            fullRange = true;
            return AV_PIX_FMT_YUV422P;
        case AV_PIX_FMT_YUVJ444P:
            fullRange = true;
            return AV_PIX_FMT_YUV444P;
        case AV_PIX_FMT_YUVJ440P:
            fullRange = true;
            return AV_PIX_FMT_YUV440P;
        case AV_PIX_FMT_YUVJ411P:
            fullRange = true;
            return AV_PIX_FMT_YUV411P;
        default:
            return fmt;
    }
}

class HostH264Encoder : public IH264Encoder {
public:
    ~HostH264Encoder() override { stop(); }

    bool start(uint32_t width, uint32_t height, uint32_t fps) override {
        quietLibavLogsOnce();
        // Prefer software x264/openh264 for predictable Annex-B output; also try
        // the V4L2 M2M hardware encoder available on Raspberry Pi / Orange Pi.
        const char* names[] = {"libx264", "libopenh264", "h264_v4l2m2m", nullptr};
        const AVCodec* enc = nullptr;
        for (int i = 0; names[i] != nullptr; ++i) {
            enc = avcodec_find_encoder_by_name(names[i]);
            if (enc != nullptr) {
                break;
            }
        }
        if (enc == nullptr) {
            enc = avcodec_find_encoder(AV_CODEC_ID_H264);
        }
        if (enc == nullptr) {
            ALOGW("host encoder: no H.264 encoder available in libavcodec");
            return false;
        }

        m_ctx = avcodec_alloc_context3(enc);
        if (m_ctx == nullptr) {
            return false;
        }
        m_ctx->width = static_cast<int>(width);
        m_ctx->height = static_cast<int>(height);
        m_ctx->time_base = AVRational{1, static_cast<int>(fps > 0 ? fps : 30)};
        m_ctx->framerate = AVRational{static_cast<int>(fps > 0 ? fps : 30), 1};
        m_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
        m_ctx->gop_size = static_cast<int>(fps > 0 ? fps : 30);
        m_ctx->max_b_frames = 0;
        av_opt_set(m_ctx->priv_data, "preset", "ultrafast", 0);
        av_opt_set(m_ctx->priv_data, "tune", "zerolatency", 0);

        if (avcodec_open2(m_ctx, enc, nullptr) < 0) {
            ALOGW("host encoder: avcodec_open2 failed for %s", enc->name);
            stop();
            return false;
        }

        m_frame = av_frame_alloc();
        m_frame->format = AV_PIX_FMT_YUV420P;
        m_frame->width = static_cast<int>(width);
        m_frame->height = static_cast<int>(height);
        if (av_frame_get_buffer(m_frame, 32) < 0) {
            stop();
            return false;
        }
        m_packet = av_packet_alloc();
        m_width = width;
        m_height = height;
        m_fps = fps > 0 ? fps : 30;
        ALOGN("host H.264 encoder ready (%s, %ux%u@%u)", enc->name, width, height, m_fps);
        return true;
    }

    bool encode(const uint8_t* yuyv, size_t size, uint32_t width, uint32_t height, uint64_t ptsNs,
                std::vector<uint8_t>& outAnnexB, bool& keyFrame) override {
        outAnnexB.clear();
        keyFrame = false;
        if (m_ctx == nullptr || yuyv == nullptr) {
            return false;
        }
        if (size < static_cast<size_t>(width) * height * 2) {
            return false;
        }

        m_sws = sws_getCachedContext(m_sws, static_cast<int>(width), static_cast<int>(height),
                                     AV_PIX_FMT_YUYV422, static_cast<int>(width),
                                     static_cast<int>(height), AV_PIX_FMT_YUV420P, SWS_BILINEAR,
                                     nullptr, nullptr, nullptr);
        if (m_sws == nullptr) {
            return false;
        }
        if (av_frame_make_writable(m_frame) < 0) {
            return false;
        }
        const uint8_t* srcSlices[1] = {yuyv};
        const int srcStride[1] = {static_cast<int>(width) * 2};
        sws_scale(m_sws, srcSlices, srcStride, 0, static_cast<int>(height), m_frame->data,
                  m_frame->linesize);

        m_frame->pts = static_cast<int64_t>(m_frameIndex++);
        (void)ptsNs;

        if (avcodec_send_frame(m_ctx, m_frame) < 0) {
            return false;
        }
        while (true) {
            int ret = avcodec_receive_packet(m_ctx, m_packet);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            }
            if (ret < 0) {
                return false;
            }
            const size_t off = outAnnexB.size();
            outAnnexB.resize(off + m_packet->size);
            std::memcpy(outAnnexB.data() + off, m_packet->data, m_packet->size);
            if ((m_packet->flags & AV_PKT_FLAG_KEY) != 0) {
                keyFrame = true;
            }
            av_packet_unref(m_packet);
        }
        return true;
    }

    void stop() override {
        if (m_sws != nullptr) {
            sws_freeContext(m_sws);
            m_sws = nullptr;
        }
        if (m_packet != nullptr) {
            av_packet_free(&m_packet);
        }
        if (m_frame != nullptr) {
            av_frame_free(&m_frame);
        }
        if (m_ctx != nullptr) {
            avcodec_free_context(&m_ctx);
        }
    }

    const char* backendName() const override { return "host-ffmpeg"; }

private:
    AVCodecContext* m_ctx = nullptr;
    AVFrame* m_frame = nullptr;
    AVPacket* m_packet = nullptr;
    SwsContext* m_sws = nullptr;
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint32_t m_fps = 30;
    uint64_t m_frameIndex = 0;
};

class HostH264Decoder : public IH264Decoder {
public:
    ~HostH264Decoder() override { stop(); }

    bool begin(int width, int height) override {
        quietLibavLogsOnce();

        // Use the V4L2 M2M hardware decoder only when a device actually advertises
        // H.264 decode (Raspberry Pi 4 / Orange Pi). On HEVC-only hardware (e.g.
        // Raspberry Pi 5) fall back to software so H.264 still decodes.
        if (v4l2HasH264Decoder()) {
            const AVCodec* hw = avcodec_find_decoder_by_name("h264_v4l2m2m");
            if (hw != nullptr) {
                AVCodecContext* ctx = avcodec_alloc_context3(hw);
                if (ctx != nullptr) {
                    if (width > 0 && height > 0) {
                        ctx->width = width;
                        ctx->height = height;
                    }
                    if (avcodec_open2(ctx, hw, nullptr) == 0) {
                        m_ctx = ctx;
                        m_frame = av_frame_alloc();
                        m_packet = av_packet_alloc();
                        m_backendName = "host-ffmpeg(hw)";
                        ALOGN("host H.264 decoder ready (hardware: h264_v4l2m2m)");
                        return true;
                    }
                    avcodec_free_context(&ctx);
                }
            }
        }

        const AVCodec* dec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (dec == nullptr) {
            ALOGW("host decoder: no H.264 decoder available in libavcodec");
            return false;
        }
        m_ctx = avcodec_alloc_context3(dec);
        if (m_ctx == nullptr) {
            return false;
        }
        if (width > 0 && height > 0) {
            m_ctx->width = width;
            m_ctx->height = height;
        }
        if (avcodec_open2(m_ctx, dec, nullptr) < 0) {
            stop();
            return false;
        }
        m_frame = av_frame_alloc();
        m_packet = av_packet_alloc();
        m_backendName = "host-ffmpeg(sw)";
        ALOGN("host H.264 decoder ready (software)");
        return true;
    }

    bool decode(const uint8_t* annexB, size_t size, uint64_t pts, DecodedNv12& out) override {
        if (m_ctx == nullptr || annexB == nullptr || size == 0) {
            return false;
        }
        m_packet->data = const_cast<uint8_t*>(annexB);
        m_packet->size = static_cast<int>(size);
        if (avcodec_send_packet(m_ctx, m_packet) < 0) {
            return false;
        }
        int ret = avcodec_receive_frame(m_ctx, m_frame);
        if (ret < 0) {
            return false;  // need more data
        }

        const int w = m_frame->width;
        const int h = m_frame->height;
        // Many H.264 streams decode to the deprecated full-range YUVJ420P format.
        // Map it to the canonical YUV420P (identical memory layout) so libswscale
        // does not emit the "deprecated pixel format" warning, and flag full range.
        bool srcFullRange = false;
        const AVPixelFormat srcFmt =
            normalizeDeprecatedPixfmt(static_cast<AVPixelFormat>(m_frame->format), srcFullRange);
        m_sws = sws_getCachedContext(m_sws, w, h, srcFmt, w, h, AV_PIX_FMT_NV12, SWS_BILINEAR,
                                     nullptr, nullptr, nullptr);
        if (m_sws == nullptr) {
            return false;
        }
        if (srcFullRange) {
            int *invTable = nullptr, *table = nullptr, srcRange = 0, dstRange = 0, brightness = 0,
                contrast = 0, saturation = 0;
            if (sws_getColorspaceDetails(m_sws, &invTable, &srcRange, &table, &dstRange,
                                         &brightness, &contrast, &saturation) >= 0) {
                sws_setColorspaceDetails(m_sws, sws_getCoefficients(SWS_CS_DEFAULT), 1, table,
                                         dstRange, brightness, contrast, saturation);
            }
        }
        out.width = w;
        out.height = h;
        out.stride = w;
        out.pts = pts;
        out.data.resize(static_cast<size_t>(w) * h * 3 / 2);
        uint8_t* dstData[2] = {out.data.data(), out.data.data() + static_cast<size_t>(w) * h};
        int dstStride[2] = {w, w};
        sws_scale(m_sws, m_frame->data, m_frame->linesize, 0, h, dstData, dstStride);
        return true;
    }

    void stop() override {
        if (m_sws != nullptr) {
            sws_freeContext(m_sws);
            m_sws = nullptr;
        }
        if (m_packet != nullptr) {
            av_packet_free(&m_packet);
        }
        if (m_frame != nullptr) {
            av_frame_free(&m_frame);
        }
        if (m_ctx != nullptr) {
            avcodec_free_context(&m_ctx);
        }
    }

    const char* backendName() const override { return m_backendName; }

private:
    AVCodecContext* m_ctx = nullptr;
    AVFrame* m_frame = nullptr;
    AVPacket* m_packet = nullptr;
    SwsContext* m_sws = nullptr;
    const char* m_backendName = "host-ffmpeg";
};

}  // namespace

std::unique_ptr<IH264Encoder> createHostH264Encoder() {
    return std::make_unique<HostH264Encoder>();
}

std::unique_ptr<IH264Decoder> createHostH264Decoder() {
    return std::make_unique<HostH264Decoder>();
}

}  // namespace stereo_depth::host_backend

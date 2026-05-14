#pragma once

#include <ax_vdec_api.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace stereo_depth::axera_pipeline {

class AxVdecH264Decoder {
public:
    struct DecodedFrame {
        AX_VIDEO_FRAME_INFO_T frameInfo = {};
    };
    using DecodedFrameHandle = std::shared_ptr<DecodedFrame>;

    AxVdecH264Decoder();
    ~AxVdecH264Decoder();

    bool beginSequence(int width, int height, std::string& error);
    bool decodeAccessUnitToFrame(const std::vector<std::byte>& payload, uint64_t pts,
                                 DecodedFrameHandle& decodedFrame, std::string& error);
    void stop();

private:
    static constexpr AX_VDEC_CHN kOutputChannel = 1;

    struct ReleaseContext {
        std::mutex mtx;
        bool alive = false;
        AX_VDEC_GRP group = -1;
        AX_VDEC_CHN channel = 0;
        std::atomic<uint32_t> outstanding{0};
    };

    bool initModule(std::string& error);
    bool createGroup(int width, int height, std::string& error);
    bool startGroup(std::string& error);
    bool resetGroup(std::string& error);
    bool waitForOutstandingFrames(std::string& error);
    bool waitDecodedFrame(DecodedFrameHandle& decodedFrame, std::string& error);
    bool retainDecodedFrame(const AX_VIDEO_FRAME_INFO_T& frameInfo,
                            DecodedFrameHandle& decodedFrame, std::string& error);
    void releaseDecodedFrame(const AX_VIDEO_FRAME_INFO_T& frameInfo) const;
    bool destroyGroup(std::string* error = nullptr);
    void deinitModule();

    AX_VDEC_GRP m_group = -1;
    int m_width = 0;
    int m_height = 0;
    bool m_moduleInitialized = false;
    bool m_groupStarted = false;
    std::shared_ptr<ReleaseContext> m_releaseCtx;
};

}  // namespace stereo_depth::axera_pipeline

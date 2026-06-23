#pragma once

#include "BackendSelect.hpp"
#include "FoxgloveWrapper.hpp"
#include "FrameInputSource.hpp"
#include "ax_stereo_depth_api.h"

#include <atomic>
#include <cstdint>
#include <string>

namespace stereo_depth::app {

struct RuntimeOptions {
    bool perfTrace = false;
    uint32_t foxgloveFps = 15;
    bool enableVo = false;
    std::string dumpMcapPrefix;
    Backend imgProcBackend = Backend::Auto;
};

class StereoDepthAppRuntime {
public:
    StereoDepthAppRuntime(const RuntimeOptions& options, AX_STEREO_HANDLE hPipeline,
                          stereo_depth::FrameInputSource& inputSource, FoxgloveWrapper& foxglove);

    int run(std::atomic<bool>& running);

private:
    const RuntimeOptions& m_options;
    AX_STEREO_HANDLE m_hPipeline;
    stereo_depth::FrameInputSource& m_inputSource;
    FoxgloveWrapper& m_foxglove;
};

}  // namespace stereo_depth::app
#pragma once

#include "../../StereoDepthPipeline.hpp"

#include <cstdint>
#include <string>

namespace stereo_depth::axera_pipeline {

class AxVoDisplay {
public:
    AxVoDisplay() = default;
    ~AxVoDisplay();

    bool start(const std::string& outputSpec = "hdmi@1080P60@dev0");
    void stop();
    bool present(const stereo_depth::PipelineOutput& output);

    bool isActive() const { return m_active; }

private:
    struct Impl;
    Impl* m_impl = nullptr;
    bool m_active = false;
};

}  // namespace stereo_depth::axera_pipeline
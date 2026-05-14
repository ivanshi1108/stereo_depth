#pragma once

#include "InputFrameFormat.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ax_global_type.h"

namespace stereo_depth {

constexpr int kInputImageWidth = 1280 * 2;
constexpr int kInputImageHeight = 720;
constexpr int kDewarpImageWidth = 640;
constexpr int kDewarpImageHeight = 384;
constexpr float kCameraPrincipalPointX = 320.0f;
constexpr float kCameraPrincipalPointY = 182.0f;
constexpr float kCameraFocalLengthXPixels = 400.0f;
constexpr float kCameraFocalLengthYPixels = 480.0f;

inline bool isSupportedStereoInputResolution(int width, int height) {
    return (width == (1280 * 2) && height == 720) || (width == (1920 * 2) && height == 1080);
}

struct PipelineOutput {
    uint64_t frameTimestampNs = 0;
    uint32_t rawWidth = 0;
    uint32_t rawHeight = 0;
    std::shared_ptr<std::vector<std::byte>> rawYuyvData;
    AX_VIDEO_FRAME_T rawNv12Frame = {};
    AX_VIDEO_FRAME_T leftNv12Frame = {};
    std::shared_ptr<void> voFrameOwner;
    std::vector<std::byte> rgbData;
    std::vector<std::byte> depthData;
    std::vector<std::byte> confidenceData;
    std::vector<std::byte> zGridAvgData;
    std::vector<std::byte> pointCloudData;
};

enum class InferenceEngine {
    NPU = 0,
    DSP = 1,
};

enum class GdcMeshMode {
    Default = 0,
    DynamicReuse = 1,
    DynamicForce = 2,
};

const char* inferenceEngineName(InferenceEngine engine);
const char* defaultNpuModelPath();
std::string calibrationRuntimeDirectory();
std::string calibrationIniPath(const std::string& serialNumber);
int downloadCalibrationFile(const std::string& serialNumber);
int generateMeshFiles(const std::string& serialNumber, bool forceRegenerate,
                      std::string& leftMeshPath, std::string& rightMeshPath,
                      uint32_t inputWidth = kInputImageWidth,
                      uint32_t inputHeight = kInputImageHeight);
bool tryLoadCalibrationBaselineMeters(const std::string& serialNumber, float& baselineMeters);

class StereoDepthPipeline {
public:
    struct PostprocessStats {
        uint64_t depthUs = 0;
        uint64_t pointCloudUs = 0;
    };

    struct FrameContext;
    using FrameContextPtr = std::shared_ptr<FrameContext>;

    StereoDepthPipeline() = default;
    ~StereoDepthPipeline();

    int initialize(InferenceEngine engine = InferenceEngine::NPU, bool enableGdc = true,
                   GdcMeshMode gdcMeshMode = GdcMeshMode::DynamicReuse, bool dspDualCore = true,
                   bool exportVoFrames = false, const std::string& npuModelPath = "",
                   const std::string& cameraSerialNumber = "", int inputWidth = kInputImageWidth,
                   int inputHeight = kInputImageHeight);
    void shutdown();
    int preprocessFrame(FrameContextPtr& context, const void* inputFrame, size_t inputFrameSize,
                        uint64_t frameTimestampNs = 0,
                        InputFrameFormat inputFrameFormat = InputFrameFormat::Yuyv);
    int preprocessFrame(FrameContextPtr& context, const AX_VIDEO_FRAME_T* inputFrame,
                        uint64_t frameTimestampNs = 0,
                        InputFrameFormat inputFrameFormat = InputFrameFormat::Nv12,
                        std::shared_ptr<void> inputFrameOwner = {});
    int inferFrame(const FrameContextPtr& context);
    int postprocessFrame(FrameContextPtr& context, PipelineOutput& output,
                         PostprocessStats* stats = nullptr);
    void releaseFrameContext(FrameContextPtr& context);
    int processFrame(PipelineOutput& output, const void* inputFrame, size_t inputFrameSize,
                     uint64_t frameTimestampNs = 0,
                     InputFrameFormat inputFrameFormat = InputFrameFormat::Yuyv);
    InferenceEngine inferenceEngine() const { return m_inferenceEngine; }
    float cameraBaselineMeters() const { return m_cameraBaselineMeters; }

private:
    int preprocessPreparedNv12(FrameContextPtr& context);

    std::string m_meshLeftPath = "/opt/data/npu_disp/mesh_left.txt";
    std::string m_meshRightPath = "/opt/data/npu_disp/mesh_right.txt";
    InferenceEngine m_inferenceEngine = InferenceEngine::NPU;
    bool m_sysInited = false;
    bool m_ivpsInited = false;
    bool m_dspInited = false;
    bool m_dspInitedSecondary = false;
    bool m_dspCvInited = false;
    bool m_dspCvInitedSecondary = false;
    bool m_engineInited = false;
    bool m_npuInited = false;
    bool m_dspDualCoreEnabled = false;
    bool m_dspDualCoreRequested = true;
    bool m_enableGdc = true;
    bool m_exportVoFrames = false;
    GdcMeshMode m_gdcMeshMode = GdcMeshMode::DynamicReuse;
    float m_cameraBaselineMeters = 0.0629277f;
    int m_inputWidth = kInputImageWidth;
    int m_inputHeight = kInputImageHeight;
    int m_pointCloudScaleWidth = 0;
    int m_pointCloudScaleHeight = 0;
    float m_pointCloudScaleBaselineMeters = -1.0f;
    std::vector<float> m_pointCloudXScale;
    std::vector<float> m_pointCloudYScale;
};

}  // namespace stereo_depth

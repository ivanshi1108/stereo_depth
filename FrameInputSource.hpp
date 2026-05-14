#pragma once

#include "InputFrameFormat.hpp"
#include "V4L2Capture.hpp"

#include <ax_global_type.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace stereo_depth {

namespace axera_pipeline {
class AxVdecH264Decoder;
}  // namespace axera_pipeline

enum class McapImportStream {
    RawYuyv,
    H264,
};

enum class InputMode {
    Uvc,
    ImageFile,
};

struct InputSourceOptions {
    std::string device = "/dev/video0";
    int width = 0;
    int height = 0;
    int fps = 15;
    V4L2Capture::UvcControlSettings uvcControls;
    bool useImageFile = false;
    std::string imageFile;
    McapImportStream mcapImportStream = McapImportStream::RawYuyv;
};

struct InputSourceInfo {
    InputMode mode = InputMode::Uvc;
    std::string device;
    std::string serialNumber;
    bool isUsb = false;
    int width = 0;
    int height = 0;
    int fps = 0;
};

struct FrameLaserDistance {
    int32_t distanceMm = -1;

    bool valid() const { return distanceMm >= 0; }
};

class FrameInputSource {
public:
    struct ImportedFrameMeta {
        uint64_t frameTimestampNs = 0;
        int32_t laserDistanceMm = -1;
        bool keyFrame = false;
    };

    struct ImportedFrameSelection {
        size_t index = 0;
        size_t count = 0;
        uint64_t frameTimestampNs = 0;
    };

    explicit FrameInputSource(const InputSourceOptions& options);
    ~FrameInputSource();

    bool initialize();
    bool getFrame(const void*& frameData, size_t& frameSize, uint64_t& frameTimestampNs,
                  FrameLaserDistance* laserDistance = nullptr,
                  InputFrameFormat* frameFormat = nullptr,
                  const AX_VIDEO_FRAME_T** externalFrame = nullptr,
                  std::shared_ptr<void>* externalFrameOwner = nullptr);
    void releaseFrame();
    void shutdown();

    InputMode mode() const { return m_mode; }
    bool isMcapInput() const {
        return m_mode == InputMode::ImageFile && !m_importedMcapPath.empty();
    }
    const InputSourceInfo& info() const { return m_info; }
    bool usesRecordedReplayTiming() const;
    bool supportsImportedFrameSelection() const;
    ImportedFrameSelection importedFrameSelection() const;
    bool stepImportedFrame(int delta, ImportedFrameSelection& selection);
    const char* importedFrameSourceName() const;

private:
    struct H264McapReplayCursor;

    enum class ImportedMcapSource {
        None,
        RawYuyv,
        H264,
    };

    struct ImportedFrame {
        std::vector<std::byte> data;
        InputFrameFormat frameFormat = InputFrameFormat::Yuyv;
        AX_VIDEO_FRAME_T externalFrame = {};
        std::shared_ptr<void> externalFrameOwner;
        uint64_t frameTimestampNs = 0;
        int32_t laserDistanceMm = -1;
    };

    bool initializeUvc();
    bool initializeImageFile();
    bool loadImportedFrame(size_t frameIndex, ImportedFrameSelection& selection,
                           std::string& error);
    bool prepareImportedH264ReplayCursor(std::string& error);
    bool readNextImportedH264AccessUnit(std::vector<std::byte>& accessUnit,
                                        uint64_t& frameTimestampNs, std::string& error);
    bool loadNextImportedH264Frame(std::shared_ptr<const ImportedFrame>& frame, std::string& error);
    void resetImportedH264ReplayState();

    InputSourceOptions m_options;
    InputMode m_mode = InputMode::Uvc;
    std::unique_ptr<V4L2Capture> m_capture;
    bool m_captureStarted = false;
    std::vector<void*> m_captureBuffers;
    V4L2Capture::Frame m_currentFrame{};
    bool m_hasCurrentFrame = false;
    std::vector<std::byte> m_imageData;
    std::string m_importedMcapPath;
    ImportedMcapSource m_importedMcapSource = ImportedMcapSource::None;
    std::vector<ImportedFrameMeta> m_importedFrames;
    mutable std::mutex m_importedFrameMutex;
    std::shared_ptr<const ImportedFrame> m_selectedImportedFrame;
    std::shared_ptr<const ImportedFrame> m_leasedImportedFrame;
    std::atomic<size_t> m_selectedImportedFrameIndex{0};
    bool m_replayClockStarted = false;
    uint64_t m_replayStartTimestampNs = 0;
    std::chrono::steady_clock::time_point m_replayStartSteadyTime{};
    std::unique_ptr<axera_pipeline::AxVdecH264Decoder> m_h264ReplayDecoder;
    std::unique_ptr<H264McapReplayCursor> m_h264McapReplayCursor;
    InputSourceInfo m_info;
};

}  // namespace stereo_depth

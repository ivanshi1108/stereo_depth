#include "AppRuntime.hpp"

#include "AxVencH264Encoder.hpp"

#include "AppTopics.hpp"
#include "AxVoDisplay.hpp"
#include "LaserRangefinder.hpp"
#include "ax_stereo_depth_api.h"

#define SAMPLE_LOG_TAG "RUNTIME"
#include "sample_log.h"

#include <foxglove/mcap.hpp>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using stereo_depth::axera_pipeline::AxVoDisplay;

namespace stereo_depth::app {
namespace {

enum class KeyboardInputAction {
    None,
    ToggleRecord,
    SingleFrameDump,
    PrevImportedFrame,
    NextImportedFrame,
};

constexpr uint64_t kMinReadyFramesBeforeCaptureActions = 15;
constexpr uint64_t kMaxMcapFileSizeBytes = 512ULL * 1024ULL * 1024ULL;

constexpr int kKeyboardEscapeReadTimeoutUs = 2000;
constexpr size_t kMaxKeyboardEscapeSequenceLength = 16;

bool readByteWithTimeout(int fd, char& ch, int timeoutUs) {
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(fd, &readSet);

    timeval timeout;
    timeout.tv_sec = timeoutUs / 1000000;
    timeout.tv_usec = timeoutUs % 1000000;

    const int ready = ::select(fd + 1, &readSet, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        return false;
    }

    return ::read(fd, &ch, 1) == 1;
}

std::string readKeyboardEscapeSequence(int fd) {
    std::string sequence;
    sequence.reserve(kMaxKeyboardEscapeSequenceLength);

    while (sequence.size() < kMaxKeyboardEscapeSequenceLength) {
        char ch = 0;
        if (!readByteWithTimeout(fd, ch, kKeyboardEscapeReadTimeoutUs)) {
            break;
        }

        sequence.push_back(ch);
        if (std::isalpha(static_cast<unsigned char>(ch)) || ch == '~') {
            break;
        }
    }

    return sequence;
}

KeyboardInputAction parseKeyboardEscapeSequence(const std::string& sequence) {
    if (sequence.size() < 2) {
        return KeyboardInputAction::None;
    }

    if (sequence[0] != '[') {
        return KeyboardInputAction::None;
    }

    const char finalChar = sequence.back();
    if (finalChar == 'D') {
        return KeyboardInputAction::PrevImportedFrame;
    }
    if (finalChar == 'C') {
        return KeyboardInputAction::NextImportedFrame;
    }
    return KeyboardInputAction::None;
}

KeyboardInputAction readKeyboardInputAction() {
    char ch = 0;
    const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
    if (n <= 0) {
        return KeyboardInputAction::None;
    }

    if (ch == 'r' || ch == 'R') {
        return KeyboardInputAction::ToggleRecord;
    }
    if (ch == 'd' || ch == 'D') {
        return KeyboardInputAction::SingleFrameDump;
    }
    if (ch != '\x1b') {
        return KeyboardInputAction::None;
    }

    return parseKeyboardEscapeSequence(readKeyboardEscapeSequence(STDIN_FILENO));
}

struct FramePacket {
    uint64_t frameTimestampNs = 0;
    uint64_t captureSteadyNs = 0;
    LaserDistanceSample laserDistance;
    stereo_depth::InputFrameFormat format = stereo_depth::InputFrameFormat::Yuyv;
    AX_VIDEO_FRAME_T externalFrame = {};
    std::shared_ptr<void> externalFrameOwner;
    std::vector<std::byte> data;
};

struct StereoOutputOwner {
    AX_STEREO_OUTPUT_T stereoOutput = {};

    stereo_depth::PipelineOutput* data() { return AX_STEREO_OutputGetData(&stereoOutput); }
    const stereo_depth::PipelineOutput* data() const {
        return const_cast<StereoOutputOwner*>(this)->data();
    }
    ~StereoOutputOwner() {
        if (stereoOutput.pPrivateData) {
            AX_STEREO_ReleaseOutput(&stereoOutput);
        }
    }
    StereoOutputOwner() = default;
    StereoOutputOwner(const StereoOutputOwner&) = delete;
    StereoOutputOwner& operator=(const StereoOutputOwner&) = delete;
    StereoOutputOwner(StereoOutputOwner&& o) noexcept : stereoOutput(o.stereoOutput) {
        o.stereoOutput = {};
    }
    StereoOutputOwner& operator=(StereoOutputOwner&& o) noexcept {
        if (this != &o) {
            if (stereoOutput.pPrivateData) AX_STEREO_ReleaseOutput(&stereoOutput);
            stereoOutput = o.stereoOutput;
            o.stereoOutput = {};
        }
        return *this;
    }
};

struct OutputPacket {
    uint64_t captureSteadyNs = 0;
    LaserDistanceSample laserDistance;
    std::shared_ptr<StereoOutputOwner> output;
};

struct VoPacket {
    std::shared_ptr<StereoOutputOwner> output;
};

struct ProcessStagePacket {
    AX_STEREO_FRAME_CTX context = nullptr;
    uint64_t captureSteadyNs = 0;
    LaserDistanceSample laserDistance;
    uint64_t preWaitInUs = 0;
    uint64_t inferWaitInUs = 0;
    uint64_t preprocessUs = 0;
    uint64_t inferUs = 0;
    size_t inQDepth = 0;
};

struct DumpTask {
    std::shared_ptr<StereoOutputOwner> output;
    LaserDistanceSample laserDistance;
    TopicFlags dataFlags;
    bool singleFrame = false;
    bool recordH264 = false;
};

enum class H264RecordState {
    Disabled,
    Active,
    Failed,
};

struct PerfAccum {
    uint64_t frames = 0;
    uint64_t samples = 0;
    uint64_t dropped = 0;
    uint64_t waitInUs = 0;
    uint64_t workUs = 0;
    uint64_t preWaitInUs = 0;
    uint64_t inferWaitInUs = 0;
    uint64_t postWaitInUs = 0;
    uint64_t preUs = 0;
    uint64_t inferUs = 0;
    uint64_t postUs = 0;
    uint64_t e2eUs = 0;
    size_t qIn = 0;
    size_t qOut = 0;
};

constexpr size_t kFrameQueueCapacity = 4;
constexpr size_t kOutputQueueCapacity = 4;
constexpr size_t kPreInferQueueCapacity = 1;
constexpr size_t kInferPostQueueCapacity = 4;
constexpr size_t kVoQueueCapacity = 2;
constexpr size_t kRecordQueueCapacity = 8;
constexpr int kDashboardInnerWidth = 51;
constexpr auto kImageFileInputFrameInterval = std::chrono::milliseconds(200);

enum class RuntimeLogKind {
    Notice,
    Record,
    Dump,
};

constexpr const char* kAnsiReset = "\033[0m";
constexpr const char* kRuntimeNoticeColor = "\033[1;36m";
constexpr const char* kRuntimeRecordColor = "\033[1;35m";
constexpr const char* kRuntimeDumpColor = "\033[1;34m";

const char* runtimeLogColor(RuntimeLogKind kind) {
    switch (kind) {
        case RuntimeLogKind::Record:
            return kRuntimeRecordColor;
        case RuntimeLogKind::Dump:
            return kRuntimeDumpColor;
        case RuntimeLogKind::Notice:
        default:
            return kRuntimeNoticeColor;
    }
}

std::string formatRuntimeLogLine(RuntimeLogKind kind, const std::string& msg) {
    const char* kindStr = "NOTICE";
    switch (kind) {
        case RuntimeLogKind::Record:
            kindStr = "RECORD";
            break;
        case RuntimeLogKind::Dump:
            kindStr = "DUMP";
            break;
        case RuntimeLogKind::Notice:
        default:
            break;
    }

    std::ostringstream oss;
    oss << runtimeLogColor(kind) << kindStr << ":[" << std::left << std::setw(SAMPLE_LOG_TAG_WIDTH)
        << SAMPLE_LOG_TAG << "] " << msg << kAnsiReset;
    return oss.str();
}

uint64_t nowSteadyNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

std::string formatWallClockNs(uint64_t timeNs) {
    const auto timePoint =
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
            std::chrono::nanoseconds(timeNs));
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(timePoint);
    const auto subsecNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint - seconds).count();

    const std::time_t timeValue = std::chrono::system_clock::to_time_t(seconds);
    std::tm localTm = {};
    localtime_r(&timeValue, &localTm);

    std::ostringstream oss;
    oss << std::put_time(&localTm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(9) << std::setfill('0')
        << subsecNs;
    return oss.str();
}

std::string formatDurationNs(uint64_t durationNs) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << (static_cast<double>(durationNs) / 1000000000.0)
        << "s";
    return oss.str();
}

std::string formatByteSize(uintmax_t sizeBytes) {
    static constexpr const char* kUnits[] = {"B", "KiB", "MiB", "GiB", "TiB"};
    double size = static_cast<double>(sizeBytes);
    size_t unitIndex = 0;
    while (size >= 1024.0 && unitIndex < (sizeof(kUnits) / sizeof(kUnits[0])) - 1) {
        size /= 1024.0;
        ++unitIndex;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(unitIndex == 0 ? 0 : 2) << size << kUnits[unitIndex];
    return oss.str();
}

std::string sanitizePathComponent(const std::string& value) {
    std::string sanitized;
    sanitized.reserve(std::min<size_t>(value.size(), 64));
    for (unsigned char ch : value) {
        if (std::isalnum(ch) || ch == '-' || ch == '_') {
            sanitized.push_back(static_cast<char>(ch));
        } else {
            sanitized.push_back('_');
        }

        if (sanitized.size() >= 64) {
            break;
        }
    }

    while (!sanitized.empty() && sanitized.back() == '_') {
        sanitized.pop_back();
    }

    if (sanitized.empty()) {
        sanitized = "unknown";
    }

    return sanitized;
}

uint64_t buildTimestampedOutputPath(const std::string& prefix, const std::string& fileTag,
                                    const std::string& serialNumber, const std::string& extension,
                                    std::string& path) {
    const uint64_t nowNs =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count());
    std::ostringstream oss;
    oss << prefix;
    if (!fileTag.empty()) {
        oss << '_' << fileTag;
    }
    if (!serialNumber.empty()) {
        const std::string safeSerial = sanitizePathComponent(serialNumber);
        if (!safeSerial.empty()) {
            oss << "_sn" << safeSerial;
        }
    }
    oss << '_' << nowNs << extension;
    path = oss.str();
    return nowNs;
}

void buildSegmentedOutputPath(const std::string& prefix, const std::string& fileTag,
                              const std::string& serialNumber, uint64_t timestampNs,
                              uint32_t segmentIndex, const std::string& extension,
                              std::string& path) {
    std::ostringstream oss;
    oss << prefix;
    if (!fileTag.empty()) {
        oss << '_' << fileTag;
    }
    if (!serialNumber.empty()) {
        const std::string safeSerial = sanitizePathComponent(serialNumber);
        if (!safeSerial.empty()) {
            oss << "_sn" << safeSerial;
        }
    }
    oss << '_' << timestampNs;
    if (segmentIndex > 0) {
        oss << "_part" << std::setw(2) << std::setfill('0') << (segmentIndex + 1);
    }
    oss << extension;
    path = oss.str();
}

uintmax_t getFileSizeBytes(const std::string& path) {
    if (path.empty()) {
        return 0;
    }

    std::error_code fileErr;
    const uintmax_t fileSizeBytes = std::filesystem::file_size(path, fileErr);
    return fileErr ? 0 : fileSizeBytes;
}

struct RoiRegionAverage {
    const char* name = "";
    uint32_t centerX = 0;
    uint32_t centerY = 0;
    uint32_t validSampleCount = 0;
    double zAvgMeters = 0.0;
    uint32_t confidenceSampleCount = 0;
    double confidenceAvg = 0.0;
    bool valid = false;
    bool confidenceValid = false;
};

std::array<RoiRegionAverage, 9> computeRoiRegionAverages(const stereo_depth::PipelineOutput& output,
                                                         float focalLengthPixels,
                                                         float baselineMeters) {
    std::array<RoiRegionAverage, 9> regions = {{
        {"ROI-LT"},
        {"ROI-MT"},
        {"ROI-RT"},
        {"ROI-LM"},
        {"ROI-CT"},
        {"ROI-RM"},
        {"ROI-LD"},
        {"ROI-MD"},
        {"ROI-RD"},
    }};

    constexpr uint32_t kWindowRadius = 5;
    constexpr uint32_t kDisparityScale = 16;
    const size_t expectedBytes = static_cast<size_t>(AX_STEREO_DEWARP_IMAGE_WIDTH) *
                                 static_cast<size_t>(AX_STEREO_DEWARP_IMAGE_HEIGHT) *
                                 sizeof(uint16_t);
    if (output.depthData.size() < expectedBytes) {
        return regions;
    }

    const auto* disparityFixed = reinterpret_cast<const uint16_t*>(output.depthData.data());
    const size_t expectedConfidenceBytes = static_cast<size_t>(AX_STEREO_DEWARP_IMAGE_WIDTH) *
                                           static_cast<size_t>(AX_STEREO_DEWARP_IMAGE_HEIGHT) *
                                           sizeof(float);
    const auto* confidenceValues =
        output.confidenceData.size() >= expectedConfidenceBytes
            ? reinterpret_cast<const float*>(output.confidenceData.data())
            : nullptr;
    const double zScale =
        static_cast<double>(focalLengthPixels) * static_cast<double>(baselineMeters);
    static constexpr const uint32_t kRoiOrder[9][2] = {
        {0, 0}, {1, 0}, {2, 0}, {0, 1}, {1, 1}, {2, 1}, {0, 2}, {1, 2}, {2, 2},
    };

    static constexpr uint32_t kRoiX[3] = {180, 320, 460};
    static constexpr uint32_t kRoiY[3] = {96, 192, 288};

    for (size_t i = 0; i < regions.size(); ++i) {
        const uint32_t gridX = kRoiOrder[i][0];
        const uint32_t gridY = kRoiOrder[i][1];
        RoiRegionAverage& region = regions[i];
        region.centerX = kRoiX[gridX];
        region.centerY = kRoiY[gridY];

        const uint32_t xBegin =
            region.centerX > kWindowRadius ? (region.centerX - kWindowRadius) : 0;
        const uint32_t xEnd =
            std::min<uint32_t>(AX_STEREO_DEWARP_IMAGE_WIDTH - 1, region.centerX + kWindowRadius);
        const uint32_t yBegin =
            region.centerY > kWindowRadius ? (region.centerY - kWindowRadius) : 0;
        const uint32_t yEnd =
            std::min<uint32_t>(AX_STEREO_DEWARP_IMAGE_HEIGHT - 1, region.centerY + kWindowRadius);

        double sumZ = 0.0;
        double confidenceSum = 0.0;
        uint32_t validCount = 0;
        uint32_t confidenceCount = 0;
        for (uint32_t y = yBegin; y <= yEnd; ++y) {
            const size_t rowBase = static_cast<size_t>(y) * AX_STEREO_DEWARP_IMAGE_WIDTH;
            for (uint32_t x = xBegin; x <= xEnd; ++x) {
                if (confidenceValues != nullptr) {
                    const float confidenceValue = confidenceValues[rowBase + x];
                    if (std::isfinite(confidenceValue)) {
                        confidenceSum += static_cast<double>(confidenceValue);
                        ++confidenceCount;
                    }
                }

                const uint16_t disparityValue = disparityFixed[rowBase + x];
                if (disparityValue == 0) {
                    continue;
                }

                const double disparity =
                    static_cast<double>(disparityValue) / static_cast<double>(kDisparityScale);
                if (disparity <= 0.0) {
                    continue;
                }

                sumZ += zScale / disparity;
                ++validCount;
            }
        }

        region.validSampleCount = validCount;
        region.valid = validCount > 0;
        if (region.valid) {
            region.zAvgMeters = sumZ / static_cast<double>(validCount);
        }
        region.confidenceSampleCount = confidenceCount;
        region.confidenceValid = confidenceCount > 0;
        if (region.confidenceValid) {
            region.confidenceAvg = confidenceSum / static_cast<double>(confidenceCount);
        }
    }

    return regions;
}

std::vector<std::byte> makeRoiRegionAveragesJson(const stereo_depth::PipelineOutput& output,
                                                 float focalLengthPixels, float baselineMeters,
                                                 const LaserDistanceSample& laserDistance) {
    const auto regions = computeRoiRegionAverages(output, focalLengthPixels, baselineMeters);

    std::ostringstream oss;
    oss << '{';
    oss << "\"frame_timestamp_ns\":" << output.frameTimestampNs << ',';
    oss << "\"items\":[";
    oss << '{';
    oss << "\"name\":\"laser_distance_mm\",";
    oss << "\"z_avg_mm\":";
    if (laserDistance.valid()) {
        oss << laserDistance.distanceMm;
    } else {
        oss << "null";
    }
    oss << '}';
    for (size_t i = 0; i < regions.size(); ++i) {
        const RoiRegionAverage& region = regions[i];
        oss << ',';

        oss << '{';
        oss << "\"name\":\"" << region.name << '[' << region.centerX << ',' << region.centerY
            << "]\",";
        oss << "\"z_avg_mm\":";
        if (region.valid) {
            oss << std::fixed << std::setprecision(3) << (region.zAvgMeters * 1000.0);
        } else {
            oss << "null";
        }
        oss << ",\"confidence_avg\":";
        if (region.confidenceValid) {
            oss << std::fixed << std::setprecision(6) << region.confidenceAvg;
        } else {
            oss << "null";
        }
        oss << '}';
    }
    oss << "]}";

    const std::string json = oss.str();
    std::vector<std::byte> payload(json.size());
    std::memcpy(payload.data(), json.data(), json.size());
    return payload;
}

std::string formatDashboardRow(const std::string& stage, double fps, double waitInMs, double workMs,
                               uint64_t dropped) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << "| " << std::left << std::setw(5) << stage << " | "
        << std::right << std::setw(8) << fps << " | " << std::setw(10) << waitInMs << " | "
        << std::setw(9) << workMs << " | " << std::setw(7) << dropped << " |";
    return oss.str();
}

std::string formatMetricLine(const std::string& text) {
    std::ostringstream oss;
    oss << "| " << std::left << std::setw(kDashboardInnerWidth) << text << " |";
    return oss.str();
}

std::optional<uint64_t> readProcessRssKb() {
    std::ifstream statusFile("/proc/self/status");
    if (!statusFile.is_open()) {
        return std::nullopt;
    }

    std::string line;
    while (std::getline(statusFile, line)) {
        if (line.rfind("VmRSS:", 0) != 0) {
            continue;
        }

        std::istringstream iss(line.substr(std::strlen("VmRSS:")));
        uint64_t rssKb = 0;
        if (iss >> rssKb) {
            return rssKb;
        }
        break;
    }

    return std::nullopt;
}

void notifyAll(std::condition_variable& frameQueueNotEmptyCv,
               std::condition_variable& frameQueueNotFullCv,
               std::condition_variable& preInferQueueNotEmptyCv,
               std::condition_variable& preInferQueueNotFullCv,
               std::condition_variable& inferPostQueueNotEmptyCv,
               std::condition_variable& inferPostQueueNotFullCv,
               std::condition_variable& outputQueueNotEmptyCv,
               std::condition_variable& outputQueueNotFullCv,
               std::condition_variable& voQueueNotEmptyCv) {
    frameQueueNotEmptyCv.notify_all();
    frameQueueNotFullCv.notify_all();
    preInferQueueNotEmptyCv.notify_all();
    preInferQueueNotFullCv.notify_all();
    inferPostQueueNotEmptyCv.notify_all();
    inferPostQueueNotFullCv.notify_all();
    outputQueueNotEmptyCv.notify_all();
    outputQueueNotFullCv.notify_all();
    voQueueNotEmptyCv.notify_all();
}

}  // namespace

StereoDepthAppRuntime::StereoDepthAppRuntime(const RuntimeOptions& options,
                                             AX_STEREO_HANDLE hPipeline,
                                             stereo_depth::FrameInputSource& inputSource,
                                             FoxgloveWrapper& foxglove)
    : m_options(options),
      m_hPipeline(hPipeline),
      m_inputSource(inputSource),
      m_foxglove(foxglove) {}

int StereoDepthAppRuntime::run(std::atomic<bool>& running) {
    StereoDepthTopics topics;
    if (!topics.registerFoxgloveChannels(m_foxglove)) {
        return 1;
    }
    const auto& inputInfo = m_inputSource.info();
    topics.publishDeviceInfoToFoxglove(m_foxglove, inputInfo);

    const bool perfTrace = m_options.perfTrace;
    const bool perfTraceTty = perfTrace && (::isatty(STDOUT_FILENO) != 0);

    std::ostringstream keyboardHelp;
    keyboardHelp
        << "Keyboard: 'r' toggles MCAP recording with H.264, first 'd' starts continuous H.264 "
           "recording and every 'd' saves a single-frame MCAP snapshot with raw YUYV";
    if (m_inputSource.supportsImportedFrameSelection()) {
        const auto selection = m_inputSource.importedFrameSelection();
        keyboardHelp << ", Left/Right selects previous/next imported "
                     << m_inputSource.importedFrameSourceName() << " frame (default 1/"
                     << selection.count << ')';
    }
    ALOGN("%s", keyboardHelp.str().c_str());

    std::mutex perfLogMutex;
    std::mutex dumpLogMutex;
    std::deque<std::string> dumpLogs;
    std::mutex foxglovePublishMutex;
    std::unique_ptr<AxVoDisplay> voDisplay;

    if (m_options.enableVo) {
        voDisplay = std::make_unique<AxVoDisplay>();
        if (!voDisplay->start()) {
            ALOGE("failed to initialize VO HDMI output");
            return 1;
        }
    }
    AxVoDisplay* const voDisplayPtr = voDisplay.get();
    const bool voEnabled = (voDisplayPtr != nullptr);

    auto appendRuntimeLog = [&](RuntimeLogKind kind, const std::string& msg) {
        const std::string formattedMsg = formatRuntimeLogLine(kind, msg);
        std::lock_guard<std::mutex> lock(dumpLogMutex);
        dumpLogs.push_back(formattedMsg);
        if (!perfTrace) {
            std::cout << formattedMsg << std::endl;
        }
    };
    auto appendRecordLog = [&](const std::string& msg) {
        appendRuntimeLog(RuntimeLogKind::Record, msg);
    };
    auto appendSingleDumpLog = [&](const std::string& msg) {
        appendRuntimeLog(RuntimeLogKind::Dump, msg);
    };

    std::mutex perfAccumMutex;
    PerfAccum capPerf;
    PerfAccum procPerf;
    PerfAccum pubPerf;
    PerfAccum voPerf;
    PerfAccum dumpPerf;
    std::atomic<uint64_t> droppedFrameQueue{0};
    std::atomic<uint64_t> droppedPreInferQueue{0};
    std::atomic<uint64_t> droppedInferPostQueue{0};
    std::atomic<uint64_t> droppedOutputQueue{0};
    std::atomic<uint64_t> droppedVoQueue{0};
    std::atomic<uint64_t> droppedDumpQueue{0};
    std::atomic<size_t> maxFrameQueueDepth{0};
    std::atomic<size_t> maxPreInferQueueDepth{0};
    std::atomic<size_t> maxInferPostQueueDepth{0};
    std::atomic<size_t> maxOutputQueueDepth{0};
    std::atomic<size_t> maxVoQueueDepth{0};
    std::atomic<size_t> maxDumpQueueDepth{0};

    auto updateMaxDepth = [](std::atomic<size_t>& target, size_t value) {
        size_t current = target.load(std::memory_order_relaxed);
        while (current < value &&
               !target.compare_exchange_weak(current, value, std::memory_order_relaxed,
                                             std::memory_order_relaxed)) {
        }
    };

    std::atomic<bool> perfMonitorStop{false};
    std::thread perfMonitorThread;
    if (perfTrace) {
        perfMonitorThread = std::thread([&]() {
            auto lastReportTime = std::chrono::steady_clock::now();
            bool firstRender = true;
            bool cursorHidden = false;
            size_t renderedDumpCount = 0;
            size_t renderedDashboardLineCount = 0;
            const std::string staticTitleLine = "[perf] StereoDepth Dashboard (window=1s)";

            while (!perfMonitorStop.load()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (perfMonitorStop.load()) {
                    break;
                }

                const auto now = std::chrono::steady_clock::now();
                const auto elapsedMs =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReportTime)
                        .count();
                if (elapsedMs <= 0) {
                    continue;
                }

                PerfAccum capSnapshot;
                PerfAccum procSnapshot;
                PerfAccum pubSnapshot;
                PerfAccum voSnapshot;
                PerfAccum dumpSnapshot;
                {
                    std::lock_guard<std::mutex> lock(perfAccumMutex);
                    capSnapshot = capPerf;
                    procSnapshot = procPerf;
                    pubSnapshot = pubPerf;
                    voSnapshot = voPerf;
                    dumpSnapshot = dumpPerf;
                    capPerf = PerfAccum{};
                    procPerf = PerfAccum{};
                    pubPerf = PerfAccum{};
                    voPerf = PerfAccum{};
                    dumpPerf = PerfAccum{};
                }

                auto toFps = [&](uint64_t frames) {
                    return static_cast<double>(frames) * 1000.0 / static_cast<double>(elapsedMs);
                };
                auto toAvgMs = [](uint64_t totalUs, uint64_t frames) {
                    return frames > 0
                               ? static_cast<double>(totalUs) / 1000.0 / static_cast<double>(frames)
                               : 0.0;
                };

                const double procPreWaitInMs =
                    toAvgMs(procSnapshot.preWaitInUs, procSnapshot.frames);
                const double procInferWaitInMs =
                    toAvgMs(procSnapshot.inferWaitInUs, procSnapshot.frames);
                const double procPostWaitInMs =
                    toAvgMs(procSnapshot.postWaitInUs, procSnapshot.frames);
                const double procPreMs = toAvgMs(procSnapshot.preUs, procSnapshot.frames);
                const double procInferMs = toAvgMs(procSnapshot.inferUs, procSnapshot.frames);
                const double procPostMs = toAvgMs(procSnapshot.postUs, procSnapshot.frames);
                const double pubE2eMs = toAvgMs(pubSnapshot.e2eUs, pubSnapshot.frames);
                const uint64_t frameDrop = droppedFrameQueue.exchange(0);
                const uint64_t preInferDrop = droppedPreInferQueue.exchange(0);
                const uint64_t inferPostDrop = droppedInferPostQueue.exchange(0);
                const uint64_t outputDrop = droppedOutputQueue.exchange(0);
                const uint64_t voDrop = droppedVoQueue.exchange(0);
                const uint64_t dumpDrop = droppedDumpQueue.exchange(0);
                const size_t frameDepthMax = maxFrameQueueDepth.exchange(0);
                const size_t preInferDepthMax = maxPreInferQueueDepth.exchange(0);
                const size_t inferPostDepthMax = maxInferPostQueueDepth.exchange(0);
                const size_t outputDepthMax = maxOutputQueueDepth.exchange(0);
                const size_t voDepthMax = maxVoQueueDepth.exchange(0);
                const size_t dumpDepthMax = maxDumpQueueDepth.exchange(0);
                const FoxgloveWrapper::SubscriberStats subStats = m_foxglove.getSubscriberStats();

                std::ostringstream titleStream;
                titleStream << "subs_total=" << subStats.total
                            << " rgb=" << m_foxglove.getSubscriberCount(topics.rgbTopic())
                            << " depth=" << m_foxglove.getSubscriberCount(topics.depthTopic())
                            << " gridavg=" << m_foxglove.getSubscriberCount(topics.gridAvgTopic())
                            << " cloud=" << m_foxglove.getSubscriberCount(topics.cloudTopic())
                            << " grid=" << m_foxglove.getSubscriberCount(topics.gridTopic());
                const std::string dynamicTitleLine = titleStream.str();
                const std::string splitLine =
                    "+-------+----------+------------+-----------+---------+";
                const std::string headerLine =
                    "| stage |      fps | wait_in_ms |   work_ms | dropped |";
                const std::string capLine =
                    formatDashboardRow("cap", toFps(capSnapshot.frames),
                                       toAvgMs(capSnapshot.waitInUs, capSnapshot.frames),
                                       toAvgMs(capSnapshot.workUs, capSnapshot.frames), frameDrop);
                const std::string preLine = formatDashboardRow(
                    "pre", toFps(procSnapshot.frames), procPreWaitInMs, procPreMs, preInferDrop);
                const std::string inferLine =
                    formatDashboardRow("infer", toFps(procSnapshot.frames), procInferWaitInMs,
                                       procInferMs, inferPostDrop);
                const std::string postLine = formatDashboardRow(
                    "post", toFps(procSnapshot.frames), procPostWaitInMs, procPostMs, outputDrop);
                const std::string pubLine = formatDashboardRow(
                    "pub", toFps(pubSnapshot.frames),
                    toAvgMs(pubSnapshot.waitInUs, pubSnapshot.samples),
                    toAvgMs(pubSnapshot.workUs, pubSnapshot.samples), pubSnapshot.dropped);
                const std::string voLine =
                    formatDashboardRow("vo", toFps(voSnapshot.frames),
                                       toAvgMs(voSnapshot.waitInUs, voSnapshot.samples),
                                       toAvgMs(voSnapshot.workUs, voSnapshot.samples), voDrop);
                const std::string dumpLine =
                    formatDashboardRow("dump", toFps(dumpSnapshot.frames),
                                       toAvgMs(dumpSnapshot.waitInUs, dumpSnapshot.samples),
                                       toAvgMs(dumpSnapshot.workUs, dumpSnapshot.frames), dumpDrop);

                std::ostringstream latencyLineStream;
                latencyLineStream << std::fixed << std::setprecision(2) << "e2e_ms=" << pubE2eMs;

                const std::optional<uint64_t> rssKb = readProcessRssKb();
                if (rssKb.has_value()) {
                    latencyLineStream << " rss_mb=" << (static_cast<double>(*rssKb) / 1024.0);
                } else {
                    latencyLineStream << " rss_mb=n/a";
                }
                const std::string latencyLine = formatMetricLine(latencyLineStream.str());

                std::ostringstream depthLineStream;
                depthLineStream << "qmax frame=" << frameDepthMax
                                << " preinfer=" << preInferDepthMax
                                << " inferpost=" << inferPostDepthMax
                                << " output=" << outputDepthMax << " vo=" << voDepthMax
                                << " dump=" << dumpDepthMax;
                const std::string depthLine = formatMetricLine(depthLineStream.str());

                std::vector<std::string> dumpLogSnapshot;
                {
                    std::lock_guard<std::mutex> dumpLock(dumpLogMutex);
                    dumpLogSnapshot.assign(dumpLogs.begin(), dumpLogs.end());
                }

                std::vector<std::string> dashboardLines;
                dashboardLines.push_back(staticTitleLine);
                dashboardLines.push_back(dynamicTitleLine);
                dashboardLines.push_back(splitLine);
                dashboardLines.push_back(headerLine);
                dashboardLines.push_back(splitLine);
                dashboardLines.push_back(capLine);
                dashboardLines.push_back(preLine);
                dashboardLines.push_back(inferLine);
                dashboardLines.push_back(postLine);
                dashboardLines.push_back(pubLine);
                dashboardLines.push_back(voLine);
                dashboardLines.push_back(dumpLine);
                dashboardLines.push_back(latencyLine);
                dashboardLines.push_back(depthLine);
                dashboardLines.push_back(splitLine);

                std::lock_guard<std::mutex> lock(perfLogMutex);
                if (perfTraceTty) {
                    if (!cursorHidden) {
                        std::cout << "\033[?25l";
                        cursorHidden = true;
                    }

                    if (!firstRender && renderedDashboardLineCount > 0) {
                        std::cout << "\033[" << renderedDashboardLineCount << "F";
                        for (size_t i = 0; i < renderedDashboardLineCount; ++i) {
                            std::cout << "\r\033[2K";
                            if (i + 1 < renderedDashboardLineCount) {
                                std::cout << "\n";
                            }
                        }
                        if (renderedDashboardLineCount > 1) {
                            std::cout << "\033[" << (renderedDashboardLineCount - 1) << "F";
                        }
                        std::cout << "\r";
                    }

                    if (dumpLogSnapshot.size() > renderedDumpCount) {
                        for (size_t i = renderedDumpCount; i < dumpLogSnapshot.size(); ++i) {
                            std::cout << "\r\033[2K" << dumpLogSnapshot[i] << "\n";
                        }
                        renderedDumpCount = dumpLogSnapshot.size();
                    }

                    for (const auto& line : dashboardLines) {
                        std::cout << "\r\033[2K" << line << "\n";
                    }
                    renderedDashboardLineCount = dashboardLines.size();
                    std::cout << std::flush;
                } else {
                    if (dumpLogSnapshot.size() > renderedDumpCount) {
                        for (size_t i = renderedDumpCount; i < dumpLogSnapshot.size(); ++i) {
                            std::cout << dumpLogSnapshot[i] << std::endl;
                        }
                        renderedDumpCount = dumpLogSnapshot.size();
                    }
                    for (const auto& line : dashboardLines) {
                        std::cout << line << std::endl;
                    }
                }

                firstRender = false;
                lastReportTime = now;
            }

            std::lock_guard<std::mutex> lock(perfLogMutex);
            if (perfTraceTty && cursorHidden) {
                std::cout << "\033[?25h" << std::flush;
            }
        });
    }

    std::atomic<bool> recording{false};
    std::atomic<bool> singleFrameDumpRequested{false};
    std::atomic<uint64_t> processedOutputFrameCount{0};
    std::atomic<bool> inputThreadStop{false};
    std::thread inputThread;
    struct termios originalStdinTermios {};
    bool stdinRawModeEnabled = false;
    const bool stdinIsTty = (::isatty(STDIN_FILENO) != 0);

    if (stdinIsTty) {
        if (::tcgetattr(STDIN_FILENO, &originalStdinTermios) == 0) {
            struct termios raw = originalStdinTermios;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                stdinRawModeEnabled = true;
                inputThread = std::thread([&]() {
                    while (running && !inputThreadStop.load()) {
                        fd_set readfds;
                        FD_ZERO(&readfds);
                        FD_SET(STDIN_FILENO, &readfds);
                        struct timeval timeout;
                        timeout.tv_sec = 0;
                        timeout.tv_usec = 200000;

                        const int rv =
                            ::select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
                        if (rv <= 0 || !FD_ISSET(STDIN_FILENO, &readfds)) {
                            continue;
                        }

                        const KeyboardInputAction action = readKeyboardInputAction();
                        if (action == KeyboardInputAction::None) {
                            continue;
                        }

                        if (action == KeyboardInputAction::ToggleRecord) {
                            const bool prev = recording.load(std::memory_order_acquire);
                            if (!prev && processedOutputFrameCount.load(std::memory_order_acquire) <
                                             kMinReadyFramesBeforeCaptureActions) {
                                std::ostringstream oss;
                                oss << "recording start ignored: pipeline is still warming up ("
                                    << processedOutputFrameCount.load(std::memory_order_acquire)
                                    << '/' << kMinReadyFramesBeforeCaptureActions
                                    << " ready frames)";
                                appendRecordLog(oss.str());
                                continue;
                            }
                            recording.store(!prev, std::memory_order_release);
                        } else if (action == KeyboardInputAction::SingleFrameDump) {
                            if (processedOutputFrameCount.load(std::memory_order_acquire) <
                                kMinReadyFramesBeforeCaptureActions) {
                                std::ostringstream oss;
                                oss << "single-frame dump ignored: pipeline is still warming up "
                                       "("
                                    << processedOutputFrameCount.load(std::memory_order_acquire)
                                    << '/' << kMinReadyFramesBeforeCaptureActions
                                    << " ready frames)";
                                appendSingleDumpLog(oss.str());
                                continue;
                            }
                            singleFrameDumpRequested.store(true, std::memory_order_release);
                        } else if (action == KeyboardInputAction::PrevImportedFrame ||
                                   action == KeyboardInputAction::NextImportedFrame) {
                            if (!m_inputSource.supportsImportedFrameSelection()) {
                                continue;
                            }

                            FrameInputSource::ImportedFrameSelection selection;
                            const bool switched = m_inputSource.stepImportedFrame(
                                action == KeyboardInputAction::NextImportedFrame ? 1 : -1,
                                selection);
                            std::ostringstream oss;
                            if (switched) {
                                oss << "imported " << m_inputSource.importedFrameSourceName()
                                    << " frame selected: " << (selection.index + 1) << '/'
                                    << selection.count;
                                if (selection.frameTimestampNs != 0) {
                                    oss << ", frame_ts=" << selection.frameTimestampNs;
                                }
                            } else {
                                selection = m_inputSource.importedFrameSelection();
                                oss << "imported " << m_inputSource.importedFrameSourceName()
                                    << " frame stays at " << (selection.index + 1) << '/'
                                    << selection.count;
                            }
                            appendRuntimeLog(RuntimeLogKind::Notice, oss.str());
                        }
                    }
                });
            } else {
                ALOGW("failed to set stdin raw mode, keyboard dump is disabled");
            }
        } else {
            ALOGW("failed to get stdin termios, keyboard dump is disabled");
        }
    } else {
        ALOGW("stdin is not a TTY, keyboard dump is disabled");
    }

    std::mutex frameQueueMutex;
    std::condition_variable frameQueueNotEmptyCv;
    std::condition_variable frameQueueNotFullCv;
    std::deque<FramePacket> frameQueue;

    std::mutex outputQueueMutex;
    std::condition_variable outputQueueNotEmptyCv;
    std::condition_variable outputQueueNotFullCv;
    std::deque<OutputPacket> outputQueue;

    std::mutex voQueueMutex;
    std::condition_variable voQueueNotEmptyCv;
    std::deque<VoPacket> voQueue;

    std::mutex dumpQueueMutex;
    std::condition_variable dumpQueueNotEmptyCv;
    std::deque<DumpTask> dumpQueue;
    std::atomic<bool> dumpThreadStop{false};

    std::atomic<H264RecordState> h264RecordState{H264RecordState::Disabled};
    stereo_depth::axera_pipeline::AxVencH264Encoder h264Encoder;
    const uint32_t h264Fps =
        std::max<uint32_t>(1, static_cast<uint32_t>(std::max(inputInfo.fps, 1)));
    const bool h264Ready =
        h264Encoder.start(static_cast<uint32_t>(inputInfo.width),
                          static_cast<uint32_t>(inputInfo.height), h264Fps, appendSingleDumpLog);

    if (!h264Ready) {
        h264RecordState.store(H264RecordState::Failed, std::memory_order_release);
        appendSingleDumpLog(
            "H.264 VENC pre-initialization failed; continuous H.264 recording is unavailable");
    } else {
        std::ostringstream oss;
        oss << "H.264 VENC pre-initialized: " << inputInfo.width << 'x' << inputInfo.height << " @ "
            << h264Fps << "fps";
        appendSingleDumpLog(oss.str());
    }

    auto enqueueDumpTask = [&](DumpTask&& task) {
        {
            std::lock_guard<std::mutex> lock(dumpQueueMutex);
            if (dumpQueue.size() >= kRecordQueueCapacity) {
                dumpQueue.pop_front();
                droppedDumpQueue.fetch_add(1);
            }
            dumpQueue.push_back(std::move(task));
            updateMaxDepth(maxDumpQueueDepth, dumpQueue.size());
        }
        dumpQueueNotEmptyCv.notify_one();
    };

    std::mutex preInferQueueMutex;
    std::condition_variable preInferQueueNotEmptyCv;
    std::condition_variable preInferQueueNotFullCv;
    std::deque<ProcessStagePacket> preInferQueue;

    std::mutex inferPostQueueMutex;
    std::condition_variable inferPostQueueNotEmptyCv;
    std::condition_variable inferPostQueueNotFullCv;
    std::deque<ProcessStagePacket> inferPostQueue;

    std::atomic<bool> captureDone{false};
    std::atomic<bool> preprocessDone{false};
    std::atomic<bool> inferDone{false};
    std::atomic<bool> processDone{false};
    std::atomic<bool> publishDone{false};
    std::atomic<int> asyncError{0};
    LaserRangefinder rangefinder;

    auto releasePendingStagePackets = [&](std::deque<ProcessStagePacket>& queue,
                                          std::mutex& queueMutex) {
        std::deque<ProcessStagePacket> pendingPackets;
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            pendingPackets.swap(queue);
        }

        for (auto& packet : pendingPackets) {
            if (packet.context != nullptr) {
                AX_STEREO_ReleaseFrame(m_hPipeline, packet.context);
                packet.context = nullptr;
            }
        }
    };

    auto clearPendingQueue = [&](auto& queue, std::mutex& queueMutex) {
        using QueueType = std::decay_t<decltype(queue)>;
        QueueType emptyQueue;
        std::lock_guard<std::mutex> lock(queueMutex);
        queue.swap(emptyQueue);
    };

    std::thread captureThread([&]() {
        const bool throttleImageFileInput =
            m_inputSource.mode() == stereo_depth::InputMode::ImageFile &&
            !m_inputSource.usesRecordedReplayTiming();
        while (running) {
            const auto capBegin = std::chrono::steady_clock::now();
            const void* frameData = nullptr;
            size_t frameSize = 0;
            uint64_t frameTimestampNs = 0;
            stereo_depth::FrameLaserDistance importedLaserDistance;
            stereo_depth::InputFrameFormat inputFrameFormat = stereo_depth::InputFrameFormat::Yuyv;
            const AX_VIDEO_FRAME_T* externalFrame = nullptr;
            std::shared_ptr<void> externalFrameOwner;
            if (!m_inputSource.getFrame(frameData, frameSize, frameTimestampNs,
                                        &importedLaserDistance, &inputFrameFormat, &externalFrame,
                                        &externalFrameOwner)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            FramePacket packet;
            packet.frameTimestampNs = frameTimestampNs;
            packet.captureSteadyNs = nowSteadyNs();
            if (m_inputSource.isMcapInput()) {
                packet.laserDistance.sampleTimestampNs = frameTimestampNs;
                packet.laserDistance.distanceMm = importedLaserDistance.distanceMm;
            } else {
                packet.laserDistance = rangefinder.readSample(frameTimestampNs);
            }
            packet.format = inputFrameFormat;
            if (externalFrame != nullptr && externalFrameOwner) {
                packet.externalFrame = *externalFrame;
                packet.externalFrameOwner = std::move(externalFrameOwner);
            } else {
                packet.data.resize(frameSize);
                std::memcpy(packet.data.data(), frameData, frameSize);
            }
            m_inputSource.releaseFrame();

            const auto capEnd = std::chrono::steady_clock::now();
            const uint64_t capUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(capEnd - capBegin).count());

            std::unique_lock<std::mutex> lock(frameQueueMutex);
            if (!running) {
                break;
            }

            if (frameQueue.size() >= kFrameQueueCapacity) {
                frameQueue.pop_front();
                droppedFrameQueue.fetch_add(1);
            }

            frameQueue.push_back(std::move(packet));
            const size_t qDepth = frameQueue.size();
            updateMaxDepth(maxFrameQueueDepth, qDepth);
            lock.unlock();
            frameQueueNotEmptyCv.notify_one();

            if (perfTrace) {
                std::lock_guard<std::mutex> lockPerf(perfAccumMutex);
                capPerf.frames++;
                capPerf.workUs += capUs;
                capPerf.qOut = qDepth;
            }

            if (throttleImageFileInput) {
                const auto frameDeadline = capBegin + kImageFileInputFrameInterval;
                const auto now = std::chrono::steady_clock::now();
                if (now < frameDeadline) {
                    std::this_thread::sleep_until(frameDeadline);
                }
            }
        }

        captureDone = true;
        frameQueueNotEmptyCv.notify_all();
    });

    std::thread preprocessThread([&]() {
        while (running) {
            FramePacket packet;
            size_t inQDepth = 0;
            uint64_t waitInUs = 0;
            {
                std::unique_lock<std::mutex> lock(frameQueueMutex);
                const auto waitBegin = std::chrono::steady_clock::now();
                frameQueueNotEmptyCv.wait(
                    lock, [&]() { return !running || !frameQueue.empty() || captureDone.load(); });
                const auto waitEnd = std::chrono::steady_clock::now();
                waitInUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(waitEnd - waitBegin)
                        .count());

                if (frameQueue.empty()) {
                    if (captureDone.load()) {
                        break;
                    }
                    continue;
                }

                packet = std::move(frameQueue.front());
                frameQueue.pop_front();
                inQDepth = frameQueue.size();
            }
            frameQueueNotFullCv.notify_one();

            ProcessStagePacket stagePacket;
            stagePacket.captureSteadyNs = packet.captureSteadyNs;
            stagePacket.laserDistance = packet.laserDistance;
            stagePacket.preWaitInUs = waitInUs;
            stagePacket.inQDepth = inQDepth;

            const auto preprocessBegin = std::chrono::steady_clock::now();
            const auto apiInputFormat = packet.format == stereo_depth::InputFrameFormat::Nv12
                                            ? AX_STEREO_INPUT_FORMAT_NV12
                                            : AX_STEREO_INPUT_FORMAT_YUYV;
            const int preprocessRet =
                (packet.format == stereo_depth::InputFrameFormat::Nv12 && packet.externalFrameOwner)
                    ? AX_STEREO_SendVideoFrameWithOwner(
                          m_hPipeline, &packet.externalFrame, apiInputFormat,
                          packet.frameTimestampNs, packet.externalFrameOwner, &stagePacket.context)
                    : AX_STEREO_SendFrameEx(m_hPipeline, packet.data.data(),
                                            static_cast<AX_U32>(packet.data.size()), apiInputFormat,
                                            packet.frameTimestampNs, &stagePacket.context);
            const auto preprocessEnd = std::chrono::steady_clock::now();
            stagePacket.preprocessUs =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                          preprocessEnd - preprocessBegin)
                                          .count());

            if (preprocessRet != 0) {
                asyncError = preprocessRet;
                running = false;
                notifyAll(frameQueueNotEmptyCv, frameQueueNotFullCv, preInferQueueNotEmptyCv,
                          preInferQueueNotFullCv, inferPostQueueNotEmptyCv, inferPostQueueNotFullCv,
                          outputQueueNotEmptyCv, outputQueueNotFullCv, voQueueNotEmptyCv);
                break;
            }

            ProcessStagePacket droppedStagePacket;
            bool hasDroppedStagePacket = false;
            std::unique_lock<std::mutex> lock(preInferQueueMutex);
            if (!running) {
                AX_STEREO_ReleaseFrame(m_hPipeline, stagePacket.context);
                break;
            }

            if (preInferQueue.size() >= kPreInferQueueCapacity) {
                droppedStagePacket = std::move(preInferQueue.front());
                preInferQueue.pop_front();
                hasDroppedStagePacket = true;
                droppedPreInferQueue.fetch_add(1);
            }

            preInferQueue.push_back(std::move(stagePacket));
            updateMaxDepth(maxPreInferQueueDepth, preInferQueue.size());
            lock.unlock();
            if (hasDroppedStagePacket) {
                AX_STEREO_ReleaseFrame(m_hPipeline, droppedStagePacket.context);
            }
            preInferQueueNotEmptyCv.notify_one();
        }

        preprocessDone = true;
        preInferQueueNotEmptyCv.notify_all();
    });

    std::thread inferThread([&]() {
        while (running) {
            ProcessStagePacket stagePacket;
            uint64_t waitInUs = 0;
            {
                std::unique_lock<std::mutex> lock(preInferQueueMutex);
                const auto waitBegin = std::chrono::steady_clock::now();
                preInferQueueNotEmptyCv.wait(lock, [&]() {
                    return !running || !preInferQueue.empty() || preprocessDone.load();
                });
                const auto waitEnd = std::chrono::steady_clock::now();
                waitInUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(waitEnd - waitBegin)
                        .count());

                if (preInferQueue.empty()) {
                    if (preprocessDone.load()) {
                        break;
                    }
                    continue;
                }

                stagePacket = std::move(preInferQueue.front());
                preInferQueue.pop_front();
            }
            stagePacket.inferWaitInUs = waitInUs;
            preInferQueueNotFullCv.notify_one();

            const auto inferBegin = std::chrono::steady_clock::now();
            const int inferRet = AX_STEREO_RunInference(m_hPipeline, stagePacket.context);
            const auto inferEnd = std::chrono::steady_clock::now();
            stagePacket.inferUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(inferEnd - inferBegin)
                    .count());

            if (inferRet != 0) {
                AX_STEREO_ReleaseFrame(m_hPipeline, stagePacket.context);
                asyncError = inferRet;
                running = false;
                notifyAll(frameQueueNotEmptyCv, frameQueueNotFullCv, preInferQueueNotEmptyCv,
                          preInferQueueNotFullCv, inferPostQueueNotEmptyCv, inferPostQueueNotFullCv,
                          outputQueueNotEmptyCv, outputQueueNotFullCv, voQueueNotEmptyCv);
                break;
            }

            std::unique_lock<std::mutex> lock(inferPostQueueMutex);
            inferPostQueueNotFullCv.wait(lock, [&]() {
                return !running || inferPostQueue.size() < kInferPostQueueCapacity;
            });

            if (!running) {
                AX_STEREO_ReleaseFrame(m_hPipeline, stagePacket.context);
                break;
            }

            inferPostQueue.push_back(std::move(stagePacket));
            updateMaxDepth(maxInferPostQueueDepth, inferPostQueue.size());
            lock.unlock();
            inferPostQueueNotEmptyCv.notify_one();
        }

        inferDone = true;
        inferPostQueueNotEmptyCv.notify_all();
    });

    std::thread postprocessThread([&]() {
        while (running) {
            ProcessStagePacket stagePacket;
            uint64_t postWaitInUs = 0;
            {
                std::unique_lock<std::mutex> lock(inferPostQueueMutex);
                const auto waitBegin = std::chrono::steady_clock::now();
                inferPostQueueNotEmptyCv.wait(lock, [&]() {
                    return !running || !inferPostQueue.empty() || inferDone.load();
                });
                const auto waitEnd = std::chrono::steady_clock::now();
                postWaitInUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(waitEnd - waitBegin)
                        .count());

                if (inferPostQueue.empty()) {
                    if (inferDone.load()) {
                        break;
                    }
                    continue;
                }

                stagePacket = std::move(inferPostQueue.front());
                inferPostQueue.pop_front();
            }
            inferPostQueueNotFullCv.notify_one();

            OutputPacket outPacket;
            outPacket.captureSteadyNs = stagePacket.captureSteadyNs;
            outPacket.laserDistance = stagePacket.laserDistance;
            outPacket.output = std::make_shared<StereoOutputOwner>();
            const auto postBegin = std::chrono::steady_clock::now();
            const int postRet = AX_STEREO_GetResult(m_hPipeline, stagePacket.context,
                                                    &outPacket.output->stereoOutput);
            const auto postEnd = std::chrono::steady_clock::now();
            const uint64_t postUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(postEnd - postBegin).count());
            AX_STEREO_ReleaseFrame(m_hPipeline, stagePacket.context);

            if (postRet != 0) {
                asyncError = postRet;
                running = false;
                notifyAll(frameQueueNotEmptyCv, frameQueueNotFullCv, preInferQueueNotEmptyCv,
                          preInferQueueNotFullCv, inferPostQueueNotEmptyCv, inferPostQueueNotFullCv,
                          outputQueueNotEmptyCv, outputQueueNotFullCv, voQueueNotEmptyCv);
                break;
            }
            processedOutputFrameCount.fetch_add(1, std::memory_order_acq_rel);

            std::unique_lock<std::mutex> lock(outputQueueMutex);
            outputQueueNotFullCv.wait(
                lock, [&]() { return !running || outputQueue.size() < kOutputQueueCapacity; });
            if (!running) {
                break;
            }

            outputQueue.push_back(std::move(outPacket));
            const size_t outQDepth = outputQueue.size();
            updateMaxDepth(maxOutputQueueDepth, outQDepth);
            lock.unlock();
            outputQueueNotEmptyCv.notify_one();

            if (perfTrace) {
                std::lock_guard<std::mutex> lockPerf(perfAccumMutex);
                procPerf.frames++;
                procPerf.waitInUs += stagePacket.preWaitInUs;
                procPerf.workUs += stagePacket.preprocessUs + stagePacket.inferUs + postUs;
                procPerf.preWaitInUs += stagePacket.preWaitInUs;
                procPerf.inferWaitInUs += stagePacket.inferWaitInUs;
                procPerf.postWaitInUs += postWaitInUs;
                procPerf.preUs += stagePacket.preprocessUs;
                procPerf.inferUs += stagePacket.inferUs;
                procPerf.postUs += postUs;
                procPerf.qIn = stagePacket.inQDepth;
                procPerf.qOut = outQDepth;
            }
        }

        processDone = true;
        outputQueueNotEmptyCv.notify_all();
    });

    std::thread dumpThread([&]() {
        uint64_t singleDumpH264Frames = 0;
        std::optional<foxglove::McapWriter> activeWriter;
        std::optional<foxglove::Context> activeContext;
        StereoDepthTopics::McapChannels activeChannels;
        std::string activePath;
        uint64_t recordedFrames = 0;
        uint64_t recordedH264Frames = 0;
        uint64_t recordingStartNs = 0;
        uint32_t activeSegmentIndex = 0;
        std::optional<foxglove::McapWriter> singleDumpWriter;
        std::optional<foxglove::Context> singleDumpContext;
        StereoDepthTopics::McapChannels singleDumpChannels;
        std::string singleDumpPath;
        uint64_t singleDumpFrames = 0;
        uint64_t singleDumpStartNs = 0;
        uint32_t singleDumpSegmentIndex = 0;

        auto openWriter =
            [&](const TopicFlags& flags, const std::string& fileTag,
                const StereoDepthTopics::LogFn& logFn, std::optional<foxglove::McapWriter>& writer,
                std::optional<foxglove::Context>& context,
                StereoDepthTopics::McapChannels& channels, std::string& path, uint64_t& startNs,
                uint32_t segmentIndex = 0, bool writeDeviceInfoOnOpen = true,
                bool includeCompressedVideo = false) -> bool {
            foxglove::McapWriterOptions mcapOptions;
            context = foxglove::Context::create();
            mcapOptions.context = *context;
            if (startNs == 0) {
                startNs =
                    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count());
            }
            buildSegmentedOutputPath(m_options.dumpMcapPrefix, fileTag, inputInfo.serialNumber,
                                     startNs, segmentIndex, ".mcap", path);
            mcapOptions.path = path;
            mcapOptions.compression = foxglove::McapCompression::None;
            auto mcapResult = foxglove::McapWriter::create(mcapOptions);
            if (!mcapResult.has_value()) {
                std::ostringstream oss;
                oss << "recording failed: create writer " << foxglove::strerror(mcapResult.error());
                logFn(oss.str());
                topics.resetMcapChannels(channels);
                context.reset();
                path.clear();
                return false;
            }

            writer.emplace(std::move(mcapResult.value()));
            if (!topics.createMcapChannels(*context, flags, channels, logFn,
                                           includeCompressedVideo)) {
                topics.resetMcapChannels(channels);
                const auto closeErr = writer->close();
                if (closeErr != foxglove::FoxgloveError::Ok) {
                    std::ostringstream oss;
                    oss << "recording close failed: " << foxglove::strerror(closeErr)
                        << " path=" << path;
                    logFn(oss.str());
                }
                writer.reset();
                context.reset();
                path.clear();
                return false;
            }

            if (writeDeviceInfoOnOpen) {
                topics.logDeviceInfoToMcap(channels, inputInfo, startNs, logFn);
            }
            return true;
        };

        auto closeWriterSegment =
            [&](const char* label, std::optional<foxglove::McapWriter>& writer,
                std::optional<foxglove::Context>& context,
                StereoDepthTopics::McapChannels& channels, const std::string& path,
                const StereoDepthTopics::LogFn& logFn) {
                topics.resetMcapChannels(channels);
                if (!writer.has_value()) {
                    context.reset();
                    return;
                }

                const auto closeErr = writer->close();
                if (closeErr != foxglove::FoxgloveError::Ok) {
                    if (logFn) {
                        std::ostringstream oss;
                        oss << label << " close failed: " << foxglove::strerror(closeErr)
                            << " path=" << path;
                        logFn(oss.str());
                    }
                }

                writer.reset();
                context.reset();
            };

        auto rotateWriterIfNeeded =
            [&](const char* label, const TopicFlags& flags, const std::string& fileTag,
                const StereoDepthTopics::LogFn& logFn, std::optional<foxglove::McapWriter>& writer,
                std::optional<foxglove::Context>& context,
                StereoDepthTopics::McapChannels& channels, std::string& path, uint64_t& startNs,
                uint32_t& segmentIndex, bool writeDeviceInfoOnOpen,
                bool includeCompressedVideo) -> bool {
            if (!writer.has_value()) {
                return true;
            }

            const uintmax_t fileSizeBytes = getFileSizeBytes(path);
            if (fileSizeBytes < kMaxMcapFileSizeBytes) {
                return true;
            }

            const std::string oldPath = path;
            closeWriterSegment(label, writer, context, channels, oldPath, logFn);
            path.clear();
            ++segmentIndex;

            if (!openWriter(flags, fileTag, logFn, writer, context, channels, path, startNs,
                            segmentIndex, writeDeviceInfoOnOpen, includeCompressedVideo)) {
                return false;
            }

            if (logFn) {
                std::ostringstream oss;
                oss << label << " rolled over: " << oldPath << " -> " << path << " after reaching "
                    << formatByteSize(fileSizeBytes) << " (" << fileSizeBytes << " bytes)";
                logFn(oss.str());
            }
            return true;
        };

        auto closeActiveRecording = [&]() {
            if (!activeWriter.has_value()) {
                topics.resetMcapChannels(activeChannels);
                return;
            }

            const uint64_t recordingEndNs =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count());
            topics.resetMcapChannels(activeChannels);
            const auto closeErr = activeWriter->close();
            if (closeErr != foxglove::FoxgloveError::Ok) {
                std::ostringstream oss;
                oss << "recording close failed: " << foxglove::strerror(closeErr)
                    << " path=" << activePath;
                appendRecordLog(oss.str());
            } else {
                std::ostringstream oss;
                oss << "recording stopped: " << activePath;
                appendRecordLog(oss.str());

                const uintmax_t fileSizeBytes = getFileSizeBytes(activePath);

                const uint64_t durationNs =
                    recordingEndNs >= recordingStartNs ? (recordingEndNs - recordingStartNs) : 0;
                const double durationSec = static_cast<double>(durationNs) / 1000000000.0;
                const double fps =
                    durationSec > 0.0 ? static_cast<double>(recordedFrames) / durationSec : 0.0;

                std::ostringstream stats;
                stats << "recording summary: start=" << formatWallClockNs(recordingStartNs)
                      << ", end=" << formatWallClockNs(recordingEndNs)
                      << ", duration=" << formatDurationNs(durationNs)
                      << ", frames=" << recordedFrames << ", h264_frames=" << recordedH264Frames
                      << ", fps=" << std::fixed << std::setprecision(2) << fps
                      << ", size=" << formatByteSize(fileSizeBytes) << " (" << fileSizeBytes
                      << " bytes)";
                appendRecordLog(stats.str());
            }

            activeWriter.reset();
            activeContext.reset();
            activePath.clear();
            recordedFrames = 0;
            recordedH264Frames = 0;
            recordingStartNs = 0;
            activeSegmentIndex = 0;
        };

        auto closeSingleFrameDump = [&]() {
            h264Encoder.stop(appendSingleDumpLog);
            if (!singleDumpWriter.has_value()) {
                topics.resetMcapChannels(singleDumpChannels);
                return;
            }

            const uint64_t singleDumpEndNs =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count());
            topics.resetMcapChannels(singleDumpChannels);
            const auto closeErr = singleDumpWriter->close();
            if (closeErr != foxglove::FoxgloveError::Ok) {
                std::ostringstream oss;
                oss << "single-frame dump close failed: " << foxglove::strerror(closeErr)
                    << " path=" << singleDumpPath;
                appendSingleDumpLog(oss.str());
            } else {
                const uintmax_t fileSizeBytes = getFileSizeBytes(singleDumpPath);

                const uint64_t durationNs = singleDumpEndNs >= singleDumpStartNs
                                                ? (singleDumpEndNs - singleDumpStartNs)
                                                : 0;

                std::ostringstream oss;
                oss << "single-frame dump closed: " << singleDumpPath
                    << ", start=" << formatWallClockNs(singleDumpStartNs)
                    << ", end=" << formatWallClockNs(singleDumpEndNs)
                    << ", duration=" << formatDurationNs(durationNs)
                    << ", frames=" << singleDumpFrames << ", h264_frames=" << singleDumpH264Frames
                    << ", size=" << formatByteSize(fileSizeBytes) << " (" << fileSizeBytes
                    << " bytes)";
                appendSingleDumpLog(oss.str());
            }

            singleDumpWriter.reset();
            singleDumpContext.reset();
            singleDumpPath.clear();
            singleDumpFrames = 0;
            singleDumpStartNs = 0;
            singleDumpH264Frames = 0;
            singleDumpSegmentIndex = 0;
        };

        auto encodeAndLogH264 = [&](const DumpTask& task, bool writeSingleDump,
                                    bool writeActiveRecording) -> bool {
            if ((!writeSingleDump && !writeActiveRecording) || !h264Encoder.isActive() ||
                !task.output) {
                return false;
            }

            const auto* outputData = task.output->data();
            if (outputData == nullptr || outputData->rawYuyvData == nullptr ||
                outputData->rawYuyvData->empty()) {
                return false;
            }

            stereo_depth::axera_pipeline::EncodedH264Frame encodedFrame;
            if (!h264Encoder.encodeFrame(*outputData->rawYuyvData, outputData->rawWidth,
                                         outputData->rawHeight, outputData->frameTimestampNs,
                                         encodedFrame, appendRecordLog)) {
                return false;
            }

            if (writeSingleDump && singleDumpWriter.has_value()) {
                topics.logCompressedVideoToMcap(singleDumpChannels, encodedFrame.payload,
                                                encodedFrame.timestampNs, appendSingleDumpLog);
                ++singleDumpH264Frames;
            }
            if (writeActiveRecording && activeWriter.has_value()) {
                topics.logCompressedVideoToMcap(activeChannels, encodedFrame.payload,
                                                encodedFrame.timestampNs, appendRecordLog);
                ++recordedH264Frames;
            }

            return true;
        };

        while (true) {
            DumpTask task;
            uint64_t waitInUs = 0;
            {
                std::unique_lock<std::mutex> lock(dumpQueueMutex);
                const auto waitBegin = std::chrono::steady_clock::now();
                dumpQueueNotEmptyCv.wait(
                    lock, [&]() { return dumpThreadStop.load() || !dumpQueue.empty(); });
                const auto waitEnd = std::chrono::steady_clock::now();
                waitInUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(waitEnd - waitBegin)
                        .count());

                if (dumpQueue.empty()) {
                    if (dumpThreadStop.load()) {
                        break;
                    }
                    continue;
                }

                task = std::move(dumpQueue.front());
                dumpQueue.pop_front();
            }

            const auto dumpBegin = std::chrono::steady_clock::now();
            if (!task.output) {
                closeActiveRecording();
                continue;
            }

            if (task.singleFrame) {
                if (!singleDumpWriter.has_value()) {
                    const bool includeSingleDumpH264 = h264Encoder.isActive();
                    if (!openWriter(task.dataFlags, "single", appendSingleDumpLog, singleDumpWriter,
                                    singleDumpContext, singleDumpChannels, singleDumpPath,
                                    singleDumpStartNs, singleDumpSegmentIndex, false,
                                    includeSingleDumpH264)) {
                        continue;
                    }

                    singleDumpFrames = 0;
                    singleDumpH264Frames = 0;
                    appendSingleDumpLog(std::string("single-frame dump started: ") +
                                        singleDumpPath);
                    if (!includeSingleDumpH264) {
                        appendSingleDumpLog(
                            "single-frame dump started without H.264 because VENC is unavailable");
                    }
                } else if (!rotateWriterIfNeeded(
                               "single-frame dump", task.dataFlags, "single", appendSingleDumpLog,
                               singleDumpWriter, singleDumpContext, singleDumpChannels,
                               singleDumpPath, singleDumpStartNs, singleDumpSegmentIndex, false,
                               h264Encoder.isActive())) {
                    continue;
                }

                const auto roiZAvgPayload = makeRoiRegionAveragesJson(
                    *task.output->data(), AX_STEREO_CAMERA_FOCAL_LENGTH_PIXELS,
                    AX_STEREO_GetBaselineMeters(m_hPipeline), task.laserDistance);
                topics.logDeviceInfoToMcap(singleDumpChannels, inputInfo,
                                           task.output->data()->frameTimestampNs,
                                           appendSingleDumpLog);
                topics.logRoiZAvgToMcap(singleDumpChannels, roiZAvgPayload,
                                        task.output->data()->frameTimestampNs, appendSingleDumpLog);
                topics.logToMcap(singleDumpChannels, *task.output->data(), task.dataFlags,
                                 appendSingleDumpLog);
                ++singleDumpFrames;

                const uintmax_t fileSizeBytes = getFileSizeBytes(singleDumpPath);

                std::ostringstream oss;
                oss << "single-frame dump appended: " << singleDumpPath
                    << ", frame_ts=" << task.output->data()->frameTimestampNs
                    << ", total_frames=" << singleDumpFrames
                    << ", size=" << formatByteSize(fileSizeBytes) << " (" << fileSizeBytes
                    << " bytes)";
                appendSingleDumpLog(oss.str());
            }

            if (task.recordH264 && singleDumpWriter.has_value()) {
                if (!rotateWriterIfNeeded("single-frame dump", task.dataFlags, "single",
                                          appendSingleDumpLog, singleDumpWriter, singleDumpContext,
                                          singleDumpChannels, singleDumpPath, singleDumpStartNs,
                                          singleDumpSegmentIndex, false, h264Encoder.isActive())) {
                    continue;
                }
                if (h264RecordState.load(std::memory_order_acquire) == H264RecordState::Active &&
                    h264Encoder.isActive()) {
                    const bool encoded = encodeAndLogH264(task, true, activeWriter.has_value());
                    if (!encoded) {
                        h264RecordState.store(H264RecordState::Failed, std::memory_order_release);
                    }
                }
            }

            if (!task.singleFrame && !task.recordH264) {
                if (!activeWriter.has_value()) {
                    const bool includeActiveRecordingH264 = h264Encoder.isActive();
                    if (!openWriter(task.dataFlags, "", appendRecordLog, activeWriter,
                                    activeContext, activeChannels, activePath, recordingStartNs,
                                    activeSegmentIndex, true, includeActiveRecordingH264)) {
                        continue;
                    }

                    recordedFrames = 0;
                    recordedH264Frames = 0;
                    appendRecordLog(std::string("recording started: ") + activePath);
                    if (!includeActiveRecordingH264) {
                        appendRecordLog(
                            "recording started without H.264 because VENC is unavailable");
                    }
                } else if (!rotateWriterIfNeeded("recording", task.dataFlags, "", appendRecordLog,
                                                 activeWriter, activeContext, activeChannels,
                                                 activePath, recordingStartNs, activeSegmentIndex,
                                                 true, h264Encoder.isActive())) {
                    continue;
                }

                const auto roiZAvgPayload = makeRoiRegionAveragesJson(
                    *task.output->data(), AX_STEREO_CAMERA_FOCAL_LENGTH_PIXELS,
                    AX_STEREO_GetBaselineMeters(m_hPipeline), task.laserDistance);
                topics.logRoiZAvgToMcap(activeChannels, roiZAvgPayload,
                                        task.output->data()->frameTimestampNs, appendRecordLog);
                topics.logToMcap(activeChannels, *task.output->data(), task.dataFlags,
                                 appendRecordLog);
                ++recordedFrames;

                if (h264RecordState.load(std::memory_order_acquire) != H264RecordState::Active &&
                    activeChannels.h264.has_value()) {
                    if (!encodeAndLogH264(task, false, true)) {
                        appendRecordLog(
                            "recording warning: failed to append H.264 frame to regular MCAP");
                    }
                }
            }

            const auto dumpEnd = std::chrono::steady_clock::now();
            const uint64_t dumpUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(dumpEnd - dumpBegin).count());
            if (perfTrace) {
                std::lock_guard<std::mutex> lockPerf(perfAccumMutex);
                dumpPerf.samples++;
                dumpPerf.frames++;
                dumpPerf.waitInUs += waitInUs;
                dumpPerf.workUs += dumpUs;
            }
        }

        closeActiveRecording();
        closeSingleFrameDump();
    });

    std::thread voThread([&]() {
        while (true) {
            VoPacket packet;
            uint64_t waitInUs = 0;
            {
                std::unique_lock<std::mutex> lock(voQueueMutex);
                const auto waitBegin = std::chrono::steady_clock::now();
                voQueueNotEmptyCv.wait(
                    lock, [&]() { return !running || !voQueue.empty() || publishDone.load(); });
                const auto waitEnd = std::chrono::steady_clock::now();
                waitInUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(waitEnd - waitBegin)
                        .count());

                if (voQueue.empty()) {
                    if (!running || publishDone.load()) {
                        break;
                    }
                    continue;
                }

                packet = std::move(voQueue.front());
                voQueue.pop_front();
            }

            const auto voBegin = std::chrono::steady_clock::now();
            const bool presented = voEnabled && packet.output && packet.output->data() &&
                                   voDisplayPtr->present(*packet.output->data());
            const auto voEnd = std::chrono::steady_clock::now();
            const uint64_t voUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(voEnd - voBegin).count());

            if (perfTrace) {
                std::lock_guard<std::mutex> lockPerf(perfAccumMutex);
                voPerf.samples++;
                voPerf.frames += presented ? 1 : 0;
                voPerf.waitInUs += waitInUs;
                voPerf.workUs += voUs;
            }
        }
    });

    std::thread publishThread([&]() {
        const bool enablePubLimiter = m_options.foxgloveFps > 0;
        const double tokensPerUs =
            enablePubLimiter ? static_cast<double>(m_options.foxgloveFps) / 1000000.0 : 0.0;
        double pubTokens = enablePubLimiter ? 1.0 : 0.0;
        const double maxPubTokens =
            enablePubLimiter ? std::max(1.0, static_cast<double>(m_options.foxgloveFps)) : 0.0;
        auto tokenClock = std::chrono::steady_clock::now();
        bool wasRecording = false;
        uint64_t lastDeviceInfoSubscribeGeneration =
            m_foxglove.getSubscribeGeneration(topics.deviceInfoTopic());

        while (true) {
            OutputPacket packet;
            size_t outQDepth = 0;
            {
                std::unique_lock<std::mutex> lock(outputQueueMutex);
                const auto waitBegin = std::chrono::steady_clock::now();
                outputQueueNotEmptyCv.wait(
                    lock, [&]() { return !running || !outputQueue.empty() || processDone.load(); });
                const auto waitEnd = std::chrono::steady_clock::now();
                const uint64_t waitInUs = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(waitEnd - waitBegin)
                        .count());

                if (outputQueue.empty()) {
                    if (processDone.load()) {
                        break;
                    }
                    continue;
                }

                packet = std::move(outputQueue.front());
                outputQueue.pop_front();
                outQDepth = outputQueue.size();

                if (perfTrace) {
                    std::lock_guard<std::mutex> lockPerf(perfAccumMutex);
                    pubPerf.waitInUs += waitInUs;
                    pubPerf.qIn = outQDepth;
                }
            }
            outputQueueNotFullCv.notify_one();

            if (voEnabled) {
                std::lock_guard<std::mutex> lock(voQueueMutex);
                if (voQueue.size() >= kVoQueueCapacity) {
                    voQueue.pop_front();
                    droppedVoQueue.fetch_add(1);
                }
                voQueue.push_back(VoPacket{packet.output});
                updateMaxDepth(maxVoQueueDepth, voQueue.size());
                voQueueNotEmptyCv.notify_one();
            }

            TopicFlags dataFlags = makeTopicFlags(*packet.output->data());
            const bool enableLiveRawYuyv =
                m_inputSource.mode() == stereo_depth::InputMode::ImageFile;
            dataFlags.rawYuyv = enableLiveRawYuyv &&
                                packet.output->data()->rawYuyvData != nullptr &&
                                !packet.output->data()->rawYuyvData->empty();
            const std::vector<std::string> publishTopics =
                topics.collectPublishTopics(dataFlags, true);
            const bool hasPublishCandidate = m_foxglove.hasAnySubscribers(publishTopics);
            const uint64_t deviceInfoSubscriberCount =
                m_foxglove.getSubscriberCount(topics.deviceInfoTopic());
            const uint64_t deviceInfoSubscribeGeneration =
                m_foxglove.getSubscribeGeneration(topics.deviceInfoTopic());
            const bool shouldPublishRoiZAvg = m_foxglove.hasSubscribers(topics.roiZAvgTopic());

            const bool shouldRepublishDeviceInfo =
                deviceInfoSubscriberCount > 0 &&
                deviceInfoSubscribeGeneration != lastDeviceInfoSubscribeGeneration;

            if (shouldRepublishDeviceInfo) {
                std::lock_guard<std::mutex> publishLock(foxglovePublishMutex);
                topics.publishDeviceInfoToFoxglove(m_foxglove, inputInfo);
            }
            lastDeviceInfoSubscribeGeneration = deviceInfoSubscribeGeneration;

            bool framePublished = false;
            bool frameDroppedByLimiter = false;
            const auto pubBegin = std::chrono::steady_clock::now();

            const bool recordingEnabled = recording.load(std::memory_order_acquire);
            const bool singleFrameRequested =
                singleFrameDumpRequested.exchange(false, std::memory_order_acq_rel);
            if (singleFrameRequested) {
                H264RecordState expected = H264RecordState::Disabled;
                if (h264RecordState.compare_exchange_strong(expected, H264RecordState::Active,
                                                            std::memory_order_acq_rel,
                                                            std::memory_order_acquire)) {
                    appendSingleDumpLog(
                        "first single-frame dump triggered continuous H.264 recording into the "
                        "same _single.mcap");
                    appendSingleDumpLog(std::string("single-frame dump H.264 stream started: ") +
                                        topics.h264Topic());
                } else if (expected == H264RecordState::Failed) {
                    appendSingleDumpLog(
                        "single-frame dump H.264 stream is unavailable because VENC initialization "
                        "failed at startup");
                }
            }
            std::shared_ptr<StereoOutputOwner> dumpOutput;
            auto makeDumpOutput = [&]() -> std::shared_ptr<StereoOutputOwner> {
                if (!dumpOutput) {
                    dumpOutput = packet.output;
                }
                return dumpOutput;
            };

            if (recordingEnabled) {
                DumpTask task;
                task.output = makeDumpOutput();
                task.laserDistance = packet.laserDistance;
                task.dataFlags = dataFlags;
                enqueueDumpTask(std::move(task));
                wasRecording = true;
            } else if (wasRecording) {
                {
                    std::lock_guard<std::mutex> lock(dumpQueueMutex);
                    DumpTask stopTask;
                    dumpQueue.push_back(std::move(stopTask));
                }
                dumpQueueNotEmptyCv.notify_one();
                wasRecording = false;
            }

            if (singleFrameRequested) {
                DumpTask singleTask;
                singleTask.output = makeDumpOutput();
                singleTask.laserDistance = packet.laserDistance;
                singleTask.dataFlags = dataFlags;
                singleTask.dataFlags.rawYuyv = singleTask.output->data()->rawYuyvData &&
                                               !singleTask.output->data()->rawYuyvData->empty();
                singleTask.singleFrame = true;
                singleTask.recordH264 =
                    h264RecordState.load(std::memory_order_acquire) == H264RecordState::Active;
                enqueueDumpTask(std::move(singleTask));
            } else if (h264RecordState.load(std::memory_order_acquire) == H264RecordState::Active) {
                DumpTask h264Task;
                h264Task.output = makeDumpOutput();
                h264Task.recordH264 = true;
                enqueueDumpTask(std::move(h264Task));
            }

            bool allowPublishThisFrame = true;
            if (hasPublishCandidate && enablePubLimiter) {
                const auto now = std::chrono::steady_clock::now();
                const auto elapsedUs =
                    std::chrono::duration_cast<std::chrono::microseconds>(now - tokenClock).count();
                tokenClock = now;
                if (elapsedUs > 0) {
                    pubTokens += static_cast<double>(elapsedUs) * tokensPerUs;
                    if (pubTokens > maxPubTokens) {
                        pubTokens = maxPubTokens;
                    }
                }

                if (pubTokens >= 1.0) {
                    pubTokens -= 1.0;
                } else {
                    allowPublishThisFrame = false;
                }
            }

            if (hasPublishCandidate && allowPublishThisFrame) {
                std::lock_guard<std::mutex> publishLock(foxglovePublishMutex);
                std::optional<std::vector<std::byte>> roiZAvgPayload;
                if (shouldPublishRoiZAvg) {
                    roiZAvgPayload.emplace(makeRoiRegionAveragesJson(
                        *packet.output->data(), AX_STEREO_CAMERA_FOCAL_LENGTH_PIXELS,
                        AX_STEREO_GetBaselineMeters(m_hPipeline), packet.laserDistance));
                }
                const auto [_topicCount, normalPublished] =
                    topics.publishToFoxglove(m_foxglove, *packet.output->data(), dataFlags,
                                             roiZAvgPayload ? &*roiZAvgPayload : nullptr);
                (void)_topicCount;
                framePublished = normalPublished;
            } else if (hasPublishCandidate) {
                frameDroppedByLimiter = true;
            }

            const auto pubEnd = std::chrono::steady_clock::now();
            const uint64_t publishUs = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(pubEnd - pubBegin).count());
            uint64_t e2eUs = 0;
            if (packet.captureSteadyNs != 0) {
                const uint64_t steadyNs = nowSteadyNs();
                if (steadyNs >= packet.captureSteadyNs) {
                    e2eUs = (steadyNs - packet.captureSteadyNs) / 1000ULL;
                }
            }

            if (perfTrace) {
                std::lock_guard<std::mutex> lockPerf(perfAccumMutex);
                pubPerf.samples++;
                pubPerf.frames += framePublished ? 1 : 0;
                pubPerf.dropped += frameDroppedByLimiter ? 1 : 0;
                pubPerf.workUs += publishUs;
                pubPerf.e2eUs += framePublished ? e2eUs : 0;
            }
        }

        publishDone = true;
        voQueueNotEmptyCv.notify_all();
    });

    while (running && asyncError.load() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    running = false;
    perfMonitorStop = true;
    inputThreadStop = true;
    dumpThreadStop = true;
    notifyAll(frameQueueNotEmptyCv, frameQueueNotFullCv, preInferQueueNotEmptyCv,
              preInferQueueNotFullCv, inferPostQueueNotEmptyCv, inferPostQueueNotFullCv,
              outputQueueNotEmptyCv, outputQueueNotFullCv, voQueueNotEmptyCv);

    captureThread.join();
    preprocessThread.join();
    inferThread.join();
    postprocessThread.join();
    publishThread.join();
    if (voThread.joinable()) {
        voThread.join();
    }
    dumpQueueNotEmptyCv.notify_all();
    if (dumpThread.joinable()) {
        dumpThread.join();
    }

    clearPendingQueue(frameQueue, frameQueueMutex);
    releasePendingStagePackets(preInferQueue, preInferQueueMutex);
    releasePendingStagePackets(inferPostQueue, inferPostQueueMutex);
    clearPendingQueue(outputQueue, outputQueueMutex);
    clearPendingQueue(voQueue, voQueueMutex);
    clearPendingQueue(dumpQueue, dumpQueueMutex);

    if (inputThread.joinable()) {
        inputThread.join();
    }
    if (stdinRawModeEnabled) {
        ::tcsetattr(STDIN_FILENO, TCSANOW, &originalStdinTermios);
    }

    if (perfTrace && perfMonitorThread.joinable()) {
        perfMonitorStop = true;
        perfMonitorThread.join();
    }

    if (voEnabled) {
        voDisplayPtr->stop();
    }

    if (asyncError.load() != 0) {
        ALOGE("runPipeline fail, ret %d", asyncError.load());
    }
    return asyncError.load();
}

}  // namespace stereo_depth::app
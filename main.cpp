#include "AppRuntime.hpp"
#include "BackendSelect.hpp"
#include "FoxgloveWrapper.hpp"
#include "FrameInputSource.hpp"
#include "StereoDepthPipeline.hpp"
#include "V4L2Capture.hpp"
#include "ax_stereo_depth_api.h"
#include "host_backend/HostPrecheck.hpp"
#include "host_backend/ImageProcBackend.hpp"

#define SAMPLE_LOG_TAG "APP"
#include "sample_log.h"

#include <getopt.h>

#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <climits>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

std::atomic<bool> running{true};

constexpr char kDefaultDumpMcapPrefix[] = "/tmp/stereo_depth_dump";
constexpr char kDefaultDumpMcapBaseName[] = "stereo_depth_dump";

void signalHandler(int sig) {
    if (sig == SIGINT) {
        running = false;
        return;
    }
}

struct AppOptions {
    stereo_depth::InputSourceOptions input;
    bool listUvcModes = false;
    bool listAllUvcControls = false;
    bool resetUvcControls = false;
    bool perfTrace = false;
    bool enableVo = true;
    std::string npuModelPath = AX_STEREO_GetDefaultModelPath();
    bool enableGdc = true;
    AX_STEREO_GDC_MESH_MODE_E gdcMeshMode = AX_STEREO_GDC_MESH_DYNAMIC_REUSE;
    uint32_t foxgloveFps = 15;
    size_t messageBacklogSize = 10;
    std::string dumpMcapPrefix = kDefaultDumpMcapPrefix;
    stereo_depth::Backend imgProcBackend = stereo_depth::Backend::Auto;
};

void printUsage(const char* progName) {
    ALOGN("Usage: %s [options]", progName);
    ALOGN("Options:");
    ALOGN("  -d <device>    UVC device (default: /dev/video0)");
    ALOGN("  -s <WxH>       Stereo resolution: 2560x720 or 3840x1080");
    ALOGN("  -w <width>     Stereo width (default: 2560; also supports 3840 for FHD)");
    ALOGN("  -h <height>    Stereo height (default: 720; also supports 1080 for FHD)");
    ALOGN("  -f <fps>       UVC fps (default: 15)");
    ALOGN("  -F <fps>       Foxglove publish max fps (default: 15, 0 means unlimited)");
    ALOGN("  -r <path>      MCAP record prefix or directory (default: /tmp/stereo_depth_dump)");
    ALOGN("  -l             List supported UVC resolutions and fps, then exit");
    ALOGN("  -i <file>      Use specified input .yuyv or .mcap file instead of UVC capture");
    ALOGN("  --mcap-stream <mode>            MCAP import source: yuyv | h264 (default: yuyv)");
    ALOGN("  -m <model>     NPU model path (default: %s)", AX_STEREO_GetDefaultModelPath());
    ALOGN(
        "  -g <gdc>       GDC mode: on | force | builtin | off (on=reuse/download ini, reuse "
        "mesh or generate if missing, force=always regenerate mesh, builtin=use built-in mesh, "
        "default: "
        "on)");
    ALOGN("  -q <depth>     Foxglove message backlog size per client (default: 10)");
    ALOGN("  -t             Enable performance trace logs");
    ALOGN("  --vo                           Enable VO preview window (default: on)");
    ALOGN("  --no-vo                        Disable the VO preview window");
    ALOGN(
        "  --imgproc <backend>            Image processing backend: host | axcl | auto (default: "
        "auto)");
    ALOGN(
        "  --uvc-list-all-controls          List all V4L2 controls exposed by the device, then "
        "exit");
    ALOGN(
        "  --uvc-reset-controls             Reset writable UVC controls to driver defaults, then "
        "exit");
    ALOGN("  --uvc-brightness <n>             Set V4L2_CID_BRIGHTNESS");
    ALOGN("  --uvc-contrast <n>               Set V4L2_CID_CONTRAST");
    ALOGN("  --uvc-saturation <n>             Set V4L2_CID_SATURATION");
    ALOGN("  --uvc-gamma <n>                  Set V4L2_CID_GAMMA");
    ALOGN("  --uvc-sharpness <n>              Set V4L2_CID_SHARPNESS");
    ALOGN("  --uvc-white-balance-auto <bool>  Set V4L2_CID_AUTO_WHITE_BALANCE");
    ALOGN("  --uvc-white-balance-temperature <n> Set V4L2_CID_WHITE_BALANCE_TEMPERATURE");
    ALOGN("  --uvc-power-line-frequency <mode> Set V4L2_CID_POWER_LINE_FREQUENCY: off|50|60");
    ALOGN("  --uvc-gain <n>                   Set V4L2_CID_GAIN");
}

bool parseBoolOption(const char* arg, bool& value) {
    if (arg == nullptr) {
        return false;
    }

    std::string text(arg);
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (text == "1" || text == "true" || text == "on" || text == "yes") {
        value = true;
        return true;
    }
    if (text == "0" || text == "false" || text == "off" || text == "no") {
        value = false;
        return true;
    }
    return false;
}

bool parseGdcOption(const char* arg, bool& enableGdc, AX_STEREO_GDC_MESH_MODE_E& meshMode) {
    if (arg == nullptr) {
        return false;
    }

    std::string text(arg);
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (text == "builtin") {
        enableGdc = true;
        meshMode = AX_STEREO_GDC_MESH_DEFAULT;
        return true;
    }
    if (text == "force") {
        enableGdc = true;
        meshMode = AX_STEREO_GDC_MESH_DYNAMIC_FORCE;
        return true;
    }
    if (parseBoolOption(arg, enableGdc)) {
        meshMode = enableGdc ? AX_STEREO_GDC_MESH_DYNAMIC_REUSE : AX_STEREO_GDC_MESH_DEFAULT;
        return true;
    }
    return false;
}

const char* gdcModeName(bool enableGdc, AX_STEREO_GDC_MESH_MODE_E meshMode) {
    if (!enableGdc) {
        return "off";
    }

    switch (meshMode) {
        case AX_STEREO_GDC_MESH_DEFAULT:
            return "builtin";
        case AX_STEREO_GDC_MESH_DYNAMIC_REUSE:
            return "on";
        case AX_STEREO_GDC_MESH_DYNAMIC_FORCE:
            return "force";
        default:
            return "unknown";
    }
}

bool parseIntOption(const char* arg, int32_t& value) {
    if (arg == nullptr || *arg == '\0') {
        return false;
    }

    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(arg, &end, 10);
    if (errno != 0 || end == arg || *end != '\0' || parsed < INT32_MIN || parsed > INT32_MAX) {
        return false;
    }

    value = static_cast<int32_t>(parsed);
    return true;
}

bool parsePowerLineFrequencyOption(const char* arg, int32_t& value) {
    if (arg == nullptr) {
        return false;
    }

    std::string text(arg);
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (text == "0" || text == "off" || text == "disabled") {
        value = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
        return true;
    }
    if (text == "1" || text == "50" || text == "50hz") {
        value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
        return true;
    }
    if (text == "2" || text == "60" || text == "60hz") {
        value = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
        return true;
    }

    return false;
}

bool parseResolutionOption(const char* arg, int& width, int& height) {
    if (arg == nullptr || arg[0] == '\0') {
        return false;
    }

    const std::string text(arg);
    const size_t separatorPos = text.find_first_of("xX");
    if (separatorPos == std::string::npos || separatorPos == 0 ||
        separatorPos == (text.size() - 1)) {
        return false;
    }

    const std::string widthStr = text.substr(0, separatorPos);
    const std::string heightStr = text.substr(separatorPos + 1);
    const auto isDigits = [](const std::string& value) {
        return !value.empty() && std::all_of(value.begin(), value.end(), [](unsigned char ch) {
            return std::isdigit(ch) != 0;
        });
    };

    if (!isDigits(widthStr) || !isDigits(heightStr)) {
        return false;
    }

    width = std::atoi(widthStr.c_str());
    height = std::atoi(heightStr.c_str());
    return width > 0 && height > 0;
}

bool parseMcapStreamOption(const char* arg, stereo_depth::McapImportStream& value) {
    if (arg == nullptr) {
        return false;
    }

    std::string text(arg);
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (text == "yuyv") {
        value = stereo_depth::McapImportStream::RawYuyv;
        return true;
    }
    if (text == "h264") {
        value = stereo_depth::McapImportStream::H264;
        return true;
    }
    return false;
}

bool isDirectoryPath(const std::string& path) {
    struct stat pathStat {};
    return ::stat(path.c_str(), &pathStat) == 0 && S_ISDIR(pathStat.st_mode);
}

std::string normalizeDumpMcapPrefix(const char* arg) {
    std::string value = arg != nullptr ? arg : "";
    if (value.empty()) {
        return kDefaultDumpMcapPrefix;
    }

    const bool endsWithSlash = !value.empty() && value.back() == '/';
    std::string normalized = value;
    while (normalized.size() > 1 && normalized.back() == '/') {
        normalized.pop_back();
    }

    if (endsWithSlash || isDirectoryPath(normalized)) {
        if (normalized == "/") {
            return std::string("/") + kDefaultDumpMcapBaseName;
        }
        return normalized + "/" + kDefaultDumpMcapBaseName;
    }

    return normalized;
}

bool parseArgs(int argc, char** argv, AppOptions& options) {
    options.input.width = AX_STEREO_DEFAULT_INPUT_WIDTH;
    options.input.height = AX_STEREO_DEFAULT_INPUT_HEIGHT;

    enum LongOptionId {
        kOptUvcBrightness = 1000,
        kOptUvcContrast,
        kOptUvcSaturation,
        kOptUvcGamma,
        kOptUvcSharpness,
        kOptUvcWhiteBalanceAuto,
        kOptUvcWhiteBalanceTemperature,
        kOptUvcPowerLineFrequency,
        kOptVoEnable,
        kOptVoDisable,
        kOptUvcListAllControls,
        kOptUvcResetControls,
        kOptUvcGain,
        kOptMcapStream,
        kOptImgProcBackend,
    };

    static const option longOptions[] = {
        {"uvc-brightness", required_argument, nullptr, kOptUvcBrightness},
        {"uvc-contrast", required_argument, nullptr, kOptUvcContrast},
        {"uvc-saturation", required_argument, nullptr, kOptUvcSaturation},
        {"uvc-gamma", required_argument, nullptr, kOptUvcGamma},
        {"uvc-sharpness", required_argument, nullptr, kOptUvcSharpness},
        {"uvc-white-balance-auto", required_argument, nullptr, kOptUvcWhiteBalanceAuto},
        {"uvc-white-balance-temperature", required_argument, nullptr,
         kOptUvcWhiteBalanceTemperature},
        {"uvc-power-line-frequency", required_argument, nullptr, kOptUvcPowerLineFrequency},
        {"vo", no_argument, nullptr, kOptVoEnable},
        {"no-vo", no_argument, nullptr, kOptVoDisable},
        {"uvc-list-all-controls", no_argument, nullptr, kOptUvcListAllControls},
        {"uvc-reset-controls", no_argument, nullptr, kOptUvcResetControls},
        {"uvc-gain", required_argument, nullptr, kOptUvcGain},
        {"mcap-stream", required_argument, nullptr, kOptMcapStream},
        {"imgproc", required_argument, nullptr, kOptImgProcBackend},
        {nullptr, 0, nullptr, 0},
    };

    int opt = 0;
    int longIndex = 0;
    while ((opt = getopt_long(argc, argv, "d:s:w:h:f:F:r:m:g:i:q:lt", longOptions, &longIndex)) !=
           -1) {
        switch (opt) {
            case 'd':
                options.input.device = optarg;
                break;
            case 's':
                if (!parseResolutionOption(optarg, options.input.width, options.input.height)) {
                    return false;
                }
                break;
            case 'w':
                options.input.width = std::atoi(optarg);
                break;
            case 'h':
                options.input.height = std::atoi(optarg);
                break;
            case 'f':
                options.input.fps = std::atoi(optarg);
                break;
            case 'F': {
                const long fps = std::strtol(optarg, nullptr, 10);
                if (fps < 0) {
                    return false;
                }
                options.foxgloveFps = static_cast<uint32_t>(fps);
                break;
            }
            case 'r':
                if (optarg[0] == '\0') {
                    return false;
                }
                options.dumpMcapPrefix = normalizeDumpMcapPrefix(optarg);
                break;
            case 'm':
                if (optarg[0] == '\0') {
                    return false;
                }
                options.npuModelPath = optarg;
                break;
            case 'g':
                if (!parseGdcOption(optarg, options.enableGdc, options.gdcMeshMode)) {
                    return false;
                }
                break;
            case 'i':
                options.input.useImageFile = true;
                options.input.imageFile = optarg;
                break;
            case 'q': {
                const long depth = std::strtol(optarg, nullptr, 10);
                if (depth < 0) {
                    return false;
                }
                options.messageBacklogSize = static_cast<size_t>(depth);
                break;
            }
            case 'l':
                options.listUvcModes = true;
                break;
            case 't':
                options.perfTrace = true;
                break;
            case kOptVoEnable:
                options.enableVo = true;
                break;
            case kOptVoDisable:
                options.enableVo = false;
                break;
            case kOptUvcBrightness: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.brightness = value;
                break;
            }
            case kOptUvcContrast: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.contrast = value;
                break;
            }
            case kOptUvcSaturation: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.saturation = value;
                break;
            }
            case kOptUvcGamma: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.gamma = value;
                break;
            }
            case kOptUvcSharpness: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.sharpness = value;
                break;
            }
            case kOptUvcWhiteBalanceAuto: {
                bool value = false;
                if (!parseBoolOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.autoWhiteBalance = value ? 1 : 0;
                break;
            }
            case kOptUvcWhiteBalanceTemperature: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.whiteBalanceTemperature = value;
                break;
            }
            case kOptUvcPowerLineFrequency: {
                int32_t value = 0;
                if (!parsePowerLineFrequencyOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.powerLineFrequency = value;
                break;
            }
            case kOptUvcListAllControls:
                options.listAllUvcControls = true;
                break;
            case kOptUvcResetControls:
                options.resetUvcControls = true;
                break;
            case kOptUvcGain: {
                int32_t value = 0;
                if (!parseIntOption(optarg, value)) {
                    return false;
                }
                options.input.uvcControls.gain = value;
                break;
            }
            case kOptMcapStream:
                if (!parseMcapStreamOption(optarg, options.input.mcapImportStream)) {
                    return false;
                }
                break;
            case kOptImgProcBackend:
                if (!stereo_depth::parseBackend(optarg ? optarg : "", options.imgProcBackend)) {
                    return false;
                }
                break;
            default:
                return false;
        }
    }

    return true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    AppOptions options;
    if (!parseArgs(argc, argv, options)) {
        printUsage(argv[0]);
        return 1;
    }

    if (options.listUvcModes) {
        return V4L2Capture::printSupportedModes(options.input.device) ? 0 : 1;
    }

    if (options.listAllUvcControls) {
        return V4L2Capture::printAllControls(options.input.device) ? 0 : 1;
    }

    if (options.resetUvcControls) {
        return V4L2Capture::resetAllControlsToDefault(options.input.device) ? 0 : 1;
    }

    // Refuse to run a second instance: the Axera card cannot be shared between
    // processes (the second one deadlocks).
    {
        std::string reason;
        if (!stereo_depth::host_backend::acquireSingleInstanceLock(reason)) {
            ALOGE("%s", reason.c_str());
            return 1;
        }
    }

    // Verify the AXCL host driver is loaded and axcl-smi works before touching
    // the card.
    {
        std::string reason;
        if (!stereo_depth::host_backend::checkAxclHostReady(reason)) {
            ALOGE("%s", reason.c_str());
            return 1;
        }
    }

    if (!options.input.useImageFile && !stereo_depth::isSupportedStereoInputResolution(
                                           options.input.width, options.input.height)) {
        ALOGE("unsupported stereo resolution: %dx%d (supported: 2560x720, 3840x1080)",
              options.input.width, options.input.height);
        printUsage(argv[0]);
        return 1;
    }

    stereo_depth::FrameInputSource inputSource(options.input);
    if (!inputSource.initialize()) {
        return 1;
    }

    // Select the image-processing backend (host CPU vs. AXCL IVPS). This is done
    // only after the input source is ready so a missing camera does not leave
    // the AXCL card half-initialized (which aborts at exit). It must run before
    // AX_STEREO_Create so the pipeline can pick up the AXCL dewarp path.
    stereo_depth::setImageProcBackend(options.imgProcBackend);

    /* Create stereo depth pipeline via AX API */
    AX_STEREO_ATTR_T stStereoAttr;
    std::memset(&stStereoAttr, 0, sizeof(stStereoAttr));
    stStereoAttr.bEnableGdc = options.enableGdc ? AX_TRUE : AX_FALSE;
    stStereoAttr.eGdcMeshMode = options.gdcMeshMode;
    stStereoAttr.bExportVoFrames = options.enableVo ? AX_TRUE : AX_FALSE;
    stStereoAttr.s32InputWidth = inputSource.info().width;
    stStereoAttr.s32InputHeight = inputSource.info().height;
    if (!options.npuModelPath.empty()) {
        std::strncpy(stStereoAttr.szNpuModelPath, options.npuModelPath.c_str(),
                     sizeof(stStereoAttr.szNpuModelPath) - 1);
    }
    if (!inputSource.info().serialNumber.empty()) {
        std::strncpy(stStereoAttr.szCameraSerialNumber, inputSource.info().serialNumber.c_str(),
                     sizeof(stStereoAttr.szCameraSerialNumber) - 1);
    }

    AX_STEREO_HANDLE hPipeline = AX_NULL;
    int ret = AX_STEREO_Create(&hPipeline, &stStereoAttr);
    if (ret != 0) {
        stereo_depth::shutdownImageProcBackend();
        return ret;
    }

    usleep(100);

    auto& foxglove = FoxgloveWrapper::getInstance();
    FoxgloveWrapper::Options foxgloveOptions;
    foxgloveOptions.port = 8765;
    foxgloveOptions.max_publish_fps = 0;
    foxgloveOptions.message_backlog_size = options.messageBacklogSize;
    if (!foxglove.start(foxgloveOptions)) {
        AX_STEREO_Destroy(hPipeline);
        stereo_depth::shutdownImageProcBackend();
        return 1;
    }
    ALOGN("Press Ctrl+C to stop. engine=%s gdc=%s model=%s",
          AX_STEREO_GetEngineName(AX_STEREO_ENGINE_NPU),
          gdcModeName(options.enableGdc, options.gdcMeshMode), options.npuModelPath.c_str());

    stereo_depth::app::RuntimeOptions runtimeOptions;
    runtimeOptions.perfTrace = options.perfTrace;
    runtimeOptions.foxgloveFps = options.foxgloveFps;
    runtimeOptions.enableVo = options.enableVo;
    runtimeOptions.dumpMcapPrefix = options.dumpMcapPrefix;
    runtimeOptions.imgProcBackend = options.imgProcBackend;

    stereo_depth::app::StereoDepthAppRuntime runtime(runtimeOptions, hPipeline, inputSource,
                                                     foxglove);
    ret = runtime.run(running);

    foxglove.stop();
    ALOGN("Visualization stopped");

    AX_STEREO_Destroy(hPipeline);
    stereo_depth::shutdownImageProcBackend();

    return ret;
}

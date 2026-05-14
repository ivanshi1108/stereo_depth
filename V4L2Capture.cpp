#include "V4L2Capture.hpp"

#include "StereoDepthPipeline.hpp"

#define SAMPLE_LOG_TAG "V4L2"
#include "sample_log.h"

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace {

bool queryControlRange(int fd, uint32_t id, struct v4l2_queryctrl& query);

int32_t normalizeControlValue(const struct v4l2_queryctrl& query, int32_t value) {
    int32_t normalized = value;
    if (normalized < query.minimum) {
        normalized = query.minimum;
    }
    if (normalized > query.maximum) {
        normalized = query.maximum;
    }

    if (query.step > 1) {
        const int32_t base = query.minimum;
        const int32_t offset = normalized - base;
        const int32_t steps = offset / query.step;
        normalized = base + steps * query.step;
        if (normalized > query.maximum) {
            normalized = query.maximum;
        }
    }

    return normalized;
}

const char* controlName(uint32_t id) {
    switch (id) {
        case V4L2_CID_BRIGHTNESS:
            return "brightness";
        case V4L2_CID_CONTRAST:
            return "contrast";
        case V4L2_CID_SATURATION:
            return "saturation";
        case V4L2_CID_GAMMA:
            return "gamma";
        case V4L2_CID_SHARPNESS:
            return "sharpness";
        case V4L2_CID_AUTO_WHITE_BALANCE:
            return "white_balance_auto";
        case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
            return "white_balance_temperature";
        case V4L2_CID_POWER_LINE_FREQUENCY:
            return "power_line_frequency";
        case V4L2_CID_GAIN:
            return "gain";
        default:
            return "unknown";
    }
}

bool isSampleExposedControl(uint32_t id) {
    switch (id) {
        case V4L2_CID_BRIGHTNESS:
        case V4L2_CID_CONTRAST:
        case V4L2_CID_SATURATION:
        case V4L2_CID_GAMMA:
        case V4L2_CID_SHARPNESS:
        case V4L2_CID_AUTO_WHITE_BALANCE:
        case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
        case V4L2_CID_POWER_LINE_FREQUENCY:
        case V4L2_CID_GAIN:
            return true;
        default:
            return false;
    }
}

bool isControlAdjustableNow(const struct v4l2_queryctrl& query) {
    return (query.flags &
            (V4L2_CTRL_FLAG_DISABLED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE)) == 0;
}

const char* controlTypeName(__u32 type) {
    switch (type) {
        case V4L2_CTRL_TYPE_INTEGER:
            return "int";
        case V4L2_CTRL_TYPE_BOOLEAN:
            return "bool";
        case V4L2_CTRL_TYPE_MENU:
            return "menu";
        case V4L2_CTRL_TYPE_BUTTON:
            return "button";
        case V4L2_CTRL_TYPE_INTEGER64:
            return "int64";
        case V4L2_CTRL_TYPE_CTRL_CLASS:
            return "class";
        case V4L2_CTRL_TYPE_STRING:
            return "string";
        case V4L2_CTRL_TYPE_BITMASK:
            return "bitmask";
        case V4L2_CTRL_TYPE_INTEGER_MENU:
            return "int_menu";
        default:
            return "other";
    }
}

std::string controlFlagsString(__u32 flags) {
    std::string text;
    const auto append = [&](const char* name) {
        if (!text.empty()) {
            text += ",";
        }
        text += name;
    };

    if ((flags & V4L2_CTRL_FLAG_DISABLED) != 0) {
        append("disabled");
    }
    if ((flags & V4L2_CTRL_FLAG_READ_ONLY) != 0) {
        append("readonly");
    }
    if ((flags & V4L2_CTRL_FLAG_VOLATILE) != 0) {
        append("volatile");
    }
    if ((flags & V4L2_CTRL_FLAG_INACTIVE) != 0) {
        append("inactive");
    }
    if ((flags & V4L2_CTRL_FLAG_WRITE_ONLY) != 0) {
        append("writeonly");
    }
    if ((flags & V4L2_CTRL_FLAG_SLIDER) != 0) {
        append("slider");
    }

    return text.empty() ? "-" : text;
}

bool isResettableControl(const struct v4l2_queryctrl& query) {
    if ((query.flags & V4L2_CTRL_FLAG_DISABLED) != 0 ||
        (query.flags & V4L2_CTRL_FLAG_READ_ONLY) != 0) {
        return false;
    }

    switch (query.type) {
        case V4L2_CTRL_TYPE_INTEGER:
        case V4L2_CTRL_TYPE_BOOLEAN:
        case V4L2_CTRL_TYPE_MENU:
        case V4L2_CTRL_TYPE_INTEGER_MENU:
        case V4L2_CTRL_TYPE_BITMASK:
            return true;
        default:
            return false;
    }
}

bool queryControlRange(int fd, uint32_t id, struct v4l2_queryctrl& query) {
    std::memset(&query, 0, sizeof(query));
    query.id = id;
    if (::ioctl(fd, VIDIOC_QUERYCTRL, &query) != 0) {
        return false;
    }

    return (query.flags & V4L2_CTRL_FLAG_DISABLED) == 0;
}

void applyOptionalControl(int fd, const std::string& device, uint32_t id,
                          const std::optional<int32_t>& requestedValue) {
    if (!requestedValue.has_value()) {
        return;
    }

    struct v4l2_queryctrl query = {};
    if (!queryControlRange(fd, id, query)) {
        ALOGW("UVC control %s is not supported by %s", controlName(id), device.c_str());
        return;
    }
    if ((query.flags & V4L2_CTRL_FLAG_READ_ONLY) != 0) {
        ALOGW("UVC control %s is read-only on %s", controlName(id), device.c_str());
        return;
    }
    if ((query.flags & V4L2_CTRL_FLAG_INACTIVE) != 0) {
        ALOGW(
            "UVC control %s is currently inactive on %s and may be locked by another auto setting",
            controlName(id), device.c_str());
        return;
    }

    const int32_t requested = *requestedValue;
    int32_t finalValue = normalizeControlValue(query, requested);
    if (finalValue != requested) {
        ALOGW("UVC control %s=%d adjusted to %d within range [%d, %d] step %d on %s",
              controlName(id), requested, finalValue, query.minimum, query.maximum, query.step,
              device.c_str());
    }

    struct v4l2_control ctrl = {};
    ctrl.id = id;
    ctrl.value = finalValue;
    if (::ioctl(fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ALOGW("Failed to set UVC control %s=%d on %s: %s", controlName(id), finalValue,
              device.c_str(), strerror(errno));
        return;
    }

    ALOGN("Set UVC control %s=%d on %s (range [%d, %d], step %d, default %d)", controlName(id),
          finalValue, device.c_str(), query.minimum, query.maximum, query.step,
          query.default_value);
}

void applyUvcControls(int fd, const std::string& device,
                      const V4L2Capture::UvcControlSettings& controls) {
    applyOptionalControl(fd, device, V4L2_CID_AUTO_WHITE_BALANCE, controls.autoWhiteBalance);
    applyOptionalControl(fd, device, V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                         controls.whiteBalanceTemperature);
    applyOptionalControl(fd, device, V4L2_CID_POWER_LINE_FREQUENCY, controls.powerLineFrequency);
    applyOptionalControl(fd, device, V4L2_CID_BRIGHTNESS, controls.brightness);
    applyOptionalControl(fd, device, V4L2_CID_CONTRAST, controls.contrast);
    applyOptionalControl(fd, device, V4L2_CID_SATURATION, controls.saturation);
    applyOptionalControl(fd, device, V4L2_CID_GAMMA, controls.gamma);
    applyOptionalControl(fd, device, V4L2_CID_SHARPNESS, controls.sharpness);
    applyOptionalControl(fd, device, V4L2_CID_GAIN, controls.gain);
}

bool resetControlToDefault(int fd, uint32_t id) {
    struct v4l2_queryctrl query = {};
    if (!queryControlRange(fd, id, query)) {
        return true;
    }
    if (!isResettableControl(query)) {
        ALOGN("Skip reset control %s (id=0x%08x) because it is not writable", controlName(id), id);
        return true;
    }
    if ((query.flags & V4L2_CTRL_FLAG_INACTIVE) != 0) {
        ALOGN("Skip reset control %s (id=0x%08x) because it is currently inactive", controlName(id),
              id);
        return true;
    }

    struct v4l2_control ctrl = {};
    ctrl.id = id;
    ctrl.value = query.default_value;
    if (::ioctl(fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ALOGW("Failed to reset control %s (id=0x%08x) to default=%d: %s", controlName(id), id,
              query.default_value, strerror(errno));
        return false;
    }

    ALOGN("Reset control %s (id=0x%08x) to default=%d", controlName(id), id, query.default_value);
    return true;
}

std::string trimWhitespace(const std::string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }

    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string getBaseName(const std::string& path) {
    const size_t pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return path;
    }
    return path.substr(pos + 1);
}

std::string getParentPath(const std::string& path) {
    const size_t pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return "";
    }
    if (pos == 0) {
        return "/";
    }
    return path.substr(0, pos);
}

std::string readSingleLineFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return "";
    }

    std::string line;
    std::getline(file, line);
    return trimWhitespace(line);
}

std::string readUeventValue(const std::string& path, const std::string& key) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return "";
    }

    const std::string prefix = key + "=";
    std::string line;
    while (std::getline(file, line)) {
        line = trimWhitespace(line);
        if (line.rfind(prefix, 0) == 0) {
            return trimWhitespace(line.substr(prefix.size()));
        }
    }

    return "";
}

bool isDirectory(const std::string& path) {
    struct stat st = {};
    if (::stat(path.c_str(), &st) != 0) {
        return false;
    }
    return S_ISDIR(st.st_mode);
}

bool shouldSkipDirectoryName(const std::string& name) { return name == "." || name == ".."; }

bool isUsbDeviceNode(const std::string& path) {
    const std::string baseName = getBaseName(path);
    if (baseName.empty() || baseName.find(':') != std::string::npos) {
        return false;
    }

    return !readSingleLineFile(path + "/idVendor").empty() &&
           !readSingleLineFile(path + "/idProduct").empty();
}

std::string findUsbDeviceAncestor(const std::string& startPath) {
    std::string currentPath = startPath;
    while (!currentPath.empty() && currentPath != "/") {
        if (isUsbDeviceNode(currentPath)) {
            return currentPath;
        }
        currentPath = getParentPath(currentPath);
    }
    return "";
}

bool findUeventValueInSubtree(const std::string& rootPath, const std::string& key, int depth,
                              std::string& outValue, std::string& outPath) {
    if (depth < 0 || !isDirectory(rootPath)) {
        return false;
    }

    const std::string ueventPath = rootPath + "/uevent";
    const std::string value = readUeventValue(ueventPath, key);
    if (!value.empty()) {
        outValue = value;
        outPath = rootPath;
        return true;
    }

    DIR* dir = ::opendir(rootPath.c_str());
    if (dir == nullptr) {
        return false;
    }

    struct dirent* entry = nullptr;
    bool found = false;
    while ((entry = ::readdir(dir)) != nullptr) {
        const std::string name(entry->d_name);
        if (shouldSkipDirectoryName(name)) {
            continue;
        }

        const std::string childPath = rootPath + "/" + name;
        if (!isDirectory(childPath)) {
            continue;
        }

        if (findUeventValueInSubtree(childPath, key, depth - 1, outValue, outPath)) {
            found = true;
            break;
        }
    }

    ::closedir(dir);
    return found;
}

bool findAssociatedHidUniq(const std::string& startPath, std::string& outUniq,
                           std::string& outPath) {
    const std::string usbDevicePath = findUsbDeviceAncestor(startPath);
    if (usbDevicePath.empty()) {
        return false;
    }

    return findUeventValueInSubtree(usbDevicePath, "HID_UNIQ", 4, outUniq, outPath);
}

}  // namespace

V4L2Capture::DeviceInfo V4L2Capture::probeDeviceInfo(const std::string& device) {
    DeviceInfo info;
    info.devicePath = device;

    const std::string videoNode = getBaseName(device);
    if (videoNode.empty()) {
        return info;
    }

    const std::string sysfsPath = "/sys/class/video4linux/" + videoNode + "/device";
    char resolvedPath[PATH_MAX] = {};
    if (::realpath(sysfsPath.c_str(), resolvedPath) == nullptr) {
        return info;
    }

    std::string currentPath(resolvedPath);
    const std::string usbDevicePath = findUsbDeviceAncestor(currentPath);
    info.isUsb = !usbDevicePath.empty();

    std::string hidUniq;
    std::string hidPath;
    if (findAssociatedHidUniq(currentPath, hidUniq, hidPath)) {
        info.serialNumber = hidUniq;
        return info;
    }

    return info;
}

V4L2Capture::V4L2Capture(const std::string& device, int width, int height, int fps,
                         const UvcControlSettings& controls)
    : device_(device),
      width_(width),
      height_(height),
      fps_(fps),
      fd_(-1),
      is_started_(false),
      buffer_count_(0),
      frame_size_(0),
      controls_(controls) {
    std::memset(buffers_, 0, sizeof(buffers_));
    std::memset(buffer_lengths_, 0, sizeof(buffer_lengths_));
    device_info_ = probeDeviceInfo(device_);
}

V4L2Capture::~V4L2Capture() {
    stop();
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool V4L2Capture::printSupportedModes(const std::string& device) {
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGE("open %s failed: %s", device.c_str(), strerror(errno));
        return false;
    }

    const DeviceInfo deviceInfo = probeDeviceInfo(device);
    if (!deviceInfo.serialNumber.empty()) {
        ALOGN("SerialNumber: %s", deviceInfo.serialNumber.c_str());
    } else {
        ALOGN("SerialNumber: (not found)");
    }
    ALOGN("UVC capabilities for %s", device.c_str());
    ALOGN("Filtered to stereo_depth supported resolutions: 2560x720, 3840x1080");

    struct v4l2_fmtdesc fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (fmt.index = 0; ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
        char fourcc[5] = {static_cast<char>(fmt.pixelformat & 0xFF),
                          static_cast<char>((fmt.pixelformat >> 8) & 0xFF),
                          static_cast<char>((fmt.pixelformat >> 16) & 0xFF),
                          static_cast<char>((fmt.pixelformat >> 24) & 0xFF), '\0'};

        bool printedFormatHeader = false;

        struct v4l2_frmsizeenum fsize = {};
        fsize.pixel_format = fmt.pixelformat;
        for (fsize.index = 0; ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize) == 0; ++fsize.index) {
            if (fsize.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
                continue;
            }

            const uint32_t width = fsize.discrete.width;
            const uint32_t height = fsize.discrete.height;
            if (!stereo_depth::isSupportedStereoInputResolution(static_cast<int>(width),
                                                                static_cast<int>(height))) {
                continue;
            }

            if (!printedFormatHeader) {
                ALOGN("[format] %s - %s", fourcc, reinterpret_cast<const char*>(fmt.description));
                printedFormatHeader = true;
            }

            ALOGN("  [size] %ux%u", width, height);

            struct v4l2_frmivalenum fival = {};
            fival.pixel_format = fmt.pixelformat;
            fival.width = width;
            fival.height = height;

            bool foundFps = false;
            for (fival.index = 0; ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival) == 0;
                 ++fival.index) {
                if (fival.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
                    continue;
                }
                if (fival.discrete.numerator == 0) {
                    continue;
                }

                const double fps = static_cast<double>(fival.discrete.denominator) /
                                   static_cast<double>(fival.discrete.numerator);
                ALOGN("    fps: %.3f", fps);
                foundFps = true;
            }

            if (!foundFps) {
                ALOGN("    fps: (not reported by driver)");
            }
        }
    }

    close(fd);
    return true;
}

bool V4L2Capture::printAllControls(const std::string& device) {
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGE("open %s failed: %s", device.c_str(), strerror(errno));
        return false;
    }

    const DeviceInfo deviceInfo = probeDeviceInfo(device);
    if (!deviceInfo.serialNumber.empty()) {
        ALOGN("SerialNumber: %s", deviceInfo.serialNumber.c_str());
    } else {
        ALOGN("SerialNumber: (not found)");
    }

    ALOGN("All V4L2 controls for %s", device.c_str());

    struct v4l2_queryctrl query = {};
    query.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (::ioctl(fd, VIDIOC_QUERYCTRL, &query) == 0) {
        struct v4l2_control ctrl = {};
        ctrl.id = query.id;
        const bool gotCurrent = ((query.flags & V4L2_CTRL_FLAG_DISABLED) == 0) &&
                                (::ioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0);
        const bool sampleExposed = isSampleExposedControl(query.id);
        const bool adjustableNow = isControlAdjustableNow(query);

        ALOGN(
            "[control] id=0x%08x name=%s type=%s sample=%s adjustable_now=%s current=%s%d "
            "range=[%d,%d] step=%d default=%d flags=%s",
            query.id, reinterpret_cast<const char*>(query.name), controlTypeName(query.type),
            sampleExposed ? "yes" : "no", adjustableNow ? "yes" : "no", gotCurrent ? "" : "(n/a)",
            gotCurrent ? ctrl.value : 0, query.minimum, query.maximum, query.step,
            query.default_value, controlFlagsString(query.flags).c_str());

        query.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    close(fd);
    return true;
}

bool V4L2Capture::resetAllControlsToDefault(const std::string& device) {
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGE("open %s failed: %s", device.c_str(), strerror(errno));
        return false;
    }

    const DeviceInfo deviceInfo = probeDeviceInfo(device);
    if (!deviceInfo.serialNumber.empty()) {
        ALOGN("SerialNumber: %s", deviceInfo.serialNumber.c_str());
    } else {
        ALOGN("SerialNumber: (not found)");
    }

    ALOGN("Reset sample-supported UVC controls to driver defaults for %s", device.c_str());

    static const std::array<uint32_t, 9> kResetOrder = {
        V4L2_CID_AUTO_WHITE_BALANCE,
        V4L2_CID_WHITE_BALANCE_TEMPERATURE,
        V4L2_CID_POWER_LINE_FREQUENCY,
        V4L2_CID_BRIGHTNESS,
        V4L2_CID_CONTRAST,
        V4L2_CID_SATURATION,
        V4L2_CID_GAMMA,
        V4L2_CID_SHARPNESS,
        V4L2_CID_GAIN,
    };

    bool success = true;
    for (uint32_t id : kResetOrder) {
        if (!resetControlToDefault(fd, id)) {
            success = false;
        }
    }

    close(fd);
    return success;
}

size_t V4L2Capture::initialize() {
    if (fd_ >= 0) {
        return frame_size_;
    }

    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        ALOGE("open %s failed: %s", device_.c_str(), strerror(errno));
        return 0;
    }

    /* ---------- set format ---------- */
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        ALOGE("VIDIOC_S_FMT failed on %s: %s", device_.c_str(), strerror(errno));
        return 0;
    }

    // Update actual width/height/size from driver
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    frame_size_ = fmt.fmt.pix.sizeimage;

    /* ---------- set fps ---------- */
    struct v4l2_streamparm sp = {};
    sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sp.parm.capture.timeperframe.numerator = 1;
    sp.parm.capture.timeperframe.denominator = fps_;

    const int setParmRet = ioctl(fd_, VIDIOC_S_PARM, &sp);
    if (setParmRet < 0) {
        ALOGW("VIDIOC_S_PARM failed on %s: %s", device_.c_str(), strerror(errno));
    }

    struct v4l2_streamparm actualSp = {};
    actualSp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_G_PARM, &actualSp) == 0 &&
        actualSp.parm.capture.timeperframe.numerator != 0) {
        const double negotiatedFps =
            static_cast<double>(actualSp.parm.capture.timeperframe.denominator) /
            static_cast<double>(actualSp.parm.capture.timeperframe.numerator);
        if (negotiatedFps > 0.0) {
            fps_ = std::max(1, static_cast<int>(negotiatedFps + 0.5));
        }
    } else if (setParmRet == 0 && sp.parm.capture.timeperframe.numerator != 0) {
        const double negotiatedFps = static_cast<double>(sp.parm.capture.timeperframe.denominator) /
                                     static_cast<double>(sp.parm.capture.timeperframe.numerator);
        if (negotiatedFps > 0.0) {
            fps_ = std::max(1, static_cast<int>(negotiatedFps + 0.5));
        }
    }

    applyUvcControls(fd_, device_, controls_);

    return frame_size_;
}

bool V4L2Capture::start() {
    if (is_started_) return true;

    if (initialize() == 0) {
        return false;
    }

    /* ---------- request buffers ---------- */
    struct v4l2_requestbuffers req = {};
    req.count = MAX_BUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        ALOGE("VIDIOC_REQBUFS failed: %s", strerror(errno));
        return false;
    }

    if (req.count == 0 || req.count > MAX_BUFS) {
        ALOGE("unexpected MMAP buffer count %u on %s", req.count, device_.c_str());
        return false;
    }

    buffer_count_ = req.count;

    for (uint32_t i = 0; i < buffer_count_; i++) {
        struct v4l2_buffer buf = {};
        buf.type = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            ALOGE("VIDIOC_QUERYBUF failed for buffer %u: %s", i, strerror(errno));
            return false;
        }

        buffers_[i] =
            mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (buffers_[i] == MAP_FAILED) {
            buffers_[i] = nullptr;
            ALOGE("mmap failed for buffer %u: %s", i, strerror(errno));
            return false;
        }
        buffer_lengths_[i] = buf.length;

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            ALOGE("VIDIOC_QBUF failed for buffer %u: %s", i, strerror(errno));
            return false;
        }
    }

    enum v4l2_buf_type type = static_cast<enum v4l2_buf_type>(req.type);
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        ALOGE("VIDIOC_STREAMON failed: %s", strerror(errno));
        return false;
    }

    is_started_ = true;
    return true;
}

void V4L2Capture::stop() {
    if (!is_started_) return;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);

    is_started_ = false;

    for (uint32_t i = 0; i < buffer_count_; i++) {
        if (buffers_[i] != nullptr) {
            munmap(buffers_[i], buffer_lengths_[i]);
            buffers_[i] = nullptr;
        }
        buffer_lengths_[i] = 0;
    }
    buffer_count_ = 0;
}

bool V4L2Capture::grab(Frame& frame) {
    if (!is_started_) return false;

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return false;
        }
        ALOGE("VIDIOC_DQBUF failed: %s", strerror(errno));
        return false;
    }

    if (buf.index >= buffer_count_ || buffers_[buf.index] == nullptr) {
        ALOGE("VIDIOC_DQBUF returned invalid buffer index %u", buf.index);
        return false;
    }

    if (buf.bytesused == 0 || buf.bytesused < frame_size_) {
        ALOGW("Dropping incomplete UVC frame on %s: bytesused=%u expected=%zu index=%u",
              device_.c_str(), buf.bytesused, frame_size_, buf.index);

        struct v4l2_buffer requeue = {};
        requeue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        requeue.memory = V4L2_MEMORY_MMAP;
        requeue.index = buf.index;
        if (ioctl(fd_, VIDIOC_QBUF, &requeue) < 0) {
            ALOGE("VIDIOC_QBUF failed while dropping incomplete frame %u: %s", buf.index,
                  strerror(errno));
        }
        return false;
    }

    frame.data = buffers_[buf.index];
    frame.size = buf.bytesused;
    frame.index = buf.index;
    frame.timestampNs = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000000ULL +
                        static_cast<uint64_t>(buf.timestamp.tv_usec) * 1000ULL;

    return true;
}

void V4L2Capture::release(const Frame& frame) {
    if (!is_started_) return;

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = frame.index;

    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        ALOGE("VIDIOC_QBUF failed for buffer %d: %s", frame.index, strerror(errno));
    }
}

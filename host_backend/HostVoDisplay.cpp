// Host VO: render the 2x2 preview (left / raw input / disparity / depth-grid)
// to a desktop window via OpenCV highgui, matching the original board VO layout.

#include "AxVoDisplay.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define SAMPLE_LOG_TAG "VO"
#include "sample_log.h"

namespace stereo_depth::axera_pipeline {

namespace {

// Tile geometry matches the original 1920x1080 / 2x2 HDMI layout.
constexpr int kTileW = 960;
constexpr int kTileH = 540;
constexpr int kContentW = stereo_depth::kDewarpImageWidth;   // 640
constexpr int kContentH = stereo_depth::kDewarpImageHeight;  // 384

constexpr uint32_t kRoiBoxSizePx = 11;
constexpr uint32_t kDepthDisplayMax = 2048;
constexpr uint32_t kDisparityScale = 16;
constexpr float kDefaultBaselineMeters = 0.0629277f;
constexpr double kRoiTextScale = 0.45;
constexpr int kRoiTextThickness = 1;
constexpr const uint32_t kRoiX[3] = {180, 320, 460};
constexpr const uint32_t kRoiY[3] = {96, 192, 288};

struct RoiDepthValue {
    uint32_t centerX = 0;
    uint32_t centerY = 0;
    bool valid = false;
    float depthMeters = 0.0f;
    bool confidenceValid = false;
    float confidenceAvg = 0.0f;
};

// 3x3 ROI grid: average depth (from disparity) and confidence over an 11x11 box.
std::array<RoiDepthValue, 9> computeRoiDepthValues(const stereo_depth::PipelineOutput& output) {
    std::array<RoiDepthValue, 9> values = {};
    const size_t expectedBytes = static_cast<size_t>(kContentW) * kContentH * sizeof(uint16_t);
    if (output.depthData.size() < expectedBytes) {
        return values;
    }

    const auto* disparity = reinterpret_cast<const uint16_t*>(output.depthData.data());
    const size_t expectedConfidenceBytes =
        static_cast<size_t>(kContentW) * kContentH * sizeof(float);
    const auto* confidenceValues =
        output.confidenceData.size() >= expectedConfidenceBytes
            ? reinterpret_cast<const float*>(output.confidenceData.data())
            : nullptr;

    for (size_t i = 0; i < values.size(); ++i) {
        RoiDepthValue& value = values[i];
        value.centerX = kRoiX[i % 3];
        value.centerY = kRoiY[i / 3];

        const uint32_t xBegin =
            value.centerX > (kRoiBoxSizePx / 2) ? value.centerX - (kRoiBoxSizePx / 2) : 0;
        const uint32_t yBegin =
            value.centerY > (kRoiBoxSizePx / 2) ? value.centerY - (kRoiBoxSizePx / 2) : 0;
        const uint32_t xEnd =
            std::min<uint32_t>(kContentW - 1, value.centerX + (kRoiBoxSizePx / 2));
        const uint32_t yEnd =
            std::min<uint32_t>(kContentH - 1, value.centerY + (kRoiBoxSizePx / 2));

        double depthSum = 0.0;
        uint32_t sampleCount = 0;
        double confidenceSum = 0.0;
        uint32_t confidenceCount = 0;
        for (uint32_t y = yBegin; y <= yEnd; ++y) {
            const size_t rowBase = static_cast<size_t>(y) * kContentW;
            for (uint32_t x = xBegin; x <= xEnd; ++x) {
                if (confidenceValues != nullptr) {
                    const float cval = confidenceValues[rowBase + x];
                    if (std::isfinite(cval)) {
                        confidenceSum += static_cast<double>(cval);
                        ++confidenceCount;
                    }
                }
                const uint16_t disparityFixed = disparity[rowBase + x];
                if (disparityFixed == 0) {
                    continue;
                }
                const float disparityPixels = static_cast<float>(disparityFixed) / kDisparityScale;
                if (disparityPixels <= 0.0f) {
                    continue;
                }
                depthSum += (stereo_depth::kCameraFocalLengthXPixels * kDefaultBaselineMeters) /
                            disparityPixels;
                ++sampleCount;
            }
        }
        if (sampleCount > 0) {
            value.valid = true;
            value.depthMeters = static_cast<float>(depthSum / sampleCount);
        }
        if (confidenceCount > 0) {
            value.confidenceValid = true;
            value.confidenceAvg = static_cast<float>(confidenceSum / confidenceCount);
        }
    }
    return values;
}

std::string formatDepthMeters(bool valid, float depthMeters) {
    if (!valid) {
        return "--.-m";
    }
    char buffer[16] = {};
    std::snprintf(buffer, sizeof(buffer), "%.2fm", depthMeters);
    return std::string(buffer);
}

std::string formatConfidencePercent(bool valid, float confidenceAvg) {
    if (!valid || !std::isfinite(confidenceAvg)) {
        return "--%";
    }
    double percent = static_cast<double>(confidenceAvg);
    if (percent <= 1.0) {
        percent *= 100.0;
    }
    percent = std::max(0.0, std::min(100.0, percent));
    char buffer[16] = {};
    std::snprintf(buffer, sizeof(buffer), "%.0f%%", percent);
    return std::string(buffer);
}

void drawTextBox(cv::Mat& image, int centerX, int centerY, const std::string& text,
                 const cv::Scalar& textColor, const cv::Scalar& bgColor) {
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, kRoiTextScale,
                                              kRoiTextThickness, &baseline);
    const int width = textSize.width + 8;
    const int height = textSize.height + baseline + 8;
    const int left = std::max(0, std::min(image.cols - width, centerX - width / 2));
    const int top = std::max(0, std::min(image.rows - height, centerY - height / 2));
    cv::rectangle(image, cv::Rect(left, top, width, height), bgColor, cv::FILLED);
    cv::rectangle(image, cv::Rect(left, top, width, height), textColor, 1);
    cv::putText(image, text, cv::Point(left + 4, top + 4 + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, kRoiTextScale, textColor, kRoiTextThickness, cv::LINE_AA);
}

// Per-pixel disparity -> TURBO colormap (disparity in U16 fixed-point), zeros black.
cv::Mat makeDepthColorMap(const stereo_depth::PipelineOutput& output) {
    const size_t expectedBytes = static_cast<size_t>(kContentW) * kContentH * sizeof(uint16_t);
    if (output.depthData.size() < expectedBytes) {
        return cv::Mat::zeros(kContentH, kContentW, CV_8UC3);
    }
    cv::Mat disparity16(kContentH, kContentW, CV_16UC1,
                        const_cast<std::byte*>(output.depthData.data()));
    cv::Mat clipped16;
    cv::min(disparity16, static_cast<double>(kDepthDisplayMax), clipped16);
    cv::Mat normalized8;
    clipped16.convertTo(normalized8, CV_8UC1, 255.0 / static_cast<double>(kDepthDisplayMax));
    cv::Mat color;
    cv::applyColorMap(normalized8, color, cv::COLORMAP_TURBO);
    color.setTo(cv::Scalar(0, 0, 0), disparity16 == 0);
    return color;
}

// 11x11 block-averaged disparity -> TURBO colormap, zeros black.
cv::Mat makeBlockAveragedDepthColorMap(const stereo_depth::PipelineOutput& output) {
    const size_t expectedBytes = static_cast<size_t>(kContentW) * kContentH * sizeof(uint16_t);
    if (output.depthData.size() < expectedBytes) {
        return cv::Mat::zeros(kContentH, kContentW, CV_8UC3);
    }
    const auto* disparity = reinterpret_cast<const uint16_t*>(output.depthData.data());
    cv::Mat normalized(kContentH, kContentW, CV_8UC1, cv::Scalar(0));
    for (uint32_t y0 = 0; y0 < static_cast<uint32_t>(kContentH); y0 += kRoiBoxSizePx) {
        const uint32_t yEnd = std::min<uint32_t>(kContentH, y0 + kRoiBoxSizePx);
        for (uint32_t x0 = 0; x0 < static_cast<uint32_t>(kContentW); x0 += kRoiBoxSizePx) {
            const uint32_t xEnd = std::min<uint32_t>(kContentW, x0 + kRoiBoxSizePx);
            uint32_t disparitySum = 0;
            uint32_t sampleCount = 0;
            for (uint32_t y = y0; y < yEnd; ++y) {
                const size_t rowBase = static_cast<size_t>(y) * kContentW;
                for (uint32_t x = x0; x < xEnd; ++x) {
                    const uint16_t d = disparity[rowBase + x];
                    if (d == 0) {
                        continue;
                    }
                    disparitySum += d;
                    ++sampleCount;
                }
            }
            uint8_t displayValue = 0;
            if (sampleCount > 0) {
                const uint32_t avg = disparitySum / sampleCount;
                const uint32_t clipped = std::min<uint32_t>(avg, kDepthDisplayMax);
                displayValue = static_cast<uint8_t>(
                    std::lround((static_cast<double>(clipped) * 255.0) / kDepthDisplayMax));
            }
            for (uint32_t y = y0; y < yEnd; ++y) {
                auto* row = normalized.ptr<uint8_t>(static_cast<int>(y));
                for (uint32_t x = x0; x < xEnd; ++x) {
                    row[x] = displayValue;
                }
            }
        }
    }
    cv::Mat color;
    cv::applyColorMap(normalized, color, cv::COLORMAP_TURBO);
    color.setTo(cv::Scalar(0, 0, 0), normalized == 0);
    return color;
}

void annotateRoiFrame(cv::Mat& frame, const std::array<RoiDepthValue, 9>& roiDepthValues) {
    if (frame.empty()) {
        return;
    }
    for (const auto& value : roiDepthValues) {
        const int left =
            std::max<int>(0, static_cast<int>(value.centerX) - static_cast<int>(kRoiBoxSizePx / 2));
        const int top =
            std::max<int>(0, static_cast<int>(value.centerY) - static_cast<int>(kRoiBoxSizePx / 2));
        cv::rectangle(frame, cv::Rect(left, top, kRoiBoxSizePx, kRoiBoxSizePx),
                      cv::Scalar(255, 255, 255), 1);
        cv::rectangle(frame,
                      cv::Rect(static_cast<int>(value.centerX) - 1,
                               static_cast<int>(value.centerY) - 1, 3, 3),
                      cv::Scalar(0, 0, 0), cv::FILLED);
        const std::string text =
            formatDepthMeters(value.valid, value.depthMeters) + " " +
            formatConfidencePercent(value.confidenceValid, value.confidenceAvg);
        drawTextBox(frame, static_cast<int>(value.centerX),
                    std::max(10, static_cast<int>(value.centerY) - 20), text,
                    cv::Scalar(255, 255, 255), cv::Scalar(0, 0, 0));
    }
    cv::rectangle(frame, cv::Rect(0, 0, frame.cols, frame.rows), cv::Scalar(255, 255, 255), 4);
}

// Wrap a contiguous (stride == width) NV12 frame and convert to BGR.
bool nv12FrameToBgr(const AX_VIDEO_FRAME_T& frame, cv::Mat& bgr) {
    if (frame.u64VirAddr[0] == 0 || frame.u32Width == 0 || frame.u32Height == 0) {
        return false;
    }
    const int w = static_cast<int>(frame.u32Width);
    const int h = static_cast<int>(frame.u32Height);
    const int stride = static_cast<int>(frame.u32PicStride[0]);
    if (stride != w) {
        return false;  // host pipeline VO frames are contiguous
    }
    cv::Mat nv12(h * 3 / 2, w, CV_8UC1, reinterpret_cast<void*>(frame.u64VirAddr[0]), stride);
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    return true;
}

// Scale to fill the tile (stretch).
cv::Mat tileStretch(const cv::Mat& src) {
    cv::Mat tile;
    if (src.empty()) {
        return cv::Mat::zeros(kTileH, kTileW, CV_8UC3);
    }
    cv::resize(src, tile, cv::Size(kTileW, kTileH));
    return tile;
}

// Scale preserving aspect ratio, centered on a black tile (letterbox). Used for
// the raw stereo input so it is not stretched.
cv::Mat tileLetterbox(const cv::Mat& src) {
    cv::Mat tile = cv::Mat::zeros(kTileH, kTileW, CV_8UC3);
    if (src.empty()) {
        return tile;
    }
    const double scale =
        std::min(static_cast<double>(kTileW) / src.cols, static_cast<double>(kTileH) / src.rows);
    const int w = std::max(1, static_cast<int>(std::lround(src.cols * scale)));
    const int h = std::max(1, static_cast<int>(std::lround(src.rows * scale)));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(w, h));
    const int x = (kTileW - w) / 2;
    const int y = (kTileH - h) / 2;
    resized.copyTo(tile(cv::Rect(x, y, w, h)));
    return tile;
}

void labelTile(cv::Mat& tile, const std::string& text) {
    cv::putText(tile, text, cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0),
                3, cv::LINE_AA);
    cv::putText(tile, text, cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

std::optional<int> queryNpuUtilizationPercent() {
    FILE* pipe = ::popen("axcl-smi sh cat /proc/ax_proc/npu/top 2>/dev/null", "r");
    if (pipe == nullptr) {
        return std::nullopt;
    }

    std::string output;
    char buffer[256] = {};
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    ::pclose(pipe);

    const std::string marker = "utilization:";
    const size_t markerPos = output.find(marker);
    if (markerPos == std::string::npos) {
        if (output.find("not updated") != std::string::npos) {
            return 0;
        }
        return std::nullopt;
    }

    size_t digitsBegin = markerPos + marker.size();
    while (digitsBegin < output.size() &&
           std::isspace(static_cast<unsigned char>(output[digitsBegin])) != 0) {
        ++digitsBegin;
    }

    size_t digitsEnd = digitsBegin;
    while (digitsEnd < output.size() &&
           std::isdigit(static_cast<unsigned char>(output[digitsEnd])) != 0) {
        ++digitsEnd;
    }
    if (digitsBegin == digitsEnd) {
        return std::nullopt;
    }

    const int value = std::atoi(output.substr(digitsBegin, digitsEnd - digitsBegin).c_str());
    return std::max(0, std::min(100, value));
}

void drawOverlayText(cv::Mat& image, const std::string& text, const cv::Point& origin,
                     double fontScale = 0.65) {
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 0), 3,
                cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(255, 255, 255),
                1, cv::LINE_AA);
}

void drawStatusOverlay(cv::Mat& image, int npuUtilizationPercent) {
    const std::string utilText = "NPU " + std::to_string(npuUtilizationPercent) + "%";

    drawOverlayText(image, utilText, cv::Point(16, image.rows - 22));
}

// The Rockchip OpenCV/GTK3 stack on Orange Pi boards does not implement
// WND_PROP_VISIBLE (it returns -1 even for a live, on-screen window), so those
// boards need a different window-close probe than the default highgui path that
// already works on amd64 Ubuntu (Qt) and Raspberry Pi. Detect the board from its
// device-tree model so ONLY Orange Pi takes the alternative path: amd64 has no
// device-tree model file and Raspberry Pi reports "Raspberry Pi ...", so neither
// matches and their behaviour is left completely unchanged.
bool detectOrangePiBoard() {
    for (const char* path : {"/proc/device-tree/model", "/sys/firmware/devicetree/base/model"}) {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            continue;
        }
        std::string model;
        std::getline(file, model);
        std::transform(model.begin(), model.end(), model.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (model.find("orange pi") != std::string::npos ||
            model.find("orangepi") != std::string::npos || model.find("opi") != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace

struct AxVoDisplay::Impl {
    std::string windowName = "stereo_depth";
    bool windowCreated = false;
    bool windowEverSeen = false;
    bool isOrangePi = false;
    bool disabled = false;
    bool closed = false;
    std::atomic<bool> stopPolling{false};
    std::atomic<int> npuUtilizationPercent{0};
    std::thread npuPollThread;
};

AxVoDisplay::~AxVoDisplay() { stop(); }

bool AxVoDisplay::isWindowClosed() const { return m_impl != nullptr && m_impl->closed; }

bool AxVoDisplay::start(const std::string& outputSpec) {
    (void)outputSpec;
    if (m_impl == nullptr) {
        m_impl = new Impl();
    }
    m_impl->isOrangePi = detectOrangePiBoard();
    m_active = true;

    // highgui needs a display server; Qt/GTK hard-aborts (not catchable) with no
    // display, so disable the preview up front when headless.
    const char* display = std::getenv("DISPLAY");
    const char* wayland = std::getenv("WAYLAND_DISPLAY");
    const bool hasDisplay =
        (display != nullptr && display[0] != '\0') || (wayland != nullptr && wayland[0] != '\0');
    if (!hasDisplay) {
        m_impl->disabled = true;
        ALOGW("--vo requested but no desktop display is available; running headless.");
        return true;
    }

    m_impl->stopPolling.store(false);
    m_impl->npuPollThread = std::thread([impl = m_impl]() {
        while (!impl->stopPolling.load()) {
            if (const auto utilization = queryNpuUtilizationPercent(); utilization.has_value()) {
                impl->npuUtilizationPercent.store(*utilization);
            } else {
                impl->npuUtilizationPercent.store(0);
            }

            for (int i = 0; i < 10 && !impl->stopPolling.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    });

    ALOGN("host VO preview enabled (OpenCV window, 2x2 layout).");
    return true;
}

void AxVoDisplay::stop() {
    if (m_impl != nullptr) {
        m_impl->stopPolling.store(true);
        if (m_impl->npuPollThread.joinable()) {
            m_impl->npuPollThread.join();
        }
        if (m_impl->windowCreated) {
            try {
                cv::destroyWindow(m_impl->windowName);
            } catch (const cv::Exception&) {
            }
        }
        delete m_impl;
        m_impl = nullptr;
    }
    m_active = false;
}

bool AxVoDisplay::present(const stereo_depth::PipelineOutput& output) {
    if (m_impl == nullptr || m_impl->disabled || m_impl->closed) {
        return false;
    }

    cv::Mat leftBgr;
    cv::Mat rawBgr;
    nv12FrameToBgr(output.leftNv12Frame, leftBgr);
    if (!nv12FrameToBgr(output.rawNv12Frame, rawBgr) && leftBgr.empty() &&
        output.rgbData.size() >= static_cast<size_t>(kContentW) * kContentH * 3) {
        leftBgr = cv::Mat(kContentH, kContentW, CV_8UC3,
                          const_cast<void*>(static_cast<const void*>(output.rgbData.data())))
                      .clone();
    }

    cv::Mat dispColor = makeDepthColorMap(output);
    cv::Mat zGrid = makeBlockAveragedDepthColorMap(output);
    annotateRoiFrame(zGrid, computeRoiDepthValues(output));

    cv::Mat tl = tileStretch(leftBgr);
    cv::Mat tr = tileLetterbox(rawBgr);  // raw input: aspect preserved, not stretched
    cv::Mat bl = tileStretch(dispColor);
    cv::Mat br = tileStretch(zGrid);
    labelTile(tl, "left rgb");
    labelTile(tr, "raw input");
    labelTile(bl, "disparity");
    labelTile(br, "depth(z) grid");

    cv::Mat mosaic(kTileH * 2, kTileW * 2, CV_8UC3);
    tl.copyTo(mosaic(cv::Rect(0, 0, kTileW, kTileH)));
    tr.copyTo(mosaic(cv::Rect(kTileW, 0, kTileW, kTileH)));
    bl.copyTo(mosaic(cv::Rect(0, kTileH, kTileW, kTileH)));
    br.copyTo(mosaic(cv::Rect(kTileW, kTileH, kTileW, kTileH)));
    drawStatusOverlay(mosaic, m_impl->npuUtilizationPercent.load());

    try {
        if (!m_impl->windowCreated) {
            // WINDOW_GUI_NORMAL removes the Qt status bar (the x/y + RGB readout).
            cv::namedWindow(m_impl->windowName, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
            cv::resizeWindow(m_impl->windowName, kTileW, kTileH);
            m_impl->windowCreated = true;
        }
        if (m_impl->isOrangePi) {
            // Orange Pi only: its GTK3 highgui never implements WND_PROP_VISIBLE
            // (always -1, even while the window is on screen), so the default test in
            // the else-branch would mark the window closed on the very first frame and
            // the preview would never appear. Detect the close without WND_PROP_VISIBLE.
            //
            // Pump GUI events first so a click on [x] is delivered and OpenCV drops the
            // window from its internal list during waitKey (never during imshow).
            cv::waitKey(1);
            // A destroyed window returns -1 for EVERY property; while it is alive at
            // least one implemented property is >= 0 (ASPECT_RATIO/AUTOSIZE are valid on
            // this build). Treat the window as closed only after it has been seen alive
            // once, so the brief creation race cannot exit before the preview appears.
            const double aliveSignal = std::max({
                cv::getWindowProperty(m_impl->windowName, cv::WND_PROP_ASPECT_RATIO),
                cv::getWindowProperty(m_impl->windowName, cv::WND_PROP_AUTOSIZE),
                cv::getWindowProperty(m_impl->windowName, cv::WND_PROP_VISIBLE),
            });
            if (aliveSignal >= 0.0) {
                m_impl->windowEverSeen = true;
            } else if (m_impl->windowEverSeen) {
                ALOGN("host VO window closed by user; stopping preview.");
                m_impl->closed = true;
                return false;
            }
        } else {
            // amd64 Ubuntu / Raspberry Pi: unchanged. Stop re-creating the window once
            // the user closes it.
            if (cv::getWindowProperty(m_impl->windowName, cv::WND_PROP_VISIBLE) < 1.0) {
                m_impl->closed = true;
                return false;
            }
        }
        cv::imshow(m_impl->windowName, mosaic);
        cv::waitKey(1);
    } catch (const cv::Exception& ex) {
        ALOGW("host VO disabled (no display backend available): %s", ex.what());
        m_impl->disabled = true;
        return false;
    }
    return true;
}

}  // namespace stereo_depth::axera_pipeline

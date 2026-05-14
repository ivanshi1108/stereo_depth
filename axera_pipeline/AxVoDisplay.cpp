#include "include/AxVoDisplay.hpp"

#define SAMPLE_LOG_TAG "VO"
#include "sample_log.h"

extern "C" {
#include "ax_ivps_api.h"
#include "ax_sys_api.h"
#include "ax_vo_api.h"
}

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace stereo_depth::axera_pipeline {
namespace {

constexpr uint32_t kVoWidth = 1920;
constexpr uint32_t kVoHeight = 1080;
constexpr uint32_t kVoChannelCount = 4;
constexpr uint32_t kVoChannelFifoDepth = 1;
constexpr uint32_t kVoChannelFrameSlotCount = kVoChannelFifoDepth + 3;
constexpr uint32_t kIvpsOutputChannel = 0;
constexpr uint32_t kIvpsOutputFilterIndex = 1;
constexpr uint32_t kVoTileWidth = kVoWidth / 2;
constexpr uint32_t kVoTileHeight = kVoHeight / 2;
constexpr uint32_t kVoMaxRawWidth = 1920 * 2;
constexpr uint32_t kVoMaxRawHeight = 1080;
constexpr uint32_t kRoiBoxSizePx = 11;
constexpr uint32_t kDepthDisplayMax = 2048;
constexpr float kZGridDisplayMaxMeters = 12.0f;
constexpr int kPlaceholderBorder = 8;
constexpr float kDefaultBaselineMeters = 0.0629277f;
constexpr uint32_t kDisparityScale = 16;
constexpr double kRoiTextScale = 0.45;
constexpr int kRoiTextThickness = 1;
constexpr const uint32_t kRoiX[3] = {180, 320, 460};
constexpr const uint32_t kRoiY[3] = {96, 192, 288};

struct VoFrameBuffer {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    uint32_t alignedHeight = 0;
    uint32_t yPlaneSize = 0;
    uint32_t frameSize = 0;
    AX_IMG_FORMAT_E format = AX_FORMAT_YUV420_SEMIPLANAR;
};

struct VoFrameSlot {
    AX_BLK blockId = AX_INVALID_BLOCKID;
    AX_U64 phyAddr = 0;
    AX_U64 virAddr = 0;
};

struct VoOutputSpec {
    VO_DEV devId = 0;
    VO_LAYER layerId = 0;
    AX_VO_MODE_E mode = AX_VO_MODE_OFFLINE;
    AX_VO_INTF_TYPE_E intfType = AX_VO_INTF_HDMI;
    AX_VO_INTF_SYNC_E intfSync = AX_VO_OUTPUT_1080P60;
};

struct IvpsChannelContext {
    IVPS_GRP grpId = -1;
    bool created = false;
    bool started = false;
    bool chnEnabled = false;
    bool linkedToVo = false;
};

AX_VO_RECT_T makeVoChannelRect(uint32_t channelIndex) {
    AX_VO_RECT_T rect = {};
    rect.u32Width = kVoTileWidth;
    rect.u32Height = kVoTileHeight;

    switch (channelIndex) {
        case 0:
            rect.u32X = 0;
            rect.u32Y = 0;
            break;
        case 1:
            rect.u32X = kVoTileWidth;
            rect.u32Y = 0;
            break;
        case 2:
            rect.u32X = 0;
            rect.u32Y = kVoTileHeight;
            break;
        case 3:
            rect.u32X = kVoTileWidth;
            rect.u32Y = kVoTileHeight;
            break;
        default:
            break;
    }

    return rect;
}

bool parseVoOutputSpec(const std::string& outputSpec, VoOutputSpec& voSpec) {
    std::vector<char> buffer(outputSpec.begin(), outputSpec.end());
    buffer.push_back('\0');

    char* spec = buffer.data();
    char* p = spec;
    char* end = nullptr;

    if (std::strstr(p, "hdmi") != nullptr) {
        voSpec.intfType = AX_VO_INTF_HDMI;
    } else if (std::strstr(p, "dpi") != nullptr) {
        voSpec.intfType = AX_VO_INTF_DPI;
    } else if (std::strstr(p, "dsi") != nullptr) {
        voSpec.intfType = AX_VO_INTF_DSI;
    } else {
        ALOGE("unsupported VO interface spec: %s", outputSpec.c_str());
        return false;
    }

    end = std::strstr(p, "@");
    if (end == nullptr) {
        ALOGE("VO output spec missing resolution: %s", outputSpec.c_str());
        return false;
    }
    p = end + 1;
    end = std::strstr(p, "@dev");
    if (end == nullptr) {
        ALOGE("VO output spec missing device: %s", outputSpec.c_str());
        return false;
    }
    *end = '\0';

    static const AX_CHAR* kSyncNames[AX_VO_OUTPUT_BUTT] = {
        [AX_VO_OUTPUT_576P50] = "576P50",
        [AX_VO_OUTPUT_480P60] = "480P60",
        [AX_VO_OUTPUT_720P50] = "720P50",
        [AX_VO_OUTPUT_720P60] = "720P60",
        [AX_VO_OUTPUT_1080P24] = "1080P24",
        [AX_VO_OUTPUT_1080P25] = "1080P25",
        [AX_VO_OUTPUT_1080P30] = "1080P30",
        [AX_VO_OUTPUT_1080P50] = "1080P50",
        [AX_VO_OUTPUT_1080P60] = "1080P60",
        [AX_VO_OUTPUT_640x480_60] = "640x480_60",
        [AX_VO_OUTPUT_800x600_60] = "800x600_60",
        [AX_VO_OUTPUT_1024x768_60] = "1024x768_60",
        [AX_VO_OUTPUT_1280x1024_60] = "1280x1024_60",
        [AX_VO_OUTPUT_1366x768_60] = "1366x768_60",
        [AX_VO_OUTPUT_1280x800_60] = "1280x800_60",
        [AX_VO_OUTPUT_1440x900_60] = "1440x900_60",
        [AX_VO_OUTPUT_1600x1200_60] = "1600x1200_60",
        [AX_VO_OUTPUT_1680x1050_60] = "1680x1050_60",
        [AX_VO_OUTPUT_1920x1200_60] = "1920x1200_60",
        [AX_VO_OUTPUT_2560x1600_60] = "2560x1600_60",
        [AX_VO_OUTPUT_3840x2160_24] = "3840x2160_24",
        [AX_VO_OUTPUT_3840x2160_25] = "3840x2160_25",
        [AX_VO_OUTPUT_3840x2160_30] = "3840x2160_30",
        [AX_VO_OUTPUT_3840x2160_50] = "3840x2160_50",
        [AX_VO_OUTPUT_3840x2160_60] = "3840x2160_60",
        [AX_VO_OUTPUT_4096x2160_24] = "4096x2160_24",
        [AX_VO_OUTPUT_4096x2160_25] = "4096x2160_25",
        [AX_VO_OUTPUT_4096x2160_30] = "4096x2160_30",
        [AX_VO_OUTPUT_4096x2160_50] = "4096x2160_50",
        [AX_VO_OUTPUT_4096x2160_60] = "4096x2160_60",
        [AX_VO_OUTPUT_720x1280_60] = "720x1280_60",
        [AX_VO_OUTPUT_1080x1920_60] = "1080x1920_60",
    };

    bool syncFound = false;
    for (int i = 0; i < AX_VO_OUTPUT_BUTT; ++i) {
        if (kSyncNames[i] != nullptr && std::strcmp(p, kSyncNames[i]) == 0) {
            voSpec.intfSync = static_cast<AX_VO_INTF_SYNC_E>(i);
            syncFound = true;
            break;
        }
    }
    if (!syncFound) {
        ALOGE("unsupported VO sync spec: %s", p);
        return false;
    }

    p = end + 4;
    voSpec.devId = static_cast<VO_DEV>(std::strtoul(p, &end, 10));
    if (voSpec.devId >= AX_VO_DEV_MAX) {
        ALOGE("unsupported VO device in spec: %s", outputSpec.c_str());
        return false;
    }

    return true;
}

AX_POOL createPool(uint32_t blockCount, uint64_t blockSize, const char* poolName) {
    AX_POOL_CONFIG_T config = {};
    config.MetaSize = 512;
    config.BlkCnt = blockCount;
    config.BlkSize = blockSize;
    config.CacheMode = POOL_CACHE_MODE_NONCACHE;
    std::strncpy(reinterpret_cast<char*>(config.PartitionName), "anonymous",
                 sizeof(config.PartitionName) - 1);
    std::strncpy(reinterpret_cast<char*>(config.PoolName), poolName, sizeof(config.PoolName) - 1);
    return AX_POOL_CreatePool(&config);
}

constexpr uint32_t alignUp(uint32_t value, uint32_t alignment) {
    return ((value + alignment - 1U) / alignment) * alignment;
}

bool allocateFrameBuffer(VoFrameBuffer& buffer, uint32_t width, uint32_t height,
                         AX_IMG_FORMAT_E format = AX_FORMAT_YUV420_SEMIPLANAR) {
    buffer.width = width;
    buffer.height = height;
    buffer.format = format;
    if (format == AX_FORMAT_YUV420_SEMIPLANAR) {
        buffer.stride = alignUp(width, 16);
        buffer.alignedHeight = alignUp(height, 2);
        buffer.yPlaneSize = buffer.stride * buffer.alignedHeight;
        buffer.frameSize = buffer.yPlaneSize + (buffer.yPlaneSize / 2);
        return true;
    }

    if (format == AX_FORMAT_BGR888) {
        buffer.stride = alignUp(width * 3, 16);
        buffer.alignedHeight = height;
        buffer.yPlaneSize = 0;
        buffer.frameSize = buffer.stride * buffer.alignedHeight;
        return true;
    }

    return false;
}

inline uint8_t clampToByte(int value) {
    return static_cast<uint8_t>(std::max(0, std::min(255, value)));
}

cv::Mat makePlaceholderTile(int width, int height, const cv::Vec3b& baseColor,
                            const cv::Vec3b& accentColor) {
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(baseColor[0], baseColor[1], baseColor[2]));
    cv::rectangle(image, cv::Rect(0, 0, width, height),
                  cv::Scalar(accentColor[0], accentColor[1], accentColor[2]), kPlaceholderBorder);
    cv::rectangle(image, cv::Rect(width / 6, height / 6, width * 2 / 3, height * 2 / 3),
                  cv::Scalar(clampToByte(baseColor[0] + 12), clampToByte(baseColor[1] + 12),
                             clampToByte(baseColor[2] + 12)),
                  cv::FILLED);
    cv::rectangle(image, cv::Rect(width / 6, height / 6, width * 2 / 3, height * 2 / 3),
                  cv::Scalar(accentColor[0], accentColor[1], accentColor[2]), 4);
    return image;
}

void drawTextBox(cv::Mat& image, int centerX, int centerY, const std::string& text,
                 const cv::Vec3b& textColor, const cv::Vec3b& backgroundColor) {
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, kRoiTextScale,
                                              kRoiTextThickness, &baseline);
    const int width = textSize.width + 8;
    const int height = textSize.height + baseline + 8;
    const int left = std::max(0, std::min(image.cols - width, centerX - width / 2));
    const int top = std::max(0, std::min(image.rows - height, centerY - height / 2));
    cv::rectangle(image, cv::Rect(left, top, width, height),
                  cv::Scalar(backgroundColor[0], backgroundColor[1], backgroundColor[2]),
                  cv::FILLED);
    cv::rectangle(image, cv::Rect(left, top, width, height),
                  cv::Scalar(textColor[0], textColor[1], textColor[2]), 1);
    cv::putText(image, text, cv::Point(left + 4, top + 4 + textSize.height),
                cv::FONT_HERSHEY_SIMPLEX, kRoiTextScale,
                cv::Scalar(textColor[0], textColor[1], textColor[2]), kRoiTextThickness,
                cv::LINE_AA);
}

struct RoiDepthValue {
    uint32_t centerX = 0;
    uint32_t centerY = 0;
    bool valid = false;
    float depthMeters = 0.0f;
    bool confidenceValid = false;
    float confidenceAvg = 0.0f;
};

std::array<RoiDepthValue, 9> computeRoiDepthValues(const stereo_depth::PipelineOutput& output) {
    std::array<RoiDepthValue, 9> values = {};
    const size_t expectedBytes = static_cast<size_t>(stereo_depth::kDewarpImageWidth) *
                                 static_cast<size_t>(stereo_depth::kDewarpImageHeight) *
                                 sizeof(uint16_t);
    if (output.depthData.size() < expectedBytes) {
        return values;
    }

    const auto* disparity = reinterpret_cast<const uint16_t*>(output.depthData.data());
    const size_t expectedConfidenceBytes = static_cast<size_t>(stereo_depth::kDewarpImageWidth) *
                                           static_cast<size_t>(stereo_depth::kDewarpImageHeight) *
                                           sizeof(float);
    const auto* confidenceValues =
        output.confidenceData.size() >= expectedConfidenceBytes
            ? reinterpret_cast<const float*>(output.confidenceData.data())
            : nullptr;
    for (size_t i = 0; i < values.size(); ++i) {
        const uint32_t gridX = static_cast<uint32_t>(i % 3);
        const uint32_t gridY = static_cast<uint32_t>(i / 3);
        RoiDepthValue& value = values[i];
        value.centerX = kRoiX[gridX];
        value.centerY = kRoiY[gridY];

        const uint32_t xBegin =
            value.centerX > (kRoiBoxSizePx / 2) ? value.centerX - (kRoiBoxSizePx / 2) : 0;
        const uint32_t yBegin =
            value.centerY > (kRoiBoxSizePx / 2) ? value.centerY - (kRoiBoxSizePx / 2) : 0;
        const uint32_t xEnd = std::min<uint32_t>(stereo_depth::kDewarpImageWidth - 1,
                                                 value.centerX + (kRoiBoxSizePx / 2));
        const uint32_t yEnd = std::min<uint32_t>(stereo_depth::kDewarpImageHeight - 1,
                                                 value.centerY + (kRoiBoxSizePx / 2));

        double depthSum = 0.0;
        uint32_t sampleCount = 0;
        double confidenceSum = 0.0;
        uint32_t confidenceCount = 0;
        for (uint32_t y = yBegin; y <= yEnd; ++y) {
            const size_t rowBase =
                static_cast<size_t>(y) * static_cast<size_t>(stereo_depth::kDewarpImageWidth);
            for (uint32_t x = xBegin; x <= xEnd; ++x) {
                if (confidenceValues != nullptr) {
                    const float confidenceValue = confidenceValues[rowBase + x];
                    if (std::isfinite(confidenceValue)) {
                        confidenceSum += static_cast<double>(confidenceValue);
                        ++confidenceCount;
                    }
                }

                const uint16_t disparityFixed = disparity[rowBase + x];
                if (disparityFixed == 0) {
                    continue;
                }
                const float disparityPixels =
                    static_cast<float>(disparityFixed) / static_cast<float>(kDisparityScale);
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
            value.depthMeters = static_cast<float>(depthSum / static_cast<double>(sampleCount));
        }
        if (confidenceCount > 0) {
            value.confidenceValid = true;
            value.confidenceAvg =
                static_cast<float>(confidenceSum / static_cast<double>(confidenceCount));
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

std::string formatRoiOverlayText(const RoiDepthValue& value) {
    return formatDepthMeters(value.valid, value.depthMeters) + " " +
           formatConfidencePercent(value.confidenceValid, value.confidenceAvg);
}

cv::Mat makeDepthColorMap(const stereo_depth::PipelineOutput& output) {
    if (output.depthData.empty()) {
        return makePlaceholderTile(stereo_depth::kDewarpImageWidth,
                                   stereo_depth::kDewarpImageHeight, cv::Vec3b(20, 20, 28),
                                   cv::Vec3b(64, 96, 224));
    }

    cv::Mat disparity16(stereo_depth::kDewarpImageHeight, stereo_depth::kDewarpImageWidth, CV_16UC1,
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

cv::Mat makeBlockAveragedDepthColorMap(const stereo_depth::PipelineOutput& output) {
    const size_t expectedBytes = static_cast<size_t>(stereo_depth::kDewarpImageWidth) *
                                 static_cast<size_t>(stereo_depth::kDewarpImageHeight) *
                                 sizeof(uint16_t);
    if (output.depthData.size() < expectedBytes) {
        return makePlaceholderTile(stereo_depth::kDewarpImageWidth,
                                   stereo_depth::kDewarpImageHeight, cv::Vec3b(18, 24, 30),
                                   cv::Vec3b(80, 180, 220));
    }

    const auto* disparity = reinterpret_cast<const uint16_t*>(output.depthData.data());
    cv::Mat normalized(stereo_depth::kDewarpImageHeight, stereo_depth::kDewarpImageWidth, CV_8UC1,
                       cv::Scalar(0));

    for (uint32_t y0 = 0; y0 < stereo_depth::kDewarpImageHeight; y0 += kRoiBoxSizePx) {
        const uint32_t yEnd =
            std::min<uint32_t>(stereo_depth::kDewarpImageHeight, y0 + kRoiBoxSizePx);
        for (uint32_t x0 = 0; x0 < stereo_depth::kDewarpImageWidth; x0 += kRoiBoxSizePx) {
            const uint32_t xEnd =
                std::min<uint32_t>(stereo_depth::kDewarpImageWidth, x0 + kRoiBoxSizePx);

            uint32_t disparitySum = 0;
            uint32_t sampleCount = 0;
            for (uint32_t y = y0; y < yEnd; ++y) {
                const size_t rowBase = static_cast<size_t>(y) * stereo_depth::kDewarpImageWidth;
                for (uint32_t x = x0; x < xEnd; ++x) {
                    const uint16_t disparityFixed = disparity[rowBase + x];
                    if (disparityFixed == 0) {
                        continue;
                    }

                    disparitySum += disparityFixed;
                    ++sampleCount;
                }
            }

            uint8_t displayValue = 0;
            if (sampleCount > 0) {
                const uint32_t avgDisparityFixed = disparitySum / sampleCount;
                const uint32_t clippedDisparity =
                    std::min<uint32_t>(avgDisparityFixed, kDepthDisplayMax);
                displayValue = static_cast<uint8_t>(
                    std::lround((static_cast<double>(clippedDisparity) * 255.0) /
                                static_cast<double>(kDepthDisplayMax)));
            }

            for (uint32_t y = y0; y < yEnd; ++y) {
                auto* normalizedRow = normalized.ptr<uint8_t>(static_cast<int>(y));
                for (uint32_t x = x0; x < xEnd; ++x) {
                    normalizedRow[x] = displayValue;
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

        const std::string text = formatRoiOverlayText(value);
        drawTextBox(frame, static_cast<int>(value.centerX),
                    std::max(10, static_cast<int>(value.centerY) - 20), text,
                    cv::Vec3b(255, 255, 255), cv::Vec3b(0, 0, 0));
    }

    cv::rectangle(frame, cv::Rect(0, 0, frame.cols, frame.rows), cv::Scalar(255, 255, 255), 4);
}

AX_VIDEO_FRAME_T makeBgrFrame(const VoFrameBuffer& buffer, const VoFrameSlot& slot, uint32_t width,
                              uint32_t height, uint64_t pts) {
    AX_VIDEO_FRAME_T frame = {};
    frame.enImgFormat = AX_FORMAT_RGB888;
    frame.u32Width = width;
    frame.u32Height = height;
    frame.u32PicStride[0] = buffer.stride;
    frame.u32FrameSize = buffer.stride * height;
    frame.u64PhyAddr[0] = slot.phyAddr;
    frame.u64VirAddr[0] = slot.virAddr;
    frame.u32BlkId[0] = slot.blockId;
    frame.u64PTS = pts;
    return frame;
}

bool copyMatToSlot(const cv::Mat& source, const VoFrameBuffer& buffer, const VoFrameSlot& slot) {
    if (source.empty() || source.type() != CV_8UC3 ||
        source.cols > static_cast<int>(buffer.width) ||
        source.rows > static_cast<int>(buffer.height) || slot.virAddr == 0) {
        return false;
    }

    auto* dstBase = reinterpret_cast<uint8_t*>(slot.virAddr);
    std::memset(dstBase, 0, buffer.frameSize);
    for (int row = 0; row < source.rows; ++row) {
        std::memcpy(dstBase + static_cast<size_t>(row) * buffer.stride, source.ptr(row),
                    static_cast<size_t>(source.cols) * 3);
    }

    return true;
}

bool prepareMatAsBgrFrame(const VoFrameBuffer& buffer, VoFrameSlot& slot, const cv::Mat& source,
                          uint64_t pts, AX_VIDEO_FRAME_T& dstFrame) {
    if (buffer.format != AX_FORMAT_BGR888 || !copyMatToSlot(source, buffer, slot)) {
        return false;
    }

    dstFrame = makeBgrFrame(buffer, slot, static_cast<uint32_t>(source.cols),
                            static_cast<uint32_t>(source.rows), pts);
    return true;
}

void configureAspectRatio(AX_IVPS_ASPECT_RATIO_T& aspectRatio, bool preserveAspect) {
    std::memset(&aspectRatio, 0, sizeof(aspectRatio));
    aspectRatio.eMode = preserveAspect ? AX_IVPS_ASPECT_RATIO_AUTO : AX_IVPS_ASPECT_RATIO_STRETCH;
    aspectRatio.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    aspectRatio.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    aspectRatio.nBgColor = 0x000000;
}

bool configureIvpsChannel(IvpsChannelContext& ivps, uint32_t channelIndex) {
    const bool isRawPreviewChannel = (channelIndex == 1);

    AX_IVPS_GRP_ATTR_T grpAttr = {};
    grpAttr.nInFifoDepth = 1;
    grpAttr.ePipeline = AX_IVPS_PIPELINE_DEFAULT;

    AX_S32 ret = AX_IVPS_CreateGrpEx(&ivps.grpId, &grpAttr);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_IVPS_CreateGrpEx failed for chn=%u ret=0x%x", channelIndex, ret);
        ivps.grpId = -1;
        return false;
    }
    ivps.created = true;

    AX_IVPS_PIPELINE_ATTR_T pipelineAttr = {};
    pipelineAttr.nOutChnNum = 1;
    pipelineAttr.nOutFifoDepth[kIvpsOutputChannel] = 0;
    auto& filter = pipelineAttr.tFilter[kIvpsOutputFilterIndex][0];
    filter.bEngage = AX_TRUE;
    filter.eEngine = isRawPreviewChannel ? AX_IVPS_ENGINE_TDP : AX_IVPS_ENGINE_VPP;
    filter.nDstPicWidth = kVoTileWidth;
    filter.nDstPicHeight = kVoTileHeight;
    filter.nDstPicStride = kVoTileWidth;
    filter.eDstPicFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    filter.tCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    configureAspectRatio(filter.tAspectRatio, isRawPreviewChannel);

    ret = AX_IVPS_SetPipelineAttr(ivps.grpId, &pipelineAttr);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_IVPS_SetPipelineAttr failed for chn=%u grp=%d ret=0x%x", channelIndex, ivps.grpId,
              ret);
        return false;
    }

    AX_IVPS_POOL_ATTR_T poolAttr = {};
    poolAttr.ePoolSrc = POOL_SOURCE_PRIVATE;
    poolAttr.nFrmBufNum = 4;
    ret = AX_IVPS_SetGrpPoolAttr(ivps.grpId, &poolAttr);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_IVPS_SetGrpPoolAttr failed for chn=%u grp=%d ret=0x%x", channelIndex, ivps.grpId,
              ret);
        return false;
    }
    ret = AX_IVPS_SetChnPoolAttr(ivps.grpId, kIvpsOutputChannel, &poolAttr);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_IVPS_SetChnPoolAttr failed for chn=%u grp=%d ret=0x%x", channelIndex, ivps.grpId,
              ret);
        return false;
    }

    ret = AX_IVPS_EnableChn(ivps.grpId, kIvpsOutputChannel);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_IVPS_EnableChn failed for chn=%u grp=%d ret=0x%x", channelIndex, ivps.grpId, ret);
        return false;
    }
    ivps.chnEnabled = true;

    ret = AX_IVPS_StartGrp(ivps.grpId);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_IVPS_StartGrp failed for chn=%u grp=%d ret=0x%x", channelIndex, ivps.grpId, ret);
        return false;
    }
    ivps.started = true;

    return true;
}

bool linkIvpsToVo(const IvpsChannelContext& ivps, VO_LAYER layerId, VO_CHN channelIndex) {
    AX_MOD_INFO_T srcMod = {};
    srcMod.enModId = AX_ID_IVPS;
    srcMod.s32GrpId = ivps.grpId;
    srcMod.s32ChnId = kIvpsOutputChannel;

    AX_MOD_INFO_T dstMod = {};
    dstMod.enModId = AX_ID_VO;
    dstMod.s32GrpId = layerId;
    dstMod.s32ChnId = channelIndex;

    const AX_S32 ret = AX_SYS_Link(&srcMod, &dstMod);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_SYS_Link IVPS->VO failed grp=%d layer=%d chn=%d ret=0x%x", ivps.grpId, layerId,
              channelIndex, ret);
        return false;
    }

    return true;
}

void unlinkIvpsFromVo(const IvpsChannelContext& ivps, VO_LAYER layerId, VO_CHN channelIndex) {
    if (!ivps.linkedToVo || ivps.grpId < 0) {
        return;
    }

    AX_MOD_INFO_T srcMod = {};
    srcMod.enModId = AX_ID_IVPS;
    srcMod.s32GrpId = ivps.grpId;
    srcMod.s32ChnId = kIvpsOutputChannel;

    AX_MOD_INFO_T dstMod = {};
    dstMod.enModId = AX_ID_VO;
    dstMod.s32GrpId = layerId;
    dstMod.s32ChnId = channelIndex;

    AX_SYS_UnLink(&srcMod, &dstMod);
}

bool sendFrameToIvps(const IvpsChannelContext& ivps, const AX_VIDEO_FRAME_T& frame) {
    if (ivps.grpId < 0) {
        return false;
    }

    const AX_S32 ret = AX_IVPS_SendFrame(ivps.grpId, &frame, 0);
    if (ret != AX_SUCCESS) {
        ALOGW("AX_IVPS_SendFrame failed grp=%d ret=0x%x", ivps.grpId, ret);
        return false;
    }

    return true;
}

}  // namespace

struct AxVoDisplay::Impl {
    VoOutputSpec outputSpec = {};
    std::array<AX_POOL, kVoChannelCount> channelPoolIds = {AX_INVALID_POOLID};
    AX_POOL layerPoolId = AX_INVALID_POOLID;
    std::array<VoFrameBuffer, kVoChannelCount> channelBuffers = {};
    std::array<std::array<VoFrameSlot, kVoChannelFrameSlotCount>, kVoChannelCount> frameSlots = {};
    std::array<size_t, kVoChannelCount> nextSlotIndex = {};
    std::array<IvpsChannelContext, kVoChannelCount> ivpsChannels = {};
    bool initialized = false;
    bool deviceEnabled = false;
    bool layerCreated = false;
    bool layerBound = false;
    bool layerEnabled = false;
    std::array<bool, kVoChannelCount> channelEnabled = {};
};

bool allocateFrameSlots(AX_POOL poolId, const VoFrameBuffer& buffer,
                        std::array<VoFrameSlot, kVoChannelFrameSlotCount>& slots) {
    for (auto& slot : slots) {
        slot.blockId = AX_POOL_GetBlock(poolId, buffer.frameSize, nullptr);
        if (slot.blockId == AX_INVALID_BLOCKID) {
            ALOGE("AX_POOL_GetBlock failed while allocating persistent VO frame slots");
            return false;
        }

        slot.phyAddr = AX_POOL_Handle2PhysAddr(slot.blockId);
        if (slot.phyAddr == 0) {
            ALOGE("AX_POOL_Handle2PhysAddr failed for persistent VO frame slot");
            return false;
        }

        AX_VOID* virAddr = AX_POOL_GetBlockVirAddr(slot.blockId);
        if (virAddr == nullptr) {
            ALOGE("AX_POOL_GetBlockVirAddr failed for persistent VO frame slot");
            return false;
        }
        slot.virAddr = reinterpret_cast<AX_U64>(virAddr);
    }

    return true;
}

void releaseFrameSlots(std::array<VoFrameSlot, kVoChannelFrameSlotCount>& slots) {
    for (auto& slot : slots) {
        if (slot.blockId != AX_INVALID_BLOCKID) {
            AX_POOL_ReleaseBlock(slot.blockId);
            slot.blockId = AX_INVALID_BLOCKID;
            slot.phyAddr = 0;
            slot.virAddr = 0;
        }
    }
}

AxVoDisplay::~AxVoDisplay() { stop(); }

bool AxVoDisplay::start(const std::string& outputSpec) {
    if (m_active) {
        return true;
    }

    std::unique_ptr<Impl> impl(new Impl());
    if (!parseVoOutputSpec(outputSpec, impl->outputSpec)) {
        return false;
    }

    m_impl = impl.release();
    m_active = true;
    Impl* implPtr = m_impl;

    const uint64_t layerBlockSize = static_cast<uint64_t>(alignUp(kVoWidth, 16)) *
                                    static_cast<uint64_t>(alignUp(kVoHeight, 2)) * 3ULL / 2ULL;
    implPtr->layerPoolId = createPool(3, layerBlockSize, "stereo_vo_layer");
    if (implPtr->layerPoolId == AX_INVALID_POOLID) {
        ALOGE("failed to create VO layer pool");
        stop();
        return false;
    }

    allocateFrameBuffer(implPtr->channelBuffers[0], stereo_depth::kDewarpImageWidth,
                        stereo_depth::kDewarpImageHeight, AX_FORMAT_YUV420_SEMIPLANAR);
    allocateFrameBuffer(implPtr->channelBuffers[1], kVoMaxRawWidth, kVoMaxRawHeight,
                        AX_FORMAT_YUV420_SEMIPLANAR);
    allocateFrameBuffer(implPtr->channelBuffers[2], stereo_depth::kDewarpImageWidth,
                        stereo_depth::kDewarpImageHeight, AX_FORMAT_BGR888);
    allocateFrameBuffer(implPtr->channelBuffers[3], stereo_depth::kDewarpImageWidth,
                        stereo_depth::kDewarpImageHeight, AX_FORMAT_BGR888);

    for (uint32_t i = 0; i < kVoChannelCount; ++i) {
        char poolName[32] = {};
        std::snprintf(poolName, sizeof(poolName), "stereo_vo_ch%u", i);
        implPtr->channelPoolIds[i] =
            createPool(kVoChannelFrameSlotCount, implPtr->channelBuffers[i].frameSize, poolName);
        if (implPtr->channelPoolIds[i] == AX_INVALID_POOLID) {
            ALOGE("failed to create VO channel pool %u", i);
            stop();
            return false;
        }
        if (!allocateFrameSlots(implPtr->channelPoolIds[i], implPtr->channelBuffers[i],
                                implPtr->frameSlots[i])) {
            releaseFrameSlots(implPtr->frameSlots[i]);
            stop();
            return false;
        }
    }

    AX_S32 ret = AX_VO_Init();
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_Init failed: 0x%x", ret);
        stop();
        return false;
    }
    implPtr->initialized = true;

    ret = AX_VO_CreateVideoLayer(&implPtr->outputSpec.layerId);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_CreateVideoLayer failed: 0x%x", ret);
        stop();
        return false;
    }
    implPtr->layerCreated = true;

    AX_VO_PUB_ATTR_T stPubAttr = {};
    stPubAttr.enIntfType = implPtr->outputSpec.intfType;
    stPubAttr.enIntfSync = implPtr->outputSpec.intfSync;
    stPubAttr.enMode = implPtr->outputSpec.mode;
    ret = AX_VO_SetPubAttr(implPtr->outputSpec.devId, &stPubAttr);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_SetPubAttr failed: 0x%x", ret);
        stop();
        return false;
    }

    ret = AX_VO_Enable(implPtr->outputSpec.devId);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_Enable failed: 0x%x", ret);
        stop();
        return false;
    }
    implPtr->deviceEnabled = true;

    AX_VO_VIDEO_LAYER_ATTR_T stLayerAttr = {};
    stLayerAttr.stDispRect.u32Width = kVoWidth;
    stLayerAttr.stDispRect.u32Height = kVoHeight;
    stLayerAttr.stImageSize.u32Width = kVoWidth;
    stLayerAttr.stImageSize.u32Height = kVoHeight;
    stLayerAttr.enPixFmt = AX_FORMAT_YUV420_SEMIPLANAR;
    stLayerAttr.enSyncMode = AX_VO_LAYER_SYNC_NORMAL;
    stLayerAttr.f32FrmRate = 30.0f;
    stLayerAttr.u32FifoDepth = 3;
    stLayerAttr.u32BkClr = 0;
    stLayerAttr.u32PrimaryChnId = 0;
    stLayerAttr.enWBMode = AX_VO_LAYER_WB_POOL;
    stLayerAttr.u32PoolId = implPtr->layerPoolId;
    stLayerAttr.u32DispatchMode = AX_VO_LAYER_OUT_TO_LINK;
    stLayerAttr.enPartMode = AX_VO_PART_MODE_MULTI;
    ret = AX_VO_SetVideoLayerAttr(implPtr->outputSpec.layerId, &stLayerAttr);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_SetVideoLayerAttr failed: 0x%x", ret);
        stop();
        return false;
    }

    ret = AX_VO_BindVideoLayer(implPtr->outputSpec.layerId, implPtr->outputSpec.devId);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_BindVideoLayer failed: 0x%x", ret);
        stop();
        return false;
    }
    implPtr->layerBound = true;

    ret = AX_VO_EnableVideoLayer(implPtr->outputSpec.layerId);
    if (ret != AX_SUCCESS) {
        ALOGE("AX_VO_EnableVideoLayer failed: 0x%x", ret);
        stop();
        return false;
    }
    implPtr->layerEnabled = true;

    for (uint32_t chn = 0; chn < kVoChannelCount; ++chn) {
        AX_VO_CHN_ATTR_T stChnAttr = {};
        stChnAttr.stRect = makeVoChannelRect(chn);
        stChnAttr.u32FifoDepth = kVoChannelFifoDepth;
        stChnAttr.u32Priority = 0;
        stChnAttr.bKeepPrevFr = AX_TRUE;
        stChnAttr.bInUseFrOutput = AX_FALSE;

        ret = AX_VO_SetChnAttr(implPtr->outputSpec.layerId, chn, &stChnAttr);
        if (ret != AX_SUCCESS) {
            ALOGE("AX_VO_SetChnAttr failed for chn=%u ret=0x%x", chn, ret);
            stop();
            return false;
        }

        ret = AX_VO_SetChnFrameRate(implPtr->outputSpec.layerId, chn, 30.0f);
        if (ret != AX_SUCCESS) {
            ALOGE("AX_VO_SetChnFrameRate failed for chn=%u ret=0x%x", chn, ret);
            stop();
            return false;
        }

        ret = AX_VO_EnableChn(implPtr->outputSpec.layerId, chn);
        if (ret != AX_SUCCESS) {
            ALOGE("AX_VO_EnableChn failed for chn=%u ret=0x%x", chn, ret);
            stop();
            return false;
        }
        implPtr->channelEnabled[chn] = true;
    }

    for (uint32_t chn = 0; chn < kVoChannelCount; ++chn) {
        if (!configureIvpsChannel(implPtr->ivpsChannels[chn], chn)) {
            stop();
            return false;
        }
        if (!linkIvpsToVo(implPtr->ivpsChannels[chn], implPtr->outputSpec.layerId, chn)) {
            stop();
            return false;
        }
        implPtr->ivpsChannels[chn].linkedToVo = true;
    }

    ALOGN("VO HDMI output enabled: %s, 4 channels", outputSpec.c_str());
    return true;
}

void AxVoDisplay::stop() {
    if (m_impl == nullptr) {
        m_active = false;
        return;
    }

    std::unique_ptr<Impl> impl(m_impl);
    m_impl = nullptr;
    m_active = false;

    for (uint32_t chn = 0; chn < kVoChannelCount; ++chn) {
        unlinkIvpsFromVo(impl->ivpsChannels[chn], impl->outputSpec.layerId, chn);
        impl->ivpsChannels[chn].linkedToVo = false;
        if (impl->ivpsChannels[chn].started) {
            AX_IVPS_StopGrp(impl->ivpsChannels[chn].grpId);
            impl->ivpsChannels[chn].started = false;
        }
        if (impl->ivpsChannels[chn].chnEnabled) {
            AX_IVPS_DisableChn(impl->ivpsChannels[chn].grpId, kIvpsOutputChannel);
            impl->ivpsChannels[chn].chnEnabled = false;
        }
        if (impl->ivpsChannels[chn].created) {
            AX_IVPS_DestoryGrp(impl->ivpsChannels[chn].grpId);
            impl->ivpsChannels[chn].created = false;
            impl->ivpsChannels[chn].grpId = -1;
        }
    }

    for (uint32_t chn = 0; chn < kVoChannelCount; ++chn) {
        if (impl->channelEnabled[chn]) {
            AX_VO_DisableChn(impl->outputSpec.layerId, chn);
            impl->channelEnabled[chn] = false;
        }
    }

    if (impl->layerEnabled) {
        AX_VO_DisableVideoLayer(impl->outputSpec.layerId);
        impl->layerEnabled = false;
    }
    if (impl->layerBound) {
        AX_VO_UnBindVideoLayer(impl->outputSpec.layerId, impl->outputSpec.devId);
        impl->layerBound = false;
    }
    if (impl->deviceEnabled) {
        AX_VO_Disable(impl->outputSpec.devId);
        impl->deviceEnabled = false;
    }
    if (impl->layerCreated) {
        AX_VO_DestroyVideoLayer(impl->outputSpec.layerId);
        impl->layerCreated = false;
    }
    if (impl->initialized) {
        AX_VO_Deinit();
        impl->initialized = false;
    }
    for (uint32_t chn = 0; chn < kVoChannelCount; ++chn) {
        releaseFrameSlots(impl->frameSlots[chn]);
        if (impl->channelPoolIds[chn] != AX_INVALID_POOLID) {
            AX_POOL_DestroyPool(impl->channelPoolIds[chn]);
            impl->channelPoolIds[chn] = AX_INVALID_POOLID;
        }
    }
    if (impl->layerPoolId != AX_INVALID_POOLID) {
        AX_POOL_DestroyPool(impl->layerPoolId);
        impl->layerPoolId = AX_INVALID_POOLID;
    }
}

bool AxVoDisplay::present(const stereo_depth::PipelineOutput& output) {
    if (!m_active || m_impl == nullptr) {
        return false;
    }

    if (output.rawNv12Frame.u64PhyAddr[0] == 0 || output.rawNv12Frame.u64PhyAddr[1] == 0 ||
        output.leftNv12Frame.u64PhyAddr[0] == 0 || output.leftNv12Frame.u64PhyAddr[1] == 0) {
        ALOGW("VO present skipped: required NV12 inputs are unavailable");
        return false;
    }

    cv::Mat depthFrame = makeDepthColorMap(output);
    cv::Mat zGridFrame = makeBlockAveragedDepthColorMap(output);
    const auto roiDepthValues = computeRoiDepthValues(output);
    annotateRoiFrame(zGridFrame, roiDepthValues);

    auto& slot2 = m_impl->frameSlots[2][m_impl->nextSlotIndex[2]];
    auto& slot3 = m_impl->frameSlots[3][m_impl->nextSlotIndex[3]];

    AX_VIDEO_FRAME_T frame0 = output.leftNv12Frame;
    frame0.u64PTS = output.frameTimestampNs;
    const bool ok0 = sendFrameToIvps(m_impl->ivpsChannels[0], frame0);

    AX_VIDEO_FRAME_T frame1 = output.rawNv12Frame;
    frame1.u64PTS = output.frameTimestampNs;
    const bool ok1 = sendFrameToIvps(m_impl->ivpsChannels[1], frame1);

    AX_VIDEO_FRAME_T frame2 = {};
    AX_VIDEO_FRAME_T frame3 = {};
    const bool ok2 = prepareMatAsBgrFrame(m_impl->channelBuffers[2], slot2, depthFrame,
                                          output.frameTimestampNs, frame2) &&
                     sendFrameToIvps(m_impl->ivpsChannels[2], frame2);
    const bool ok3 = prepareMatAsBgrFrame(m_impl->channelBuffers[3], slot3, zGridFrame,
                                          output.frameTimestampNs, frame3) &&
                     sendFrameToIvps(m_impl->ivpsChannels[3], frame3);

    for (uint32_t chn = 0; chn < kVoChannelCount; ++chn) {
        m_impl->nextSlotIndex[chn] = (m_impl->nextSlotIndex[chn] + 1) % kVoChannelFrameSlotCount;
    }

    return ok0 && ok1 && ok2 && ok3;
}

}  // namespace stereo_depth::axera_pipeline
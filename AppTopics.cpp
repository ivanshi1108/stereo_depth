#include "AppTopics.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace stereo_depth::app {
namespace {

constexpr uint32_t kGridCellWidthPx = 11;
constexpr uint32_t kGridCellHeightPx = 11;
constexpr uint32_t kPointStride = 16;

std::vector<uint32_t> makeGridPositions(uint32_t size, uint32_t cellSizePx) {
    std::vector<uint32_t> positions;
    if (size == 0 || cellSizePx == 0) {
        return positions;
    }

    positions.reserve((size / cellSizePx) + 2);
    positions.push_back(0);
    for (uint32_t p = cellSizePx; p < size; p += cellSizePx) {
        positions.push_back(p);
    }
    if (positions.back() != (size - 1)) {
        positions.push_back(size - 1);
    }
    return positions;
}

foxglove::schemas::ImageAnnotations makeGridAnnotations(uint32_t width, uint32_t height,
                                                        uint32_t cellWidthPx,
                                                        uint32_t cellHeightPx) {
    foxglove::schemas::ImageAnnotations annotations;
    if (width == 0 || height == 0 || cellWidthPx == 0 || cellHeightPx == 0) {
        return annotations;
    }

    foxglove::schemas::PointsAnnotation gridLines;
    gridLines.type = foxglove::schemas::PointsAnnotation::PointsAnnotationType::LINE_LIST;
    gridLines.thickness = 1.0;
    gridLines.outline_color = foxglove::schemas::Color{0.0, 1.0, 0.0, 1.0};

    const auto xs = makeGridPositions(width, cellWidthPx);
    const auto ys = makeGridPositions(height, cellHeightPx);
    gridLines.points.reserve((xs.size() + ys.size()) * 2);
    const double maxX = static_cast<double>(width - 1);
    const double maxY = static_cast<double>(height - 1);

    for (uint32_t y : ys) {
        gridLines.points.push_back(foxglove::schemas::Point2{0.0, static_cast<double>(y)});
        gridLines.points.push_back(foxglove::schemas::Point2{maxX, static_cast<double>(y)});
    }

    for (uint32_t x : xs) {
        gridLines.points.push_back(foxglove::schemas::Point2{static_cast<double>(x), 0.0});
        gridLines.points.push_back(foxglove::schemas::Point2{static_cast<double>(x), maxY});
    }

    annotations.points.push_back(std::move(gridLines));
    return annotations;
}

foxglove::schemas::RawImage makeRawImage(const std::vector<std::byte>& data, uint32_t width,
                                         uint32_t height, const std::string& encoding,
                                         const std::string& frameId) {
    foxglove::schemas::RawImage msg;
    msg.width = width;
    msg.height = height;
    msg.step = height > 0 ? static_cast<uint32_t>(data.size() / height) : 0;
    msg.encoding = encoding;
    msg.frame_id = frameId;
    msg.data = data;
    return msg;
}

foxglove::schemas::Timestamp toFoxgloveTimestampNs(uint64_t tsNs) {
    foxglove::schemas::Timestamp ts;
    ts.sec = static_cast<uint32_t>(tsNs / 1000000000ULL);
    ts.nsec = static_cast<uint32_t>(tsNs % 1000000000ULL);
    return ts;
}

bool logChannelCreateError(const char* name, foxglove::FoxgloveError error,
                           const StereoDepthTopics::LogFn& logFn) {
    if (!logFn) {
        return false;
    }
    std::ostringstream oss;
    oss << "recording failed: create " << name << " channel " << foxglove::strerror(error);
    logFn(oss.str());
    return false;
}

std::string escapeJsonString(const std::string& input) {
    std::string escaped;
    escaped.reserve(input.size() + 8);
    for (unsigned char ch : input) {
        switch (ch) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\b':
                escaped += "\\b";
                break;
            case '\f':
                escaped += "\\f";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                if (ch < 0x20) {
                    std::ostringstream oss;
                    oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(ch);
                    escaped += oss.str();
                } else {
                    escaped.push_back(static_cast<char>(ch));
                }
                break;
        }
    }
    return escaped;
}

}  // namespace

TopicFlags makeTopicFlags(const stereo_depth::PipelineOutput& output) {
    TopicFlags flags;
    flags.rgb = !output.rgbData.empty();
    flags.depth = !output.depthData.empty();
    flags.gridAvg = !output.zGridAvgData.empty();
    flags.grid = true;
    flags.cloud = !output.pointCloudData.empty();
    return flags;
}

StereoDepthTopics::StereoDepthTopics()
    : m_rgbTopic("/camera/rgb"),
      m_depthTopic("/camera/disparity"),
      m_gridAvgTopic("/camera/z_grid_avg"),
      m_cloudTopic("/camera/pointcloud"),
      m_gridTopic("/camera/grid"),
      m_deviceInfoTopic("/camera/device_info"),
      m_roiZAvgTopic("/camera/roi_z_avg"),
      m_rawYuyvTopic("/camera/yuyv"),
      m_h264Topic("/camera/h264"),
      m_frameId("camera_link"),
      m_gridAnnotations(makeGridAnnotations(stereo_depth::kDewarpImageWidth,
                                            stereo_depth::kDewarpImageHeight, kGridCellWidthPx,
                                            kGridCellHeightPx)),
      m_pointFields({
          {"x", 0, foxglove::schemas::PackedElementField::NumericType::FLOAT32},
          {"y", 4, foxglove::schemas::PackedElementField::NumericType::FLOAT32},
          {"z", 8, foxglove::schemas::PackedElementField::NumericType::FLOAT32},
          {"alpha", 12, foxglove::schemas::PackedElementField::NumericType::UINT8},
          {"red", 13, foxglove::schemas::PackedElementField::NumericType::UINT8},
          {"green", 14, foxglove::schemas::PackedElementField::NumericType::UINT8},
          {"blue", 15, foxglove::schemas::PackedElementField::NumericType::UINT8},
      }) {}

bool StereoDepthTopics::registerFoxgloveChannels(FoxgloveWrapper& foxglove) const {
    return foxglove.addImageChannel(m_rawYuyvTopic) && foxglove.addImageChannel(m_rgbTopic) &&
           foxglove.addImageChannel(m_depthTopic) && foxglove.addImageChannel(m_gridAvgTopic) &&
           foxglove.addPointCloudChannel(m_cloudTopic) &&
           foxglove.addImageAnnotationsChannel(m_gridTopic) &&
           foxglove.addRawChannel(m_deviceInfoTopic, "json") &&
           foxglove.addRawChannel(m_roiZAvgTopic, "json");
}

std::vector<std::byte> StereoDepthTopics::makeDeviceInfoJson(
    const stereo_depth::InputSourceInfo& inputInfo) const {
    std::ostringstream oss;
    oss << "{" << "\"input_mode\":\""
        << (inputInfo.mode == stereo_depth::InputMode::Uvc ? "uvc" : "image_file") << "\",";

    oss << "\"device\":\"" << escapeJsonString(inputInfo.device) << "\",";
    oss << "\"serial_number\":\"" << escapeJsonString(inputInfo.serialNumber) << "\",";
    oss << "\"is_usb\":" << (inputInfo.isUsb ? "true" : "false") << ",";
    oss << "\"width\":" << inputInfo.width << ",";
    oss << "\"height\":" << inputInfo.height << ",";
    oss << "\"fps\":" << inputInfo.fps << "}";

    const std::string json = oss.str();
    std::vector<std::byte> payload(json.size());
    std::memcpy(payload.data(), json.data(), json.size());
    return payload;
}

bool StereoDepthTopics::publishDeviceInfoToFoxglove(
    FoxgloveWrapper& foxglove, const stereo_depth::InputSourceInfo& inputInfo) const {
    const uint64_t nowNs =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count());
    return foxglove.publishRaw(m_deviceInfoTopic, makeDeviceInfoJson(inputInfo), nowNs, false);
}

std::vector<std::string> StereoDepthTopics::collectPublishTopics(const TopicFlags& flags,
                                                                 bool includeRoiZAvgTopic) const {
    std::vector<std::string> topics;
    topics.reserve(7);
    if (includeRoiZAvgTopic) {
        topics.push_back(m_roiZAvgTopic);
    }
    if (flags.rawYuyv) {
        topics.push_back(m_rawYuyvTopic);
    }
    if (flags.rgb) {
        topics.push_back(m_rgbTopic);
    }
    if (flags.depth) {
        topics.push_back(m_depthTopic);
    }
    if (flags.gridAvg) {
        topics.push_back(m_gridAvgTopic);
    }
    topics.push_back(m_gridTopic);
    if (flags.cloud) {
        topics.push_back(m_cloudTopic);
    }
    return topics;
}

std::pair<uint64_t, bool> StereoDepthTopics::publishToFoxglove(
    FoxgloveWrapper& foxglove, const stereo_depth::PipelineOutput& output, const TopicFlags& flags,
    const std::vector<std::byte>* roiZAvgPayload) const {
    uint64_t publishedCount = 0;
    bool anyPublished = false;

    if (roiZAvgPayload != nullptr && !roiZAvgPayload->empty() &&
        foxglove.publishRaw(m_roiZAvgTopic, *roiZAvgPayload, output.frameTimestampNs, true)) {
        ++publishedCount;
        anyPublished = true;
    }

    if (flags.rawYuyv && output.rawYuyvData != nullptr && !output.rawYuyvData->empty() &&
        foxglove.publishImage(m_rawYuyvTopic, output.rawWidth, output.rawHeight,
                              *output.rawYuyvData, "yuyv", m_frameId, output.frameTimestampNs)) {
        ++publishedCount;
        anyPublished = true;
    }

    if (flags.rgb && foxglove.publishImage(m_rgbTopic, stereo_depth::kDewarpImageWidth,
                                           stereo_depth::kDewarpImageHeight, output.rgbData, "bgr8",
                                           m_frameId, output.frameTimestampNs)) {
        ++publishedCount;
        anyPublished = true;
    }

    if (flags.depth && foxglove.publishImage(m_depthTopic, stereo_depth::kDewarpImageWidth,
                                             stereo_depth::kDewarpImageHeight, output.depthData,
                                             "mono16", m_frameId, output.frameTimestampNs)) {
        ++publishedCount;
        anyPublished = true;
    }

    if (flags.gridAvg &&
        foxglove.publishImage(m_gridAvgTopic, stereo_depth::kDewarpImageWidth,
                              stereo_depth::kDewarpImageHeight, output.zGridAvgData, "32FC1",
                              m_frameId, output.frameTimestampNs)) {
        ++publishedCount;
        anyPublished = true;
    }

    if (flags.grid &&
        foxglove.publishImageAnnotations(m_gridTopic, m_gridAnnotations, output.frameTimestampNs)) {
        ++publishedCount;
        anyPublished = true;
    }

    if (flags.cloud &&
        foxglove.publishPointCloud(m_cloudTopic, output.pointCloudData, kPointStride, m_pointFields,
                                   m_frameId, output.frameTimestampNs)) {
        ++publishedCount;
        anyPublished = true;
    }

    return {publishedCount, anyPublished};
}

void StereoDepthTopics::resetMcapChannels(McapChannels& channels) const {
    channels.deviceInfo.reset();
    channels.roiZAvg.reset();
    channels.h264.reset();
    channels.rawYuyv.reset();
    channels.grid.reset();
    channels.cloud.reset();
    channels.gridAvg.reset();
    channels.depth.reset();
    channels.rgb.reset();
}

bool StereoDepthTopics::createMcapChannels(const foxglove::Context& context,
                                           const TopicFlags& flags, McapChannels& channels,
                                           const LogFn& logFn, bool includeCompressedVideo) const {
    auto deviceInfoResult =
        foxglove::RawChannel::create(m_deviceInfoTopic, "json", std::nullopt, context);
    if (!deviceInfoResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("device_info", deviceInfoResult.error(), logFn);
    }
    channels.deviceInfo.emplace(std::move(deviceInfoResult.value()));

    auto roiZAvgResult =
        foxglove::RawChannel::create(m_roiZAvgTopic, "json", std::nullopt, context);
    if (!roiZAvgResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("roi_z_avg", roiZAvgResult.error(), logFn);
    }
    channels.roiZAvg.emplace(std::move(roiZAvgResult.value()));

    if (includeCompressedVideo) {
        auto h264Result = foxglove::schemas::CompressedVideoChannel::create(m_h264Topic, context);
        if (!h264Result.has_value()) {
            resetMcapChannels(channels);
            return logChannelCreateError("h264", h264Result.error(), logFn);
        }
        channels.h264.emplace(std::move(h264Result.value()));
    }

    if (flags.rawYuyv) {
        auto rawYuyvResult = foxglove::schemas::RawImageChannel::create(m_rawYuyvTopic, context);
        if (!rawYuyvResult.has_value()) {
            return logChannelCreateError("yuyv", rawYuyvResult.error(), logFn);
        }
        channels.rawYuyv.emplace(std::move(rawYuyvResult.value()));
    }

    auto rgbResult = foxglove::schemas::RawImageChannel::create(m_rgbTopic, context);
    if (!rgbResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("rgb", rgbResult.error(), logFn);
    }
    channels.rgb.emplace(std::move(rgbResult.value()));

    auto depthResult = foxglove::schemas::RawImageChannel::create(m_depthTopic, context);
    if (!depthResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("depth", depthResult.error(), logFn);
    }
    channels.depth.emplace(std::move(depthResult.value()));

    auto gridAvgResult = foxglove::schemas::RawImageChannel::create(m_gridAvgTopic, context);
    if (!gridAvgResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("z_grid_avg", gridAvgResult.error(), logFn);
    }
    channels.gridAvg.emplace(std::move(gridAvgResult.value()));

    auto cloudResult = foxglove::schemas::PointCloudChannel::create(m_cloudTopic, context);
    if (!cloudResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("pointcloud", cloudResult.error(), logFn);
    }
    channels.cloud.emplace(std::move(cloudResult.value()));

    auto gridResult = foxglove::schemas::ImageAnnotationsChannel::create(m_gridTopic, context);
    if (!gridResult.has_value()) {
        resetMcapChannels(channels);
        return logChannelCreateError("grid", gridResult.error(), logFn);
    }
    channels.grid.emplace(std::move(gridResult.value()));
    return true;
}

void StereoDepthTopics::logDeviceInfoToMcap(McapChannels& channels,
                                            const stereo_depth::InputSourceInfo& inputInfo,
                                            uint64_t timestampNs, const LogFn& logFn) const {
    if (!channels.deviceInfo.has_value()) {
        return;
    }

    const auto payload = makeDeviceInfoJson(inputInfo);
    const auto err = channels.deviceInfo->log(payload.data(), payload.size(), timestampNs);
    if (err != foxglove::FoxgloveError::Ok && logFn) {
        logFn(std::string("recording failed: write device_info ") + foxglove::strerror(err));
    }
}

void StereoDepthTopics::logRoiZAvgToMcap(McapChannels& channels,
                                         const std::vector<std::byte>& payload,
                                         uint64_t timestampNs, const LogFn& logFn) const {
    if (!channels.roiZAvg.has_value() || payload.empty()) {
        return;
    }

    const auto err = channels.roiZAvg->log(payload.data(), payload.size(), timestampNs);
    if (err != foxglove::FoxgloveError::Ok && logFn) {
        logFn(std::string("recording failed: write roi_z_avg ") + foxglove::strerror(err));
    }
}

void StereoDepthTopics::logCompressedVideoToMcap(McapChannels& channels,
                                                 const std::vector<std::byte>& payload,
                                                 uint64_t timestampNs, const LogFn& logFn) const {
    if (!channels.h264.has_value() || payload.empty()) {
        return;
    }

    foxglove::schemas::CompressedVideo msg;
    msg.timestamp = toFoxgloveTimestampNs(timestampNs);
    msg.frame_id = m_frameId;
    msg.data = payload;
    msg.format = "h264";

    const auto err = channels.h264->log(msg, timestampNs);
    if (err != foxglove::FoxgloveError::Ok && logFn) {
        logFn(std::string("recording failed: write h264 ") + foxglove::strerror(err));
    }
}

void StereoDepthTopics::logToMcap(McapChannels& channels,
                                  const stereo_depth::PipelineOutput& output,
                                  const TopicFlags& flags, const LogFn& logFn) const {
    const uint64_t tsNs = output.frameTimestampNs;

    if (flags.rawYuyv && channels.rawYuyv.has_value() && output.rawYuyvData &&
        !output.rawYuyvData->empty()) {
        auto msg =
            makeRawImage(*output.rawYuyvData, output.rawWidth, output.rawHeight, "yuyv", m_frameId);
        const auto err = channels.rawYuyv->log(msg, tsNs);
        if (err != foxglove::FoxgloveError::Ok && logFn) {
            logFn(std::string("recording failed: write yuyv ") + foxglove::strerror(err));
        }
    }

    if (flags.rgb && channels.rgb.has_value()) {
        auto msg = makeRawImage(output.rgbData, stereo_depth::kDewarpImageWidth,
                                stereo_depth::kDewarpImageHeight, "bgr8", m_frameId);
        const auto err = channels.rgb->log(msg, tsNs);
        if (err != foxglove::FoxgloveError::Ok && logFn) {
            logFn(std::string("recording failed: write rgb ") + foxglove::strerror(err));
        }
    }

    if (flags.depth && channels.depth.has_value()) {
        auto msg = makeRawImage(output.depthData, stereo_depth::kDewarpImageWidth,
                                stereo_depth::kDewarpImageHeight, "mono16", m_frameId);
        const auto err = channels.depth->log(msg, tsNs);
        if (err != foxglove::FoxgloveError::Ok && logFn) {
            logFn(std::string("recording failed: write depth ") + foxglove::strerror(err));
        }
    }

    if (flags.gridAvg && channels.gridAvg.has_value()) {
        auto msg = makeRawImage(output.zGridAvgData, stereo_depth::kDewarpImageWidth,
                                stereo_depth::kDewarpImageHeight, "32FC1", m_frameId);
        const auto err = channels.gridAvg->log(msg, tsNs);
        if (err != foxglove::FoxgloveError::Ok && logFn) {
            logFn(std::string("recording failed: write z_grid_avg ") + foxglove::strerror(err));
        }
    }

    if (flags.cloud && channels.cloud.has_value()) {
        foxglove::schemas::PointCloud msg;
        msg.point_stride = kPointStride;
        msg.fields = m_pointFields;
        msg.frame_id = m_frameId;
        msg.data = output.pointCloudData;
        const auto err = channels.cloud->log(msg, tsNs);
        if (err != foxglove::FoxgloveError::Ok && logFn) {
            logFn(std::string("recording failed: write pointcloud ") + foxglove::strerror(err));
        }
    }

    if (flags.grid && channels.grid.has_value()) {
        const auto err = channels.grid->log(m_gridAnnotations, tsNs);
        if (err != foxglove::FoxgloveError::Ok && logFn) {
            logFn(std::string("recording failed: write grid ") + foxglove::strerror(err));
        }
    }
}

}  // namespace stereo_depth::app
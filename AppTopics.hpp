#pragma once

#include "FoxgloveWrapper.hpp"
#include "FrameInputSource.hpp"
#include "StereoDepthPipeline.hpp"

#include <foxglove/channel.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/schemas.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace stereo_depth::app {

struct TopicFlags {
    bool rgb = false;
    bool depth = false;
    bool gridAvg = false;
    bool grid = true;
    bool cloud = false;
    bool rawYuyv = false;
};

TopicFlags makeTopicFlags(const stereo_depth::PipelineOutput& output);

class StereoDepthTopics {
public:
    struct McapChannels {
        std::optional<foxglove::RawChannel> deviceInfo;
        std::optional<foxglove::RawChannel> roiZAvg;
        std::optional<foxglove::schemas::CompressedVideoChannel> h264;
        std::optional<foxglove::schemas::RawImageChannel> rawYuyv;
        std::optional<foxglove::schemas::RawImageChannel> rgb;
        std::optional<foxglove::schemas::RawImageChannel> depth;
        std::optional<foxglove::schemas::RawImageChannel> gridAvg;
        std::optional<foxglove::schemas::PointCloudChannel> cloud;
        std::optional<foxglove::schemas::ImageAnnotationsChannel> grid;
    };

    using LogFn = std::function<void(const std::string&)>;

    StereoDepthTopics();

    bool registerFoxgloveChannels(FoxgloveWrapper& foxglove) const;
    bool publishDeviceInfoToFoxglove(FoxgloveWrapper& foxglove,
                                     const stereo_depth::InputSourceInfo& inputInfo) const;
    std::vector<std::string> collectPublishTopics(const TopicFlags& flags,
                                                  bool includeRoiZAvgTopic) const;
    std::pair<uint64_t, bool> publishToFoxglove(
        FoxgloveWrapper& foxglove, const stereo_depth::PipelineOutput& output,
        const TopicFlags& flags, const std::vector<std::byte>* roiZAvgPayload = nullptr) const;

    void resetMcapChannels(McapChannels& channels) const;
    bool createMcapChannels(const foxglove::Context& context, const TopicFlags& flags,
                            McapChannels& channels, const LogFn& logFn,
                            bool includeCompressedVideo = false) const;
    void logDeviceInfoToMcap(McapChannels& channels, const stereo_depth::InputSourceInfo& inputInfo,
                             uint64_t timestampNs, const LogFn& logFn) const;
    void logRoiZAvgToMcap(McapChannels& channels, const std::vector<std::byte>& payload,
                          uint64_t timestampNs, const LogFn& logFn) const;
    void logCompressedVideoToMcap(McapChannels& channels, const std::vector<std::byte>& payload,
                                  uint64_t timestampNs, const LogFn& logFn) const;
    void logToMcap(McapChannels& channels, const stereo_depth::PipelineOutput& output,
                   const TopicFlags& flags, const LogFn& logFn) const;

    const std::string& rgbTopic() const { return m_rgbTopic; }
    const std::string& depthTopic() const { return m_depthTopic; }
    const std::string& gridAvgTopic() const { return m_gridAvgTopic; }
    const std::string& cloudTopic() const { return m_cloudTopic; }
    const std::string& gridTopic() const { return m_gridTopic; }
    const std::string& deviceInfoTopic() const { return m_deviceInfoTopic; }
    const std::string& roiZAvgTopic() const { return m_roiZAvgTopic; }
    const std::string& rawYuyvTopic() const { return m_rawYuyvTopic; }
    const std::string& h264Topic() const { return m_h264Topic; }
    const std::string& frameId() const { return m_frameId; }

private:
    std::vector<std::byte> makeDeviceInfoJson(const stereo_depth::InputSourceInfo& inputInfo) const;

    std::string m_rgbTopic;
    std::string m_depthTopic;
    std::string m_gridAvgTopic;
    std::string m_cloudTopic;
    std::string m_gridTopic;
    std::string m_deviceInfoTopic;
    std::string m_roiZAvgTopic;
    std::string m_rawYuyvTopic;
    std::string m_h264Topic;
    std::string m_frameId;
    foxglove::schemas::ImageAnnotations m_gridAnnotations;
    std::vector<foxglove::schemas::PackedElementField> m_pointFields;
};

}  // namespace stereo_depth::app
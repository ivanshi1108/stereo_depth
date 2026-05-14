#pragma once

#include <chrono>
#include <foxglove/channel.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

class FoxgloveWrapper {
public:
    struct PublishStats {
        uint64_t published = 0;
        uint64_t dropped = 0;
    };

    struct SubscriberStats {
        uint64_t total = 0;
        std::map<std::string, uint64_t> topic_counts;
    };

    struct Options {
        std::string host;
        uint16_t port;
        uint32_t max_publish_fps;
        size_t message_backlog_size;
        Options() : host("0.0.0.0"), port(8765), max_publish_fps(15), message_backlog_size(0) {}
    };

    static FoxgloveWrapper& getInstance();

    // Delete copy and move constructors and assign operators
    FoxgloveWrapper(const FoxgloveWrapper&) = delete;
    FoxgloveWrapper& operator=(const FoxgloveWrapper&) = delete;
    FoxgloveWrapper(FoxgloveWrapper&&) = delete;
    FoxgloveWrapper& operator=(FoxgloveWrapper&&) = delete;

    // Start the server
    bool start(const Options& options = Options());

    // Stop the server
    void stop();

    // Check if server is running
    bool isRunning() const;

    // Add a channel for publishing raw images
    // Returns true if successful or channel already exists
    bool addImageChannel(const std::string& topic);

    // Publish a raw image to a topic
    // Returns false if channel doesn't exist
    bool publishImage(const std::string& topic, uint32_t width, uint32_t height,
                      const std::vector<std::byte>& data, const std::string& encoding = "mono8",
                      const std::string& frame_id = "", uint64_t frameTimestampNs = 0);

    // Add a channel for publishing compressed images
    bool addCompressedImageChannel(const std::string& topic);

    // Publish a compressed image (e.g., PNG/JPEG)
    bool publishCompressedImage(const std::string& topic, const std::vector<std::byte>& data,
                                const std::string& format, const std::string& frame_id = "");

    // Add a channel for publishing point clouds
    bool addPointCloudChannel(const std::string& topic);

    // Publish a point cloud (xyz + rgb float32)
    bool publishPointCloud(const std::string& topic, const std::vector<std::byte>& data,
                           uint32_t point_stride,
                           const std::vector<foxglove::schemas::PackedElementField>& fields,
                           const std::string& frame_id = "", uint64_t frameTimestampNs = 0);

    // Add a channel for publishing image annotations
    bool addImageAnnotationsChannel(const std::string& topic);

    // Add a raw channel for JSON or other custom payloads.
    bool addRawChannel(const std::string& topic, const std::string& messageEncoding,
                       std::optional<foxglove::Schema> schema = std::nullopt);

    // Publish raw bytes to a topic.
    bool publishRaw(const std::string& topic, const std::vector<std::byte>& data,
                    uint64_t timestampNs = 0, bool requireSubscribers = true);

    // Add a channel for publishing key-value metadata entries.
    bool addKeyValueChannel(const std::string& topic);

    // Publish a single key-value metadata entry.
    bool publishKeyValue(const std::string& topic, const std::string& key, const std::string& value,
                         uint64_t timestampNs = 0);

    // Publish image annotations to be overlaid in the Image panel
    bool publishImageAnnotations(const std::string& topic,
                                 const foxglove::schemas::ImageAnnotations& annotations,
                                 uint64_t frameTimestampNs = 0);

    // Check if a topic has any subscribers
    bool hasSubscribers(const std::string& topic);

    // Check whether a topic currently has any downstream subscribers.
    bool shouldPublish(const std::string& topic);

    // Check whether any topic in the set currently has downstream subscribers.
    bool hasAnySubscribers(const std::vector<std::string>& topics);

    // Get the current subscriber count for a topic.
    uint64_t getSubscriberCount(const std::string& topic) const;

    // Get the subscribe-event generation for a topic.
    uint64_t getSubscribeGeneration(const std::string& topic) const;

    // Get and reset publish/drop stats since last call
    PublishStats consumePublishStats();

    // Get current subscriber stats
    SubscriberStats getSubscriberStats() const;

private:
    FoxgloveWrapper() = default;
    ~FoxgloveWrapper() = default;

    bool shouldPublishLocked(const std::string& topic);

    std::optional<foxglove::WebSocketServer> server_;
    Options options_;
    std::map<std::string, std::unique_ptr<foxglove::schemas::RawImageChannel>> image_channels_;
    std::map<std::string, std::unique_ptr<foxglove::schemas::CompressedImageChannel>>
        compressed_image_channels_;
    std::map<std::string, std::unique_ptr<foxglove::schemas::PointCloudChannel>>
        point_cloud_channels_;
    std::map<std::string, std::unique_ptr<foxglove::schemas::ImageAnnotationsChannel>>
        image_annotations_channels_;
    std::map<std::string, std::unique_ptr<foxglove::RawChannel>> raw_channels_;
    std::map<std::string, std::unique_ptr<foxglove::schemas::KeyValuePairChannel>>
        key_value_channels_;
    std::map<std::string, std::chrono::steady_clock::time_point> last_publish_time_;
    PublishStats publish_stats_;

    std::map<uint64_t, int> channel_subscriber_counts_;
    std::map<std::string, uint64_t> topic_subscriber_counts_;
    std::map<std::string, uint64_t> topic_subscribe_generations_;
    uint64_t total_subscriber_count_ = 0;
    mutable std::mutex mutex_;
};

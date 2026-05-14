#include "FoxgloveWrapper.hpp"

#define SAMPLE_LOG_TAG "FOXGLOVE"
#include "sample_log.h"

#include <chrono>
#include <iostream>

namespace {

uint64_t normalizeTimestampNs(uint64_t frameTimestampNs) {
    if (frameTimestampNs != 0) {
        return frameTimestampNs;
    }
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

foxglove::schemas::Timestamp toFoxgloveTimestamp(uint64_t tsNs) {
    foxglove::schemas::Timestamp ts;
    ts.sec = static_cast<uint32_t>(tsNs / 1000000000ULL);
    ts.nsec = static_cast<uint32_t>(tsNs % 1000000000ULL);
    return ts;
}

}  // namespace

FoxgloveWrapper& FoxgloveWrapper::getInstance() {
    static FoxgloveWrapper instance;
    return instance;
}

bool FoxgloveWrapper::start(const Options& options) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (server_.has_value()) {
        ALOGW("Foxglove server is already running");
        return true;
    }
    options_ = options;
    last_publish_time_.clear();
    publish_stats_ = PublishStats{};
    topic_subscriber_counts_.clear();
    topic_subscribe_generations_.clear();
    total_subscriber_count_ = 0;

    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = options.host;
    ws_options.port = options.port;
    ws_options.supported_encodings = {"json"};
    ws_options.message_backlog_size = options.message_backlog_size;

    ws_options.callbacks.onSubscribe = [this](uint64_t channel_id,
                                              const foxglove::ClientMetadata& client) {
        std::lock_guard<std::mutex> lock(mutex_);
        (void)client;
        channel_subscriber_counts_[channel_id]++;
        total_subscriber_count_++;

        auto updateTopicCount = [&](const auto& channels) {
            for (const auto& entry : channels) {
                if (entry.second->id() == channel_id) {
                    topic_subscriber_counts_[entry.first] =
                        static_cast<uint64_t>(std::max(0, channel_subscriber_counts_[channel_id]));
                    topic_subscribe_generations_[entry.first]++;
                    return true;
                }
            }
            return false;
        };

        if (updateTopicCount(image_channels_) || updateTopicCount(compressed_image_channels_) ||
            updateTopicCount(point_cloud_channels_) ||
            updateTopicCount(image_annotations_channels_) || updateTopicCount(raw_channels_) ||
            updateTopicCount(key_value_channels_)) {
            return;
        }
    };
    ws_options.callbacks.onUnsubscribe = [this](uint64_t channel_id,
                                                const foxglove::ClientMetadata& client) {
        std::lock_guard<std::mutex> lock(mutex_);
        (void)client;
        if (channel_subscriber_counts_[channel_id] > 0) {
            channel_subscriber_counts_[channel_id]--;
            if (total_subscriber_count_ > 0) {
                total_subscriber_count_--;
            }
        }

        auto updateTopicCount = [&](const auto& channels) {
            for (const auto& entry : channels) {
                if (entry.second->id() == channel_id) {
                    topic_subscriber_counts_[entry.first] =
                        static_cast<uint64_t>(std::max(0, channel_subscriber_counts_[channel_id]));
                    return true;
                }
            }
            return false;
        };

        if (updateTopicCount(image_channels_) || updateTopicCount(compressed_image_channels_) ||
            updateTopicCount(point_cloud_channels_) ||
            updateTopicCount(image_annotations_channels_) || updateTopicCount(raw_channels_) ||
            updateTopicCount(key_value_channels_)) {
            return;
        }
    };

    auto server_result = foxglove::WebSocketServer::create(std::move(ws_options));
    if (!server_result.has_value()) {
        ALOGE("failed to create server: %s", foxglove::strerror(server_result.error()));
        return false;
    }

    server_ = std::move(server_result.value());
    ALOGN("Foxglove server started on port %u", server_->port());
    return true;
}

void FoxgloveWrapper::stop() {
    std::optional<foxglove::WebSocketServer> serverToStop;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        image_channels_.clear();
        compressed_image_channels_.clear();
        point_cloud_channels_.clear();
        image_annotations_channels_.clear();
        raw_channels_.clear();
        key_value_channels_.clear();
        channel_subscriber_counts_.clear();
        topic_subscriber_counts_.clear();
        topic_subscribe_generations_.clear();
        total_subscriber_count_ = 0;
        last_publish_time_.clear();
        publish_stats_ = PublishStats{};
        serverToStop = std::move(server_);
        server_.reset();
    }

    serverToStop.reset();
    ALOGN("Foxglove server stopped");
}

bool FoxgloveWrapper::isRunning() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return server_.has_value();
}

bool FoxgloveWrapper::addImageChannel(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (image_channels_.find(topic) != image_channels_.end()) {
        return true;
    }

    auto channel_result = foxglove::schemas::RawImageChannel::create(topic);
    if (!channel_result.has_value()) {
        ALOGE("failed to create image channel '%s': %s", topic.c_str(),
              foxglove::strerror(channel_result.error()));
        return false;
    }

    image_channels_[topic] =
        std::make_unique<foxglove::schemas::RawImageChannel>(std::move(channel_result.value()));
    topic_subscriber_counts_[topic] = 0;
    topic_subscribe_generations_[topic] = 0;
    return true;
}

bool FoxgloveWrapper::publishImage(const std::string& topic, uint32_t width, uint32_t height,
                                   const std::vector<std::byte>& data, const std::string& encoding,
                                   const std::string& frame_id, uint64_t frameTimestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = image_channels_.find(topic);
    if (it == image_channels_.end()) {
        ALOGW("topic '%s' not found. Call addImageChannel first", topic.c_str());
        return false;
    }

    if (!shouldPublishLocked(topic)) {
        return false;
    }

    foxglove::schemas::RawImage msg;
    msg.width = width;
    msg.height = height;

    if (height > 0) {
        msg.step = static_cast<uint32_t>(data.size() / height);
    } else {
        msg.step = 0;
    }

    msg.encoding = encoding;
    msg.frame_id = frame_id;
    msg.data = data;

    const uint64_t tsNs = normalizeTimestampNs(frameTimestampNs);
    msg.timestamp = toFoxgloveTimestamp(tsNs);

    it->second->log(msg, tsNs);
    ++publish_stats_.published;
    return true;
}

bool FoxgloveWrapper::addCompressedImageChannel(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (compressed_image_channels_.find(topic) != compressed_image_channels_.end()) {
        return true;
    }

    auto channel_result = foxglove::schemas::CompressedImageChannel::create(topic);
    if (!channel_result.has_value()) {
        ALOGE("failed to create compressed image channel '%s': %s", topic.c_str(),
              foxglove::strerror(channel_result.error()));
        return false;
    }

    compressed_image_channels_[topic] = std::make_unique<foxglove::schemas::CompressedImageChannel>(
        std::move(channel_result.value()));
    topic_subscriber_counts_[topic] = 0;
    topic_subscribe_generations_[topic] = 0;
    return true;
}

bool FoxgloveWrapper::publishCompressedImage(const std::string& topic,
                                             const std::vector<std::byte>& data,
                                             const std::string& format,
                                             const std::string& frame_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = compressed_image_channels_.find(topic);
    if (it == compressed_image_channels_.end()) {
        ALOGW("topic '%s' not found. Call addCompressedImageChannel first", topic.c_str());
        return false;
    }

    if (!shouldPublishLocked(topic)) {
        return false;
    }

    foxglove::schemas::CompressedImage msg;
    msg.format = format;
    msg.frame_id = frame_id;
    msg.data = data;

    auto now = std::chrono::system_clock::now();
    auto time_since_epoch = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time_since_epoch);
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch - seconds);

    foxglove::schemas::Timestamp ts;
    ts.sec = static_cast<uint32_t>(seconds.count());
    ts.nsec = static_cast<uint32_t>(nanoseconds.count());
    msg.timestamp = ts;

    it->second->log(msg);
    ++publish_stats_.published;
    return true;
}

bool FoxgloveWrapper::addPointCloudChannel(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (point_cloud_channels_.find(topic) != point_cloud_channels_.end()) {
        return true;
    }

    auto channel_result = foxglove::schemas::PointCloudChannel::create(topic);
    if (!channel_result.has_value()) {
        ALOGE("failed to create point cloud channel '%s': %s", topic.c_str(),
              foxglove::strerror(channel_result.error()));
        return false;
    }

    point_cloud_channels_[topic] =
        std::make_unique<foxglove::schemas::PointCloudChannel>(std::move(channel_result.value()));
    topic_subscriber_counts_[topic] = 0;
    topic_subscribe_generations_[topic] = 0;
    return true;
}

bool FoxgloveWrapper::publishPointCloud(
    const std::string& topic, const std::vector<std::byte>& data, uint32_t point_stride,
    const std::vector<foxglove::schemas::PackedElementField>& fields, const std::string& frame_id,
    uint64_t frameTimestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = point_cloud_channels_.find(topic);
    if (it == point_cloud_channels_.end()) {
        ALOGW("topic '%s' not found. Call addPointCloudChannel first", topic.c_str());
        return false;
    }

    if (!shouldPublishLocked(topic)) {
        return false;
    }

    foxglove::schemas::PointCloud msg;
    msg.point_stride = point_stride;
    msg.fields = fields;
    msg.frame_id = frame_id;
    msg.data = data;

    const uint64_t tsNs = normalizeTimestampNs(frameTimestampNs);
    msg.timestamp = toFoxgloveTimestamp(tsNs);

    it->second->log(msg, tsNs);
    ++publish_stats_.published;
    return true;
}

bool FoxgloveWrapper::addImageAnnotationsChannel(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (image_annotations_channels_.find(topic) != image_annotations_channels_.end()) {
        return true;
    }

    auto channel_result = foxglove::schemas::ImageAnnotationsChannel::create(topic);
    if (!channel_result.has_value()) {
        ALOGE("failed to create image annotations channel '%s': %s", topic.c_str(),
              foxglove::strerror(channel_result.error()));
        return false;
    }

    image_annotations_channels_[topic] =
        std::make_unique<foxglove::schemas::ImageAnnotationsChannel>(
            std::move(channel_result.value()));
    topic_subscriber_counts_[topic] = 0;
    topic_subscribe_generations_[topic] = 0;
    return true;
}

bool FoxgloveWrapper::addRawChannel(const std::string& topic, const std::string& messageEncoding,
                                    std::optional<foxglove::Schema> schema) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (raw_channels_.find(topic) != raw_channels_.end()) {
        return true;
    }

    auto channel_result =
        foxglove::RawChannel::create(topic, messageEncoding, schema, foxglove::Context());
    if (!channel_result.has_value()) {
        ALOGE("failed to create raw channel '%s': %s", topic.c_str(),
              foxglove::strerror(channel_result.error()));
        return false;
    }

    raw_channels_[topic] =
        std::make_unique<foxglove::RawChannel>(std::move(channel_result.value()));
    topic_subscriber_counts_[topic] = 0;
    topic_subscribe_generations_[topic] = 0;
    ALOGN("registered raw channel '%s' encoding='%s'", topic.c_str(), messageEncoding.c_str());
    return true;
}

bool FoxgloveWrapper::publishRaw(const std::string& topic, const std::vector<std::byte>& data,
                                 uint64_t timestampNs, bool requireSubscribers) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = raw_channels_.find(topic);
    if (it == raw_channels_.end()) {
        ALOGW("topic '%s' not found. Call addRawChannel first", topic.c_str());
        return false;
    }

    if (data.empty()) {
        ALOGW("raw payload for '%s' is empty", topic.c_str());
        return false;
    }

    if (requireSubscribers && !shouldPublishLocked(topic)) {
        return false;
    }

    const uint64_t tsNs = normalizeTimestampNs(timestampNs);
    const auto err = it->second->log(data.data(), data.size(), tsNs);
    if (err != foxglove::FoxgloveError::Ok) {
        ALOGE("failed to publish raw message on '%s': %s", topic.c_str(), foxglove::strerror(err));
        return false;
    }

    ++publish_stats_.published;
    return true;
}

bool FoxgloveWrapper::addKeyValueChannel(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (key_value_channels_.find(topic) != key_value_channels_.end()) {
        return true;
    }

    auto channel_result = foxglove::schemas::KeyValuePairChannel::create(topic);
    if (!channel_result.has_value()) {
        ALOGE("failed to create key-value channel '%s': %s", topic.c_str(),
              foxglove::strerror(channel_result.error()));
        return false;
    }

    key_value_channels_[topic] =
        std::make_unique<foxglove::schemas::KeyValuePairChannel>(std::move(channel_result.value()));
    topic_subscriber_counts_[topic] = 0;
    topic_subscribe_generations_[topic] = 0;
    return true;
}

bool FoxgloveWrapper::publishKeyValue(const std::string& topic, const std::string& key,
                                      const std::string& value, uint64_t timestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = key_value_channels_.find(topic);
    if (it == key_value_channels_.end()) {
        ALOGW("topic '%s' not found. Call addKeyValueChannel first", topic.c_str());
        return false;
    }

    if (!shouldPublishLocked(topic)) {
        return false;
    }

    foxglove::schemas::KeyValuePair msg;
    msg.key = key;
    msg.value = value;

    const uint64_t tsNs = normalizeTimestampNs(timestampNs);
    const auto err = it->second->log(msg, tsNs);
    if (err != foxglove::FoxgloveError::Ok) {
        ALOGE("failed to publish key-value on '%s': %s", topic.c_str(), foxglove::strerror(err));
        return false;
    }

    ++publish_stats_.published;
    return true;
}

bool FoxgloveWrapper::publishImageAnnotations(
    const std::string& topic, const foxglove::schemas::ImageAnnotations& annotations,
    uint64_t frameTimestampNs) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = image_annotations_channels_.find(topic);
    if (it == image_annotations_channels_.end()) {
        ALOGW("topic '%s' not found. Call addImageAnnotationsChannel first", topic.c_str());
        return false;
    }

    if (!shouldPublishLocked(topic)) {
        return false;
    }

    const uint64_t tsNs = normalizeTimestampNs(frameTimestampNs);
    it->second->log(annotations, tsNs);
    ++publish_stats_.published;
    return true;
}

bool FoxgloveWrapper::shouldPublishLocked(const std::string& topic) {
    if (topic_subscriber_counts_.empty()) {
        return false;
    }

    if (auto it = topic_subscriber_counts_.find(topic); it != topic_subscriber_counts_.end()) {
        return it->second > 0;
    }

    return false;
}

bool FoxgloveWrapper::shouldPublish(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    return shouldPublishLocked(topic);
}

bool FoxgloveWrapper::hasAnySubscribers(const std::vector<std::string>& topics) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const std::string& topic : topics) {
        if (shouldPublishLocked(topic)) {
            return true;
        }
    }
    return false;
}

uint64_t FoxgloveWrapper::getSubscriberCount(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = topic_subscriber_counts_.find(topic); it != topic_subscriber_counts_.end()) {
        return it->second;
    }
    return 0;
}

uint64_t FoxgloveWrapper::getSubscribeGeneration(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = topic_subscribe_generations_.find(topic);
        it != topic_subscribe_generations_.end()) {
        return it->second;
    }
    return 0;
}

FoxgloveWrapper::PublishStats FoxgloveWrapper::consumePublishStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishStats stats = publish_stats_;
    publish_stats_ = PublishStats{};
    return stats;
}

FoxgloveWrapper::SubscriberStats FoxgloveWrapper::getSubscriberStats() const {
    std::lock_guard<std::mutex> lock(mutex_);

    SubscriberStats stats;
    stats.total = total_subscriber_count_;
    stats.topic_counts = topic_subscriber_counts_;

    return stats;
}

bool FoxgloveWrapper::hasSubscribers(const std::string& topic) { return shouldPublish(topic); }

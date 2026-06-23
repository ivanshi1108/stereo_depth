#pragma once

#include <linux/videodev2.h>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

class V4L2Capture {
public:
    struct DeviceInfo {
        std::string devicePath;
        std::string serialNumber;
        bool isUsb = false;
    };

    struct Frame {
        void* data;
        size_t size;
        uint32_t index;
        uint64_t timestampNs;
    };

    struct UvcControlSettings {
        std::optional<int32_t> brightness;
        std::optional<int32_t> contrast;
        std::optional<int32_t> saturation;
        std::optional<int32_t> gamma;
        std::optional<int32_t> sharpness;
        std::optional<int32_t> autoWhiteBalance;
        std::optional<int32_t> whiteBalanceTemperature;
        std::optional<int32_t> powerLineFrequency;
        std::optional<int32_t> gain;
    };

    V4L2Capture(const std::string& device, int width, int height, int fps,
                const UvcControlSettings& controls = {});
    ~V4L2Capture();

    // Init device, set format. Returns negotiated frame size.
    size_t initialize();

    // Start streaming with MMAP capture buffers.
    bool start();
    void stop();

    // Grab a frame (DQBUF). Returns false if failed or timeout.
    bool grab(Frame& frame);

    // Release a frame (QBUF).
    void release(const Frame& frame);

    static bool printSupportedModes(const std::string& device);
    static bool printAllControls(const std::string& device);
    static bool resetAllControlsToDefault(const std::string& device);
    static DeviceInfo probeDeviceInfo(const std::string& device);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getFps() const { return fps_; }
    size_t getFrameSize() const { return frame_size_; }
    const DeviceInfo& getDeviceInfo() const { return device_info_; }

private:
    std::string device_;
    int width_;
    int height_;
    int fps_;
    int fd_;
    bool is_started_;

    static const int MAX_BUFS = 6;
    void* buffers_[MAX_BUFS];
    size_t buffer_lengths_[MAX_BUFS];
    uint32_t buffer_count_;
    size_t frame_size_;
    DeviceInfo device_info_;
    UvcControlSettings controls_;
};

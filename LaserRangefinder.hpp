#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace stereo_depth::app {

struct LaserDistanceSample {
    int32_t distanceMm = -1;
    uint64_t sampleTimestampNs = 0;

    bool valid() const { return distanceMm >= 0; }
};

class LaserRangefinder {
public:
    LaserRangefinder();
    ~LaserRangefinder();

    LaserRangefinder(const LaserRangefinder&) = delete;
    LaserRangefinder& operator=(const LaserRangefinder&) = delete;

    LaserDistanceSample readSample(uint64_t frameTimestampNs);

private:
    bool ensureDeviceOpen();
    void closeDevice();
    bool configurePort(int fd) const;
    bool waitForReadable(int timeoutMs);
    bool fillRxBuffer(int timeoutMs);
    bool tryExtractDistanceMm(int32_t& distanceMm);

    std::string devicePath_;
    int fd_ = -1;
    std::vector<uint8_t> rxBuffer_;
    int32_t lastDistanceMm_ = -1;
    uint64_t lastSampleSteadyNs_ = 0;
    bool loggedOpenFailure_ = false;
    bool loggedReadFailure_ = false;
};

}  // namespace stereo_depth::app
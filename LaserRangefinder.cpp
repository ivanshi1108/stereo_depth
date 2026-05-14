#include "LaserRangefinder.hpp"

#define SAMPLE_LOG_TAG "LRF"
#include "sample_log.h"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>

namespace stereo_depth::app {

namespace {

constexpr char kDefaultDevicePath[] = "/dev/ttyUSB0";
constexpr uint8_t kDeviceAddress = 0x01;
constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint8_t kDistancePayloadBytes = 0x04;
constexpr size_t kFrameBytes = 9;
constexpr int kReadTimeoutMs = 20;
constexpr uint64_t kSampleHoldNs = 500000000ULL;

uint64_t nowSteadyNs() {
    timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

uint16_t computeModbusCrc(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

int32_t decodeDistanceMm(const uint8_t* payload) {
    const uint32_t rawValue =
        (static_cast<uint32_t>(payload[0]) << 24U) | (static_cast<uint32_t>(payload[1]) << 16U) |
        (static_cast<uint32_t>(payload[2]) << 8U) | static_cast<uint32_t>(payload[3]);
    return static_cast<int32_t>((rawValue + 5U) / 10U);
}

}  // namespace

LaserRangefinder::LaserRangefinder() : devicePath_(kDefaultDevicePath) { rxBuffer_.reserve(64); }

LaserRangefinder::~LaserRangefinder() { closeDevice(); }

bool LaserRangefinder::configurePort(int fd) const {
    termios tty = {};
    if (::tcgetattr(fd, &tty) != 0) {
        ALOGW("tcgetattr(%s) failed: %s", devicePath_.c_str(), std::strerror(errno));
        return false;
    }

    ::cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::cfsetispeed(&tty, B9600) != 0 || ::cfsetospeed(&tty, B9600) != 0) {
        ALOGW("failed to set baud rate on %s: %s", devicePath_.c_str(), std::strerror(errno));
        return false;
    }

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        ALOGW("tcsetattr(%s) failed: %s", devicePath_.c_str(), std::strerror(errno));
        return false;
    }

    ::tcflush(fd, TCIFLUSH);
    return true;
}

bool LaserRangefinder::ensureDeviceOpen() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(devicePath_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        if (!loggedOpenFailure_) {
            ALOGW("open %s failed: %s", devicePath_.c_str(), std::strerror(errno));
            loggedOpenFailure_ = true;
        }
        return false;
    }

    if (!configurePort(fd_)) {
        closeDevice();
        return false;
    }

    loggedOpenFailure_ = false;
    loggedReadFailure_ = false;
    rxBuffer_.clear();
    ALOGN("laser rangefinder opened on %s (9600 8N1)", devicePath_.c_str());
    return true;
}

void LaserRangefinder::closeDevice() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    rxBuffer_.clear();
}

bool LaserRangefinder::waitForReadable(int timeoutMs) {
    if (fd_ < 0) {
        return false;
    }

    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(fd_, &readSet);

    timeval timeout = {};
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    const int ready = ::select(fd_ + 1, &readSet, nullptr, nullptr, &timeout);
    if (ready < 0 && errno != EINTR) {
        if (!loggedReadFailure_) {
            ALOGW("select(%s) failed: %s", devicePath_.c_str(), std::strerror(errno));
            loggedReadFailure_ = true;
        }
        closeDevice();
    }
    return ready > 0;
}

bool LaserRangefinder::fillRxBuffer(int timeoutMs) {
    if (!waitForReadable(timeoutMs)) {
        return false;
    }

    std::array<uint8_t, 64> chunk = {};
    const ssize_t bytesRead = ::read(fd_, chunk.data(), chunk.size());
    if (bytesRead > 0) {
        loggedReadFailure_ = false;
        rxBuffer_.insert(rxBuffer_.end(), chunk.begin(), chunk.begin() + bytesRead);
        return true;
    }

    if (bytesRead < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        return false;
    }

    if (!loggedReadFailure_) {
        ALOGW("read(%s) failed: %s", devicePath_.c_str(),
              bytesRead == 0 ? "device disconnected" : std::strerror(errno));
        loggedReadFailure_ = true;
    }
    closeDevice();
    return false;
}

bool LaserRangefinder::tryExtractDistanceMm(int32_t& distanceMm) {
    while (rxBuffer_.size() >= kFrameBytes) {
        if (rxBuffer_[0] != kDeviceAddress || rxBuffer_[1] != kReadFunctionCode ||
            rxBuffer_[2] != kDistancePayloadBytes) {
            rxBuffer_.erase(rxBuffer_.begin());
            continue;
        }

        const uint16_t expectedCrc = computeModbusCrc(rxBuffer_.data(), kFrameBytes - 2);
        const uint16_t actualCrc = static_cast<uint16_t>(rxBuffer_[kFrameBytes - 2]) |
                                   (static_cast<uint16_t>(rxBuffer_[kFrameBytes - 1]) << 8U);
        if (expectedCrc != actualCrc) {
            ALOGW("discard laser rangefinder packet with invalid CRC: expected 0x%04X, got 0x%04X",
                  expectedCrc, actualCrc);
            rxBuffer_.erase(rxBuffer_.begin());
            continue;
        }

        distanceMm = decodeDistanceMm(&rxBuffer_[3]);
        rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.begin() + kFrameBytes);
        return true;
    }

    return false;
}

LaserDistanceSample LaserRangefinder::readSample(uint64_t frameTimestampNs) {
    LaserDistanceSample sample;
    sample.sampleTimestampNs = frameTimestampNs;

    if (!ensureDeviceOpen()) {
        return sample;
    }

    int32_t distanceMm = -1;
    bool gotFreshSample = false;

    while (tryExtractDistanceMm(distanceMm)) {
        gotFreshSample = true;
    }

    if (!gotFreshSample && fillRxBuffer(kReadTimeoutMs)) {
        while (tryExtractDistanceMm(distanceMm)) {
            gotFreshSample = true;
        }
    }

    if (gotFreshSample) {
        lastDistanceMm_ = distanceMm;
        lastSampleSteadyNs_ = nowSteadyNs();
        sample.distanceMm = distanceMm;
        return sample;
    }

    if (lastDistanceMm_ >= 0) {
        const uint64_t ageNs = nowSteadyNs() - lastSampleSteadyNs_;
        if (ageNs <= kSampleHoldNs) {
            sample.distanceMm = lastDistanceMm_;
        }
    }

    return sample;
}

}  // namespace stereo_depth::app
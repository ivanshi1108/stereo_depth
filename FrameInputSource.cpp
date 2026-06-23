#include "FrameInputSource.hpp"

#include "AxVdecH264Decoder.hpp"

#include "StereoDepthPipeline.hpp"

#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#define SAMPLE_LOG_TAG "INPUT"
#include "sample_log.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <thread>
#include <unordered_map>

namespace stereo_depth {
namespace {

constexpr char kRawYuyvTopic[] = "/camera/yuyv";
constexpr char kH264Topic[] = "/camera/h264";
constexpr char kDeviceInfoTopic[] = "/camera/device_info";
constexpr char kRoiZAvgTopic[] = "/camera/roi_z_avg";

bool isH264Topic(std::string_view topic) { return topic == kH264Topic; }

bool isRecoverableH264DecodeError(const std::string& error) {
    return error.find("0x80080181") != std::string::npos ||
           error.find("AX_VDEC_GetChnFrame failed") != std::string::npos ||
           error.find("AX_VDEC_SendStream failed") != std::string::npos ||
           error.find("timed out waiting for decoded H.264 frame") != std::string::npos;
}

struct DecodedRawImage {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t step = 0;
    std::string encoding;
    std::vector<std::byte> data;
};

struct DecodedCompressedVideo {
    std::string format;
    std::vector<std::byte> data;
};

class ByteCursor {
public:
    ByteCursor(const std::byte* data, size_t size) : m_data(data), m_size(size) {}

    size_t remaining() const { return m_size - m_offset; }

    bool readU8(uint8_t& value) {
        if (remaining() < 1) {
            return false;
        }
        value = static_cast<uint8_t>(m_data[m_offset]);
        ++m_offset;
        return true;
    }

    bool readU16(uint16_t& value) {
        if (remaining() < sizeof(uint16_t)) {
            return false;
        }
        value = static_cast<uint16_t>(static_cast<uint8_t>(m_data[m_offset])) |
                (static_cast<uint16_t>(static_cast<uint8_t>(m_data[m_offset + 1])) << 8);
        m_offset += sizeof(uint16_t);
        return true;
    }

    bool readU32(uint32_t& value) {
        if (remaining() < sizeof(uint32_t)) {
            return false;
        }
        value = static_cast<uint32_t>(static_cast<uint8_t>(m_data[m_offset])) |
                (static_cast<uint32_t>(static_cast<uint8_t>(m_data[m_offset + 1])) << 8) |
                (static_cast<uint32_t>(static_cast<uint8_t>(m_data[m_offset + 2])) << 16) |
                (static_cast<uint32_t>(static_cast<uint8_t>(m_data[m_offset + 3])) << 24);
        m_offset += sizeof(uint32_t);
        return true;
    }

    bool readU64(uint64_t& value) {
        if (remaining() < sizeof(uint64_t)) {
            return false;
        }
        value = static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset])) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 1])) << 8) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 2])) << 16) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 3])) << 24) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 4])) << 32) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 5])) << 40) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 6])) << 48) |
                (static_cast<uint64_t>(static_cast<uint8_t>(m_data[m_offset + 7])) << 56);
        m_offset += sizeof(uint64_t);
        return true;
    }

    bool readString32(std::string& value) {
        uint32_t length = 0;
        if (!readU32(length) || remaining() < length) {
            return false;
        }
        value.assign(reinterpret_cast<const char*>(m_data + m_offset), length);
        m_offset += length;
        return true;
    }

    bool skip(size_t length) {
        if (remaining() < length) {
            return false;
        }
        m_offset += length;
        return true;
    }

    bool readSubcursor(size_t length, ByteCursor& subcursor) {
        if (remaining() < length) {
            return false;
        }
        subcursor = ByteCursor(m_data + m_offset, length);
        m_offset += length;
        return true;
    }

    const std::byte* currentData() const { return m_data + m_offset; }

private:
    const std::byte* m_data = nullptr;
    size_t m_size = 0;
    size_t m_offset = 0;
};

std::string toLowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

std::string getExtensionLowercase(const std::string& path) {
    const size_t slashPos = path.find_last_of("/\\");
    const size_t dotPos = path.find_last_of('.');
    if (dotPos == std::string::npos || (slashPos != std::string::npos && dotPos < slashPos)) {
        return std::string();
    }
    return toLowerAscii(path.substr(dotPos));
}

bool readFileToBytes(const std::string& path, std::vector<std::byte>& bytes, std::string& error) {
    std::ifstream inputFile(path, std::ios::binary | std::ios::ate);
    if (!inputFile.is_open()) {
        error = "Failed to open input file: " + path;
        return false;
    }

    const auto fileEndPos = inputFile.tellg();
    if (fileEndPos < 0) {
        error = "Failed to get input file size: " + path;
        return false;
    }

    const auto fileSize = static_cast<size_t>(fileEndPos);
    if (fileSize == 0) {
        error = "Input file is empty: " + path;
        return false;
    }

    bytes.resize(fileSize);
    inputFile.seekg(0, std::ios::beg);
    inputFile.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(fileSize));
    if (!inputFile) {
        error = "Failed to read input file: " + path;
        return false;
    }
    return true;
}

bool readVarint(ByteCursor& cursor, uint64_t& value) {
    value = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        uint8_t byte = 0;
        if (!cursor.readU8(byte)) {
            return false;
        }
        value |= (static_cast<uint64_t>(byte & 0x7F) << shift);
        if ((byte & 0x80) == 0) {
            return true;
        }
    }
    return false;
}

bool skipProtobufField(ByteCursor& cursor, uint8_t wireType) {
    uint64_t length = 0;
    switch (wireType) {
        case 0:
            return readVarint(cursor, length);
        case 1:
            return cursor.skip(8);
        case 2:
            return readVarint(cursor, length) && length <= std::numeric_limits<size_t>::max() &&
                   cursor.skip(static_cast<size_t>(length));
        case 5:
            return cursor.skip(4);
        default:
            return false;
    }
}

bool extractJsonStringField(std::string_view json, const char* key, std::string& value) {
    const std::string pattern = std::string{"\""} + key + "\":\"";
    const size_t keyPos = json.find(pattern);
    if (keyPos == std::string_view::npos) {
        return false;
    }

    const size_t valueStart = keyPos + pattern.size();
    std::string parsed;
    parsed.reserve(32);

    for (size_t i = valueStart; i < json.size(); ++i) {
        const char ch = json[i];
        if (ch == '\\') {
            if (i + 1 >= json.size()) {
                return false;
            }
            const char escaped = json[++i];
            switch (escaped) {
                case '\\':
                case '"':
                case '/':
                    parsed.push_back(escaped);
                    break;
                case 'b':
                    parsed.push_back('\b');
                    break;
                case 'f':
                    parsed.push_back('\f');
                    break;
                case 'n':
                    parsed.push_back('\n');
                    break;
                case 'r':
                    parsed.push_back('\r');
                    break;
                case 't':
                    parsed.push_back('\t');
                    break;
                default:
                    return false;
            }
            continue;
        }

        if (ch == '"') {
            value = std::move(parsed);
            return true;
        }

        parsed.push_back(ch);
    }

    return false;
}

bool extractJsonBoolField(std::string_view json, const char* key, bool& value) {
    const std::string pattern = std::string{"\""} + key + "\":";
    const size_t keyPos = json.find(pattern);
    if (keyPos == std::string_view::npos) {
        return false;
    }

    const size_t valueStart = keyPos + pattern.size();
    if (json.substr(valueStart, 4) == "true") {
        value = true;
        return true;
    }
    if (json.substr(valueStart, 5) == "false") {
        value = false;
        return true;
    }
    return false;
}

bool extractJsonIntField(std::string_view json, const char* key, int& value) {
    const std::string pattern = std::string{"\""} + key + "\":";
    const size_t keyPos = json.find(pattern);
    if (keyPos == std::string_view::npos) {
        return false;
    }

    size_t valuePos = keyPos + pattern.size();
    while (valuePos < json.size() &&
           std::isspace(static_cast<unsigned char>(json[valuePos])) != 0) {
        ++valuePos;
    }
    if (valuePos >= json.size()) {
        return false;
    }

    size_t valueEnd = valuePos;
    if (json[valueEnd] == '-') {
        ++valueEnd;
    }
    while (valueEnd < json.size() &&
           std::isdigit(static_cast<unsigned char>(json[valueEnd])) != 0) {
        ++valueEnd;
    }
    if (valueEnd == valuePos || (json[valuePos] == '-' && valueEnd == (valuePos + 1))) {
        return false;
    }

    const std::string numberText(json.substr(valuePos, valueEnd - valuePos));
    char* parseEnd = nullptr;
    const long parsed = std::strtol(numberText.c_str(), &parseEnd, 10);
    if (parseEnd == nullptr || *parseEnd != '\0' || parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max()) {
        return false;
    }

    value = static_cast<int>(parsed);
    return true;
}

bool extractJsonNullableIntField(std::string_view json, const char* key, int32_t& value) {
    const std::string pattern = std::string{"\""} + key + "\":";
    const size_t keyPos = json.find(pattern);
    if (keyPos == std::string_view::npos) {
        return false;
    }

    size_t valuePos = keyPos + pattern.size();
    while (valuePos < json.size() &&
           std::isspace(static_cast<unsigned char>(json[valuePos])) != 0) {
        ++valuePos;
    }
    if (valuePos >= json.size()) {
        return false;
    }

    if (json.substr(valuePos, 4) == "null") {
        value = -1;
        return true;
    }

    size_t valueEnd = valuePos;
    if (json[valueEnd] == '-') {
        ++valueEnd;
    }
    while (valueEnd < json.size() &&
           std::isdigit(static_cast<unsigned char>(json[valueEnd])) != 0) {
        ++valueEnd;
    }
    if (valueEnd == valuePos || (json[valuePos] == '-' && valueEnd == (valuePos + 1))) {
        return false;
    }

    const std::string numberText(json.substr(valuePos, valueEnd - valuePos));
    char* parseEnd = nullptr;
    const long parsed = std::strtol(numberText.c_str(), &parseEnd, 10);
    if (parseEnd == nullptr || *parseEnd != '\0' || parsed < std::numeric_limits<int32_t>::min() ||
        parsed > std::numeric_limits<int32_t>::max()) {
        return false;
    }

    value = static_cast<int32_t>(parsed);
    return true;
}

bool parseRoiZAvgLaserDistanceMm(const std::byte* data, size_t size, int32_t& distanceMm) {
    const std::string_view json(reinterpret_cast<const char*>(data), size);
    const std::string namePatternMm = "\"name\":\"laser_distance_mm\"";
    const std::string namePatternLegacy = "\"name\":\"laser_distance_m\"";
    size_t namePos = json.find(namePatternMm);
    if (namePos == std::string_view::npos) {
        namePos = json.find(namePatternLegacy);
    }
    if (namePos == std::string_view::npos) {
        return false;
    }

    const size_t objectEnd = json.find('}', namePos);
    if (objectEnd == std::string_view::npos) {
        return false;
    }

    return extractJsonNullableIntField(json.substr(namePos, objectEnd - namePos + 1), "z_avg_mm",
                                       distanceMm);
}

void parseDeviceInfoJson(const std::byte* data, size_t size, InputSourceInfo& info) {
    const std::string_view json(reinterpret_cast<const char*>(data), size);

    std::string serialNumber;
    if (extractJsonStringField(json, "serial_number", serialNumber)) {
        info.serialNumber = std::move(serialNumber);
    }

    std::string device;
    if (extractJsonStringField(json, "device", device)) {
        info.device = std::move(device);
    }

    bool isUsb = false;
    if (extractJsonBoolField(json, "is_usb", isUsb)) {
        info.isUsb = isUsb;
    }

    int width = 0;
    if (extractJsonIntField(json, "width", width)) {
        info.width = width;
    }

    int height = 0;
    if (extractJsonIntField(json, "height", height)) {
        info.height = height;
    }
}

bool decodeRawImageMessage(const std::byte* messageData, size_t messageSize, DecodedRawImage& image,
                           std::string& error) {
    ByteCursor cursor(messageData, messageSize);
    while (cursor.remaining() > 0) {
        uint64_t tag = 0;
        if (!readVarint(cursor, tag) || tag == 0) {
            error = "Failed to decode RawImage protobuf tag";
            return false;
        }

        const uint32_t fieldNumber = static_cast<uint32_t>(tag >> 3);
        const uint8_t wireType = static_cast<uint8_t>(tag & 0x07);

        switch (fieldNumber) {
            case 2: {
                uint32_t value = 0;
                if (wireType != 5 || !cursor.readU32(value)) {
                    error = "Failed to decode RawImage width";
                    return false;
                }
                image.width = value;
                break;
            }
            case 3: {
                uint32_t value = 0;
                if (wireType != 5 || !cursor.readU32(value)) {
                    error = "Failed to decode RawImage height";
                    return false;
                }
                image.height = value;
                break;
            }
            case 4: {
                if (wireType != 2) {
                    error = "RawImage encoding has unexpected wire type";
                    return false;
                }
                uint64_t length = 0;
                if (!readVarint(cursor, length) || length > cursor.remaining()) {
                    error = "RawImage encoding is truncated";
                    return false;
                }
                image.encoding.assign(reinterpret_cast<const char*>(cursor.currentData()),
                                      static_cast<size_t>(length));
                if (!cursor.skip(static_cast<size_t>(length))) {
                    error = "Failed to read RawImage encoding";
                    return false;
                }
                break;
            }
            case 5: {
                uint32_t value = 0;
                if (wireType != 5 || !cursor.readU32(value)) {
                    error = "Failed to decode RawImage step";
                    return false;
                }
                image.step = value;
                break;
            }
            case 6: {
                if (wireType != 2) {
                    error = "RawImage data has unexpected wire type";
                    return false;
                }
                uint64_t length = 0;
                if (!readVarint(cursor, length) || length > cursor.remaining()) {
                    error = "RawImage data is truncated";
                    return false;
                }
                image.data.resize(static_cast<size_t>(length));
                std::memcpy(image.data.data(), cursor.currentData(), static_cast<size_t>(length));
                if (!cursor.skip(static_cast<size_t>(length))) {
                    error = "Failed to read RawImage data";
                    return false;
                }
                break;
            }
            case 7: {
                if (wireType != 2) {
                    error = "RawImage frame_id has unexpected wire type";
                    return false;
                }
                uint64_t length = 0;
                if (!readVarint(cursor, length) || length > cursor.remaining()) {
                    error = "RawImage frame_id is truncated";
                    return false;
                }
                if (!cursor.skip(static_cast<size_t>(length))) {
                    error = "Failed to skip RawImage frame_id";
                    return false;
                }
                break;
            }
            default:
                if (!skipProtobufField(cursor, wireType)) {
                    error = "Failed to skip unsupported RawImage protobuf field";
                    return false;
                }
                break;
        }
    }
    return true;
}

bool looksLikeAsciiString(const std::byte* data, size_t size) {
    if (size == 0 || size > 32) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        const unsigned char ch = static_cast<unsigned char>(data[i]);
        if (!(std::isalnum(ch) != 0 || ch == '_' || ch == '-' || ch == '/' || ch == '.')) {
            return false;
        }
    }
    return true;
}

bool isLikelyAnnexB(const std::vector<std::byte>& payload) {
    return payload.size() >= 4 && ((payload[0] == std::byte{0} && payload[1] == std::byte{0} &&
                                    payload[2] == std::byte{1}) ||
                                   (payload[0] == std::byte{0} && payload[1] == std::byte{0} &&
                                    payload[2] == std::byte{0} && payload[3] == std::byte{1}));
}

bool decodeCompressedVideoMessage(const std::byte* messageData, size_t messageSize,
                                  DecodedCompressedVideo& video, std::string& error) {
    ByteCursor cursor(messageData, messageSize);
    std::vector<std::byte> payloadCandidate;
    std::string formatCandidate;

    while (cursor.remaining() > 0) {
        uint64_t tag = 0;
        if (!readVarint(cursor, tag) || tag == 0) {
            error = "Failed to decode CompressedVideo protobuf tag";
            return false;
        }

        const uint8_t wireType = static_cast<uint8_t>(tag & 0x07);
        if (wireType != 2) {
            if (!skipProtobufField(cursor, wireType)) {
                error = "Failed to skip unsupported CompressedVideo protobuf field";
                return false;
            }
            continue;
        }

        uint64_t length = 0;
        if (!readVarint(cursor, length) || length > cursor.remaining()) {
            error = "CompressedVideo field is truncated";
            return false;
        }

        const std::byte* fieldData = cursor.currentData();
        const size_t fieldSize = static_cast<size_t>(length);
        if (looksLikeAsciiString(fieldData, fieldSize)) {
            std::string text(reinterpret_cast<const char*>(fieldData), fieldSize);
            text = toLowerAscii(std::move(text));
            if (text == "h264" || text == "h265") {
                formatCandidate = std::move(text);
            }
        }
        if (fieldSize > payloadCandidate.size()) {
            payloadCandidate.assign(fieldData, fieldData + fieldSize);
        }
        if (!cursor.skip(fieldSize)) {
            error = "Failed to advance CompressedVideo cursor";
            return false;
        }
    }

    if (payloadCandidate.empty()) {
        error = "CompressedVideo payload is empty";
        return false;
    }
    if (formatCandidate.empty() && isLikelyAnnexB(payloadCandidate)) {
        formatCandidate = "h264";
    }
    if (formatCandidate != "h264") {
        error = "MCAP /camera/h264 format is '" + formatCandidate + "', expected h264";
        return false;
    }

    video.format = std::move(formatCandidate);
    video.data = std::move(payloadCandidate);
    return true;
}

bool containsH264Idr(const std::vector<std::byte>& payload) {
    auto startCodeSizeAt = [&](size_t pos) -> size_t {
        if (pos + 3 <= payload.size() && payload[pos] == std::byte{0} &&
            payload[pos + 1] == std::byte{0} && payload[pos + 2] == std::byte{1}) {
            return 3;
        }
        if (pos + 4 <= payload.size() && payload[pos] == std::byte{0} &&
            payload[pos + 1] == std::byte{0} && payload[pos + 2] == std::byte{0} &&
            payload[pos + 3] == std::byte{1}) {
            return 4;
        }
        return 0;
    };

    for (size_t pos = 0; pos + 4 <= payload.size(); ++pos) {
        const size_t codeSize = startCodeSizeAt(pos);
        if (codeSize == 0 || pos + codeSize >= payload.size()) {
            continue;
        }
        const uint8_t nalType = static_cast<uint8_t>(payload[pos + codeSize]) & 0x1F;
        if (nalType == 5) {
            return true;
        }
    }
    return false;
}

bool resolveMcapYuyvGeometry(const DecodedRawImage& image, int& detectedWidth, int& detectedHeight,
                             std::string& error) {
    if (image.width == 0 || image.height == 0) {
        error = "MCAP /camera/yuyv has invalid resolution";
        return false;
    }

    if (image.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        image.height > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        error = "MCAP /camera/yuyv resolution is too large";
        return false;
    }

    const int frameWidth = static_cast<int>(image.width);
    const int frameHeight = static_cast<int>(image.height);
    if (detectedWidth == 0 || detectedHeight == 0) {
        detectedWidth = frameWidth;
        detectedHeight = frameHeight;
        return true;
    }

    if (frameWidth != detectedWidth || frameHeight != detectedHeight) {
        error = "MCAP contains mixed /camera/yuyv resolutions, got " + std::to_string(frameWidth) +
                "x" + std::to_string(frameHeight) + ", expected " + std::to_string(detectedWidth) +
                "x" + std::to_string(detectedHeight);
        return false;
    }

    return true;
}

bool extractYuyvFrame(const DecodedRawImage& image, int inputWidth, int inputHeight,
                      std::vector<std::byte>& frame, std::string& error) {
    const size_t requiredRowBytes = static_cast<size_t>(inputWidth) * 2;
    const size_t expectedFrameSize = requiredRowBytes * static_cast<size_t>(inputHeight);
    const std::string encoding = toLowerAscii(image.encoding);

    if (image.width != static_cast<uint32_t>(inputWidth) ||
        image.height != static_cast<uint32_t>(inputHeight)) {
        error = "MCAP YUYV resolution mismatch, got " + std::to_string(image.width) + "x" +
                std::to_string(image.height) + ", expected " + std::to_string(inputWidth) + "x" +
                std::to_string(inputHeight);
        return false;
    }
    if (encoding != "yuyv" && encoding != "yuv422_yuy2") {
        error = "MCAP /camera/yuyv encoding is '" + image.encoding + "', expected yuyv";
        return false;
    }
    if (image.step < requiredRowBytes) {
        error = "MCAP YUYV row stride is too small: " + std::to_string(image.step);
        return false;
    }

    const size_t sourceBytes = static_cast<size_t>(image.step) * image.height;
    if (image.data.size() < sourceBytes) {
        error = "MCAP YUYV payload is truncated";
        return false;
    }

    frame.resize(expectedFrameSize);
    for (size_t row = 0; row < static_cast<size_t>(inputHeight); ++row) {
        const std::byte* sourceRow = image.data.data() + (row * image.step);
        std::byte* targetRow = frame.data() + (row * requiredRowBytes);
        std::memcpy(targetRow, sourceRow, requiredRowBytes);
    }
    return true;
}

bool loadYuyvFrameFromRawFile(const std::string& path, int inputWidth, int inputHeight,
                              std::vector<std::byte>& frame, std::string& error) {
    std::vector<std::byte> fileBytes;
    if (!readFileToBytes(path, fileBytes, error)) {
        return false;
    }

    const size_t expectedFrameSize =
        static_cast<size_t>(inputWidth) * static_cast<size_t>(inputHeight) * 2;
    if (fileBytes.size() < expectedFrameSize) {
        error = "Input image file is too small, got " + std::to_string(fileBytes.size()) +
                " bytes, expected at least " + std::to_string(expectedFrameSize) + " bytes";
        return false;
    }

    frame.assign(fileBytes.begin(),
                 fileBytes.begin() + static_cast<std::ptrdiff_t>(expectedFrameSize));
    if (fileBytes.size() > expectedFrameSize) {
        ALOGW("input image file is larger than one frame, only first %zu bytes will be used",
              expectedFrameSize);
    }
    return true;
}

bool indexYuyvFramesInMcapFile(const std::string& path,
                               std::vector<FrameInputSource::ImportedFrameMeta>& frames,
                               InputSourceInfo& info, std::vector<std::byte>& firstFrameData,
                               uint64_t& firstFrameTimestampNs, int& detectedWidth,
                               int& detectedHeight, std::string& error) {
    mcap::McapReader reader;
    const auto openStatus = reader.open(path);
    if (!openStatus.ok()) {
        error = "Failed to open MCAP file: " + path + ", error=" + openStatus.message;
        return false;
    }

    std::string firstFrameError;
    std::unordered_map<uint64_t, int32_t> laserDistanceByTimestamp;

    auto onProblem = [&](const mcap::Status& problem) {
        if (error.empty()) {
            error = problem.message;
        }
    };

    for (const auto& msgView : reader.readMessages(onProblem)) {
        if (msgView.channel == nullptr) {
            continue;
        }

        const auto& channel = *msgView.channel;
        if (channel.topic == kDeviceInfoTopic && channel.messageEncoding == "json") {
            parseDeviceInfoJson(msgView.message.data, msgView.message.dataSize, info);
            continue;
        }

        if (channel.topic == kRoiZAvgTopic && channel.messageEncoding == "json") {
            int32_t laserDistanceMm = -1;
            if (parseRoiZAvgLaserDistanceMm(msgView.message.data, msgView.message.dataSize,
                                            laserDistanceMm)) {
                const uint64_t timestampNs = msgView.message.logTime != 0
                                                 ? msgView.message.logTime
                                                 : msgView.message.publishTime;
                laserDistanceByTimestamp[timestampNs] = laserDistanceMm;
            }
            continue;
        }

        if (channel.topic != kRawYuyvTopic) {
            continue;
        }

        if (msgView.schema != nullptr) {
            if (msgView.schema->encoding != "protobuf") {
                if (firstFrameError.empty()) {
                    firstFrameError = "MCAP /camera/yuyv schema encoding is '" +
                                      msgView.schema->encoding + "', expected protobuf";
                }
                continue;
            }
            if (msgView.schema->name != "foxglove.RawImage") {
                if (firstFrameError.empty()) {
                    firstFrameError = "MCAP /camera/yuyv schema is '" + msgView.schema->name +
                                      "', expected foxglove.RawImage";
                }
                continue;
            }
        }

        if (channel.messageEncoding != "protobuf") {
            if (firstFrameError.empty()) {
                firstFrameError = "MCAP /camera/yuyv message encoding is '" +
                                  channel.messageEncoding + "', expected protobuf";
            }
            continue;
        }

        if (msgView.message.data == nullptr || msgView.message.dataSize == 0) {
            if (firstFrameError.empty()) {
                firstFrameError = "MCAP /camera/yuyv message payload is empty";
            }
            continue;
        }

        DecodedRawImage image;
        std::string frameError;
        if (!decodeRawImageMessage(msgView.message.data, msgView.message.dataSize, image,
                                   frameError)) {
            if (firstFrameError.empty()) {
                firstFrameError = frameError;
            }
            continue;
        }
        std::vector<std::byte> frame;
        if (!resolveMcapYuyvGeometry(image, detectedWidth, detectedHeight, frameError) ||
            !extractYuyvFrame(image, detectedWidth, detectedHeight, frame, frameError)) {
            if (firstFrameError.empty()) {
                firstFrameError = frameError;
            }
            continue;
        }

        const uint64_t frameTimestampNs =
            msgView.message.logTime != 0 ? msgView.message.logTime : msgView.message.publishTime;
        if (frames.empty()) {
            firstFrameData = frame;
            firstFrameTimestampNs = frameTimestampNs;
        }
        FrameInputSource::ImportedFrameMeta meta;
        meta.frameTimestampNs = frameTimestampNs;
        frames.push_back(meta);
    }

    for (auto& frameMeta : frames) {
        const auto laserIt = laserDistanceByTimestamp.find(frameMeta.frameTimestampNs);
        if (laserIt != laserDistanceByTimestamp.end()) {
            frameMeta.laserDistanceMm = laserIt->second;
        }
    }

    reader.close();

    if (frames.empty()) {
        if (!firstFrameError.empty()) {
            error = firstFrameError;
        } else if (error.empty()) {
            error = "No valid /camera/yuyv frame found in MCAP: " + path;
        }
        return false;
    }

    if (info.width <= 0 || info.height <= 0) {
        error = "MCAP /camera/device_info is missing declared width/height";
        return false;
    }

    if (info.width != detectedWidth || info.height != detectedHeight) {
        error = "MCAP /camera/device_info resolution mismatch, declared " +
                std::to_string(info.width) + "x" + std::to_string(info.height) +
                ", but /camera/yuyv is " + std::to_string(detectedWidth) + "x" +
                std::to_string(detectedHeight);
        return false;
    }

    return true;
}

bool indexH264FramesInMcapFile(const std::string& path,
                               std::vector<FrameInputSource::ImportedFrameMeta>& frames,
                               InputSourceInfo& info, std::string& error) {
    mcap::McapReader reader;
    const auto openStatus = reader.open(path);
    if (!openStatus.ok()) {
        error = "Failed to open MCAP file: " + path + ", error=" + openStatus.message;
        return false;
    }

    std::string firstFrameError;
    std::unordered_map<uint64_t, int32_t> laserDistanceByTimestamp;

    auto onProblem = [&](const mcap::Status& problem) {
        if (error.empty()) {
            error = problem.message;
        }
    };

    for (const auto& msgView : reader.readMessages(onProblem)) {
        if (msgView.channel == nullptr) {
            continue;
        }

        const auto& channel = *msgView.channel;
        if (channel.topic == kDeviceInfoTopic && channel.messageEncoding == "json") {
            parseDeviceInfoJson(msgView.message.data, msgView.message.dataSize, info);
            continue;
        }

        if (channel.topic == kRoiZAvgTopic && channel.messageEncoding == "json") {
            int32_t laserDistanceMm = -1;
            if (parseRoiZAvgLaserDistanceMm(msgView.message.data, msgView.message.dataSize,
                                            laserDistanceMm)) {
                const uint64_t timestampNs = msgView.message.logTime != 0
                                                 ? msgView.message.logTime
                                                 : msgView.message.publishTime;
                laserDistanceByTimestamp[timestampNs] = laserDistanceMm;
            }
            continue;
        }

        if (channel.topic != kH264Topic) {
            continue;
        }

        if (msgView.schema != nullptr) {
            if (msgView.schema->encoding != "protobuf") {
                if (firstFrameError.empty()) {
                    firstFrameError = "MCAP /camera/h264 schema encoding is '" +
                                      msgView.schema->encoding + "', expected protobuf";
                }
                continue;
            }
            if (msgView.schema->name != "foxglove.CompressedVideo") {
                if (firstFrameError.empty()) {
                    firstFrameError = "MCAP /camera/h264 schema is '" + msgView.schema->name +
                                      "', expected foxglove.CompressedVideo";
                }
                continue;
            }
        }

        if (channel.messageEncoding != "protobuf" || msgView.message.data == nullptr ||
            msgView.message.dataSize == 0) {
            if (firstFrameError.empty()) {
                firstFrameError = "MCAP /camera/h264 payload is empty or not protobuf";
            }
            continue;
        }

        DecodedCompressedVideo video;
        std::string frameError;
        if (!decodeCompressedVideoMessage(msgView.message.data, msgView.message.dataSize, video,
                                          frameError)) {
            if (firstFrameError.empty()) {
                firstFrameError = frameError;
            }
            continue;
        }

        FrameInputSource::ImportedFrameMeta meta;
        meta.frameTimestampNs =
            msgView.message.logTime != 0 ? msgView.message.logTime : msgView.message.publishTime;
        meta.keyFrame = containsH264Idr(video.data);
        frames.push_back(meta);
    }

    for (auto& frameMeta : frames) {
        const auto laserIt = laserDistanceByTimestamp.find(frameMeta.frameTimestampNs);
        if (laserIt != laserDistanceByTimestamp.end()) {
            frameMeta.laserDistanceMm = laserIt->second;
        }
    }

    reader.close();

    if (frames.empty()) {
        if (!firstFrameError.empty()) {
            error = firstFrameError;
        } else if (error.empty()) {
            error = "No valid /camera/h264 frame found in MCAP: " + path;
        }
        return false;
    }

    if (std::none_of(frames.begin(), frames.end(),
                     [](const auto& frame) { return frame.keyFrame; })) {
        frames.front().keyFrame = true;
    } else {
        size_t firstKeyFrameIndex = 0;
        while (firstKeyFrameIndex < frames.size() && !frames[firstKeyFrameIndex].keyFrame) {
            ++firstKeyFrameIndex;
        }
        if (firstKeyFrameIndex > 0) {
            frames.erase(frames.begin(), frames.begin() + firstKeyFrameIndex);
        }
    }

    if (info.width <= 0 || info.height <= 0) {
        error = "MCAP /camera/device_info is missing declared width/height";
        return false;
    }

    return true;
}

int estimateRecordedFps(const std::vector<FrameInputSource::ImportedFrameMeta>& frames,
                        int fallbackFps) {
    if (frames.size() < 2) {
        return std::max(1, fallbackFps);
    }

    const uint64_t firstTimestampNs = frames.front().frameTimestampNs;
    const uint64_t lastTimestampNs = frames.back().frameTimestampNs;
    if (lastTimestampNs <= firstTimestampNs) {
        return std::max(1, fallbackFps);
    }

    const double fps = static_cast<double>(frames.size() - 1) * 1000000000.0 /
                       static_cast<double>(lastTimestampNs - firstTimestampNs);
    if (!std::isfinite(fps) || fps <= 0.0) {
        return std::max(1, fallbackFps);
    }

    return std::clamp(static_cast<int>(std::llround(fps)), 1, 240);
}

bool loadIndexedYuyvFrameFromMcapFile(const std::string& path, size_t targetIndex, int inputWidth,
                                      int inputHeight, std::vector<std::byte>& frame,
                                      uint64_t& frameTimestampNs, std::string& error) {
    mcap::McapReader reader;
    const auto openStatus = reader.open(path);
    if (!openStatus.ok()) {
        error = "Failed to open MCAP file: " + path + ", error=" + openStatus.message;
        return false;
    }

    std::string lastFrameError;
    size_t validFrameIndex = 0;

    auto onProblem = [&](const mcap::Status& problem) {
        if (error.empty()) {
            error = problem.message;
        }
    };

    for (const auto& msgView : reader.readMessages(onProblem)) {
        if (msgView.channel == nullptr) {
            continue;
        }

        const auto& channel = *msgView.channel;
        if (channel.topic != kRawYuyvTopic) {
            continue;
        }

        if (msgView.schema != nullptr) {
            if (msgView.schema->encoding != "protobuf" ||
                msgView.schema->name != "foxglove.RawImage") {
                continue;
            }
        }

        if (channel.messageEncoding != "protobuf" || msgView.message.data == nullptr ||
            msgView.message.dataSize == 0) {
            continue;
        }

        DecodedRawImage image;
        std::string frameError;
        if (!decodeRawImageMessage(msgView.message.data, msgView.message.dataSize, image,
                                   frameError)) {
            lastFrameError = frameError;
            continue;
        }

        std::vector<std::byte> decodedFrame;
        if (!extractYuyvFrame(image, inputWidth, inputHeight, decodedFrame, frameError)) {
            lastFrameError = frameError;
            continue;
        }

        if (validFrameIndex == targetIndex) {
            frame = std::move(decodedFrame);
            frameTimestampNs = msgView.message.logTime != 0 ? msgView.message.logTime
                                                            : msgView.message.publishTime;
            reader.close();
            return true;
        }

        ++validFrameIndex;
    }

    reader.close();
    if (!lastFrameError.empty()) {
        error = lastFrameError;
    } else if (error.empty()) {
        error = "Failed to load indexed /camera/yuyv frame from MCAP: " + path;
    }
    return false;
}

}  // namespace

struct FrameInputSource::H264McapReplayCursor {
    mcap::McapReader reader;
    std::optional<mcap::LinearMessageView> view;
    std::optional<mcap::LinearMessageView::Iterator> iter;
    std::optional<mcap::LinearMessageView::Iterator> end;
};

FrameInputSource::FrameInputSource(const InputSourceOptions& options) : m_options(options) {
    m_mode = m_options.useImageFile ? InputMode::ImageFile : InputMode::Uvc;
    m_info.mode = m_mode;
    m_info.device = m_options.device;
    m_info.width = m_options.width;
    m_info.height = m_options.height;
    m_info.fps = m_options.fps;
}

FrameInputSource::~FrameInputSource() { shutdown(); }

bool FrameInputSource::initialize() {
    if (m_mode == InputMode::ImageFile) {
        return initializeImageFile();
    }
    return initializeUvc();
}

bool FrameInputSource::initializeUvc() {
    if (!isSupportedStereoInputResolution(m_options.width, m_options.height)) {
        ALOGE("unsupported UVC stereo resolution: %dx%d (supported: 2560x720, 3840x1080)",
              m_options.width, m_options.height);
        return false;
    }

    m_capture = std::make_unique<V4L2Capture>(m_options.device, m_options.width, m_options.height,
                                              m_options.fps, m_options.uvcControls);

    const size_t frameSize = m_capture->initialize();
    if (frameSize == 0) {
        ALOGE("failed to initialize UVC camera: %s", m_options.device.c_str());
        return false;
    }

    if (!m_capture->start()) {
        ALOGE("failed to start UVC camera: %s", m_options.device.c_str());
        return false;
    }
    m_captureStarted = true;

    if (m_capture->getWidth() != m_options.width || m_capture->getHeight() != m_options.height) {
        ALOGE("UVC resolution mismatch, got %dx%d, expected %dx%d", m_capture->getWidth(),
              m_capture->getHeight(), m_options.width, m_options.height);
        return false;
    }

    const V4L2Capture::DeviceInfo& deviceInfo = m_capture->getDeviceInfo();
    m_info.mode = InputMode::Uvc;
    m_info.device = deviceInfo.devicePath.empty() ? m_options.device : deviceInfo.devicePath;
    m_info.serialNumber = deviceInfo.serialNumber;
    m_info.isUsb = deviceInfo.isUsb;
    m_info.width = m_capture->getWidth();
    m_info.height = m_capture->getHeight();
    m_info.fps = m_capture->getFps();

    ALOGN("Input mode: UVC capture from %s (io_mode=mmap)", m_options.device.c_str());
    return true;
}

bool FrameInputSource::initializeImageFile() {
    if (m_options.imageFile.empty()) {
        ALOGE("input image file is not specified. Please use -i <file>");
        return false;
    }

    std::string error;
    const std::string extension = getExtensionLowercase(m_options.imageFile);
    const bool isMcapInput = (extension == ".mcap");
    InputSourceInfo importedInfo;
    bool loaded = false;
    if (isMcapInput) {
        std::vector<ImportedFrameMeta> frames;
        if (m_options.mcapImportStream == McapImportStream::H264) {
            loaded = indexH264FramesInMcapFile(m_options.imageFile, frames, importedInfo, error);
        } else {
            int detectedWidth = 0;
            int detectedHeight = 0;
            std::vector<std::byte> firstFrameData;
            uint64_t firstFrameTimestampNs = 0;
            loaded = indexYuyvFramesInMcapFile(m_options.imageFile, frames, importedInfo,
                                               firstFrameData, firstFrameTimestampNs, detectedWidth,
                                               detectedHeight, error);
            if (loaded) {
                auto importedFrame = std::make_shared<ImportedFrame>();
                importedFrame->data = std::move(firstFrameData);
                importedFrame->frameFormat = InputFrameFormat::Yuyv;
                importedFrame->frameTimestampNs = firstFrameTimestampNs;
                importedFrame->laserDistanceMm =
                    frames.empty() ? -1 : frames.front().laserDistanceMm;

                {
                    std::lock_guard<std::mutex> lock(m_importedFrameMutex);
                    m_selectedImportedFrame = std::move(importedFrame);
                    m_leasedImportedFrame.reset();
                }

                ALOGN("MCAP input geometry verified: device_info=%dx%d /camera/yuyv=%dx%d",
                      importedInfo.width, importedInfo.height, detectedWidth, detectedHeight);
            }
        }
        if (loaded) {
            m_options.width = importedInfo.width;
            m_options.height = importedInfo.height;
            m_importedMcapPath = m_options.imageFile;
            m_importedMcapSource = m_options.mcapImportStream == McapImportStream::H264
                                       ? ImportedMcapSource::H264
                                       : ImportedMcapSource::RawYuyv;
            if (!isSupportedStereoInputResolution(m_options.width, m_options.height)) {
                error = "unsupported MCAP stereo resolution: " + std::to_string(m_options.width) +
                        "x" + std::to_string(m_options.height) +
                        " (supported: 2560x720, 3840x1080)";
                loaded = false;
            }
            if (loaded) {
                m_importedFrames = std::move(frames);
                m_selectedImportedFrameIndex.store(0, std::memory_order_release);
                resetImportedH264ReplayState();
            }
        }
    } else {
        if (!isSupportedStereoInputResolution(m_options.width, m_options.height)) {
            ALOGE("unsupported raw YUYV stereo resolution: %dx%d (supported: 2560x720, 3840x1080)",
                  m_options.width, m_options.height);
            return false;
        }
        loaded = loadYuyvFrameFromRawFile(m_options.imageFile, m_options.width, m_options.height,
                                          m_imageData, error);
    }
    if (!loaded) {
        ALOGE("%s", error.c_str());
        return false;
    }

    m_info.mode = InputMode::ImageFile;
    m_info.device = isMcapInput ? importedInfo.device : std::string();
    m_info.serialNumber = isMcapInput ? importedInfo.serialNumber : std::string();
    m_info.isUsb = isMcapInput ? importedInfo.isUsb : false;
    m_info.width = m_options.width;
    m_info.height = m_options.height;
    m_info.fps = (isMcapInput && m_importedMcapSource == ImportedMcapSource::H264)
                     ? estimateRecordedFps(m_importedFrames, 15)
                     : 5;

    if (isMcapInput) {
        std::string replayDescription;
        if (m_importedMcapSource == ImportedMcapSource::H264) {
            replayDescription =
                ", replay=" + std::to_string(m_info.fps) + "fps from recorded timestamps";
        } else {
            replayDescription = ", default frame 1/" + std::to_string(m_importedFrames.size());
        }
        ALOGN("Input mode: file injection from %s (parsed %zu %s frames from MCAP%s)",
              m_options.imageFile.c_str(), m_importedFrames.size(), importedFrameSourceName(),
              replayDescription.c_str());
    } else {
        ALOGN("Input mode: file injection from %s", m_options.imageFile.c_str());
    }
    return true;
}

bool FrameInputSource::getFrame(const void*& frameData, size_t& frameSize,
                                uint64_t& frameTimestampNs, FrameLaserDistance* laserDistance,
                                InputFrameFormat* frameFormat,
                                const AX_VIDEO_FRAME_T** externalFrame,
                                std::shared_ptr<void>* externalFrameOwner) {
    if (laserDistance != nullptr) {
        *laserDistance = {};
    }
    if (frameFormat != nullptr) {
        *frameFormat = InputFrameFormat::Yuyv;
    }
    if (externalFrame != nullptr) {
        *externalFrame = nullptr;
    }
    if (externalFrameOwner != nullptr) {
        externalFrameOwner->reset();
    }

    if (m_mode == InputMode::ImageFile) {
        if (!m_importedFrames.empty()) {
            if (m_importedMcapSource == ImportedMcapSource::H264) {
                std::shared_ptr<const ImportedFrame> importedFrame;
                std::string error;
                if (!loadNextImportedH264Frame(importedFrame, error)) {
                    if (!error.empty()) {
                        ALOGE("%s", error.c_str());
                    }
                    return false;
                }

                std::lock_guard<std::mutex> lock(m_importedFrameMutex);
                m_leasedImportedFrame = std::move(importedFrame);
                frameData = m_leasedImportedFrame->data.empty()
                                ? reinterpret_cast<const void*>(
                                      m_leasedImportedFrame->externalFrame.u64VirAddr[0])
                                : m_leasedImportedFrame->data.data();
                frameSize =
                    m_leasedImportedFrame->data.empty()
                        ? static_cast<size_t>(m_leasedImportedFrame->externalFrame.u32FrameSize)
                        : m_leasedImportedFrame->data.size();
                frameTimestampNs = m_leasedImportedFrame->frameTimestampNs;
                if (frameFormat != nullptr) {
                    *frameFormat = m_leasedImportedFrame->frameFormat;
                }
                if (externalFrame != nullptr && m_leasedImportedFrame->externalFrameOwner) {
                    *externalFrame = &m_leasedImportedFrame->externalFrame;
                }
                if (externalFrameOwner != nullptr) {
                    *externalFrameOwner = m_leasedImportedFrame->externalFrameOwner;
                }
                if (laserDistance != nullptr) {
                    laserDistance->distanceMm = m_leasedImportedFrame->laserDistanceMm;
                }
                return true;
            }

            {
                std::lock_guard<std::mutex> lock(m_importedFrameMutex);
                if (m_selectedImportedFrame) {
                    m_leasedImportedFrame = m_selectedImportedFrame;
                    frameData = m_leasedImportedFrame->data.empty()
                                    ? reinterpret_cast<const void*>(
                                          m_leasedImportedFrame->externalFrame.u64VirAddr[0])
                                    : m_leasedImportedFrame->data.data();
                    frameSize =
                        m_leasedImportedFrame->data.empty()
                            ? static_cast<size_t>(m_leasedImportedFrame->externalFrame.u32FrameSize)
                            : m_leasedImportedFrame->data.size();
                    frameTimestampNs =
                        m_leasedImportedFrame->frameTimestampNs != 0
                            ? m_leasedImportedFrame->frameTimestampNs
                            : static_cast<uint64_t>(
                                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count());
                    if (frameFormat != nullptr) {
                        *frameFormat = m_leasedImportedFrame->frameFormat;
                    }
                    if (externalFrame != nullptr && m_leasedImportedFrame->externalFrameOwner) {
                        *externalFrame = &m_leasedImportedFrame->externalFrame;
                    }
                    if (externalFrameOwner != nullptr) {
                        *externalFrameOwner = m_leasedImportedFrame->externalFrameOwner;
                    }
                    if (laserDistance != nullptr) {
                        laserDistance->distanceMm = m_leasedImportedFrame->laserDistanceMm;
                    }
                    return true;
                }
            }

            ImportedFrameSelection selection = importedFrameSelection();
            std::string error;
            if (!loadImportedFrame(selection.index, selection, error)) {
                if (!error.empty()) {
                    ALOGE("%s", error.c_str());
                }
                return false;
            }
            return getFrame(frameData, frameSize, frameTimestampNs, laserDistance, frameFormat,
                            externalFrame, externalFrameOwner);
        } else {
            frameData = m_imageData.data();
            frameSize = m_imageData.size();
            frameTimestampNs =
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count());
        }
        return true;
    }

    if (m_capture == nullptr) {
        return false;
    }

    if (!m_capture->grab(m_currentFrame)) {
        return false;
    }

    m_hasCurrentFrame = true;
    frameData = m_currentFrame.data;
    frameSize = m_currentFrame.size;
    frameTimestampNs = m_currentFrame.timestampNs;
    return true;
}

void FrameInputSource::releaseFrame() {
    if (m_mode == InputMode::Uvc && m_capture != nullptr && m_hasCurrentFrame) {
        m_capture->release(m_currentFrame);
        m_hasCurrentFrame = false;
    } else if (m_mode == InputMode::ImageFile && !m_importedFrames.empty()) {
        std::lock_guard<std::mutex> lock(m_importedFrameMutex);
        m_leasedImportedFrame.reset();
    }
}

void FrameInputSource::shutdown() {
    releaseFrame();

    if (m_captureStarted && m_capture != nullptr) {
        m_capture->stop();
        m_captureStarted = false;
    }

    for (void* buffer : m_captureBuffers) {
        free(buffer);
    }
    m_captureBuffers.clear();

    m_capture.reset();
    m_imageData.clear();
    {
        std::lock_guard<std::mutex> lock(m_importedFrameMutex);
        m_selectedImportedFrame.reset();
        m_leasedImportedFrame.reset();
    }
    m_importedMcapPath.clear();
    m_importedMcapSource = ImportedMcapSource::None;
    m_importedFrames.clear();
    m_selectedImportedFrameIndex.store(0, std::memory_order_release);
    resetImportedH264ReplayState();
}

bool FrameInputSource::usesRecordedReplayTiming() const {
    return m_mode == InputMode::ImageFile && m_importedMcapSource == ImportedMcapSource::H264;
}

bool FrameInputSource::supportsImportedFrameSelection() const {
    return m_mode == InputMode::ImageFile && m_importedMcapSource != ImportedMcapSource::H264 &&
           m_importedFrames.size() > 1;
}

FrameInputSource::ImportedFrameSelection FrameInputSource::importedFrameSelection() const {
    ImportedFrameSelection selection;
    selection.count = m_importedFrames.size();
    if (m_importedFrames.empty()) {
        return selection;
    }

    selection.index = std::min(m_selectedImportedFrameIndex.load(std::memory_order_acquire),
                               m_importedFrames.size() - 1);
    selection.frameTimestampNs = m_importedFrames[selection.index].frameTimestampNs;
    return selection;
}

bool FrameInputSource::stepImportedFrame(int delta, ImportedFrameSelection& selection) {
    selection = importedFrameSelection();
    if (selection.count == 0 || delta == 0) {
        return false;
    }

    const size_t currentIndex = selection.index;
    size_t nextIndex = currentIndex;
    if (delta > 0) {
        const size_t step = static_cast<size_t>(delta);
        nextIndex = std::min(currentIndex + step, selection.count - 1);
    } else {
        const size_t step = static_cast<size_t>(-delta);
        nextIndex = step > currentIndex ? 0 : (currentIndex - step);
    }

    if (nextIndex == currentIndex) {
        return false;
    }

    std::string error;
    if (!loadImportedFrame(nextIndex, selection, error)) {
        if (!error.empty()) {
            ALOGE("%s", error.c_str());
        }
        return false;
    }
    return true;
}

bool FrameInputSource::loadImportedFrame(size_t frameIndex, ImportedFrameSelection& selection,
                                         std::string& error) {
    if (frameIndex >= m_importedFrames.size()) {
        error = "Imported frame index out of range";
        return false;
    }

    std::vector<std::byte> frameData;
    uint64_t frameTimestampNs = 0;
    if (m_importedMcapSource == ImportedMcapSource::H264) {
        error = "Imported H.264 frame selection is not supported; use sequential replay";
        return false;
    } else {
        if (!loadIndexedYuyvFrameFromMcapFile(m_importedMcapPath, frameIndex, m_options.width,
                                              m_options.height, frameData, frameTimestampNs,
                                              error)) {
            return false;
        }
    }

    auto importedFrame = std::make_shared<ImportedFrame>();
    importedFrame->data = std::move(frameData);
    importedFrame->frameFormat = m_importedMcapSource == ImportedMcapSource::H264
                                     ? InputFrameFormat::Nv12
                                     : InputFrameFormat::Yuyv;
    importedFrame->frameTimestampNs = frameTimestampNs;
    importedFrame->laserDistanceMm = m_importedFrames[frameIndex].laserDistanceMm;

    {
        std::lock_guard<std::mutex> lock(m_importedFrameMutex);
        m_selectedImportedFrame = std::move(importedFrame);
        m_leasedImportedFrame.reset();
        m_selectedImportedFrameIndex.store(frameIndex, std::memory_order_release);
    }

    selection.index = frameIndex;
    selection.count = m_importedFrames.size();
    selection.frameTimestampNs = frameTimestampNs;
    return true;
}

bool FrameInputSource::loadNextImportedH264Frame(std::shared_ptr<const ImportedFrame>& frame,
                                                 std::string& error) {
    if (m_importedMcapSource != ImportedMcapSource::H264 || m_importedFrames.empty()) {
        error = "Imported H.264 replay state is invalid";
        return false;
    }

    if (!m_h264ReplayDecoder) {
        m_h264ReplayDecoder = std::make_unique<stereo_depth::axera_pipeline::AxVdecH264Decoder>();
    }

    auto findMetaIndexByPts = [this](uint64_t pts) -> size_t {
        if (m_importedFrames.empty()) {
            return 0;
        }
        auto it = std::lower_bound(m_importedFrames.begin(), m_importedFrames.end(), pts,
                                   [](const ImportedFrameMeta& meta, uint64_t value) {
                                       return meta.frameTimestampNs < value;
                                   });
        if (it == m_importedFrames.end()) {
            return m_importedFrames.size() - 1;
        }
        if (it != m_importedFrames.begin() && it->frameTimestampNs != pts) {
            auto prev = std::prev(it);
            const uint64_t prevDelta =
                pts > prev->frameTimestampNs ? pts - prev->frameTimestampNs : 0;
            const uint64_t curDelta = it->frameTimestampNs > pts ? it->frameTimestampNs - pts : 0;
            if (prevDelta <= curDelta) {
                it = prev;
            }
        }
        return static_cast<size_t>(std::distance(m_importedFrames.begin(), it));
    };

    const size_t maxAttempts = std::max<size_t>(m_importedFrames.size() * 2, 32);
    for (size_t attempt = 0; attempt < maxAttempts; ++attempt) {
        // Create/start the decoder once (after construction or a replay reset).
        // Do not recreate it while skipping leading non-IDR access units.
        if (!m_h264ReplayDecoder->isStarted()) {
            if (!m_h264ReplayDecoder->beginSequence(m_options.width, m_options.height, error)) {
                return false;
            }
        }

        std::vector<std::byte> accessUnit;
        uint64_t frameTimestampNs = 0;
        if (!readNextImportedH264AccessUnit(accessUnit, frameTimestampNs, error)) {
            if (error == "No more valid /camera/h264 frame found in MCAP replay stream") {
                resetImportedH264ReplayState();
                error.clear();
                continue;
            }
            return false;
        }

        if (!m_replayClockStarted && !containsH264Idr(accessUnit)) {
            continue;
        }

        if (!m_replayClockStarted) {
            m_replayStartSteadyTime = std::chrono::steady_clock::now();
            m_replayStartTimestampNs = frameTimestampNs;
            m_replayClockStarted = true;
        } else if (frameTimestampNs > m_replayStartTimestampNs) {
            const auto replayDeadline =
                m_replayStartSteadyTime +
                std::chrono::nanoseconds(frameTimestampNs - m_replayStartTimestampNs);
            const auto now = std::chrono::steady_clock::now();
            if (now < replayDeadline) {
                std::this_thread::sleep_until(replayDeadline);
            }
        }

        stereo_depth::axera_pipeline::AxVdecH264Decoder::DecodedFrameHandle decodedFrame;
        if (!m_h264ReplayDecoder->decodeAccessUnitToFrame(accessUnit, frameTimestampNs,
                                                          decodedFrame, error)) {
            if (isRecoverableH264DecodeError(error)) {
                error.clear();
                continue;
            }
            return false;
        }

        const size_t metaIndex = findMetaIndexByPts(frameTimestampNs);
        const ImportedFrameMeta& meta = m_importedFrames[metaIndex];

        auto importedFrame = std::make_shared<ImportedFrame>();
        importedFrame->frameFormat = InputFrameFormat::Nv12;
        importedFrame->externalFrame = decodedFrame->frameInfo.stVFrame;
        importedFrame->externalFrameOwner = std::static_pointer_cast<void>(decodedFrame);
        importedFrame->frameTimestampNs = frameTimestampNs;
        importedFrame->laserDistanceMm = meta.laserDistanceMm;
        frame = std::move(importedFrame);

        m_selectedImportedFrameIndex.store(metaIndex, std::memory_order_release);
        return true;
    }

    error = "Unable to obtain valid decoded H.264 frame after retries";
    return false;
}

void FrameInputSource::resetImportedH264ReplayState() {
    m_replayClockStarted = false;
    m_replayStartTimestampNs = 0;
    m_replayStartSteadyTime = std::chrono::steady_clock::time_point{};
    if (m_h264McapReplayCursor) {
        m_h264McapReplayCursor->iter.reset();
        m_h264McapReplayCursor->end.reset();
        m_h264McapReplayCursor->view.reset();
        m_h264McapReplayCursor->reader.close();
        m_h264McapReplayCursor.reset();
    }
    if (m_h264ReplayDecoder) {
        m_h264ReplayDecoder->stop();
    }
}

bool FrameInputSource::prepareImportedH264ReplayCursor(std::string& error) {
    if (m_h264McapReplayCursor) {
        return true;
    }

    auto cursor = std::make_unique<H264McapReplayCursor>();
    const auto openStatus = cursor->reader.open(m_importedMcapPath);
    if (!openStatus.ok()) {
        error = "Failed to open MCAP file: " + m_importedMcapPath + ", error=" + openStatus.message;
        return false;
    }

    auto onProblem = [&](const mcap::Status& problem) {
        if (error.empty()) {
            error = problem.message;
        }
    };

    mcap::ReadMessageOptions options;
    options.readOrder = mcap::ReadMessageOptions::ReadOrder::FileOrder;
    options.topicFilter = [](std::string_view topic) { return isH264Topic(topic); };

    cursor->view.emplace(cursor->reader.readMessages(onProblem, options));
    cursor->iter.emplace(cursor->view->begin());
    cursor->end.emplace(cursor->view->end());

    m_h264McapReplayCursor = std::move(cursor);
    return true;
}

bool FrameInputSource::readNextImportedH264AccessUnit(std::vector<std::byte>& accessUnit,
                                                      uint64_t& frameTimestampNs,
                                                      std::string& error) {
    accessUnit.clear();
    frameTimestampNs = 0;

    if (!prepareImportedH264ReplayCursor(error)) {
        return false;
    }

    while (m_h264McapReplayCursor->iter && m_h264McapReplayCursor->end &&
           *m_h264McapReplayCursor->iter != *m_h264McapReplayCursor->end) {
        const auto& msgView = **m_h264McapReplayCursor->iter;
        ++(*m_h264McapReplayCursor->iter);

        if (msgView.channel == nullptr || msgView.channel->topic != kH264Topic) {
            continue;
        }

        if (msgView.schema != nullptr) {
            if (msgView.schema->encoding != "protobuf") {
                continue;
            }
            if (msgView.schema->name != "foxglove.CompressedVideo") {
                continue;
            }
        }

        if (msgView.channel->messageEncoding != "protobuf" || msgView.message.data == nullptr ||
            msgView.message.dataSize == 0) {
            continue;
        }

        DecodedCompressedVideo video;
        std::string frameError;
        if (!decodeCompressedVideoMessage(msgView.message.data, msgView.message.dataSize, video,
                                          frameError)) {
            continue;
        }

        accessUnit = std::move(video.data);
        frameTimestampNs =
            msgView.message.logTime != 0 ? msgView.message.logTime : msgView.message.publishTime;
        if (!m_importedFrames.empty() &&
            frameTimestampNs < m_importedFrames.front().frameTimestampNs) {
            continue;
        }
        return true;
    }

    if (error.empty()) {
        error = "No more valid /camera/h264 frame found in MCAP replay stream";
    }
    return false;
}

const char* FrameInputSource::importedFrameSourceName() const {
    switch (m_importedMcapSource) {
        case ImportedMcapSource::H264:
            return "H.264";
        case ImportedMcapSource::RawYuyv:
            return "YUYV";
        default:
            return "imported";
    }
}

}  // namespace stereo_depth

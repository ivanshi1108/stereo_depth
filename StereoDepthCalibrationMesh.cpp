#include "StereoDepthPipeline.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <curl/curl.h>

#include "ax_sys_api.h"

#define SAMPLE_LOG_TAG "CALIB"
#include "sample_log.h"

namespace stereo_depth {
namespace {

constexpr AX_U32 kMeshTableRows = 33;
constexpr AX_U32 kMeshTableCols = 33;
constexpr AX_U32 kMeshPaddingSize = 3;
constexpr AX_U32 kMeshAlignSize = 16;
constexpr AX_U32 kMeshDiffFixFactor = 1024;
constexpr AX_U32 kMeshOutputWidth = stereo_depth::kDewarpImageWidth;
constexpr AX_U32 kMeshOutputHeight = stereo_depth::kDewarpImageHeight;
constexpr double kMeshOutputCx = 320.0;
constexpr double kMeshOutputCy = 192.0;
constexpr double kMeshOutputFx = 400.0;
constexpr double kMeshOutputFy = 460.0;
struct CalibrationGeometry {
    AX_U32 stereoWidth = 0;
    AX_U32 perEyeWidth = 0;
    AX_U32 height = 0;
    std::string resolution;
};

struct CameraParameter {
    AX_U32 imageWidth = kMeshOutputWidth;
    AX_U32 imageHeight = kMeshOutputHeight;
    AX_U32 outputWidth = kMeshOutputWidth;
    AX_U32 outputHeight = kMeshOutputHeight;
    double cxOut = kMeshOutputCx;
    double cyOut = kMeshOutputCy;
    double fxOut = kMeshOutputFx;
    double fyOut = kMeshOutputFy;
    double cx = 0.0;
    double cy = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    cv::Matx33d rotation = cv::Matx33d::eye();
    std::array<double, 4> distParams = {0.0, 0.0, 0.0, 0.0};
    bool hasDistParams = false;
};

struct StereoCalibrationData {
    cv::Matx33d K1 = cv::Matx33d::eye();
    cv::Vec4d D1 = cv::Vec4d::all(0.0);
    cv::Matx33d K2 = cv::Matx33d::eye();
    cv::Vec4d D2 = cv::Vec4d::all(0.0);
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d T = cv::Vec3d::all(0.0);
};

struct RectificationData {
    cv::Matx33d R1 = cv::Matx33d::eye();
    cv::Matx33d R2 = cv::Matx33d::eye();
};

using IniSection = std::map<std::string, double>;
using IniMap = std::map<std::string, IniSection>;

std::string trimString(const std::string& value) {
    const size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return std::string();
    }
    const size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string toLowerString(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

bool isNonEmptyFile(const std::string& path) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
        return false;
    }
    return std::filesystem::file_size(path, ec) > 0 && !ec;
}

bool parseIniFile(const std::string& path, IniMap& iniValues, std::string& error) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        error = "open calibration ini failed: " + path;
        return false;
    }

    std::string currentSection;
    std::string line;
    while (std::getline(ifs, line)) {
        line = trimString(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            currentSection = line.substr(1, line.size() - 2);
            continue;
        }

        const size_t equalsPos = line.find('=');
        if (equalsPos == std::string::npos || currentSection.empty()) {
            continue;
        }

        const std::string key = toLowerString(trimString(line.substr(0, equalsPos)));
        const std::string value = trimString(line.substr(equalsPos + 1));
        try {
            iniValues[currentSection][key] = std::stod(value);
        } catch (const std::exception&) {
            error = "parse calibration ini value failed: section=" + currentSection + " key=" + key;
            return false;
        }
    }

    return true;
}

bool getIniDouble(const IniMap& iniValues, const std::string& section, const std::string& key,
                  double& value) {
    const auto sectionIt = iniValues.find(section);
    if (sectionIt == iniValues.end()) {
        return false;
    }
    const auto valueIt = sectionIt->second.find(toLowerString(key));
    if (valueIt == sectionIt->second.end()) {
        return false;
    }
    value = valueIt->second;
    return true;
}

cv::Matx33d eulerToRotationMatrixXYZ(const cv::Vec3d& eulerAngles) {
    const double rx = eulerAngles[0];
    const double ry = eulerAngles[1];
    const double rz = eulerAngles[2];

    const cv::Matx33d rxMat(1.0, 0.0, 0.0, 0.0, std::cos(rx), -std::sin(rx), 0.0, std::sin(rx),
                            std::cos(rx));
    const cv::Matx33d ryMat(std::cos(ry), 0.0, std::sin(ry), 0.0, 1.0, 0.0, -std::sin(ry), 0.0,
                            std::cos(ry));
    const cv::Matx33d rzMat(std::cos(rz), -std::sin(rz), 0.0, std::sin(rz), std::cos(rz), 0.0, 0.0,
                            0.0, 1.0);
    return rzMat * ryMat * rxMat;
}

bool resolveCalibrationGeometry(AX_U32 inputStereoWidth, AX_U32 inputHeight,
                                CalibrationGeometry& geometry, std::string& error) {
    switch (inputHeight) {
        case 1080:
            geometry = {3840, 1920, 1080, "FHD"};
            break;
        case 720:
            geometry = {2560, 1280, 720, "HD"};
            break;
        default:
            error = "unsupported stereo input geometry: " + std::to_string(inputStereoWidth) + "x" +
                    std::to_string(inputHeight);
            return false;
    }

    if (inputStereoWidth != geometry.stereoWidth) {
        error = "unsupported stereo input width for " + geometry.resolution + ": got " +
                std::to_string(inputStereoWidth) + ", expected " +
                std::to_string(geometry.stereoWidth);
        return false;
    }

    return true;
}

bool loadStereoCalibration(const std::string& iniPath, AX_U32 inputStereoWidth, AX_U32 inputHeight,
                           CalibrationGeometry& geometry, StereoCalibrationData& calibration,
                           std::string& error) {
    IniMap iniValues;
    if (!parseIniFile(iniPath, iniValues, error)) {
        return false;
    }

    if (!resolveCalibrationGeometry(inputStereoWidth, inputHeight, geometry, error)) {
        return false;
    }

    const std::string leftSection = std::string("LEFT_CAM_") + geometry.resolution;
    const std::string rightSection = std::string("RIGHT_CAM_") + geometry.resolution;
    const std::string stereoSection = "STEREO";
    const std::string rxKey = "RX_" + geometry.resolution;
    const std::string cvKey = "CV_" + geometry.resolution;
    const std::string rzKey = "RZ_" + geometry.resolution;

    double leftFx = 0.0;
    double leftFy = 0.0;
    double leftCx = 0.0;
    double leftCy = 0.0;
    double leftK1 = 0.0;
    double leftK2 = 0.0;
    double leftK3 = 0.0;
    double leftK4 = 0.0;
    double rightFx = 0.0;
    double rightFy = 0.0;
    double rightCx = 0.0;
    double rightCy = 0.0;
    double rightK1 = 0.0;
    double rightK2 = 0.0;
    double rightK3 = 0.0;
    double rightK4 = 0.0;
    double baseline = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double rx = 0.0;
    double cv = 0.0;
    double rz = 0.0;

    const bool ok = getIniDouble(iniValues, leftSection, "fx", leftFx) &&
                    getIniDouble(iniValues, leftSection, "fy", leftFy) &&
                    getIniDouble(iniValues, leftSection, "cx", leftCx) &&
                    getIniDouble(iniValues, leftSection, "cy", leftCy) &&
                    getIniDouble(iniValues, leftSection, "k1", leftK1) &&
                    getIniDouble(iniValues, leftSection, "k2", leftK2) &&
                    getIniDouble(iniValues, leftSection, "k3", leftK3) &&
                    getIniDouble(iniValues, leftSection, "k4", leftK4) &&
                    getIniDouble(iniValues, rightSection, "fx", rightFx) &&
                    getIniDouble(iniValues, rightSection, "fy", rightFy) &&
                    getIniDouble(iniValues, rightSection, "cx", rightCx) &&
                    getIniDouble(iniValues, rightSection, "cy", rightCy) &&
                    getIniDouble(iniValues, rightSection, "k1", rightK1) &&
                    getIniDouble(iniValues, rightSection, "k2", rightK2) &&
                    getIniDouble(iniValues, rightSection, "k3", rightK3) &&
                    getIniDouble(iniValues, rightSection, "k4", rightK4) &&
                    getIniDouble(iniValues, stereoSection, "Baseline", baseline) &&
                    getIniDouble(iniValues, stereoSection, "TY", ty) &&
                    getIniDouble(iniValues, stereoSection, "TZ", tz) &&
                    getIniDouble(iniValues, stereoSection, rxKey, rx) &&
                    getIniDouble(iniValues, stereoSection, cvKey, cv) &&
                    getIniDouble(iniValues, stereoSection, rzKey, rz);
    if (!ok) {
        error = "missing required fields in calibration ini: " + iniPath;
        return false;
    }

    ALOGN(
        "mesh calibration source: ini=%s input=%ux%u per-eye=%ux%u resolution=%s left=%s right=%s "
        "stereo_keys=[%s,%s,%s]",
        iniPath.c_str(), inputStereoWidth, inputHeight, geometry.perEyeWidth, geometry.height,
        geometry.resolution.c_str(), leftSection.c_str(), rightSection.c_str(), rxKey.c_str(),
        cvKey.c_str(), rzKey.c_str());

    calibration.K1 = cv::Matx33d(leftFx, 0.0, leftCx, 0.0, leftFy, leftCy, 0.0, 0.0, 1.0);
    calibration.D1 = cv::Vec4d(leftK1, leftK2, leftK3, leftK4);
    calibration.K2 = cv::Matx33d(rightFx, 0.0, rightCx, 0.0, rightFy, rightCy, 0.0, 0.0, 1.0);
    calibration.D2 = cv::Vec4d(rightK1, rightK2, rightK3, rightK4);
    calibration.T = cv::Vec3d(baseline / 1000.0, ty / 1000.0, tz / 1000.0);
    calibration.R = eulerToRotationMatrixXYZ(cv::Vec3d(rx, cv, rz));
    return true;
}

bool computeRectification(const StereoCalibrationData& calibration,
                          const CalibrationGeometry& geometry, RectificationData& rectification,
                          std::string& error) {
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;

    try {
        cv::fisheye::stereoRectify(
            cv::Mat(calibration.K1), cv::Mat(calibration.D1), cv::Mat(calibration.K2),
            cv::Mat(calibration.D2),
            cv::Size(static_cast<int>(geometry.perEyeWidth), static_cast<int>(geometry.height)),
            cv::Mat(calibration.R), cv::Mat(calibration.T), R1, R2, P1, P2, Q, 0,
            cv::Size(static_cast<int>(kMeshOutputWidth), static_cast<int>(kMeshOutputHeight)));
    } catch (const cv::Exception& ex) {
        error = std::string("stereoRectify failed: ") + ex.what();
        return false;
    }

    P1.at<double>(0, 0) = kMeshOutputFx;
    P2.at<double>(0, 0) = kMeshOutputFx;
    P1.at<double>(1, 1) = kMeshOutputFy;
    P2.at<double>(1, 1) = kMeshOutputFy;
    P1.at<double>(0, 2) = kMeshOutputCx;
    P2.at<double>(0, 2) = kMeshOutputCx;
    P1.at<double>(1, 2) = kMeshOutputCy;
    P2.at<double>(1, 2) = kMeshOutputCy;

    rectification.R1 = R1;
    rectification.R2 = R2;
    return true;
}

std::pair<double, double> fisheyeDistort(const std::array<double, 4>& dist, double x, double y) {
    const double r = std::sqrt(x * x + y * y);
    if (r < 1e-8) {
        return {0.0, 0.0};
    }

    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta6 * theta2;
    const double thetaD =
        theta * (1.0 + dist[0] * theta2 + dist[1] * theta4 + dist[2] * theta6 + dist[3] * theta8);
    const double scale = thetaD / r;
    return {scale * x, scale * y};
}

std::pair<double, double> undistortProject(const CameraParameter& param, double x, double y) {
    double ux = (x - param.cxOut) / param.fxOut;
    double uy = (y - param.cyOut) / param.fyOut;

    const cv::Vec3d rectified(ux, uy, 1.0);
    const cv::Vec3d camera = param.rotation.t() * rectified;
    ux = camera[0] / camera[2];
    uy = camera[1] / camera[2];

    if (!param.hasDistParams) {
        return {ux * param.fx + param.cx, uy * param.fy + param.cy};
    }

    const auto distorted = fisheyeDistort(param.distParams, ux, uy);
    return {distorted.first * param.fx + param.cx, distorted.second * param.fy + param.cy};
}

AX_U32 roundUpDiv(AX_U32 x, AX_U32 a) { return (x + (a - 1)) / a; }

AX_U32 alignUp(AX_U32 x, AX_U32 a) { return roundUpDiv(x, a) * a; }

void generateMeshTableM76Gdc(const CameraParameter& param, std::vector<AX_U64>& meshData,
                             AX_U32& meshCellWidth, AX_U32& meshCellHeight) {
    meshCellWidth = alignUp(roundUpDiv(param.outputWidth, kMeshTableCols - 1), kMeshAlignSize);
    meshCellHeight = alignUp(roundUpDiv(param.outputHeight, kMeshTableRows - 1), kMeshAlignSize);

    meshData.clear();
    meshData.reserve((kMeshTableRows * (kMeshTableCols + kMeshPaddingSize)) * 2);
    for (AX_U32 row = 0; row < kMeshTableRows; ++row) {
        const AX_U32 rowIndex = row * meshCellHeight;
        for (AX_U32 col = 0; col < kMeshTableCols; ++col) {
            const AX_U32 colIndex = col * meshCellWidth;
            auto projected = undistortProject(param, static_cast<double>(colIndex),
                                              static_cast<double>(rowIndex));
            projected.first =
                std::max(0.0, std::min(projected.first, static_cast<double>(param.imageWidth - 1)));
            projected.second = std::max(
                0.0, std::min(projected.second, static_cast<double>(param.imageHeight - 1)));

            const int64_t diffCols =
                static_cast<int64_t>((projected.first - colIndex) * kMeshDiffFixFactor);
            const int64_t diffRows =
                static_cast<int64_t>((projected.second - rowIndex) * kMeshDiffFixFactor);
            const AX_U64 meshValue = (static_cast<AX_U64>(static_cast<uint32_t>(diffRows)) << 32) |
                                     static_cast<AX_U64>(static_cast<uint32_t>(diffCols));
            meshData.push_back(meshValue);
            meshData.push_back(0);
        }

        for (AX_U32 padding = 0; padding < kMeshPaddingSize; ++padding) {
            meshData.push_back(0);
            meshData.push_back(0);
        }
    }
}

bool dumpMeshTable(const std::string& path, const std::vector<AX_U64>& meshData) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << std::uppercase << std::hex << std::setfill('0');

    for (size_t index = 0; index < meshData.size(); ++index) {
        ofs << std::setw(16) << meshData[index];
        if (((index + 1) % 2) == 0) {
            ofs << '\n';
        }
    }

    if ((meshData.size() % 2) != 0) {
        ofs << '\n';
    }

    return static_cast<bool>(ofs);
}

bool isSafeCalibrationSerial(const std::string& serialNumber) {
    if (serialNumber.empty()) {
        return false;
    }

    for (unsigned char ch : serialNumber) {
        if (!std::isalnum(ch) && ch != '-' && ch != '_') {
            return false;
        }
    }
    return true;
}

size_t writeCalibrationFile(void* buffer, size_t size, size_t nmemb, void* userdata) {
    if (userdata == nullptr) {
        return 0;
    }
    return std::fwrite(buffer, size, nmemb, static_cast<std::FILE*>(userdata));
}

CURLcode ensureCurlInitialized() {
    static std::once_flag initOnce;
    static CURLcode initResult = CURLE_OK;
    std::call_once(initOnce, []() { initResult = curl_global_init(CURL_GLOBAL_DEFAULT); });
    return initResult;
}

std::filesystem::path calibrationRuntimeDirectoryPath() {
    static const std::filesystem::path resolvedPath = []() {
        std::error_code ec;
        const std::filesystem::path procExe("/proc/self/exe");
        const std::filesystem::path executablePath = std::filesystem::read_symlink(procExe, ec);
        if (!ec && !executablePath.empty()) {
            return executablePath.parent_path() / "zed";
        }

        ec.clear();
        const std::filesystem::path currentPath = std::filesystem::current_path(ec);
        if (!ec && !currentPath.empty()) {
            return currentPath / "zed";
        }

        return std::filesystem::path("./zed");
    }();

    return resolvedPath;
}

}  // namespace

std::string calibrationRuntimeDirectory() { return calibrationRuntimeDirectoryPath().string(); }

std::string calibrationIniPath(const std::string& serialNumber) {
    const std::filesystem::path runtimeDirectory = calibrationRuntimeDirectoryPath();
    const std::array<std::filesystem::path, 2> candidatePaths = {
        runtimeDirectory / ("zed_" + serialNumber + ".ini"),
        runtimeDirectory / ("SN" + serialNumber + ".conf"),
    };

    for (const auto& candidatePath : candidatePaths) {
        std::error_code ec;
        if (std::filesystem::exists(candidatePath, ec) && !ec) {
            return candidatePath.string();
        }
    }

    return candidatePaths[0].string();
}

int generateMeshFiles(const std::string& serialNumber, bool forceRegenerate,
                      std::string& leftMeshPath, std::string& rightMeshPath, uint32_t inputWidth,
                      uint32_t inputHeight) {
    const std::filesystem::path calibrationPath = calibrationIniPath(serialNumber);
    const std::string baseName = calibrationPath.stem().string();
    CalibrationGeometry geometry;
    std::string error;
    if (!resolveCalibrationGeometry(inputWidth, inputHeight, geometry, error)) {
        ALOGE("%s", error.c_str());
        return -1;
    }

    std::string resolutionSuffix = geometry.resolution;
    std::transform(resolutionSuffix.begin(), resolutionSuffix.end(), resolutionSuffix.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    leftMeshPath =
        (calibrationPath.parent_path() / (baseName + "_" + resolutionSuffix + "_l_32x16.txt"))
            .string();
    rightMeshPath =
        (calibrationPath.parent_path() / (baseName + "_" + resolutionSuffix + "_r_32x16.txt"))
            .string();

    if (!forceRegenerate && isNonEmptyFile(leftMeshPath) && isNonEmptyFile(rightMeshPath)) {
        ALOGN("Mesh file exists, reuse: %s", leftMeshPath.c_str());
        ALOGN("Mesh file exists, reuse: %s", rightMeshPath.c_str());
        return 0;
    }

    StereoCalibrationData calibration;
    if (!loadStereoCalibration(calibrationPath.string(), inputWidth, inputHeight, geometry,
                               calibration, error)) {
        ALOGE("%s", error.c_str());
        return -1;
    }

    RectificationData rectification;
    if (!computeRectification(calibration, geometry, rectification, error)) {
        ALOGE("%s", error.c_str());
        return -1;
    }

    CameraParameter leftParam;
    leftParam.imageWidth = geometry.perEyeWidth;
    leftParam.imageHeight = geometry.height;
    leftParam.outputWidth = kMeshOutputWidth;
    leftParam.outputHeight = kMeshOutputHeight;
    leftParam.cxOut = kMeshOutputCx;
    leftParam.cyOut = kMeshOutputCy;
    leftParam.fxOut = kMeshOutputFx;
    leftParam.fyOut = kMeshOutputFy;
    leftParam.fx = calibration.K1(0, 0);
    leftParam.fy = calibration.K1(1, 1);
    leftParam.cx = calibration.K1(0, 2);
    leftParam.cy = calibration.K1(1, 2);
    leftParam.rotation = rectification.R1;
    leftParam.distParams = {calibration.D1[0], calibration.D1[1], calibration.D1[2],
                            calibration.D1[3]};
    leftParam.hasDistParams = true;

    CameraParameter rightParam;
    rightParam.imageWidth = geometry.perEyeWidth;
    rightParam.imageHeight = geometry.height;
    rightParam.outputWidth = kMeshOutputWidth;
    rightParam.outputHeight = kMeshOutputHeight;
    rightParam.cxOut = kMeshOutputCx;
    rightParam.cyOut = kMeshOutputCy;
    rightParam.fxOut = kMeshOutputFx;
    rightParam.fyOut = kMeshOutputFy;
    rightParam.fx = calibration.K2(0, 0);
    rightParam.fy = calibration.K2(1, 1);
    rightParam.cx = calibration.K2(0, 2);
    rightParam.cy = calibration.K2(1, 2);
    rightParam.rotation = rectification.R2;
    rightParam.distParams = {calibration.D2[0], calibration.D2[1], calibration.D2[2],
                             calibration.D2[3]};
    rightParam.hasDistParams = true;

    AX_U32 leftMeshWidth = 0;
    AX_U32 leftMeshHeight = 0;
    AX_U32 rightMeshWidth = 0;
    AX_U32 rightMeshHeight = 0;
    std::vector<AX_U64> leftMeshData;
    std::vector<AX_U64> rightMeshData;
    generateMeshTableM76Gdc(leftParam, leftMeshData, leftMeshWidth, leftMeshHeight);
    generateMeshTableM76Gdc(rightParam, rightMeshData, rightMeshWidth, rightMeshHeight);

    leftMeshPath = (calibrationPath.parent_path() /
                    (baseName + "_" + resolutionSuffix + "_l_" + std::to_string(leftMeshWidth) +
                     "x" + std::to_string(leftMeshHeight) + ".txt"))
                       .string();
    rightMeshPath = (calibrationPath.parent_path() /
                     (baseName + "_" + resolutionSuffix + "_r_" + std::to_string(rightMeshWidth) +
                      "x" + std::to_string(rightMeshHeight) + ".txt"))
                        .string();

    if (!dumpMeshTable(leftMeshPath, leftMeshData) ||
        !dumpMeshTable(rightMeshPath, rightMeshData)) {
        ALOGE("dump mesh file failed: left=%s right=%s", leftMeshPath.c_str(),
              rightMeshPath.c_str());
        return -1;
    }

    ALOGN("%s%s", forceRegenerate ? "Mesh file regenerated: " : "Mesh file generated: ",
          leftMeshPath.c_str());
    ALOGN("%s%s", forceRegenerate ? "Mesh file regenerated: " : "Mesh file generated: ",
          rightMeshPath.c_str());
    return 0;
}

int downloadCalibrationFile(const std::string& serialNumber) {
    if (serialNumber.empty()) {
        return 0;
    }
    if (!isSafeCalibrationSerial(serialNumber)) {
        ALOGE("invalid calibration serial number: %s", serialNumber.c_str());
        return -1;
    }

    std::error_code ec;
    const std::filesystem::path runtimeDirectory = calibrationRuntimeDirectoryPath();
    std::filesystem::create_directories(runtimeDirectory, ec);
    if (ec) {
        ALOGE("create calibration directory failed: %s, ec=%d", runtimeDirectory.c_str(),
              ec.value());
        return -1;
    }

    const std::string outputPath = calibrationIniPath(serialNumber);
    if (std::filesystem::exists(outputPath, ec) && !ec) {
        const auto fileSize = std::filesystem::file_size(outputPath, ec);
        if (!ec && fileSize > 0) {
            ALOGN("Calibration file exists, reuse: %s", outputPath.c_str());
            ALOGI("reuse calibration file: %s", outputPath.c_str());
            return 0;
        }
        ec.clear();
    }

    const std::string url =
        std::string("https://www.stereolabs.com/developers/calib/?SN=") + serialNumber;
    const std::string tempPath = outputPath + ".tmp";

    std::filesystem::remove(tempPath, ec);

    const CURLcode initResult = ensureCurlInitialized();
    if (initResult != CURLE_OK) {
        ALOGE("curl global init failed: %s", curl_easy_strerror(initResult));
        return -1;
    }

    std::FILE* tempFile = std::fopen(tempPath.c_str(), "wb");
    if (tempFile == nullptr) {
        ALOGE("open temp calibration file failed: %s", tempPath.c_str());
        return -1;
    }

    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        std::fclose(tempFile);
        std::filesystem::remove(tempPath, ec);
        ALOGE("curl easy init failed");
        return -1;
    }

    char errorBuffer[CURL_ERROR_SIZE] = {0};
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla/4.0");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &writeCalibrationFile);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, tempFile);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

    const CURLcode performResult = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    std::fclose(tempFile);

    if (performResult != CURLE_OK) {
        std::filesystem::remove(tempPath, ec);
        ALOGE("download calibration file failed: serial=%s err=%s detail=%s", serialNumber.c_str(),
              curl_easy_strerror(performResult), errorBuffer[0] != '\0' ? errorBuffer : "n/a");
        return -1;
    }

    const auto tempSize = std::filesystem::file_size(tempPath, ec);
    if (ec || tempSize == 0) {
        std::filesystem::remove(tempPath, ec);
        ALOGE("download calibration file produced empty output: %s", tempPath.c_str());
        return -1;
    }

    std::error_code renameEc;
    std::filesystem::rename(tempPath, outputPath, renameEc);
    if (renameEc) {
        std::filesystem::remove(tempPath, ec);
        ALOGE("rename calibration file failed: %s -> %s, ec=%d", tempPath.c_str(),
              outputPath.c_str(), renameEc.value());
        return -1;
    }

    ALOGN("Calibration file downloaded: %s", outputPath.c_str());
    ALOGI("calibration file ready: %s", outputPath.c_str());
    return 0;
}

bool tryLoadCalibrationBaselineMeters(const std::string& serialNumber, float& baselineMeters) {
    if (serialNumber.empty()) {
        return false;
    }
    if (!isSafeCalibrationSerial(serialNumber)) {
        ALOGW("skip calibration baseline load due to invalid serial: %s", serialNumber.c_str());
        return false;
    }

    const std::filesystem::path calibrationPath = calibrationIniPath(serialNumber);
    if (!isNonEmptyFile(calibrationPath.string())) {
        return false;
    }

    IniMap iniValues;
    std::string error;
    if (!parseIniFile(calibrationPath.string(), iniValues, error)) {
        ALOGW("load calibration baseline failed: %s", error.c_str());
        return false;
    }

    double baselineMillimeters = 0.0;
    if (!getIniDouble(iniValues, "STEREO", "Baseline", baselineMillimeters)) {
        ALOGW("calibration baseline missing in %s", calibrationPath.c_str());
        return false;
    }

    baselineMeters = static_cast<float>(baselineMillimeters / 1000.0);
    return true;
}

}  // namespace stereo_depth
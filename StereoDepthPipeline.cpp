#include "StereoDepthPipeline.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <queue>
#include <sstream>
#include <thread>

#include "AxclImageProc.hpp"
#include "ImageProcBackend.hpp"
#include "sample_engine.h"
#include "sample_image.h"

#define SAMPLE_LOG_TAG "PIPELINE"
#include "sample_log.h"

namespace stereo_depth {

const char* inferenceEngineName(InferenceEngine engine) {
    switch (engine) {
        case InferenceEngine::NPU:
            return "npu";
        default:
            return "unknown";
    }
}

namespace {

std::filesystem::path executableAdjacentPath(const char* childName) {
    std::error_code ec;
    const std::filesystem::path executablePath =
        std::filesystem::read_symlink(std::filesystem::path("/proc/self/exe"), ec);
    if (ec || executablePath.empty()) {
        return {};
    }
    return executablePath.parent_path() / childName;
}

const std::string& defaultNpuModelPathString() {
    static const std::string resolvedPath = []() {
        std::error_code ec;

        if (const char* envPath = std::getenv("STEREO_DEPTH_MODEL_DIR");
            envPath != nullptr && envPath[0] != '\0') {
            const std::filesystem::path candidate =
                std::filesystem::path(envPath) / "axstereo_pro.axmodel";
            if (std::filesystem::exists(candidate, ec) && !ec) {
                return candidate.string();
            }
        }

        {
            const std::filesystem::path candidate =
                executableAdjacentPath("models") / "axstereo_pro.axmodel";
            if (!candidate.empty() && std::filesystem::exists(candidate, ec) && !ec) {
                return candidate.string();
            }
        }

        {
            const std::filesystem::path candidate =
                std::filesystem::path(STEREO_DEPTH_APP_MODEL_DIR) / "axstereo_pro.axmodel";
            if (std::filesystem::exists(candidate, ec) && !ec) {
                return candidate.string();
            }
        }

        {
            const std::filesystem::path candidate =
                std::filesystem::path("/opt/bin/sample_stereo_depth/models") /
                "axstereo_pro.axmodel";
            if (std::filesystem::exists(candidate, ec) && !ec) {
                return candidate.string();
            }
        }

        return (std::filesystem::path(STEREO_DEPTH_APP_MODEL_DIR) / "axstereo_pro.axmodel")
            .string();
    }();

    return resolvedPath;
}

class ThreadPool {
public:
    explicit ThreadPool(size_t workerCount) {
        if (workerCount == 0) {
            workerCount = 1;
        }
        workers_.reserve(workerCount);
        for (size_t i = 0; i < workerCount; ++i) {
            workers_.emplace_back([this]() {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mutex_);
                        cv_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });
                        if (stop_ && tasks_.empty()) {
                            return;
                        }
                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }
                    task();
                }
            });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void enqueue(std::function<void()> func) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace(std::move(func));
        }
        cv_.notify_one();
    }

    size_t workerCount() const { return workers_.size(); }

    template <typename Fn>
    void parallelForRows(int totalRows, int taskCount, Fn&& func) {
        if (totalRows <= 0) {
            return;
        }
        if (taskCount <= 1) {
            func(0, totalRows);
            return;
        }

        const int clampedTaskCount = std::min(taskCount, totalRows);

        struct TaskGroup {
            explicit TaskGroup(int taskCount) : remaining(taskCount) {}

            void finish(std::exception_ptr error = nullptr) {
                std::lock_guard<std::mutex> lock(mutex);
                if (error && !firstError) {
                    firstError = error;
                }
                --remaining;
                if (remaining == 0) {
                    cv.notify_one();
                }
            }

            void wait() {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [&]() { return remaining == 0; });
                if (firstError) {
                    std::rethrow_exception(firstError);
                }
            }

            std::mutex mutex;
            std::condition_variable cv;
            int remaining;
            std::exception_ptr firstError;
        };

        const int rowsPerTask = totalRows / clampedTaskCount;
        auto taskGroup = std::make_unique<TaskGroup>(clampedTaskCount);

        int rowBegin = 0;
        for (int i = 0; i < clampedTaskCount; ++i) {
            int rowEnd = rowBegin + rowsPerTask;
            if (i == clampedTaskCount - 1) {
                rowEnd = totalRows;
            }

            enqueue([rowBegin, rowEnd, &func, group = taskGroup.get()]() {
                try {
                    func(rowBegin, rowEnd);
                    group->finish();
                } catch (...) {
                    group->finish(std::current_exception());
                }
            });
            rowBegin = rowEnd;
        }

        taskGroup->wait();
    }

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
};

ThreadPool& getPostProcessThreadPool() {
    static ThreadPool pool([]() {
        unsigned int hw = std::thread::hardware_concurrency();
        if (hw == 0) {
            hw = 4;
        }
        return static_cast<size_t>(std::min(std::max(1U, hw - 1U), 7U));
    }());
    static const bool logged = [](size_t workerCount) {
        ALOGI(
            "postprocess thread pool initialized: 1+%zu mode, scheduler thread does not process "
            "pointcloud rows\n",
            workerCount, workerCount);
        return true;
    }(pool.workerCount());
    (void)logged;
    return pool;
}

constexpr AX_S32 PIPE_NUM = 2;
constexpr AX_S32 SGBM_DUAL_CORE_OVERLAP_ROWS = 16;
constexpr AX_F32 SGBM_DISPARITY_SCALE = 16.0f;
constexpr AX_F32 NPU_DISPARITY_OFFSET = 0.3f;

constexpr AX_F32 CAM_CX = static_cast<AX_F32>(kCameraPrincipalPointX);
constexpr AX_F32 CAM_CY = static_cast<AX_F32>(kCameraPrincipalPointY);
constexpr AX_F32 CAM_FX = static_cast<AX_F32>(kCameraFocalLengthXPixels);
constexpr AX_F32 CAM_FY = static_cast<AX_F32>(kCameraFocalLengthYPixels);
constexpr AX_F32 CAM_B = 0.0629277f;
constexpr AX_U32 Z_GRID_CELL_WIDTH = 11;
constexpr AX_U32 Z_GRID_CELL_HEIGHT = 11;

typedef struct {
    AX_F32 x;
    AX_F32 y;
    AX_F32 z;
    AX_U8 a;
    AX_U8 r;
    AX_U8 g;
    AX_U8 b;
} Point3D_ARGB;

// Host GDC: the board dewarped each eye with the IVPS GDC block driven by a
// hardware mesh. On the host the same fisheye rectification is rebuilt from the
// calibration INI as OpenCV remap tables and applied on the CPU.
struct HostGdcMaps {
    bool attempted = false;
    bool ready = false;
    cv::Mat mapX[PIPE_NUM];  // kDewarpImageHeight x kDewarpImageWidth, CV_32FC1
    cv::Mat mapY[PIPE_NUM];
    cv::Mat uvMapX[PIPE_NUM];  // half size for the NV12 chroma plane
    cv::Mat uvMapY[PIPE_NUM];
    float baselineMeters = 0.0f;
};

HostGdcMaps gHostGdc;

void resetHostGdcMaps() { gHostGdc = HostGdcMaps(); }

// Fill gHostGdc from the four per-eye remap vectors (left/right X/Y).
void finalizeHostGdcMaps(std::vector<float>& lx, std::vector<float>& ly, std::vector<float>& rx,
                         std::vector<float>& ry, float baseline) {
    const int w = kDewarpImageWidth;
    const int h = kDewarpImageHeight;
    auto toMat = [&](std::vector<float>& v) { return cv::Mat(h, w, CV_32FC1, v.data()).clone(); };
    gHostGdc.mapX[0] = toMat(lx);
    gHostGdc.mapY[0] = toMat(ly);
    gHostGdc.mapX[1] = toMat(rx);
    gHostGdc.mapY[1] = toMat(ry);
    for (int eye = 0; eye < PIPE_NUM; ++eye) {
        cv::resize(gHostGdc.mapX[eye], gHostGdc.uvMapX[eye], cv::Size(w / 2, h / 2), 0, 0,
                   cv::INTER_NEAREST);
        cv::resize(gHostGdc.mapY[eye], gHostGdc.uvMapY[eye], cv::Size(w / 2, h / 2), 0, 0,
                   cv::INTER_NEAREST);
        gHostGdc.uvMapX[eye] *= 0.5f;
        gHostGdc.uvMapY[eye] *= 0.5f;
    }
    gHostGdc.baselineMeters = baseline;
    gHostGdc.ready = true;
}

bool ensureHostGdcMaps(const std::string& serialNumber, int inputWidth, int inputHeight) {
    if (gHostGdc.attempted) {
        return gHostGdc.ready;
    }
    gHostGdc.attempted = true;

    std::vector<float> lx;
    std::vector<float> ly;
    std::vector<float> rx;
    std::vector<float> ry;
    float baseline = 0.0f;
    if (!buildHostRectifyMaps(serialNumber, static_cast<uint32_t>(inputWidth),
                              static_cast<uint32_t>(inputHeight), lx, ly, rx, ry, baseline)) {
        return false;
    }
    finalizeHostGdcMaps(lx, ly, rx, ry, baseline);
    return true;
}

// Build the host remap tables directly from GDC mesh .txt files (used with the
// generic default mesh, when no calibration INI is available).
bool ensureHostGdcMapsFromMesh(const std::string& leftMeshPath, const std::string& rightMeshPath,
                               float baselineMeters) {
    if (gHostGdc.attempted) {
        return gHostGdc.ready;
    }
    gHostGdc.attempted = true;

    std::vector<float> lx;
    std::vector<float> ly;
    std::vector<float> rx;
    std::vector<float> ry;
    if (!buildHostRectifyMapsFromMesh(leftMeshPath, rightMeshPath, lx, ly, rx, ry)) {
        return false;
    }
    finalizeHostGdcMaps(lx, ly, rx, ry, baselineMeters);
    return true;
}

// Dewarp one eye: src is a (strided) NV12 half-frame, dst a contiguous NV12
// kDewarpImageWidth x kDewarpImageHeight frame.
int hostGdcDewarpEye(int eye, const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) {
    if (eye < 0 || eye >= PIPE_NUM || !gHostGdc.ready) {
        return -1;
    }
    const int inW = static_cast<int>(src->u32Width);
    const int inH = static_cast<int>(src->u32Height);
    const int inStride = static_cast<int>(src->u32PicStride[0]);
    const int outStride = static_cast<int>(dst->u32PicStride[0]);

    cv::Mat yIn(inH, inW, CV_8UC1, reinterpret_cast<void*>(src->u64VirAddr[0]), inStride);
    cv::Mat uvIn(inH / 2, inW / 2, CV_8UC2, reinterpret_cast<void*>(src->u64VirAddr[1]), inStride);
    cv::Mat yOut(static_cast<int>(dst->u32Height), static_cast<int>(dst->u32Width), CV_8UC1,
                 reinterpret_cast<void*>(dst->u64VirAddr[0]), outStride);
    cv::Mat uvOut(static_cast<int>(dst->u32Height) / 2, static_cast<int>(dst->u32Width) / 2,
                  CV_8UC2, reinterpret_cast<void*>(dst->u64VirAddr[1]), outStride);

    cv::remap(yIn, yOut, gHostGdc.mapX[eye], gHostGdc.mapY[eye], cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
    cv::remap(uvIn, uvOut, gHostGdc.uvMapX[eye], gHostGdc.uvMapY[eye], cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
    return 0;
}

bool isNonEmptyFile(const std::string& path) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
        return false;
    }
    return std::filesystem::file_size(path, ec) > 0 && !ec;
}

void disparityToPointCloudArgb(const AX_F32* disparityBuf, const AX_U8* bgrBuf, AX_S32 width,
                               AX_S32 height, AX_F32 fx, AX_F32 b, const std::vector<float>& xScale,
                               const std::vector<float>& yScale,
                               std::vector<std::byte>& pointCloudData) {
    pointCloudData.resize(static_cast<size_t>(width) * static_cast<size_t>(height) *
                          sizeof(Point3D_ARGB));
    auto* pointCloudBuf = reinterpret_cast<Point3D_ARGB*>(pointCloudData.data());
    const AX_F32 zScale = fx * b;

    auto processRows = [&](AX_S32 rowBegin, AX_S32 rowEnd) {
        for (AX_S32 v = rowBegin; v < rowEnd; ++v) {
            const AX_F32 yScaled = yScale[static_cast<size_t>(v)];
            const AX_S32 rowBase = v * width;
            for (AX_S32 u = 0; u < width; ++u) {
                const AX_S32 idx = rowBase + u;
                AX_F32 d = disparityBuf[idx];
                if (d <= 0.0f) {
                    d = 1e-6f;
                }

                const AX_F32 invD = 1.0f / d;
                pointCloudBuf[idx].x = xScale[static_cast<size_t>(u)] * invD;
                pointCloudBuf[idx].y = yScaled * invD;
                pointCloudBuf[idx].z = zScale * invD;
                pointCloudBuf[idx].a = 255;
                pointCloudBuf[idx].r = bgrBuf[idx * 3 + 2];
                pointCloudBuf[idx].g = bgrBuf[idx * 3 + 1];
                pointCloudBuf[idx].b = bgrBuf[idx * 3 + 0];
            }
        }
    };

    ThreadPool& pool = getPostProcessThreadPool();
    unsigned int workerCount = static_cast<unsigned int>(pool.workerCount());
    workerCount = std::max(1U, workerCount);
    workerCount = std::min(workerCount, static_cast<unsigned int>(height));

    pool.parallelForRows(height, static_cast<int>(workerCount), [&](int rowBegin, int rowEnd) {
        processRows(static_cast<AX_S32>(rowBegin), static_cast<AX_S32>(rowEnd));
    });
}

void disparityFloatToU16(const AX_F32* disparityBuf, AX_S32 width, AX_S32 height, AX_F32 maxDisp,
                         std::vector<std::byte>& depthData) {
    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    depthData.resize(pixelCount * sizeof(AX_U16));
    auto* u16Output = reinterpret_cast<AX_U16*>(depthData.data());

    for (size_t i = 0; i < pixelCount; ++i) {
        AX_F32 disp = disparityBuf[i];
        if (disp < 0.0f) {
            disp = 0.0f;
        } else if (disp > maxDisp) {
            disp = maxDisp;
        }

        AX_U16 u16Fixed = static_cast<AX_U16>(disp * 16.0f + 0.5f);
        u16Output[i] = u16Fixed;
    }
}

void copyFloatOutputImage(const AX_F32* srcBuf, AX_S32 width, AX_S32 height,
                          std::vector<std::byte>& outputData) {
    if (srcBuf == nullptr || width <= 0 || height <= 0) {
        outputData.clear();
        return;
    }

    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    outputData.resize(pixelCount * sizeof(AX_F32));
    std::memcpy(outputData.data(), srcBuf, outputData.size());
}

void copyAdjustedNpuDisparityImage(const AX_F32* srcBuf, AX_S32 width, AX_S32 height, AX_F32 offset,
                                   std::vector<AX_F32>& outputData) {
    if (srcBuf == nullptr || width <= 0 || height <= 0) {
        outputData.clear();
        return;
    }

    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    outputData.resize(pixelCount);
    for (size_t i = 0; i < pixelCount; ++i) {
        outputData[i] = srcBuf[i] + offset;
    }
}

void disparityToGridAveragedZFloatImage(const AX_F32* disparityBuf, AX_U32 width, AX_U32 height,
                                        AX_F32 f, AX_F32 B, AX_U32 cellWidth, AX_U32 cellHeight,
                                        std::vector<std::byte>& zGridAvgData) {
    const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
    zGridAvgData.resize(pixelCount * sizeof(AX_F32));
    auto* zOutput = reinterpret_cast<AX_F32*>(zGridAvgData.data());

    if (width == 0 || height == 0 || cellWidth == 0 || cellHeight == 0) {
        return;
    }

    const AX_F32 zScale = f * B;
    for (AX_U32 y0 = 0; y0 < height; y0 += cellHeight) {
        const AX_U32 y1 = std::min(height, y0 + cellHeight);
        for (AX_U32 x0 = 0; x0 < width; x0 += cellWidth) {
            const AX_U32 x1 = std::min(width, x0 + cellWidth);

            AX_F32 sumZ = 0.0f;
            AX_U32 count = 0;
            for (AX_U32 y = y0; y < y1; ++y) {
                const size_t rowBase = static_cast<size_t>(y) * width;
                for (AX_U32 x = x0; x < x1; ++x) {
                    const size_t idx = rowBase + x;
                    AX_F32 d = disparityBuf[idx];
                    if (d <= 0.0f) {
                        d = 1e-6f;
                    }
                    sumZ += zScale / d;
                    ++count;
                }
            }

            const AX_F32 avgZ = (count > 0) ? (sumZ / static_cast<AX_F32>(count)) : 0.0f;
            for (AX_U32 y = y0; y < y1; ++y) {
                const size_t rowBase = static_cast<size_t>(y) * width;
                for (AX_U32 x = x0; x < x1; ++x) {
                    zOutput[rowBase + x] = avgZ;
                }
            }
        }
    }
}

void convertDisparityU16ToDepthAndFloat(const AX_U16* disparityBuf, AX_U32 width, AX_U32 height,
                                        AX_U32 pitch, std::vector<std::byte>& depthData,
                                        std::vector<AX_F32>& disparityFloat) {
    depthData.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * sizeof(AX_U16));
    disparityFloat.resize(static_cast<size_t>(width) * static_cast<size_t>(height));

    AX_U16* dst = reinterpret_cast<AX_U16*>(depthData.data());
    AX_F32* dispDst = disparityFloat.data();
    for (AX_U32 h = 0; h < height; ++h) {
        const AX_U16* srcRow = disparityBuf + static_cast<size_t>(h) * pitch;
        AX_U16* depthRow = dst + static_cast<size_t>(h) * width;
        AX_F32* dispRow = dispDst + static_cast<size_t>(h) * width;
        for (AX_U32 w = 0; w < width; ++w) {
            const AX_U16 d = srcRow[w];
            depthRow[w] = d;
            dispRow[w] = static_cast<AX_F32>(d) / SGBM_DISPARITY_SCALE;
        }
    }
}

}  // namespace

namespace {

void releaseFrameContextInternal(StereoDepthPipeline::FrameContext& context);

}  // namespace

struct StereoDepthPipeline::FrameContext {
    SAMPLE_RESOURCE_T stResource = {0};
    SAMPLE_MODEL_OUTPUT_T stOutput = {0};
    bool frameCreated = false;
    bool gdcApplied = false;
    bool borrowedCscFrame = false;
    AX_VIDEO_FRAME_T ownedCscFrame = {};
    PipelineOutput output;
    std::shared_ptr<void> inputFrameOwner;

    FrameContext() { stOutput.blockId = AX_INVALID_BLOCKID; }
    ~FrameContext();
};

namespace {

void releaseFrameContextInternal(StereoDepthPipeline::FrameContext& context) {
    // The NPU output now lives in a host buffer owned by this context.
    sample_engine_release_output(&context.stOutput);
    if (context.borrowedCscFrame) {
        context.stResource.cscFrame = context.ownedCscFrame;
        std::memset(&context.ownedCscFrame, 0, sizeof(context.ownedCscFrame));
        context.borrowedCscFrame = false;
    }
    if (context.frameCreated) {
        sample_release_image_frame(&context.stResource);
        context.frameCreated = false;
    }
    context.inputFrameOwner.reset();
}

}  // namespace

StereoDepthPipeline::FrameContext::~FrameContext() { releaseFrameContextInternal(*this); }

const char* defaultNpuModelPath() { return defaultNpuModelPathString().c_str(); }

StereoDepthPipeline::~StereoDepthPipeline() { shutdown(); }

int StereoDepthPipeline::initialize(InferenceEngine engine, bool enableGdc, GdcMeshMode gdcMeshMode,
                                    bool exportVoFrames, const std::string& npuModelPath,
                                    const std::string& cameraSerialNumber, int inputWidth,
                                    int inputHeight) {
    m_inferenceEngine = engine;
    m_enableGdc = enableGdc;
    m_exportVoFrames = exportVoFrames;
    m_gdcMeshMode = gdcMeshMode;
    m_inputWidth = inputWidth;
    m_inputHeight = inputHeight;
    m_meshLeftPath = "/opt/data/npu_disp/mesh_left.txt";
    m_meshRightPath = "/opt/data/npu_disp/mesh_right.txt";
    m_cameraBaselineMeters = CAM_B;
    m_pointCloudScaleWidth = 0;
    m_pointCloudScaleHeight = 0;
    m_pointCloudScaleBaselineMeters = -1.0f;

    AX_S32 ret = 0;

    if (m_inputWidth <= 0 || m_inputHeight <= 0 || (m_inputWidth % 2) != 0 ||
        !isSupportedStereoInputResolution(m_inputWidth, m_inputHeight)) {
        ALOGE("unsupported input geometry: %dx%d", m_inputWidth, m_inputHeight);
        return -1;
    }

    // Resolve the GDC mesh: prefer the per-camera dynamic mesh (serial INI),
    // otherwise fall back to the generic default mesh shipped under models/.
    bool meshReady = false;
    bool usingDefaultMesh = false;
    std::string defaultMeshLeft;
    std::string defaultMeshRight;
    const bool haveDefaultMesh = resolveDefaultMeshPaths(static_cast<uint32_t>(m_inputWidth),
                                                         static_cast<uint32_t>(m_inputHeight),
                                                         defaultMeshLeft, defaultMeshRight);

    if (m_enableGdc) {
        if (m_gdcMeshMode != GdcMeshMode::Default && !cameraSerialNumber.empty()) {
            const std::string cachedCalibrationIniPath = calibrationIniPath(cameraSerialNumber);
            ALOGN("Dynamic calibration runtime directory: %s",
                  calibrationRuntimeDirectory().c_str());
            bool iniReady = false;
            if (isNonEmptyFile(cachedCalibrationIniPath)) {
                ALOGN("Calibration INI exists, reuse: %s", cachedCalibrationIniPath.c_str());
                iniReady = true;
            } else if (downloadCalibrationFile(cameraSerialNumber) == 0) {
                iniReady = true;
            } else {
                ALOGW("calibration INI missing and download failed: serial=%s",
                      cameraSerialNumber.c_str());
            }
            if (iniReady &&
                generateMeshFiles(cameraSerialNumber, m_gdcMeshMode == GdcMeshMode::DynamicForce,
                                  m_meshLeftPath, m_meshRightPath,
                                  static_cast<uint32_t>(m_inputWidth),
                                  static_cast<uint32_t>(m_inputHeight)) == 0) {
                meshReady = true;
            } else {
                ALOGW(
                    "dynamic mesh unavailable, falling back to default mesh: serial=%s "
                    "input=%dx%d",
                    cameraSerialNumber.c_str(), m_inputWidth, m_inputHeight);
            }
        }

        if (!meshReady) {
            // No serial number, builtin mesh mode, or dynamic generation failed:
            // use the generic default mesh table shipped under models/.
            m_cameraBaselineMeters = CAM_B;
            if (haveDefaultMesh) {
                m_meshLeftPath = defaultMeshLeft;
                m_meshRightPath = defaultMeshRight;
                meshReady = true;
                usingDefaultMesh = true;
                ALOGN("GDC uses default mesh files: %s, %s", m_meshLeftPath.c_str(),
                      m_meshRightPath.c_str());
            } else {
                ALOGW(
                    "no default mesh found for %dx%d under models/ (mesh_<res>_l/r_32x16.txt); "
                    "GDC will fall back to resize",
                    m_inputWidth, m_inputHeight);
            }
        }
    }

    AX_F32 calibrationBaselineMeters = 0.0f;
    if (m_enableGdc && !usingDefaultMesh &&
        tryLoadCalibrationBaselineMeters(cameraSerialNumber, calibrationBaselineMeters)) {
        m_cameraBaselineMeters = calibrationBaselineMeters;
        ALOGN("use calibration baseline: serial=%s baseline=%.6f m", cameraSerialNumber.c_str(),
              m_cameraBaselineMeters);
    }

    // Host/AXCL port: there is no on-chip SYS/IVPS/DSP to bring up here. Image
    // pre/post-processing runs on the host CPU (OpenCV) and the NPU runs on the
    // Axera card via AXCL, which is initialized inside sample_npu_init().
    if (m_inferenceEngine != InferenceEngine::NPU) {
        ALOGE("unsupported inference engine for host/AXCL port");
        return -1;
    }

    // Set up the GDC dewarp. Prefer the card (AXCL_IVPS_Dewarp) when the AXCL
    // image-processing backend is active; otherwise build host OpenCV remap
    // tables (from the calibration INI when available, else from the mesh .txt).
    resetHostGdcMaps();
    if (m_enableGdc && meshReady) {
        bool axclDewarp = false;
        if (stereo_depth::imageProcUsesAxcl()) {
            if (host_backend::axclDewarpInit(m_meshLeftPath, m_meshRightPath, m_inputWidth / 2,
                                             m_inputHeight, kDewarpImageWidth,
                                             kDewarpImageHeight)) {
                axclDewarp = true;
                ALOGN("AXCL GDC dewarp enabled (AXCL_IVPS_Dewarp on the card)");
            } else {
                ALOGW("AXCL GDC dewarp init failed, falling back to host remap");
            }
        }
        if (!axclDewarp) {
            bool hostOk = false;
            if (usingDefaultMesh) {
                hostOk = ensureHostGdcMapsFromMesh(m_meshLeftPath, m_meshRightPath,
                                                   m_cameraBaselineMeters);
                if (hostOk) {
                    ALOGN("host GDC dewarp enabled (OpenCV remap from default mesh)");
                }
            } else {
                hostOk = ensureHostGdcMaps(cameraSerialNumber, m_inputWidth, m_inputHeight);
                if (hostOk) {
                    if (gHostGdc.baselineMeters > 0.0f) {
                        m_cameraBaselineMeters = gHostGdc.baselineMeters;
                    }
                    ALOGN("host GDC dewarp enabled (OpenCV remap from calibration)");
                }
            }
            if (!hostOk) {
                ALOGW("host GDC maps unavailable, using stretched resize (try -g off to silence)");
            }
        }
    }

    const std::string resolvedModelPath =
        npuModelPath.empty() ? defaultNpuModelPathString() : npuModelPath;
    ALOGN("Resolved NPU model path: %s", resolvedModelPath.c_str());

    ret = sample_npu_init(resolvedModelPath.c_str());
    if (ret != 0) {
        ALOGE("sample_npu_init fail, ret %d, model=%s!", ret, resolvedModelPath.c_str());
        return ret;
    }
    m_npuInited = true;

    ALOGN("StereoDepthPipeline initialized with engine=%s gdc=%s (host/AXCL)",
          inferenceEngineName(m_inferenceEngine), m_enableGdc ? "on" : "off");

    return 0;
}

int StereoDepthPipeline::preprocessPreparedNv12(FrameContextPtr& context) {
    if (!context) {
        return -1;
    }

    FrameContext& frameContext = *context;
    AX_S32 ret = 0;

    AX_VIDEO_FRAME_T frameLeft = frameContext.stResource.cscFrame;
    frameLeft.u32Width = frameContext.stResource.cscFrame.u32Width >> 1;

    AX_VIDEO_FRAME_T frameRight = frameContext.stResource.cscFrame;
    frameRight.u32Width = frameContext.stResource.cscFrame.u32Width >> 1;
    frameRight.u64PhyAddr[0] = frameContext.stResource.cscFrame.u64PhyAddr[0] + frameRight.u32Width;
    frameRight.u64VirAddr[0] =
        frameContext.stResource.cscFrame.u64VirAddr[0] != 0
            ? frameContext.stResource.cscFrame.u64VirAddr[0] + frameRight.u32Width
            : 0;
    frameRight.u64PhyAddr[1] = frameContext.stResource.cscFrame.u64PhyAddr[1] + frameRight.u32Width;
    frameRight.u64VirAddr[1] =
        frameContext.stResource.cscFrame.u64VirAddr[1] != 0
            ? frameContext.stResource.cscFrame.u64VirAddr[1] + frameRight.u32Width
            : 0;

    AX_VIDEO_FRAME_T* stereoLeftFrame = &frameContext.stResource.resizeFrame[0];
    AX_VIDEO_FRAME_T* stereoRightFrame = &frameContext.stResource.resizeFrame[1];

    bool gdcDone = false;
    // Prefer the card-side dewarp (AXCL_IVPS_Dewarp) when --imgproc axcl made it
    // available; otherwise use the host OpenCV remap.
    if (m_enableGdc && host_backend::axclDewarpReady()) {
        if (host_backend::axclDewarpEye(0, &frameLeft, &frameContext.stResource.dewarpFrame[0]) &&
            host_backend::axclDewarpEye(1, &frameRight, &frameContext.stResource.dewarpFrame[1])) {
            stereoLeftFrame = &frameContext.stResource.dewarpFrame[0];
            stereoRightFrame = &frameContext.stResource.dewarpFrame[1];
            frameContext.gdcApplied = true;
            gdcDone = true;
        } else {
            ALOGW("AXCL GDC dewarp failed, falling back to resize for this frame");
        }
    }

    if (!gdcDone && m_enableGdc && gHostGdc.ready) {
        if (hostGdcDewarpEye(0, &frameLeft, &frameContext.stResource.dewarpFrame[0]) == 0 &&
            hostGdcDewarpEye(1, &frameRight, &frameContext.stResource.dewarpFrame[1]) == 0) {
            stereoLeftFrame = &frameContext.stResource.dewarpFrame[0];
            stereoRightFrame = &frameContext.stResource.dewarpFrame[1];
            frameContext.gdcApplied = true;
            gdcDone = true;
        } else {
            ALOGW("host GDC dewarp failed, falling back to resize for this frame");
        }
    }

    if (!gdcDone) {
        // No GDC (either disabled or calibration maps unavailable): stretch each
        // stereo half to the model input size.
        ret = sample_crop_resize(&frameLeft, &frameContext.stResource.resizeFrame[0]);
        if (ret) {
            ALOGE("sample_crop_resize left fail, ret %d!", ret);
            releaseFrameContextInternal(frameContext);
            context.reset();
            return ret;
        }

        ret = sample_crop_resize(&frameRight, &frameContext.stResource.resizeFrame[1]);
        if (ret) {
            ALOGE("sample_crop_resize right fail, ret %d!", ret);
            releaseFrameContextInternal(frameContext);
            context.reset();
            return ret;
        }
    }

    ret = sample_csc_nv12_bgr(stereoLeftFrame, &frameContext.stResource.bgrFrame);
    if (ret) {
        ALOGE("sample_csc_nv12_bgr fail, ret %d!", ret);
        releaseFrameContextInternal(frameContext);
        context.reset();
        return ret;
    }

    const size_t rgbSize = static_cast<size_t>(kDewarpImageWidth) * kDewarpImageHeight * 3;
    frameContext.output.rgbData.resize(rgbSize);
    std::memcpy(frameContext.output.rgbData.data(),
                reinterpret_cast<void*>(frameContext.stResource.bgrFrame.u64VirAddr[0]), rgbSize);

    (void)stereoRightFrame;
    return 0;
}

int StereoDepthPipeline::preprocessFrame(FrameContextPtr& context, const void* inputFrame,
                                         size_t inputFrameSize, uint64_t frameTimestampNs,
                                         InputFrameFormat inputFrameFormat) {
    context = std::make_shared<FrameContext>();
    FrameContext& frameContext = *context;
    frameContext.output.frameTimestampNs = frameTimestampNs;
    frameContext.output.rawWidth = static_cast<uint32_t>(m_inputWidth);
    frameContext.output.rawHeight = static_cast<uint32_t>(m_inputHeight);

    AX_S32 ret =
        sample_create_image_frame(&frameContext.stResource, static_cast<AX_U32>(m_inputWidth),
                                  static_cast<AX_U32>(m_inputHeight));
    if (ret) {
        ALOGE("sample_create_image_frame fail, ret %d!", ret);
        context.reset();
        return ret;
    }
    frameContext.frameCreated = true;

    const size_t requiredInputSize =
        inputFrameFormat == InputFrameFormat::Nv12
            ? (static_cast<size_t>(m_inputWidth) * static_cast<size_t>(m_inputHeight) * 3 / 2)
            : static_cast<size_t>(frameContext.stResource.srcFrame.u32FrameSize);
    if (inputFrame == nullptr || inputFrameSize < requiredInputSize) {
        ALOGE("invalid input frame, size %zu (expect >= %zu)", inputFrameSize, requiredInputSize);
        releaseFrameContextInternal(frameContext);
        context.reset();
        return -1;
    }

    if (inputFrameFormat == InputFrameFormat::Nv12) {
        const auto* src = reinterpret_cast<const std::byte*>(inputFrame);
        const size_t yPlaneBytes =
            static_cast<size_t>(m_inputWidth) * static_cast<size_t>(m_inputHeight);
        auto* dstY = reinterpret_cast<std::byte*>(frameContext.stResource.cscFrame.u64VirAddr[0]);
        auto* dstUv = reinterpret_cast<std::byte*>(frameContext.stResource.cscFrame.u64VirAddr[1]);
        const size_t dstYStride = frameContext.stResource.cscFrame.u32PicStride[0];
        const size_t dstUvStride = frameContext.stResource.cscFrame.u32PicStride[1] != 0
                                       ? frameContext.stResource.cscFrame.u32PicStride[1]
                                       : dstYStride;

        for (int row = 0; row < m_inputHeight; ++row) {
            std::memcpy(dstY + static_cast<size_t>(row) * dstYStride,
                        src + static_cast<size_t>(row) * static_cast<size_t>(m_inputWidth),
                        static_cast<size_t>(m_inputWidth));
        }
        for (int row = 0; row < (m_inputHeight / 2); ++row) {
            std::memcpy(
                dstUv + static_cast<size_t>(row) * dstUvStride,
                src + yPlaneBytes + static_cast<size_t>(row) * static_cast<size_t>(m_inputWidth),
                static_cast<size_t>(m_inputWidth));
        }
    } else {
        const size_t rawYuyvSize =
            static_cast<size_t>(frameContext.stResource.srcFrame.u32FrameSize);
        frameContext.output.rawYuyvData = std::make_shared<std::vector<std::byte>>(rawYuyvSize);
        std::memcpy(frameContext.output.rawYuyvData->data(), inputFrame, rawYuyvSize);

        std::memcpy(reinterpret_cast<void*>(frameContext.stResource.srcFrame.u64VirAddr[0]),
                    inputFrame, frameContext.stResource.srcFrame.u32FrameSize);

        ret = sample_csc_yuyv_nv12(&frameContext.stResource.srcFrame,
                                   &frameContext.stResource.cscFrame);
        if (ret) {
            ALOGE("sample_csc_yuyv_nv12 fail, ret %d!", ret);
            releaseFrameContextInternal(frameContext);
            context.reset();
            return ret;
        }
    }

    return preprocessPreparedNv12(context);
}

int StereoDepthPipeline::preprocessFrame(FrameContextPtr& context,
                                         const AX_VIDEO_FRAME_T* inputFrame,
                                         uint64_t frameTimestampNs,
                                         InputFrameFormat inputFrameFormat,
                                         std::shared_ptr<void> inputFrameOwner) {
    if (inputFrame == nullptr || inputFrameFormat != InputFrameFormat::Nv12) {
        return -1;
    }

    context = std::make_shared<FrameContext>();
    FrameContext& frameContext = *context;
    frameContext.output.frameTimestampNs = frameTimestampNs;
    frameContext.output.rawWidth = static_cast<uint32_t>(m_inputWidth);
    frameContext.output.rawHeight = static_cast<uint32_t>(m_inputHeight);

    AX_S32 ret = sample_create_processing_image_frame(&frameContext.stResource,
                                                      static_cast<AX_U32>(m_inputWidth),
                                                      static_cast<AX_U32>(m_inputHeight));
    if (ret) {
        ALOGE("sample_create_image_frame fail, ret %d!", ret);
        context.reset();
        return ret;
    }
    frameContext.frameCreated = true;

    const AX_VIDEO_FRAME_T& srcFrame = *inputFrame;
    if (srcFrame.enImgFormat != AX_FORMAT_YUV420_SEMIPLANAR ||
        srcFrame.u32Width != static_cast<AX_U32>(m_inputWidth) ||
        srcFrame.u32Height != static_cast<AX_U32>(m_inputHeight) || srcFrame.u64PhyAddr[0] == 0 ||
        srcFrame.u64PhyAddr[1] == 0) {
        ALOGE("invalid external NV12 frame for preprocess");
        releaseFrameContextInternal(frameContext);
        context.reset();
        return -1;
    }

    frameContext.ownedCscFrame = frameContext.stResource.cscFrame;
    frameContext.stResource.cscFrame = srcFrame;
    frameContext.borrowedCscFrame = true;
    frameContext.inputFrameOwner = std::move(inputFrameOwner);

    return preprocessPreparedNv12(context);
}

int StereoDepthPipeline::inferFrame(const FrameContextPtr& context) {
    if (!context) {
        return -1;
    }

    const AX_VIDEO_FRAME_T* leftFrame = context->gdcApplied ? &context->stResource.dewarpFrame[0]
                                                            : &context->stResource.resizeFrame[0];
    const AX_VIDEO_FRAME_T* rightFrame = context->gdcApplied ? &context->stResource.dewarpFrame[1]
                                                             : &context->stResource.resizeFrame[1];
    AX_S32 ret = sample_run_axmodel(leftFrame, rightFrame, &context->stOutput);
    if (ret) {
        ALOGE("sample_run_axmodel fail, ret %d!", ret);
        return ret;
    }
    return 0;
}

int StereoDepthPipeline::postprocessFrame(FrameContextPtr& context, PipelineOutput& output,
                                          PostprocessStats* stats) {
    if (!context) {
        return -1;
    }

    output.frameTimestampNs = context->output.frameTimestampNs;
    output.rawWidth = context->output.rawWidth;
    output.rawHeight = context->output.rawHeight;
    output.rawYuyvData = std::move(context->output.rawYuyvData);
    output.rawNv12Frame = {};
    output.leftNv12Frame = {};
    output.voFrameOwner.reset();
    output.rgbData = std::move(context->output.rgbData);

    if (m_exportVoFrames) {
        output.rawNv12Frame = context->stResource.cscFrame;
        output.leftNv12Frame = context->gdcApplied ? context->stResource.dewarpFrame[0]
                                                   : context->stResource.resizeFrame[0];
        output.voFrameOwner = std::static_pointer_cast<void>(context);
    }

    uint64_t depthUs = 0;
    uint64_t pointCloudUs = 0;

    const AX_U8* bgrBuf = reinterpret_cast<AX_U8*>(context->stResource.bgrFrame.u64VirAddr[0]);
    const AX_F32* disparityFloatBuf = nullptr;
    std::vector<AX_F32> adjustedNpuDisparity;
    AX_U32 disparityWidth = kDewarpImageWidth;
    AX_U32 disparityHeight = kDewarpImageHeight;

    {
        copyAdjustedNpuDisparityImage(static_cast<const AX_F32*>(context->stOutput.virAddr),
                                      static_cast<AX_S32>(disparityWidth),
                                      static_cast<AX_S32>(disparityHeight), NPU_DISPARITY_OFFSET,
                                      adjustedNpuDisparity);
        disparityFloatBuf = adjustedNpuDisparity.data();
        const auto depthBegin = std::chrono::steady_clock::now();
        disparityFloatToU16(disparityFloatBuf, static_cast<AX_S32>(disparityWidth),
                            static_cast<AX_S32>(disparityHeight), 128.0f, output.depthData);
        const size_t expectedFloatBytes = static_cast<size_t>(disparityWidth) *
                                          static_cast<size_t>(disparityHeight) * sizeof(AX_F32);
        if (context->stOutput.confidenceVirAddr != nullptr &&
            context->stOutput.confidenceSize >= expectedFloatBytes) {
            copyFloatOutputImage(static_cast<const AX_F32*>(context->stOutput.confidenceVirAddr),
                                 static_cast<AX_S32>(disparityWidth),
                                 static_cast<AX_S32>(disparityHeight), output.confidenceData);
        } else {
            output.confidenceData.clear();
        }
        const auto depthEnd = std::chrono::steady_clock::now();
        depthUs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(depthEnd - depthBegin).count());
    }

    if (m_pointCloudScaleWidth != static_cast<int>(disparityWidth) ||
        m_pointCloudScaleHeight != static_cast<int>(disparityHeight) ||
        std::fabs(m_pointCloudScaleBaselineMeters - m_cameraBaselineMeters) > 1e-9f) {
        m_pointCloudScaleWidth = static_cast<int>(disparityWidth);
        m_pointCloudScaleHeight = static_cast<int>(disparityHeight);
        m_pointCloudScaleBaselineMeters = m_cameraBaselineMeters;
        m_pointCloudXScale.resize(static_cast<size_t>(disparityWidth));
        m_pointCloudYScale.resize(static_cast<size_t>(disparityHeight));
        for (AX_U32 u = 0; u < disparityWidth; ++u) {
            m_pointCloudXScale[static_cast<size_t>(u)] =
                (static_cast<AX_F32>(u) - CAM_CX) * m_cameraBaselineMeters;
        }
        for (AX_U32 v = 0; v < disparityHeight; ++v) {
            m_pointCloudYScale[static_cast<size_t>(v)] =
                (static_cast<AX_F32>(v) - CAM_CY) * m_cameraBaselineMeters * (CAM_FX / CAM_FY);
        }
    }

    disparityToGridAveragedZFloatImage(disparityFloatBuf, disparityWidth, disparityHeight, CAM_FX,
                                       m_cameraBaselineMeters, Z_GRID_CELL_WIDTH,
                                       Z_GRID_CELL_HEIGHT, output.zGridAvgData);

    // Point cloud generation is CPU-heavy; skip it entirely when no one
    // subscribes to or records the point cloud (controlled via
    // setComputePointCloud()).
    if (m_computePointCloud.load(std::memory_order_relaxed)) {
        const auto pointCloudBegin = std::chrono::steady_clock::now();
        disparityToPointCloudArgb(disparityFloatBuf, bgrBuf, static_cast<AX_S32>(disparityWidth),
                                  static_cast<AX_S32>(disparityHeight), CAM_FX,
                                  m_cameraBaselineMeters, m_pointCloudXScale, m_pointCloudYScale,
                                  output.pointCloudData);
        const auto pointCloudEnd = std::chrono::steady_clock::now();
        pointCloudUs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(pointCloudEnd - pointCloudBegin)
                .count());
    } else {
        output.pointCloudData.clear();
        pointCloudUs = 0;
    }

    if (stats != nullptr) {
        stats->depthUs = depthUs;
        stats->pointCloudUs = pointCloudUs;
    }

    return 0;
}

void StereoDepthPipeline::releaseFrameContext(FrameContextPtr& context) {
    if (!context) {
        return;
    }
    context.reset();
}

void StereoDepthPipeline::shutdown() {
    resetHostGdcMaps();
    host_backend::axclDewarpShutdown();

    if (m_npuInited) {
        sample_npu_deinit();
        m_npuInited = false;
    }
}

int StereoDepthPipeline::processFrame(PipelineOutput& output, const void* inputFrame,
                                      size_t inputFrameSize, uint64_t frameTimestampNs,
                                      InputFrameFormat inputFrameFormat) {
    FrameContextPtr context;
    int ret =
        preprocessFrame(context, inputFrame, inputFrameSize, frameTimestampNs, inputFrameFormat);
    if (ret != 0) {
        return ret;
    }

    ret = inferFrame(context);
    if (ret != 0) {
        releaseFrameContext(context);
        return ret;
    }

    ret = postprocessFrame(context, output);
    releaseFrameContext(context);
    return ret;
}

}  // namespace stereo_depth

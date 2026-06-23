#include "AxclImageProc.hpp"

#include <axcl.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "AxclRuntime.hpp"

#define SAMPLE_LOG_TAG "IMGPROC"
#include "sample_log.h"

namespace stereo_depth::host_backend {

namespace {

std::mutex g_mutex;
bool g_inited = false;
bool g_sysInited = false;
bool g_ivpsInited = false;

// Reusable pinned-host staging buffers (DMA capable).
struct HostPin {
    void* ptr = nullptr;
    size_t cap = 0;
};
// Reusable device CMM buffers (IVPS operates on the physical address).
struct Cmm {
    AX_U64 phy = 0;
    void* vir = nullptr;
    size_t cap = 0;
};

HostPin g_hin;
HostPin g_hout;
Cmm g_cin;
Cmm g_cout;

bool ensureHostPin(HostPin& h, size_t need) {
    if (h.cap >= need && h.ptr != nullptr) {
        return true;
    }
    if (h.ptr != nullptr) {
        axclrtFreeHost(h.ptr);
        h.ptr = nullptr;
        h.cap = 0;
    }
    if (axclrtMallocHost(&h.ptr, need) != 0 || h.ptr == nullptr) {
        h.ptr = nullptr;
        h.cap = 0;
        return false;
    }
    h.cap = need;
    return true;
}

bool ensureCmm(Cmm& c, size_t need, const char* token) {
    if (c.cap >= need && c.vir != nullptr) {
        return true;
    }
    if (c.vir != nullptr) {
        AXCL_SYS_MemFree(c.phy, c.vir);
        c.vir = nullptr;
        c.phy = 0;
        c.cap = 0;
    }
    if (AXCL_SYS_MemAlloc(&c.phy, &c.vir, static_cast<AX_U32>(need), 256,
                          reinterpret_cast<const AX_S8*>(token)) != 0 ||
        c.vir == nullptr) {
        c.vir = nullptr;
        c.phy = 0;
        c.cap = 0;
        return false;
    }
    c.cap = need;
    return true;
}

// Pack a (possibly strided) NV12 frame into a contiguous (stride == width)
// buffer. Returns the packed byte count.
size_t packNv12(const AX_VIDEO_FRAME_T* f, uint8_t* dst) {
    const int w = static_cast<int>(f->u32Width);
    const int h = static_cast<int>(f->u32Height);
    const int stride = static_cast<int>(f->u32PicStride[0]);
    const auto* yb = reinterpret_cast<const uint8_t*>(f->u64VirAddr[0]);
    const auto* uvb = reinterpret_cast<const uint8_t*>(f->u64VirAddr[1]);
    for (int r = 0; r < h; ++r) {
        std::memcpy(dst + static_cast<size_t>(r) * w, yb + static_cast<size_t>(r) * stride, w);
    }
    uint8_t* uvDst = dst + static_cast<size_t>(w) * h;
    for (int r = 0; r < h / 2; ++r) {
        std::memcpy(uvDst + static_cast<size_t>(r) * w, uvb + static_cast<size_t>(r) * stride, w);
    }
    return static_cast<size_t>(w) * h * 3 / 2;
}

// Unpack a contiguous NV12 buffer into a (possibly strided) destination frame.
void unpackNv12(const uint8_t* src, AX_VIDEO_FRAME_T* f) {
    const int w = static_cast<int>(f->u32Width);
    const int h = static_cast<int>(f->u32Height);
    const int stride = static_cast<int>(f->u32PicStride[0]);
    auto* yb = reinterpret_cast<uint8_t*>(f->u64VirAddr[0]);
    auto* uvb = reinterpret_cast<uint8_t*>(f->u64VirAddr[1]);
    for (int r = 0; r < h; ++r) {
        std::memcpy(yb + static_cast<size_t>(r) * stride, src + static_cast<size_t>(r) * w, w);
    }
    const uint8_t* uvSrc = src + static_cast<size_t>(w) * h;
    for (int r = 0; r < h / 2; ++r) {
        std::memcpy(uvb + static_cast<size_t>(r) * stride, uvSrc + static_cast<size_t>(r) * w, w);
    }
}

void unpackBgr(const uint8_t* src, AX_VIDEO_FRAME_T* f) {
    const int w = static_cast<int>(f->u32Width);
    const int h = static_cast<int>(f->u32Height);
    const int stride = static_cast<int>(f->u32PicStride[0]);
    auto* db = reinterpret_cast<uint8_t*>(f->u64VirAddr[0]);
    for (int r = 0; r < h; ++r) {
        std::memcpy(db + static_cast<size_t>(r) * stride, src + static_cast<size_t>(r) * w * 3,
                    static_cast<size_t>(w) * 3);
    }
}

AX_VIDEO_FRAME_T makeContigNv12(int w, int h, AX_U64 phy) {
    AX_VIDEO_FRAME_T f = {};
    f.u32Width = static_cast<AX_U32>(w);
    f.u32Height = static_cast<AX_U32>(h);
    f.u32PicStride[0] = static_cast<AX_U32>(w);
    f.u32FrameSize = static_cast<AX_U32>(w * h * 3 / 2);
    f.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    f.u64PhyAddr[0] = phy;
    f.u64PhyAddr[1] = phy + static_cast<AX_U64>(w) * h;
    return f;
}

}  // namespace

bool axclImageProcInit() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_inited) {
        return true;
    }
    if (!AxclRuntime::instance().acquire()) {
        ALOGW("AXCL image processing unavailable: runtime acquire failed");
        return false;
    }
    AxclRuntime::instance().ensureThreadContext();

    if (!g_sysInited) {
        const int ret = AXCL_SYS_Init();
        if (ret != 0) {
            ALOGW("AXCL_SYS_Init failed (0x%x); image processing falls back to host CPU", ret);
            AxclRuntime::instance().release();
            return false;
        }
        g_sysInited = true;
    }
    if (!g_ivpsInited) {
        const int ret = AXCL_IVPS_Init();
        if (ret != 0) {
            ALOGW("AXCL_IVPS_Init failed (0x%x); image processing falls back to host CPU", ret);
            AXCL_SYS_Deinit();
            g_sysInited = false;
            AxclRuntime::instance().release();
            return false;
        }
        g_ivpsInited = true;
    }
    g_inited = true;
    ALOGN(
        "AXCL IVPS image processing initialized (GDC dewarp + crop/resize + NV12->BGR on the "
        "card; YUYV->NV12 stays on host CPU)");
    return true;
}

void axclImageProcShutdown() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_inited) {
        return;
    }
    if (g_hin.ptr) {
        axclrtFreeHost(g_hin.ptr);
        g_hin = {};
    }
    if (g_hout.ptr) {
        axclrtFreeHost(g_hout.ptr);
        g_hout = {};
    }
    if (g_cin.vir) {
        AXCL_SYS_MemFree(g_cin.phy, g_cin.vir);
        g_cin = {};
    }
    if (g_cout.vir) {
        AXCL_SYS_MemFree(g_cout.phy, g_cout.vir);
        g_cout = {};
    }
    if (g_ivpsInited) {
        AXCL_IVPS_Deinit();
        g_ivpsInited = false;
    }
    if (g_sysInited) {
        AXCL_SYS_Deinit();
        g_sysInited = false;
    }
    AxclRuntime::instance().release();
    g_inited = false;
}

bool axclCropResizeNv12(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) {
    if (src == nullptr || dst == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_inited) {
        return false;
    }
    AxclRuntime::instance().ensureThreadContext();

    const int sw = static_cast<int>(src->u32Width);
    const int sh = static_cast<int>(src->u32Height);
    const int dw = static_cast<int>(dst->u32Width);
    const int dh = static_cast<int>(dst->u32Height);
    const size_t srcBytes = static_cast<size_t>(sw) * sh * 3 / 2;
    const size_t dstBytes = static_cast<size_t>(dw) * dh * 3 / 2;

    if (!ensureHostPin(g_hin, srcBytes) || !ensureHostPin(g_hout, dstBytes) ||
        !ensureCmm(g_cin, srcBytes, "ivps_in") || !ensureCmm(g_cout, dstBytes, "ivps_out")) {
        return false;
    }

    packNv12(src, static_cast<uint8_t*>(g_hin.ptr));
    if (axclrtMemcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(g_cin.phy)), g_hin.ptr,
                     srcBytes, AXCL_MEMCPY_HOST_TO_DEVICE) != 0) {
        return false;
    }

    AX_VIDEO_FRAME_T sf = makeContigNv12(sw, sh, g_cin.phy);
    AX_VIDEO_FRAME_T df = makeContigNv12(dw, dh, g_cout.phy);
    AX_IVPS_ASPECT_RATIO_T ar = {};
    ar.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    ar.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    ar.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    ar.nBgColor = 0x000000;

    if (AXCL_IVPS_CropResizeTdp(&sf, &df, &ar) != 0) {
        return false;
    }
    if (axclrtMemcpy(g_hout.ptr, reinterpret_cast<void*>(static_cast<uintptr_t>(g_cout.phy)),
                     dstBytes, AXCL_MEMCPY_DEVICE_TO_HOST) != 0) {
        return false;
    }
    unpackNv12(static_cast<const uint8_t*>(g_hout.ptr), dst);
    return true;
}

bool axclCscNv12ToBgr(const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) {
    if (src == nullptr || dst == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_inited) {
        return false;
    }
    AxclRuntime::instance().ensureThreadContext();

    const int w = static_cast<int>(src->u32Width);
    const int h = static_cast<int>(src->u32Height);
    const size_t srcBytes = static_cast<size_t>(w) * h * 3 / 2;
    const size_t dstBytes = static_cast<size_t>(w) * h * 3;  // BGR888

    if (!ensureHostPin(g_hin, srcBytes) || !ensureHostPin(g_hout, dstBytes) ||
        !ensureCmm(g_cin, srcBytes, "ivps_in") || !ensureCmm(g_cout, dstBytes, "ivps_out")) {
        return false;
    }

    packNv12(src, static_cast<uint8_t*>(g_hin.ptr));
    if (axclrtMemcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(g_cin.phy)), g_hin.ptr,
                     srcBytes, AXCL_MEMCPY_HOST_TO_DEVICE) != 0) {
        return false;
    }

    AX_VIDEO_FRAME_T sf = makeContigNv12(w, h, g_cin.phy);
    AX_VIDEO_FRAME_T df = {};
    df.u32Width = static_cast<AX_U32>(w);
    df.u32Height = static_cast<AX_U32>(h);
    df.u32PicStride[0] = static_cast<AX_U32>(w * 3);
    df.u32FrameSize = static_cast<AX_U32>(w * h * 3);
    df.enImgFormat = AX_FORMAT_BGR888;
    df.u64PhyAddr[0] = g_cout.phy;

    if (AXCL_IVPS_CscTdp(&sf, &df) != 0) {
        return false;
    }
    if (axclrtMemcpy(g_hout.ptr, reinterpret_cast<void*>(static_cast<uintptr_t>(g_cout.phy)),
                     dstBytes, AXCL_MEMCPY_DEVICE_TO_HOST) != 0) {
        return false;
    }
    unpackBgr(static_cast<const uint8_t*>(g_hout.ptr), dst);
    return true;
}

// GDC dewarp via AXCL_IVPS_Dewarp (per-eye user mesh).
namespace {

#define IMGPROC_ALIGN_UP(x, a) ((((x) + (a) - 1) / (a)) * (a))
#define IMGPROC_RUP_DIV(x, a) (((x) + (a) - 1) / (a))

struct DewarpEye {
    Cmm mesh;
    AX_IVPS_DEWARP_ATTR_T attr = {};
    bool ready = false;
};

bool g_dewarpInited = false;
DewarpEye g_dewarp[2];
HostPin g_dwHin;
HostPin g_dwHout;
Cmm g_dwCin;
Cmm g_dwCout;

// Load a GDC mesh .txt (16-hex-char lines, each one AX_U64) into device memory.
bool loadMeshToCmm(const std::string& path, Cmm& mesh) {
    FILE* fp = std::fopen(path.c_str(), "r");
    if (fp == nullptr) {
        ALOGW("dewarp: cannot open mesh %s", path.c_str());
        return false;
    }
    std::vector<AX_U64> table;
    char line[17];
    while (std::fgets(line, sizeof(line), fp) != nullptr) {
        if (std::strcmp(line, "\n") == 0) {
            continue;
        }
        unsigned long long value = 0;
        if (std::sscanf(line, "%llx", &value) == 1) {
            table.push_back(static_cast<AX_U64>(value));
        }
    }
    std::fclose(fp);
    if (table.empty()) {
        ALOGW("dewarp: empty mesh %s", path.c_str());
        return false;
    }

    const size_t bytes = table.size() * sizeof(AX_U64);
    if (!ensureCmm(mesh, bytes, "dewarp_mesh")) {
        return false;
    }
    return axclrtMemcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(mesh.phy)), table.data(),
                        bytes, AXCL_MEMCPY_HOST_TO_DEVICE) == 0;
}

}  // namespace

bool axclDewarpInit(const std::string& leftMeshPath, const std::string& rightMeshPath, int srcWidth,
                    int srcHeight, int dstWidth, int dstHeight) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_inited) {
        return false;
    }
    AxclRuntime::instance().ensureThreadContext();

    const std::string paths[2] = {leftMeshPath, rightMeshPath};
    const AX_U16 dstStride = static_cast<AX_U16>(IMGPROC_ALIGN_UP(dstWidth, 128));
    const AX_U16 meshW = static_cast<AX_U16>(IMGPROC_ALIGN_UP(IMGPROC_RUP_DIV(dstWidth, 32), 16));
    const AX_U16 meshH = static_cast<AX_U16>(IMGPROC_ALIGN_UP(IMGPROC_RUP_DIV(dstHeight, 32), 16));

    for (int eye = 0; eye < 2; ++eye) {
        g_dewarp[eye].ready = false;
        if (!loadMeshToCmm(paths[eye], g_dewarp[eye].mesh)) {
            ALOGW("dewarp: mesh load failed for eye %d (%s); AXCL dewarp disabled", eye,
                  paths[eye].c_str());
            return false;
        }
        AX_IVPS_DEWARP_ATTR_T& at = g_dewarp[eye].attr;
        std::memset(&at, 0, sizeof(at));
        at.bCrop = AX_FALSE;
        at.nDstWidth = static_cast<AX_U16>(dstWidth);
        at.nDstHeight = static_cast<AX_U16>(dstHeight);
        at.nDstStride = dstStride;
        at.eImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        at.eDewarpType = AX_IVPS_DEWARP_MAP_USER;
        at.tMapUserAttr.nMeshStartX = 0;
        at.tMapUserAttr.nMeshStartY = 0;
        at.tMapUserAttr.nMeshWidth = meshW;
        at.tMapUserAttr.nMeshHeight = meshH;
        at.tMapUserAttr.nMeshNumH = 33;
        at.tMapUserAttr.nMeshNumV = 33;
        at.tMapUserAttr.pUserMap = reinterpret_cast<AX_S32*>(g_dewarp[eye].mesh.vir);
        at.tMapUserAttr.nMeshTablePhyAddr = g_dewarp[eye].mesh.phy;
        g_dewarp[eye].ready = true;
    }

    g_dewarpInited = true;
    ALOGN("AXCL GDC dewarp ready (AXCL_IVPS_Dewarp, %dx%d -> %dx%d per eye)", srcWidth, srcHeight,
          dstWidth, dstHeight);
    return true;
}

bool axclDewarpReady() { return g_dewarpInited && g_dewarp[0].ready && g_dewarp[1].ready; }

bool axclDewarpEye(int eye, const AX_VIDEO_FRAME_T* src, AX_VIDEO_FRAME_T* dst) {
    if (eye < 0 || eye > 1 || src == nullptr || dst == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_dewarpInited || !g_dewarp[eye].ready) {
        return false;
    }
    AxclRuntime::instance().ensureThreadContext();

    const int sw = static_cast<int>(src->u32Width);
    const int sh = static_cast<int>(src->u32Height);
    const int dw = static_cast<int>(dst->u32Width);
    const int dh = static_cast<int>(dst->u32Height);
    const int dstStride = IMGPROC_ALIGN_UP(dw, 128);
    const size_t srcBytes = static_cast<size_t>(sw) * sh * 3 / 2;
    const size_t dstBytes = static_cast<size_t>(dstStride) * dh * 3 / 2;

    if (!ensureHostPin(g_dwHin, srcBytes) || !ensureHostPin(g_dwHout, dstBytes) ||
        !ensureCmm(g_dwCin, srcBytes, "dewarp_in") ||
        !ensureCmm(g_dwCout, dstBytes, "dewarp_out")) {
        return false;
    }

    packNv12(src, static_cast<uint8_t*>(g_dwHin.ptr));
    if (axclrtMemcpy(reinterpret_cast<void*>(static_cast<uintptr_t>(g_dwCin.phy)), g_dwHin.ptr,
                     srcBytes, AXCL_MEMCPY_HOST_TO_DEVICE) != 0) {
        return false;
    }

    AX_VIDEO_FRAME_T sf = makeContigNv12(sw, sh, g_dwCin.phy);
    AX_VIDEO_FRAME_T df = {};
    df.u32Width = static_cast<AX_U32>(dw);
    df.u32Height = static_cast<AX_U32>(dh);
    df.u32PicStride[0] = static_cast<AX_U32>(dstStride);
    df.u32FrameSize = static_cast<AX_U32>(dstBytes);
    df.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    df.u64PhyAddr[0] = g_dwCout.phy;
    df.u64PhyAddr[1] = g_dwCout.phy + static_cast<AX_U64>(dstStride) * dh;
    df.u64VirAddr[0] = reinterpret_cast<AX_U64>(g_dwCout.vir);
    df.u64VirAddr[1] = reinterpret_cast<AX_U64>(g_dwCout.vir) + static_cast<AX_U64>(dstStride) * dh;

    if (AXCL_IVPS_Dewarp(&sf, &df, &g_dewarp[eye].attr) != 0) {
        return false;
    }
    if (axclrtMemcpy(g_dwHout.ptr, reinterpret_cast<void*>(static_cast<uintptr_t>(g_dwCout.phy)),
                     dstBytes, AXCL_MEMCPY_DEVICE_TO_HOST) != 0) {
        return false;
    }

    // Unpack the contiguous (dstStride) NV12 result into the host dst frame.
    const auto* srcBuf = static_cast<const uint8_t*>(g_dwHout.ptr);
    auto* yb = reinterpret_cast<uint8_t*>(dst->u64VirAddr[0]);
    auto* uvb = reinterpret_cast<uint8_t*>(dst->u64VirAddr[1]);
    const int dStride = static_cast<int>(dst->u32PicStride[0]);
    for (int r = 0; r < dh; ++r) {
        std::memcpy(yb + static_cast<size_t>(r) * dStride,
                    srcBuf + static_cast<size_t>(r) * dstStride, dw);
    }
    const uint8_t* uvSrc = srcBuf + static_cast<size_t>(dstStride) * dh;
    for (int r = 0; r < dh / 2; ++r) {
        std::memcpy(uvb + static_cast<size_t>(r) * dStride,
                    uvSrc + static_cast<size_t>(r) * dstStride, dw);
    }
    return true;
}

void axclDewarpShutdown() {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto& eye : g_dewarp) {
        if (eye.mesh.vir) {
            AXCL_SYS_MemFree(eye.mesh.phy, eye.mesh.vir);
            eye.mesh = {};
        }
        eye.ready = false;
    }
    if (g_dwHin.ptr) {
        axclrtFreeHost(g_dwHin.ptr);
        g_dwHin = {};
    }
    if (g_dwHout.ptr) {
        axclrtFreeHost(g_dwHout.ptr);
        g_dwHout = {};
    }
    if (g_dwCin.vir) {
        AXCL_SYS_MemFree(g_dwCin.phy, g_dwCin.vir);
        g_dwCin = {};
    }
    if (g_dwCout.vir) {
        AXCL_SYS_MemFree(g_dwCout.phy, g_dwCout.vir);
        g_dwCout = {};
    }
    g_dewarpInited = false;
}

}  // namespace stereo_depth::host_backend

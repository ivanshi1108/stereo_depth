// Host implementation of the stereo image helpers (CPU/OpenCV). Frame buffers
// are plain host allocations; u64VirAddr holds the host pointer and u64PhyAddr
// mirrors it so the existing non-zero checks pass.

#include "sample_image.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sys/stat.h>

#include "AxclImageProc.hpp"
#include "ImageProcBackend.hpp"

#define SAMPLE_LOG_TAG "IMAGE"
#include "sample_log.h"

#define RESIZE_IMAGE_WIDTH (640)
#define RESIZE_IMAGE_HEIGHT (384)
#define DEWARP_IMAGE_WIDTH (RESIZE_IMAGE_WIDTH)
#define DEWARP_IMAGE_HEIGHT (RESIZE_IMAGE_HEIGHT)

#define AX_ALIGN_UP(x, a) ((((x) + (a) - 1) / (a)) * (a))

#define CHECK_POINTER(p)          \
    if (!(p)) {                   \
        ALOGE("Null pointer!\n"); \
        return AX_ERR_NULL_PTR;   \
    }

namespace {

void freeFramePlane(AX_VIDEO_FRAME_T* pFrame) {
    if (pFrame->u64VirAddr[0]) {
        std::free(reinterpret_cast<void*>(pFrame->u64VirAddr[0]));
    }
    for (int i = 0; i < 3; ++i) {
        pFrame->u64PhyAddr[i] = 0;
        pFrame->u64VirAddr[i] = 0;
    }
}

// Allocate a single host buffer for the frame and publish virt/phys pointers.
// phys is set equal to virt purely so callers' "addr != 0" guards hold.
AX_S32 allocFrame(AX_VIDEO_FRAME_T* pFrame, AX_U32 size) {
    void* p = std::malloc(size);
    if (p == nullptr) {
        ALOGE("host malloc fail, size %u\n", size);
        return -1;
    }
    std::memset(p, 0, size);
    pFrame->u64VirAddr[0] = reinterpret_cast<AX_ULONG>(p);
    pFrame->u64PhyAddr[0] = reinterpret_cast<AX_ULONG>(p);
    return 0;
}

void setNv12Planes(AX_VIDEO_FRAME_T* pFrame) {
    const AX_ULONG base = pFrame->u64VirAddr[0];
    const AX_U32 stride = pFrame->u32PicStride[0];
    const AX_ULONG off = static_cast<AX_ULONG>(stride) * pFrame->u32Height;
    pFrame->u64VirAddr[1] = base + off;
    pFrame->u64PhyAddr[1] = pFrame->u64PhyAddr[0] + off;
}

}  // namespace

AX_S32 Get_FileSize(const AX_CHAR* filename) {
    CHECK_POINTER(filename);
    struct stat statbuf;
    if (stat(filename, &statbuf) != 0) {
        return -1;
    }
    return static_cast<AX_S32>(statbuf.st_size);
}

AX_S32 sample_load_bin(const AX_CHAR* pszBinFileName, AX_VOID* vaddr) {
    CHECK_POINTER(pszBinFileName);
    CHECK_POINTER(vaddr);
    AX_S32 fSz = Get_FileSize(pszBinFileName);
    FILE* fp = fopen(pszBinFileName, "rb");
    if (!fp) {
        ALOGE("sample_load_bin:open %s fail!\n", pszBinFileName);
        return -1;
    }
    size_t ret = fread(vaddr, fSz, 1, fp);
    fclose(fp);
    return (ret == 1) ? 0 : -1;
}

AX_S32 sample_save_bin(const char* pszBinFileName, AX_VOID* vaddr, AX_U32 size) {
    CHECK_POINTER(pszBinFileName);
    CHECK_POINTER(vaddr);
    FILE* fp = fopen(pszBinFileName, "wb+");
    if (!fp) {
        ALOGE("sample_save_bin:open %s fail!\n", pszBinFileName);
        return -1;
    }
    size_t ret = fwrite(vaddr, size, 1, fp);
    fclose(fp);
    return (ret == 1) ? 0 : -1;
}

AX_S32 sample_release_image_frame(SAMPLE_RESOURCE_T* pResource) {
    CHECK_POINTER(pResource);
    freeFramePlane(&pResource->srcFrame);
    freeFramePlane(&pResource->cscFrame);
    for (int i = 0; i < SAMPLE_PIPE_NUM; i++) {
        freeFramePlane(&pResource->resizeFrame[i]);
    }
    for (int i = 0; i < SAMPLE_PIPE_NUM; i++) {
        freeFramePlane(&pResource->dewarpFrame[i]);
    }
    freeFramePlane(&pResource->bgrFrame);
    return 0;
}

AX_S32 sample_create_image_frame(SAMPLE_RESOURCE_T* pResource, AX_U32 inputWidth,
                                 AX_U32 inputHeight) {
    CHECK_POINTER(pResource);
    if (inputWidth == 0 || inputHeight == 0 || (inputWidth % 2) != 0) {
        ALOGE("invalid input geometry %ux%u\n", inputWidth, inputHeight);
        return -1;
    }

    AX_S32 ret = 0;

    AX_VIDEO_FRAME_T* pFrame = &pResource->srcFrame;
    pFrame->u32Width = inputWidth;
    pFrame->u32Height = inputHeight;
    pFrame->u32PicStride[0] = pFrame->u32Width * 2;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->enImgFormat = AX_FORMAT_YUV422_INTERLEAVED_YUVY;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;

    pFrame = &pResource->cscFrame;
    pFrame->u32Width = inputWidth;
    pFrame->u32Height = inputHeight;
    pFrame->u32PicStride[0] = pFrame->u32Width;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
    pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;
    setNv12Planes(pFrame);

    for (int i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->resizeFrame[i];
        pFrame->u32Width = RESIZE_IMAGE_WIDTH;
        pFrame->u32Height = RESIZE_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;
        setNv12Planes(pFrame);
    }

    for (int i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->dewarpFrame[i];
        pFrame->u32Width = DEWARP_IMAGE_WIDTH;
        pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;
        setNv12Planes(pFrame);
    }

    pFrame = &pResource->bgrFrame;
    pFrame->u32Width = DEWARP_IMAGE_WIDTH;
    pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
    pFrame->u32PicStride[0] = pFrame->u32Width * 3;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->enImgFormat = AX_FORMAT_BGR888;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;

    return 0;

RET_ERR:
    sample_release_image_frame(pResource);
    return ret;
}

AX_S32 sample_create_processing_image_frame(SAMPLE_RESOURCE_T* pResource, AX_U32 inputWidth,
                                            AX_U32 inputHeight) {
    CHECK_POINTER(pResource);
    if (inputWidth == 0 || inputHeight == 0 || (inputWidth % 2) != 0) {
        ALOGE("invalid input geometry %ux%u\n", inputWidth, inputHeight);
        return -1;
    }

    AX_S32 ret = 0;
    AX_VIDEO_FRAME_T* pFrame = nullptr;

    for (int i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->resizeFrame[i];
        pFrame->u32Width = RESIZE_IMAGE_WIDTH;
        pFrame->u32Height = RESIZE_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;
        setNv12Planes(pFrame);
    }

    for (int i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->dewarpFrame[i];
        pFrame->u32Width = DEWARP_IMAGE_WIDTH;
        pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;
        setNv12Planes(pFrame);
    }

    pFrame = &pResource->bgrFrame;
    pFrame->u32Width = DEWARP_IMAGE_WIDTH;
    pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
    pFrame->u32PicStride[0] = pFrame->u32Width * 3;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->enImgFormat = AX_FORMAT_BGR888;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    if ((ret = allocFrame(pFrame, pFrame->u32FrameSize)) != 0) goto RET_ERR;

    return 0;

RET_ERR:
    sample_release_image_frame(pResource);
    return ret;
}

// Direct YUYV (4:2:2 packed) -> NV12 (4:2:0 semi-planar) conversion on the host
// CPU. This avoids the costly YUYV->BGR->YUV roundtrip: the luma is copied as-is
// and the chroma is vertically averaged between the two source rows of each NV12
// chroma row. (AXCL IVPS cannot accept packed YUYV and the AXCL host SDK does
// not expose the board's DSP CvtColor, so this conversion stays on the CPU; the
// direct path keeps its cost low.)
AX_S32 sample_csc_yuyv_nv12(AX_VIDEO_FRAME_T* pSrcFrame, AX_VIDEO_FRAME_T* pCscFrame) {
    CHECK_POINTER(pSrcFrame);
    CHECK_POINTER(pCscFrame);

    const int w = static_cast<int>(pSrcFrame->u32Width);
    const int h = static_cast<int>(pSrcFrame->u32Height);
    const int srcStride = static_cast<int>(pSrcFrame->u32PicStride[0]);  // w*2 for YUYV
    const auto* src = reinterpret_cast<const AX_U8*>(pSrcFrame->u64VirAddr[0]);
    auto* yPtr = reinterpret_cast<AX_U8*>(pCscFrame->u64VirAddr[0]);
    auto* uvPtr = reinterpret_cast<AX_U8*>(pCscFrame->u64VirAddr[1]);
    const int yStride = static_cast<int>(pCscFrame->u32PicStride[0]);
    const int uvStride =
        pCscFrame->u32PicStride[1] != 0 ? static_cast<int>(pCscFrame->u32PicStride[1]) : yStride;

    // Luma: Y is every even byte of each YUYV row.
    for (int y = 0; y < h; ++y) {
        const AX_U8* srcRow = src + static_cast<size_t>(y) * srcStride;
        AX_U8* yRow = yPtr + static_cast<size_t>(y) * yStride;
        for (int x = 0; x < w; ++x) {
            yRow[x] = srcRow[2 * x];
        }
    }

    // Chroma: NV12 UV is half height/width; average U and V across the two source
    // rows that map to each chroma row.
    for (int cy = 0; cy < h / 2; ++cy) {
        const AX_U8* row0 = src + static_cast<size_t>(2 * cy) * srcStride;
        const AX_U8* row1 = src + static_cast<size_t>(2 * cy + 1) * srcStride;
        AX_U8* uvRow = uvPtr + static_cast<size_t>(cy) * uvStride;
        for (int cx = 0; cx < w / 2; ++cx) {
            const int base = 4 * cx;  // YUYV: [Y0 U Y1 V] per 2 pixels
            const int u = (static_cast<int>(row0[base + 1]) + row1[base + 1] + 1) >> 1;
            const int v = (static_cast<int>(row0[base + 3]) + row1[base + 3] + 1) >> 1;
            uvRow[2 * cx + 0] = static_cast<AX_U8>(u);
            uvRow[2 * cx + 1] = static_cast<AX_U8>(v);
        }
    }
    return 0;
}

AX_S32 sample_crop_resize(const AX_VIDEO_FRAME_T* pInFrame, AX_VIDEO_FRAME_T* pOutFrame) {
    CHECK_POINTER(pInFrame);
    CHECK_POINTER(pOutFrame);

    // AXCL IVPS offload (when --imgproc axcl); falls back to host CPU on failure.
    if (stereo_depth::imageProcUsesAxcl() &&
        stereo_depth::host_backend::axclCropResizeNv12(pInFrame, pOutFrame)) {
        return 0;
    }

    const int inW = static_cast<int>(pInFrame->u32Width);
    const int inH = static_cast<int>(pInFrame->u32Height);
    const int inStride = static_cast<int>(pInFrame->u32PicStride[0]);
    const int outW = static_cast<int>(pOutFrame->u32Width);
    const int outH = static_cast<int>(pOutFrame->u32Height);
    const int outStride = static_cast<int>(pOutFrame->u32PicStride[0]);

    // NV12 input: luma plane + interleaved chroma plane (each at half size).
    cv::Mat yIn(inH, inW, CV_8UC1, reinterpret_cast<void*>(pInFrame->u64VirAddr[0]), inStride);
    cv::Mat uvIn(inH / 2, inW / 2, CV_8UC2, reinterpret_cast<void*>(pInFrame->u64VirAddr[1]),
                 inStride);

    cv::Mat yOut(outH, outW, CV_8UC1, reinterpret_cast<void*>(pOutFrame->u64VirAddr[0]), outStride);
    cv::Mat uvOut(outH / 2, outW / 2, CV_8UC2, reinterpret_cast<void*>(pOutFrame->u64VirAddr[1]),
                  outStride);

    cv::resize(yIn, yOut, yOut.size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(uvIn, uvOut, uvOut.size(), 0, 0, cv::INTER_LINEAR);
    return 0;
}

AX_S32 sample_csc_nv12_bgr(const AX_VIDEO_FRAME_T* pInFrame, AX_VIDEO_FRAME_T* pOutFrame) {
    CHECK_POINTER(pInFrame);
    CHECK_POINTER(pOutFrame);

    // AXCL IVPS offload (when --imgproc axcl); falls back to host CPU on failure.
    if (stereo_depth::imageProcUsesAxcl() &&
        stereo_depth::host_backend::axclCscNv12ToBgr(pInFrame, pOutFrame)) {
        return 0;
    }

    const int w = static_cast<int>(pInFrame->u32Width);
    const int h = static_cast<int>(pInFrame->u32Height);
    const int inStride = static_cast<int>(pInFrame->u32PicStride[0]);
    const int outStride = static_cast<int>(pOutFrame->u32PicStride[0]);

    cv::Mat bgr(h, w, CV_8UC3, reinterpret_cast<void*>(pOutFrame->u64VirAddr[0]), outStride);

    if (inStride == w) {
        // Contiguous NV12 (Y immediately followed by UV) -> single-shot convert.
        cv::Mat nv12(h * 3 / 2, w, CV_8UC1, reinterpret_cast<void*>(pInFrame->u64VirAddr[0]),
                     inStride);
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    } else {
        // Strided planes: assemble a packed NV12 scratch first.
        cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
        cv::Mat yIn(h, w, CV_8UC1, reinterpret_cast<void*>(pInFrame->u64VirAddr[0]), inStride);
        cv::Mat uvIn(h / 2, w, CV_8UC1, reinterpret_cast<void*>(pInFrame->u64VirAddr[1]), inStride);
        yIn.copyTo(nv12.rowRange(0, h));
        uvIn.copyTo(nv12.rowRange(h, h * 3 / 2));
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    }
    return 0;
}

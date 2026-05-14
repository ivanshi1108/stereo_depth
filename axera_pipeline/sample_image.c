#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "ax_dsp_api.h"
#include "ax_dsp_cv_api.h"
#include "ax_ivps_api.h"
#include "ax_sys_api.h"

#include "sample_engine.h"
#include "sample_image.h"
#include "sample_log.h"

#define DSP_ID (AX_DSP_ID_0)
#define INPUT_IMAGE_WIDTH (672 * 2)
#define INPUT_IMAGE_HEIGHT (376)
#define RESIZE_IMAGE_WIDTH (640)
#define RESIZE_IMAGE_HEIGHT (384)
#define DEWARP_IMAGE_WIDTH (RESIZE_IMAGE_WIDTH)
#define DEWARP_IMAGE_HEIGHT (RESIZE_IMAGE_HEIGHT)

#define AX_ALIGN_UP(x, a) ((((x) + (a)-1) / (a)) * (a))

#define CHECK_POINTER(p)          \
    if (!p) {                     \
        ALOGE("Null pointer!\n"); \
        return AX_ERR_NULL_PTR;   \
    }

#define CSC_DSP_ID AX_DSP_ID_1

AX_S32 Get_FileSize(const AX_CHAR* filename) {
    CHECK_POINTER(filename);
    struct stat statbuf;
    AX_S32 size;
    stat(filename, &statbuf);
    size = statbuf.st_size;
    return size;
}

AX_S32 sample_load_bin(const AX_CHAR* pszBinFileName, AX_VOID* vaddr) {
    FILE* fp;
    AX_S32 fSz;
    AX_S32 ret;
    CHECK_POINTER(pszBinFileName);
    CHECK_POINTER(vaddr);

    fSz = Get_FileSize(pszBinFileName);
    fp = fopen(pszBinFileName, "rb");
    if (!fp) {
        printf("sample_load_bin:open %s fail!\n", pszBinFileName);
        return AX_DSP_OPEN_FAIL;
    }
    ret = fread((void*)vaddr, fSz, 1, fp);
    if (ret != 1) {
        printf("sample_load_bin:ret = %x, fSz =  %x\n", ret, fSz);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

AX_S32 sample_save_bin(const char* pszBinFileName, AX_VOID* vaddr, AX_U32 size) {
    FILE* fp;
    AX_S32 ret;
    CHECK_POINTER(pszBinFileName);
    CHECK_POINTER(vaddr);

    fp = fopen(pszBinFileName, "wb+");
    if (!fp) {
        printf("sample_save_bin:open %s fail!\n", pszBinFileName);
        return AX_DSP_OPEN_FAIL;
    }
    ret = fwrite((void*)vaddr, size, 1, fp);
    if (ret != 1) {
        printf("sample_save_bin:ret = %x, fSz =  %x\n", ret, size);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

AX_S32 sample_release_image_frame(SAMPLE_RESOURCE_T* pResource) {
    AX_S32 i = 0;
    CHECK_POINTER(pResource);

    AX_VIDEO_FRAME_T* pFrame = &pResource->srcFrame;
    if (pFrame->u64PhyAddr[0] && pFrame->u64VirAddr[0]) {
        AX_SYS_MemFree(pFrame->u64PhyAddr[0], (AX_VOID*)pFrame->u64VirAddr[0]);
        pFrame->u64PhyAddr[0] = 0;
        pFrame->u64VirAddr[0] = 0;
    }

    pFrame = &pResource->cscFrame;
    if (pFrame->u64PhyAddr[0] && pFrame->u64VirAddr[0]) {
        AX_SYS_MemFree(pFrame->u64PhyAddr[0], (AX_VOID*)pFrame->u64VirAddr[0]);
        pFrame->u64PhyAddr[0] = 0;
        pFrame->u64VirAddr[0] = 0;
    }

    for (i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->resizeFrame[i];
        if (pFrame->u64PhyAddr[0] && pFrame->u64VirAddr[0]) {
            AX_SYS_MemFree(pFrame->u64PhyAddr[0], (AX_VOID*)pFrame->u64VirAddr[0]);
            pFrame->u64PhyAddr[0] = 0;
            pFrame->u64VirAddr[0] = 0;
        }
    }

    for (i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->dewarpFrame[i];
        if (pFrame->u64PhyAddr[0] && pFrame->u64VirAddr[0]) {
            AX_SYS_MemFree(pFrame->u64PhyAddr[0], (AX_VOID*)pFrame->u64VirAddr[0]);
            pFrame->u64PhyAddr[0] = 0;
            pFrame->u64VirAddr[0] = 0;
        }
    }

    pFrame = &pResource->bgrFrame;
    if (pFrame->u64PhyAddr[0] && pFrame->u64VirAddr[0]) {
        AX_SYS_MemFree(pFrame->u64PhyAddr[0], (AX_VOID*)pFrame->u64VirAddr[0]);
        pFrame->u64PhyAddr[0] = 0;
        pFrame->u64VirAddr[0] = 0;
    }

    return 0;
}

AX_S32 sample_create_image_frame(SAMPLE_RESOURCE_T* pResource, AX_U32 inputWidth,
                                 AX_U32 inputHeight) {
    AX_S32 ret = 0;
    AX_S32 i;
    AX_U64 phyAddr = 0;
    AX_VOID* pVirAddr = NULL;
    CHECK_POINTER(pResource);

    if (inputWidth == 0 || inputHeight == 0 || (inputWidth % 2) != 0) {
        ALOGE("invalid input geometry %ux%u\n", inputWidth, inputHeight);
        return -1;
    }

    AX_VIDEO_FRAME_T* pFrame = &pResource->srcFrame;
    pFrame->u32Width = inputWidth;
    pFrame->u32Height = inputHeight;
    pFrame->u32PicStride[0] = pFrame->u32Width * 2;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->enImgFormat = AX_FORMAT_YUV422_INTERLEAVED_YUVY;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                          (AX_S8*)"srcFrame");
    if (ret) {
        ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
        goto RET_ERR;
    }
    pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
    pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;

    pFrame = &pResource->cscFrame;
    pFrame->u32Width = inputWidth;
    pFrame->u32Height = inputHeight;
    pFrame->u32PicStride[0] = pFrame->u32Width;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
    pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                          (AX_S8*)"cscFrame");
    if (ret) {
        ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
        goto RET_ERR;
    }
    pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
    pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
    pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;

    for (i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->resizeFrame[i];
        pFrame->u32Width = RESIZE_IMAGE_WIDTH;
        pFrame->u32Height = RESIZE_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                              (AX_S8*)"resizeFrame");
        if (ret) {
            ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
            goto RET_ERR;
        }
        pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
        pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
        pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
        pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    }

    for (i = 0; i < SAMPLE_PIPE_NUM; i++) {
        pFrame = &pResource->dewarpFrame[i];
        pFrame->u32Width = DEWARP_IMAGE_WIDTH;
        pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                              (AX_S8*)"dewarpFrame");
        if (ret) {
            ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
            goto RET_ERR;
        }
        pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
        pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
        pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
        pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    }

    pFrame = &pResource->bgrFrame;
    pFrame->u32Width = DEWARP_IMAGE_WIDTH;
    pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
    pFrame->u32PicStride[0] = pFrame->u32Width * 3;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->enImgFormat = AX_FORMAT_BGR888;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                          (AX_S8*)"bgrFrame");
    if (ret) {
        ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
        goto RET_ERR;
    }
    pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
    pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->u64PhyAddr[2] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height * 2;
    pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
    pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->u64VirAddr[2] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height * 2;

    return 0;

RET_ERR:
    sample_release_image_frame(pResource);
    return ret;
}

AX_S32 sample_create_processing_image_frame(SAMPLE_RESOURCE_T* pResource, AX_U32 inputWidth,
                                            AX_U32 inputHeight) {
    AX_S32 ret = 0;
    AX_S32 i;
    AX_U64 phyAddr = 0;
    AX_VOID* pVirAddr = NULL;
    CHECK_POINTER(pResource);

    if (inputWidth == 0 || inputHeight == 0 || (inputWidth % 2) != 0) {
        ALOGE("invalid input geometry %ux%u\n", inputWidth, inputHeight);
        return -1;
    }

    for (i = 0; i < SAMPLE_PIPE_NUM; i++) {
        AX_VIDEO_FRAME_T* pFrame = &pResource->resizeFrame[i];
        pFrame->u32Width = RESIZE_IMAGE_WIDTH;
        pFrame->u32Height = RESIZE_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                              (AX_S8*)"resizeFrame");
        if (ret) {
            ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
            goto RET_ERR;
        }
        pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
        pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
        pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
        pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    }

    for (i = 0; i < SAMPLE_PIPE_NUM; i++) {
        AX_VIDEO_FRAME_T* pFrame = &pResource->dewarpFrame[i];
        pFrame->u32Width = DEWARP_IMAGE_WIDTH;
        pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
        pFrame->u32PicStride[0] = AX_ALIGN_UP(pFrame->u32Width, 128);
        pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height * 3 / 2;
        pFrame->enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
        pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
        ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                              (AX_S8*)"dewarpFrame");
        if (ret) {
            ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
            goto RET_ERR;
        }
        pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
        pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
        pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
        pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    }

    AX_VIDEO_FRAME_T* pFrame = &pResource->bgrFrame;
    pFrame->u32Width = DEWARP_IMAGE_WIDTH;
    pFrame->u32Height = DEWARP_IMAGE_HEIGHT;
    pFrame->u32PicStride[0] = pFrame->u32Width * 3;
    pFrame->u32FrameSize = pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->enImgFormat = AX_FORMAT_BGR888;
    pFrame->stCompressInfo.enCompressMode = AX_COMPRESS_MODE_NONE;
    ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr, pFrame->u32FrameSize, 1,
                          (AX_S8*)"bgrFrame");
    if (ret) {
        ALOGE("AX_SYS_MemAlloc fail, frameSize %d, ret %d!\n", pFrame->u32FrameSize, ret);
        goto RET_ERR;
    }
    pFrame->u64PhyAddr[0] = (AX_ULONG)phyAddr;
    pFrame->u64PhyAddr[1] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->u64PhyAddr[2] = (AX_ULONG)phyAddr + pFrame->u32PicStride[0] * pFrame->u32Height * 2;
    pFrame->u64VirAddr[0] = (AX_ULONG)pVirAddr;
    pFrame->u64VirAddr[1] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height;
    pFrame->u64VirAddr[2] = (AX_ULONG)pVirAddr + pFrame->u32PicStride[0] * pFrame->u32Height * 2;

    return 0;

RET_ERR:
    sample_release_image_frame(pResource);
    return ret;
}

AX_S32 sample_csc_yuyv_nv12(AX_VIDEO_FRAME_T* pSrcFrame, AX_VIDEO_FRAME_T* pCscFrame) {
    AX_S32 ret = 0;
    AX_DSP_CV_CVTCOLOR_PARAM_T param;
    AX_MEM_INFO_T in_buf[3];
    AX_MEM_INFO_T out_buf[3];
    struct timeval tv_start, tv_end;
    CHECK_POINTER(pSrcFrame);
    CHECK_POINTER(pCscFrame);

    param.src_width = pSrcFrame->u32Width;
    param.src_height = pSrcFrame->u32Height;
    param.src_stride = pSrcFrame->u32PicStride[0];
    param.dst_stride = param.src_width;

    in_buf[0].u64PhyAddr = pSrcFrame->u64PhyAddr[0];
    out_buf[0].u64PhyAddr = pCscFrame->u64PhyAddr[0];
    out_buf[1].u64PhyAddr = pCscFrame->u64PhyAddr[1];

    gettimeofday(&tv_start, NULL);
    ret = AX_DSP_CV_CvtColor(CSC_DSP_ID, AX_DSP_CV_CVTCOLOR_YUYV_NV12, in_buf, out_buf, &param);
    if (ret != 0) {
        ALOGE("%s AX_DSP_CV_CvtColor(dsp=%d) error %x\n", __func__, CSC_DSP_ID, ret);
        return -1;
    }
    gettimeofday(&tv_end, NULL);
    ALOGI("%s AX_DSP_CV_CvtColor cost time: tv_usec: %ld\n", __func__,
          (tv_end.tv_sec - tv_start.tv_sec) * 1000000 + tv_end.tv_usec - tv_start.tv_usec);

#if SAVE_FRAME
    AX_CHAR str[100];
    sprintf(str, "%s_%dx%d.nv12", "yuyv_csc", pCscFrame->u32Width, pCscFrame->u32Height);
    sample_save_bin(str, (AX_VOID*)pCscFrame->u64VirAddr[0], pCscFrame->u32FrameSize);
    printf("%s saved!\n", str);
#endif

    return 0;
}

AX_S32 sample_crop_resize(const AX_VIDEO_FRAME_T* pInFrame, AX_VIDEO_FRAME_T* pOutFrame) {
    AX_S32 ret;
    CHECK_POINTER(pInFrame);
    CHECK_POINTER(pOutFrame);

    AX_IVPS_ASPECT_RATIO_T tAspectRatio;
    memset(&tAspectRatio, 0x00, sizeof(AX_IVPS_ASPECT_RATIO_T));
    tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    tAspectRatio.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
    tAspectRatio.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
    tAspectRatio.nBgColor = 0x0000FF;

    ret = AX_IVPS_CropResizeTdp(pInFrame, pOutFrame, &tAspectRatio);
    if (ret) {
        ALOGE("AX_IVPS_CropResizeTdp failed, ret = %d\n", ret);
        return ret;
    }

#if SAVE_FRAME
    AX_CHAR str[100];
    sprintf(str, "%s_%dx%d.nv12", "resize", pOutFrame->u32Width, pOutFrame->u32Height);
    sample_save_bin(str, (AX_VOID*)pOutFrame->u64VirAddr[0], pOutFrame->u32FrameSize);
    printf("%s saved!\n", str);
#endif

    return 0;
}

AX_S32 sample_csc_nv12_bgr(const AX_VIDEO_FRAME_T* pInFrame, AX_VIDEO_FRAME_T* pOutFrame) {
    AX_S32 ret;
    CHECK_POINTER(pInFrame);
    CHECK_POINTER(pOutFrame);

    ret = AX_IVPS_CscTdp(pInFrame, pOutFrame);
    if (ret) {
        ALOGE("AX_IVPS_CscTdp failed, ret = %d\n", ret);
        return ret;
    }

#if SAVE_FRAME
    AX_CHAR str[100];
    sprintf(str, "%dx%d.bgr888", pOutFrame->u32Width, pOutFrame->u32Height);
    sample_save_bin(str, (AX_VOID*)pOutFrame->u64VirAddr[0], pOutFrame->u32FrameSize);
    printf("%s saved!\n", str);
#endif

    return 0;
}

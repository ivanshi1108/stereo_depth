#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "sample_gdc.h"

#define SAMPLE_LOG_TAG "GDC"
#include "sample_log.h"

#define AX_ROUND_UP_DIV(x, a) (((x) + (a)-1) / (a))
#define AX_ALIGN_UP(x, a) ((((x) + (a)-1) / (a)) * (a))

#define CHECK_POINTER(p)        \
    if (!(p)) {                 \
        ALOGE("Null pointer!"); \
        return AX_ERR_NULL_PTR; \
    }

AX_S32 sample_load_mesh_file_to_cmm(const AX_CHAR* pFile, AX_VOID** ppVirAddr, AX_U64* pU64PhyAddr,
                                    AX_U32* pMeshTableSize) {
    AX_U64* pData;
    CHECK_POINTER(pFile);
    CHECK_POINTER(ppVirAddr);
    CHECK_POINTER(pU64PhyAddr);
    CHECK_POINTER(pMeshTableSize);

    FILE* fp = fopen(pFile, "r");
    if (!fp) {
        ALOGE("open %s fail, %s", pFile, strerror(errno));
        return AX_FALSE;
    }

    fseek(fp, 0, SEEK_END);
    AX_U32 nFileSize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (0 == nFileSize) {
        ALOGE("nFileSize is 0");
        fclose(fp);
        return AX_FALSE;
    }

    AX_U64 phyAddr = 0;
    AX_VOID* pVirAddr = NULL;
    AX_S32 s32Ret = AX_SYS_MemAlloc((AX_U64*)&phyAddr, (AX_VOID**)&pVirAddr,
                                    sizeof(AX_U8) * nFileSize, 1, (AX_S8*)"avm");
    if (s32Ret) {
        ALOGE("AX_SYS_MemAlloc fail, size %d, s32Ret %d!", (AX_U32)(sizeof(AX_U8) * nFileSize),
              s32Ret);
        fclose(fp);
        return AX_FALSE;
    }

    *pU64PhyAddr = phyAddr;
    *ppVirAddr = pVirAddr;
    assert(*ppVirAddr != NULL);

    AX_CHAR szLine[17] = {0};
    AX_U64 nValue = 0;
    AX_U32 nCount = 0;

    pData = (AX_U64*)*ppVirAddr;
    while (fgets(szLine, 17, fp)) {
        if (!strcmp(szLine, "\n")) {
            continue;
        }
        sscanf(szLine, "%llx", &nValue);
        pData[nCount++] = nValue;
    }

    *pMeshTableSize = nCount * 8;

    fclose(fp);

    return 0;
}

AX_S32 sample_gdc_init(AX_CHAR* pMeshFile, const AX_VIDEO_FRAME_T* pSrcFrame,
                       AX_VIDEO_FRAME_T* pDstFrame, GDC_HANDLE* pGdcHandle, AX_U64* pMeshPhyAddr,
                       AX_VOID** pMeshVirAddr) {
    CHECK_POINTER(pMeshFile);
    CHECK_POINTER(pSrcFrame);
    CHECK_POINTER(pDstFrame);
    CHECK_POINTER(pGdcHandle);
    CHECK_POINTER(pMeshPhyAddr);
    CHECK_POINTER(pMeshVirAddr);

    AX_S32 ret = 0;
    AX_U64 phyAddr = 0;
    AX_VOID* pVirAddr = NULL;
    AX_U32 meshGrid = 33;
    AX_U32 meshAlign = 16;
    AX_U32 meshSize = meshGrid * meshGrid * 16;
    AX_S32 srcWidth = pSrcFrame->u32Width;
    AX_S32 srcHeight = pSrcFrame->u32Height;
    AX_S32 dstWidth = pDstFrame->u32Width;
    AX_S32 dstHeight = pDstFrame->u32Height;

    ret =
        sample_load_mesh_file_to_cmm(pMeshFile, (AX_VOID**)&pVirAddr, (AX_U64*)&phyAddr, &meshSize);
    if (ret) {
        ALOGE("sample_load_mesh_file_to_cmm fail, ret %d!", ret);
        goto RET_ERR1;
    }

    *pMeshPhyAddr = phyAddr;
    *pMeshVirAddr = pVirAddr;

    AX_IVPS_GDC_ATTR_T gdcAttr;
    memset(&gdcAttr, 0, sizeof(AX_IVPS_GDC_ATTR_T));
    gdcAttr.eGdcType = AX_IVPS_GDC_MAP_USER;
    gdcAttr.tMapUserAttr.nMeshStartX = 0;
    gdcAttr.tMapUserAttr.nMeshStartY = 0;
    gdcAttr.tMapUserAttr.nMeshWidth =
        AX_ALIGN_UP(AX_ROUND_UP_DIV(dstWidth, meshGrid - 1), meshAlign);
    gdcAttr.tMapUserAttr.nMeshHeight =
        AX_ALIGN_UP(AX_ROUND_UP_DIV(dstHeight, meshGrid - 1), meshAlign);
    gdcAttr.tMapUserAttr.nMeshNumH = meshGrid;
    gdcAttr.tMapUserAttr.nMeshNumV = meshGrid;
    gdcAttr.tMapUserAttr.pUserMap = NULL;
    gdcAttr.tMapUserAttr.nMeshTablePhyAddr = phyAddr;
    gdcAttr.nSrcWidth = AX_ALIGN_UP(srcWidth, 2);
    gdcAttr.nSrcHeight = AX_ALIGN_UP(srcHeight, 2);
    gdcAttr.nDstWidth = AX_ALIGN_UP(dstWidth, 2);
    gdcAttr.nDstHeight = AX_ALIGN_UP(dstHeight, 2);
    gdcAttr.nDstStride = AX_ALIGN_UP(pDstFrame->u32PicStride[0], 128);
    gdcAttr.eDstFormat = AX_FORMAT_YUV420_SEMIPLANAR;

    ALOGD("GDC Init: srcWidth=%d, srcHeight=%d, dstWidth=%d, dstHeight=%d, dstStride=%d",
          gdcAttr.nSrcWidth, gdcAttr.nSrcHeight, gdcAttr.nDstWidth, gdcAttr.nDstHeight,
          gdcAttr.nDstStride);
    ALOGD("GDC Init: nMeshWidth=%d, nMeshHeight=%d", gdcAttr.tMapUserAttr.nMeshWidth,
          gdcAttr.tMapUserAttr.nMeshHeight);

    ret = AX_IVPS_GdcWorkCreate(pGdcHandle);
    if (ret) {
        ALOGE("AX_IVPS_GdcWorkCreate failed! Error Code:0x%X", ret);
        goto RET_ERR1;
    }
    ret = AX_IVPS_GdcWorkAttrSet(*pGdcHandle, &gdcAttr);
    if (ret) {
        ALOGE("AX_IVPS_GdcWorkAttrSet FAILED! ret:0x%x", ret);
        goto RET_ERR2;
    }

    return 0;

RET_ERR1:
    if (phyAddr && pVirAddr) {
        AX_SYS_MemFree(phyAddr, (AX_VOID*)pVirAddr);
        *pMeshPhyAddr = 0;
        *pMeshVirAddr = NULL;
    }
RET_ERR2:
    if (*pGdcHandle) {
        AX_IVPS_GdcWorkDestroy(*pGdcHandle);
        *pGdcHandle = 0;
    }

    return -1;
}

AX_S32 sample_gdc_deinit(SAMPLE_RESOURCE_T* pResource, AX_S32 idx) {
    CHECK_POINTER(pResource);

    GDC_HANDLE gdcHandle = pResource->gdcHandle[idx];
    AX_U64* pMeshPhyAddr = &pResource->meshPhyAddr[idx];
    AX_VOID** pMeshVirAddr = &pResource->meshVirAddr[idx];

    CHECK_POINTER(pMeshPhyAddr);
    CHECK_POINTER(pMeshVirAddr);

    if (gdcHandle) {
        AX_IVPS_GdcWorkDestroy(gdcHandle);
        pResource->gdcHandle[idx] = 0;
    }

    if (*pMeshPhyAddr && *pMeshVirAddr) {
        AX_SYS_MemFree(*pMeshPhyAddr, *pMeshVirAddr);
        *pMeshPhyAddr = 0;
        *pMeshVirAddr = NULL;
    }

    return 0;
}

#ifndef SAMPLE_RESOURCE_H
#define SAMPLE_RESOURCE_H

// Host/AXCL port: only AX data types are needed here (GDC_HANDLE lives in
// ax_ivps_type.h, AX_VIDEO_FRAME_T in ax_global_type.h). ax_ivps_type.h refers
// to AX_POOL types, so ax_pool_type.h must be included first.
#include "ax_global_type.h"
#include "ax_ivps_type.h"
#include "ax_pool_type.h"

#define SAMPLE_PIPE_NUM (2)

typedef struct SAMPLE_RESOURCE_S {
    AX_VIDEO_FRAME_T srcFrame;
    AX_VIDEO_FRAME_T cscFrame;
    AX_VIDEO_FRAME_T resizeFrame[SAMPLE_PIPE_NUM];
    AX_VIDEO_FRAME_T dewarpFrame[SAMPLE_PIPE_NUM];
    AX_VIDEO_FRAME_T bgrFrame;
    AX_U64 meshPhyAddr[SAMPLE_PIPE_NUM];
    AX_VOID* meshVirAddr[SAMPLE_PIPE_NUM];
    GDC_HANDLE gdcHandle[SAMPLE_PIPE_NUM];
} SAMPLE_RESOURCE_T;

#endif

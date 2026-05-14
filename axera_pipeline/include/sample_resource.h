#ifndef SAMPLE_RESOURCE_H
#define SAMPLE_RESOURCE_H

#include "ax_ivps_api.h"
#include "ax_sys_api.h"

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

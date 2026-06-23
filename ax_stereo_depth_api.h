#ifndef _AX_STEREO_DEPTH_API_H_
#define _AX_STEREO_DEPTH_API_H_

#include "ax_base_type.h"
#include "ax_global_type.h"

#ifdef __cplusplus
#include <memory>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define AX_STEREO_DEFAULT_INPUT_WIDTH (1280 * 2)
#define AX_STEREO_DEFAULT_INPUT_HEIGHT 720
#define AX_STEREO_DEWARP_IMAGE_WIDTH 640
#define AX_STEREO_DEWARP_IMAGE_HEIGHT 384
#define AX_STEREO_CAMERA_CX_PIXELS 320.0f
#define AX_STEREO_CAMERA_CY_PIXELS 182.0f
#define AX_STEREO_CAMERA_FOCAL_LENGTH_X_PIXELS 400.0f
#define AX_STEREO_CAMERA_FOCAL_LENGTH_Y_PIXELS 480.0f
#define AX_STEREO_CAMERA_FOCAL_LENGTH_PIXELS AX_STEREO_CAMERA_FOCAL_LENGTH_X_PIXELS

typedef AX_VOID* AX_STEREO_HANDLE;
typedef AX_VOID* AX_STEREO_FRAME_CTX;

typedef enum { AX_STEREO_ENGINE_NPU = 0, AX_STEREO_ENGINE_BUTT } AX_STEREO_ENGINE_E;

typedef enum {
    AX_STEREO_GDC_MESH_DEFAULT = 0,
    AX_STEREO_GDC_MESH_DYNAMIC_REUSE = 1,
    AX_STEREO_GDC_MESH_DYNAMIC_FORCE = 2,
    AX_STEREO_GDC_MESH_BUTT
} AX_STEREO_GDC_MESH_MODE_E;

typedef enum {
    AX_STEREO_INPUT_FORMAT_YUYV = 0,
    AX_STEREO_INPUT_FORMAT_NV12 = 1,
    AX_STEREO_INPUT_FORMAT_BUTT
} AX_STEREO_INPUT_FORMAT_E;

typedef struct axSTEREO_ATTR_T {
    AX_STEREO_ENGINE_E eEngine;             /* RW; Inference engine type. */
    AX_BOOL bEnableGdc;                     /* RW; Enable GDC dewarp. */
    AX_STEREO_GDC_MESH_MODE_E eGdcMeshMode; /* RW; GDC mesh mode. */
    AX_BOOL bExportVoFrames;                /* RW; Export NV12 frames for VO display. */
    AX_CHAR szNpuModelPath[256];            /* RW; NPU model file path. */
    AX_CHAR szCameraSerialNumber[128];      /* RW; Camera serial number for calibration. */
    AX_S32 s32InputWidth;                   /* RW; Input stereo image width. */
    AX_S32 s32InputHeight;                  /* RW; Input stereo image height. */
} AX_STEREO_ATTR_T;

typedef struct axSTEREO_OUTPUT_T {
    AX_U64 u64FrameTimestampNs; /* R; Frame timestamp in nanoseconds. */
    AX_U32 u32RawWidth;         /* R; Raw input image width. */
    AX_U32 u32RawHeight;        /* R; Raw input image height. */
    AX_VOID* pPrivateData;      /* Internal use only. Do not modify. */
} AX_STEREO_OUTPUT_T;

/*
 * AX_STEREO_Create
 *   Create a stereo depth pipeline.
 *
 *   phPipeline [out]: handle to the created pipeline.
 *   pstAttr    [in]:  pipeline attributes. If NULL, default attributes are used.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_Create(AX_STEREO_HANDLE* phPipeline, const AX_STEREO_ATTR_T* pstAttr);

/*
 * AX_STEREO_Destroy
 *   Destroy a stereo depth pipeline and release all resources.
 *
 *   hPipeline [in]: pipeline handle from AX_STEREO_Create.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_Destroy(AX_STEREO_HANDLE hPipeline);

/*
 * AX_STEREO_GetAttr
 *   Get pipeline attributes.
 *
 *   hPipeline [in]:  pipeline handle.
 *   pstAttr   [out]: receives the current pipeline attributes.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_GetAttr(AX_STEREO_HANDLE hPipeline, AX_STEREO_ATTR_T* pstAttr);

/*
 * AX_STEREO_SendFrame
 *   Submit a YUYV stereo frame for preprocessing. Produces a frame context for
 *   subsequent inference and result retrieval.
 *
 *   hPipeline        [in]:  pipeline handle.
 *   pFrameData       [in]:  pointer to YUYV frame data.
 *   u32FrameSize     [in]:  size of frame data in bytes.
 *   u64TimestampNs   [in]:  frame capture timestamp in nanoseconds.
 *   pContext          [out]: receives an opaque frame context.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_SendFrame(AX_STEREO_HANDLE hPipeline, const AX_VOID* pFrameData,
                           AX_U32 u32FrameSize, AX_U64 u64TimestampNs,
                           AX_STEREO_FRAME_CTX* pContext);

/*
 * AX_STEREO_SendFrameEx
 *   Submit a stereo frame with an explicit input pixel format. NV12 input is
 *   injected after the YUYV->NV12 conversion stage used by AX_STEREO_SendFrame.
 *
 *   hPipeline        [in]:  pipeline handle.
 *   pFrameData       [in]:  pointer to frame data.
 *   u32FrameSize     [in]:  size of frame data in bytes.
 *   eInputFormat     [in]:  input frame format.
 *   u64TimestampNs   [in]:  frame capture timestamp in nanoseconds.
 *   pContext         [out]: receives an opaque frame context.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_SendFrameEx(AX_STEREO_HANDLE hPipeline, const AX_VOID* pFrameData,
                             AX_U32 u32FrameSize, AX_STEREO_INPUT_FORMAT_E eInputFormat,
                             AX_U64 u64TimestampNs, AX_STEREO_FRAME_CTX* pContext);

AX_S32 AX_STEREO_SendVideoFrame(AX_STEREO_HANDLE hPipeline, const AX_VIDEO_FRAME_T* pFrame,
                                AX_STEREO_INPUT_FORMAT_E eInputFormat, AX_U64 u64TimestampNs,
                                AX_STEREO_FRAME_CTX* pContext);

/*
 * AX_STEREO_RunInference
 *   Run depth inference on a preprocessed frame.
 *
 *   hPipeline [in]: pipeline handle.
 *   context   [in]: frame context from AX_STEREO_SendFrame.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_RunInference(AX_STEREO_HANDLE hPipeline, AX_STEREO_FRAME_CTX context);

/*
 * AX_STEREO_GetResult
 *   Perform post-processing on an inferred frame and retrieve the output.
 *
 *   hPipeline [in]:  pipeline handle.
 *   context   [in]:  frame context after AX_STEREO_RunInference.
 *   pOutput   [out]: receives the output data. Call AX_STEREO_ReleaseOutput when done.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_GetResult(AX_STEREO_HANDLE hPipeline, AX_STEREO_FRAME_CTX context,
                           AX_STEREO_OUTPUT_T* pOutput);

/*
 * AX_STEREO_ReleaseFrame
 *   Release a frame context obtained from AX_STEREO_SendFrame.
 *   Must be called after AX_STEREO_GetResult or on error.
 *
 *   hPipeline [in]: pipeline handle.
 *   context   [in]: frame context to release.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_ReleaseFrame(AX_STEREO_HANDLE hPipeline, AX_STEREO_FRAME_CTX context);

/*
 * AX_STEREO_ReleaseOutput
 *   Release output data obtained from AX_STEREO_GetResult.
 *
 *   pOutput [in]: output structure to release.
 *
 *   Return: 0 on success, non-zero on failure.
 */
AX_S32 AX_STEREO_ReleaseOutput(AX_STEREO_OUTPUT_T* pOutput);

/*
 * AX_STEREO_GetBaselineMeters
 *   Get the camera baseline distance in meters.
 *
 *   hPipeline [in]: pipeline handle.
 *
 *   Return: baseline distance in meters.
 */
AX_F32 AX_STEREO_GetBaselineMeters(AX_STEREO_HANDLE hPipeline);

/*
 * AX_STEREO_SetComputePointCloud
 *   Enable/disable the CPU-heavy point cloud generation at runtime. When
 *   disabled, the postprocess stage leaves the point cloud output empty.
 *
 *   hPipeline [in]: pipeline handle.
 *   bEnable   [in]: AX_TRUE to compute the point cloud, AX_FALSE to skip it.
 *
 *   Return: 0 on success, non-zero on error.
 */
AX_S32 AX_STEREO_SetComputePointCloud(AX_STEREO_HANDLE hPipeline, AX_BOOL bEnable);

/*
 * AX_STEREO_GetEngine
 *   Get the inference engine type of the pipeline.
 *
 *   hPipeline [in]: pipeline handle.
 *
 *   Return: inference engine type.
 */
AX_STEREO_ENGINE_E AX_STEREO_GetEngine(AX_STEREO_HANDLE hPipeline);

/*
 * AX_STEREO_GetDefaultModelPath
 *   Get the default NPU model file path.
 *
 *   Return: null-terminated string with the default model path.
 */
const AX_CHAR* AX_STEREO_GetDefaultModelPath(AX_VOID);

/*
 * AX_STEREO_GetEngineName
 *   Get a human-readable name for the given engine type.
 *
 *   eEngine [in]: engine type.
 *
 *   Return: null-terminated string with the engine name.
 */
const AX_CHAR* AX_STEREO_GetEngineName(AX_STEREO_ENGINE_E eEngine);

#ifdef __cplusplus
}

/*
 * C++ helper: retrieve the internal PipelineOutput pointer from AX_STEREO_OUTPUT_T.
 * This allows C++ application code to access the rich output data (rgb, depth, point cloud, etc.).
 */
namespace stereo_depth {
struct PipelineOutput;
}
stereo_depth::PipelineOutput* AX_STEREO_OutputGetData(AX_STEREO_OUTPUT_T* pOutput);
AX_S32 AX_STEREO_SendVideoFrameWithOwner(AX_STEREO_HANDLE hPipeline, const AX_VIDEO_FRAME_T* pFrame,
                                         AX_STEREO_INPUT_FORMAT_E eInputFormat,
                                         AX_U64 u64TimestampNs, std::shared_ptr<void> frameOwner,
                                         AX_STEREO_FRAME_CTX* pContext);

#endif /* __cplusplus */

#endif /* _AX_STEREO_DEPTH_API_H_ */

/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#ifndef _AX_DSP_FIK_API_H_
#define _AX_DSP_FIK_API_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <ax_base_type.h>

#define FRAME_ZERO_PADDING          0u
#define FRAME_CONSTANT_PADDING      1u
#define FRAME_EDGE_PADDING          2u
#define FRAME_PADDING_REFLECT_101   3u
#define FRAME_PADDING_MAX           4u

typedef enum {
    FIK_TYPE_U8 = 0,
    FIK_TYPE_S16,
    FIK_TYPE_U16
} AX_DSP_FIK_KERNEL_TYPE_E;

typedef struct {
    AX_S16 x;
    AX_S16 y;
} AX_DSP_FIK_POINT_S16_T;

typedef struct {
    AX_F32 x;
    AX_F32 y;
} AX_DSP_FIK_POINT_F32_T;

/**
 * Argument structure used for function xvfFilterFAST_U8 / S16.
 * @param height         image height for src and dst
 * @param width          image width for src and dst
 * @param srcPitch       image pitch for src
 * @param dstPitch       image pitch for dst
 * @param borderType     borderType = FRAME_ZERO_PADDING / FRAME_EDGE_PADDING
 * @param threshold      threshold value
 * @param tileWidth      tile width
 * @param tileHeight     tile height
 * @param src            source image pointer
 * @param dst            destination image pointer
 */
typedef struct {
    AX_U32          height;
    AX_U32          width;
    AX_U32          srcPitch;
    AX_U32          dstPitch;
    AX_S32          borderType;
    AX_S32          threshold;
    AX_S32          bytesPerPel;
    AX_U8           *src;
    AX_U8           *dst;
} AX_DSP_FIK_FILTER_FAST_T;

/**
 * Argument structure used for function xvfCornerShiTomasi().
 * @param height         image height for src and dst
 * @param width          image width for src and dst
 * @param srcPitch       image pitch for src
 * @param dstPitch       image pitch for dst
 * @param borderType     borderType = FRAME_ZERO_PADDING / FRAME_EDGE_PADDING
 * @param kernelSize     kernel size - 3, 5 etc
 * @param blockSize      covariance block size
 * @param bytesPerPelSrc number of bytes per pixel for src 1 for U8, 2 for S16
 * @param bytesPerPelDst number of bytes per pixel for dst 2 for S16, 4 for S32
 * @param tileWidth      width of tile for IVP processing
 * @param tileHeight     height of tile for IVP processing
 * @param src            source image pointer
 * @param dst            destination image pointer
 */
typedef struct {
    AX_U32          height;
    AX_U32          width;
    AX_U32          srcPitch;
    AX_U32          dstPitch;
    AX_U32          borderType;
    AX_U32          kernelSize;
    AX_U32          blockSize;
    AX_U32          bytesPerPelSrc;
    AX_U32          bytesPerPelDst;
    AX_VOID         *src;
    AX_VOID         *dst;
} AX_DSP_FIK_CORNER_SHITOMASI_T;

typedef struct {
    AX_U32              height;
    AX_U32              width;
    AX_U32              srcPitch;
    AX_U32              dstPitch;
    AX_S32              borderType;
    AX_S32              threshold;
    AX_U32              bytesPerPelSrc;
    AX_U32              bytesPerPelDst;
    AX_S16              *src;
    AX_DSP_FIK_POINT_S16_T  *dst_pts;
    AX_S16              *dst_wts;
    AX_S32               count;
} AX_DSP_FIK_EXTRACT_POINTS_T;

/**
 * Argument structure used for function xvfSort_S16.
 * @param order_flag     order ascending/descending flag
 * @param width          width of src array
 * @param srcPitch       Pitch of src array
 * @param src_keys       keys src pointer
 * @param src_values     values src pointer
 */
typedef struct {
    AX_BOOL         order_flag;
    AX_U32          width;
    AX_U32          srcPitch;
    const AX_S16    *src_keys;
    AX_U32          *src_values;
} AX_DSP_FIK_SORT_T;

/**
 * Argument structure used for function xvfCornerHarrisNonMaxima().
 * @param height         image height for src and dst
 * @param width          image width for src and dst
 * @param srcPitch       image pitch for src
 * @param dstPitch       image pitch for dst
 * @param min_dist       minimum distance
 * @param count          output count after NonMaxima
 * @param src_pts        source points pointer
 * @param src_wts        source weights pointer
 */
typedef struct {
    AX_U32          height;
    AX_U32          width;
    AX_U32          srcPitch;
    AX_U32          dstPitch;
    AX_S32          min_dist;
    AX_S32          count;
    AX_S16          *src_pts;
    AX_S16          *src_wts;
} AX_DSP_FIK_CORNER_HARRIS_NONMAXIMA_T;

#define MAX_PYRAMID	10
/**
 * Argument structure used for function xvfLKPyrDown_U8().
 * @param height         image height for level 0
 * @param width          image width for level 0
 * @param srcPitch       image pitch for level 0
 * @param  minEigThreshold	minimum threshold for termination
 * @param  criteria_epsilon	epsilon value
 * @param  maxIters		 maximum iterations for termination
 * @param  numPoints	 number of points to be tracked
 * @param borderType     borderType = ZERO_PADDING / EDGE_PADDING
 * @param numPryLevels	 number of pyramid layers
 * @param winWidth	 	 window width
 * @param winHeight	 	 window height
 * @param status         track point status
 * @param PyrTileWidth   Tile width
 * @param PyrTileWidth   Tile height
 * @param prev           array of pointers of previous image pyramid
 * @param next           array of pointers of next image pyramid
 * @param pointsToTrack  points to track
 * @param trackedPoint	 tracked points
 */
typedef struct {
    AX_U32      height;
    AX_U32      width;
    AX_U32      srcPitch;
    AX_F32      minEigThreshold;
    AX_F32      criteria_epsilon;
    AX_S32	    maxIters;
    AX_S32      numPoints;
    AX_S32      borderType;
    AX_S32	    numPryLevels;
    AX_S32      winWidth;
    AX_S32	    winHeight;
    AX_S32	    PyrTileWidth;
    AX_S32	    PyrTileHeight;
    AX_U8       *status;
    AX_U8 *prev[MAX_PYRAMID];
    AX_U8 *next[MAX_PYRAMID];
    AX_DSP_FIK_POINT_F32_T *pointsToTrack;
    AX_DSP_FIK_POINT_F32_T *trackedPoint;
} AX_DSP_FIK_LKPYRDOWN_T;

/**
 * Argument structure used for function axMaxPooling16x16.
 * @param height         image height for src and dst
 * @param width          image width for src and dst
 * @param srcPitch       image pitch for src
 * @param dstPitch       image pitch for dst
 * @param borderType     borderType = ZERO_PADDING / EDGE_PADDING
 * @param kernelType     TYPE_U8/TYPE_S16/TYPE_U16: 0/1/2
 * @param qualityLevel   quality level for point extraction, range 0-1
 * @param wtsEnable      enable dst_wts output or not
 * @param src            source current image pointer
 * @param dst_pts        destination points pointer
 * @param dst_wts        destination weights pointer
 * @param maxValue       output max value of full image response
 * @param count          output points count
 */
typedef struct {
    AX_U32                  height;
    AX_U32                  width;
    AX_U32                  srcPitch;
    AX_U32                  dstPitch;
    AX_S32                  borderType;
    AX_U32                  kernelType;
    AX_F32                  qualityLevel;
    AX_BOOL                 wtsEnable;
    AX_VOID                 *src;
    AX_DSP_FIK_POINT_S16_T  *dst_pts;
    AX_S16                  *dst_wts;
    AX_S32                  maxValue;
    AX_S32                  count;
} AX_DSP_FIK_MAX_POOLING_16X16_T;

AX_S32 AX_DSP_FIK_FilterFAST(AX_S32 dsp_id, AX_DSP_FIK_FILTER_FAST_T *param);
AX_S32 AX_DSP_FIK_CornerShiTomasi(AX_S32 dsp_id, AX_DSP_FIK_CORNER_SHITOMASI_T *param);
AX_S32 AX_DSP_FIK_ExtractPoints(AX_S32 dsp_id, AX_DSP_FIK_EXTRACT_POINTS_T *param);
AX_S32 AX_DSP_FIK_Sort(AX_S32 dsp_id, AX_DSP_FIK_SORT_T *param);
AX_S32 AX_DSP_FIK_CornerHarrisNonMaxima(AX_S32 dsp_id, AX_DSP_FIK_CORNER_HARRIS_NONMAXIMA_T *param);
AX_S32 AX_DSP_FIK_LKPyrDown(AX_S32 dsp_id, AX_DSP_FIK_LKPYRDOWN_T *param);
AX_S32 AX_DSP_FIK_MaxPooling16x16(AX_S32 dsp_id, AX_DSP_FIK_MAX_POOLING_16X16_T *param);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif/* End of #ifdef __cplusplus */
#endif/*_AX_DSP_FIK_API_H_*/

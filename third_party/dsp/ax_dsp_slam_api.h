/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#ifndef _AX_DSP_SLAM_API_H_
#define _AX_DSP_SLAM_API_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <ax_base_type.h>

#define AX_SLAM_MERGE_MAX_NMU                       (4)
#define AX_SLAM_MAX_PARTIAL_NUM                     (50)
#define AX_SLAM_MAX_MATCH_PROCESS_NUM               (1000)
#define AX_SLAM_MAX_OP_KEY_POINTS                   (4096)
#define AX_SLAM_SBAN_MAX_PROJ_PER_FRAME             (512)
#define AX_SLAM_SBAN_MAX_TOTAL_MAP_POINTS           (2064)
#define AX_SLAM_SBAN_MAX_KEY_FRAMES                 (20)
#define AX_SLAM_MAX_KEY_POINTS                      (4096 * 8)
#define AX_SLAM_MAX_KEPTS_PER_CORE                  (6000)
#define AX_SLAM_MAX_DESCRIPTOR_SIZE_IN_BYTES        (128)
#define AX_SLAM_DESCRIPTOR_SIZE_IN_BYTES_ORB        (32)
#define AX_SLAM_DESCRIPTOR_SIZE_IN_BYTES_FREAK      (64)
#define AX_SLAM_DESCRIPTOR_SIZE_IN_BYTES_SIFT       (128)
#define AX_SLAM_MAX_NEIGHBOURING_KEY_FRAMES         (8)
#define AX_SLAM_MAX_KEY_POINTS_T1_EXTENDED          (64000)
#define AX_SLAM_MAX_SBA2_MAX_POINTS                 (1024)
#define AX_SLAM_MAX_MATCH_PAIRS                     (2048)
#define AX_SLAM_NUM_LEVELS                          (8)
#define AX_SLAM_MAX_MATCHPAIRS                      (1024)

typedef struct {
    AX_U32 width;
    AX_U32 height;
    AX_U32 stride;
    AX_U32 num;
    AX_U8  *src[AX_SLAM_MERGE_MAX_NMU];
    AX_U8  *dst;
}AX_DSP_SLAM_MERGE_PARAM_T;

typedef struct {
    AX_U32 num;
    AX_F32 weightGain;
}AX_DSP_SLAM_PART_WEI_UP_PARAM_T;

typedef struct {
    AX_F32 p0_x;
    AX_F32 p0_y;
    AX_F32 p0_z;
    AX_F32 p1_x;
    AX_F32 p1_y;
    AX_F32 p1_z;
} AX_DSP_SLAM_PERCEP_POINTVEC_T;

typedef struct {
    AX_F32 p0_x;
    AX_F32 p0_y;
    AX_F32 p0_z;
    AX_F32 p1_x;
    AX_F32 p1_y;
    AX_F32 p1_z;
} AX_DSP_SLAM_MAP_POINTVEC_T;

typedef struct {
    AX_F32 weight;
    AX_F32 qw;
    AX_F32 qx;
    AX_F32 qy;
    AX_F32 qz;
    AX_F32 tx;
    AX_F32 ty;
    AX_F32 tz;
    AX_U32 PointNum;
    AX_DSP_SLAM_PERCEP_POINTVEC_T percep_pointvec_set[AX_SLAM_MAX_MATCH_PROCESS_NUM];
    AX_DSP_SLAM_MAP_POINTVEC_T map_pointvec_set[AX_SLAM_MAX_MATCH_PROCESS_NUM];
} AX_DSP_SLAM_SINGLE_PARTILE_LOSS_T;

typedef struct {
    AX_U32 partilalNum;
    AX_DSP_SLAM_SINGLE_PARTILE_LOSS_T partilal_container[AX_SLAM_MAX_PARTIAL_NUM];
} AX_DSP_SLAM_COMPUTE_PARTICLES_LOSS_T;

typedef struct {
    AX_U16    *pX1;                    // pointer to x1 co-ordinate of matching pairs [nMatchingPairs]
    AX_U16    *pY1;                    // pointer to y1 co-ordinate of matching pairs [nMatchingPairs]
    AX_U16    *pX2;                    // pointer to x2 co-ordinate of matching pairs [nMatchingPairs]
    AX_U16    *pY2;                    // pointer to y2 co-ordinate of matching pairs [nMatchingPairs]
    AX_U16    *pHammingDistance;       // Hamming distance between matching pairs [nMatchingPairs]
    AX_U16     nMatchingPairs;         // Input/Output: number of matching pairs

    // Following helper buffers are required for internal purpose for optimized DMA functioning.
    AX_U16    *pAngle2;                // pointer to ORB Angles of KeyPoint2[] i.e. (pX2[],pY2[])
    AX_U8     *pLevel2;                // pointer to pyramid layers of KeyPoint2[] i.e. (pX2[],pY2[])
    AX_U8     *pMask2;                 // pointer to Mask2. 32 bytes/256 bits each.
    AX_U8     *pDescriptor2;           // pointer to Descriptor2 array. 32 bytes/256 bits each.
}AX_DSP_SLAM_ORB_MATCHPOINTS_TABLE_T;

typedef struct
{
    AX_F32      *px1;                    // Pointer to Matched pairs x1[nMatchedPairs]
    AX_F32      *py1;                    // Pointer to Matched pairs y1[nMatchedPairs]
    AX_F32      *px2;                    // Pointer to Matched pairs x2[nMatchedPairs]
    AX_F32      *py2;                    // Pointer to Matched pairs y2[nMatchedPairs]
    AX_U16      *pIdx0;                  // Pointer to matched pairs index0[nMatchedPairs]
    AX_U16      *pIdx1;                  // Pointer to matched pairs index1[nMatchedPairs]
    AX_F32      *pDepth0;                // Pointer to matched pairs depth0[nMatchedPairs] for stereo/RGBD
    AX_F32      *pDepth1;                // Pointer to matched pairs depth1[nMatchedPairs] for stereo/RGBD
    AX_U16      nMatchedPairs;           // Total number of matched pairs
}AX_DSP_SLAM_MATCHED_PAIRS_T;

typedef struct
{
    //____________________________________________________________________________
    // Description: convertes U16 matched pairs
    // (x1[], y1[], x2[], y2[])  in pixels  to normalized co-ordinates using camera
    // calibration parameters as shown below:
    //
    // fx = CameraCalib[0 * 3 + 0];
    // fy = CameraCalib[1 * 3 + 1];
    // cx = CameraCalib[0 * 3 + 2];
    // cy = CameraCalib[1 * 3 + 2];
    //
    // normX1[i] = (x1[i] - cx) / fx;
    // normX2[i] = (x2[i] - cx) / fx;
    // normY1[i] = (y1[i] - cy) / fy;
    // normY2[i] = (y2[i] - cy) / fy;
    //____________________________________________________________________________
    AX_DSP_SLAM_ORB_MATCHPOINTS_TABLE_T   *pORBMatchPointsTable;      // Input: U16 matched pairs
    AX_F32                                *pCameraCalib;              // Input: pointer to camera calibration [3x3] matrix
    AX_DSP_SLAM_MATCHED_PAIRS_T           *pMatchedPairs;             // Output: F32 normalized matched pairs
}AX_DSP_SLAM_NORMALIZE_POINTS_T;

typedef struct {
    // Estimated global-map-points a.k.a. World Points
    AX_F32                    *X;                             // pointer to X[nPoints]
    AX_F32                    *Y;                             // pointer to Y[nPoints]
    AX_F32                    *Z;                             // pointer to Z[nPoints]
    AX_S32                    nPoints;                        // number of estimated global-map-points
}AX_DSP_SLAM_WORLD_POINTS_T;

typedef struct {
    AX_S32                    nKeyFrames;                     // input: Number of key-frames involved
    AX_S32                    nIterations;                    // input: Number of iterations for BA-N-frames
    AX_F32                    errorScaleDown;                 // input: Error scale-down factor
    AX_F32                    lambdaInit;                     // input: Initial lambda
    AX_F32                    lambdaScale;                    // input: Lamda scale factor
    AX_F32                    lambdaMin;                      // input: Minimum lambda
    AX_F32                    lambdaMax;                      // input: Maximum lambda
}AX_DSP_SLAM_BUNDLE_ADJUSTMENT_N_FRAME_CTL_T;

typedef struct {
    AX_F32                         *pPose;                    // Input/Output: [kf][9+3+3]; rot + H + T
    AX_DSP_SLAM_WORLD_POINTS_T     *LocalWorldPoints;         // Input/Output: local 3D World points
    AX_U16                         *nPointsInFrame;           // Input: pointer to nPointsInFrame[nKeyFrames]
    AX_U32                         nTotProjections;           // Input: total projections for all nKeyFrames
    AX_S32                         nMaxProjectionsPerFrame;   // Input: Max. projections in a key-frame
    AX_DSP_SLAM_BUNDLE_ADJUSTMENT_N_FRAME_CTL_T
    budle_adjustment_n_frames_ctl;                            // Input: Control parameters
    AX_U16        *AdjMat;                                    // Input: pointer to adjacency matrix[nTotProjections]
                                                              // It maps visible 2D point index to local 3D worldpoint index
    AX_F32        *xProj;                                     // Input: pointer to x-projections[nTotProjections]
    AX_F32        *yProj;                                     // Input: pointer to y-projections[nTotProjections]
    AX_F32        *pJc;                                       // Input: pointer to Jc buffer
    AX_F32        *pDelta;                                    // Input: pointer to delta buffer
                                                              // delta[3* nPoints + 7* nKeyFrames]
    AX_F32        *pJp;                                       // Input: pointer to Jp buffer
    AX_F32        *pError;                                    // Input: pointer to error buffer
    AX_F32        *pQ;                                        // Input: pointer to Quaternion buffer
    AX_F32        *pScratch;                                  // Input: pointer to scratch memory
}AX_DSP_SLAM_BUNDLE_ADJUSTMENT_N_FRAMET_T;

typedef struct
{
    AX_U8                  *y;         // Data pointer to the top left corner
    AX_U32                 width;      // image width
    AX_U32                 pitch;      // image pitch
    AX_U32                 height;     // image height
}AX_DSP_SLAM_Y_IMAGE_T;

typedef struct {
    AX_U16    *pX;                    // pointer to array of X co-ordinates of Key-points[nKeyPoints]
    AX_U16    *pY;                    // pointer to array of Y co-ordinates of Key-points[nKeyPoints]
    AX_U16    *pAngle;                // pointer to array of ORB Angles[nKeyPoints]
    AX_U8     *pLevel;                // pointer to array of level[nKeyPoints]
    AX_U8     *pMask;                 // pointer to Mask. 256 bits (32 Bytes) each.
    AX_U8     *pDescriptor;           // pointer to descriptor array. 256 bits (32 Bytes) each.
    AX_U16    nKeyPoints;             // total Key-Points in Image
    AX_F32    *pXf;                   // pointer to array of AX_F32 X co-ordinates of Key-points[nKeyPoints]
    AX_F32    *pYf;                   // pointer to array of AX_F32 Y co-ordinates of Key-points[nKeyPoints]
    AX_F32    *pDepth;                // pointer to array of depths of Key-points[nKeyPoints]
    AX_U16    *pIdx;                  // pointer to array of indices of Key-points[nKeyPoints]
}AX_DSP_SLAM_ORB_KEYPOINTS_T;

typedef enum {
    AX_DSP_SLAM_DESCRIPTOR_TYPE_ORB = 1,
    AX_DSP_SLAM_DESCRIPTOR_TYPE_FREAK = 2,
    AX_DSP_SLAM_DESCRIPTOR_TYPE_SIFT = 3
}AX_DSP_SLAM_DESCRIPTOR_TYPE_E;

typedef struct {
    AX_DSP_SLAM_ORB_KEYPOINTS_T      *keyPointsLeft;                // input: left keypoints
    AX_DSP_SLAM_ORB_KEYPOINTS_T      *keyPointsRight;               // input: right keypoints
    AX_DSP_SLAM_Y_IMAGE_T            *imageLeft;                    // input: left image
    AX_DSP_SLAM_Y_IMAGE_T            *imageRight;                   // input: right imgage
    AX_F32                           *cameraCalibration;            // input: camera calibration
    AX_F32                           bf;                            // input: stereo baseline * f
    AX_F32                           downScaleFactor;               // input: downscale factor
    AX_DSP_SLAM_DESCRIPTOR_TYPE_E    descriptorType;                // input: descriptor type (ORB:1 or SIFT:3)
    AX_S16                           minDisparityROI;               // input: Minimum Disparity for ROI search
    AX_S16                           minDisparity;                  // input: Minimum Disparity for final depth computation
    AX_S16                           maxDisparity;                  // input: Maximum Disparity for final depth computation
    AX_U16                           distanceThreshold;             // input: Minimum hamming/euclidean distance for matching
    AX_F32                           *depth;                        // output: stereo depth estimation
    AX_F32                           *bestRx;                       // output: best right x coordinate
    AX_U16                           *bestDist;                     // output: best distance
}AX_DSP_SLAM_DEPTH_FROM_STEREO_T;


typedef enum {
    AX_DSP_SLAM_SENSOR_TYPE_MONO = 1,
    AX_DSP_SLAM_SENSOR_TYPE_STEREO = 2,
    AX_DSP_SLAM_SENSOR_TYPE_RGBD = 3
}AX_DSP_SLAM_SENSOR_TYPE_E;

typedef struct
{
    //  We have (x1[], y1[], x2[], y2[], HamDist[])
    //  1. If we have any (x2[i],y2[i]) repeated, then we have multiple matches.
    //  2. In such case, we will retain matching pair with minimum hamming distance.
    //  3. For example,
    //     [(x1[m],y1[m]), (x2[m],y2[m]), HamDist[m]]
    //     [(x1[n],y1[n]), (x2[n],y2[n]), HamDist[n]]
    //     where x2[m] = x2[n] and y2[m] = y2[n],
    //     then we keep the pair with min (HamDist[m], HamDist[n]);
    AX_DSP_SLAM_ORB_MATCHPOINTS_TABLE_T *pORBMatchPointsTable;
    AX_U16                *pMatchedIndices0;        // Input/Output: pointer to matched Indices0[nMatchingPairs]
    AX_U16                *pMatchedIndices1;        // Input/Output: pointer to matched Indices1[nMatchingPairs]
    AX_F32                *pMatchedDepth0;          // Input/Output: pointer to matched depth0[nMatchingPairs]
    AX_F32                *pMatchedDepth1;          // Input/Output: pointer to matched depth1[nMatchingPairs]
    AX_DSP_SLAM_SENSOR_TYPE_E sensorType;           // Input: Sensor type as Mono/Stereo/RGBD
}AX_DSP_SLAM_ELIMINATE_MULTIPLEMATCHES_T;

typedef struct {
    // key-points (Corners) buffers
    AX_U16              *xi;                            // [AX_SLAM_MAX_OP_KEY_POINTS];
    AX_U16              *yi;                            // [AX_SLAM_MAX_OP_KEY_POINTS];
    AX_U16              *angle;                         // [AX_SLAM_MAX_KEPTS_PER_CORE];
    AX_U8               *descriptor;                    // [AX_SLAM_MAX_OP_KEY_POINTS * 32]; ORB
                                                        // [AX_SLAM_MAX_KEPTS_PER_CORE * 64];    [-FREAK-]
                                                        // [AX_SLAM_MAX_KEPTS_PER_CORE * 128];    [-SIFT-]
    AX_U8               *mask;                          // [AX_SLAM_MAX_OP_KEY_POINTS * 32]; [-ORB-]
    AX_U16              *scale;                         // [AX_SLAM_MAX_KEPTS_PER_CORE]; FREAK and SIFT only [0:64]
    AX_U8               *level;                         // [AX_SLAM_MAX_KEPTS_PER_CORE]; ORB only [0:7]
    AX_F32              *xf;                            // [AX_SLAM_MAX_OP_KEY_POINTS];
    AX_F32              *yf;                            // [AX_SLAM_MAX_OP_KEY_POINTS];
    AX_S32              *worldPointIdx;                 // [AX_SLAM_MAX_OP_KEY_POINTS]; [0:MAX_WORLD_POINTS]
    AX_F32              *depth;                         // [AX_SLAM_MAX_OP_KEY_POINTS]; already divided by depth scale
    AX_S32              nCorners;                        // number of corners detected in this key-frame
    AX_F32              *pose;                           // Camera pose: [12] 3x3 rotation + 3 H
    AX_F32              *camera;                         // Camera location: [3] tx, ty, tz
    AX_S32              frame;                           // frame number
    AX_F32              *X;                              // pointer to X-coord of 3D point
    AX_F32              *Y;                              // pointer to Y-coord of 3D point
    AX_F32              *Z;                              // pointer to Z-coord of 3D point
}AX_DSP_SLAM_KEY_FRAME_T;

typedef struct {
    AX_F32                    zThresholdLo;                    // input: z-threshold low
    AX_F32                    zThresholdHi;                    // input: z-threshold high
    AX_F32                    triangulationErrorThreshold;     // input: triangulation error threshold
    AX_F32                    fundamentalErrorThreshold;       // input: fundamental error threshold
    AX_F32                    outlierScaleThreshold;           // input: outlier scale threshold for RGBD/Stereo
    AX_S32                    nKeyFrames;                      // input: no. of key-farmes
    AX_U16                    hamDistThreshold;                // input: hamming distance threshol
    AX_U16                    xDiffThreshold;                  // input: ROI-X for match
    AX_U16                    yDiffThreshold;                  // input: ROI_Y for match
    AX_U16                    angleDiffThreshold;              // input: angle difference threshold
    AX_U16                    levelDiffThreshold;              // input: level difference threshold
}AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_CTL_T;

typedef struct {
    AX_DSP_SLAM_KEY_FRAME_T            *previousKeyFrames[AX_SLAM_MAX_NEIGHBOURING_KEY_FRAMES];
    AX_DSP_SLAM_KEY_FRAME_T            *currentFrame;          // Input: current frame info.
    AX_S32                              imgWidth;              // input: image width
    AX_S32                              descriptorType;        // input: descriptor type
    AX_S32                              sensorType;            // input: sensor type
    AX_F32                              *cameraCalib;          // input: camera calibration parameters
    AX_F32                              scale;                 // input: estimated scale
    AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_CTL_T
                                        triangulate_new_map_points_ctl;
    AX_DSP_SLAM_WORLD_POINTS_T          *worldPoints;         // output: World 3D points
    AX_S32                              *nTriangulatedPoints; // output: number of newly triangulated points
}AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_T;

typedef struct {
    AX_F32                    zThresholdLo;                     // input: z-threshold low
    AX_F32                    zThresholdHi;                     // input: z-threshold high
    AX_S32                    nMaxTriangulatedPoints;           // input: maximum tringulated points
}AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_USING_DEPTH_CTL_T;

typedef struct {
    AX_DSP_SLAM_KEY_FRAME_T     *currentFrame;                  // Input: current frame info.
    AX_F32                      *cameraCalib;                   // input: camera calibration parameters
    AX_F32                      scale;                          // input: estimated scale
    AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_USING_DEPTH_CTL_T
                                triangulate_new_map_points_ctl; // input: control parameters
    AX_DSP_SLAM_WORLD_POINTS_T  *worldPoints;                   // output: World 3D points
}AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_USING_DEPTH_T;

#define AX_DSP_SLAM_METHOD_RANSAC 1
typedef struct {
    AX_F32* x;
    AX_F32* y;
    AX_S32 nPoints;
}AX_DSP_SLAM_POINTS_T;

typedef struct {
    AX_DSP_SLAM_POINTS_T* srcPoints;
    AX_DSP_SLAM_POINTS_T* dstPoints;
    AX_S32            method;
    AX_F32            ransacReprojThreshold;
    AX_U8             *inliers_mask;
    AX_S32            maxIters;
    AX_F32            confidence;
    AX_F32            *H;
}AX_DSP_SLAM_FIND_HOMOGRAPHY_T;

#define AX_DSP_SLAM_IMU_N_PARAMS        (10)    // bias_w[3], bias_a[3], g[3], scale
#define AX_DSP_SLAM_IMU_N_MAX_READINGS  (10)    // IMU max readings between 2 frames
#define AX_DSP_SLAM_IMU_N_ERR_TERMS     (10)    // q[4], v[3], T[3]
#define AX_DSP_SLAM_IMU_WT_N_POINTS     (1000)  // IMU weight [1.0:0.0] for nPoints [0:AX_DSP_SLAM_IMU_WT_N_POINTS]

enum AX_DSP_SLAM_IMU_PARAMS {
    AX_DSP_SLAM_IMU_BIAS_WX, AX_DSP_SLAM_IMU_BIAS_WY, AX_DSP_SLAM_IMU_BIAS_WZ,
    AX_DSP_SLAM_IMU_BIAS_AX, AX_DSP_SLAM_IMU_BIAS_AY, AX_DSP_SLAM_IMU_BIAS_AZ,
    AX_DSP_SLAM_IMU_GX, AX_DSP_SLAM_IMU_GY, AX_DSP_SLAM_IMU_GZ, AX_DSP_SLAM_IMU_SCALE
};

typedef struct
{
    AX_F32      param[AX_DSP_SLAM_IMU_N_PARAMS];            // Gyro_bias[3], acc_bias[3], gravity[3]
    AX_U64      timeStampPrevFrame;                         // time stamp for Previous frame
    AX_U64      timeStampCurrFrame;                         // time stamp for current frame
    AX_F32      w[3 * AX_DSP_SLAM_IMU_N_MAX_READINGS];      // Gyroscope readings
    AX_F32      a[3 * AX_DSP_SLAM_IMU_N_MAX_READINGS];      // Accelerometer readings
    AX_S32      nIMUReadings;                               // number of IMU readings between 2 frames
    AX_F32      TcamPrevF[3];                               // Camera Postion for previous frame
    AX_F32      q[4];                                       // Quaternion q[4] in IMU frame of reference
    AX_F32      v[3];                                       // Velocity v[3] in IMU frame of reference
    AX_F32      T[3];                                       // Position T[3] in IMU frame of reference
    AX_BOOL     initialized;                                // IMU bias and gravity estimated?
    AX_F32      q_EstCamCurrF[4];                           // Estimated Quaternion q[4] in camera frame of reference
    AX_F32      v_avgCam[3];                                // Estimated average Velocity in camera frame of reference
    AX_F32      T_EstCamCurrF[3];                           // Estimated Position in camera frame of reference
    AX_F32      aDrift[3];                                  // Drift value to be subtracted before double integration
    AX_S32      DriftnPointsPO;                             // nPoints in visual path
    AX_F32      DriftnOutliersPOPerc;                       // outliers percentage in visual path

    AX_CHAR*    CTL_fileName;                               // IMU File name with path
    AX_F32*     CTL_Ric;                                    // Fixed Rotation between IMU and Camera.
    AX_F32      CTL_delta_t_readings;                       // time in seconds between 2 consecutive IMU readings (e.g. 5ms => 0.005)
    AX_F32      CTL_delta_t_frames;                         // time in seconds between 2 consecutive imahes/frames
    AX_F32      CTL_v_weight;                               // weight to velocity error terms in pose optimization
    AX_F32      CTL_T_weight;                               // weight to position error terms in pose optimization
    AX_S32      CTL_DriftnPointsPOThreshold;                // Update drift ONLY if these many nPoints in visual path
    AX_F32      CTL_DriftnOutliersPOPercThd;                // Update drift ONLY if %outliers less than this threshold

    // Calibration parameters
    AX_F32      init_lamda;                                 // IMU calibration lamda init
    AX_F32      init_T_weight;                              // IMU Translation weight for the error vector
    AX_F32      init_step_size;                             // IMU step size for finding jacobians
    AX_F32      init_gravity_step_size;                     // IMU step size of gravity for finding jacobians
    AX_F32      init_gravity_idle_frames;                   // IMU initial idle frames for rough estimate of gravity
    AX_F32      init_lamda_scale;                           // IMU lamda scale
    AX_F32      init_lamda_max;                             // IMU lamda max

    AX_S32        IMUReadingsCntperKF[100];                 // Added for findbiasngravity, max 100 key-frames for init
}AX_DSP_SLAM_IMU_T;

typedef struct {
    //___________________________________________________________________________
    // Contains information about set of 3D points seen in current frame and
    // their related information.
    //___________________________________________________________________________
    // Selected Global-Map-absolute-3D-points, which are 'matched' in 'current' frame
    AX_F32  *X;             // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_F32  *Y;             // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_F32  *Z;             // [nPoints] ...max (N_KEY_POINTS * 2)

    // Observed    normalized matched 2D points in current frame
    AX_F32  *xn;            // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_F32  *yn;            // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_F32  *infoWeight;    // [nPoints] ...max (N_KEY_POINTS * 2)
                                                                    //    based on which layer corner is detected

    // Matching    pairs for Histogram based Outlier-Rejection
    AX_U16  *x1p;           // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_U16  *y1p;           // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_U16  *x2p;           // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_U16  *y2p;           // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_U16  *angle1;        // [nPoints] ...max (N_KEY_POINTS * 2)
    AX_U16  *angle2;        // [nPoints] ...max (N_KEY_POINTS * 2)

    // KeyFrame insertion logic
    AX_S32  nPointsRefKF;   // points matched for current frame = prev key frames' frame + 1
    AX_S32  nPointsRefKF2;  // points matched for current frame > prev key frames' frame + 1
    AX_S32  nPoints;        // number of 3D points observed in current frame
}AX_DSP_SLAM_LOCAL_MAP_T;

typedef struct {
    // Pose-optimization buffers
    AX_F32  *infoWeightApply;           // [nPoints] ...max (N_KEY_POINTS * 2)
                                        // Actual weight applied. One of the following
                                        // 1. infoWeight
                                        // 2. sqrtf(POSE_OPT_ERR_DELTA / err)
                                        // 3. 0.0
    AX_U8   *outlier;                   // pointer to outlier buffer
    AX_F32  *E;                         // pointer to error vector
    AX_F32  *Jc;                        // pointer to Jc matrix
    AX_F32  *pose;                      // pointer to pose[12] = 3x3 rotation + 3x1 H
}AX_DSP_SLAM_POSEOPT_BUFF_T;

typedef struct {
    // Pose-optimization control parameters
    AX_F32  lambdaInit;                 // input: LAMBDA_INIT
    AX_F32  lambdaScale;                // input: LAMBDA_SCALE
    AX_F32  lambdaMin;                  // input: LAMBDA_MIN
    AX_F32  lambdaMax;                  // input: LAMBDA_MAX
    AX_F32  scaleDown;                  // Input: Error scale-down factor
    AX_F32  threshold1;                 // Input: threshold for robust Huber loss
    AX_F32  threshold2;                 // Input: initial outlier threshold
    AX_U16  nOuterIterations;           // Input: Number of outer loop iterations
    AX_U16  nInnerIterations;           // Input: Number of inner loop iterations
    AX_BOOL IMUon;                      // Input: IMU ON/OFF
}AX_DSP_SLAM_POSEOPT_CTL_T;

typedef struct {
    AX_DSP_SLAM_LOCAL_MAP_T *local_map;             // input
                // AX_F32    *X;                    // input: pointer to X[nPoints] (3D point)
                // AX_F32    *Y;                    // input: pointer to Y[nPoints] (3D point)
                // AX_F32    *Z;                    // input: pointer to Z[nPoints] (3D point)
                // AX_F32    *xn;                   // input: pointer to xn[nPoints] (observation)
                // AX_F32    *yn;                   // input: pointer to yn[nPoints] (observation)
                // AX_F32    *infoWeight;           // input: pointer to infoWeight[nPoints]
                // AX_S32    nPoints;               // input: number of 3D points in curr. frame

    AX_DSP_SLAM_POSEOPT_CTL_T   *pose_opt_ctl;      // input control parameters
                // AX_F32    lambdaInit;            // input: Initial lambda
                // AX_F32    lambdaScale;           // input: Lambda scale to modify lambda
                // AX_F32    lambdaMin;             // input: Minimum lambda
                // AX_F32    lambdaMax;             // input: Maximum lambda
                // AX_F32    scaleDown;             // Input: Error scale-down factor
                // AX_F32    threshold1;            // Input: threshold for robust Huber loss
                // AX_F32    threshold2;            // Input: initial outlier threshold
                // AX_U16    nOuterIterations;      // Input: Number of outer loop iterations
                // AX_U16    nInnerIterations;      // Input: Number of inner loop iterations

    AX_DSP_SLAM_POSEOPT_BUFF_T    *pose_opt_buff;   // buffers
                // AX_F32    *infoWeightApply;      // pointer to weight[nPoints]
                // AX_U8     *outlier;              // pointer to outlier[nPoints]
                // AX_F32    *E;                    // pointer to Error vector
                // AX_F32    *Jc;                   // pointer to Jc matrix
                // AX_F32    *pose;                 // pointer to pose[12] =
                                                    // 3x3 rotation + 3x1 H

    AX_DSP_SLAM_IMU_T   *IMU;                       // IMU addition
}AX_DSP_SLAM_POSE_OPTIMIZATION_T;

typedef struct {
    AX_F32      *x1;            //pointer to normalized x-coord of keypoint in image 1
    AX_F32      *y1;            //pointer to normalized y-coord of keypoint in image 1
    AX_F32      *x2;            //pointer to normalized x-coord of keypoint in image 2
    AX_F32      *y2;            //pointer to normalized y-coord of keypoint in image 2
    AX_F32      *X;             // pointer to X-coord of 3D point
    AX_F32      *Y;             // pointer to Y-coord of 3D point
    AX_F32      *Z;             // pointer to Z-coord of 3D point
    AX_U16      *idx3D;         //unused
    AX_U16      *matchcount;    //number of matched pairs
}AX_DSP_SLAM_LMPT;

typedef struct {

    AX_S32      nIterations;                    //number of iterations
    AX_U16      nMaxProjectionsPerFrame;        //max projections per frame
    AX_F32      errorScaleDown;                 // error scale down factor
    AX_F32      lambdaInit;                     //initial lambda value
    AX_F32      lambdaScale;                    //lambda scaling factor
    AX_F32      lambdaMin;                      //min lambda
    AX_F32      lambdaMax;                      //max lambda
}AX_DSP_SLAM_SPASEBA_2_FRAMES_CTL_T;

typedef struct {
    AX_DSP_SLAM_LMPT                    *sm_MatTab;             // Input/Output: MatchPoints Table containing normalized matched pairs
    AX_F32                              *H;                     // Input/Output: pointer to Pose
    AX_DSP_SLAM_SPASEBA_2_FRAMES_CTL_T  sparseba2_frames_ctl;   // Input: control paramters
                                                                // nIterations, nMaxProjectionsPerFrame,
                                                                // errorScaleDown, lambdaInit,
                                                                // lambdaScale, lambdaMin, lambdaMax
    AX_F32                              *pJp;                   // Input: pointer to Jp buffer
    AX_F32                              *pJc;                   // Input: pointer to Jc buffer
    AX_F32                              *pDelta;                // Input: pointer to delta buffer
    AX_F32                              *pError;                // Input: pointer to error buffer
    AX_F32                              *pScratch;              // Input: pointer to scratch memory
}AX_DSP_SLAM_SPASEBUNDLEADJUSTMENT_2_FRAMES_T;

typedef struct {
    AX_U16      desc_type;                          // Input: Specifies Descriptor type as follows:
                                                    // AX_DSP_SLAM_DESCRIPTOR_TYPE_ORB
                                                    // AX_DSP_SLAM_DESCRIPTOR_TYPE_FREAK
                                                    // AX_DSP_SLAM_DESCRIPTOR_TYPE_SIFT
    AX_U16      sensorType;                         // Input: Specified sensor type as follows:
                                                    // AX_DSP_SLAM_SENSOR_TYPE_MONO
                                                    // AX_DSP_SLAM_SENSOR_TYPE_STEREO
                                                    // AX_DSP_SLAM_SENSOR_TYPE_RGBD
    AX_U16      hamDistThreshold;                   // Input: Maximum Hamming Distance to classify as a matching pair
    AX_U16      xThreshold;                         // Input: Maximum x pixel value allowed to be visible in current image (IMG_WIDTH)
    AX_U16      yThreshold;                         // Input: Maximum y pixel value allowed to be visible in current image (IMG_HEIGHT)
    AX_U16      xDiffThreshold;                     // Input: Maximum x pixel difference allowed between matched pairs
    AX_U16      yDiffThreshold;                     // Input: Maximum y pixel difference allowed between matched pairs
    AX_U16      angleDiffThreshold;                 // Input: Maximum ORB angle difference allowed between matched pairs
    AX_U16      levelDiffThreshold;                 // Input: Maximum layer (level) difference allowed between matched pairs: [ORB]

    AX_U16      loopCount;                          // Input:  Number of neighbouring key-frames
    AX_F32      secondBestThreshold;                // Input:  Second Best Match threshold
    AX_F32      outlierScaleThreshold;              // Input:  Outlier Scale Threshold (if not MONO)
    AX_F32      minZr;                              // Input:  minimum Zr allowed
    AX_F32      maxZr;                              // Input:  maximum Zr allowed
    AX_F32      scale;                              // Input:  scale estimated
    AX_F32      *infoWeightTable;                   // Input:  [8]
    AX_U16      maxLocalMapPoints;                  // Input:  maximum local points allowed in local map
    AX_U8       flag;                               // Input: To check the calling function; 0: SearchByProjection, 1: SearchByProjectionNKF
}AX_DSP_SLAM_SEARCH_BY_PROJ_CTL_T;

typedef struct {
    AX_DSP_SLAM_KEY_FRAME_T             *previousKeyFrame;              // Input: previous key-frame info.
    AX_DSP_SLAM_KEY_FRAME_T             *currentFrame;                  // Input: current frame info
    AX_DSP_SLAM_SEARCH_BY_PROJ_CTL_T    *search_by_projection_ctl;      // Input: Control Parameters
    AX_F32                              *cameraCalibrartion;            // Input: Camera Calibration
    AX_DSP_SLAM_LOCAL_MAP_T             *local_map;
                                                                        // Output: local_map X[], Y[], Z[], xn[], yn[],
                                                                        // x1p[], y1p[], x2p[], y2p[]
                                                                        // local_map.infoWeight[]
                                                                        // local_map.angle1[]
                                                                        // local_map.angle2[]
}AX_DSP_SLAM_SEARCH_BY_PROJECTIONT_T;

typedef struct {
    AX_DSP_SLAM_KEY_FRAME_T            *previousKeyFrames[AX_SLAM_MAX_NEIGHBOURING_KEY_FRAMES]; // Input: previous key-frame info.
    AX_DSP_SLAM_KEY_FRAME_T            *currentFrame;                                           // Input: current frame info.
    AX_DSP_SLAM_SEARCH_BY_PROJ_CTL_T   *search_by_projection_ctl;                               // Input: Control parameters
    AX_F32                             *cameraCalibrartion;                                     // Input: fx, fy, cx, cy
    AX_DSP_SLAM_LOCAL_MAP_T            *local_map;
    // Output: local_map X[], Y[], Z[], xn[], yn[],
    // x1p[], y1p[], x2p[], y2p[]
    // local_map.infoWeight[]
    // local_map.angle1[]
    // local_map.angle2[]
}AX_DSP_SLAM_SEARCH_BY_PROJECTIONT_NEIGHBOURING_KEYFRAMES_T;

typedef struct
{
    AX_U8     *ptr[AX_SLAM_NUM_LEVELS];     // pointer to image at different layers
    AX_U32    width[AX_SLAM_NUM_LEVELS];    // width of the image at different layers
    AX_U32    pitch[AX_SLAM_NUM_LEVELS];    // pitch of the image at different layers
    AX_U32    height[AX_SLAM_NUM_LEVELS];   // height of the image at different layers
    AX_U8     nLayers;                      // Number of layers. Must be between [2:8]
}AX_DSP_SLAM_IMAGE_PYRAMID_T;

typedef struct
{
    AX_F32    downScaleFactor;       // Input: Down-scale factor
                                        // Range: 1.0 < Downscale factor <= 2.0
                                        // For example, downScaleFactor = 1.2 will down-size
                                        // image 640x480 at Layer-0 to 533x400 for Layer-1
    AX_U8     fast9Threshold;           // Input: FAST-9 Threshold to decide corner or not.
    AX_U32    fast9EdgeThreshold;       // Input: number of boundary rows/columns to be excluded for corner computation in each stage of image pyramid
    AX_U16    nDesiredKeyPoints;        // Input: number of desired unique keypoints to be detected
}AX_DSP_SLAM_ORB_CTL_T;

typedef struct
{
    // Description: calculates ORB descriptors for key-points for
    // entire 8-bit grey image/frame. It does following:
    // 1. Accepts input image and downscales it by downScaleFactor at every layer
    //    to create nPyramidLayers (<=8)  pyramid levels of images.
    // 2. Performs FAST9() based key-point detection using fast9Threshold.
    // 3. Performs NMS3x3() non-maxima supression on detected key-points.
    // 4. Performs NthElement8bBit() to select desired number of key-points with
    //    high scores. Note that same corner may be detected at multiple levels.
    // 5. Calculates ORB angle for every key-point.
    // 6. Calculate Oriented and Rotated 256-bit BRIEF descriptor for every
    //    key-point.
    // 7. Outputs (x, y) co-ordinate of and 256-bit descriptor for all key-points.
    //____________________________________________________________________________
    AX_DSP_SLAM_IMAGE_PYRAMID_T     *pImagePyramid;
    AX_DSP_SLAM_ORB_CTL_T           *pORB_CTL;
    AX_DSP_SLAM_ORB_KEYPOINTS_T     ORBKeyPoints;
}AX_DSP_SLAM_FAST9_DETECT_ORB_COMPUTE_T;

typedef struct {
    AX_F32          *depthOut;  // Output: 1D depth array pointer
    AX_U16          *depthIn;   // Input: 2D depth array pointer
    AX_U16          *xi;        // Input: x co-ordinates pointer
    AX_U16          *yi;        // Input: y co-ordinates pointer
    AX_S32          imgWidth;   // Input: Image width
    AX_S32          imgHeight;  // Input: Image height
    AX_S32          nCorners;   // Input: Number of corners in image
    AX_F32          DepScale;   // Input: Depth scale
} AX_DSP_SLAM_CONVERT_2D_DEPTH_TO_1D_T;

#define AX_DSP_SLAM_SGBM_WIN_SIZE                    (3)
#define AX_DSP_SLAM_SGBM_SOBEL_WIN_SIZE              (3)
#define AX_DSP_SLAM_SGBM_UNIQUENESS_RATIO            (10)
#define AX_DSP_SLAM_SGBM_DISP_LEFT_RIGHT_MAX_DIFF    (1)
#define AX_DSP_SLAM_SGBM_PRE_FILT_CAP                (63)
#define AX_DSP_SLAM_SGBM_MAX_DISPARITY               (128)

typedef struct {
    AX_S32 minDisparity;
    AX_S32 numberOfDisparities;
    AX_S32 SADWindowSize;
    AX_S32 preFilterCap;
    AX_S32 uniquenessRatio;
    AX_S32 P1;
    AX_S32 P2;
    AX_S32 disp12MaxDiff;
} AX_DSP_SLAM_SGBM_CTL_T;

typedef struct {
    AX_DSP_SLAM_Y_IMAGE_T imageLeft;
    AX_DSP_SLAM_Y_IMAGE_T imageRight;
    AX_S16                *disp;
    AX_U8                 *dispConf;
    AX_U16                *dispPostOut;
    AX_F32                *postSysMem;
    AX_DSP_SLAM_SGBM_CTL_T sgbm_ctl;
    AX_BOOL               postControl;  // postControl = AX_FALSE , disp need not post process ,
                                        // postControl = AX_TRUE , disp need post process
} AX_DSP_SLAM_SGBM_T;

typedef struct {
    AX_U8     hamDistThreshold;     // Maximum Hamming Distance to classify as a matching pair
    AX_U16    ROI_X;                // Maximum X difference allowed between matched pairs
    AX_U16    ROI_Y;                // Maximum Y difference allowed between matched pairs
    AX_U16    angleDiffThreshold;   // Maximum ORB angle diff. allowed between matched pairs
    AX_U8     levelDiffThreshold;   // Maximum layer (layer) diff. allowed between matched pairs
}AX_DSP_SLAM_HAMMING_CTL_T;

typedef struct {
    AX_DSP_SLAM_ORB_KEYPOINTS_T         *pKeyPointsImage1;      // Input: Image1 Key-points details
    AX_DSP_SLAM_ORB_KEYPOINTS_T         *pKeyPointsImage2;      // Input: Image2 Key-points details
    AX_DSP_SLAM_HAMMING_CTL_T           *pHammingThresholds;    // Input: Hamming distance, R.O.I., thresholds
    AX_DSP_SLAM_ORB_MATCHPOINTS_TABLE_T *pMatchPointsTable;     // Output: Match points table
    AX_U16                              *pMatchedIndices0;      // Output: Image 1 Matched Indices
    AX_U16                              *pMatchedIndices1;      // Output: Image 2 Matched Indices
    AX_F32                              *pMatchedDepth0;        // Output: Image 1 Matched Depth
    AX_F32                              *pMatchedDepth1;        // Output: Image 2 Matched Depth
    AX_S32                              sensorType;             // Input: Sesnsor type
}AX_DSP_SLAM_ORB_FORWARD_KEYPOINT_MATCHING_T;

typedef struct {
    AX_DSP_SLAM_ORB_KEYPOINTS_T         *pKeyPointsImage1;      // Input: Image1 Key-points details
    AX_DSP_SLAM_HAMMING_CTL_T           *pHammingThresholds;    // Input: Hamming distance, R.O.I., thresholds
    AX_DSP_SLAM_ORB_MATCHPOINTS_TABLE_T *pMatchPointsTable;     // Input/Output: Match points table
    AX_U16                              *pMatchedIndices0;      // Input/Output: Image 1 Matched Indices
    AX_U16                              *pMatchedIndices1;      // Input/Output: Image 2 Matched Indices
    AX_F32                              *pMatchedDepth0;        // Input/Output: Image 1 Matched Depth
    AX_F32                              *pMatchedDepth1;        // Input/Output: Image 2 Matched Depth
    AX_S32                              sensorType;             // Input:  Sesnsor type
}AX_DSP_SLAM_REVERSR_KEYPOINT_MATCHING_T;

typedef struct
{
    AX_U16  nIterations;            // Input: Number of RANSAC iterations
    AX_F32  inlierErrorThreshold;   // Input: RANSAC inlier/outlier threshold
    AX_F32  triangulationSigma2;    // Input: Triangulation threshold
    AX_F32  triangulationZThreshold;// Input: Triangulation maximum Z threshold
}AX_DSP_SLAM_ESTIMATEPOSE_CTL_T;

typedef struct
{
    AX_F32  *pX;            // Pointer to triangulated 3D point X[nPoints]
    AX_F32  *pY;            // Pointer to triangulated 3D point Y[nPoints]
    AX_F32  *pZ;            // Pointer to triangulated 3D point Z[nPoints]
    AX_U16  nPoints;        // Number of triangulated points
}AX_DSP_SLAM_TRIANGULATED_POINTS_T;

typedef struct
{
    AX_DSP_SLAM_MATCHED_PAIRS_T         *pMatchedPairs;         // Input/Output: pointer to Matched (normalized) pairs
    AX_DSP_SLAM_ESTIMATEPOSE_CTL_T      *pEstimatePose_CTL;     // Input: nIterations
    AX_DSP_SLAM_TRIANGULATED_POINTS_T   *pTriangulatedPoints;   // Output: pointer to triangulated points
    AX_F32                              *pose;                  // Output: Pointer to pose[12] output
    AX_U8                               *pScratch;              // system memory scratch buffer of size
                                                                // (44 * NMATCHPAIRS + (8) * RANSAC_ITERATIONS + 1248) * 4 bytes
}AX_DSP_SLAM_ESTIMATEPOSE_T;

AX_S32 AX_DSP_SLAM_Merge(AX_S32 dsp_id,AX_DSP_SLAM_MERGE_PARAM_T *param);
AX_S32 AX_DSP_SLAM_ComputeParticles(AX_S32 dsp_id,AX_DSP_SLAM_COMPUTE_PARTICLES_LOSS_T *particles,AX_DSP_SLAM_PART_WEI_UP_PARAM_T *param,AX_F32 *pweight);
AX_S32 AX_DSP_SLAM_NormalizePoints(AX_S32 dsp_id,AX_DSP_SLAM_NORMALIZE_POINTS_T *param);
AX_S32 AX_DSP_SLAM_SparseBundleAdjustment(AX_S32 dsp_id,AX_DSP_SLAM_BUNDLE_ADJUSTMENT_N_FRAMET_T *param);
AX_S32 AX_DSP_SLAM_DepthFromStereo(AX_S32 dsp_id,AX_DSP_SLAM_DEPTH_FROM_STEREO_T *param);
AX_S32 AX_DSP_SLAM_EliminateMultipleMatches(AX_S32 dsp_id,AX_DSP_SLAM_ELIMINATE_MULTIPLEMATCHES_T *param);
AX_S32 AX_DSP_SLAM_TriangulateNewMapPoints(AX_S32 dsp_id,AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_T *param);
AX_S32 AX_DSP_SLAM_TriangulateNewMapPointsUsingDepth(AX_S32 dsp_id,AX_DSP_SLAM_TRIANGULATE_NEWMAP_POINTS_USING_DEPTH_T *param);
AX_S32 AX_DSP_SLAM_FindHomography(AX_S32 dsp_id,AX_DSP_SLAM_FIND_HOMOGRAPHY_T *param);
AX_S32 AX_DSP_SLAM_PoseOptimization(AX_S32 dsp_id,AX_DSP_SLAM_POSE_OPTIMIZATION_T *param);
AX_S32 AX_DSP_SLAM_SparseBundleAdjustment2Frames(AX_S32 dsp_id,AX_DSP_SLAM_SPASEBUNDLEADJUSTMENT_2_FRAMES_T *param);
AX_S32 AX_DSP_SLAM_SearchByProjection(AX_S32 dsp_id,AX_DSP_SLAM_SEARCH_BY_PROJECTIONT_T *param);
AX_S32 AX_DSP_SLAM_SearchByProjectionNeighbouringKeyFrames(AX_S32 dsp_id,AX_DSP_SLAM_SEARCH_BY_PROJECTIONT_NEIGHBOURING_KEYFRAMES_T *param);
AX_S32 AX_DSP_SLAM_FAST9DetectORBCompute(AX_S32 dsp_id,AX_DSP_SLAM_FAST9_DETECT_ORB_COMPUTE_T *param);
AX_S32 AX_DSP_SLAM_Convert2DdepthTo1D(AX_S32 dsp_id,AX_DSP_SLAM_CONVERT_2D_DEPTH_TO_1D_T *param);
AX_S32 AX_DSP_SLAM_SgbmDisparityCompute(AX_S32 dsp_id,AX_DSP_SLAM_SGBM_T *param);
AX_S32 AX_DSP_SLAM_SgbmFastGlobalSmoother(AX_S32 dsp_id,AX_DSP_SLAM_SGBM_T *param);
AX_S32 AX_DSP_SLAM_ORBforwardKeypointMatching(AX_S32 dsp_id,AX_DSP_SLAM_ORB_FORWARD_KEYPOINT_MATCHING_T *param);
AX_S32 AX_DSP_SLAM_ORBreverseKeypointMatching(AX_S32 dsp_id,AX_DSP_SLAM_REVERSR_KEYPOINT_MATCHING_T *param);
AX_S32 AX_DSP_SLAM_EstimatePose(AX_S32 dsp_id,AX_DSP_SLAM_ESTIMATEPOSE_T *param);
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif/* End of #ifdef __cplusplus */
#endif/*_AX_SVP_DSP_SLAM_H_*/

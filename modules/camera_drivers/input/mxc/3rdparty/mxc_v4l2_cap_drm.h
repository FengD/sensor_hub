//
// Created by lx on 20-3-6.
//

#ifndef MODULES_CAMERA_DRIVERS_INPUT_MXC_MXC_V4L2_CAP_DRM_H_
#define MODULES_CAMERA_DRIVERS_INPUT_MXC_MXC_V4L2_CAP_DRM_H_

#include <asm/types.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <malloc.h>
#include <pthread.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>

#include <glog/logging.h>

#include <opencv2/opencv.hpp>


extern int open_camera_N;

// extern cv::Mat g_ims[3];
// extern int g_channel_dex[3];

typedef unsigned char ImgType;

// extern ImgType *g_im;
// extern ImgType *g_im1;
// extern ImgType *g_im2;

// extern cv::Mat g_im;
// extern cv::Mat g_im1;
// extern cv::Mat g_im2;
static bool saveImage = false;

static bool angle_ = true;
static bool bridge_ = true;

static bool leftCAM = true;
static bool rightCAM = true;

static bool cameraSetup = false;

static bool quitAngle = false;
static bool quitBridge = false;
extern int g_channel_dex;
extern int g_channel1_dex;
extern int g_channel2_dex;
// #ifdef __cplusplus
// extern "C"{
void mxc_v4l2_cap_init(int ch_id);
int mxc_v4l2_cap_run(int ch_id, unsigned char ** yuyv_ptr);
void mxc_v4l2_cap_deinit(int status, int ch_id);

    // void read_mxc_v4l2_cap(int camera_mode);
// }

// #endif //__cplusplus
void* img_read_thread(void * g_im_data);
void* img_process_thread(void * g_im_data);

#endif  //  MODULES_CAMERA_DRIVERS_INPUT_MXC_MXC_V4L2_CAP_DRM_H_

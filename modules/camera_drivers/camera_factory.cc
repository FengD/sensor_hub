// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera_factory

#include "camera_drivers/camera.h"
#include "camera_drivers/encoder/cv_encoder/cv_encoder.h"
#include "camera_drivers/encoder/turbo_encoder/turbo_encoder.h"
#include "camera_drivers/undistortion/cv/cv_undistortion.h"
#include "camera_drivers/input/testing/testing.h"
#ifdef WITH_ROS2
#include "camera_drivers/input/ros2/ros2_input.h"
#endif

#ifdef WITH_A6
#include "camera_drivers/input/mxc/mxc.h"
#endif

#ifdef WITH_IPC
#include "camera_drivers/input/gstcamera/gstcamera.h"
#endif

#ifdef WITH_TDA4
#include "camera_drivers/encoder/h264_encoder/h264_encoder.h"
#include "camera_drivers/input/tiovx_camera/tiovx_camera.h"
#endif

namespace sensor {
namespace hub {

REGISTER_ENCODER(CvEncoder);
REGISTER_ENCODER(TurboEncoder);

REGISTER_UNDISTORTION(CvUndistortion);
REGISTER_UNDISTORTION(Undistortion);

REGISTER_CAMERA_INPUT(TestingCamera);
#ifdef WITH_ROS2
REGISTER_CAMERA_INPUT(Ros2Input);
#endif

#ifdef WITH_IPC
REGISTER_CAMERA_INPUT(GstCamera);
#endif

#ifdef WITH_TDA4
REGISTER_ENCODER(H264Encoder);
REGISTER_CAMERA_INPUT(TiovxCamera);
#endif

#ifdef WITH_A6
REGISTER_CAMERA_INPUT(MxcCamera);
#endif

}  // namespace hub
}  // namespace sensor


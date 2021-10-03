// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera_factory

#include "camera_drivers/camera.h"
// #include "camera/encoder/cv_encoder/cv_encoder.h" 
// #include "camera/encoder/turbo_encoder/turbo_encoder.h"
// #include "camera/undistortion/cv/cv_undistortion.h"

// #ifdef PLATFORM_XGO 
// #ifdef PYLON_FOUND 
// #include "camera/input/basler/basler.h" 
// #endif // PYLON_FOUND 
// #ifdef GSTREAMER FOUND
// #include "camera/input/gstreamer/gst.h" 
// #endif // GSTREAMER FOUND 
// #include "camera/input/hk/hikvision.h" 
// #include "camera/decoder/nv_decoder/nv_decoder.h" 
// #else 
// #include "camera/input/fpga/fpga.h" 
// #endif


namespace crdc {
namespace airi {
// REGISTER_ENCODER (CvEncoder);
// REGISTER_ENCODER (TurboEncoder);
// REGISTER_UNDISTORTION (CvUndistortion);
// REGISTER_UNDISTORTION (Undistortion);

// #ifdef PLATFORM_XGO 
// REGISTER_DECODER (NvidiaDecoder);
// #ifdef GSTREAMER FOUND
// REGISTER_CAMERA_INPUT (GstCamera);
// #endif // GSTREAMER_FOUND
// REGISTER_CAMERA_INPUT (Hikvision Camera);
// #ifdef PYLON_FOUND
// REGISTER_CAMERA_INPUT (BaslerCamera);
// #endif // PYLON_FOUND
// #else
// REGISTER_CAMERA_INPUT (FPGA Camera);
// #endif

}  // namespace airi 
}  // namespace crdc


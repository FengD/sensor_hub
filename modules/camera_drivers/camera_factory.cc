// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera_factory

#include "camera_drivers/camera.h"
#include "camera_drivers/encoder/cv_encoder/cv_encoder.h"
#include "camera_drivers/encoder/turbo_encoder/turbo_encoder.h"
#include "camera_drivers/undistortion/cv/cv_undistortion.h"
#include "camera_drivers/input/sensing/sensing.h"
#include "camera_drivers/input/maxim/maxim.h"
#include "camera_drivers/input/testing/testing.h"

namespace crdc {
namespace airi {

REGISTER_ENCODER(CvEncoder);
REGISTER_ENCODER(TurboEncoder);

REGISTER_UNDISTORTION(CvUndistortion);
REGISTER_UNDISTORTION(Undistortion);

REGISTER_CAMERA_INPUT(SensingCamera);
REGISTER_CAMERA_INPUT(MaximCamera);
REGISTER_CAMERA_INPUT(TestingCamera);

}  // namespace airi
}  // namespace crdc


// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input sensing

#pragma once

#include <memory>
#include "common/common.h"
#include "camera_drivers/input/input.h"

namespace crdc {
namespace airi {

class SensingCamera : public CameraInput {
 public:
  SensingCamera() = default;
  virtual ~SensingCamera();
  bool camera_init() override;
  bool camera_start() override;
  bool camera_stop() override;
  
};

}  // namespace airi
}  // namespace crdc
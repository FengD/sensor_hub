// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input maxim

#pragma once

#include <memory>
#include "common/common.h"
#include "camera_drivers/input/input.h"

namespace sensor {
namespace hub {

class TestingCamera : public CameraInput {
 public:
  TestingCamera() = default;
  virtual ~TestingCamera();
  bool camera_init() override;
  bool camera_start() override;
  bool camera_stop() override;

 private:
  void get_data();
};

}  // namespace hub
}  // namespace sensor

// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input maxim

#pragma once

#include <memory>
#include "common/common.h"
#include "camera_drivers/input/input.h"

namespace crdc {
namespace airi {

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

}  // namespace airi
}  // namespace crdc
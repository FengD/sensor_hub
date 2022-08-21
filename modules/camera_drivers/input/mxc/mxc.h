// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Ruopeng ZHANG
// Description: camera input mxc

#pragma once

#include <memory>
#include "common/common.h"
#include "camera_drivers/input/input.h"

namespace crdc {
namespace airi {

class MxcCamera : public CameraInput {
 public:
  MxcCamera() = default;
  virtual ~MxcCamera();
  bool camera_init() override;
  bool camera_start() override;
  bool camera_stop() override;

 private:
  int ch_id_;
  int frame_id_map();
  void get_data();
};

}  // namespace airi
}  // namespace crdc

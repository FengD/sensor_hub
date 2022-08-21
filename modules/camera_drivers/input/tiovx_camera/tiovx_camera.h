// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: LINLIN WANG
// Description: TI OpenVX based camera input
// Contributor: shichong.wang

#pragma once

#include <memory>
#include "common/common.h"
#include "camera_drivers/input/input.h"

namespace crdc {
namespace airi {

class TiovxCamera : public CameraInput {
 public:
  TiovxCamera() = default;
  virtual ~TiovxCamera();
  bool camera_init() override;
  bool camera_start() override;
  bool camera_stop() override;

 private:
  int flag_data;
  int flag_init;
  int flag_start;
  int count;
  uint32_t img_length;
  uint32_t num_camera;
  uint32_t data_size;
  unsigned char* img_ptr;
  void get_data();
};

}  // namespace airi
}  // namespace crdc

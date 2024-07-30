// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: TI OpenVX based camera input

#pragma once

#include <tiovx_camera_lib.h>
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
  bool camera_start(std::shared_ptr<CamIntrinsicParam>& camera_intrinsic) override;
  bool camera_stop() override;

 private:
  int flag_data;
  int flag_init;
  int flag_start;
  int count;
  uint32_t img_length;
  uint32_t num_camera;
  tiovx_camera_param tiovx_camera_prm;
  uint32_t data_size;
  unsigned char* img_ptr;
  void receive_camera_intrinsic(std::shared_ptr<CamIntrinsicParam>& camera_intrinsic);
  void get_data();
};

}  // namespace airi
}  // namespace crdc

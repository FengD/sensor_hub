// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input gstcamera

#pragma once
#include <memory>
#include <string>
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
#include "common/common.h"
#include "camera_drivers/input/input.h"

namespace crdc {
namespace airi {

class GstCamera : public CameraInput {
 public:
  GstCamera() = default;
  virtual ~GstCamera();
  bool camera_init() override;
  bool camera_start() override;
  bool camera_stop() override;
 private:
  // General gstreamer configuration
  std::string gsconfig_;
  // Gstreamer structures
  GstElement *pipeline_;
  GstElement *sink_;
  // Camera configuration
  int width_, height_;
  void get_data();
};

}  // namespace airi
}  // namespace crdc

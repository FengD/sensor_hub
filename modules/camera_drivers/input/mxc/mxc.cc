// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Ruopeng ZHANG
// Description: camera input mxc

#include <iostream>
#include <string>
#include "camera_drivers/input/mxc/mxc.h"
#include "camera_drivers/input/mxc/3rdparty/mxc_v4l2_cap_drm.h"

namespace crdc {
namespace airi {

MxcCamera::~MxcCamera() {
  if (is_running_.load()) {
    stop();
  }
}

bool MxcCamera::camera_init() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input init.";
  ch_id_ = config_.mxc_config().channel_id();
  mxc_v4l2_cap_init(ch_id_);
  return true;
}

bool MxcCamera::camera_start() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input start.";
  auto data_thread = new std::thread(&MxcCamera::get_data, this);
  return true;
}

bool MxcCamera::camera_stop() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input stopped.";
  mxc_v4l2_cap_deinit(0, ch_id_);
  return true;
}

void MxcCamera::get_data() {
  unsigned char *yuyv_ptr;
  int status = 0;
  while (status == 0) {
    auto start = get_now_microsecond();
    status = mxc_v4l2_cap_run(ch_id_, &yuyv_ptr);
    std::shared_ptr<CameraRawData> raw_data = get_raw_data(1, get_now_microsecond(), yuyv_ptr);
    raw_data->data_type = "Y";
    if (yuyv_ptr == nullptr) {
      LOG(WARNING) << "[" << config_.frame_id() << "] get raw data failed.";
    } else {
      raw_data->data_size = config_.width() * config_.height();
      raw_data_queue_.enqueue(raw_data);
      LOG(INFO) << "[" << config_.frame_id() << "] get raw data success.";
    }
    matching_fps_by_sleep(start, get_now_microsecond());
  }
  if (status == -1) {
    mxc_v4l2_cap_deinit(status, ch_id_);
    LOG(INFO) << "[" << config_.frame_id() << "] get raw data failed. camera input stopped.";
  }
}

}  // namespace airi
}  // namespace crdc

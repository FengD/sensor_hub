// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input maxim

#include "camera_drivers/input/testing/testing.h"

namespace crdc {
namespace airi {

TestingCamera::~TestingCamera() {
  if (is_running_.load()) {
    stop();
  }
}

bool TestingCamera::camera_init() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input init.";

  if (!config_.has_testing_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] without testing_config.";
    return false;
  }
  return true;
}

bool TestingCamera::camera_start() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input start.";
  auto data_thread = new std::thread(&TestingCamera::get_data, this);
  return true;
}

bool TestingCamera::camera_stop() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input stopped.";
  return true;
}

void TestingCamera::get_data() {
  unsigned char data[10] = "1212321";
  while (1) {
    auto start = get_now_microsecond();
    std::shared_ptr<CameraRawData> raw_data = get_raw_data(1, start, data);
    raw_data->data_type = "UYUV";
    raw_data->data_size = 10;
    raw_data_queue_.enqueue(raw_data);
    matching_fps_by_sleep(start, get_now_microsecond());
  }
}

}  // namespace airi
}  // namespace crdc

// Copyright (C) 2020 Hirain Technologies
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
  while (1) {
    std::shared_ptr<CameraRawData> raw_data = raw_pool_->GetObject();
    if (raw_data == nullptr) {
      raw_data = std::make_shared<CameraRawData>();
      raw_data->image_ = cv::Mat(config_.height(), config_.width(), CV_8UC3);
    }
    raw_data->exposure_time_ = 1;
    raw_data->utime_ = get_now_microsecond();
    raw_data->data_type = "UYUV";
    raw_data_queue_.enqueue(raw_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / config_.fps()));
  }
}

}  // namespace airi
}  // namespace crdc

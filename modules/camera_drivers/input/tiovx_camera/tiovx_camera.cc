// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: LINLIN WANG
// Description: TI OpenVX based camera input
// Contributor: shichong.wang

#include <tiovx_camera_lib.h>
#include "camera_drivers/input/tiovx_camera/tiovx_camera.h"

#define VX_FAILURE 1
#define VX_SUCCESS 0
#define STOP_TASK 1
#define RUN_TASK 0
#define NV12_CHANNEL_SIZE 1.5

namespace crdc {
namespace airi {

TiovxCamera::~TiovxCamera() {
  if (is_running_.load()) {
    stop();
  }
}

bool TiovxCamera::camera_init() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input init.";

  if (!config_.has_tiovx_camera_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] without tiovx_camera_config.";
    return false;
  }

  flag_data = RUN_TASK;
  count = 0;
  img_length = config_.height() * config_.width() * NV12_CHANNEL_SIZE;
  num_camera = config_.tiovx_camera_config().num_camera();
  data_size = img_length * num_camera;
  img_ptr = (unsigned char *)malloc(data_size * sizeof(unsigned char));
  LOG(INFO) << "[" << config_.frame_id() << "] camera data_size: "
            << data_size;
  LOG(INFO) << "[" << config_.frame_id() << "] camera num_camera: "
            << config_.tiovx_camera_config().num_camera();

  flag_init = VX_SUCCESS;
  flag_init = init_tiovx_camera(num_camera);
  if (flag_init == VX_FAILURE) {
    LOG(ERROR) << "[" << config_.frame_id() << "] camera input init failed.";
    return false;
  } else {
    LOG(INFO) << "[" << config_.frame_id() << "] camera input init success.";
  }
  return true;
}

bool TiovxCamera::camera_start() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input start.";
  flag_start = VX_SUCCESS;
  flag_start = start_tiovx_camera();
  if (flag_start == VX_FAILURE) {
    LOG(ERROR) << "[" << config_.frame_id() << "] camera input start failed.";
    return false;
  } else {
    LOG(INFO) << "[" << config_.frame_id() << "] camera input start success.";
    auto data_thread = new std::thread(&TiovxCamera::get_data, this);
  }
  return true;
}

bool TiovxCamera::camera_stop() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input stopped.";
  stop_tiovx_camera();
  if (img_ptr != nullptr) {
    free(img_ptr);
  }
  return true;
}

void TiovxCamera::get_data() {
  while (1) {
    auto start = get_now_microsecond();
    flag_data = get_tiovx_camera_frame(&img_ptr, data_size, num_camera, count);
    if (flag_data == STOP_TASK) {
        LOG(ERROR) << "[" << config_.frame_id() << "] get_tiovx_camera_frame to be set stop.";
        continue;
    }
    count++;
    std::shared_ptr<CameraRawData> raw_data = get_raw_data(0, get_now_microsecond(), img_ptr);
    raw_data->data_type = "NV12";
    raw_data->data_size = data_size;
    raw_data->image_number = num_camera;
    raw_data_queue_.enqueue(raw_data);
    matching_fps_by_sleep(start, get_now_microsecond());
  }
}

}  // namespace airi
}  // namespace crdc

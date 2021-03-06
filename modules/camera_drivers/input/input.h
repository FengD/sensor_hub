// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input

#pragma once

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "common/common.h"
#include "camera_drivers/proto/camera_config.pb.h"

namespace crdc {
namespace airi {

struct CameraRawData {
  float exposure_time_ = 0;
  uint64_t utime_ = 0;
  cv::Mat image_;
  std::string data_type;
  CameraRawData() {}
};

class CameraInput {
 public:
  CameraInput() = default;
  virtual ~CameraInput() = default;

  /**
   * @brief init the input
   * @param the input config
   * @return status
   */
  bool init(const CameraInputConfig& config);

  /**
   * @brief start
   * @return status
   */
  bool start();

  /**
   * @brief stop
   * @return status
   */
  bool stop();

  /**
   * @brief This function is used to get the camera data
   * @param the raw data
   * @return status
   */
  virtual int32_t get_camera_data(std::shared_ptr<const CameraRawData>* data) {
    if (raw_data_queue_.wait_for_dequeue(data)) {
      return DeviceStatus::SUCCESS;
    } else {
      return DeviceStatus::CAMERA_TIMEOUT;
    }
  }

  /**
   * @brief Used to release some tmp camera data
   */
  virtual void release_camera_data() {}

  /**
   * @brief This function is used to update the camera data
   * @param the raw data
   * @return status
   */
  virtual bool put_camera_data(std::shared_ptr<const CameraRawData>* data) {
    return true;
  }

  std::string get_name() const {
    return name_;
  }

 protected:
  /**
   * @brief Init the camera. It needs to be redefined for each subclass.
   * @return status
   */
  virtual bool camera_init() { return false; }

  /**
   * @brief Start the camera. It needs to be redefined for each subclass.
   * @return status
   */
  virtual bool camera_start() { return true; }

  /**
   * @brief Stop the camera. It needs to be redefined for each subclass.
   * @return status
   */
  virtual bool camera_stop() { return false; }
  bool init_pool();

  std::shared_ptr<CameraRawData> get_raw_data(float exposure_time,
                                              uint64_t utime,
                                              unsigned char* data);
  common::ThreadSafeQueue<std::shared_ptr<const CameraRawData>> raw_data_queue_;
  std::shared_ptr<common::CCObjectPool<CameraRawData>> raw_pool_ = nullptr;
  CameraInputConfig config_;
  std::atomic<bool> is_running_;
  std::string name_;
};

REGISTER_COMPONENT(CameraInput);
#define REGISTER_CAMERA_INPUT(name) REGISTER_CLASS(CameraInput, name)

}  // namespace airi
}  // namespace crdc

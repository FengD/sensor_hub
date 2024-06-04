// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "common/common.h"
#include "camera_drivers/proto/camera_config.pb.h"

namespace crdc {
namespace airi {

struct CameraRawData {
  float exposure_time_ = 0;
  uint64_t utime_ = 0;
  uint32_t image_number = 1;
  cv::Mat image_;
  std::string data_type;
  uint32_t data_size;
  CameraRawData() {}
};

typedef struct {
  float fx;
  float fy;
  float cx;
  float cy;
  std::vector<float> distortion_params;
} CamIntrinsicParam;

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
  bool start(std::shared_ptr<CamIntrinsicParam>& camera_intrinsic);
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
    if (raw_data_queue_.wait_for_dequeue(data, 100000)) {
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

  /**
   * @brief This function is only used for db3 input
   * @param the topic name
   * @param the raw data
   * @return status
   */
  virtual bool get_topic_name(std::string* topic_name,
                              std::shared_ptr<const CameraRawData>& raw_data) {
    return false;
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
  virtual bool camera_start(std::shared_ptr<CamIntrinsicParam>& camera_intrinsic) { return true; }

  /**
   * @brief Stop the camera. It needs to be redefined for each subclass.
   * @return status
   */
  virtual bool camera_stop() { return false; }

  bool init_pool();

  std::shared_ptr<CameraRawData> get_raw_data(float exposure_time,
                                              uint64_t utime,
                                              unsigned char* data);

  /**
   * @brief sleep a period for matching the fps configed.
   * @param period start time [us] [in]
   * @param period end time [us] [in]
   */
  void matching_fps_by_sleep(const uint64_t& start, const uint64_t& end);
  common::ThreadSafeQueue<std::shared_ptr<const CameraRawData>> raw_data_queue_;
  std::shared_ptr<common::CCObjectPool<CameraRawData>> raw_pool_ = nullptr;
  CameraInputConfig config_;
  std::atomic<bool> is_running_;
  std::string name_;
  int32_t period_;
};

REGISTER_COMPONENT(CameraInput);
#define REGISTER_CAMERA_INPUT(name) REGISTER_CLASS(CameraInput, name)

}  // namespace airi
}  // namespace crdc

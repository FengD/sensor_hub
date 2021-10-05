// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera

#pragma once

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include "common/common.h"
#include "camera_drivers/input/input.h"
#include "camera_drivers/encoder/encoder.h"
#include "camera_drivers/undistortion/undistortion.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "camera_drivers/proto/camera_component_config.pb.h"
#include "camera_drivers/proto/encoder_config.pb.h"
#include "cyber/sensor_proto/image.pb.h"

namespace crdc {
namespace airi {

class Camera : public common::Thread {
 public:
  explicit Camera(const CameraComponentConfig& config);
  virtual ~Camera() = default;
  std::string get_name() const {
    return camera_name_;
  }
  void stop();

 private:
  /**
   * @brief The run function execute the whole process of the camera loop process.
   */
  void run() override;

  /**
   * @brief Load the sensor parameters for example the undistortion array, the distance correct array.
   * @param The output sensor configuration.
   */
  void load_sensor_config(CameraSensorConfig& sensor_config);

  /**
   * @brief Init the camera config by camera config to sensor config.
   * @param The input sensor config 
   * @param The output config
   */
  void init_camera_config(const CameraSensorConfig& sensor_config, CameraConfig& config);

  /**
   * @brief Init the encoder if defined.
   */
  bool init_encoder();

  /**
   * @brief Init the input if defined.
   */
  bool init_input();

  /**
   * @brief Init the  undistortion if needed.
   */
  bool init_undistortion();

  /**
   * @brief Init the output proto image message.
   */
  void init_proto_image();

  std::string camera_name_;
  bool stop_;
  CameraComponentConfig config_;
  CameraConfig camera_config_;

  std::shared_ptr<CameraInput> input_;
  std::shared_ptr<Undistortion> undistortion_;
  std::shared_ptr<Encoder> encoder_;

  cv::Mat image_undistorted_;
  std::shared_ptr<Image> proto_image_;
  std::shared_ptr<Image> proto_encode_image_;

  uint32_t status_cnt_;
  uint32_t camera_fail_cnt_;
  uint32_t sensor_position_id_;
};
}  // namespace airi
}  // namespace crdc

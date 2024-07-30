// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING, Zilou Cao
// Description: camera

#pragma once
#ifdef WITH_TEST
#include <gtest/gtest.h>
#endif
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
#include "module_diagnose/module_diagnose.h"
#include "camera_drivers/camera_diagnose/camera_diagnose.h"
#include "camera_drivers/proto/camera_diagnose_config.pb.h"
#ifndef WITH_ROS2
#include "cyber/sensor_proto/image.pb.h"
#else
#include "sensor_msgs/msg/image.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#ifdef WITH_TDA4
#include "camera_drivers/camera_calibration/calibrate.h"
#endif
using Level = diagnostic_msgs::msg::DiagnosticStatus;
#define MODULE "CameraDriver"
#endif

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

#ifdef WITH_ROS2
  /**
   * @brief Init the output msg image message.
   */
  void init_msg_image();
#else
  /**
   * @brief Init the output proto image message.
   */
  void init_proto_image();
#endif

  /**
   * @brief dill raw image.
   * @param the camera raw data.
   * @param the offset of the image data, if multipule image contained.
   * @param the size of one image.
   */
  void dill_raw_image(const std::shared_ptr<const CameraRawData>& raw_data,
                      const uint32_t& offset, const uint32_t& one_image_size);

  /**
   * @brief dill encode image.
   * @param the camera raw data.
   * @param the offset of the image data, if multipule image contained.
   * @return if the treatement is correct or not.
   */
  bool dill_encode_image(const std::shared_ptr<const CameraRawData>& raw_data,
                         const uint32_t& offset);

  /**
   * @brief dill mask image.
   * @param the camera mask data.
   * @param the offset of the image data, if multipule image contained.
   * @param the size of one image.
   */
  void dill_mask_image(const std::shared_ptr<const CameraRawData>& raw_data,
                       const uint32_t& offset, const uint32_t& one_image_size);

  /**
   * @brief camera start.
   */
  void camera_start();

  /**
   * @brief send raw data message.
   */
  void send_raw_data(std::shared_ptr<const CameraRawData>& raw_data,
                                         uint32_t one_image_size, int32_t ret);

  /**
   * @brief send mask message.
   */
  void send_mask_topic(std::shared_ptr<const CameraRawData>& raw_data,
                       uint32_t one_image_size, std::string& topic_name);

  /**
   * @brief Send diagnose message
   */
#ifdef WITH_ROS2
  void send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t level,
                                const std::string& custom_desc, const std::string& context);
#ifdef WITH_TDA4
  bool calibrate(std::shared_ptr<const CameraRawData>& raw_data,
                 std::shared_ptr<CamIntrinsicParam>& camera_intrinsic);
#endif
#else
  void send_diagnose_input(const uint32_t& position_id,
                           const DeviceStatus& error, const Level& level,
                           const std::string& custom_desc, const std::string& context);
#endif

  /**
   * @brief The function is used to diagnose images through different logic
   * @param the camera raw data
   * @param the address offset
   * @param the diagnostics of camera data
  */
  void diagnose_images(const std::shared_ptr<const CameraRawData>& raw_data,
                      const uint32_t& offset, const uint32_t& height,
                      const uint32_t& width, const int32_t& ret);

  // CameraComponent component_config_;
  std::string camera_name_;
  bool stop_;
  bool validate_calib_;
  bool diagnose_executed_;
  CameraComponentConfig config_;
  CameraConfig camera_config_;
  CameraDiagnoseConfig diagnose_config_;
  DeviceStatus device_status_;
  std::unordered_map<int, int> diagnose_results_;
  std::uint32_t image_count_;
  std::shared_ptr<CameraDiagnoser> camera_diagnoser_;
  std::shared_ptr<CameraInput> input_;
  std::shared_ptr<Undistortion> undistortion_;
  std::shared_ptr<CamIntrinsicParam> cam_intrinsic_param_;
  std::shared_ptr<Encoder> encoder_;

  bool flag_status_;
  cv::Mat image_undistorted_;
#ifdef WITH_ROS2
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CompressedImage encode_image_msg_;
  sensor_msgs::msg::CompressedImage encode_mask_image_;
#ifdef WITH_TDA4
  std::shared_ptr<Calibrate> calibrate_process_;
#endif
#else
  std::shared_ptr<Image2> proto_image_;
  std::shared_ptr<Image2> proto_encode_image_;
#endif

  uint32_t image_seq_;
  uint32_t encode_image_seq_;
  uint32_t sensor_position_id_;
  uint32_t diagnose_begin_count_;
  uint32_t diagnose_end_count_;
  cv::Mat before_encoder_rgb_;
  std::vector<DiagnoseInput> diagnose_inputs_;

  uint32_t image_fps_index_;
  uint32_t encode_image_downsampling_each_image_frame_;
#ifdef WITH_ROS2
  void write_data_intermittently(uint32_t last_data_fps_index,
                                 uint32_t downsampling_each_frame,
                                 std::string channel_name,
                                 sensor_msgs::msg::CompressedImage& data);
#endif

#ifdef WITH_TEST
  FRIEND_TEST(CameraDriverTest, init_encoder_test);
  FRIEND_TEST(CameraDriverTest, init_input_test);
  FRIEND_TEST(CameraDriverTest, init_undistortion_test);
  FRIEND_TEST(CameraDriverTest, dill_encode_image_test);
#endif
};
}  // namespace airi
}  // namespace crdc

/**
* @copyright Copyright (c) 2023 Beijing Jingwei Hirain Technologies Co., Inc. All rights reserved
* @file camera_calibration.h
* @brief Intergrated camera calibration function to ros2 service server
* @author yangyang.liu1
* @date 2023-06-12
* @license Modified BSD Software License Agreement
*/

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <filesystem>
#include "common/util.h"
#include "camera_tools/calibration/extrinsic.h"
#include "camera_drivers/input/input.h"
#include "tools/AF_server/calibrate_server.h"
#include "tools/service_server/calib_utils.h"
#include "tools/service_server/data_encode_decode.h"

namespace crdc {
namespace airi {
class CameraCalibrate {
 public:
  CameraCalibrate() = default;
  bool init(const std::string &config_path, const int img_width, const int img_height);
  virtual ~CameraCalibrate() = default;
  int process(std::vector<std::shared_ptr<const CameraRawData>> &images_raw, std::string &path,
              std::shared_ptr<DbusSendData> &dbus_send_data,
              std::shared_ptr<CamIntrinsicParam> &cam_intrinsic_param);

  std::string get_name() const { return "CameraCalibrate";}
  void init_instrinsic_distortion_coeffs(
      std::shared_ptr<CamIntrinsicParam> &camera_intrinsic_param);
  void generate_real_distance_lut();
  bool cal_world_coord(const cv::Mat &rvec, const cv::Mat &tvec);
  std::string get_params_file_path();

 private:
  std::string write_calib_res_yaml(const std::string &luts_path, cv::Mat &rvec, cv::Mat &tvec);
  bool convert_image(std::vector<std::shared_ptr<const CameraRawData>> &images_raw);
  void get_calib_image_points(algorithm::CameraCalibrationConfig config,
                              std::vector<cv::Point2f> &image_points);
  void read_camera_intrinsic_param(algorithm::CameraCalibrationConfig config,
                                  std::shared_ptr<CamIntrinsicParam>& camera_intrinsic_param);
  void write_params_to_prototxt(std::string params_file_path, const cv::Mat &rvec,
                                const cv::Mat &tvec);
  void struct_deep_copy(
      const std::shared_ptr<CamIntrinsicParam> &cam_intrinsic_param,
      const std::shared_ptr<DbusSendData> &dbus_send_data,
      std::shared_ptr<algorithm::CameraIntrinsicParams> &data_frame_intrinsic,
      std::shared_ptr<algorithm::CameraCalibMeasurmentData> &data_frame_calib_data);
  std::shared_ptr<PerceptionDataFrame<int>> data_frame_;
  std::shared_ptr<algorithm::CameraExtrinsic> extrinsic_calibrate_;
  algorithm::CameraCalibrationConfig extrinsic_config_;
  std::shared_ptr<DataEncoderDecoder> data_converter_;
  std::shared_ptr<CalibrationUtils> calib_utils_;
  int version_num_;
  std::string camera_position_;
  std::string res_save_path_;
  int image_width_;
  int image_height_;
  cv::Mat distortion_coeffs_;
  std::vector<cv::Point2f> vehicle_points_;
  cv::Mat intrinsic_ = cv::Mat(cv::Size(3, 3), CV_64FC1);
  std::string params_save_path_;
  std::string current_time_str_;
  std::string vehicle_number_;
  std::string product_name_;
  std::string file_save_prefix_;
};
}  // namespace airi
}  // namespace crdc

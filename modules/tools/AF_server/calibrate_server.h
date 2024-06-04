/**
 * Copyright (c) 2023 Beijing Jingwei Hirain Technologies Co., Inc. All rights reserved
 * @file calibration_status.h
 * @brief calibration status service
 * @author Jiao HE
 * @date 2023-07-05
 * @license Modified BSD Software License Agreement
 */
#pragma once

#include <string>
#include <chrono>
#include "common/common.h"

#include "tools/AF_server/generated/diagnostic_stub.h"
#include "utils/Log.h"
#include <opencv2/opencv.hpp>

namespace crdc {
namespace airi {
typedef struct {
  int calib_mode; // 0: fixed points  1: calib board  2: regerate luts
  std::vector<cv::Point3f> real_world_points_meas;
  std::vector<cv::Point3f> real_world_points_valid;
  float checkboard_dis;
  float checkboard_height;
  float area_low_threshold;
  cv::Mat rotation_vector;
  cv::Mat translation_vector;
} DbusSendData;

class CalibrateStatus : public hirain::diag::aiStub, public common::Thread {
    public:
      CalibrateStatus() {}
      void triggerCalibration(uint32_t type, const std::string& param, uint32_t& res) override;
      void getCalibrationStatus(uint32_t& state) override;
      void getCalibrationFilePath(std::string& path) override;
      void reset_caliration_state();
      void set_status(const uint32_t state);
      void set_calibrate_path(const std::string path);
      void get_request_id(int& id);
      void get_lidar_calib_mode(int &mode);
      std::shared_ptr<DbusSendData> extracted_calib_param();
      void receive_dbus_send_data(const std::string& param);
      void run();

    private:
      bool request_id_ = false;
      std::mutex mutex_;
      int calib_status = 1;
      std::string calib_file_path = "";
      int calib_request_id = 0;
      int lidar_calib_mode = 0;
      std::shared_ptr<DbusSendData> dbus_send_data_;
};
} // namespace airi
} // namespace crdc

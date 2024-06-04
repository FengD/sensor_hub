/**
* @copyright Copyright (c) 2023 Beijing Jingwei Hirain Technologies Co., Inc. All rights reserved
* @file calibrate.h
* @brief calibrate process include serveice of vehicle mode and status
* @author yangyang.liu1
* @date 2023-07-13
* @license Modified BSD Software License Agreement
*/
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "tools/AF_server/calibrate_server.h"
#include "camera_drivers/camera_calibration/camera_calibration.h"
#include "framework/IDBusProxy.h"
#include "framework/IpcConnection.h"

namespace crdc {
namespace airi {
class Calibrate {
 public:
  void init(const std::string &config_path, const int img_width, const int img_height);
  Calibrate() = default;
  virtual ~Calibrate() = default;
  /**
  * @brief process for calibration
  * @param  images_raw [in], input image raw from camera.cc
  * @return int , process status
  */
  int process(std::shared_ptr<const CameraRawData> &images_raw,
              std::shared_ptr<CamIntrinsicParam> &camera_intrinsic);
  /**
  * @brief Get the vehicle mode object
  * @return int , vehicle_mode which same with vehcilemode service
  */
  int get_vehicle_mode();
  std::string get_name() const { return "CalibrateProcess";}
  void begin();
  void set_calib_end() { calib_end_ = true; }

 private:
  hirain::sp<CalibrateStatus> status_server_;
  std::shared_ptr<CameraCalibrate> camera_calib_;
  hirain::sp<IpcConnection> connection;
  int process_state_ = 1;
  bool calib_end_ = false;
  int vehicle_mode_;
  int image_count_;
  std::string save_path_;
  std::mutex mutex_;
  std::string product_name_;
  std::vector<std::shared_ptr<const CameraRawData>> images_;
  std::shared_ptr<DbusSendData> calib_input_data_;
};
}  // namespace airi
}  // namespace crdc

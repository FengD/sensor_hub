#include <string>
#include "common/common.h"
#include "tools/AF_server/calibrate_server.h"
#include "utils/Log.h"

namespace crdc {
namespace airi {
void CalibrateStatus::triggerCalibration(uint32_t type, const std::string& param, uint32_t& res) {
  request_id_ = true;
  res = 1;
  std::lock_guard<std::mutex> lock(mutex_);
  calib_request_id = 2;
  lidar_calib_mode = type;
  receive_dbus_send_data(param);
}

void CalibrateStatus::receive_dbus_send_data(const std::string& param) {
  const char* byte_ptr = param.data();
  int calib_mode;
  dbus_send_data_ = std::make_shared<DbusSendData>();
  std::memcpy(&calib_mode, byte_ptr, sizeof(int));
  dbus_send_data_->calib_mode = calib_mode;
  byte_ptr += sizeof(int);
  dbus_send_data_->real_world_points_meas.clear();
  dbus_send_data_->real_world_points_valid.clear();
  switch (dbus_send_data_->calib_mode) {
    case 0: {
      float z;
      std::memcpy(&z, byte_ptr, sizeof(float));
      byte_ptr += sizeof(float);
      std::memcpy(&dbus_send_data_->area_low_threshold, byte_ptr, sizeof(float));
      byte_ptr += sizeof(float);
      for (int i = 0; i < 6; ++i) {
        float x, y;
        std::memcpy(&x, byte_ptr, sizeof(float));
        byte_ptr += sizeof(float);
        std::memcpy(&y, byte_ptr, sizeof(float));
        byte_ptr += sizeof(float);
        if (x != 0 && y != 0) {
          dbus_send_data_->real_world_points_meas.push_back(cv::Point3f(x, y, z));
        }
      }

      for (int j = 0; j < 2; ++j) {
        float x, y;
        std::memcpy(&x, byte_ptr, sizeof(float));
        byte_ptr += sizeof(float);
        std::memcpy(&y, byte_ptr, sizeof(float));
        byte_ptr += sizeof(float);
        if (x != 0 && y != 0) {
          dbus_send_data_->real_world_points_valid.push_back(cv::Point3f(x, y, z));
        }
      }
      break;
    }
    case 1: {
      std::memcpy(&dbus_send_data_->checkboard_dis, byte_ptr, sizeof(float));
      byte_ptr += sizeof(float);
      std::memcpy(&dbus_send_data_->checkboard_height, byte_ptr, sizeof(float));
      break;
    }
    case 2: {
      cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
      cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
      dbus_send_data_->rotation_vector = cv::Mat::zeros(1, 3, CV_64FC1);
      dbus_send_data_->translation_vector = cv::Mat::zeros(1, 3, CV_64FC1);
      for (int i = 0; i < 3; ++i) {
        float ri;
        std::memcpy(&ri, byte_ptr, sizeof(float));
        rvec.at<double>(0, i) = static_cast<double>(ri);
        byte_ptr += sizeof(float);
      }
      for (int j = 0; j < 3; ++j) {
        float tj;
        std::memcpy(&tj, byte_ptr, sizeof(float));
        tvec.at<double>(0, j) = static_cast<double>(tj);
        byte_ptr += sizeof(float);
      }
      dbus_send_data_->rotation_vector = rvec;
      dbus_send_data_->translation_vector = tvec;
      break;
    }
  }
}

std::shared_ptr<DbusSendData> CalibrateStatus::extracted_calib_param(){
  std::lock_guard<std::mutex> lock(mutex_);
  return dbus_send_data_;
}

void CalibrateStatus::reset_caliration_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    calib_request_id = 0;
}

void CalibrateStatus::get_request_id(int& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    LOG(INFO) << calib_request_id;
    LOG(INFO) << calib_status;
    id = calib_request_id;
}

void CalibrateStatus::get_lidar_calib_mode(int &mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << lidar_calib_mode;
  mode = lidar_calib_mode;
}

void CalibrateStatus::set_status(const uint32_t state) {
    std::lock_guard<std::mutex> lock(mutex_);
    calib_status = state;
}

void CalibrateStatus::getCalibrationStatus(uint32_t& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state = calib_status;
}

void CalibrateStatus::getCalibrationFilePath(std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    path = calib_file_path;
}

void CalibrateStatus::set_calibrate_path(const std::string path) {
    std::lock_guard<std::mutex> lock(mutex_);
    calib_file_path = path;
}

void CalibrateStatus::run() {
  LOG(INFO) << "vehicle mode server start.";
  while (1) {
    sleep(5);
  }
}

} // namespace airi
} // namespace crdc

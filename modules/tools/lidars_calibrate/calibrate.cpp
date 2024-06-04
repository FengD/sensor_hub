#include <chrono>
#include "tools/lidars_calibrate/calibrate.h"
#include "framework/internal/Global.h"

namespace crdc {
namespace airi {

void Calibrate::init(const std::string &config_path) {
  // init calibration
  lidar_calib_ = std::make_shared<LidarsCalibration>();
  lidar_calib_->init(config_path);
  process_state_ = 1;
  vehicle_mode_ = 0;
  begin();
}

void Calibrate::dynamic_init(const std::string &frame_id) {
  dynamic_calibration_mode_.init(frame_id);
  process_state_ = 1;
  vehicle_mode_ = 0;
  dynamic_calibrate_end_ = false;
}

void Calibrate::begin() {
  connection = IpcConnection::getConnection(IpcConnection::IPC_TYPE_DBUS,
                                            String8::format("%s", "LidarDrivers"));
  Global::setMainConnection(connection);
  status_server_ = new CalibrateStatus();

  connection->addService(status_server_);
  connection->start();
  status_server_->start();
  std::string path = lidar_calib_->get_save_path();
  status_server_->set_calibrate_path(path);
}

int Calibrate::get_vehicle_mode() {
  int request_id = 0;
  status_server_->get_request_id(request_id);
  vehicle_mode_ = request_id;
  return vehicle_mode_;
}

void Calibrate::set_calib_end() {
  calib_end_ = true;
}

int Calibrate::get_calib_mode() {
  int calib_mode = 0;
  status_server_->get_lidar_calib_mode(calib_mode);
  calib_mode_ = calib_mode;
  return calib_mode_;
}

int Calibrate::process(pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr fr_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr rl_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr rr_cloud) {
  if (calib_end_) {
    calib_end_ = false;
    process_state_ = 1;
    status_server_->set_status(process_state_);
  }
  if (process_state_ != 3) {
    // preparing input of calibration
    process_state_ = 1;
    status_server_->set_status(process_state_);
    // start calibration process
    process_state_ = 2;
    status_server_->set_status(process_state_);
    while (!lidar_calib_->cali_flag_) {
      lidar_calib_->process(fl_cloud, fr_cloud, rl_cloud, rr_cloud, save_path_);
    }
    // send response
    if (lidar_calib_->calib_finshed_) {
      process_state_ = 3;
      status_server_->set_status(process_state_);
      status_server_->reset_caliration_state();
    }
    return 0;
  }
  return 1;
}

void Calibrate::set_dynamic_calibrate_end() {
  dynamic_calibrate_end_ = true;
}

int Calibrate::dynamic_process(std::shared_ptr<LidarPointCloud> &cloud) {
  if (dynamic_calibrate_end_) {
    dynamic_calibrate_end_ = false;
    dynamic_calibration_mode_.reset_param();
  }
  // preparing input of calibration
  dynamic_calibration_mode_.process(cloud);
  process_state_ = dynamic_calibration_mode_.get_process_state();
  status_server_->set_status(process_state_);
  if (process_state_ == 3 || process_state_ == 4) {
    status_server_->reset_caliration_state();
    return 0;
  }
  return 1;
}

}  // namespace airi
}  // namespace crdc

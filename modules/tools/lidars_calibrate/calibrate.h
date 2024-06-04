#pragma once

#include <memory>
#include <string>
#include <vector>

#include "framework/IDBusProxy.h"
#include "framework/IpcConnection.h"
#include "tools/AF_server/calibrate_server.h"
#include "tools/lidars_calibrate/lidars_calibration.h"
#include "tools/lidar_dynamic_calibrate/dynamic_calibration.h"
namespace crdc {
namespace airi {
class Calibrate {
 public:
  Calibrate() = default;
  virtual ~Calibrate() = default;
  void init(const std::string& config_path);
  int process(pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr fr_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr rl_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr rr_cloud);
  int get_vehicle_mode();
  int get_calib_mode();
  void set_calib_end();

  std::string get_name() const {
    return "LidarProcess";
  }
  void begin();
  void dynamic_init(const std::string& frame_id);
  void set_dynamic_calibrate_end();
  int dynamic_process(std::shared_ptr<LidarPointCloud>& cloud);

 private:
  hirain::sp<CalibrateStatus> status_server_;
  hirain::sp<IpcConnection> connection;
  std::shared_ptr<LidarsCalibration> lidar_calib_;
  DynamicCalibration dynamic_calibration_mode_;
  bool calib_end_ = false;
  int process_state_ = 1;
  int vehicle_mode_;
  int calib_mode_;
  bool dynamic_calibrate_end_;
  std::string save_path_;
  std::mutex mutex_;
  void get_request_id(int& id);
  void set_calibrate_path(const std::string path);
  void set_status(const uint32_t state);
};
}  // namespace airi
}  // namespace crdc

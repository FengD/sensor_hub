// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar

#pragma once

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include "common/common.h"
#include "module_diagnose/module_diagnose.h"
#include "lidar_drivers/input/input.h"
#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/compensator/compensator.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#ifdef WITH_ROS2
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msg/msg/point_clouds2.hpp"
#include "sensor_msg/msg/packet.hpp"
#include "sensor_msg/msg/module_status.hpp"
#include "sensor_msg/msg/level.hpp"
#include "lidar_drivers/output/af_output.h"
#ifdef WITH_TDA4
#include "tools/lidars_calibrate/calibrate.h"
#else
#include "tools/lidar_dynamic_calibrate/dynamic_calibration.h"
#endif
#else
#include "cyber/sensor_proto/lidar.pb.h"
#include "lidar_drivers/output/cyber_output.h"

#endif

#ifdef WITH_ROS2
using Level = sensor_msg::msg::Level;
using Packet = sensor_msg::msg::Packet;
using LidarOutput = crdc::airi::LidarAFOutput;
#else
using LidarOutput = crdc::airi::LidarCyberOutput;
#endif

namespace crdc {
namespace airi {

class Lidar : public common::Thread {
 public:
  explicit Lidar(const LidarComponentConfig& config);
  virtual ~Lidar() = default;
  void stop();
  void set_callback(std::function<void(const std::shared_ptr<LidarPointCloud>&)> callback) {
    callback_ = callback;
  }
  void set_flag(bool flag) {
    std::lock_guard<std::mutex> lock(mutex_);
    flag_ = flag;
  }

  bool get_flag(void) {
    std::lock_guard<std::mutex> lock(mutex_);
    return flag_;
  }

  bool get_iftransform(void) {
#ifdef WITH_TDA4
    std::lock_guard<std::mutex> lock(mutex_);
#endif
    return iftransform_;
  }

#ifdef WITH_TDA4
  void set_iftransform(bool state) {
    std::lock_guard<std::mutex> lock(mutex_);
    iftransform_ = state;
  }

  void get_calbrate_server(std::shared_ptr<Calibrate>& server) {
    if (!getserver_) {
      dynamic_calibrate_mode_ = server;
      getserver_ = true;
    }
  }
#endif

  std::string get_name() const {
    return lidar_name_;
  }

 private:
  /**
   * @brief The run function execute the whole process of the lidar loop process.
   */
  void run() override;
  /**
   * @brief Init the encoder if defined.
   */
  bool init_parser();

  /**
   * @brief Init the input if defined.
   */
  bool init_input();
  /**
   * @brief Send diagnose message
   */
  void verify_emptiness_and_send_diagnose(const int32_t code,
                                          const std::shared_ptr<LidarPointCloud>& cloud);
#ifdef WITH_ROS2
  void send_diagnose_input(const uint32_t& position_id,
                           const DeviceStatus& error, const uint8_t& level,
                           const std::string& custom_desc, const std::string& context);
  void init_dyncalibrate(const std::string& frame_id);
  void dynamic_calibrate(std::shared_ptr<LidarPointCloud>& cloud);
#ifdef WITH_TDA4
  std::shared_ptr<Calibrate> dynamic_calibrate_mode_;
#else
  DynamicCalibration dynamic_calibration_mode_;
#endif
#else
  void send_diagnose_input(const uint32_t& position_id,
                           const DeviceStatus& error, const Level& level,
                           const std::string& custom_desc, const std::string& context);
#endif
  void get_parser_lidar(Packet* raw_packet, int32_t code);

  bool stop_;
  bool flag_ = true;
  bool iftransform_;
  bool dynamic_calibrate_init_;
  bool getserver_;
  int dynamic_calibrate_id_;
  int32_t reload_;
  std::string lidar_name_;
  LidarComponentConfig config_;
  LidarConfig lidar_config_;
  std::function<void(const std::shared_ptr<LidarPointCloud>&)> callback_;
  std::shared_ptr<LidarInput> input_;
  std::shared_ptr<LidarParser> parser_;
  uint32_t sensor_position_id_;
  std::vector<DiagnoseInput> diagnose_inputs_;
  int32_t data_fps_index_;
  int32_t raw_data_downsampling_each_cloud_frame_;
  int32_t last_raw_data_fps_index_;
  int32_t cloud_data_downsampling_each_cloud_frame_;
  int32_t last_cloud_data_fps_index_;
  Compensator compensator_;

  /**
   * @brief control the data written frequency, written-fps = real-fps / downsampling_each_frame.
   */
  template<typename T>
  void write_data_intermittently(int32_t &last_data_fps_index,
                                 int32_t downsampling_each_frame,
                                 std::string channel_name,
                                 T data,
                                 std::function<bool(const std::string&, const T&)> write_func);
};

}  // namespace airi
}  // namespace crdc

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
#include <rclcpp/rclcpp.hpp>
#include "sensor_msg/srv/auto_calibration_status.hpp"
#include "common/common.h"

using CalibStatusSrv = sensor_msg::srv::AutoCalibrationStatus;

namespace crdc {
namespace airi {

class CalibStatusServer : public rclcpp::Node, public common::Thread {
 public:
  CalibStatusServer(const std::string& server_name)
      : Node("calib_status_server"), common::Thread(true), state_flag_(0), response_flag_(0) {
    service_ = create_service<sensor_msg::srv::AutoCalibrationStatus>(
        server_name, std::bind(&CalibStatusServer::request_callback, this,
                                                std::placeholders::_1, std::placeholders::_2));
  }
  ~CalibStatusServer() override = default;
  void set_state(int state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_flag_ = state;
  }
  bool check_calib_failure();
  void set_calib_start_time() { start_ = clock_.now(); }
  void check_timeout();
  void send_calib_status_response(
      std::shared_ptr<sensor_msg::srv::AutoCalibrationStatus::Response>& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response = response_;
    service_->send_response(*request_id_, *response);
  }

 protected:
  void run() override;

 private:
  std::mutex mutex_;
  int state_flag_;
  bool response_flag_;
  rclcpp::Clock clock_;
  rclcpp::Time start_;
  rclcpp::Time now_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<CalibStatusSrv>::SharedPtr service_;
  std::shared_ptr<rmw_request_id_t> request_id_;
  std::shared_ptr<CalibStatusSrv::Response> response_ =
      std::make_shared<CalibStatusSrv::Response>();
  void handle_request(
      const std::shared_ptr<sensor_msg::srv::AutoCalibrationStatus::Request> request);
  void request_callback(
      std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<sensor_msg::srv::AutoCalibrationStatus::Request> request);
};
}  // namespace airi
}  // namespace crdc

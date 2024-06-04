/**
* @copyright Copyright (c) 2023 Beijing Jingwei Hirain Technologies Co., Inc. All rights reserved
* @file server_node.h
* @brief Common server node for sensor calibration
* @author yangyang.liu1
* @date 2023-06-12
* @license Modified BSD Software License Agreement
*/
#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msg/srv/vehicle_mode.hpp"
#include "common/common.h"

using CalibSrv = sensor_msg::srv::VehicleMode;

namespace crdc {
namespace airi {

class CalibrateServer : public rclcpp::Node, public common::Thread {
 public:
  CalibrateServer(const std::string &process_name)
      : Node("calibrate_server_node"), common::Thread(true), state_flag_(0) {
    service_ = create_service<sensor_msg::srv::VehicleMode>(process_name,
                                                            std::bind(&CalibrateServer::get_request_callback,
                                                            this, std::placeholders::_1,
                                                            std::placeholders::_2));
  }
  ~CalibrateServer() override = default;

  void get_state(int &state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state = state_flag_;
  }

  void send_response(std::shared_ptr<CalibSrv::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response_ = response;
    service_->send_response(*request_id_, *response);
    is_receive_request_ = false;
  }

 protected:
  void run() {
      LOG(INFO) << "vehicle mode server start.";
    while(rclcpp::ok() && !is_receive_request_) {
      rclcpp::spin_some(shared_from_this());
    }

    std::lock_guard<std::mutex> lock(mutex_);
    state_flag_ = request_->mode_type;
  }

 private:
  std::mutex mutex_;
  int state_flag_;
  bool is_receive_request_ = false;
  rclcpp::Service<CalibSrv>::SharedPtr service_;

  std::shared_ptr<rmw_request_id_t> request_id_ = std::make_shared<rmw_request_id_t>();
  std::shared_ptr<CalibSrv::Request> request_ = std::make_shared<CalibSrv::Request>();

  std::shared_ptr<CalibSrv::Response> response_ =
      std::make_shared<CalibSrv::Response>();
  void get_request_callback(std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<CalibSrv::Request> request) {
    if (request->mode_type < 0 || request->mode_type > 4) {
      LOG(FATAL) << "Request: Mode Type wrong!";
    } else {
      request_ = request;
      request_id_ = request_header;

      std::lock_guard<std::mutex> lock(mutex_);
      is_receive_request_ = true;
    }
  }
};
}  // namespace airi
}  // namespace crdc
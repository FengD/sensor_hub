#include "tools/service_server/calibration_status.h"

using CalibStatusSrv = sensor_msg::srv::AutoCalibrationStatus;
namespace crdc {
namespace airi {

void CalibStatusServer::request_callback(
    std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<sensor_msg::srv::AutoCalibrationStatus::Request> request) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request->status < 0 || request->status > 1) {
    LOG(FATAL) << "Request: Request Type wrong!";
  } else {
    handle_request(request);
    request_id_ = request_header;
    response_flag_ = true;
    service_->send_response(*request_id_, *response_);
  }
}

bool CalibStatusServer::check_calib_failure() {
  if (state_flag_ == 4) {
    return true;
  }
  return false;
}

void CalibStatusServer::check_timeout() {
  now_ = clock_.now();
  rclcpp::Duration duration = now_ - start_;
  double minute_diff = duration.nanoseconds() / 60.0 / 10e9;
  if (minute_diff > 5.0) {
    state_flag_ == 4;
  }
}

void CalibStatusServer::handle_request(
    const std::shared_ptr<sensor_msg::srv::AutoCalibrationStatus::Request> request) {
  int task_status = static_cast<uint8_t>(request->status);
  check_timeout();
  if (task_status == 0 || state_flag_ == 0) {
    response_->response = "connection success";
    response_->calibration_status = 0;
  } else if (task_status == 1) {
    if (state_flag_ == 1) {
      response_->response = "calibration preparing";
      response_->calibration_status = 1;
    } else if (state_flag_ == 2) {
      response_->response = "calibration processing";
      response_->calibration_status = 2;
    } else if (state_flag_ == 3) {
      response_->response = "calibration finished";
      response_->calibration_status = 3;
    } else if (state_flag_ == 4) {
      response_->response = "calibration failed, maybe timeout or validation failure";
      response_->calibration_status = 4;
    }
  }
}

void CalibStatusServer::run() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration status service started");

  while (!response_flag_) {
    rclcpp::spin(shared_from_this());
  }
}

}  // namespace airi
}  // namespace crdc
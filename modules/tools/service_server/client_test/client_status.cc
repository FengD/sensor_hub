#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msg/srv/auto_calibration_status.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "testtttt");
  if (argc != 2) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage test");
    return 1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "create node status");
  std::shared_ptr<rclcpp::Node> calib_status_node =
      rclcpp::Node::make_shared("test_status_service");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "create client status");
  rclcpp::Client<sensor_msg::srv::AutoCalibrationStatus>::SharedPtr calib_status_client =
      calib_status_node->create_client<sensor_msg::srv::AutoCalibrationStatus>("test_calib_status");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request 2 ");
  auto calib_status_request = std::make_shared<sensor_msg::srv::AutoCalibrationStatus::Request>();
  calib_status_request->status = atoll(argv[1]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " %d ", calib_status_request->status);

  while (!calib_status_client->wait_for_service(10s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting");
  }

  while (rclcpp::ok()) {
    auto result = calib_status_client->async_send_request(calib_status_request);

    if (rclcpp::spin_until_future_complete(calib_status_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result.get();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result calibration_status: %d",
                  response->calibration_status);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result response : %s",
                  response->response.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call calib status service ");
    }
  }

  rclcpp::spin(calib_status_node);
  rclcpp::shutdown();
  return 0;
}

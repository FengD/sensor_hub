#include "rclcpp/rclcpp.hpp"
#include "sensor_msg/srv/vehicle_mode.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

void receive_response_callback(rclcpp::Client<sensor_msg::srv::VehicleMode>::SharedFuture response) {
  auto result = response.get();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result is_success: %d", result->is_success);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result response : %s", result->response.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result ldc_map[0]: %s ", result->ldc_map[0].c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result x[0]: %lf ", result->x[0]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result y[0]: %lf ", result->y[0]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result z[0]: %lf ", result->z[0]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result roll[0]: %lf ", result->roll[0]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result pitch[0]: %lf ", result->pitch[0]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " result yaw[0]: %lf ", result->yaw[0]);
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  if(argc!=2) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"usage test");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_client");
  rclcpp::Client<sensor_msg::srv::VehicleMode>::SharedPtr client =
      node->create_client<sensor_msg::srv::VehicleMode>("test_calib");
  auto request = std::make_shared<sensor_msg::srv::VehicleMode::Request>();
  request->mode_type = atoll(argv[1]);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " %d ", request->mode_type);
  while (!client->wait_for_service(1s)) {
    if(!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"exiting.");
        return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"waiting");
  }
  client->async_send_request(request, &receive_response_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

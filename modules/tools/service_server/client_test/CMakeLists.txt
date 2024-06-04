cmake_minimum_required(VERSION 3.8)
project(calibrate_service)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(sensor_msg_DIR ../../../../build/modules/crdc_airi_common/share/sensor_msg/cmake)
find_package(sensor_msg REQUIRED)

add_executable(client_calib client_calib.cc)
ament_target_dependencies(client_calib rclcpp sensor_msg "rosidl_typesupport_cpp")

add_executable(client_status client_status.cc)
ament_target_dependencies(client_status rclcpp sensor_msg "rosidl_typesupport_cpp")

install(TARGETS
        client_calib
        client_status
        DESTINATION lib/${PROJECT_NAME})
ament_package()
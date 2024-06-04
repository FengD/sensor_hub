project(lidar_drivers_input_socket)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} lidar_drivers_proto)
if (WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msg
)
endif()
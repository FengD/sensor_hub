project(lidar_drivers_input_pcap)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} lidar_drivers_proto)
target_link_libraries(${PROJECT_NAME} pcap)
if (WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msg
)
endif()
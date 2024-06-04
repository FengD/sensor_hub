project(lidar_drivers_parser_vlp16)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msg
)
endif()
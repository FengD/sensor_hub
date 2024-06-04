project(lidar_drivers_input_ros2)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} lidar_drivers_proto)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
    sensor_msg
)

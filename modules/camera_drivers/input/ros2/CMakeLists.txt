project(camera_drivers_input_ros2)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
if (WITH_ROS2)
add_dependencies(${PROJECT_NAME}
    sensor_msg
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rosbag2_cpp
)
endif()
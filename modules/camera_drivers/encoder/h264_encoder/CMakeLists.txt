project(camera_drivers_encoder_h264)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} camera_drivers_proto)
target_link_libraries(${PROJECT_NAME}
    tiovx_encoder
    tiovx_common
)
if (WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
rclcpp
)
endif()
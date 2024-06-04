project(camera_drivers_input_tiovx_camera)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} 
    tiovx_camera
    tiovx_common
)
if (WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
rclcpp
)
endif()
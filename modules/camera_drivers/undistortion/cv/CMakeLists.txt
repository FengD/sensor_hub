project(camera_drivers_undistortion_cv)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} camera_drivers_proto)
if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
endif()
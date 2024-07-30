project(lidar_drivers_compensator)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  lidar_drivers_proto
)
if(WITH_ROS2)
add_dependencies(${PROJECT_NAME}
  sensor_msg
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)
endif()
project(ins_drivers_input_pcap)

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME}
  pcap
  glog
  gflags
  pthread
  common
)
if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msg
)
endif()
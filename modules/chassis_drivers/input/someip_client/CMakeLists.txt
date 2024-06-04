project(chassis_drivers_input_someip)

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME}
  glog
  gflags
  pthread
  common
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

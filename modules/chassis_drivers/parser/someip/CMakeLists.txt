project(chassis_parser_someip)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} client_adapter)
target_link_libraries(${PROJECT_NAME}  -Wl,--whole-archive
  ap_someip_api  -Wl,--no-whole-archive
  client_adapter  
)
# if (WITH_ROS2)
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   sensor_msg
#   sensor_msgs
# )
# endif()

# list(APPEND SRC_PROJ_V201_HAV02 ${COMMON_SRC})
# list(APPEND SRC_PROJ_V210_HAV30 ${COMMON_SRC})
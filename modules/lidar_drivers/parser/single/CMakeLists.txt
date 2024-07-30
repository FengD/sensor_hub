project(lidar_drivers_parser_lds50cs)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
    sensor_msg "rosidl_typesupport_cpp"
)
 
target_link_libraries(${PROJECT_NAME}
    "${cpp_type_support_target}"
)
endif()
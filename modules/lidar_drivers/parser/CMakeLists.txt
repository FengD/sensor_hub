project(lidar_drivers_parser)

add_subdirectory(pandar)
add_subdirectory(innoviz)
add_subdirectory(velodyne)
add_subdirectory(robosense)
add_subdirectory(single)
add_subdirectory(arbe)
add_subdirectory(falcon)

if (DO_TEST)
add_subdirectory(test)
endif()

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  lidar_drivers_parser_pandar
  lidar_drivers_parser_innovizone
  lidar_drivers_parser_vlp16
  lidar_drivers_parser_robosense
  lidar_drivers_parser_lds50cs
  lidar_drivers_parser_radar_phoenix_A0
  lidar_drivers_parser_falcon
  lidar_drivers_proto
)

target_link_libraries(${PROJECT_NAME}
  lidar_drivers_parser_pandar
  lidar_drivers_parser_innovizone
  lidar_drivers_parser_vlp16
  lidar_drivers_parser_robosense
  lidar_drivers_parser_lds50cs
  lidar_drivers_parser_radar_phoenix_A0
  lidar_drivers_parser_falcon
  atomic
)

if(WITH_ROS2)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msg
    sensor_msgs
)
endif()
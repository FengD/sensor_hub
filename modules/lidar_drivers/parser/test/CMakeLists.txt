project(lidar_drivers_parser_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
      lidar_drivers_proto
      lidar_drivers_parser_innovizone
      lidar_drivers_parser_pandar
      lidar_drivers_parser_robosense
      lidar_drivers_parser_vlp16
      lidar_drivers_parser_lds50cs
      lidar_drivers_parser_radar_phoenix_A0
    )
    target_link_libraries(${PROJECT_NAME}
      gtest
      gflags
      glog
      common
      cyber
      atomic
      pthread
      lidar_drivers_proto
      lidar_drivers_parser_innovizone
      lidar_drivers_parser_pandar
      lidar_drivers_parser_robosense
      lidar_drivers_parser_vlp16
      lidar_drivers_parser_lds50cs
      lidar_drivers_parser_radar_phoenix_A0
      lidar_drivers_parser
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. parser test will not build")
endif()


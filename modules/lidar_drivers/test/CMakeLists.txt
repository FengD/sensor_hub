project(lidar_drivers_test)

if (GTEST_FOUND)
    FILE(GLOB _SRCS "*.cc")
    add_executable(${PROJECT_NAME} ${_SRCS})
    add_dependencies(${PROJECT_NAME}
        lidar_drivers_input_socket
        lidar_drivers_input_cyber
        lidar_drivers_input_pcap
        lidar_drivers_proto
        lidar_drivers_parser
    )
    target_link_libraries(${PROJECT_NAME}
        gtest
        gflags
        glog
        common
        cyber
        atomic
        pthread
        lidar_drivers_input_socket
        lidar_drivers_input_cyber
        lidar_drivers_input_pcap
        lidar_drivers_proto
        lidar_drivers_parser
    )
    add_test(${PROJECT_NAME} ${PROJECT_NAME})
else()
    message(WARNING "Gtest not Found. input test will not build")
endif()

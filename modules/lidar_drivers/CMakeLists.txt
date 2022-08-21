project(lidar_drivers)

add_subdirectory(input)
add_subdirectory(parser)
add_subdirectory(proto)
add_subdirectory(compensator)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    common
    lidar_drivers_proto
    lidar_drivers_input_socket
    lidar_drivers_input_cyber
    lidar_drivers_parser
    lidar_drivers_compensator
)

target_link_libraries(${PROJECT_NAME}
    common 
    lidar_drivers_proto
    lidar_drivers_input_socket
    lidar_drivers_input_cyber
    lidar_drivers_parser
    lidar_drivers_compensator
    glog
    gflags
    fastrtps
    cyber
    pthread
)

install(DIRECTORY params DESTINATION .)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)
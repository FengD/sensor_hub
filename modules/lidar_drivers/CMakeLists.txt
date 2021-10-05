project(lidar_drivers)

add_subdirectory(input)
add_subdirectory(parser)
add_subdirectory(proto)
add_subdirectory(compensator)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    sensor_hub_common
    lidar_drivers_proto
)
target_link_libraries(${PROJECT_NAME}
    sensor_hub_common 
    lidar_drivers_proto
    glog
    gflags
    fastrtps
    cyber
)

install(DIRECTORY param DESTINATION .)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)
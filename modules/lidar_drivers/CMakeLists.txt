project(lidar_drivers)

add_subdirectory(input)
add_subdirectory(output)
add_subdirectory(parser)
add_subdirectory(proto)
add_subdirectory(compensator)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/../common)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/proto)
link_directories(/usr/local/apollo/cyber/lib/)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    lidar_drivers_output
    lidar_drivers_proto
)
target_link_libraries(${PROJECT_NAME} 
    ${PROTOBUF_LIBRARIES} 
    common 
    lidar_drivers_proto
    lidar_drivers_output
    cyber
    glog
    gflags
    fastrtps
)
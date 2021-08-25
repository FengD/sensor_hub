project(lidar_drivers)

add_subdirectory(input)
add_subdirectory(output)
add_subdirectory(parser)
add_subdirectory(proto)
add_subdirectory(compensator)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/../common)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/proto)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES} sensor_common lidar_drivers_proto)
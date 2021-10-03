project(camera_drivers)

add_subdirectory(input)
add_subdirectory(proto)

find_package(OpenCV REQUIRED)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    sensor_hub_common
    camera_drivers_proto
    camera_drivers_input
)
target_link_libraries(${PROJECT_NAME}
    sensor_hub_common 
    camera_drivers_proto
    camera_drivers_input
    glog
    gflags
    fastrtps
    cyber
    ${OpenCV_LIBS}
)
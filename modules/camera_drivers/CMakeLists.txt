project(camera_drivers)

add_subdirectory(input)
add_subdirectory(proto)
add_subdirectory(convert)
add_subdirectory(encoder)
add_subdirectory(undistortion)

find_package(OpenCV REQUIRED)

file(GLOB SRCS *.cc)
add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}
    sensor_hub_common
    camera_drivers_proto
    camera_drivers_input
    camera_drivers_convert
    camera_drivers_encoder_cv
    camera_drivers_encoder_turbo
    camera_drivers_undistortion_cv
)

target_link_libraries(${PROJECT_NAME}
    sensor_hub_common 
    camera_drivers_proto
    camera_drivers_input
    camera_drivers_convert
    camera_drivers_encoder_cv
    camera_drivers_encoder_turbo
    camera_drivers_undistortion_cv
    glog
    gflags
    fastrtps
    cyber
    ${OpenCV_LIBS}
)

install(DIRECTORY params DESTINATION .)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)
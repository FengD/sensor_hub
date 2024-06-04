project(camera_drivers_input_mxc)

add_subdirectory(3rdparty)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  mxc_v4l2_camera_driver
)

target_link_libraries(${PROJECT_NAME}
  mxc_v4l2_camera_driver
)
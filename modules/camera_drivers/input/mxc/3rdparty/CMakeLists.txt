project(mxc_v4l2_camera_driver)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
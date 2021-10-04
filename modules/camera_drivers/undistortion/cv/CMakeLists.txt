project(camera_drivers_undistortion_cv)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
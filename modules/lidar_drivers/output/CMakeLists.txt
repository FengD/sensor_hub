project(lidar_drivers_output)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
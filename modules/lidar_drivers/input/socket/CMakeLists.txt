project(lidar_drivers_input_socket)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} ${SRCS})
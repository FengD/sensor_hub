project(sensor_hub_common_io)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} ${SRCS})
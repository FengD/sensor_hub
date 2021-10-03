project(sensor_hub_common)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} ${SRCS})
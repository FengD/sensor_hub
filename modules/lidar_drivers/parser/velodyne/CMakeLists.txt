project(lidar_drivers_parser_vlp16)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
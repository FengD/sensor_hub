project(lidar_drivers_parser_pxt32)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
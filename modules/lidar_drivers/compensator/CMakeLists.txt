project(lidar_drivers_compensator)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
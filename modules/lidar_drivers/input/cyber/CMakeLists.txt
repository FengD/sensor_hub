project(lidar_drivers_input_cyber)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} lidar_drivers_proto)

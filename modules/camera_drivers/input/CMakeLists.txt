project(camera_drivers_input)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} camera_drivers_proto)
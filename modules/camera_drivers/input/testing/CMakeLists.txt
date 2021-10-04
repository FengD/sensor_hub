project(camera_drivers_input_testing)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
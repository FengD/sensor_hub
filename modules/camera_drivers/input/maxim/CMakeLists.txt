project(camera_drivers_input_maxim)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
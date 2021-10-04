project(camera_drivers_convert)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
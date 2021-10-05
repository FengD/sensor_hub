project(sensor_hub_common)

add_subdirectory(io)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} sensor_hub_common_io)
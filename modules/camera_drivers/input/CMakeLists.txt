project(camera_drivers_input)

add_subdirectory(sensing)
add_subdirectory(maxim)

file(GLOB SRCS *.cc)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME} camera_drivers_proto)
target_link_libraries(${PROJECT_NAME}
  atomic
  camera_drivers_input_sensing
  camera_drivers_input_maxim
)
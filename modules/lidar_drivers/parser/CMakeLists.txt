project(lidar_drivers_parser)

add_subdirectory(velodyne)

file(GLOB SRCS *.cc)

add_library(${PROJECT_NAME} STATIC ${SRCS})
add_dependencies(${PROJECT_NAME}
  lidar_drivers_parser_vlp16
)

target_link_libraries(${PROJECT_NAME}
  lidar_drivers_parser_vlp16
  atomic
)